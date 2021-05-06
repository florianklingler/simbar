//
// Copyright (C) 2011 David Eckhoff <eckhoff@cs.fau.de>
// Copyright (C) 2012 Bastian Bloessl, Stefan Joerer, Michele Segata <{bloessl,joerer,segata}@ccs-labs.org>
// Copyright (C) 2018 Fabian Bronner <fabian.bronner@ccs-labs.org>
// Copyright (C) 2021 Mario Franke <research@m-franke.net>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// SPDX-License-Identifier: GPL-2.0-or-later
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

/*
 * Based on Decider80211.cc from Karl Wessel
 * and modifications by Christopher Saloman
 */

#include "SimbaR/LanradioDecider80211p.h"
#include "veins/modules/phy/DeciderResult80211.h"
#include "veins/modules/messages/Mac80211Pkt_m.h"
#include "veins/base/toolbox/Signal.h"
#include "veins/modules/messages/AirFrame11p_m.h"
#include "veins/modules/phy/NistErrorRate.h"
#include "veins/modules/utility/ConstsPhy.h"

#include "veins/base/toolbox/SignalUtils.h"

using namespace veins;

simtime_t LanradioDecider80211p::processNewSignal(AirFrame* msg)
{

    AirFrame11p* frame = check_and_cast<AirFrame11p*>(msg);

    // get the receiving power of the Signal at start-time and center frequency
    Signal& signal = frame->getSignal();

    signalStates[frame] = EXPECT_END;

    if (signal.smallerAtCenterFrequency(minPowerLevel)) {

        // annotate the frame, so that we won't try decoding it at its end
        frame->setUnderMinPowerLevel(true);
        // check channel busy status. a superposition of low power frames might turn channel status to busy
        if (cca(simTime(), nullptr) == false) {
            setChannelIdleStatus(false);
        }
        return signal.getReceptionEnd();
    }
    else {

        // This value might be just an intermediate result (due to short circuiting)
        double recvPower = signal.getAtCenterFrequency();
        setChannelIdleStatus(false);

        /* Do not discard packets which are received while we are sending.
         * We want to transmit these packets on the physical channel, too.
         * The phy layer must be able to transmit and receive at the same time.
         */
        // if (phy11p->getRadioState() == Radio::TX) {
        //     std::cout << "LanradioDecider80211p: processNewSignal: begin receiving frame while sending. Decode anyway." << std::endl;
        // }
        if (!currentSignal.first) {
            // NIC is not yet synced to any frame, so lock and try to decode this frame
            currentSignal.first = frame;
            EV_TRACE << "AirFrame: " << frame->getId() << " with (" << recvPower << " > " << minPowerLevel << ") -> Trying to receive AirFrame." << std::endl;
            if (notifyRxStart) {
                phy->sendControlMsgToMac(new cMessage("RxStartStatus", MacToPhyInterface::PHY_RX_START));
            }
        }
        else {
            // NIC is currently trying to decode another frame. this frame will be simply treated as interference
            EV_TRACE << "AirFrame: " << frame->getId() << " with (" << recvPower << " > " << minPowerLevel << ") -> Already synced to another AirFrame. Treating AirFrame as interference." << std::endl;
            lara_phy11p->sendInterferenceFrameOnPhysicalChannel(frame);
        }

        // channel turned busy
        // measure communication density
        myBusyTime += signal.getDuration().dbl();
        return signal.getReceptionEnd();
    }
}

simtime_t Decider80211p::processSignalEnd(AirFrame* msg)
{

    AirFrame11p* frame = check_and_cast<AirFrame11p*>(msg);

    // here the Signal is finally processed
    Signal& signal = frame->getSignal();

    double recvPower_dBm = 10 * log10(signal.getMax());

    bool whileSending = false;

    // remove this frame from our current signals
    signalStates.erase(frame);

    DeciderResult* result;

    if (frame->getUnderMinPowerLevel()) {
        // this frame was not even detected by the radio card
        result = new DeciderResult80211(false, 0, 0, recvPower_dBm);
    }
    else {
        // if (frame->getWasTransmitting() || phy11p->getRadioState() == Radio::TX) {
        //     std::cout << "LanradioDecider80211p: processSignalEnd: Decode frame while transmitting." << std::endl;
        // }

        // first check whether this is the frame NIC is currently synced on
        if (frame == currentSignal.first) {
            // check if the snr is above the Decider's specific threshold,
            // i.e. the Decider has received it correctly
            result = checkIfSignalOk(frame);

            // after having tried to decode the frame, the NIC is no more synced to the frame
            // and it is ready for syncing on a new one
            currentSignal.first = 0;
        }
        else {
            // if this is not the frame we are synced on, we cannot receive it
            result = new DeciderResult80211(false, 0, 0, recvPower_dBm);
        }
    }

    if (result->isSignalCorrect()) {
        EV_TRACE << "packet was received correctly, it is now handed to upper layer...\n";
        // go on with processing this AirFrame, send it to the Mac-Layer
        if (notifyRxStart) {
            phy->sendControlMsgToMac(new cMessage("RxStartStatus", MacToPhyInterface::PHY_RX_END_WITH_SUCCESS));
        }
        phy->sendUp(frame, result);
    }
    else {
        if (frame->getUnderMinPowerLevel()) {
            EV_TRACE << "packet was not detected by the card. power was under minPowerLevel threshold\n";
        }
        else if (whileSending) {
            EV_TRACE << "packet was received while sending, sending it as control message to upper layer\n";
            phy->sendControlMsgToMac(new cMessage("Error", RECWHILESEND));
        }
        else {
            EV_TRACE << "packet was not received correctly, sending it as control message to upper layer\n";
            if (notifyRxStart) {
                phy->sendControlMsgToMac(new cMessage("RxStartStatus", MacToPhyInterface::PHY_RX_END_WITH_FAILURE));
            }

            if (((DeciderResult80211*) result)->isCollision()) {
                phy->sendControlMsgToMac(new cMessage("Error", Decider80211p::COLLISION));
            }
            else {
                phy->sendControlMsgToMac(new cMessage("Error", BITERROR));
            }
        }
        delete result;
    }

    if (phy11p->getRadioState() == Radio::TX) {
        EV_TRACE << "I'm currently sending\n";
    }
    // check if channel is idle now
    // we declare channel busy if CCA tells us so, or if we are currently
    // decoding a frame
    else if (cca(simTime(), frame) == false || currentSignal.first != 0) {
        EV_TRACE << "Channel not yet idle!\n";
    }
    else {
        // might have been idle before (when the packet rxpower was below sens)
        if (isChannelIdle != true) {
            EV_TRACE << "Channel idle now!\n";
            setChannelIdleStatus(true);
        }
    }
    return notAgain;
}

LanradioDecider80211p::~LanradioDecider80211p(){};
