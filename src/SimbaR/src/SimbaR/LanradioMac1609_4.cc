//
// Copyright (C) 2018 Florian Klingler <klingler@ccs-labs.org>
// Copyright (C) 2018 Dominik S. Buse <buse@ccs-labs.org>
// Copyright (C) 2021 Mario Franke <research@m-franke.net>
//
// Documentation for these modules is at http://veins.car2x.org/
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

#include "SimbaR/LanradioMac1609_4.h"
#include "veins/modules/messages/PhyControlMessage_m.h"
#include "veins/modules/phy/DeciderResult80211.h"
#include "veins/base/phyLayer/PhyToMacControlInfo.h"
#include "SimbaR/LanradioCommunication.h"
#include "SimbaR/messages/CAMMessage_m.h"
#include "SimbaR/messages/SPATMessage_m.h"

using veins::LanradioMac1609_4;

Define_Module(veins::LanradioMac1609_4);

#define DBG_MAC EV

void LanradioMac1609_4::initialize(int stage)
{
    Mac1609_4::initialize(stage);
    if (stage == 0) {
        subsequentPacketsQueue.setName("subsequentPacketsQueue");

        /* logging */
        statsReceivedBroadcasts = 0;    // from base class, there are several more stats fields
        physicalPacketCollisions.setName("physicalPacketCollisions");
        physicalPacketCollisionsPerSec = 0;
        check_physical_packet_collisions = new cMessage("check physical packet collisions");
        scheduleAt(simTime() + 1, check_physical_packet_collisions);
    }
    if (stage == 1) {
        if (par("send_to_lanradio").boolValue()) {
            setAsLanradioMac(true);
        }
    }
}

void LanradioMac1609_4::finish()
{
    // if (lara != nullptr) {
    //     setAsLanradioMac(false);
    // }
    Mac1609_4::finish();
    recordScalar("statsReceivedBroadcasts", statsReceivedBroadcasts);
}

void LanradioMac1609_4::handleSelfMsg(cMessage* msg)
{
    if (msg == nextMacEvent) {
        lastPT = PT_SIM;
    }
    if (msg == check_physical_packet_collisions) {
        physicalPacketCollisions.record(physicalPacketCollisionsPerSec);
        physicalPacketCollisionsPerSec = 0;
        scheduleAt(simTime() + 1, check_physical_packet_collisions);
    }
    Mac1609_4::handleSelfMsg(msg);
}

void LanradioMac1609_4::handleLowerControl(cMessage* msg)
{
    if (msg->getKind() == MacToPhyInterface::TX_OVER) {
        //DBG_MAC << "Successfully transmitted a packet on " << lastAC << std::endl;
        //std::cout << "LR_MAC: Successfully transmitted a packet on " << lastAC << ", at simTime: " << simTime() << std::endl;
        if (subsequentPacketsQueue.isEmpty()) {
            // std::cout << "LR_MAC: no scheduled transmission: set radio to RX state at simTime:" << simTime() << std::endl;
            phy->setRadioState(Radio::RX);

            // perfom post transmit only for simulated packets
            if (lastPT == PT_SIM) {
                if (dynamic_cast<Mac80211Ack*>(lastMac.get()) == nullptr) {
                    // message was sent
                    // update EDCA queue. go into post-transmit backoff and set cwCur to cwMin
                    myEDCA[activeChannel]->postTransmit(lastAC, lastWSM, useAcks);
                }
                // channel just turned idle.
                // don't set the chan to idle. the PHY layer decides, not us.
                if (guardActive()) {
                    throw cRuntimeError("We shouldnt have sent a packet in guard!");
                }
            }
            if (msg->getKind() == Decider80211p::COLLISION) {
                emit(sigCollision, true);
            }
        }else{
            // std::cout << "LR_MAC: got TX Over message but a subsequent transmission is alread scheduled." << std::endl;
            sendSubsequentPacket();
        }
        delete msg;
    }
    else {
        Mac1609_4::handleLowerControl(msg);
    }
}

/**
 * received packet from fellow vehicle which has to be transmitted via the physical channel
 */
void LanradioMac1609_4::handleBroadcast(Mac80211Pkt* macPkt, DeciderResult80211* res)
{
    statsReceivedBroadcasts++;
    std::unique_ptr<BaseFrame1609_4> wsm(check_and_cast<BaseFrame1609_4*>(macPkt->decapsulate()));
    wsm->setControlInfo(new PhyToMacControlInfo(res));
    if (lara != nullptr) {
        sendLanradioMsg(wsm.get());
    }
    // sendUp(wsm.release()); // send msg to application, TODO: Do we want the application to have to cam immediately or only after a real transmission
}

/**
 * forward message to virtual dut application layer
 */
void LanradioMac1609_4::toApplicationLayer(cMessage* msg)
{
    Enter_Method_Silent();
    take(msg); // take ownership of message
    std::unique_ptr<BaseFrame1609_4> wsm(check_and_cast<BaseFrame1609_4*>(msg));
    if (wsm == nullptr) {
        error("WaveMac only accepts BaseFrame1609_4s");
    }
    sendUp(wsm.release());
}

/**
 * send cam generated by virtual prototype to physical prototype in order to transmit the message in the real world
 */

void LanradioMac1609_4::sendPrototypeToSimulation(cMessage* msg) {
    Enter_Method_Silent();
    take(msg);
    ASSERT(lara != nullptr);
    if (auto camMsg = dynamic_cast<CAMMessage*>(msg)) {
        CAM_t cam = camMsg->getCam();
        auto lanradio_cam = static_cast<CAM_t*>(calloc(1, sizeof(CAM_t)));
        memcpy(lanradio_cam, &cam, sizeof(CAM_t));

        // std::string srcmac = "11:22:33:44:55:66";
        // std::stringstream smac;
        // smac << "11:22:33:44:55:" << msg->getId();
        std::string dstmac = "FF:FF:FF:FF:FF:FF";
        lara->send_WLAN(lanradio_cam, camMsg->getCreationTime(), LanradioCommunication::wlan_CAM, camMsg->getSrcmac(), dstmac, realWorldDevices::dut);
    }
    // TODO: add SPAT message
}

/**
 * send broadcast from any not virtualPrototype to ALIX3 (simulation to physical prototype)
 */
void LanradioMac1609_4::sendLanradioMsg(cMessage* msg)
{
    ASSERT(lara != nullptr);

    if (auto camMsg = dynamic_cast<CAMMessage*>(msg)) {
        CAM_t cam = camMsg->getCam();
        auto lanradio_cam = static_cast<CAM_t*>(calloc(1, sizeof(CAM_t)));
        memcpy(lanradio_cam, &cam, sizeof(CAM_t));

        // std::string srcmac = "11:22:33:44:55:66";
        // std::stringstream smac;
        // smac << "11:22:33:44:55:" << msg->getId();
        std::string dstmac = "FF:FF:FF:FF:FF:FF";
        lara->send_WLAN(lanradio_cam, camMsg->getCreationTime(), LanradioCommunication::wlan_CAM, camMsg->getSrcmac(), dstmac, realWorldDevices::simulationInterface);
    }
    else if (auto spatMsg = dynamic_cast<SPATMessage*>(msg)) {
        SPAT_PDU_t spat = spatMsg->getSpat();
        auto lanradio_spat = static_cast<SPAT_PDU_t*>(calloc(1, sizeof(SPAT_PDU_t)));
        memcpy(lanradio_spat, &spat, sizeof(SPAT_PDU_t));

        // std::string srcmac = "AA:BB:CC:DD:EE:FF";
        std::string dstmac = "FF:FF:FF:FF:FF:FF";
        lara->send_WLAN(lanradio_spat, spatMsg->getCreationTime(), LanradioCommunication::wlan_SPAT, spatMsg->getSrcmac(), dstmac, realWorldDevices::simulationInterface);
    }
    else {
        error("Lanradio Mac only accepts CAMMessages and SpatMessages to be sent to Lanradio");
    }
}

void LanradioMac1609_4::sendSubsequentPacket()
{
    cMessage* msg = static_cast<cMessage*>(subsequentPacketsQueue.pop());
    if (useSCH) {
        error("Currently no Multi-Channel Scheduling is supported, set useSCH to false to continue.");
    }

    auto thisMsg = dynamic_cast<BaseFrame1609_4*>(msg);
    if (thisMsg == nullptr) {
        error("WaveMac only accepts BaseFrame1609_4s");
    }

    // check if the channel is currently idle
    if (!idleChannel) {
        physicalPacketCollisionsPerSec++;
    }

    channelBusySelf(true);

    lastPT = PT_LARA;

    // send the packet
    auto mac = new Mac80211Pkt(thisMsg->getName(), thisMsg->getKind());
    ASSERT(mac != nullptr);
    mac->setDestAddr(LAddress::L2BROADCAST());
    mac->setSrcAddr(myMacAddr);
    mac->encapsulate(thisMsg->dup());

    PhyControlMessage* controlInfo;
    controlInfo = dynamic_cast<PhyControlMessage*>(thisMsg->getControlInfo());
    // ASSERT(controlInfo != nullptr);
    if (controlInfo == nullptr) {
        // std::cout << "LR_MAC: No controlInfo  available, creating default controlInfo." << std::endl;
        controlInfo = new PhyControlMessage();
        controlInfo->setTxPower_mW(txPower);
        controlInfo->setMcs((int)MCS::ofdm_qpsk_r_1_2);
    }
    MCS mcs = static_cast<MCS>(controlInfo->getMcs());
    double txPower_mW = controlInfo->getTxPower_mW();
    attachControlInfo(mac, Channel::cch, mcs, txPower_mW);
    // std::cout << "LR_MAC: sending subsequent packet at: " << simTime() << std::endl;
    send(mac, lowerLayerOut);
}

/**
 * integrate cam from physical prototype in the simulation
 */
void LanradioMac1609_4::sendMacPacketDirect(cMessage* msg)
{
    Enter_Method_Silent();
    take(msg);

    if (phy->getRadioState() == Radio::TX) {
        // std::cout << "LR_MAC: virtual dut is already transmitting. Add packet in waiting queue." << std::endl;
        subsequentPacketsQueue.insert(msg);
    }else{
        if (useSCH) {
            error("Currently no Multi-Channel Scheduling is supported, set useSCH to false to continue.");
        }

        auto thisMsg = dynamic_cast<BaseFrame1609_4*>(msg);
        if (thisMsg == nullptr) {
            error("WaveMac only accepts BaseFrame1609_4s");
        }

        // check if the channel is currently idle
        if (!idleChannel) {
            // std::cerr << "LR_MAC: Collision with physical transmission" << std::endl;
            physicalPacketCollisionsPerSec++;
        }

        channelBusySelf(true);

        lastPT = PT_LARA;

        // send the packet
        auto mac = new Mac80211Pkt(thisMsg->getName(), thisMsg->getKind());
        ASSERT(mac != nullptr);
        mac->setDestAddr(LAddress::L2BROADCAST());
        mac->setSrcAddr(myMacAddr);
        mac->encapsulate(thisMsg->dup());

        PhyControlMessage* controlInfo;
        controlInfo = dynamic_cast<PhyControlMessage*>(thisMsg->getControlInfo());
        // ASSERT(controlInfo != nullptr);
        if (controlInfo == nullptr) {
            // std::cout << "LR_MAC: No controlInfo  available, creating default controlInfo." << std::endl;
            controlInfo = new PhyControlMessage();
            controlInfo->setTxPower_mW(txPower);
            controlInfo->setMcs((int)MCS::ofdm_qpsk_r_1_2);
        }
        MCS mcs = static_cast<MCS>(controlInfo->getMcs());
        double txPower_mW = controlInfo->getTxPower_mW();

        phy->setRadioState(Radio::TX);

        attachControlInfo(mac, Channel::cch, mcs, txPower_mW);
        // std::cout << "LR_MAC: no ongoing transmission. Send direct at: " << simTime() << std::endl;
        send(mac, lowerLayerOut);
    }
}

void LanradioMac1609_4::setAsLanradioMac(bool isNewLanradioMac)
{
    Enter_Method_Silent();
    if (isNewLanradioMac) {
        lara = LanradioCommunicationAccess().get();
        ASSERT(lara != nullptr);
        std::cout << "LR_MAC: got access to LanradioCommunication." << std::endl;
        if (!lara->registerRadio(this)) {
            error("LR_MAC: lara->registerRadio(this) was unsuccessful");
        }
    }
    else if (lara != nullptr) {
        auto lara_bak = lara; // to avoid endless recursion
        lara = nullptr;
        std::cout << "LR_MAC: lara is nullptr!" << std::endl;
        lara_bak->registerRadio(nullptr);
    }
}
