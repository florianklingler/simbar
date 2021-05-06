//
// Copyright (C) 2011 David Eckhoff <eckhoff@cs.fau.de>
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
 * Based on PhyLayer.cc from Karl Wessel
 * and modifications by Christopher Saloman
 */

#include "veins/modules/phy/PhyLayer80211p.h"
#include "veins/base/toolbox/Signal.h"
#include "SimbaR/LanradioPhyLayer80211p.h"


using namespace veins;

Define_Module(veins::LanradioPhyLayer80211p);

void LanradioPhyLayer80211p::initialize(int stage)
{
    PhyLayer80211p::initialize(stage);
    if (stage == 1) {
        setLanradioInterferenceModule();
    }
}

void LanradioPhyLayer80211p::sendInterferenceFrameOnPhysicalChannel(AirFrame* frame)
{
    ASSERT(lara_int != nullptr);
    Signal& signal = frame->getSignal();
    double recvPower = signal.getAtCenterFrequency();   // in milliwatt
    cMessage* packet = frame->decapsulate();
    ASSERT(packet);
    Mac80211Pkt* macPkt = check_and_cast<Mac80211Pkt*>(packet);
    BaseFrame1609_4* wsm(check_and_cast<BaseFrame1609_4*>(macPkt->decapsulate()));
    ASSERT(wsm != nullptr);
    if (auto camMsg = dynamic_cast<CAMMessage*>(wsm)) {
        CAM_t cam = camMsg->getCam();
        auto interference_cam = static_cast<CAM_t*>(calloc(1, sizeof(CAM_t)));
        memcpy(interference_cam, &cam, sizeof(CAM_t));

        std::string dstmac = "FF:FF:FF:FF:FF:FF";
        // std::cout << "Interfering frame byte length: " << camMsg->getByteLength() << std::endl;
        lara_int->sendInterference(camMsg->getSrcmac(), dstmac, camMsg->getByteLength(), recvPower, packet->getArrivalTime());
    }else{
        std::cout << "Message: " << packet->getFullName() << std::endl;
    }
}

void LanradioPhyLayer80211p::setLanradioInterferenceModule()
{
    lara_int = LanradioInterferenceAccess().get();
    ASSERT(lara_int != nullptr);
}

std::unique_ptr<Decider> LanradioPhyLayer80211p::getDeciderFromName(std::string name, ParameterMap& params)
{
    if (name == "LanradioDecider80211p") {
        protocolId = IEEE_80211;
        return initializeLanradioDecider80211p(params);
    }
    return BasePhyLayer::getDeciderFromName(name, params);
}

std::unique_ptr<Decider> LanradioPhyLayer80211p::initializeLanradioDecider80211p(ParameterMap& params)
{
    double centerFreq = params["centerFrequency"];
    auto dec = make_unique<LanradioDecider80211p>(this, this, minPowerLevel, ccaThreshold, allowTxDuringRx, centerFreq, findHost()->getIndex(), collectCollisionStatistics);
    dec->setPath(getParentModule()->getFullPath());
    return std::unique_ptr<Decider>(std::move(dec));
}
