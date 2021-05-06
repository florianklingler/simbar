// This file is part of Lanradio.
//
// Lanradio is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Lanradio is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with Lanradio.  If not, see <http://www.gnu.org/licenses/>.
//
// Authors:
// Florian Klingler <klingler@ccs-labs.org>
// Dominik S. Buse <buse@ccs-labs.org>
// Mario Franke <research@m-franke.net>

#ifndef __VEINS_LARA_SIMPLEAPP_H
#define __VEINS_LARA_SIMPLEAPP_H

#include "veins/base/modules/BaseApplLayer.h"
#include "veins/modules/utility/Consts80211p.h"
#include "veins/modules/utility/TimerManager.h"
#include "SimbaR/LanradioMac1609_4.h"
#include "CAM.h"
#include "SPAT-PDU.h"
#include <map>
#include <deque>

namespace veins {

enum StaticScenarioNodeType {
    UNKNOWN = -1,
    virtualPrototype,
    fellowVehicle,
};

class TraCIMobility;
class TraCICommandInterface;
class BaseFrame1609_4;

class StaticScenarioApp : public BaseApplLayer {

public:
    ~StaticScenarioApp() = default;
    void initialize(int stage) override;

    void sendCAM();
    void sendSPAT();

protected:
    enum cMessageTypes {
        data_rate_per_sec,
    };

    cMessage* check_data_rate;

    /** @brief handle messages from below and calls the onWSM, onBSM, and onWSA functions accordingly */
    void handleLowerMsg(cMessage* msg) override;

    /** @brief handle self messages */
    void handleSelfMsg(cMessage* msg) override;

    /** @brief sets all the necessary fields in the WSM, BSM, or WSA. */
    virtual void populateWSM(BaseFrame1609_4* wsm, LAddress::L2Type rcvId = LAddress::L2BROADCAST(), int serial = 0);

    virtual CAM_t generate_CAM();
    virtual SPAT_PDU_t* generate_SPAT();

    void track_spat(SPAT_PDU_t* spat);

    virtual void finish() override;

protected:
    TraCIMobility* mobility;
    TraCICommandInterface* traci;
    TimerManager timerManager{this};
    LanradioMac1609_4* lr_mac;
    Mac1609_4* mac;
    std::string fullName;

    /* BSM (beacon) settings */
    uint32_t beaconLengthBits;
    uint32_t beaconPriority;

    std::deque<SPAT_PDU_t*> sent_spats;
    long sent_spats_max_size = 4;

    int64_t lastCamSendingTime = 0;
    double lastCamHeading = -1;

    std::string myMacAddress;
    int myId;
    bool app_debug;

    /* Logging */
    uint64_t receivedPackets;
    uint64_t sendPackets;

    cOutVector dataRateDUT;
    int recvPacketsPerSecDUT;
    int recvPacketsDUT;
    cOutVector dataRateSim;
    int recvPacketsPerSecSim;
    int recvPacketsSim;

    cOutVector applLatSimDUT;
    cOutVector applLatDUTSim;
    cOutVector applLatSimSim;
       
};

} // namespace veins

#endif
