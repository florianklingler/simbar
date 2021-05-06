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

#ifndef __VEINS_LARA_LANRADIOINTERFERENCE_H
#define __VEINS_LARA_LANRADIOINTERFERENCE_H

#include <sys/socket.h>
#include <stdlib.h>
// #include <unistd.h>
// #include <sys/types.h>
// #include <stdio.h>
// #include <arpa/inet.h>
// #include <netinet/in.h>
#include <netdb.h>
#include <string.h>

#include <omnetpp.h>
#include <thread>
#include <map>
#include <queue>
#include <mutex>
#include <chrono>
#include <sstream>
#include <zmq.h>
#include <zmq.hpp>
#include <netinet/ether.h>
#include "veins/base/utils/FindModule.h"
#include "veins/modules/utility/Consts80211p.h"
#include "wlan.pb.h"
#include "SimbaR/GeoNetHeaders.h"
#include "SimbaR/LanradioMacInterface.h"
#include "SimbaR/messages/CAMMessage_m.h"
#include "SimbaR/messages/SPATMessage_m.h"

namespace veins {

class LanradioInterference : public cSimpleModule {
public:
    enum t_wlan_type {
        wlan_CAM,
        wlan_DENM,
        wlan_SPAT
    };

    struct wlan_msg {
        std::string src_mac;
        std::string dst_mac;
        std::string payload;
    };

    virtual ~LanradioInterference() = default;
    virtual void initialize(int stage) override;
    virtual void finish() override;

    virtual void sendInterference(std::string srcmac, std::string dstmac, int packetByteLength, double Power_Milliwatt, simtime_t arrivalTime);

    int interferenceSocketUDP;

    int runNumber;

    struct sockaddr_in interferenceAddr;

    cMessage* logInterval;

    enum msg_types {
        Lanradio_Interference_LogInterval,
    };

protected:
    virtual void handleMessage(cMessage* msg) override;

    zmq::context_t* mContext;

    void activate_lanradio();
    void deactivate_radio();
    bool interference_send_command(const std::string cmd);

    zmq::socket_t* interferenceRadioConfigSocket;

    int baseport;
    bool use_interference;
    std::string interferenceHostname;
    std::string bitrates;
    std::string wlan_ip;
    std::string freq; // in MHz
    std::string bandwidth; // in MHz
    std::string experimentName;
    
    std::string SimInterferenceTxPower; // in dBm
    bool SimInterferenceModifyMac;

    simtime_t interferenceMinInterval;
    bool useInterferenceMinInterval;

    simtime_t lastInterferenceGenerated;
    simtime_t lastInterferenceArrivalTime;
    
    cOutVector numInterferingFramesPerSec;
    cOutVector numGeneratedPhysicalInterferencePerSec;
    cOutVector interArrivalTimeInterferingFrames;
    uint64_t interferingFramesPerSec;
    uint64_t generatedPhysicalInterferencePerSec;
};

class LanradioInterferenceAccess {
public:
    LanradioInterference* get()
    {
        return FindModule<LanradioInterference*>::findGlobalModule();
    };
};

} // namespace veins

#endif
