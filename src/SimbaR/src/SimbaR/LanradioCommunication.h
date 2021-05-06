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
// Mario Franke <research@m-franke.net>

#ifndef __VEINS_LARA_LANRADIOCOMMUNICATION_H
#define __VEINS_LARA_LANRADIOCOMMUNICATION_H

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

enum realWorldDevices {
    simulationInterface,
    dut,
};

class LanradioCommunication : public cSimpleModule {
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


    virtual ~LanradioCommunication() = default;
    virtual void initialize(int stage) override;
    virtual void finish() override;

    // virtual void sendMessage(std::string msg);
    virtual void send_WLAN(void* data, simtime_t simulationCreationTime, t_wlan_type type, std::string srcmac, std::string dstmac, realWorldDevices pDev);
    virtual void insert_wlan_msg_buffer(wlan_msg msg, realWorldDevices pDev, std::chrono::steady_clock::time_point tp);

    virtual bool registerRadio(LanradioMacInterface* mac);

    bool dutSubSync = false;
    bool simIntSubSync = false;

    int simIntSocketUDP;
    int dutSocketUDP;

    int runNumber;

    struct sockaddr_in simIntAddr;
    struct sockaddr_in dutAddr;

    enum msg_types {
        Lanradio_CHECK_SIMSEC_PER_SEC,
        Lanradio_CHECK_SIMINT_MSG_BUFFER,
        Lanradio_CHECK_DUT_MSG_BUFFER
    };

    /* logging */
    uint64_t receivedBytesDut;
    uint64_t receivedPacketsDut;
    uint64_t receivedBytesFellowVehicles;
    uint64_t receivedPacketsFellowVehicles;
    uint64_t sendBytesDut;
    uint64_t sendPacketsDut;
    uint64_t sendBytesFellowVehicles;
    uint64_t sendPacketsFellowVehicles;

    cOutVector dutToSimIntLat;
    cOutVector simIntToDutLat;
    cOutVector simsecPerSec;
    cOutVector simIntReceiveBufferSize;
    cOutVector dutReceiveBufferSize;
    cOutVector wrongRunNumberPackets;
    cOutVector wrongEvalSequenceNumberPacketsSimInt;
    cOutVector wrongEvalSequenceNumberPacketsDUT;

    cOutVector dataRateDut;     // just counting received packets
    cOutVector dataRateSimInt;
    cOutVector dataRateOverall;
    uint64_t packetsPerSecSimInt;
    uint64_t packetsPerSecDut;
    uint64_t packetsPerSecOverall;


    cOutVector lostPPS_Overall;
    cOutVector lostPPS_SimIntDut;
    cOutVector lostPPS_DutSimInt;
    uint64_t lostPacketsLastSecond_SimIntDut;
    uint64_t lostPacketsLastSecond_DutSimInt;

    cOutVector transmittedPPS_Overall;
    cOutVector transmittedPPS_SimIntDut;
    cOutVector transmittedPPS_DutSimInt;
    uint64_t transmittedPacketsPerSec_Overall;
    uint64_t transmittedPacketsPerSec_SimIntDut;
    uint64_t transmittedPacketsPerSec_DutSimInt;

    std::chrono::steady_clock::time_point lastSecTimestamp;

    cMessage* check_simsec_per_sec;
    cMessage* check_message_buffer;
    cMessage* check_dut_message_buffer;
    cMessage* check_simint_message_buffer;


protected:
    int evalSequenceNumber = 0;

    virtual void handleMessage(cMessage* msg) override;

    zmq::context_t* mContext;

    void activate_lanradio();
    void deactivate_radio();
    bool radio_send_command(const std::string cmd);
    bool chanload_send_command(const std::string cmd);

    virtual void process_simInt_message_buffer();
    virtual void process_dut_message_buffer();
    virtual bool received_WLAN(wlan_msg m, realWorldDevices pDev, std::chrono::steady_clock::time_point rtp);

    zmq::socket_t* simIntPublisher;
    zmq::socket_t* syncSocketSimInt;

    zmq::socket_t* dutPublisher;
    zmq::socket_t* syncSocketDut;

    // zmq::socket_t* radioConfigSocket;
    // zmq::socket_t* syncSocketRadio;
    zmq::socket_t* simIntRadioConfigSocket;
    zmq::socket_t* dutRadioConfigSocket;
    zmq::socket_t* chanloadConfigSocket;

    std::thread* simulationInterface_receive_thread;
    std::thread* dut_receive_thread;

    std::mutex mutex_simulationInterface_buffer;
    std::mutex mutex_dut_buffer;
    std::mutex mutex_dutMessageMap;
    std::mutex mutex_simIntMessageMap;

    std::mutex mutex_dut_check_msg;
    std::mutex mutex_simInt_check_msg;;

    bool ownDutCheckMsg;
    bool ownSimIntCheckMsg;

    std::queue<std::pair<std::chrono::steady_clock::time_point, wlan_msg>> simIntReceiveQueue;
    std::queue<std::pair<std::chrono::steady_clock::time_point, wlan_msg>> dutReceiveQueue;

    std::map<int, std::pair<std::chrono::steady_clock::time_point, simtime_t>> dutMessageMap;
    std::map<int, std::pair<std::chrono::steady_clock::time_point, simtime_t>> simIntMessageMap;

    LanradioMacInterface* mac_interface = nullptr;
    int baseport;
    std::string simIntHostname;
    std::string dutHostname;
    std::string wlan_interface;
    bool use_lanradio;
    std::string bitrates;
    std::string wlan_ip;
    std::string freq; // in MHz
    std::string bandwidth; // in MHz
    bool lara_debug;
    bool use_udp;
    int numEvaluationBytes;
    std::string experimentName;
    
    std::string SimInterfaceTxPower; // in dBm
    std::string DutTxPower; // in dBm
    bool SimInterfaceModifyMac;

    virtual void fillGeoNetBTPheaderForCam(CAM_t* cam, int payloadLen, std::string srcmac);
    struct GeoNetworkAndBTPHeaderCAM mGeoBtpHdrForCam;

    virtual void fillGeoNetBTPheaderForSpat(int payloadLen, std::string srcmac);
    struct GeoNetworkAndBTPHeaderSPAT mGeoBtpHdrForSpat;
};

class LanradioCommunicationAccess {
public:
    LanradioCommunication* get()
    {
        return FindModule<LanradioCommunication*>::findGlobalModule();
    };
};

} // namespace veins

#endif
