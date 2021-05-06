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

#include "SimbaR/LanradioCommunication.h"
#include <unistd.h>
#include <openssl/rand.h>
#include <stdlib.h>
#include "SimbaR/pstream.h"
#include <sstream>

#define MYDEBUG EV
#define UDP_BUFFER_SIZE 2048

using veins::LanradioCommunication;

Define_Module(veins::LanradioCommunication);

// OPENC2X ethertype for GeoNetworking
static const uint16_t ETHERTYPE_CAR = 0x8947;

namespace {

int writeOut(const void* buffer, size_t size, void* app_key)
{
    auto payload = static_cast<std::vector<uint8_t>*>(app_key);
    auto structPtr = static_cast<const uint8_t*>(buffer);
    copy_n(structPtr, size, std::back_inserter(*payload));
    return 0;
}

std::vector<uint8_t> encodeMessage(asn_TYPE_descriptor_t* td, void* structPtr)
{
    std::vector<uint8_t> payload;
    uper_encode(td, const_cast<void*>(structPtr), &writeOut, &payload);
    return payload;
}

int decodeMessage(asn_TYPE_descriptor_t* td, void** t, std::string buffer)
{
    asn_codec_ctx_t ctx{};
    asn_dec_rval_t drv = uper_decode_complete(&ctx, td, t, buffer.data(), buffer.length());
    return drv.code;
}

} // end namespace

//  recv data
static std::pair<std::string, std::chrono::steady_clock::time_point> lanradio_recv(zmq::socket_t& sock, bool use_udp, int s)
{
    if (!use_udp) {
        zmq::message_t msg;
        sock.recv(&msg);
        auto tp = std::chrono::steady_clock::now();
        // std::cout << "LR_COM: received packet via ZMQ." << std::endl;

        return std::pair<std::string, std::chrono::steady_clock::time_point>(std::string(static_cast<char*>(msg.data()), msg.size()), tp);
    }else{
        std::string data;
        int n;
        data.resize(UDP_BUFFER_SIZE);
        /* wait for message and process it */
        n = recv(s, (void*)data.c_str(), UDP_BUFFER_SIZE, 0);
        if (n < 0) {
            std::cout << "LR_COM: socket could not receive data ..." << std::endl;;
        }
        auto tp = std::chrono::steady_clock::now();
        data.resize(n);
        return std::pair<std::string, std::chrono::steady_clock::time_point>(data, tp);
    }
}

//  send multipart zmq message
static bool zmq_sndmore(zmq::socket_t& sock, const std::string& str)
{
    zmq::message_t msg(str.size());
    memcpy(msg.data(), str.data(), str.size());

    bool ret = sock.send(msg, ZMQ_SNDMORE);
    return ret;
}

//  send zmq string
static std::pair<bool, std::chrono::steady_clock::time_point> lanradio_send(zmq::socket_t& sock, const std::string& str, bool use_udp, int s, struct sockaddr_in* cliAddr)
{
    if (!use_udp) {
        zmq::message_t msg(str.size());
        memcpy(msg.data(), str.data(), str.size());

        bool ret = sock.send(msg);
        auto tp = std::chrono::steady_clock::now();
        return std::pair<bool, std::chrono::steady_clock::time_point>(ret, tp);
    }else{
        int r;
        r = sendto(s, (void*)str.c_str(), str.length(), 0, (const struct sockaddr*)cliAddr, sizeof(sockaddr));
        auto tp = std::chrono::steady_clock::now();
        // std::cout << "LR_COM: send packet via UDP using the following socket: " << s << std::endl;
        if (r > 0) {
            return std::pair<bool, std::chrono::steady_clock::time_point>(true, tp);
        }else{
            return std::pair<bool, std::chrono::steady_clock::time_point>(false, tp);
        }
    }
}

static bool radioConfigEnd(zmq::socket_t& simIntSock, zmq::socket_t& dutSock)
{
    std::string str = "END";
    zmq::message_t msg(str.size());
    zmq::message_t test(str.size());
    memcpy(msg.data(), str.data(), str.size());
    memcpy(test.data(), str.data(), str.size());

    bool simIntRet = simIntSock.send(msg);
    bool dutRet = dutSock.send(test);
    bool ret = simIntRet && dutRet;
    return ret;
}

static std::pair<bool, std::chrono::steady_clock::time_point> radioConfig_send(zmq::socket_t& simIntSock, zmq::socket_t& dutSock, const std::string& str)
{
    zmq::message_t msg(str.size());
    zmq::message_t test(str.size());
    memcpy(msg.data(), str.data(), str.size());
    memcpy(test.data(), str.data(), str.size());

    zmq::message_t reply;

    // bool ret = sock.send(msg);
    // auto tp = std::chrono::steady_clock::now();
    std::cout << "Send command: " << str << ", to simInt." << std::endl;
    bool simIntRet = simIntSock.send(msg);
    std::cout << "Send command: " << str << ", to dut." << std::endl;
    bool dutRet = dutSock.send(test);
    bool ret = simIntRet && dutRet;

    std::cout << "wait for radio config acks." << std::endl;
    simIntSock.recv(&reply);
    std::cout << "received Ack from simInt." << std::endl;
    dutSock.recv(&reply);
    std::cout << "received Ack from dut." << std::endl;

    auto tp = std::chrono::steady_clock::now();
    return std::pair<bool, std::chrono::steady_clock::time_point>(ret, tp);
}

static bool chanloadConfig_send(zmq::socket_t& chanloadSock, const std::string& str)
{
    zmq::message_t msg(str.size());
    memcpy(msg.data(), str.data(), str.size());

    zmq::message_t reply;

    std::cout << "chanloadConfig_send: Send command: " << str << std::endl;
    bool ret = chanloadSock.send(msg);

    std::cout << "chanloadConfig_send: wait for acks." << std::endl;
    chanloadSock.recv(&reply);
    std::cout << "chanloadConfig_send: received Ack." << std::endl;

    return ret;
}

static bool setCarrierSensingOff(bool value, zmq::socket_t& sock)
{
    if (value) {
        zmq::message_t cmd("echo 1 > /sys/kernel/debug/ieee80211/phy1/ath9k/carrier_sensing_off");
        std::cout << "Disable carrier sensing." << std::endl;
        sock.send(cmd);
        std::cout << "Wait for Ack for disabling carrier sensing." << std::endl;
        sock.recv(&cmd);
    }else{
        zmq::message_t cmd("echo 0 > /sys/kernel/debug/ieee80211/phy1/ath9k/carrier_sensing_off");
        std::cout << "Enable carrier sensing." << std::endl;
        sock.send(cmd);
        std::cout << "Wait for Ack for enabling carrier sensing." << std::endl;
        sock.recv(&cmd);
    }
    return false;
}

static bool copyChanloadFile(zmq::socket_t& sock, std::string device, std::string experiment, std::string runNumber)
{
    zmq::message_t cmd("scp -i /tmp/id_rsa  -o \"StrictHostKeyChecking=no\" /tmp/chanload_" + runNumber + ".txt franke@10.0.197.200:/home/franke/master-thesis/lanradio-franke/experiments/" + experiment + "/csv/chanload_" + device + "/chanload_" + runNumber + ".txt");
    std::cout << "Copy chanload file: " << device << std::endl;
    sock.send(cmd);
    std::cout << "Wait for Ack for copying chanload." << std::endl;
    sock.recv(&cmd);

    zmq::message_t rm("rm /tmp/chanload_" + runNumber + ".txt");
    std::cout << "Remove chanload file: " << device << std::endl;
    sock.send(rm);
    std::cout << "Wait for Ack for removing chanload." << std::endl;
    sock.recv(&rm);
    return true;
}

static bool copyPendingFramesFile(zmq::socket_t& sock, std::string device, std::string experiment, std::string runNumber)
{
    zmq::message_t cmd("scp -i /tmp/id_rsa  -o \"StrictHostKeyChecking=no\" /tmp/pending_frames_" + runNumber + ".txt franke@10.0.197.200:/home/franke/master-thesis/lanradio-franke/experiments/" + experiment + "/csv/pendingFrames_" + device + "/pendingFrames_" + device + "_" + runNumber + ".txt");
    std::cout << "Copy pending frames file: " << device << ", experiment: " << experiment << std::endl;
    sock.send(cmd);
    std::cout << "Wait for Ack for copying pending frames." << std::endl;
    sock.recv(&cmd);

    zmq::message_t rm("rm /tmp/pending_frames_" + runNumber + ".txt");
    std::cout << "Remove pending frames file: " << device << std::endl;
    sock.send(rm);
    std::cout << "Wait for Ack for removing pending frames file." << std::endl;
    sock.recv(&rm);
    return true;
}

static std::pair<std::string, std::chrono::steady_clock::time_point> radioConfig_recv(zmq::socket_t& sock)
{
    zmq::message_t msg;
    sock.recv(&msg);
    auto tp = std::chrono::steady_clock::now();
    return std::pair<std::string, std::chrono::steady_clock::time_point>(std::string(static_cast<char*>(msg.data()), msg.size()), tp);
}

void thread_receive_simInt(std::string simIntHostname, LanradioCommunication* communication_module, bool lara_debug, zmq::context_t* context, int port, bool use_udp)
{
    ASSERT(communication_module != nullptr);

    std::string subPort = std::to_string(port);
    std::string syncPort = std::to_string(port + 1);
    std::cout << "LR_COM: SIMINT subscibe thread: subPort: " << subPort << ", syncPort: " << syncPort << std::endl;

    /* create zmq sockets */
    auto simulationInterface = new zmq::socket_t(*context, ZMQ_SUB);
    simulationInterface->setsockopt(ZMQ_RCVTIMEO, -1);
    simulationInterface->setsockopt(ZMQ_LINGER, 0);
    simulationInterface->setsockopt(ZMQ_SUBSCRIBE, "", 0);

    auto syncSocket = new zmq::socket_t(*context, ZMQ_REQ);
    syncSocket->setsockopt(ZMQ_SNDTIMEO, -1);
    syncSocket->setsockopt(ZMQ_LINGER, 0);
    syncSocket->setsockopt(ZMQ_RCVTIMEO, -1);

    std::string message;
    if (!use_udp) {
        simulationInterface->connect(("tcp://" + simIntHostname + ":" + subPort).c_str());  // zmq_sub has to connect in order to not get old messages.
        syncSocket->bind(("tcp://*:" + syncPort).c_str());
        // SYNC
        message = lanradio_recv(*simulationInterface, use_udp, communication_module->simIntSocketUDP).first;
        if (message == "SYNC") {
            std::cout << "LR_COM: simInt subscribe thread: received SYNC request. " << std::endl;
            lanradio_send(*syncSocket, "ACK", use_udp, communication_module->simIntSocketUDP, nullptr);
            message = lanradio_recv(*syncSocket, use_udp, communication_module->simIntSocketUDP).first;
            if (message == "START") {
                std::cout << "LR_COM: simInt subscribe thread: successfully synchronized." << std::endl;
                communication_module->simIntSubSync = true;
            }else{
                std::cout << "LR_COM: simInt subscribe thread error: " << message << std::endl;
            }
        }else{
            std::cout << "LR_COM: simInt subscriber thread: received error: " << message << std::endl;
        }
    }

    std::chrono::steady_clock::time_point tp;
    std::pair<std::string, std::chrono::steady_clock::time_point> r;
    while (true) {
        try {
            r = lanradio_recv(*simulationInterface, use_udp, communication_module->simIntSocketUDP);
            message = r.first;
            if (message == "SHUTDOWN") {
                std::cout << "SimInt receive thread: Got shutdown command." << std::endl;
                break;
            }
            tp = r.second;
        }catch (zmq::error_t e) {
            std::cout << "LR_COM: simInt receive thread: " << e.what() << std::endl;
            break;
        }
        // std::cout << "LR_COM: SimulationInterface received msg with length: " << message.length() << ", at simTime: " << simTime() << std::endl;
        auto wlan = new wlanPackage::WLAN();
        bool successful = wlan->ParseFromString(message);

        if (successful) {
            std::string byteMessage;
            byteMessage = wlan->payload();

            LanradioCommunication::wlan_msg m;
            m.src_mac = wlan->srcmac();
            m.dst_mac = wlan->dstmac();
            m.payload = byteMessage;

            if (wlan->ethertype() == ETHERTYPE_CAR) {
                communication_module->insert_wlan_msg_buffer(m, veins::realWorldDevices::simulationInterface, tp);
            }
        }else{
            std::cout << "LR_COM: received something different than wlan message." << std::endl;
        }
    }
    std::cout << "before closing syncSocket" << std::endl;
    syncSocket->close();
    delete syncSocket;
    std::cout << "before closing simInt Socket" << std::endl;
    simulationInterface->close();
    delete simulationInterface;
    std::cout << "LR_COM: simInt: deleted all sockets." << std::endl;
}

void thread_receive_dut(std::string dutHostname, LanradioCommunication* communication_module, bool lara_debug, zmq::context_t* context, int port, bool use_udp)
{
    ASSERT(communication_module != nullptr);

    std::string subPort = std::to_string(port);
    std::string syncPort = std::to_string(port + 1);
    std::cout << "LR_COM: DUT subscibe thread: subPort: " << subPort << ", syncPort: " << syncPort << std::endl;

    auto dut = new zmq::socket_t(*context, ZMQ_SUB);
    dut->setsockopt(ZMQ_RCVTIMEO, -1);
    dut->setsockopt(ZMQ_LINGER, 0);
    dut->setsockopt(ZMQ_SUBSCRIBE, "", 0);

    auto syncSocket = new zmq::socket_t(*context, ZMQ_REQ);
    syncSocket->setsockopt(ZMQ_SNDTIMEO, -1);
    syncSocket->setsockopt(ZMQ_LINGER, 0);
    syncSocket->setsockopt(ZMQ_RCVTIMEO, -1);

    std::string message;
    // SYNC
    if (!use_udp) {
        dut->connect(("tcp://" + dutHostname + ":" + subPort).c_str());  // zmq_sub has to connect in order to not get old messages.
        syncSocket->bind(("tcp://*:" + syncPort).c_str());
        message = lanradio_recv(*dut, use_udp, communication_module->dutSocketUDP).first;
        if (message == "SYNC") {
            std::cout << "LR_COM: DUT subscribe thread: received SYNC request. " << std::endl;
            lanradio_send(*syncSocket, "ACK", use_udp, communication_module->dutSocketUDP, nullptr);
            message = lanradio_recv(*syncSocket, use_udp, communication_module->dutSocketUDP).first;
            if (message == "START") {
                std::cout << "LR_COM: DUT subscribe thread: successfully synchronized." << std::endl;
                communication_module->dutSubSync = true;
            }else{
                std::cout << "LR_COM: DUT subscribe thread error: " << message << std::endl;
            }
        }else{
            std::cout << "LR_COM: DUT subscriber thread: received error: " << message << std::endl;
        }
    }

    std::chrono::steady_clock::time_point tp;
    std::pair<std::string, std::chrono::steady_clock::time_point> r;
    while (true) {
        try {
            r = lanradio_recv(*dut, use_udp, communication_module->dutSocketUDP);
            message = r.first;
            if (message == "SHUTDOWN") {
                std::cout << "LR_COM: dut receive thread: Got shutdown command." << std::endl;
                break;
            }
            tp = r.second;
        }catch (zmq::error_t e) {
            std::cout << "LR_COM: dut: " << e.what() << std::endl;
            break;
        }
        // std::cout << "LR_COM: DUT received msg with length: " << message.length() << ", at simTime: " << simTime() << std::endl;
        auto wlan = new wlanPackage::WLAN();
        bool successful = wlan->ParseFromString(message);

        if (successful) {
            std::string byteMessage;
            byteMessage = wlan->payload();

            LanradioCommunication::wlan_msg m;
            m.src_mac = wlan->srcmac();
            m.dst_mac = wlan->dstmac();
            m.payload = byteMessage;

            if (wlan->ethertype() == ETHERTYPE_CAR){
                communication_module->insert_wlan_msg_buffer(m, veins::realWorldDevices::dut, tp);
            }else{
                std::cout << "LR_COM: received something different than wlan message." << std::endl;
            }
        }
    }
    std::cout << "LR_COM: dut receive thread: before closing syncSocket" << std::endl;
    syncSocket->close();
    delete syncSocket;
    std::cout << "LR_COM: dut receive thread: before closing dut Socket" << std::endl;
    dut->close();
    delete dut;
    std::cout << "LR_COM: dut: deleted all sockets." << std::endl;
}

void LanradioCommunication::insert_wlan_msg_buffer(wlan_msg m, realWorldDevices pDev, std::chrono::steady_clock::time_point tp)
{
    if (pDev == realWorldDevices::simulationInterface) {
        mutex_simulationInterface_buffer.lock();
        simIntReceiveQueue.push(std::make_pair(tp, m));
        mutex_simulationInterface_buffer.unlock();
        // std::cout << "LR_COM: SimInt: inserted packet in receiveQueue at: " << simTime() << std::endl;
        mutex_simInt_check_msg.lock();
        // if (check_simint_message_buffer->getOwner() == this) {
        if (ownSimIntCheckMsg) {
            // std::cout << "LR_COM: SimInt: check_simint_message_buffer is owned by communication_module." << std::endl;
            if (!check_simint_message_buffer->isScheduled()){
                check_simint_message_buffer->setArrival(getId(), -1, simTime());
                getSimulation()->insertEvent(check_simint_message_buffer);
                ownSimIntCheckMsg = false;
                // std::cout << "LR_COM: SimInt: scheduled check_simint_message_buffer at: " << simTime() << std::endl;
            }// else{
                // std::cout << "LR_COM: SimInt: check_simint_message_buffer is alread scheduled, at: " << simTime() << std::endl;
            // }
        }// else{
            // std::cout << "LR_COM: communication_module is not owner of check_simint_message_buffer." << std::endl;
        // }
        mutex_simInt_check_msg.unlock();
    }else if (pDev == realWorldDevices::dut) {
        mutex_dut_buffer.lock();
        dutReceiveQueue.push(std::make_pair(tp, m));
        mutex_dut_buffer.unlock();
        // std::cout << "LR_COM: DUT: inserted packet in receiveQueue at: " << simTime() << std::endl;
        mutex_dut_check_msg.lock();
        // if (check_dut_message_buffer->getOwner() == this) {
        if (ownDutCheckMsg) {
            // std::cout << "LR_COM: DUT: check_dut_message_buffer is owned by communication_module." << std::endl;
            if (!check_dut_message_buffer->isScheduled()){
                check_dut_message_buffer->setArrival(getId(), -1, simTime());
                getSimulation()->insertEvent(check_dut_message_buffer);
                ownDutCheckMsg = false;
                // std::cout << "LR_COM: DUT: scheduled check_dut_message_buffer at: " << simTime() << std::endl;
            }// else{
                // std::cout << "LR_COM: DUT: check_dut_message_buffer is alread scheduled, at: " << simTime() << std::endl;
            // }
        }// else{
            // std::cout << "LR_COM: DUT: communication_module is not owner of check_dut_message_buffer." << std::endl;
        // }
        mutex_dut_check_msg.unlock();
    }
}

void LanradioCommunication::handleMessage(cMessage* msg)
{
    take(msg);
    switch (msg->getKind()) {
        case Lanradio_CHECK_SIMSEC_PER_SEC: {
            // simsec per sec
            std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
            simsecPerSec.record(std::chrono::duration<double, std::nano>(now - lastSecTimestamp).count());
            lastSecTimestamp = now;
            // data rate physical channel
            dataRateDut.record(packetsPerSecDut);
            dataRateSimInt.record(packetsPerSecSimInt);
            dataRateOverall.record(packetsPerSecOverall);
            packetsPerSecDut = 0;
            packetsPerSecSimInt = 0;
            packetsPerSecOverall = 0;
            // transmitted packets
            transmittedPPS_Overall.record(transmittedPacketsPerSec_Overall);
            transmittedPacketsPerSec_Overall = 0;
            transmittedPPS_SimIntDut.record(transmittedPacketsPerSec_SimIntDut);
            transmittedPacketsPerSec_SimIntDut = 0;
            transmittedPPS_DutSimInt.record(transmittedPacketsPerSec_DutSimInt);
            transmittedPacketsPerSec_DutSimInt = 0;
            // lost packets
            lostPPS_SimIntDut.record(simIntMessageMap.size() - lostPacketsLastSecond_SimIntDut);
            lostPPS_DutSimInt.record(dutMessageMap.size() - lostPacketsLastSecond_DutSimInt);
            lostPPS_Overall.record((simIntMessageMap.size() + dutMessageMap.size()) - lostPacketsLastSecond_SimIntDut - lostPacketsLastSecond_DutSimInt);
            lostPacketsLastSecond_SimIntDut = simIntMessageMap.size();
            lostPacketsLastSecond_DutSimInt = dutMessageMap.size();
            // re-schedule
            scheduleAt(simTime() + 1, check_simsec_per_sec);
            break;
        }
        case Lanradio_CHECK_SIMINT_MSG_BUFFER: {
            mutex_simInt_check_msg.lock();
            ownSimIntCheckMsg = true;
            mutex_simInt_check_msg.unlock();
            // std::cout << "LR_COM: process Lanradio_CHECK_SIMINT_MSG_BUFFER." << std::endl;
            process_simInt_message_buffer();
            simIntReceiveBufferSize.record(simIntReceiveQueue.size());
            break;
        }
        case Lanradio_CHECK_DUT_MSG_BUFFER: {
            mutex_dut_check_msg.lock();
            ownDutCheckMsg = true;
            mutex_dut_check_msg.unlock();
            // std::cout << "LR_COM: process Lanradio_CHECK_DUT_MSG_BUFFER." << std::endl;
            process_dut_message_buffer();
            dutReceiveBufferSize.record(dutReceiveQueue.size());
            break;
        }
        default: {
            if (msg != nullptr) {
                if (lara_debug) std::cerr << "APP: Error: Got Self Message of unknown kind! Name: " << msg->getName() << endl;
                delete msg;
            }
            break;
        }
    }
}

void LanradioCommunication::process_simInt_message_buffer()
{
    mutex_simulationInterface_buffer.lock();
    while (!simIntReceiveQueue.empty()) {
        auto simIntPacket = simIntReceiveQueue.front();
        received_WLAN(simIntPacket.second, realWorldDevices::simulationInterface, simIntPacket.first);
        simIntReceiveQueue.pop();
    }
    mutex_simulationInterface_buffer.unlock();
}

void LanradioCommunication::process_dut_message_buffer()
{
    mutex_dut_buffer.lock();
    while (!dutReceiveQueue.empty()) {
        auto dutPacket = dutReceiveQueue.front();
        received_WLAN(dutPacket.second, realWorldDevices::dut, dutPacket.first);
        dutReceiveQueue.pop();
    }
    mutex_dut_buffer.unlock();
}

void LanradioCommunication::initialize(int stage)
{
    sleep(5);   // to avoid zmq::error_t: Address already in use when a new run is started.
    /*
     * setup zmq socket for configuring Lanradio
     */
    if (stage == 0) {

        /* reset ethernet interface */
        // int sysReturn = 0;
        // sysReturn = system("ip link set enp0s31f6 down");
        // std::cout << "LR_COM: set ethernet interface down, ret value: " << sysReturn << std::endl;
        // sleep(2);
        // sysReturn = system("ip link set enp0s31f6 up");
        // std::cout << "LR_COM: set ethernet interface up, ret value: " << sysReturn << std::endl;
        // sysReturn = system("tc qdisc replace dev enp0s31f6 root handle 1: pfifo");
        // std::cout << "LR_COM: set qdisc to pfifo, ret value: " << sysReturn << std::endl;

        use_lanradio = par("use_lanradio").boolValue();
        use_udp = par("use_udp").boolValue();
        baseport = par("baseport").intValue();
        simIntHostname = par("simIntHostname").stringValue();
        dutHostname = par("dutHostname").stringValue();
        wlan_interface = par("wlan_interface").stringValue();
        bitrates = par("bitrates").stringValue();
        wlan_ip = par("wlan_ip").stringValue();
        freq = par("freq").stringValue();
        bandwidth = par("bandwidth").stringValue();
        lara_debug = par("lara_debug").boolValue();
        numEvaluationBytes = par("numEvaluationBytes").intValue();
        experimentName = par("experimentName").stringValue();
        
        SimInterfaceTxPower = par("SimInterfaceTxPower").stringValue();
        DutTxPower = par("DutTxPower").stringValue();
        SimInterfaceModifyMac = par("SimInterfaceModifyMac").boolValue();

        /* logging */
        receivedBytesDut = 0;
        receivedPacketsDut = 0;
        receivedBytesFellowVehicles= 0;
        receivedPacketsFellowVehicles= 0;
        sendBytesDut = 0;
        sendPacketsDut = 0;
        sendBytesFellowVehicles = 0;
        sendPacketsFellowVehicles = 0;

        packetsPerSecSimInt = 0;
        packetsPerSecDut = 0;
        packetsPerSecOverall = 0;

        lostPacketsLastSecond_SimIntDut = 0;
        lostPacketsLastSecond_DutSimInt = 0;
        lostPPS_Overall.setName("lostPacketsPerSecOverall");
        lostPPS_SimIntDut.setName("lostPacketsPerSecSimIntDut");
        lostPPS_DutSimInt.setName("lostPacketsPerSecDutSimInt");

        transmittedPacketsPerSec_Overall = 0;
        transmittedPacketsPerSec_SimIntDut = 0;
        transmittedPacketsPerSec_DutSimInt = 0;
        transmittedPPS_Overall.setName("transmittedPacketsPerSecOverall");
        transmittedPPS_SimIntDut.setName("transmittedPacketsPerSecSimIntDut");
        transmittedPPS_DutSimInt.setName("transmittedPacketsPerSecDutSimInt");

        dutToSimIntLat.setName("dutToSimIntLat");
        simIntToDutLat.setName("simIntToDutLat");
        simsecPerSec.setName("simsecPerSec");
        simIntReceiveBufferSize.setName("simIntReceiveBufferSize");
        dutReceiveBufferSize.setName("dutReceiveBufferSize");
        wrongRunNumberPackets.setName("wrongRunNumberPackets");
        wrongEvalSequenceNumberPacketsSimInt.setName("wrongEvalSequenceNumberPacketsSimInt");
        wrongEvalSequenceNumberPacketsDUT.setName("wrongEvalSequenceNumberPacketsDUT");
        dataRateDut.setName("dataRateDut");
        dataRateSimInt.setName("dataRateSimInt");
        dataRateOverall.setName("dataRateOverall");

        evalSequenceNumber = 0;
        /* ------- */
        runNumber = getEnvir()->getConfigEx()->getActiveRunNumber();

        check_simint_message_buffer = new cMessage("check simint msg buffer", veins::LanradioCommunication::Lanradio_CHECK_SIMINT_MSG_BUFFER);
        check_dut_message_buffer = new cMessage("check dut msg buffer", veins::LanradioCommunication::Lanradio_CHECK_DUT_MSG_BUFFER);

        if (check_simint_message_buffer->getOwner() == this) {
            mutex_simInt_check_msg.lock();
            ownSimIntCheckMsg = true;
            mutex_simInt_check_msg.unlock();
            // std::cout << "LR_COM: owning check_simint_message_buffer." << std::endl;
        }else{
            std::cout << "LR_COM: ERROR: newly created check_simint_message_buffer is not owned by lanradio module." << std::endl;
        }
        if (check_dut_message_buffer->getOwner() == this) {
            mutex_dut_check_msg.lock();
            ownDutCheckMsg = true;
            mutex_dut_check_msg.unlock();
            std::cout << "LR_COM: owning check_dut_message_buffer." << std::endl;
        }else{
            std::cout << "LR_COM: ERROR: newly created check_dut_message_buffer is not owned by lanradio module." << std::endl;
        }

        std::cout << "steady clock is steady? " << std::chrono::steady_clock::is_steady << std::endl;

        if (use_lanradio){
            activate_lanradio();

            mContext = new zmq::context_t(1);

            // Radio configuration sockets
            // radioConfigSocket = new zmq::socket_t(*mContext, ZMQ_PUB);
            // radioConfigSocket->setsockopt(ZMQ_SNDTIMEO, -1);
            // radioConfigSocket->setsockopt(ZMQ_LINGER, 0);
            // radioConfigSocket->bind(("tcp://*:" + std::to_string(baseport)).c_str());

            // syncSocketRadio = new zmq::socket_t(*mContext, ZMQ_REP);
            // syncSocketRadio->setsockopt(ZMQ_RCVTIMEO, 100);
            // syncSocketRadio->setsockopt(ZMQ_LINGER, 0);
            // syncSocketRadio->setsockopt(ZMQ_SNDTIMEO, -1);
            // syncSocketRadio->bind(("tcp://*:" + std::to_string(baseport + 1)).c_str());

            simIntRadioConfigSocket = new zmq::socket_t(*mContext, ZMQ_REQ);
            simIntRadioConfigSocket->setsockopt(ZMQ_RCVTIMEO, -1);
            simIntRadioConfigSocket->setsockopt(ZMQ_LINGER, 0);
            simIntRadioConfigSocket->setsockopt(ZMQ_SNDTIMEO, -1);
            simIntRadioConfigSocket->bind(("tcp://*:" + std::to_string(baseport)).c_str());
            dutRadioConfigSocket = new zmq::socket_t(*mContext, ZMQ_REQ);
            dutRadioConfigSocket->setsockopt(ZMQ_RCVTIMEO, -1);
            dutRadioConfigSocket->setsockopt(ZMQ_LINGER, 0);
            dutRadioConfigSocket->setsockopt(ZMQ_SNDTIMEO, -1);
            dutRadioConfigSocket->bind(("tcp://*:" + std::to_string(baseport + 1)).c_str());
            chanloadConfigSocket = new zmq::socket_t(*mContext, ZMQ_REQ);
            chanloadConfigSocket->setsockopt(ZMQ_RCVTIMEO, -1);
            chanloadConfigSocket->setsockopt(ZMQ_LINGER, 0);
            chanloadConfigSocket->setsockopt(ZMQ_SNDTIMEO, -1);
            chanloadConfigSocket->bind(("tcp://*:" + std::to_string(baseport + 9)).c_str());
            std::cout << "LR_COM: radio config socket PORTs: simInt" << baseport << ", dut: " << baseport + 1 << ", chanload: " << baseport + 9 << std::endl;

            // Simulation Interface sockets
            simIntPublisher = new zmq::socket_t(*mContext, ZMQ_PUB);
            simIntPublisher->setsockopt(ZMQ_SNDTIMEO, -1);
            simIntPublisher->setsockopt(ZMQ_LINGER, 0);
            if (!use_udp) {
                simIntPublisher->bind(("tcp://*:" + std::to_string(baseport + 2)).c_str());
            }

            syncSocketSimInt = new zmq::socket_t(*mContext, ZMQ_REP);
            syncSocketSimInt->setsockopt(ZMQ_RCVTIMEO, 500);
            syncSocketSimInt->setsockopt(ZMQ_LINGER, 0);
            syncSocketSimInt->setsockopt(ZMQ_SNDTIMEO, -1);
            if (!use_udp) {
                syncSocketSimInt->bind(("tcp://*:" + std::to_string(baseport + 3)).c_str());
                std::cout << "LR_COM: SIMINT PUB socket PORT: " << baseport + 2 << ", syncSocket port: " << baseport + 3 << std::endl;
            }

            // Dut sockets
            dutPublisher = new zmq::socket_t(*mContext, ZMQ_PUB);
            dutPublisher->setsockopt(ZMQ_SNDTIMEO, -1);
            dutPublisher->setsockopt(ZMQ_LINGER, 0);
            if (!use_udp) {
                dutPublisher->bind(("tcp://*:" + std::to_string(baseport + 4)).c_str());
            }

            syncSocketDut = new zmq::socket_t(*mContext, ZMQ_REP);
            syncSocketDut->setsockopt(ZMQ_LINGER, 0);
            syncSocketDut->setsockopt(ZMQ_RCVTIMEO, 500);
            syncSocketDut->setsockopt(ZMQ_SNDTIMEO, -1);
            if (!use_udp) {
                syncSocketDut->bind(("tcp://*:" + std::to_string(baseport + 5)).c_str());
                std::cout << "LR_COM: DUT PUB socket PORT: " << baseport + 4 << ", syncSocket port: " << baseport + 5 << std::endl;
            }

            /* create udp sockets */
            int rcSimInt;
            int rcDut;
            struct sockaddr_in myAddrSimInt, myAddrDut;
            memset(&myAddrSimInt, 0, sizeof(myAddrSimInt));
            memset(&myAddrDut, 0, sizeof(myAddrDut));

            // Simulation Interface
            simIntSocketUDP = socket (AF_INET, SOCK_DGRAM, 0);
            if (simIntSocketUDP < 0) {
                std::cout << "LR_COM: error creating simInt udp socket: " << simIntSocketUDP << std::endl;;
                exit (EXIT_FAILURE);
            }

            myAddrSimInt.sin_family = AF_INET;
            myAddrSimInt.sin_addr.s_addr = htonl (INADDR_ANY);
            myAddrSimInt.sin_port = htons (baseport + 6);

            const int optval = 1;
            setsockopt(simIntSocketUDP, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(int));
            if (use_udp) {
                rcSimInt = bind (simIntSocketUDP, (struct sockaddr *) &myAddrSimInt, sizeof(myAddrSimInt));
                if (rcSimInt < 0) {
                    std::cout << "LR_COM: simtInt udp socket could not bind to port: " << baseport + 6 << std::endl;
                    exit (EXIT_FAILURE);
                }
                std::cout << "LR_COM: simInt udp socket waiting for data on port: " << baseport + 6 << std::endl;
            }

            // Dut
            dutSocketUDP = socket (AF_INET, SOCK_DGRAM, 0);
            if (dutSocketUDP < 0) {
                std::cout << "LR_COM: error creating dut udp socket: " << dutSocketUDP << std::endl;;
                exit (EXIT_FAILURE);
            }

            myAddrDut.sin_family = AF_INET;
            myAddrDut.sin_addr.s_addr = htonl (INADDR_ANY);
            myAddrDut.sin_port = htons (baseport + 8);

            setsockopt(dutSocketUDP, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(int));
            if (use_udp) {
                rcDut = bind (dutSocketUDP, (struct sockaddr *) &myAddrDut, sizeof(myAddrDut));
                if (rcDut < 0) {
                    std::cout << "LR_COM: dut udp socket could not bind to port: " << baseport + 8 << std::endl;
                    exit (EXIT_FAILURE);
                }
                std::cout << "LR_COM: dut udp socket waiting for data on port: " << baseport + 8 << std::endl;
            }

            // client addresses
            // simInt
            struct hostent* hostinfo;
            hostinfo = gethostbyname(simIntHostname.c_str());
            if (hostinfo == NULL) {
                std::cout << "LR_COM: Error erroneous simInt client hostname: " << simIntHostname << std::endl;
            }
            simIntAddr.sin_family = AF_INET;
            simIntAddr.sin_addr = *(struct in_addr *) hostinfo->h_addr;
            simIntAddr.sin_port = htons (baseport + 6);
            // dut
            hostinfo = gethostbyname(dutHostname.c_str());
            if (hostinfo == NULL) {
                std::cout << "LR_COM: Error erroneous dut client hostname: " << dutHostname << std::endl;
            }
            dutAddr.sin_family  = AF_INET;
            dutAddr.sin_addr = *(struct in_addr *) hostinfo->h_addr;
            dutAddr.sin_port = htons (baseport + 8);

            // start receive threads
            if (lara_debug) std::cerr << "starting simulationInterface_receive_thread" << std::endl;
            simulationInterface_receive_thread = new std::thread(thread_receive_simInt, simIntHostname, this, lara_debug, mContext, baseport + 6, use_udp);

            if (lara_debug) std::cerr << "starting dut_receive_thread" << std::endl;
            dut_receive_thread = new std::thread(thread_receive_dut, dutHostname, this, lara_debug, mContext, baseport + 8, use_udp);

            // Synchronize radio configuration sockets
            std::string message;
            // int radioConfigAcks = 0;
            // int retries = 0;
            // while(radioConfigAcks < 2) {
            //     radioConfig_send(*radioConfigSocket, "SYNC");
            //     std::cout << "LR_COM: radioConfig SYNC: sent SYNC" << std::endl;
            //     message = radioConfig_recv(*syncSocketRadio).first;
            //     if (message == "ACK") {
            //         radioConfigAcks++;
            //         std::cout << "LR_COM: RADIO subscriber acked." << std::endl;
            //         radioConfig_send(*syncSocketRadio, "START");
            //     }else{
            //         std::cout << "LR_COM: received something else or timeout: " << message << std::endl;
            //         std::cout << "LR_COM: not enough ACKs received. Current number of radioConfigAcks: " << radioConfigAcks << std::endl;
            //         retries++;
            //     }
            //     message = radioConfig_recv(*syncSocketRadio).first;
            //     if (message == "ACK") {
            //         radioConfigAcks++;
            //         std::cout << "LR_COM: RADIO subscriber acked." << std::endl;
            //         radioConfig_send(*syncSocketRadio, "START");
            //     }else{
            //         std::cout << "LR_COM: received something else or timeout: " << message << std::endl;
            //         std::cout << "LR_COM: not enough ACKs received. Current number of radioConfigAcks: " << radioConfigAcks << std::endl;
            //         retries++;
            //     }
            //     // if(retries == 200){
            //     //     std::cout << "LR_COM: number of retries exceeded. Stopping run." << std::endl;
            //     //     deactivate_radio();
            //     //     std::cout << "LR_COM: deactivate radios." << std::endl;
            //     //     throw;
            //     // }
            // }
            // Synchronize PUB-SUB connections
            if (!use_udp) {
                // Synchronize SimulationInterface
                while(true) {
                    // zmq_sndmore(*lanRadioPublisher, "SIMINT");
                    // zmq_sndmore(*lanRadioPublisher, "SYNC");
                    lanradio_send(*simIntPublisher, "SYNC", use_udp, -1, nullptr);
                    message = lanradio_recv(*syncSocketSimInt, use_udp, -1).first;
                    if (message == "ACK") {
                        std::cout << "LR_COM: SIMINT subscriber sync." << std::endl;
                        lanradio_send(*syncSocketSimInt, "START", use_udp, -1, nullptr);
                        break;
                    }else{
                        std::cout << "LR_COM: no ACK received. SimInt sync socket received: " << message << std::endl;
                    }
                }
                // Synchronize Dut
                while(true) {
                    // zmq_sndmore(*lanRadioPublisher, "DUT");
                    // zmq_sndmore(*lanRadioPublisher, "SYNC");
                    lanradio_send(*dutPublisher, "SYNC", use_udp, -1, nullptr);
                    message = lanradio_recv(*syncSocketDut, use_udp, -1).first;
                    if (message == "ACK") {
                        std::cout << "LR_COM: DUT subscriber sync." << std::endl;
                        lanradio_send(*syncSocketDut, "START", use_udp, -1, nullptr);
                        break;
                    }else{
                        std::cout << "LR_COM: no ACK received. Dut sync socket received: " << message << std::endl;
                    }
                }

                while (!dutSubSync || !simIntSubSync) {
                    std::cout << "LR_COM: Subscriber threads are not synced yet" << std::endl;
                    usleep(500);
                }

                std::cout << "LR_COM: Everything synchronized, configuring radios." << std::endl;
            }

            std::cout << "LR_COM: Configured radios, starting simulation." << std::endl;
            // start chanload application
            chanload_send_command("chanload wlan1 " + freq + " 100ms 0 > /tmp/chanload_" + std::to_string(runNumber) + ".txt &");

            check_simsec_per_sec = new cMessage("check simsec per sec", Lanradio_CHECK_SIMSEC_PER_SEC);
            lastSecTimestamp = std::chrono::steady_clock::now();
            scheduleAt(simTime() + 1, check_simsec_per_sec);
        }
    }
}

void LanradioCommunication::activate_lanradio()
{
//     // configure queueing discipline
//     radio_send_command("tc qdisc replace dev eth0 root handle 1: pfifo");
//     radio_send_command("tc qdisc add dev " + wlan_interface + " root handle 2: pfifo");
//     // configure lanradio interface
//     radio_send_command("iw reg set DE");
//     radio_send_command("ip link set " + wlan_interface + " down");
//     radio_send_command("iw dev " + wlan_interface + " set type ocb");
//     radio_send_command("ip link set " + wlan_interface + " up");
//     radio_send_command("iw dev " + wlan_interface + " set bitrates " + bitrates);       // set bitrate here, setting the bitrate after ocb join command fails
//     radio_send_command("iw dev " + wlan_interface + " ocb join " + freq + " " + bandwidth);
//     radio_send_command("ifconfig " + wlan_interface + " " + wlan_ip);
//     radio_send_command("iw dev " + wlan_interface + " set txpower fixed " + txpower);
//     // configure chanload interface
//     radio_send_command("ip link set wlan0 down");
//     radio_send_command("iw dev wlan0 set type ocb");
//     radio_send_command("ip link set wlan0 up");
//     radio_send_command("iw dev wlan0 set bitrates " + bitrates);       // set bitrate here, setting the bitrate after ocb join command fails
//     radio_send_command("iw dev wlan0 ocb join " + freq + " " + bandwidth);
//     radio_send_command("ifconfig wlan0 192.168.55.2");
//     radio_send_command("iw dev wlan0 set txpower fixed " + txpower);
    // Start SimInterface
    std::stringstream cmdSimInterface;
    cmdSimInterface << std::boolalpha << "ssh root@10.0.197.103 lanradio --transmitPower " << SimInterfaceTxPower << " --modifyMac " << SimInterfaceModifyMac << " --useUDP true --dualRadio true --VeinsHostname 10.0.197.200 &";
    std::string startSimInterface = cmdSimInterface.str();
    std::cout << "Start SimInterface cmd: " <<  startSimInterface << std::endl;
    redi::ipstream startSimInterfaceProc(startSimInterface, redi::pstreams::pstdout | redi::pstreams::pstderr);
    startSimInterfaceProc.close();

    // Start DUT
    std::stringstream cmdDut;
    cmdDut << std::boolalpha << "ssh root@10.0.197.104 lanradio --transmitPower " << DutTxPower << " --modifyMac false --interface wlan1 --useUDP true --prototype true --VeinsHostname 10.0.197.200 &";
    std::string startDUT = cmdDut.str();
    std::cout << "Start DUT cmd: " <<  startDUT << std::endl;
    redi::ipstream startDUTProc(startDUT, redi::pstreams::pstdout | redi::pstreams::pstderr);
    startDUTProc.close();
    // start pending frames script
    // radio_send_command("bash /root/pending_frames.sh > /tmp/pending_frames_" + std::to_string(runNumber) + ".txt &");
    // setCarrierSensingOff(true, *simIntRadioConfigSocket);
}

void LanradioCommunication::deactivate_radio()
{
//     // shutdown lanradio interface
//     radio_send_command("iw dev " + wlan_interface + " ocb leave");
//     radio_send_command("ip link set " + wlan_interface + " down");
//     radio_send_command("ifconfig " + wlan_interface + " down");
//     // shutdown chanload interface
//     radio_send_command("iw dev wlan0 ocb leave");
//     radio_send_command("ip link set wlan0 down");
//     radio_send_command("ifconfig wlan0 down");
    // kill chanload
    chanload_send_command("killall chanload");
    // copy chanload file
    sleep(1);
    copyChanloadFile(*chanloadConfigSocket, "alix5", experimentName, std::to_string(runNumber));
    // kill pending frames script
    //radio_send_command("killall bash");
    // copy pending frames file
    //sleep(1);
    //copyPendingFramesFile(*simIntRadioConfigSocket, "alix3", experimentName, std::to_string(runNumber));
    //copyPendingFramesFile(*dutRadioConfigSocket, "alix4", experimentName, std::to_string(runNumber));
    // restart lanradio
    if (!use_udp) {
        // radio_send_command("killall lanradio && if [ \"$(cat /sys/class/net/eth0/address)\" == \"00:0d:b9:2e:c3:dc\" ]; then lanradio --VeinsHostname 10.0.197.200 > /tmp/log_" + std::to_string(runNumber) + ".txt & elif [ \"$(cat /sys/class/net/eth0/address)\" == \"00:0d:b9:2e:c3:a4\" ]; then lanradio --prototype true --VeinsHostname 10.0.197.200 > $HOME/log_" + std::to_string(rn) + ".txt & fi");
        // radio_send_command("killall lanradio && if [ \"$(cat /sys/class/net/eth0/address)\" == \"00:0d:b9:2e:c3:dc\" ]; then lanradio --VeinsHostname 10.0.197.200 & elif [ \"$(cat /sys/class/net/eth0/address)\" == \"00:0d:b9:2e:c3:a4\" ]; then lanradio --prototype true --VeinsHostname 10.0.197.200 & fi");
    }else{
        // radio_send_command("killall lanradio && if [ \"$(cat /sys/class/net/eth0/address)\" == \"00:0d:b9:2e:c3:dc\" ]; then lanradio --useUDP true --VeinsHostname 10.0.197.200 > $HOME/log_" + std::to_string(runNumber) + ".txt & elif [ \"$(cat /sys/class/net/eth0/address)\" == \"00:0d:b9:2e:c3:a4\" ]; then lanradio --useUDP true --prototype true --VeinsHostname 10.0.197.200 > $HOME/log_" + std::to_string(rn) + ".txt & fi");
        // radio_send_command("killall lanradio && if [ \"$(cat /sys/class/net/eth0/address)\" == \"00:0d:b9:2e:c3:dc\" ]; then lanradio --useUDP true --dualRadio true --VeinsHostname 10.0.197.200 > /tmp/log.txt & elif [ \"$(cat /sys/class/net/eth0/address)\" == \"00:0d:b9:2e:c3:a4\" ]; then lanradio --useUDP true --prototype true --VeinsHostname 10.0.197.200 > /tmp/log.txt & fi");
        // use this one lastest command: radio_send_command("killall lanradio && if [ \"$(cat /sys/class/net/eth0/address)\" == \"00:0d:b9:58:ab:8c\" ]; then lanradio --useUDP true --dualRadio true --VeinsHostname 10.0.197.200 > /tmp/log.txt & elif [ \"$(cat /sys/class/net/eth0/address)\" == \"00:0d:b9:58:ac:ec\" ]; then lanradio --interface wlan1 --useUDP true --prototype true --VeinsHostname 10.0.197.200 > /tmp/log.txt & fi");
        //radio_send_command("killall lanradio && if [ \"$(cat /sys/class/net/eth0/address)\" == \"00:0d:b9:2e:c3:dc\" ]; then lanradio --useUDP true --VeinsHostname 10.0.197.200 > /tmp/log.txt & elif [ \"$(cat /sys/class/net/eth0/address)\" == \"00:0d:b9:2e:c3:a4\" ]; then lanradio --useUDP true --prototype true --VeinsHostname 10.0.197.200 > /tmp/log.txt & fi");
        // radio_send_command("killall lanradio && if [ \"$(cat /sys/class/net/eth0/address)\" == \"00:0d:b9:2e:c3:dc\" ]; then lanradio --useUDP true --VeinsHostname 10.0.197.200 & elif [ \"$(cat /sys/class/net/eth0/address)\" == \"00:0d:b9:2e:c3:a4\" ]; then lanradio --useUDP true --prototype true --VeinsHostname 10.0.197.200 & fi");
        radio_send_command("killall lanradio");
    }
    // std::cout << "LR_COM: before work around call." << std::endl;
    // radioConfigEnd(*simIntSocketUDP *dutSocketUDP);
    std::cout << "LR_COM: deactivate_radio successful" << std::endl;
}

bool LanradioCommunication::radio_send_command(const std::string cmd)
{
    try {
        bool successful;
        // successful = zmq_sndmore(*lanRadioPublisher, "ALL");
        // successful = zmq_sndmore(*lanRadioPublisher, "RADIO_CONFIG");
        successful = radioConfig_send(*simIntRadioConfigSocket, *dutRadioConfigSocket, cmd).first;
        if (!successful) {
            error("connection to Lanradio was unsuccessful");
        }
        std::cout << "LR_COM: radioconfig sent: " << cmd << std::endl;
        return successful;
    }
    catch (zmq::error_t) {
        error("connection to Lanradio was unsuccessful");
    }
}

bool LanradioCommunication::chanload_send_command(const std::string cmd)
{
    try {
        bool successful;
        // successful = zmq_sndmore(*lanRadioPublisher, "ALL");
        // successful = zmq_sndmore(*lanRadioPublisher, "RADIO_CONFIG");
        successful = chanloadConfig_send(*chanloadConfigSocket, cmd);
        if (!successful) {
            error("connection to chanload was unsuccessful");
        }
        std::cout << "LR_COM: chanload config sent: " << cmd << std::endl;
        return successful;
    }
    catch (zmq::error_t) {
        error("connection to chanload was unsuccessful");
    }
}

void LanradioCommunication::finish()
{
    if (use_lanradio) {
        /* logging */
        recordScalar("receivedBytesDut", receivedBytesDut);
        recordScalar("receivedPacketsDut", receivedPacketsDut);
        recordScalar("receivedBytesFellowVehicles", receivedBytesFellowVehicles);
        recordScalar("receivedPacketsFellowVehicles", receivedPacketsFellowVehicles);
        recordScalar("sendBytesDut", sendBytesDut);
        recordScalar("sendPacketsDut", sendPacketsDut);
        recordScalar("sendBytesFellowVehicles", sendBytesFellowVehicles);
        recordScalar("sendPacketsFellowVehicles", sendPacketsFellowVehicles);

        /* shut everything down */
        deactivate_radio();

        simIntPublisher->close();
        delete simIntPublisher;

        syncSocketSimInt->close();
        delete syncSocketSimInt;

        dutPublisher->close();
        delete dutPublisher;

        syncSocketDut->close();
        delete syncSocketDut;

        // radioConfigSocket->close();
        // delete radioConfigSocket;

        // syncSocketRadio->close();
        // delete syncSocketRadio;

        simIntRadioConfigSocket->close();
        delete simIntRadioConfigSocket;

        dutRadioConfigSocket->close();
        delete dutRadioConfigSocket;

        chanloadConfigSocket->close();
        delete chanloadConfigSocket;

        zmq_ctx_term(mContext);
        delete mContext;

        simulationInterface_receive_thread->join();
        std::cout << "LR_COM: simulationInterface_receive_thread joined" << std::endl;
        dut_receive_thread->join();
        std::cout << "LR_COM: dut_receive_thread joined" << std::endl;
        delete simulationInterface_receive_thread;
        delete dut_receive_thread;
        cancelAndDelete(check_simint_message_buffer);
        cancelAndDelete(check_dut_message_buffer);

        std::cout << "LR_COM: Lanradio communication finish()!" << std::endl;
    }
}

/*
 * send a cam generated by non-virtual prototype to physical prototype (simulationToPrototype)
 * OR
 * send a cam generated by virtual prototype to physical prototype in order to send it back to the fellow vehicles (prototypeToSimulation)
 */
void LanradioCommunication::send_WLAN(void* data, simtime_t simulationCreationTime, t_wlan_type type, std::string srcmac, std::string dstmac, realWorldDevices pDev)
{

    if (type == wlan_CAM) {
        CAM_t* cam = static_cast<CAM_t*>(data);

        // Sequence numbers
        cam->cam.camParameters.evaluationContainer = static_cast<EvaluationContainer_t*>(calloc(1, sizeof(EvaluationContainer_t)));
        ASSERT(cam->cam.camParameters.evaluationContainer != nullptr);
        cam->cam.camParameters.evaluationContainer->evalSequenceNumber = evalSequenceNumber;
        evalSequenceNumber++;
        cam->cam.camParameters.evaluationContainer->runNumber = runNumber;
        // std::cout << "LR_COM: generated CAM with sequence nummber : " << cam->cam.camParameters.evaluationContainer->evalSequenceNumber << std::endl;

        // Evaluation byte buffer
        if (numEvaluationBytes > 0 ) {
            cam->cam.camParameters.evaluationContainer->byteBuffer = static_cast<ByteBuffer_t*>(calloc(1, sizeof(ByteBuffer_t)));
            cam->cam.camParameters.evaluationContainer->byteBuffer->buf = static_cast<uint8_t*>(calloc(1, numEvaluationBytes));
            cam->cam.camParameters.evaluationContainer->byteBuffer->size = numEvaluationBytes;
            // memcpy(cam->cam.camParameters.evaluationContainer->byteBuffer.buf, generationTimestamp.c_str(), cam->cam.camParameters.evaluationContainer->byteBuffer.size);
            RAND_bytes(cam->cam.camParameters.evaluationContainer->byteBuffer->buf, numEvaluationBytes);
        }

        std::vector<uint8_t> encodedCam = encodeMessage(&asn_DEF_CAM, cam);
        std::string strCam(encodedCam.begin(), encodedCam.end());

        std::string serializedData;
        wlanPackage::WLAN wlan;

        // create packet with geonet and btp header
        unsigned int geoHdrLen;
        uint8_t* geoHdr;

        fillGeoNetBTPheaderForCam(cam, strCam.size(), srcmac);
        geoHdrLen = sizeof(struct GeoNetworkAndBTPHeaderCAM);
        geoHdr = reinterpret_cast<uint8_t*>(&mGeoBtpHdrForCam);

        unsigned int packetsize = geoHdrLen + strCam.size();
        //std::cout << "Transmitting cam: camPDULen: " << strCam.size() << ", geoNetBTPCAMLength: " << packetsize << std::endl;
        unsigned char packet[packetsize];
        unsigned char* payload = packet + geoHdrLen;
        unsigned char* geoNetHdr = packet;

        // Fill in geo networking and btp header
        memcpy(geoNetHdr, geoHdr, geoHdrLen);

        // copy payload to packet
        memcpy(payload, strCam.c_str(), strCam.size());
        // std::cout << "LR_COM: send_WLAN packet size including GeoNet and BTP: " << packetsize << " bytes, camPDULen: " << strCam.size() << std::endl;
        // end create packet

        wlan.set_priority(wlanPackage::WLAN_Priority_BE);
        wlan.set_ethertype(ETHERTYPE_CAR);
        wlan.set_payload(packet, packetsize);
        wlan.set_srcmac(srcmac);
        wlan.set_dstmac(dstmac);
        wlan.SerializeToString(&serializedData);

        if (pDev == realWorldDevices::simulationInterface){
            /* sending */
            // zmq_sndmore(*lanRadioPublisher, "SIMINT");
            // zmq_sndmore(*lanRadioPublisher, "DATA");
            std::pair<bool, std::chrono::steady_clock::time_point> r  = lanradio_send(*simIntPublisher, serializedData, use_udp, simIntSocketUDP, &simIntAddr);
            /* logging */
            mutex_simIntMessageMap.lock();
            simIntMessageMap.insert(std::pair<int, std::pair<std::chrono::steady_clock::time_point, simtime_t>>(cam->cam.camParameters.evaluationContainer->evalSequenceNumber, std::pair<std::chrono::steady_clock::time_point, simtime_t>(r.second, simulationCreationTime)));
            // std::cout << "LR_COM: simInt sending cam with evaluation number: " << cam->cam.camParameters.evaluationContainer->evalSequenceNumber << ", simIntMessageMap size: " << simIntMessageMap.size() << std::endl;
            mutex_simIntMessageMap.unlock();
            if (simTime() >= getSimulation()->getWarmupPeriod()) sendBytesFellowVehicles += packetsize;
            if (simTime() >= getSimulation()->getWarmupPeriod()) sendPacketsFellowVehicles++;
            transmittedPacketsPerSec_Overall++;
            transmittedPacketsPerSec_SimIntDut++;
            // std::cout << "LR_COM: fellow vehicles send packet of size: " << packetsize << std::endl;
        }else if(pDev == realWorldDevices::dut){
            /* sending */
            // zmq_sndmore(*lanRadioPublisher, "DUT");
            // zmq_sndmore(*lanRadioPublisher, "DATA");
            std::pair<bool, std::chrono::steady_clock::time_point> r = lanradio_send(*dutPublisher, serializedData, use_udp, dutSocketUDP, &dutAddr);
            /* logging */
            mutex_dutMessageMap.lock();
            dutMessageMap.insert(std::pair<int, std::pair<std::chrono::steady_clock::time_point, simtime_t>>(cam->cam.camParameters.evaluationContainer->evalSequenceNumber, std::pair<std::chrono::steady_clock::time_point, simtime_t>(r.second, simulationCreationTime)));
            // std::cout << "LR_COM: dut sending cam with evaluation number: " << cam->cam.camParameters.evaluationContainer->evalSequenceNumber << std::endl;
            mutex_dutMessageMap.unlock();
            if (simTime() >= getSimulation()->getWarmupPeriod()) sendBytesDut += packetsize;
            if (simTime() >= getSimulation()->getWarmupPeriod()) sendPacketsDut++;
            transmittedPacketsPerSec_Overall++;
            transmittedPacketsPerSec_DutSimInt++;
            // std::cout << "LR_COM: virtual prototype sends packet of size: " << packetsize << std::endl;
        }
        // asn_DEF_CAM.free_struct(&asn_DEF_CAM, cam, 0);  // FIXME: clean up at right time
    }
    else if (type == wlan_SPAT) {
        auto spat = static_cast<SPAT_PDU_t*>(data);

        std::vector<uint8_t> encodedSpat = encodeMessage(&asn_DEF_SPAT_PDU, spat);
        std::string strSpat(encodedSpat.begin(), encodedSpat.end());

        std::string serializedData;
        wlanPackage::WLAN wlan;

        // create packet with geonet and btp header
        unsigned int geoHdrLen;
        uint8_t* geoHdr;

        fillGeoNetBTPheaderForSpat(strSpat.size(), srcmac);
        geoHdrLen = sizeof(struct GeoNetworkAndBTPHeaderSPAT);
        geoHdr = reinterpret_cast<uint8_t*>(&mGeoBtpHdrForSpat);

        unsigned int packetsize = geoHdrLen + strSpat.size();
        unsigned char packet[packetsize];
        unsigned char* payload = packet + geoHdrLen;
        unsigned char* geoNetHdr = packet;

        // Fill in geo networking and btp header
        memcpy(geoNetHdr, geoHdr, geoHdrLen);

        // copy payload to packet
        memcpy(payload, strSpat.c_str(), strSpat.size());

        // end create packet

        wlan.set_priority(wlanPackage::WLAN_Priority_BE);
        wlan.set_ethertype(ETHERTYPE_CAR);

        // std::string payload_string = std::string(packet);

        // std::cerr << "payload-length" << payload_string.size() << std::endl;
        wlan.set_payload(packet, packetsize);
        wlan.set_srcmac(srcmac);
        wlan.set_dstmac(dstmac);

        wlan.SerializeToString(&serializedData);

        if (pDev == realWorldDevices::simulationInterface){
            // zmq_sndmore(*lanRadioPublisher, "SIMINT");
            // zmq_sndmore(*lanRadioPublisher, "DATA");
            lanradio_send(*simIntPublisher, serializedData, use_udp, simIntSocketUDP, &simIntAddr);
        }else if(pDev == realWorldDevices::dut){
            // zmq_sndmore(*lanRadioPublisher, "DUT");
            // zmq_sndmore(*lanRadioPublisher, "DATA");
            lanradio_send(*dutPublisher, serializedData, use_udp, dutSocketUDP, &dutAddr);
        }

        asn_DEF_SPAT_PDU.free_struct(&asn_DEF_SPAT_PDU, spat, 0);
    }
}

void LanradioCommunication::fillGeoNetBTPheaderForCam(CAM_t* cam, int payloadLen, std::string srcmac)
{
    // GeoNetwork Header
    mGeoBtpHdrForCam.mGeoNetHdr.basicHeader.versionAndNH = 1;
    mGeoBtpHdrForCam.mGeoNetHdr.basicHeader.reserved = 0;
    mGeoBtpHdrForCam.mGeoNetHdr.basicHeader.lifetime = 241;
    mGeoBtpHdrForCam.mGeoNetHdr.basicHeader.remainingHopLimit = 1;

    mGeoBtpHdrForCam.mGeoNetHdr.commonHeader.nhAndReserved = 32;
    mGeoBtpHdrForCam.mGeoNetHdr.commonHeader.htAndHst = 80;
    mGeoBtpHdrForCam.mGeoNetHdr.commonHeader.tc = 2;
    mGeoBtpHdrForCam.mGeoNetHdr.commonHeader.payload = htons(payloadLen + sizeof(struct BTPHeader));
    mGeoBtpHdrForCam.mGeoNetHdr.commonHeader.maxHop = 1;
    mGeoBtpHdrForCam.mGeoNetHdr.commonHeader.reserved = 0;
    mGeoBtpHdrForCam.mGeoNetHdr.tsb.spv.addr.assignmentTypeCountryCode = htons(38393);
    // mGeoBtpHdrForCam.mGeoNetHdr.tsb.spv.addr.llAddr = reinterpret_cast<uint8_t*>(ether_aton(mOwnMac.c_str()));
    memcpy(&mGeoBtpHdrForCam.mGeoNetHdr.tsb.spv.addr.llAddr, ether_aton(srcmac.c_str()), ETH_ALEN);
    mGeoBtpHdrForCam.mGeoNetHdr.tsb.spv.timestamp = htonl(2810450329);
    mGeoBtpHdrForCam.mGeoNetHdr.tsb.spv.latitude = htonl(cam->cam.camParameters.basicContainer.referencePosition.latitude); // htonl(424937722);
    mGeoBtpHdrForCam.mGeoNetHdr.tsb.spv.longitude = htonl(cam->cam.camParameters.basicContainer.referencePosition.longitude); // htonl(3460636913);
    mGeoBtpHdrForCam.mGeoNetHdr.tsb.spv.speed = htons(496);
    mGeoBtpHdrForCam.mGeoNetHdr.tsb.spv.heading = htons(1996);

    mGeoBtpHdrForCam.mGeoNetHdr.tsb.reserved = htonl(0);

    // BTP Header
    mGeoBtpHdrForCam.mBTPHdr.mDestinationPort = htons(2001);
    mGeoBtpHdrForCam.mBTPHdr.mSourcePort = htons(0);
    // uint8_t* temp = reinterpret_cast<uint8_t*>(&mGeoBtpHdrForCam);
    // dumpBuffer(temp, sizeof(mGeoBtpHdrForCam));
}

void LanradioCommunication::fillGeoNetBTPheaderForSpat(int payloadLen, std::string srcmac)
{
    // GeoNetwork Header
    mGeoBtpHdrForSpat.mGeoNetHdr.basicHeader.versionAndNH = 1;
    mGeoBtpHdrForSpat.mGeoNetHdr.basicHeader.reserved = 0;
    mGeoBtpHdrForSpat.mGeoNetHdr.basicHeader.lifetime = 241;
    mGeoBtpHdrForSpat.mGeoNetHdr.basicHeader.remainingHopLimit = 1;

    mGeoBtpHdrForSpat.mGeoNetHdr.commonHeader.nhAndReserved = 32;
    mGeoBtpHdrForSpat.mGeoNetHdr.commonHeader.htAndHst = 80;
    mGeoBtpHdrForSpat.mGeoNetHdr.commonHeader.tc = 2;
    mGeoBtpHdrForSpat.mGeoNetHdr.commonHeader.payload = htons(payloadLen + sizeof(struct BTPHeader));
    mGeoBtpHdrForSpat.mGeoNetHdr.commonHeader.maxHop = 1;
    mGeoBtpHdrForSpat.mGeoNetHdr.commonHeader.reserved = 0;
    mGeoBtpHdrForSpat.mGeoNetHdr.tsb.spv.addr.assignmentTypeCountryCode = htons(38393);
    // mGeoBtpHdrForSpat.mGeoNetHdr.tsb.spv.addr.llAddr = reinterpret_cast<uint8_t*>(ether_aton(mOwnMac.c_str()));
    memcpy(&mGeoBtpHdrForSpat.mGeoNetHdr.tsb.spv.addr.llAddr, ether_aton(srcmac.c_str()), ETH_ALEN);
    mGeoBtpHdrForSpat.mGeoNetHdr.tsb.spv.timestamp = htonl(2810450329);
    mGeoBtpHdrForSpat.mGeoNetHdr.tsb.spv.latitude = htonl(424937722);
    mGeoBtpHdrForSpat.mGeoNetHdr.tsb.spv.longitude = htonl(3460636913);
    mGeoBtpHdrForSpat.mGeoNetHdr.tsb.spv.speed = htons(496);
    mGeoBtpHdrForSpat.mGeoNetHdr.tsb.spv.heading = htons(1996);

    mGeoBtpHdrForSpat.mGeoNetHdr.tsb.reserved = htonl(0);

    // BTP Header
    mGeoBtpHdrForSpat.mBTPHdr.mDestinationPort = htons(2004);
    mGeoBtpHdrForSpat.mBTPHdr.mSourcePort = htons(0);
    // uint8_t* temp = reinterpret_cast<uint8_t*>(&mGeoBtpHdrForSpat);
    // dumpBuffer(temp, sizeof(mGeoBtpHdrForSpat));
}

bool LanradioCommunication::received_WLAN(wlan_msg m, realWorldDevices pDev, std::chrono::steady_clock::time_point rtp)
{
    if (!use_lanradio or mac_interface == nullptr) return false;

    // Parse Geonet Header

    //int mBytes = m.payload.size();
    //std::cout << "<<-----------------------new-packet----------------->>" << std::endl;
    int mBytes = m.payload.size();
    //std::cout << "LR_COM: geoNetPDU (including GeoNet and BTP) size: " << mBytes << std::endl;
    int mLinkLayerLength = 0; // sizeof(struct ether_header);
    // int mLinkLayerLength = sizeof(struct ether_header);
    const char* mPacket = m.payload.c_str();
    //const char* mPacket = m.payload.c_str();

    int geoNetPDULen = mBytes - mLinkLayerLength;

    // std::cerr << "mBytes = " << mBytes << ", mLinkLayerLength = " << mLinkLayerLength << ", geoNetPDULen=" << geoNetPDULen << std::endl;
    // for(int i = 0; i<50; i++) {
    //    printf(": %x \n", (unsigned char) *(mPacket+i));
    // }

    /* read geonet header */
    unsigned int geoHdrLen;
    uint8_t* geoHdr;

    // fillGeoNetBTPheaderForCam(strCam.size(), srcmac);
    geoHdrLen = sizeof(struct GeoNetworkAndBTPHeaderCAM);
    geoHdr = reinterpret_cast<uint8_t*>(&mGeoBtpHdrForCam);

    // unsigned char packet[geoNetPDULen];
    /* unsigned char* payload = mPacket + geoHdrLen; */
    const char* geoNetHdr = mPacket;

    // Fill in geo networking and btp header
    memcpy(geoHdr, geoNetHdr, geoHdrLen);

    auto myHeader = reinterpret_cast<GeoNetworkAndBTPHeaderCAM*>(geoHdr);
    // std::cerr << "got: " << myHeader->mBTPHdr.mDestinationPort << std::endl;

    // copy payload to packet
    // memcpy(payload,strCam.c_str(),strCam.size());
    /* end read geonet header */
    //std::cout << "LR_COM: received_WLAN called. DestinationPort: " << ntohs(myHeader->mBTPHdr.mDestinationPort) << std::endl;
    const char* geoNetPDU = mPacket + mLinkLayerLength;
    if (/*geoNetPDU[5] == 80 && geoNetPDU[40,41,42,43]*/ ntohs(myHeader->mBTPHdr.mDestinationPort) == 2001) {
        // CAM
        int camPDULen = geoNetPDULen - sizeof(struct GeoNetworkAndBTPHeaderCAM);
        //std::cout << "LR_COM: camPDULen: " << camPDULen << " bytes of packet from: " << pDev << std::endl;
        const char* camPDU = geoNetPDU + sizeof(struct GeoNetworkAndBTPHeaderCAM);
        std::string serializedAsnCam(camPDU, camPDULen);
        // ReceivedPacketInfo info;
        // info.mSenderMac = senderMac;
        // info.mType = dataPackage::DATA_Type_CAM;
        // info.was_outgoing_packet = (addr.sll_pkttype == PACKET_OUTGOING);

        CAM_t* cam = nullptr;
        int res = decodeMessage(&asn_DEF_CAM, (void**) &cam, serializedAsnCam);
        if (res != 0) {
            // mLogger->logError("Failed to decode received CAM. Error code: " + to_string(res));
            if (lara_debug) std::cerr << "Failed to decode received CAM. Error code: " << std::to_string(res) << std::endl;
            return false;
        }
        // asn_fprint(stdout, &asn_DEF_CAM, cam);

        //if (lara_debug) std::cerr << "success: stationId=" << cam->header.stationID << std::endl;

        CAMMessage* cammsg = new CAMMessage("CAM");
        if (cam->cam.camParameters.evaluationContainer->runNumber != runNumber) {
            std::cout << "LR_COM: received cam with wrong runNumber: " << cam->cam.camParameters.evaluationContainer->runNumber << ", Discard it." << std::endl;
            wrongRunNumberPackets.record(1.0f);
            return false;
        }

        cammsg->setTimestamp(simTime());
        cammsg->setRecipientAddress(-1);
        cammsg->setBitLength(0);
        cammsg->setPsid(-1);
        cammsg->setChannelNumber(static_cast<int>(Channel::cch));
        cammsg->addBitLength(2896);
        cammsg->setUserPriority(1);
        cammsg->setSrcmac(m.src_mac.c_str());

        cammsg->setCam(*cam);
        simtime_t simulationCreationTime;
        // take(cammsg);

        // std::cout << "LR_COM: received cam from: " << cammsg->getCam().header.stationID << " with sequence number: " << cam->cam.camParameters.evaluationContainer->evalSequenceNumber << std::endl;
        if (pDev == realWorldDevices::simulationInterface) {
            //std::cout << "LR_COM: received cam from simInt." << std::endl;
            /* logging */
            try{
                mutex_dutMessageMap.lock();
                // std::cout << "LR_COM: simInt received packet with Id: " << cam->cam.camParameters.evaluationContainer->evalSequenceNumber << ", dutMessageMap contains the following keys:" << std::endl;
                // for (auto it = dutMessageMap.begin(); it != dutMessageMap.end(); ++it) {
                //     std::cout << it->first << ", ";
                // }
                // std::cout << std::endl;
                std::chrono::steady_clock::time_point stp = dutMessageMap.at(cam->cam.camParameters.evaluationContainer->evalSequenceNumber).first;
                simulationCreationTime = dutMessageMap.at(cam->cam.camParameters.evaluationContainer->evalSequenceNumber).second;
                dutMessageMap.erase(cam->cam.camParameters.evaluationContainer->evalSequenceNumber);
                mutex_dutMessageMap.unlock();
                // std::cout << "LR_COM: simInt latency record at: " << simTime() << std::endl;
                dutToSimIntLat.record(std::chrono::duration<double, std::nano>(rtp - stp).count());
                // Std::cout << "LR_COM: simTime: "
                //           << simTime()
                //           << ", simInt received cam from: "
                //           << cammsg->getCam().header.stationID
                //           << " with sequence number: "
                //           << cam->cam.camParameters.evaluationContainer->evalSequenceNumber
                //           << ", latency in ns: "
                //           << std::chrono::duration<double, std::nano>(rtp - stp).count()
                //           << std::endl;
            } catch (const std::exception& e) {
                wrongEvalSequenceNumberPacketsDUT.record(1);
                mutex_dutMessageMap.unlock();
                //std::cout << "LR_COM: simInt received cam with unknown sequence numner: " << cam->cam.camParameters.evaluationContainer->evalSequenceNumber << std::endl;
                //deactivate_radio();
                //throw;
                return false;
            }
            if (simTime() >= getSimulation()->getWarmupPeriod()) receivedBytesFellowVehicles += mBytes;
            if (simTime() >= getSimulation()->getWarmupPeriod()) receivedPacketsFellowVehicles++;
            packetsPerSecSimInt++;
            packetsPerSecOverall++;
            cammsg->setTimestamp(simulationCreationTime);
            /* ------- */
            // std::cout << "LR_COM: call sendMacPacketDirect at simTime: " << simTime() << std::endl;
            mac_interface->sendMacPacketDirect(cammsg); // mac_interface integrate message received by the simulation interface
        }else if (pDev == realWorldDevices::dut) {
            //std::cout << "LR_COM: received cam from dut." << std::endl;
            /* logging */
            try {
                mutex_simIntMessageMap.lock();
                // std::cout << "LR_COM: dut received msg with id: " << cam->cam.camParameters.evaluationContainer->evalSequenceNumber << ", simIntMessageMap size: " << simIntMessageMap.size() << std::endl;
                // std::cout << "LR_COM: dut received packet with Id: " << cam->cam.camParameters.evaluationContainer->evalSequenceNumber << ", simIntMessageMap contains the following keys:" << std::endl;
                // for (auto it = simIntMessageMap.begin(); it != simIntMessageMap.end(); ++it) {
                //     std::cout << it->first << ", ";
                // }
                // std::cout << std::endl;
                std::chrono::steady_clock::time_point stp = simIntMessageMap.at(cam->cam.camParameters.evaluationContainer->evalSequenceNumber).first;
                simulationCreationTime = simIntMessageMap.at(cam->cam.camParameters.evaluationContainer->evalSequenceNumber).second;
                simIntMessageMap.erase(cam->cam.camParameters.evaluationContainer->evalSequenceNumber);
                // std::cout << "LR_COM: simIntMessageMap deleted msg with id: " << cam->cam.camParameters.evaluationContainer->evalSequenceNumber << std::endl;
                mutex_simIntMessageMap.unlock();
                // std::cout << "LR_COM: dut latency record at: " << simTime() << std::endl;
                simIntToDutLat.record(std::chrono::duration<double, std::nano>(rtp - stp).count());
                // std::cout << "LR_COM: simTime: "
                //           << simTime()
                //           << ", dut received cam from: "
                //           << cammsg->getCam().header.stationID
                //           << " with sequence number: "
                //           << cam->cam.camParameters.evaluationContainer->evalSequenceNumber
                //           << ", latency in ns: "
                //           << std::chrono::duration<double, std::nano>(rtp - stp).count()
                //           << std::endl;
            } catch (const std::exception& e) {
                //std::cout << "LR_COM: dut received cam with unknown sequence numner: " << cam->cam.camParameters.evaluationContainer->evalSequenceNumber << std::endl;
                wrongEvalSequenceNumberPacketsSimInt.record(1);
                mutex_simIntMessageMap.unlock();
                //deactivate_radio();
                //throw;
                return false;
            }
            if (simTime() >= getSimulation()->getWarmupPeriod()) receivedBytesDut += mBytes;
            if (simTime() >= getSimulation()->getWarmupPeriod()) receivedPacketsDut++;
            packetsPerSecDut++;
            packetsPerSecOverall++;
            /* ------- */
            // std::cout << "LR_COM: dut received cam forwarding it to its app layer at simTime: " << simTime() << std::endl;
            cammsg->setTimestamp(simulationCreationTime);
            mac_interface->toApplicationLayer(cammsg);
        }
    }
    else if (/*geoNetPDU[5] == 66*/ ntohs(myHeader->mBTPHdr.mDestinationPort) == 2002) {
        // DENM
        error("we currently do not support DENMs");
    }
    else if (myHeader->mBTPHdr.mDestinationPort == htons(2004)) {
        if (lara_debug) std::cerr << "received a SPAT, no further processing" << std::endl;
    }
    else {
        // maybe tesla
        // TODO: Possible cases?
        // error("we currently do not support anything else than CAMs and SPATs");
    }
    // end parse Geonet Header
    return true;
}

bool LanradioCommunication::registerRadio(LanradioMacInterface* mac)
{
    if (!use_lanradio) return true;

    if (mac_interface != nullptr) {
        mac_interface->setAsLanradioMac(false);
        // this will have overridden mac_interface to nullptr, but that's okay
    }
    mac_interface = mac;

    return true;
}
