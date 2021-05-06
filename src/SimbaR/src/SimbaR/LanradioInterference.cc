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

#include "SimbaR/LanradioInterference.h"
#include <unistd.h>
#include <openssl/rand.h>
#include <stdlib.h>
#include "SimbaR/pstream.h"
#include <sstream>

#define MYDEBUG EV
#define UDP_BUFFER_SIZE 2048

using veins::LanradioInterference;

Define_Module(veins::LanradioInterference);

// OPENC2X ethertype for GeoNetworking
static const uint16_t ETHERTYPE_CAR = 0x8947;

//  send multipart zmq message
static bool zmq_sndmore(zmq::socket_t& sock, const std::string& str)
{
    zmq::message_t msg(str.size());
    memcpy(msg.data(), str.data(), str.size());

    bool ret = sock.send(msg, ZMQ_SNDMORE);
    return ret;
}

//  send zmq string
static bool interference_send(const std::string& str, int s, struct sockaddr_in* cliAddr)
{
    int r;
    r = sendto(s, (void*)str.c_str(), str.length(), 0, (const struct sockaddr*)cliAddr, sizeof(sockaddr));
    if (r > 0) {
        return true;
    }else{
        return false;
    }
}

static bool interferenceConfig_send(zmq::socket_t& sock, const std::string& str)
{
    zmq::message_t msg(str.size());
    memcpy(msg.data(), str.data(), str.size());

    zmq::message_t reply;

    std::cout << "Send command: " << str << ", to interference." << std::endl;
    bool ret = sock.send(msg);

    std::cout << "wait for interference radio config acks." << std::endl;
    sock.recv(&reply);
    std::cout << "received Ack from interference." << std::endl;

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

static std::string radioConfig_recv(zmq::socket_t& sock)
{
    zmq::message_t msg;
    sock.recv(&msg);
    return std::string(static_cast<char*>(msg.data()), msg.size());
}


void LanradioInterference::handleMessage(cMessage* msg)
{
    take(msg);
    if (msg->isSelfMessage()) {
        switch (msg->getKind()) {
            case Lanradio_Interference_LogInterval: {
                numInterferingFramesPerSec.record(interferingFramesPerSec);
                interferingFramesPerSec = 0;
                numGeneratedPhysicalInterferencePerSec.record(generatedPhysicalInterferencePerSec);
                generatedPhysicalInterferencePerSec = 0;
                scheduleAt(simTime() + 1, logInterval);
                break;
            }
            default: {
                if (msg != nullptr) {
                    std::cerr << "APP: Error: Got Self Message of unknown kind! Name: " << msg->getName() << endl;
                    delete msg;
                }
                break;
            }
        }
    }
    else{
        std::cout << "LanradioInterference: received unknown msg." << std::endl;
    }
}

void LanradioInterference::initialize(int stage)
{
    sleep(5);   // to avoid zmq::error_t: Address already in use when a new run is started.
    /*
     * setup zmq socket for configuring Lanradio
     */
    if (stage == 0) {
        use_interference = par("use_interference").boolValue();
        baseport = par("baseport").intValue();
        interferenceHostname = par("interferenceHostname").stringValue();
        std::cout << "interferenceHostname: " << interferenceHostname << std::endl;
        bitrates = par("bitrates").stringValue();
        wlan_ip = par("wlan_ip").stringValue();
        freq = par("freq").stringValue();
        bandwidth = par("bandwidth").stringValue();
        experimentName = par("experimentName").stringValue();
        interferenceMinInterval = par("interferenceMinInterval");
        useInterferenceMinInterval = par("useInterferenceMinInterval").boolValue();
        SimInterferenceTxPower = par("SimInterferenceTxPower").stringValue();
        SimInterferenceModifyMac = par("SimInterferenceModifyMac").boolValue();

        lastInterferenceGenerated = 0;

        /* logging */
        interferingFramesPerSec = 0;
        generatedPhysicalInterferencePerSec = 0;
        lastInterferenceArrivalTime = 0;
        numInterferingFramesPerSec.setName("numInterferingFramesPerSec");
        numGeneratedPhysicalInterferencePerSec.setName("numGeneratedPhysicalInterferencePerSec");
        interArrivalTimeInterferingFrames.setName("interArrivalTimeInterferingFrames");

        logInterval = new cMessage("logInterval", Lanradio_Interference_LogInterval);
        scheduleAt(simTime() + 1, logInterval);
        /* ------- */

        runNumber = getEnvir()->getConfigEx()->getActiveRunNumber();

        mContext = new zmq::context_t(1);

        interferenceRadioConfigSocket = new zmq::socket_t(*mContext, ZMQ_REQ);
        interferenceRadioConfigSocket->setsockopt(ZMQ_RCVTIMEO, -1);
        interferenceRadioConfigSocket->setsockopt(ZMQ_LINGER, 0);
        interferenceRadioConfigSocket->setsockopt(ZMQ_SNDTIMEO, -1);
        interferenceRadioConfigSocket->bind(("tcp://*:" + std::to_string(baseport)).c_str());

        /* create udp sockets */
        int rcInterference;
        struct sockaddr_in myAddrInterference;
        memset(&myAddrInterference, 0, sizeof(myAddrInterference));

        // Simulation Interface
        interferenceSocketUDP = socket (AF_INET, SOCK_DGRAM, 0);
        if (interferenceSocketUDP < 0) {
            std::cout << "LR_COM: error creating interference udp socket: " << interferenceSocketUDP << std::endl;;
            exit (EXIT_FAILURE);
        }

        myAddrInterference.sin_family = AF_INET;
        myAddrInterference.sin_addr.s_addr = htonl (INADDR_ANY);
        myAddrInterference.sin_port = htons (baseport + 1);

        const int optval = 1;
        setsockopt(interferenceSocketUDP, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(int));
        rcInterference = bind (interferenceSocketUDP, (struct sockaddr *) &myAddrInterference, sizeof(myAddrInterference));
        if (rcInterference < 0) {
            std::cout << "LR_COM: interference udp socket could not bind to port: " << baseport + 1 << std::endl;
            exit (EXIT_FAILURE);
        }
        std::cout << "LR_COM: interference udp socket waiting for data on port: " << baseport + 1 << std::endl;

        // client addresses
        // interference
        struct hostent* hostinfo;
        hostinfo = gethostbyname(interferenceHostname.c_str());
        if (hostinfo == NULL) {
            std::cout << "LR_COM: Error erroneous interference client hostname: " << interferenceHostname << std::endl;
        }
        interferenceAddr.sin_family = AF_INET;
        interferenceAddr.sin_addr = *(struct in_addr *) hostinfo->h_addr;
        interferenceAddr.sin_port = htons (baseport + 1);

        activate_lanradio();
        std::cout << "LR_COM: Configured radios, starting simulation." << std::endl;
    }
}

void LanradioInterference::activate_lanradio()
{
    // Start SimInterference
    std::stringstream cmdInterference;
    cmdInterference << std::boolalpha << "ssh root@10.0.197.106 lanradio --transmitPower " << SimInterferenceTxPower << " --modifyMac " << SimInterferenceModifyMac << " --useUDP true --InterferenceMode true --VeinsHostname 10.0.197.200 --baseport 22222 &";
    std::string startInterference = cmdInterference.str();
    std::cout << "Start interference cmd: " <<  startInterference << std::endl;
    redi::ipstream startInterferenceProc(startInterference, redi::pstreams::pstdout | redi::pstreams::pstderr);
    startInterferenceProc.close();
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
    // start chanload application
    // chanload_send_command("chanload wlan1 " + freq + " 100ms 0 > /tmp/chanload_" + std::to_string(runNumber) + ".txt &");
    // start pending frames script
    // radio_send_command("bash /root/pending_frames.sh > /tmp/pending_frames_" + std::to_string(runNumber) + ".txt &");
    // setCarrierSensingOff(true, *interferenceRadioConfigSocket);
}

void LanradioInterference::deactivate_radio()
{
//     // shutdown lanradio interface
//     radio_send_command("iw dev " + wlan_interface + " ocb leave");
//     radio_send_command("ip link set " + wlan_interface + " down");
//     radio_send_command("ifconfig " + wlan_interface + " down");
//     // shutdown chanload interface
//     radio_send_command("iw dev wlan0 ocb leave");
//     radio_send_command("ip link set wlan0 down");
//     radio_send_command("ifconfig wlan0 down");
    // radio_send_command("killall lanradio && if [ \"$(cat /sys/class/net/eth0/address)\" == \"00:0d:b9:2e:c3:dc\" ]; then lanradio --useUDP true --dualRadio true --VeinsHostname 10.0.197.200 >> /tmp/log.txt & elif [ \"$(cat /sys/class/net/eth0/address)\" == \"00:0d:b9:2e:c3:a4\" ]; then lanradio --useUDP true --prototype true --VeinsHostname 10.0.197.200 >> /tmp/log.txt & fi");
    //
    interference_send_command("killall lanradio");
    std::cout << "LR_COM: deactivate interference successful" << std::endl;
}

bool LanradioInterference::interference_send_command(const std::string cmd)
{
    try {
        bool successful;
        successful = interferenceConfig_send(*interferenceRadioConfigSocket, cmd);
        if (!successful) {
            error("connection to LanradioInterference was unsuccessful");
        }
        std::cout << "LR_COM: radioconfig sent: " << cmd << std::endl;
        return successful;
    }
    catch (zmq::error_t) {
        error("connection to Lanradio was unsuccessful");
    }
}

void LanradioInterference::finish()
{
    /* logging */

    /* shut everything down */
    deactivate_radio();

    interferenceRadioConfigSocket->close();
    delete interferenceRadioConfigSocket;

    zmq_ctx_term(mContext);
    delete mContext;

    std::cout << "LR_COM: LanradioInterference finish()!" << std::endl;
}

/*
 * transmit an interference packet with packetByteLength bytes and transmit power of Power_Milliwatt
 */
void LanradioInterference::sendInterference(std::string srcmac, std::string dstmac, int packetByteLength, double Power_Milliwatt, simtime_t arrivalTime)
{
    if (use_interference) {
        interferingFramesPerSec++;
        interArrivalTimeInterferingFrames.record(simTime() - lastInterferenceArrivalTime);
        lastInterferenceArrivalTime = simTime();
        if (!useInterferenceMinInterval || (useInterferenceMinInterval && (simTime() - lastInterferenceGenerated > interferenceMinInterval))) {
            unsigned char interference[packetByteLength];
            // std::cout << "LanradioInterference: packetByteLength: " << packetByteLength << std::endl;
            // RAND_bytes(interference, packetByteLength);
            std::string serializedData;
            wlanPackage::WLAN wlan;

            wlan.set_priority(wlanPackage::WLAN_Priority_BE);
            wlan.set_ethertype(ETHERTYPE_CAR);
            wlan.set_payload(interference, packetByteLength);
            // std::cout << "LanradioInterference: transmit interference of size: " << wlan.payload().size() << std::endl;
            wlan.set_srcmac(srcmac);
            wlan.set_dstmac(dstmac);
            wlan.set_txpower(Power_Milliwatt);
            wlan.SerializeToString(&serializedData);

            /* sending */
            bool r = interference_send(serializedData, interferenceSocketUDP, &interferenceAddr);
            //std::cout << "transmitted interference" << std::endl;
            lastInterferenceGenerated = simTime();
            generatedPhysicalInterferencePerSec++;
            /* logging */
        }
    }else{
        // std::cout << "LanradioInterference: no double interference." << std::endl;
    }
}
