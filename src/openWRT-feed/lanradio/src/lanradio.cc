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

#include <stdio.h>
#include <stdlib.h>
#include "zmq/zmq.hpp"
#include <string>
#include <iostream>
#include <pthread.h>
#include "pstreams/pstream.h"
#include "wlan.pb.h"
// #include <unistd.h>
#include <sys/socket.h>
// #include <unistd.h>
// #include <sys/types.h>
// #include <arpa/inet.h>
// #include <netinet/in.h>
#include <netdb.h>
#include <string.h>

#include "SendToHardwareViaMAC.cc"
#include "ReceiveFromHardwareViaMAC.cc"
#include "SendToTins.cpp"
#include <boost/program_options.hpp>
//#include <boost/process.hpp>

// #include<stdatomic.h>

#define DEBUG 0
#define UDP_BUFFER_SIZE 2048


namespace po = boost::program_options;
static volatile bool running = true;

static void termination_handler(int sig, siginfo_t *si, void *unused)
{
    std::cout << "called termination handler." << std::endl;
    running = false;
    if (errno == EINTR) {
        std::cout << "termination_handler: errno already set: running: " << running << std::endl;
    }
    std::cout << "termination_handler, running: " << running << std::endl;
}

//  recv zmq string
static std::string lanradio_recv(zmq::socket_t& sock, bool use_udp, int s) {
    if (!use_udp) {
        zmq::message_t msg;
        //zmq_recv(sock, &msg, 0);
        //zmq::socket_t* s = (zmq::socket_t*) sock;
        sock.recv(&msg);

        return std::string(static_cast<char *>(msg.data()), msg.size());
    }else{
        std::string data;
        int n;
        data.resize(UDP_BUFFER_SIZE);
        /* wait for message and process it */
        // std::cout << "wait for packet via udp." << std::endl;
        n = recv(s, (void*)data.c_str(), UDP_BUFFER_SIZE, 0);
        if (n <= 0) {
            std::cout << "LR_COM: socket could not receive data ..." << std::endl;;
            return "SHUTDOWN";
        }
        data.resize(n);
        return data;
    }
}

//  send zmq string
static bool lanradio_send(zmq::socket_t& sock, const std::string &str, bool use_udp, int s, struct sockaddr_in* hostAddr) {
    if (!use_udp) {
        zmq::message_t msg(str.size());
        memcpy(msg.data(), str.data(), str.size());

        // bool ret = sock.send(msg);
        // zmq::socket_t* s = (zmq::socket_t*) sock;
        bool ret = sock.send(&msg, 0);
        return ret;
    }else{
        int r;
        if (errno == EINTR || running == false) {
            std::cout << "lanradio_send: EINTR or running == false before send" << std::endl;
            return false;
        }
        r = sendto(s, (void*)str.c_str(), str.length(), 0, (const struct sockaddr*)hostAddr, sizeof(sockaddr));
        // std::cout << "send packet via udp." << std::endl;
        if (errno == EINTR || running == false) {
            std::cout << "lanradio_send: EINTR or running == false after send" << std::endl;
            return false;
        }
        if (r > 0) {
            return true;
        }else{
            return false;
        }
    }
}

//  send zmq string, sendmore
static bool myzmq_sndmore(zmq::socket_t &sock, const std::string &str) {

    zmq::message_t msg(str.size());
    memcpy(msg.data(), str.data(), str.size());

    bool ret = sock.send(msg, ZMQ_SNDMORE);
    return ret;
}

static bool radioConfig_send(zmq::socket_t& sock, const std::string& str)
{
    zmq::message_t msg(str.size());
    memcpy(msg.data(), str.data(), str.size());

    //zmq::socket_t* s = (zmq::socket_t*) sock;
    bool ret = sock.send(msg, 0);
    //bool ret = zmq_send(sock, &msg, 0);
    return ret;
}

static std::string radioConfig_recv(zmq::socket_t& sock)
{
    zmq::message_t msg;
    // zmq::socket_t* s = (zmq::socket_t*) sock;
    // TODO: test for error: not a socket
    // std::cout << "radioConfig recv: before receive call." << std::endl;
    sock.recv(&msg);
    // zmq_recv(sock, &msg, 0);
    return std::string(static_cast<char*>(msg.data()), msg.size());
}

struct radioConfigThreadData {
    zmq::context_t* context;
    std::string host;
    int port;
    bool use_udp;
    bool interferenceMode;
    int sock;
    struct sockaddr_in* hostAddr;
    pthread_t* thread_tx_wlan;
    pthread_t* thread_rx_wlan;
};

struct txWlanThreadData {
    SendToHardwareViaMAC* mac;
    zmq::context_t* context;
    std::string host;
    int port;
    bool use_udp;
    int sock;
    bool useDualRadio;
    std::string dualRadioAth9kMac;
    std::string interferenceMac;
    bool interferenceMode;
	bool usePacketInjection;
	SendToTins* pi_tx;
};

struct rxWlanThreadData {
    ReceiveFromHardwareViaMAC* mac_rx;
    SendToHardwareViaMAC* mac_tx;
    bool receive_own_msgs;
    bool receive_every_ether_type;
    zmq::context_t* context;
    std::string host;
    int port;
    bool use_udp;
    int sock;
    struct sockaddr_in* hostAddr;
    bool useDualRadio;
    std::string dualRadioAth9kMac;
    std::string interferenceMac;
	bool usePacketInjection;
	SendToTins* pi_tx;
};

void *configure_radio_thread(void* threadarg) {
    struct radioConfigThreadData *data;
    data = (struct radioConfigThreadData*) threadarg;

    std::string port = std::to_string(data->port);

    zmq::socket_t sub(*(data->context), zmq::socket_type::sub);
    // sub.setsockopt(ZMQ_RCVTIMEO, -1);
    // sub.setsockopt(ZMQ_LINGER, 0);
    // sub.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    // sub.connect(("tcp://" + data->host + ":" + subPort).c_str());

    zmq::socket_t sock(*(data->context), zmq::socket_type::rep);
    sock.setsockopt(ZMQ_SNDTIMEO, -1);
    sock.setsockopt(ZMQ_LINGER, 0);
    sock.setsockopt(ZMQ_RCVTIMEO, -1);
    sock.connect(("tcp://" + data->host + ":" + port).c_str());

    std::cout << "radio config port: " << port << std::endl;

    // Sync with publisher
    // std::string msg = "";
    // std::cout << "radioConfig: wait for SYNC" << std::endl;
    // while (msg != "SYNC") {
    //     msg = radioConfig_recv(sub);
    // }
    // radioConfig_send(sync, "ACK");
    // std::cout << "radioConfig: send ACK" << std::endl;
    // msg = radioConfig_recv(sync);
    // while (msg != "START") {
    //     while(msg == "") {
    //         msg = radioConfig_recv(sync);
    //     }
    //     if (msg == "START") {
    //         break;
    //     }else{
    //         radioConfig_send(sync, "ACK");
    //     }
    // }

    std::string req;
    int r;
    while (true) {
        req = radioConfig_recv(sock);
        radioConfig_send(sock, "ACK");
        // if (req == "SYNC") {
        //     continue;
        // }
        // std::cout << "radioConfig: received req: " << req << std::endl;
        if (req.find("chanload wlan1 ") == 0) {
            // start chanload application
            std::cout << "start chanload with command: " << req << std::endl;
            system(req.c_str());
        }else if (req.find("killall lanradio") == 0) {
            std::cout << "radioConfig: received restart command." << std::endl;
            //system(("setsid " + req).c_str());
            if (data->use_udp) {
                lanradio_send(sub, "SHUTDOWN", data->use_udp, data->sock, data->hostAddr);
            }
            std::cout << "radioConfig: kill rx/tx wlan threads" << std::endl;
            int retTxWlanThread = pthread_kill(*(data->thread_tx_wlan), SIGUSR1);
            if (!data->interferenceMode) {
                int retRxWlanThread = pthread_kill(*(data->thread_rx_wlan), SIGUSR1);
                std::cout << "radioConfig: leaving own thread: retTxWlanThread: " << retTxWlanThread << ", retRxWlanThread: " << retRxWlanThread << std::endl;
            }else{
                std::cout << "radioConfig: leaving own thread: retTxWlanThread: " << retTxWlanThread << std::endl;
            }

            break;
        }else{
            redi::ipstream proc(req, redi::pstreams::pstdout | redi::pstreams::pstderr);
            proc.close();
        }
    }
    sub.close();
    // zmq_close(sub);
    // delete sub;

    sock.close();
    std::cout << "radioConfig closed all zeromq sockets." << std::endl;
    string* cmd = new string(req);
    return (void*) cmd;
    // zmq_close(sync);
    // delete sync;

    // zmq_ctx_term(context);
    // delete context;
}

void *tx_wlan_thread(void* threadarg) {
    struct sigaction sa;
    sa.sa_flags = SA_SIGINFO;
    sigemptyset(&sa.sa_mask);
    sa.sa_sigaction = termination_handler;
    sigaction(SIGUSR1, &sa, NULL);

    struct txWlanThreadData *data;
    data = (struct txWlanThreadData*) threadarg;

    std::string subPort = std::to_string(data->port);
    std::string syncPort = std::to_string(data->port + 1);

    zmq::socket_t sub(*(data->context), zmq::socket_type::sub);
    sub.setsockopt(ZMQ_RCVTIMEO, -1);
    sub.setsockopt(ZMQ_LINGER, 0);
    sub.setsockopt(ZMQ_SUBSCRIBE, "", 0);

    int rcSub, rcSync = 0;
    int timeout = -1;
    int linger = 0;

    // void* sync = zmq_socket(context, ZMQ_REQ);
    zmq::socket_t sync(*(data->context), zmq::socket_type::req);
    sync.setsockopt(ZMQ_SNDTIMEO, -1);
    sync.setsockopt(ZMQ_LINGER, 0);
    sync.setsockopt(ZMQ_RCVTIMEO, -1);

    if (!data->use_udp) std::cout << "tx_wlan sub port: " << subPort << ", sync port: " << syncPort << std::endl;

    // Sync with publisher
    std::string msg;
    if (!data->use_udp) {
        sub.connect(("tcp://" + data->host + ":" + subPort).c_str());
        sync.connect(("tcp://" + data->host + ":" + syncPort).c_str());
        msg = lanradio_recv(sub, data->use_udp, data->sock);
        if (msg == "SYNC") {
            lanradio_send(sync, "ACK", data->use_udp, data->sock, nullptr);
        }
        lanradio_recv(sync, data->use_udp, data->sock);
    }

    std::string shutdown = "SHUTDOWN";
    while(running) {
        std::string message;
        try{
            message = lanradio_recv(sub, data->use_udp, data->sock);
            if (strcmp(message.c_str(), shutdown.c_str()) == 0){
                std::cout << "tx_wlan shutdown" << std::endl;
                break;
            }
        } catch (...) {
            std::cout << "tx_wlan catched error" << errno << std::endl;
            break;
        }

        // std::cout << "tx_wlan_thread received message from veins with msg-length: " << message.length() << std::endl;;

        wlanPackage::WLAN w;
        bool succesful = w.ParseFromString(message);

        if(succesful) {
            string payload = w.payload();
            string srcmac;
            if (!data->useDualRadio) {
                if (data->interferenceMode) {
                    srcmac = data->interferenceMac;  // SimInterference transmits always the same mac to be filtered when receiving
                    //std::cout << "txWLAN: interferenceMac: " << data->interferenceMac << std::endl;
                    w.set_srcmac(data->interferenceMac);
                }else{
                    srcmac = w.srcmac();
                }
            }else{
                if (data->interferenceMode) {
                    srcmac = data->interferenceMac;  // SimInterference transmits always the same mac to be filtered when receiving
                    //std::cout << "txWLAN: interferenceMac: " << data->interferenceMac << std::endl;
                    w.set_srcmac(data->interferenceMac);
                }else{
                    srcmac = data->dualRadioAth9kMac;  // simInt/ath9k transmits always the same mac to be filtered when receiving
                    w.set_srcmac(data->dualRadioAth9kMac);
                }
            }
            string dstmac = w.dstmac();
            uint16_t ether_type = (uint16_t) w.ethertype();

			if (data->usePacketInjection) {
                //std::cout << "txWLAN: use packetInjection" << std::endl;
				if (data->interferenceMode) {
					//std::cout << "interference send packet of size:" << payload.size() << std::endl;
					data->pi_tx->sendInterference(w, PRIORITY_BE);
				}else{
					data->pi_tx->sendWithGeoNet(w, PRIORITY_BE);
				}
			}else{
				if (data->interferenceMode) {
					//std::cout << "interference send packet of size:" << payload.size() << std::endl;
					data->mac->send(&payload, PRIORITY_BE);
				}else{
					data->mac->sendWLAN(&payload, PRIORITY_BE, &srcmac, &dstmac, &ether_type);
				}
			}
        }else{
            if (DEBUG) std::cout << "tx_wlan_thread received non wlan message from veins" << std::endl;
        }
    }
    std::cout << "tx_wlan: after while. " << std::endl;
    sub.close();
    sync.close();
    std::cout << "tx_wlan: closed all zeromq sockets. " << std::endl;
    pthread_exit(NULL);
    // zmq_close(sub);
    // zmq_close(sync);
}



void *rx_wlan_thread(void* threadarg) {
    struct sigaction sa;
    sa.sa_flags = SA_SIGINFO;
    sigemptyset(&sa.sa_mask);
    sa.sa_sigaction = termination_handler;
    sigaction(SIGUSR1, &sa, NULL);

    struct rxWlanThreadData *data;
    data = (struct rxWlanThreadData*) threadarg;
    std::string pubPort = std::to_string(data->port);
    std::string syncPort = std::to_string(data->port + 1);

    zmq::socket_t pub(*(data->context), zmq::socket_type::pub);
    pub.setsockopt(ZMQ_SNDTIMEO, -1);
    pub.setsockopt(ZMQ_LINGER, 0);
    int rcPub, rcSync = 0;

    zmq::socket_t sync(*(data->context), zmq::socket_type::rep);
    sync.setsockopt(ZMQ_SNDTIMEO, -1);
    sync.setsockopt(ZMQ_LINGER, 0);
    sync.setsockopt(ZMQ_RCVTIMEO, 500);

    if (!data->use_udp) std::cout << "rx_wlan pub port: " << pubPort << ", sync port: " << syncPort << std::endl;

    // Sync with subscriber
    std::string msg;
    if (!data->use_udp) {
        pub.bind(("tcp://*:" + pubPort).c_str());  // zmq_pub has to bind such that the subscriber does not get old messages.
        sync.connect(("tcp://" + data->host + ":" + syncPort).c_str());
        while (true) {
            lanradio_send(pub, "SYNC", data->use_udp, data->sock, nullptr);
            msg = lanradio_recv(sync, data->use_udp, data->sock);
            if (msg == "ACK") {
                std::cout << "rx_wlan synchronized with subscriber." << std::endl;
                lanradio_send(sync, "START", data->use_udp, data->sock, nullptr);
                break;
            }
            std::cout << "No Ack received." << std::endl;
        }
    }

    pair<ReceivedPacketInfo, string> receivedData;      //MAC Sender, serialized DATA
    ReceivedPacketInfo* pktInfo = &receivedData.first;
    string* serializedData = &receivedData.second;
    bool success;
    string shutdown = "SHUTDOWN";
    string repeat = "REPEAT";
    while (running) {
        try{
            if (errno == EINTR || running == false) {
                std::cout << "EINTR or running == false; call break before recv." << std::endl;
                break;
            }
            do {
                receivedData = data->mac_rx->receiveWLAN(data->receive_own_msgs, data->receive_every_ether_type, running, data->useDualRadio, data->dualRadioAth9kMac, data->interferenceMac);  //receive serialized DATA
            } while (strcmp(serializedData->c_str(), repeat.c_str()) == 0 && running);
            if (strcmp(serializedData->c_str(), shutdown.c_str()) == 0) {
                std::cout << "rx_wlan received shutdown signal after receiveWLAN" << std::endl;
                break;
            }

            //check whether the mac of the sender and our own mac are the same and discard the package if we want to ignore those packages
            // if(!should_receive_own_msgs && /*(pktInfo->mSenderMac.compare(mac_tx->mOwnMac) == 0)*/ pktInfo->was_outgoing_packet) {
            if (!data->receive_own_msgs && ((data->usePacketInjection && pktInfo->mSenderMac.compare(data->pi_tx->mOwnMac) == 0) || pktInfo->mSenderMac.compare(data->mac_tx->mOwnMac) == 0)) {
                //std::cout << "received own Message, discarding" << std::endl;
                continue;
            }

            wlanPackage::WLAN wlan;
            // if (data->usePacketInjection) {
            //     wlan.set_payload(*(serializedData + 5));
            // }else{
            //     wlan.set_payload(*serializedData);
            // }
            wlan.set_payload(*serializedData);
            //std::cout << "Received Wlan packet of size: " << wlan.payload().size() << std::endl;
            //std::cout << "rx_wlan_thread senderMac: " << pktInfo->mSenderMac << std::endl;
            wlan.set_srcmac(pktInfo->mSenderMac);
            //std::cout << "rx_wlan_thread receiverMac: " << pktInfo->mReceiverMac << std::endl;
            wlan.set_dstmac(pktInfo->mReceiverMac);
            wlan.set_priority(wlanPackage::WLAN_Priority_BE);
            //std::cout << "rx_wlan_thread ether_type: " << pktInfo->ether_type << std::endl;
            if (pktInfo->ether_type == 0x8947) {
                wlan.set_ethertype((uint32_t) pktInfo->ether_type);
            }else{
                continue;
            }
            wlan.set_isloopback(pktInfo->was_outgoing_packet);

            if (DEBUG) std::cerr << "payload-length (rx) = " << wlan.payload().size() << std::endl;

            std::string ser;
            wlan.SerializeToString(&ser);

            // myzmq_sndmore(*mPublisher, "WLAN");
            // std::cout << "send envelope: WLAN to simulation" << std::endl;

            if (errno == EINTR || running == false) {
                std::cout << "EINTR or running == false; call break before send." << std::endl;
                break;
            }
            success = lanradio_send(pub, ser, data->use_udp, data->sock, data->hostAddr);
            if (errno == EINTR || running == false) {
                std::cout << "EINTR or running == false; call break after send." << std::endl;
                break;
            }
            if (!success) {
                break;
            }
        } catch (std::exception &e) {
            std::cout << "in rx_wlan catch statement." << std::endl;
            std::cout << "rx_wlan_thread: catch statement: " << e.what() << std::endl;
            if (errno == EINTR || running == false) {
                std::cout << "rx_wlan_thread: catch statement: EINTR or running == false; call break after send." << std::endl;
            }
            break;
        }
    }
    std::cout << "rx_wlan after while" << std::endl;
    pub.close();
    sync.close();
    std::cout << "rx wlan closed all zeromq sockets." << std::endl;
    pthread_exit(NULL);
    // zmq_close(pub);
    // zmq_close(sync);
}

int main(int ac, char* av[]) {

    std::string interface;
    std::string veinsHostname;
    bool echo;
    int baseport;
    bool isPrototype;
    bool use_udp;
    bool measure_chanload;
    bool dualRadio;
    std::string dualRadioAth9kMac;
    std::string interferenceMac;
    bool interferenceMode;
	bool usePacketInjection;
    int transmitPower;
    bool modifyMac;

    try {

        po::options_description desc("Allowed options");
        desc.add_options()
                                ("help, h", "show options")
                                ("interface, i", po::value<std::string>(&interface)->default_value("wlan1"), "wlan interface")
                                ("echo, e", po::value<bool>(&echo)->default_value(false), "send copy of all transmitted packets back to sender, similar to monitor mode (without radiotap information)")
                                ("baseport, b", po::value<int>(&baseport)->default_value(19191), "radioconfig = baseport")
                                ("prototype, p", po::value<bool>(&isPrototype)->default_value(false), "specify whether this alix box is the prototype or the interface of the simulation")
                                ("VeinsHostname, h", po::value<std::string>(&veinsHostname)->default_value("10.0.197.200"), "veins hostname rsp. ip address")
                                ("useUDP, u", po::value<bool>(&use_udp)->default_value(true), "Use UDP sockets instead of zmq's TCP-based pub sub pattern.")
                                ("useChanload, c", po::value<bool>(&measure_chanload)->default_value(false), "Just measure the physical channel's busy time.")
                                ("dualRadio, d", po::value<bool>(&dualRadio)->default_value(false), "Use ath5k radio for receiving and ath9k radio for transmitting.")
                                ("dualRadioAth9kMac, m", po::value<std::string>(&dualRadioAth9kMac)->default_value("00:60:2F:AA:4C:E2"), "The mac address to be transmitted by ath9k radio, used to filter out own packets at ath5k radio.")
                                ("interferenceMac, t", po::value<std::string>(&interferenceMac)->default_value("00:60:2F:AA:4C:E1"), "The mac address to be transmitted for packets of the SimInterference.")
                                ("InterferenceMode, f", po::value<bool>(&interferenceMode)->default_value(false), "Use this lanradio to generate interference on the physical channel.")
								("usePacketInjection, j", po::value<bool>(&usePacketInjection)->default_value(true), "Use packet injection to transmit data instead.")
                                ("transmitPower, t", po::value<int>(&transmitPower)->default_value(2), "Transmit power in dBm")
								("modifyMac, 0", po::value<bool>(&modifyMac)->default_value(false), "Modify mac according to SimbaR.")
                        ;

        po::variables_map vm;
        po::store(po::parse_command_line(ac, av, desc), vm);
        po::notify(vm);

        if (vm.count("help")) {
            cout << desc << "usage: --interface wlan1 --echo 0 --baseport 19191\n";
            return 0;
        }

    }
    catch(exception& e) {
        cerr << "error: " << e.what() << "\n";
        return 1;
    }
    catch(...) {
        cerr << "Exception of unknown type!\n";
    }

    if(baseport < 1024 || baseport > 65533) {
        cerr << "baseport is not a valid high port\n";
        return 1;
    }
    // Print config
    std::cout << "Config:\n\tinterface: " << interface
              << "\n\techo: " << echo
              << "\n\tbaseport: " << baseport
              << "\n\tprototype: " << isPrototype
              << "\n\tVeinsHostname: " << veinsHostname
              << "\n\tuseUDP: " << use_udp
              << "\n\tuseChanload: " << measure_chanload
              << "\n\tdualRadio: " << dualRadio
              << "\n\tdualRadioAth9kMac: " << dualRadioAth9kMac
              << "\n\tinterferenceMode: " << interferenceMode
              << "\n\tinterferenceMac: " << interferenceMac
			  << "\n\tusePacketInjection: " << usePacketInjection
			  << "\n\ttransmitPower: " << transmitPower
			  << "\n\tmodifyMac: " << modifyMac
              << std::endl;

    /* create udp socket */
        int optval = 1;
        int sock;
        struct sockaddr_in hostAddr;
        memset(&hostAddr, 0, sizeof(hostAddr));

    if (!measure_chanload) {
        sock = socket (AF_INET, SOCK_DGRAM, 0);
        if (sock < 0) {
            std::cout << ": error creating udp socket: " << sock << std::endl;;
            exit (EXIT_FAILURE);
        }
        setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(int));

        struct hostent* hostinfo;
        hostinfo = gethostbyname(veinsHostname.c_str());
        if (hostinfo == NULL) {
            std::cout << "Error: erroneous hostname: " << veinsHostname << std::endl;
        }else{
            std::cout << "hostname: " << veinsHostname << std::endl;
        }
        hostAddr.sin_family = AF_INET;
        //hostAddr.sin_addr = *(struct in_addr *) hostinfo->h_addr;
        memcpy((void *)&hostAddr.sin_addr, hostinfo->h_addr_list[0], hostinfo->h_length);
        if (!isPrototype) {
            hostAddr.sin_port = htons (baseport + 6);
            std::cout << "hostAddr udp socket port: " << baseport + 6 << std::endl;
        }else{
            hostAddr.sin_port = htons (baseport + 8);
            std::cout << "hostAddr udp socket port: " << baseport + 8 << std::endl;
        }
        if (interferenceMode) {
            hostAddr.sin_port = htons (baseport + 1);
            std::cout << "hostAddr udp socket port: " << baseport + 1  << std::endl;
        }

        struct sockaddr_in myAddr;
        myAddr.sin_family = AF_INET;
        myAddr.sin_addr.s_addr = htonl (INADDR_ANY);
        if (!isPrototype) {
            myAddr.sin_port = htons (baseport + 6);
            std::cout << "my udp socket port: " << baseport + 6 << std::endl;
        }else{
            myAddr.sin_port = htons (baseport + 8);
            std::cout << "my udp socket port: " << baseport + 8 << std::endl;
        }
        if (interferenceMode) {
            myAddr.sin_port = htons (baseport + 1);
            std::cout << "hostAddr udp socket port: " << baseport + 1 << std::endl;
        }

        std::cout << "using udp" << std::endl;
        int r;
        if (use_udp) {
            r = bind (sock, (struct sockaddr *) &myAddr, sizeof(myAddr));
            if (r < 0) {
                exit (EXIT_FAILURE);
            }
        }
    }

    SendToHardwareViaMAC mac_tx("unknown", interface);
    // if dualRadio == true: use ath5k/wlan0 for receiving and ath9k/wlan1 for transmittig
    // else use the radio specified in interface for receiving
    ReceiveFromHardwareViaMAC mac_rx = (isPrototype || !dualRadio) ? ReceiveFromHardwareViaMAC("unknown", interface, isPrototype, usePacketInjection) : ReceiveFromHardwareViaMAC("unknown", "wlan0", isPrototype, usePacketInjection);

    std::string phyName = interface.compare("wlan0") == 0 ? "phy0" : "phy1";
    SendToTins pi_tx(interface, phyName, isPrototype, transmitPower);

    if (modifyMac) {
        std::cout << "Modify MAC." << std::endl;
        // Manipulate channel access mechanism for interference mode
        if (interferenceMode) {
            std::string fci_req("echo 1 > /sys/kernel/debug/ieee80211/" + phyName + "/ath9k/registers/force_channel_idle");
            redi::ipstream fci_proc(fci_req, redi::pstreams::pstdout | redi::pstreams::pstderr);
            fci_proc.close();
            std::string ivc_req("echo 1 > /sys/kernel/debug/ieee80211/" + phyName + "/ath9k/registers/ignore_virt_cs");
            redi::ipstream ivc_proc(ivc_req, redi::pstreams::pstdout | redi::pstreams::pstderr);
            ivc_proc.close();
            std::string iib_req("echo 1 > /sys/kernel/debug/ieee80211/" + phyName + "/ath9k/registers/ifs_ignore_backoff");
            redi::ipstream iib_proc(iib_req, redi::pstreams::pstdout | redi::pstreams::pstderr);
            iib_proc.close();
            std::string rx_disable_req("echo 1 > /sys/kernel/debug/ieee80211/" + phyName + "/ath9k/registers/diag_rx_disable");
            redi::ipstream rx_disable_proc(rx_disable_req, redi::pstreams::pstdout | redi::pstreams::pstderr);
            rx_disable_proc.close();
            std::string time_sifs_req("echo 2 > /sys/kernel/debug/ieee80211/" + phyName + "/ath9k/time_sifs");
            redi::ipstream time_sifs_proc(time_sifs_req, redi::pstreams::pstdout | redi::pstreams::pstderr);
            time_sifs_proc.close();
            std::string time_slottime_req("echo 1 > /sys/kernel/debug/ieee80211/" + phyName + "/ath9k/time_slottime");
            redi::ipstream time_slottime_proc(time_slottime_req, redi::pstreams::pstdout | redi::pstreams::pstderr);
            time_slottime_proc.close();
            std::string time_eifs_req("echo 1 > /sys/kernel/debug/ieee80211/" + phyName + "/ath9k/time_eifs");
            redi::ipstream time_eifs_proc(time_eifs_req, redi::pstreams::pstdout | redi::pstreams::pstderr);
            time_eifs_proc.close();
            std::string fcs_req("echo 1 > /sys/kernel/debug/ieee80211/" + phyName + "/ath9k/registers/diag_corrupt_fcs");
            redi::ipstream fcs_proc(fcs_req, redi::pstreams::pstdout | redi::pstreams::pstderr);
            fcs_proc.close();
        }else if (!isPrototype && !measure_chanload) { //manipualte channel access mechansim for SimInterface
            std::string fci_req("echo 1 > /sys/kernel/debug/ieee80211/" + phyName + "/ath9k/registers/force_channel_idle");
            redi::ipstream fci_proc(fci_req, redi::pstreams::pstdout | redi::pstreams::pstderr);
            fci_proc.close();
            std::string ivc_req("echo 1 > /sys/kernel/debug/ieee80211/" + phyName + "/ath9k/registers/ignore_virt_cs");
            redi::ipstream ivc_proc(ivc_req, redi::pstreams::pstdout | redi::pstreams::pstderr);
            ivc_proc.close();
            std::string iib_req("echo 1 > /sys/kernel/debug/ieee80211/" + phyName + "/ath9k/registers/ifs_ignore_backoff");
            redi::ipstream iib_proc(iib_req, redi::pstreams::pstdout | redi::pstreams::pstderr);
            iib_proc.close();
            std::string rx_disable_req("echo 1 > /sys/kernel/debug/ieee80211/" + phyName + "/ath9k/registers/diag_rx_disable");
            redi::ipstream rx_disable_proc(rx_disable_req, redi::pstreams::pstdout | redi::pstreams::pstderr);
            rx_disable_proc.close();
            std::string time_sifs_req("echo 2 > /sys/kernel/debug/ieee80211/" + phyName + "/ath9k/time_sifs");
            redi::ipstream time_sifs_proc(time_sifs_req, redi::pstreams::pstdout | redi::pstreams::pstderr);
            time_sifs_proc.close();
            std::string time_slottime_req("echo 1 > /sys/kernel/debug/ieee80211/" + phyName + "/ath9k/time_slottime");
            redi::ipstream time_slottime_proc(time_slottime_req, redi::pstreams::pstdout | redi::pstreams::pstderr);
            time_slottime_proc.close();
            std::string time_eifs_req("echo 1 > /sys/kernel/debug/ieee80211/" + phyName + "/ath9k/time_eifs");
            redi::ipstream time_eifs_proc(time_eifs_req, redi::pstreams::pstdout | redi::pstreams::pstderr);
            time_eifs_proc.close();
        }
    }

    zmq::context_t zmq_context;

    struct txWlanThreadData myTxWlanThreadData;
    pthread_t txWlanThread;
    struct rxWlanThreadData myRxWlanThreadData;;
    pthread_t rxWlanThread;
    struct radioConfigThreadData myRadioConfigThreadData;
    pthread_t radioConfigThread;
    int rc;

    if (!measure_chanload) {
        if (!isPrototype) {
            // tx wlan thread
            myTxWlanThreadData.mac = &mac_tx;
            myTxWlanThreadData.context = &zmq_context;
            myTxWlanThreadData.host = veinsHostname;
            myTxWlanThreadData.port = baseport + 2;
            myTxWlanThreadData.use_udp = use_udp;
            myTxWlanThreadData.sock = sock;
            myTxWlanThreadData.useDualRadio = dualRadio;
            myTxWlanThreadData.dualRadioAth9kMac = dualRadioAth9kMac;
            myTxWlanThreadData.interferenceMac = interferenceMac;
            myTxWlanThreadData.interferenceMode = interferenceMode;
            myTxWlanThreadData.usePacketInjection = usePacketInjection;
			myTxWlanThreadData.pi_tx = &pi_tx;

            // rx wlan thread
            myRxWlanThreadData.mac_rx = &mac_rx;
            myRxWlanThreadData.mac_tx = &mac_tx;
            myRxWlanThreadData.receive_own_msgs = false;
            myRxWlanThreadData.receive_every_ether_type = false;
            myRxWlanThreadData.context = &zmq_context;
            myRxWlanThreadData.host = veinsHostname;
            myRxWlanThreadData.port = baseport + 6;
            myRxWlanThreadData.use_udp = use_udp;
            myRxWlanThreadData.sock = sock;
            myRxWlanThreadData.hostAddr = &hostAddr;
            myRxWlanThreadData.useDualRadio = dualRadio;
            myRxWlanThreadData.dualRadioAth9kMac = dualRadioAth9kMac;
            myRxWlanThreadData.interferenceMac = interferenceMac;
            myRxWlanThreadData.usePacketInjection = usePacketInjection;
			myRxWlanThreadData.pi_tx = &pi_tx;
        }else{
            // tx wlan thread
            myTxWlanThreadData.mac = &mac_tx;
            myTxWlanThreadData.context = &zmq_context;
            myTxWlanThreadData.host = veinsHostname;
            myTxWlanThreadData.port = baseport + 4;
            myTxWlanThreadData.use_udp = use_udp;
            myTxWlanThreadData.sock = sock;
            myTxWlanThreadData.useDualRadio = dualRadio;
            myTxWlanThreadData.dualRadioAth9kMac = dualRadioAth9kMac;
            myTxWlanThreadData.interferenceMac = interferenceMac;
            myTxWlanThreadData.interferenceMode = interferenceMode;
            myTxWlanThreadData.usePacketInjection = usePacketInjection;
			myTxWlanThreadData.pi_tx = &pi_tx;

            // rx wlan thread
            myRxWlanThreadData.mac_rx = &mac_rx;
            myRxWlanThreadData.mac_tx = &mac_tx;
            myRxWlanThreadData.receive_own_msgs = false;
            myRxWlanThreadData.receive_every_ether_type = false;
            myRxWlanThreadData.context = &zmq_context;
            myRxWlanThreadData.host = veinsHostname;
            myRxWlanThreadData.port = baseport + 8;
            myRxWlanThreadData.use_udp = use_udp;
            myRxWlanThreadData.sock = sock;
            myRxWlanThreadData.hostAddr = &hostAddr;
            myRxWlanThreadData.useDualRadio = dualRadio;
            myRxWlanThreadData.dualRadioAth9kMac = dualRadioAth9kMac;
            myRxWlanThreadData.interferenceMac = interferenceMac;
            myRxWlanThreadData.usePacketInjection = usePacketInjection;
			myRxWlanThreadData.pi_tx = &pi_tx;
        }
        if (interferenceMode) {
            // tx wlan thread
            myTxWlanThreadData.mac = &mac_tx;
            myTxWlanThreadData.context = &zmq_context;
            myTxWlanThreadData.host = veinsHostname;
            myTxWlanThreadData.port = baseport + 4;
            myTxWlanThreadData.use_udp = use_udp;
            myTxWlanThreadData.sock = sock;
            myTxWlanThreadData.useDualRadio = dualRadio;
            myTxWlanThreadData.dualRadioAth9kMac = dualRadioAth9kMac;
            myTxWlanThreadData.interferenceMac = interferenceMac;
            myTxWlanThreadData.interferenceMode = interferenceMode;
            myTxWlanThreadData.usePacketInjection = usePacketInjection;
			myTxWlanThreadData.pi_tx = &pi_tx;
        }
        rc = pthread_create(&txWlanThread, NULL, tx_wlan_thread, (void*) &myTxWlanThreadData);
        if (!interferenceMode) {
            rc = pthread_create(&rxWlanThread, NULL, rx_wlan_thread, (void*) &myRxWlanThreadData);
        }

    }
    myRadioConfigThreadData.context = &zmq_context;
    myRadioConfigThreadData.host = veinsHostname;
    myRadioConfigThreadData.interferenceMode = interferenceMode;
    if (!isPrototype) {
        if (measure_chanload) {
            myRadioConfigThreadData.port = baseport + 9;
        }else if(interferenceMode) {
            myRadioConfigThreadData.port = baseport;
        }else{
            myRadioConfigThreadData.port = baseport;
        }
    }else{
        myRadioConfigThreadData.port = baseport + 1;
    }
    myRadioConfigThreadData.use_udp = use_udp;
    myRadioConfigThreadData.sock = sock;
    myRadioConfigThreadData.hostAddr = &hostAddr;
    myRadioConfigThreadData.thread_tx_wlan = &txWlanThread;
    myRadioConfigThreadData.thread_rx_wlan = &rxWlanThread;
    rc = pthread_create(&radioConfigThread, NULL, configure_radio_thread, (void*) &myRadioConfigThreadData);

    void* cmd = NULL;
    if (!measure_chanload) {
        pthread_join(txWlanThread, NULL);
        std::cout << "joined txWlanThread" << std::endl;
    }
    pthread_join(radioConfigThread, &cmd);
    std::cout << "joined radioConfigThread" << std::endl;
    if (!measure_chanload && !interferenceMode) {
        pthread_join(rxWlanThread, NULL);
        std::cout << "joined rxWlanThread" << std::endl;
    }
    std::string* restartCmd = (string*) cmd;
    std::cout << "before closing zmq context." << std::endl;
    std::cout << "restartCmd: " << *restartCmd << std::endl;
    zmq_context.close();
    std::cout << "main: zmq is shut down. restarting..." << std::endl;
    rc = shutdown(sock, SHUT_RDWR);
    if (rc != 0) {
        std::cout << "shutdown udp socket failed: " << errno << std::endl;
    }
    close(sock);
    //system((*restartCmd).c_str());
    return EXIT_SUCCESS;
}
