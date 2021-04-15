// This file is part of OpenC2X.
//
// OpenC2X is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// OpenC2X is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with OpenC2X.  If not, see <http://www.gnu.org/licenses/>.
//
// Authors:
// Sven Laux <slaux@mail.uni-paderborn.de>
// Gurjashan Singh Pannu <gurjashan.pannu@ccs-labs.org>
// Stefan Schneider <stefan.schneider@ccs-labs.org>
// Jan Tiemann <janhentie@web.de>
// Florian Klingler <klingler@ccs-labs.org>


#include "ReceiveFromHardwareViaMAC.h"
#include "GeoNetHeaders.h"

using namespace std;

ReceiveFromHardwareViaMAC::ReceiveFromHardwareViaMAC(string ownerModule, string ethernetDevice, bool isPrototype, bool usePacketInjection) {
	//mLogger = new LoggingUtility(ownerModule, expNo, loggingConf, statisticConf);

	//has root?
	if (getuid() != 0){
		//mLogger->logInfo("Program needs root privileges");
		exit(1);
	}

	// PACKET SOCKET, receives ALL incoming packages in whole (with all headers)
	mSocket = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
	if (mSocket == -1){
		//mLogger->logPError("socket creation failed");
		exit(1);
	}

	//int flags = fcntl(mSocket, F_GETFL, 0);
	//fcntl(mSocket, F_SETFL, flags & ~O_NONBLOCK);

	int enable = 1;
	if (setsockopt(mSocket, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0) {
	    exit(1);
	}

	string  ethDevice = ethernetDevice; //get via console: "ip link show"
    std::cout << "ReceiveFromHardware: ethDevice: " << ethDevice << std::endl;

	bzero(&mIfr , sizeof(mIfr));

	//get index number of Network Interface
	strncpy(mIfr.ifr_name, ethDevice.c_str(),sizeof(mIfr.ifr_name));

	if (ioctl(mSocket, SIOCGIFINDEX, &mIfr) != 0){
	    //mLogger->logPError("ioctl (SIOCGIFINDEX) failed");
	    std::cerr << "ioctl (SIOCGIFINDEX) failed" << std::endl;
	    exit(1);
	}

	//define socket address
	memset(&mTo_sock_addr, 0, sizeof(struct sockaddr_ll));
	mTo_sock_addr.sll_ifindex = mIfr.ifr_ifindex;
	mTo_sock_addr.sll_family = AF_PACKET;
	mTo_sock_addr.sll_protocol = htons(ETH_P_ALL);
	bind(mSocket, (struct sockaddr *) &mTo_sock_addr, sizeof(mTo_sock_addr));
	//std::cerr << "interface: " << mTo_sock_addr.sll_ifindex << std::endl;
    //
    useMonitorInterface = isPrototype && usePacketInjection;
    if (useMonitorInterface) {
        mLinkLayerAndRadioTapLength = mLinkLayerLength + monitorInterfaceRadioTapOffset;
        mEth_hdr = (struct ether_header*) (mPacket + monitorInterfaceRadioTapOffset);
        mPayload = mPacket + mLinkLayerAndRadioTapLength;
        std::cout << "ReceiveFromHardware: Prototype: shift eth_hdr and payload pointer." << std::endl;
    }else{
        mLinkLayerAndRadioTapLength = mLinkLayerLength;
        mEth_hdr = (struct ether_header*) mPacket;;
        mPayload = mPacket + mLinkLayerLength;
        std::cout << "ReceiveFromHardware: no Prototype: no shift." << std::endl;
    }
}

ReceiveFromHardwareViaMAC::~ReceiveFromHardwareViaMAC() {
	//delete mLogger;
	close(mSocket);
}

int ReceiveFromHardwareViaMAC::getSocket() {
    return mSocket;
}

pair<ReceivedPacketInfo, string> ReceiveFromHardwareViaMAC::receiveWLAN(bool receive_own_msgs, bool receive_every_ether_type, bool running, bool useDualRadio, std::string dualRadioAth9kMac, std::string interferenceMac) {
    // while(running) {
        /*
         * Filter outgoing start
         */

        //struct sockaddr_ll addr;
        //socklen_t addr_len = sizeof(addr);
        //int length = recvfrom(sock, packet, packetsize, 0, (struct sockaddr*)&addr, &addr_len);
        //if (length <= 0) {
        //    perror("recvfrom failed");
        //    exit(0);
        //}
        //if (addr.sll_pkttype == PACKET_OUTGOING) continue; // drop it

        //struct sockaddr_ll addr;
        //socklen_t addr_len = sizeof(addr);
        //mBytes = recvfrom(mSocket, mPacket, sizeof(mPacket), 0, (struct sockaddr*)&addr, &addr_len);
        socklen_t addr_len2 = sizeof(mTo_sock_addr);
        if (!running) {
            std::cout << "rx_wlan: ReceiveFromHardware: received shutdown signal before recvfrom" << std::endl;
            return make_pair(ReceivedPacketInfo(), "SHUTDOWN");
        }
        // std::cout << "ReceiveFromHardware: before recvfrom system call." << std::endl;
        mBytes = recvfrom(mSocket, mPacket, sizeof(mPacket), 0, (struct sockaddr*)&mTo_sock_addr, &addr_len2);
        //std::cout << "<<---------------new-packet----------------->>" << std::endl;
        // std::cout << "ReceiveFromHardware: after recvfrom system call." << std::endl;
        if (errno == EINTR) {
            std::cout << "rx_wlan: ReceiveFromHardware: received shutdown signal after recvfrom" << std::endl;
            return make_pair(ReceivedPacketInfo(), "SHUTDOWN");
        }
        if (mBytes <= 0) {
            //perror("recvfrom failed");
            // continue;
            //exit(1);
            std::cout << "ReceiveFromHardware: received to few bytes." << std::endl;
            return make_pair(ReceivedPacketInfo(), "REPEAT");
        }
        if (!receive_own_msgs && mTo_sock_addr.sll_pkttype == PACKET_OUTGOING) {
            //std::cerr << "received own Message, discarding" << std::endl;
            //std::cout << "ReceiveFromHardware: received own message." << std::endl;
            return make_pair(ReceivedPacketInfo(), "REPEAT");
        }
        //std::cout << "ReceiveFromHardware: received: " << mBytes << " bytes" << std::endl;
        //std::cout << "ReceiveFromHardware: first byte as hex: " << hex << (unsigned int) mPacket[0] << " bytes" << std::endl;


        // receive package, blocking
        //mBytes = read(mSocket, mPacket, sizeof(mPacket));
        // if (mBytes == -1) {
        //     //mLogger->logPError("reading from Socket failed");
        //     exit(1);
        // }
        if (!receive_every_ether_type && ntohs(mEth_hdr->ether_type) != ETHERTYPE_CAR) { // TODO: optimization
            // not GeoNetworking ethertype, ignore! Read next packet
            //std::cout << "ReceiveFromHardware: received message without geonetworking header." << std::endl;
            //std::cout << "ReceiveFromHardware: ether_type: " << ntohs(mEth_hdr->ether_type) << std::endl;
            return make_pair(ReceivedPacketInfo(), "REPEAT");
        }
        int geoNetPDULen;
        if (!useMonitorInterface) {
            geoNetPDULen = mBytes - mLinkLayerAndRadioTapLength;
        }else{
            geoNetPDULen = mBytes - mLinkLayerAndRadioTapLength - crcLength;
        }
        //std::cout << "ReceiveFromHardware: geoNetPDULen: " << geoNetPDULen << std::endl;
        // convert sender Mac from network byte order to char
        string senderMac = ether_ntoa((struct ether_addr*)(mEth_hdr->ether_shost));
        string recvInterferenceMac = ether_ntoa((struct ether_addr*)(mEth_hdr->ether_shost - 16));
        //std::cout << "rx_wlan: received packet from senderMac: " << senderMac << ", interferenceMac: " << recvInterferenceMac << std::endl;
        if (!receive_own_msgs && useDualRadio && senderMac.compare(dualRadioAth9kMac) == 0) {
            //std::cout << "rx_wlan: wlan0/ath5k received own packet from wlan1/ath9k." << std::endl;
            return make_pair(ReceivedPacketInfo(), "REPEAT");
        }
        if (recvInterferenceMac.compare(interferenceMac) == 0) {
            //std::cout << "rx_wlan: received interference." << std::endl;
            return make_pair(ReceivedPacketInfo(), "REPEAT");
        }

        //string receiverMac = ether_ntoa((struct ether_addr*)mEth_hdr->ether_dhost);
        string receiverMac = "FF:FF:FF:FF:FF:FF";
        //std::cout << "rx_wlan: receiveMac: " << receiverMac << std::endl;
        // Hack! As of now, we are looking for very specific bits in the GeoNetworking header
        char* geoNetPDU = mPayload;


        string msg(geoNetPDU, geoNetPDULen);
        ReceivedPacketInfo info;
        info.mSenderMac = senderMac;
        info.mReceiverMac = receiverMac;
        info.was_outgoing_packet = (mTo_sock_addr.sll_pkttype == PACKET_OUTGOING);
        info.msglength = geoNetPDULen;
        info.ether_type = ntohs(mEth_hdr->ether_type);
        // std::cout << "ReceiveFromHardware: received valid packet" << std::endl;
        return make_pair(info, msg);

    // }
    // std::cout << "ReceiveFromHardware: running: " << running << std::endl;
    // return make_pair(ReceivedPacketInfo(), "SHUTDOWN");
}
