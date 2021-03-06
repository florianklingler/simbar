// This file is part of OpenC2X.
//
// OpenC2X is free software: you can redistribute it and/or modify // it under the terms of the GNU Lesser General Public License as published by
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


#ifndef RECEIVEFROMHARDWAREVIAMAC_H_
#define RECEIVEFROMHARDWAREVIAMAC_H_

/**
 * @addtogroup dccHardware
 * @{
 */

#include "Constants.h"
#include <string>
#include <unistd.h> // Std. Fct.  getuid() and read()
#include <netinet/in.h> // Socket Fct. (incl. <sys/socket.h>)
#include <netinet/ether.h> // Ethernet Fct. u. Kcnst.
// (ETH_P_ALL or ether_ntoa())
// (struct in_addr or inet_ntoa())
#include <net/ethernet.h> // Ethernet Header Structure
#include <tins/tins.h>
#include "RsseyePDU.h"

struct ReceivedPacketInfo {
	std::string mSenderMac;
	std::string mReceiverMac;
	bool was_outgoing_packet;
	int msglength;
	uint16_t ether_type;
};


/**
 * Receives Ethernet Packets from Hardware MAC Layer.
 */
class ReceiveFromHardwareViaMAC {
public:
	/**
	 * parameter forwarded to LoggingUtility constructor
	 * @param ownerModule Module Name
	 * @param expNo Experiment Number
	 */
	ReceiveFromHardwareViaMAC(std::string ownerModule, string ethernetDevice, bool isPrototype, bool usePacketInjection);
	virtual ~ReceiveFromHardwareViaMAC();
	void init();

	std::pair<ReceivedPacketInfo, std::string> receiveWLAN(bool receive_own_msgs, bool receive_every_ether_type, bool running, bool useDualRadio, std::string dualRadioAth9kMac, std::string interferenceMac);
    int getSocket();

private:

	int mSocket;
    char mPacket[ETHERMTU];

    //num of bytes received
    int mBytes = 0;

    bool useMonitorInterface;
    int monitorInterfaceRadioTapOffset = 56;
    int crcLength = 4;

    // pointers to access header and payload
    int mLinkLayerLength = sizeof(struct ether_header);
    int mLinkLayerAndRadioTapLength;
    //struct ether_header* mEth_hdr = (struct ether_header*) mPacket;;
    struct ether_header* mEth_hdr; // = (struct ether_header*) mPacket;;
    //char* mPayload = mPacket + mLinkLayerLength;
    char* mPayload;  // = mPacket + mLinkLayerLength;

    //information of network interface
    struct ifreq mIfr;
    //socket address data structure for PACKET sockets
    struct sockaddr_ll mTo_sock_addr;
};

/**
 * @}
 */

#endif /* RECEIVEFROMHARDWAREVIAMAC_H_ */
