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


#ifndef SENDTOTINS_H_
#define SENDTOTINS_H_

/**
 * @addtogroup dccHardware
 * @{
 */
#include "SendTo.h"
#include "RsseyePDU.h"

#include "GeoNetHeaders.h"

// #include <common/utility/LoggingUtility.h>
// #include <common/utility/Constants.h>
#include "Constants.h"
#include <string.h>

#include <unistd.h> // Std. Fct.  getuid() and read()
#include <sys/socket.h>
#include <arpa/inet.h>
#include <net/ethernet.h>
#include <netinet/ether.h>

#include <linux/if_packet.h>

#include <sys/ioctl.h>
#include <net/if.h>
#include <tins/tins.h>

#include "wlan.pb.h"

/**
 * Sends messages to Hardware to be broadcasted on the MAC layer. Uses RAW_SOCKET so root is needed.
 * Data is wrapped in a minimal Ethernet packet containing only a Ethernet header and the payload.
 * No IP header.
 * All data is broadcasted on the MAC layer.
 *
 *
 * @nonStandard serialization is not standard conform. Uses protobuffer instead.
 */
class SendToTins: public SendTo {
public:
    /**
     * ownerModule and expNo forwarded to LoggingUtility constructor
     * @param ownerModule Module Name
     * @param expNo Experiment Number
     * @param ethernetDevice Device used for sending
     */
    // SendToTins(std::string interfaceName, std::string countryCode, int frequency, LoggingUtility& logger);
    SendToTins(std::string interfaceName, std::string phyName, bool prototype, int transmitPower);
    virtual ~SendToTins() = default;

    /**
     * Sends pdu to the hardware
     * @param pdu
     */
    void sendPDU(RsseyePDU& pdu, wlanPackage::WLAN& msg, uint8_t rate, bool tenMHzChannel);

    /**
     * Sends standard compliant packet to the hardware queue with the corresponding priority.
     * @param msg
     * @param priority
     */
    void sendWithGeoNet(wlanPackage::WLAN& msg, int priority);

    /**
     * Transmit a packet used to generate interference on the physical channel.
     * @param msg
     * @param priority
     */
    void sendInterference(wlanPackage::WLAN& msg, int priority);

    void fillGeoNetBTPheaderForCam(int payloadLen);

    void fillGeoNetBTPheaderForDenm(int payloadLen);

private:
    bool isPrototype;
    int transmitPower_dBm;
    // LoggingUtility& mLogger;
    //std::string interfaceName;
    Tins::PacketSender sender;
    Tins::HWAddress<6> senderAddress;
    Tins::small_uint< 12 > seq_num = 0;
    // Basic struct for geo-networking and BTP header
    // Hard-coded for interoperability with Cohda MK5 box
    struct GeoNetworkAndBTPHeaderCAM mGeoBtpHdrForCam;
    struct GeoNetworkAndBTPHeaderDENM mGeoBtpHdrForDenm;
};

/**
 * @}
 */

#endif /* SENDTOTINS_H_ */
