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


//TODO: NON BLOCKING socket?

#include "SendToTins.h"
#include <ctype.h>
// #include <common/buffers/data.pb.h>
// #include <common/utility/Constants.h>
#include <string>
#include <fstream>
#include <streambuf>
#include <boost/algorithm/string.hpp>
#include "pstreams/pstream.h"
// #include "exec.h"

using namespace std;
const Tins::PDU::PDUType RsseyePDU::pdu_flag = static_cast<Tins::PDU::PDUType>(Tins::PDU::USER_DEFINED_PDU);


//SendToTins::SendToTins(string interfaceName, string countryCode, int frequency, LoggingUtility& logger): mLogger(logger) {
SendToTins::SendToTins(std::string interfaceName, std::string phyName, bool prototype, int transmitPower) {
    std::cout << "SendToTins phyName: " << phyName << std::endl;
    // std::ifstream t("/sys/class/net/" + interfaceName + "/phy80211/name");
    //std::string phyName((std::istreambuf_iterator<char>(t)),
    //             std::istreambuf_iterator<char>());
    // std::string phyName("phy1");
    boost::algorithm::trim(phyName);

    // mLogger.logInfo("Setup card " + phyName);
    // execProcess("iw reg set " + countryCode);
    // execProcess("ip link set " + interfaceName + " down");
    // execProcess("iw dev " + interfaceName + " set type monitor");
    // execProcess("ip link set " + interfaceName + " up");
    // execProcess("iw dev " + interfaceName + " set freq " + to_string(frequency));
    std::string req("echo 10 > /sys/kernel/debug/ieee80211/" + phyName + "/ath9k/chanbw");
    redi::ipstream proc(req, redi::pstreams::pstdout | redi::pstreams::pstderr);
    proc.close();

    isPrototype = prototype;
    transmitPower_dBm = transmitPower;
    std::cout << "SendToTins Contstructor: ETHERTYPE_CAR: " << ETHERTYPE_CAR << std::endl;
    Tins::Allocators::register_allocator<Tins::EthernetII, RsseyePDU>(ETHERTYPE_CAR);

    sender.default_interface(interfaceName);
    Tins::NetworkInterface iface(interfaceName);
    auto info = iface.addresses();
    senderAddress = info.hw_addr;
    std::cout << senderAddress << std::endl;
    //fd = socket(AF_INET, SOCK_DGRAM, 0);

    mOwnMac = ether_ntoa(ether_aton(senderAddress.to_string().c_str()));

    // mLogger.logInfo("Finish init of Send");
}

void SendToTins::sendPDU(RsseyePDU& pdu, wlanPackage::WLAN& msg, uint8_t rate, bool tenMHzChannel){
    //Radiotap is multiplies of 500kbit
    // uint8_t newRate = msg.bitrate() * 2;
    //uint8_t newRate = msg.bitrate() * 2; // how to set it
    uint8_t newRate = rate; // uint is 500kbps, 24 * 500kbps = 12Mbps
    if (newRate == 0){
        std::cout << "SendToTins: Rate is 0, cannot send" << std::endl;
        return;
    }
    // int8_t newPower;
    // if (isPrototype) {
    //     newPower = 20; // dBm
    //     //std::cout << "DUT sending with 4dBm." << std::endl;
    // }else{
    //     newPower = 2; // dBm
    //     //std::cout << "simInt sending with 2dBm." << std::endl;
    // }

    // std::cout << "SendToTins: sendPDU: Create SNAP" << std::endl;;
    Tins::SNAP snap = Tins::SNAP() / pdu;
    snap.eth_type(ETHERTYPE_CAR);

    //Tins::Dot11QoSData qos = Tins::Dot11QoSData(Tins::Dot11::BROADCAST, senderAddress) / snap;
    //std::cout << "msg.srcmac()" << msg.srcmac() << std::endl;
    // std::cout << "SendToTins: sendPDU: Create Dot11QoSData" << std::endl;;
    Tins::Dot11QoSData qos;
    try{
        qos = Tins::Dot11QoSData(Tins::Dot11::BROADCAST, msg.srcmac()) / snap;
    }catch(Tins::invalid_address& e){
        //std::cout << "catched invalid address srcmac is now: " << senderAddress << std::endl;
        qos = Tins::Dot11QoSData(Tins::Dot11::BROADCAST, senderAddress) / snap;
    }
    qos.qos_control(0x00000020);
    qos.addr3(Tins::Dot11::BROADCAST);
    if (seq_num == 4095) {
        seq_num = 0;
    }else{
        seq_num = seq_num + 1;
    }
    qos.seq_num(seq_num);
    // std::cout << "SendToTins: seq_num: " << seq_num << std::endl;

    // std::cout << "SendToTins: sendPDU: Create RadioTapHeader." << std::endl;;
    Tins::RadioTap radio = Tins::RadioTap() / qos;
    //Optional set parameters (currently ignored)
    uint16_t half_rate = 0x4000;
    uint16_t five_ghz = Tins::RadioTap::ChannelType::FIVE_GZ;
    uint16_t ofdm = Tins::RadioTap::ChannelType::OFDM;
    if (tenMHzChannel) {
        radio.channel(5900,  ofdm | five_ghz | half_rate);
        // std::cout << "SendToTins: sendPDU: set radio for data" << std::endl;
    }else{
        radio.channel(5900,  ofdm | five_ghz | half_rate);
        // std::cout << "SendToTins: sendPDU: set radio for interference" << std::endl;
    }
    // std::cout << "SendToTins: sendPDU: Set rate to " <<  std::to_string(newRate) << std::endl;;
    radio.rate(newRate);
    //std::cout << "SendToTins: sendPDU: Set power to " << std::to_string(transmitPower_dBm) << std::endl;;
    radio.dbm_signal(transmitPower_dBm);

    // std::cout << "SendToTins: sendPDU: sender.send(radio)" << std::endl;
    sender.send(radio);
    //std::cout << "SendToTins: transmit packet" << std::endl;
}

void SendToTins::sendWithGeoNet(wlanPackage::WLAN& msg, int priority) {
    std::string payload = msg.payload();
    // unsigned int geoHdrLen;
    // uint8_t* geoHdr;
    // mLogger.logInfo("Start switch");
    // switch(msgData.type()) {
    //     case dataPackage::DATA_Type_DENM:
    //         fillGeoNetBTPheaderForDenm(msg.size());
    //         geoHdrLen = sizeof(struct GeoNetworkAndBTPHeaderDENM);
    //         geoHdr = reinterpret_cast<uint8_t*>(&mGeoBtpHdrForDenm);
    //         break;
    //     case dataPackage::DATA_Type_CAM:
    //         mLogger.logInfo("CAM");
    //         fillGeoNetBTPheaderForCam(msg.size());
    //         mLogger.logInfo("Fill ready");
    //         geoHdrLen = sizeof(struct GeoNetworkAndBTPHeaderCAM);
    //         geoHdr = reinterpret_cast<uint8_t*>(&mGeoBtpHdrForCam);
    //         break;
    //     default:
    //         mLogger.logError("Queued packet has invalid type: " + to_string(msgData.type()));
    //         return;
    // }
    //fillGeoNetBTPheaderForCam(payload.size());
    // std::cout << "SendToTins: Fill ready" << std::endl;
    //geoHdrLen = sizeof(struct GeoNetworkAndBTPHeaderCAM);
    //geoHdr = reinterpret_cast<uint8_t*>(&mGeoBtpHdrForCam);
    // std::cout << "SendToTins: Init vector" << std::endl;
    // std::vector<uint8_t> data(geoHdr, geoHdr + geoHdrLen);
    // std::cout << "SendToTins: sendWithGeoNet: payload size: " << payload.size() << std::endl;
    std::vector<uint8_t> data;
    std::copy(payload.c_str(), payload.c_str() + payload.size(), std::back_inserter(data));
    RsseyePDU raw = RsseyePDU(data);
    // std::cout << "SendToTins: sendWithGeoNet: Created pdu of size: " << data.size() << std::endl;
    sendPDU(raw, msg, 24, true);
}

void SendToTins::sendInterference(wlanPackage::WLAN& msg, int priority) {
    std::string payload = msg.payload();
    // std::cout << "SendToTins: sendInterference: payload size: " << payload.size() << std::endl;
    std::vector<uint8_t> data;
    std::copy(payload.c_str(), payload.c_str() + payload.size(), std::back_inserter(data));
    RsseyePDU raw = RsseyePDU(data);
    //std::cout << "SendToTins: sendInterference: Created pdu of size: " << data.size() << std::endl;
    sendPDU(raw, msg, 24, false); // TODO: Rate needs to be 12 since we want to send on a 20MHz channel, but currently only 10 MHz channels work
}

void SendToTins::fillGeoNetBTPheaderForCam(int payloadLen) {
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
    //mGeoBtpHdrForCam.mGeoNetHdr.tsb.spv.addr.llAddr = reinterpret_cast<uint8_t*>(ether_aton(mOwnMac.c_str()));

    //TODO uncomment
    memcpy(&mGeoBtpHdrForCam.mGeoNetHdr.tsb.spv.addr.llAddr, ether_aton(mOwnMac.c_str()), ETH_ALEN);
    mGeoBtpHdrForCam.mGeoNetHdr.tsb.spv.timestamp = htonl(2810450329);
    mGeoBtpHdrForCam.mGeoNetHdr.tsb.spv.latitude = htonl(424937722);
    mGeoBtpHdrForCam.mGeoNetHdr.tsb.spv.longitude = htonl(3460636913);
    mGeoBtpHdrForCam.mGeoNetHdr.tsb.spv.speed = htons(496);
    mGeoBtpHdrForCam.mGeoNetHdr.tsb.spv.heading = htons(1996);

    mGeoBtpHdrForCam.mGeoNetHdr.tsb.reserved = htonl(0);

    // BTP Header
    mGeoBtpHdrForCam.mBTPHdr.mDestinationPort = htons(2001);
    mGeoBtpHdrForCam.mBTPHdr.mSourcePort = htons(0);
    //uint8_t* temp = reinterpret_cast<uint8_t*>(&mGeoBtpHdrForCam);
    //dumpBuffer(temp, sizeof(mGeoBtpHdrForCam));
}

void SendToTins::fillGeoNetBTPheaderForDenm(int payloadLen) {
    // GeoNetwork Header
    mGeoBtpHdrForDenm.mGeoNetHdr.basicHeader.versionAndNH = 1;
    mGeoBtpHdrForDenm.mGeoNetHdr.basicHeader.reserved = 0;
    mGeoBtpHdrForDenm.mGeoNetHdr.basicHeader.lifetime = 241;
    mGeoBtpHdrForDenm.mGeoNetHdr.basicHeader.remainingHopLimit = 1;

    mGeoBtpHdrForDenm.mGeoNetHdr.commonHeader.nhAndReserved = 32;
    mGeoBtpHdrForDenm.mGeoNetHdr.commonHeader.htAndHst = 66;
    mGeoBtpHdrForDenm.mGeoNetHdr.commonHeader.tc = 1;
    mGeoBtpHdrForDenm.mGeoNetHdr.commonHeader.payload = htons(payloadLen + sizeof(struct BTPHeader));
    mGeoBtpHdrForDenm.mGeoNetHdr.commonHeader.maxHop = 1;
    mGeoBtpHdrForDenm.mGeoNetHdr.commonHeader.reserved = 0;

    mGeoBtpHdrForDenm.mGeoNetHdr.brdcst.sequenceNumber = 0;
    mGeoBtpHdrForDenm.mGeoNetHdr.brdcst.reserved = 0;
    mGeoBtpHdrForDenm.mGeoNetHdr.brdcst.spv.addr.assignmentTypeCountryCode = htons(38393);
    //mGeoBtpHdrForDenm.mGeoNetHdr.tsb.spv.addr.llAddr = reinterpret_cast<uint8_t*>(ether_aton(mOwnMac.c_str()));


    memcpy(&mGeoBtpHdrForDenm.mGeoNetHdr.brdcst.spv.addr.llAddr, ether_aton(mOwnMac.c_str()), ETH_ALEN);
    mGeoBtpHdrForDenm.mGeoNetHdr.brdcst.spv.timestamp = htonl(2810450329);
    mGeoBtpHdrForDenm.mGeoNetHdr.brdcst.spv.latitude = htonl(424939708);
    mGeoBtpHdrForDenm.mGeoNetHdr.brdcst.spv.longitude = htonl(-834330986);
    mGeoBtpHdrForDenm.mGeoNetHdr.brdcst.spv.speed = htons(496);
    mGeoBtpHdrForDenm.mGeoNetHdr.brdcst.spv.heading = htons(1996);

    mGeoBtpHdrForDenm.mGeoNetHdr.brdcst.latitude = htonl(424939708);
    mGeoBtpHdrForDenm.mGeoNetHdr.brdcst.longitude = htonl(-834330985);
    mGeoBtpHdrForDenm.mGeoNetHdr.brdcst.distA = 200;
    mGeoBtpHdrForDenm.mGeoNetHdr.brdcst.distB = 100;
    mGeoBtpHdrForDenm.mGeoNetHdr.brdcst.angle = 0;
    mGeoBtpHdrForDenm.mGeoNetHdr.brdcst.resrvd = htonl(0);

    // BTP Header
    mGeoBtpHdrForDenm.mBTPHdr.mDestinationPort = htons(2002);
    mGeoBtpHdrForDenm.mBTPHdr.mSourcePort = htons(0);
    //uint8_t* temp = reinterpret_cast<uint8_t*>(&mGeoBtpHdrForDenm);
    //dumpBuffer(temp, sizeof(mGeoBtpHdrForDenm));
}

