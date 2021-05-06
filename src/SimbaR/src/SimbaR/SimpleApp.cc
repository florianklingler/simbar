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

#include "SimbaR/SimpleApp.h"
#include "SimbaR/CAMHelper.h"
#include "SimbaR/messages/CAMMessage_m.h"
#include "SimbaR/messages/SPATMessage_m.h"
#include "veins/base/utils/Heading.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"
#include "veins/modules/messages/BaseFrame1609_4_m.h"
#include "veins/modules/messages/DemoSafetyMessage_m.h"
#include <chrono>

using namespace veins;

Define_Module(veins::SimpleApp);

void SimpleApp::initialize(int stage)
{
    BaseApplLayer::initialize(stage);

    if (stage != 0) {
        return;
    }

    // initialize pointers to other modules
    assert(FindModule<TraCIMobility*>::findSubModule(getParentModule()) != nullptr);
    mobility = TraCIMobilityAccess().get(getParentModule());
    traci = mobility->getCommandInterface();
    myId = getParentModule()->getId();
    std::cout << "externalId: " << mobility->getExternalId() << std::endl;

    // read parameters
    headerLength = par("headerLength").intValue();
    beaconLengthBits = par("beaconLengthBits").intValue();
    beaconPriority = par("beaconPriority").intValue();
    app_debug = par("app_debug").boolValue();

    if (par("sendBeacons").boolValue()) {
        simtime_t beaconInterval = par("beacon_interval");
        simtime_t startBeaconAt = par("start_beaconing_at");
        simtime_t beaconStart = std::max(startBeaconAt, simTime()) + uniform(0, beaconInterval);;
        auto beaconTimer = TimerSpecification([this]() {
            this->sendCAM();
            // this->sendSPAT();
        }).absoluteStart(beaconStart).interval(beaconInterval);
        timerManager.create(beaconTimer);
    }

    first_CAM_generated = false;
    first_SPAT_generated = false;

    // MAC Address
    int a = 0xF2; // intuniform(0,0xFF) | 0x2;
    int b = intuniform(0, 0xFF);
    int c = intuniform(0, 0xFF);
    int d = intuniform(0, 0xFF);
    int e = intuniform(0, 0xFF);
    int f = intuniform(0, 0xFF);

    // std::stringstream srcmac;
    char srcMac[18];
    sprintf(srcMac, "%x:%x:%x:%x:%x:%x", a, b, c, d, e, f);
    myMacAddress = srcMac;

    // std::cerr << "myMacAddress: " << myMacAddress << std::endl;
}

void SimpleApp::populateWSM(BaseFrame1609_4* wsm, LAddress::L2Type rcvId, int serial)
{
    wsm->setRecipientAddress(rcvId);
    wsm->setBitLength(headerLength);

    if (DemoSafetyMessage* bsm = dynamic_cast<DemoSafetyMessage*>(wsm)) {
        bsm->setSenderPos(mobility->getPositionAt(simTime()));
        bsm->setSenderSpeed(mobility->getHostSpeed());
        bsm->setPsid(-1);
        bsm->setChannelNumber(static_cast<int>(Channel::cch));
        bsm->addBitLength(beaconLengthBits);
        wsm->setUserPriority(beaconPriority);
    }
}

void SimpleApp::handleLowerMsg(cMessage* msg)
{

    auto wsm = dynamic_cast<BaseFrame1609_4*>(msg);
    ASSERT(wsm != nullptr);

    if (auto cam = dynamic_cast<CAMMessage*>(wsm)) {
        if (app_debug) std::cerr << myId << ": received CAM: stationId = " << cam->getCam().header.stationID << std::endl;
    }
    else if (auto spat = dynamic_cast<SPATMessage*>(wsm)) {
        if (app_debug) std::cerr << myId << ": received SPAT: stationId = " << spat->getSpat().header.stationID << std::endl;
    }
    else {
        error("only CAMMessages and SPATMessages are currently supported");
    }

    delete (msg);
}

void SimpleApp::handleSelfMsg(cMessage* msg)
{
    if (timerManager.handleMessage(msg)) {
        return;
    }

    if (msg != nullptr) EV_DEBUG << "APP: Error: Got Self Message of unknown kind! Name: " << msg->getName() << endl;
}

void SimpleApp::sendCAM()
{
    Enter_Method_Silent();

    CAM_t cam = generate_CAM();

    auto msg = new CAMMessage("CAM");
    msg->setTimestamp(simTime());
    msg->setRecipientAddress(-1);
    msg->setBitLength(headerLength);
    msg->setPsid(-1);
    msg->setChannelNumber(static_cast<int>(Channel::cch));
    msg->addBitLength(beaconLengthBits);
    msg->setUserPriority(beaconPriority);
    msg->setSrcmac(myMacAddress.c_str());
    msg->setCam(cam);

    if (first_CAM_generated) {
        if (app_debug) std::cerr << myId << ": sending CAM" << std::endl;
        sendDown(msg);
    }
    else {
        first_CAM_generated = true;
    }
}

void SimpleApp::sendSPAT()
{
    Enter_Method_Silent();

    SPAT_PDU_t* spat = generate_SPAT();

    auto msg = new SPATMessage("SPAT");
    msg->setTimestamp(simTime());
    msg->setRecipientAddress(-1);
    msg->setBitLength(headerLength);
    msg->setPsid(-1);
    msg->setChannelNumber(static_cast<int>(Channel::cch));
    msg->addBitLength(beaconLengthBits);
    msg->setUserPriority(beaconPriority);
    msg->setSrcmac(myMacAddress.c_str());
    msg->setSpat(*spat);
    track_spat(spat); // cam is deleted there if not needed anymore

    if (first_SPAT_generated) {
        if (app_debug) std::cerr << myId << ": sending SPAT" << std::endl;
        sendDown(msg);
    }
    else {
        first_SPAT_generated = true;
    }
}

void SimpleApp::track_spat(SPAT_PDU_t* spat)
{
    if (sent_spats.size() >= sent_spats_max_size) {
        delete sent_spats.front();
        sent_spats.pop_front();
    }
    sent_spats.push_back(spat);
}

CAM_t SimpleApp::generate_CAM()
{
    int64_t currTime = std::chrono::high_resolution_clock::now().time_since_epoch() / std::chrono::nanoseconds(1);
    auto lonlat = traci->getLonLat(mobility->getPositionAt(simTime()));
    double alt = 0; // FIXME we do not have altitude infofmation
    auto headingBasedOnNorth = mobility->getHeading().getRad() - 90;
    if (headingBasedOnNorth < 0) headingBasedOnNorth += 180.0;

    CAMHelper camHelper;
    camHelper.stationId(myId);
    camHelper.lastSendingTime(lastCamSendingTime);
    camHelper.stationType(CAMHelper::StationType::base_vehicle);
    camHelper.currentPosition(lonlat.second, lonlat.first, alt);
    camHelper.heading(headingBasedOnNorth);
    camHelper.speed(mobility->getSpeed());

    lastCamHeading = headingBasedOnNorth;
    lastCamSendingTime = currTime;

    return camHelper.build();
}

SPAT_PDU_t* SimpleApp::generate_SPAT()
{
    // mLogger->logDebug("Generating CAM as per UPER");
    SPAT_PDU_t* spat = static_cast<SPAT_PDU_t*>(calloc(1, sizeof(SPAT_PDU_t)));
    // if (!cam) {
    //    throw runtime_error("could not allocate CAM_t");
    // }
    // ITS pdu header

    spat->header.stationID = myId; // mIdCounter; //
    spat->header.messageID = messageID_spatem;
    spat->header.protocolVersion = 1; // wasItsPduHeader__protocolVersion_currentVersion;

    // spat->spat.timeStamp = 0;

    // RegionalExtension re;
    // ASN_SEQUENCE_ADD(&spat->spat.regional->list, re);
    // spat->spat.

    IntersectionState* intstate = static_cast<IntersectionState*>(calloc(1, sizeof(IntersectionState)));
    if (intstate == NULL) {
        perror("error!");
    }
    intstate->id.id = myId;
    intstate->revision = 3;

    // IntersectionStatusObject_t
    intstate->status.buf = static_cast<uint8_t*>(calloc(2, sizeof(uint8_t)));
    if (intstate->status.buf == NULL) {
        perror("error!");
    }
    intstate->status.size = 2;
    intstate->status.buf[0] = 1;
    intstate->status.buf[1] = 2;
    intstate->status.bits_unused = 0;
    // IntersectionStatusObject_t

    // MovementState
    MovementState* mov = static_cast<MovementState*>(calloc(1, sizeof(MovementState)));
    mov->signalGroup = myId; // 27;

    MovementEvent* evt = static_cast<MovementEvent*>(calloc(1, sizeof(MovementEvent)));

    ASN_SEQUENCE_ADD(&mov->state_time_speed.list, evt);

    ASN_SEQUENCE_ADD(&intstate->states.list, mov);

    ASN_SEQUENCE_ADD(&spat->spatData.intersections.list, intstate);

    return spat;
}
