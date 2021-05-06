//
// Copyright (C) 2018 Florian Klingler <klingler@ccs-labs.org>
// Copyright (C) 2018 Dominik S. Buse <buse@ccs-labs.org>
// Copyright (C) 2021 Mario Franke <research@m-franke.net>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

#ifndef __VEINS_LARA_LANRADIOMAC1609_4_H
#define __VEINS_LARA_LANRADIOMAC1609_4_H

#include "veins/modules/mac/ieee80211p/Mac1609_4.h"
#include "SimbaR/LanradioMacInterface.h"
#include "SimbaR/LanradioPhyLayer80211p.h"

namespace veins {

class LanradioCommunication;

/**
 * @brief
 * Enables LAN Radio support for Mac 1609_4
 *
 * @author Dominik S. Buse: extracted to separate class
 * @author Florian Klingler: initial version
 *
 * @see Mac1609_4
 */
class LanradioMac1609_4 : public Mac1609_4, public LanradioMacInterface {
public:
    enum t_packet_type {
        PT_SIM,
        PT_LARA
    };

public:
    void sendMacPacketDirect(cMessage* msg) override;
    void toApplicationLayer(cMessage* msg) override;
    void setAsLanradioMac(bool isNewLanradioMac) override;

    /** @brief send message to prototype to be received by the simulation */
    void sendPrototypeToSimulation(cMessage* msg);

    /* logging */
    cOutVector physicalPacketCollisions;
    int physicalPacketCollisionsPerSec = 0;

protected:
    /** @brief type of last sent packet*/
    t_packet_type lastPT;
    LanradioCommunication* lara = nullptr;

    cMessage* check_physical_packet_collisions;
    cQueue subsequentPacketsQueue;

    void sendSubsequentPacket();

protected:
    /** @brief Initialization of the module and some variables.*/
    virtual void initialize(int) override;

    virtual void finish() override;

    /** @brief Handle self messages such as timers.*/
    virtual void handleSelfMsg(cMessage*) override;

    /** @brief Handle control messages from lower layer.*/
    virtual void handleLowerControl(cMessage* msg) override;

    /** @brief Handle received broadcast */
    virtual void handleBroadcast(Mac80211Pkt* macPkt, DeciderResult80211* res) override;

    /** @brief CAMs and SPATs are copied to LanRadio in here */
    virtual void sendLanradioMsg(cMessage* msg);
};

} // namespace veins
#endif
