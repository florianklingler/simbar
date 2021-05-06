//
// Copyright (C) 2011 David Eckhoff <eckhoff@cs.fau.de>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// SPDX-License-Identifier: GPL-2.0-or-later
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

#pragma once

#include "veins/modules/phy/PhyLayer80211p.h"
#include "veins/modules/messages/Mac80211Pkt_m.h"
#include "SimbaR/LanradioMac1609_4ToLanradioPhy80211pInterface.h"
#include "SimbaR/LanradioDecider80211p.h"
#include "SimbaR/LanradioInterference.h"
#include "SimbaR/LanradioDecider80211pToLanradioPhy80211pInterface.h"

namespace veins {

/**
 * @brief
 * Adaptation of the PhyLayer class for 802.11p.
 *
 * @ingroup phyLayer
 *
 * @see DemoBaseApplLayer
 * @see Mac1609_4
 * @see PhyLayer80211p
 * @see Decider80211p
 */
class VEINS_API LanradioPhyLayer80211p : public PhyLayer80211p, public LanradioMac1609_4ToLanradioPhy80211pInterface, public LanradioDecider80211pToLanradioPhy80211pInterface {
public:
    void initialize(int stage) override;

protected:
    LanradioInterference* lara_int = nullptr;
    virtual std::unique_ptr<Decider> getDeciderFromName(std::string name, ParameterMap& params) override;

    virtual std::unique_ptr<Decider> initializeLanradioDecider80211p(ParameterMap& params);

    void sendInterferenceFrameOnPhysicalChannel(AirFrame* frame) override;

    void setLanradioInterferenceModule();
};

} // namespace veins
