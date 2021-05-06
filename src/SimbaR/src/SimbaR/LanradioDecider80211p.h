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

#include "veins/modules/phy/Decider80211p.h"
#include "veins/modules/utility/Consts80211p.h"
#include "veins/modules/mac/ieee80211p/Mac80211pToPhy11pInterface.h"
#include "veins/modules/phy/Decider80211pToPhy80211pInterface.h"
#include "SimbaR/LanradioDecider80211pToLanradioPhy80211pInterface.h"

namespace veins {

using veins::AirFrame;

/**
 * @brief
 * Based on Decider80211.h
 * @author Mario Franke
 */
class VEINS_API LanradioDecider80211p : public Decider80211p {
public:

protected:
    LanradioDecider80211pToLanradioPhy80211pInterface* lara_phy11p;

protected:
    simtime_t processNewSignal(AirFrame* frame) override;

public:
    /**
     * @brief Initializes the Decider with a pointer to its PhyLayer and
     * specific values for threshold and minPowerLevel
     */
    LanradioDecider80211p(cComponent* owner, DeciderToPhyInterface* phy, double minPowerLevel, double ccaThreshold, bool allowTxDuringRx, double centerFrequency, int myIndex = -1, bool collectCollisionStatistics = false)
        : Decider80211p(owner, phy, minPowerLevel, ccaThreshold, allowTxDuringRx, centerFrequency, myIndex, collectCollisionStatistics)
    {
        std::cout << "Using LanradioDecider80211p." << std::endl;
        lara_phy11p = dynamic_cast<LanradioDecider80211pToLanradioPhy80211pInterface*>(phy11p);
    }

    ~LanradioDecider80211p() override;
};

} // namespace veins
