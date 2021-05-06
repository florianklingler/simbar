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

namespace veins {

/**
 * @brief
 * Interface of LanradioPhyLayer80211p exposed to LanradioDecider80211p.
 *
 * @author David Eckhoff
 * @author Mario Franke
 *
 * @ingroup phyLayer
 */
class VEINS_API LanradioDecider80211pToLanradioPhy80211pInterface {
public:
    virtual ~LanradioDecider80211pToLanradioPhy80211pInterface(){};
    virtual void sendInterferenceFrameOnPhysicalChannel(AirFrame* frame) = 0;
};

} // namespace veins
