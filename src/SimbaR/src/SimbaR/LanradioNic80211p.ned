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
// Mario Franke <research@m-franke.net>

package org.car2x.veins.subprojects.SimbaR;

import org.car2x.veins.modules.nic.INic80211p;
import org.car2x.veins.subprojects.SimbaR.LanradioPhyLayer80211p;
// import org.car2x.veins.modules.phy.PhyLayer80211p;
import org.car2x.veins.subprojects.SimbaR.LanradioMac1609_4;

@namespace(veins);

// Special NIC for Lanradio
//
// Every car (node) that should be able to interact with the physical channel
// via LanradioCommunication needs to have this NIC (set via Car.nicType).
// An arbitrary number of nodes can have this module instead of the default.
// However, only one can be registered as the actual active counterpart for
// the physical channel. This will be the one that has
//  - its mac1609_4.send_to_lanradio set to true and
//  - was most recently registered (each registration overrides old ones)
// Alternatively, a registration can be manually done in code by calling
//   LanradioMacInterface::setAsLanradioMac(true), e.g., from the application.
// (LanradioMac1609_4 implements LanradioMacInterface).
module LanradioNic80211p like INic80211p
{
    parameters:
        string connectionManagerName = default("connectionManager");
    gates:
        input upperLayerIn; // to upper layers
        output upperLayerOut; // from upper layers
        output upperControlOut; // control information
        input upperControlIn; // control information
        input radioIn; // radioIn gate for sendDirect

    submodules:
        phy80211p: LanradioPhyLayer80211p {
            @display("p=69,218;i=block/process_s");
        }

        mac1609_4: LanradioMac1609_4 {
            @display("p=69,82");
        }

    connections:
        radioIn --> phy80211p.radioIn;

        mac1609_4.lowerControlOut --> phy80211p.upperControlIn;
        mac1609_4.lowerLayerOut --> phy80211p.upperLayerIn;
        phy80211p.upperLayerOut --> mac1609_4.lowerLayerIn;
        phy80211p.upperControlOut --> mac1609_4.lowerControlIn;

        mac1609_4.upperControlIn <-- upperControlIn;
        mac1609_4.upperLayerIn <-- upperLayerIn;

        mac1609_4.upperLayerOut --> upperLayerOut;
        mac1609_4.upperControlOut --> upperControlOut;
}
