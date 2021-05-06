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
import org.car2x.veins.base.modules.IBaseApplLayer;


@namespace(veins);

simple SimpleApp like IBaseApplLayer
{
    parameters:
        int headerLength = default(88bit) @unit(bit); //header length of the application
        int beaconLengthBits = default(256bit) @unit(bit); //the length of a beacon packet
        int beaconPriority = default(3); //the access category of the beacon messages

        bool app_debug @unit(bool) = default(false);

        bool sendBeacons @unit(bool) = default(false);
        double beacon_interval @unit(s) = default(1s); // interval between two beacons
        double start_beaconing_at @unit(s) = default(0s); // when to start beaconing

    gates:
        input lowerLayerIn; // from mac layer
        output lowerLayerOut; // to mac layer
        input lowerControlIn;
        output lowerControlOut;
}
