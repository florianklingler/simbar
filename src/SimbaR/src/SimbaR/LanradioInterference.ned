// This file is part of Lanradio and modified by Mario Franke.
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
// Mario Franke <research@m-franke.net>

package org.car2x.veins.subprojects.SimbaR;

@namespace(veins);

simple LanradioInterference
{
    parameters:
        @display("i=block/network2");

        bool use_interference = default(true);
        int baseport;
        string interferenceHostname = default("10.0.197.106");
        string bitrates;
        string wlan_ip;
        string freq; // in MHz
        string bandwidth; // in MHz
        string experimentName = default("StaticScenario");
        double interferenceMinInterval @unit(s) = default(0.002s);
        bool useInterferenceMinInterval = default(false);

        string SimInterferenceTxPower = default("2"); // in dBm
        bool SimInterferenceModifyMac = default(true);
}
