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

@namespace(veins);

simple LanradioCommunication
{
    parameters:
        @display("i=block/network2");

        bool use_lanradio = default(false);
        int baseport;
        string simIntHostname = default("10.0.197.103");
        string dutHostname = default("10.0.197.104");
        string wlan_interface;
        string bitrates;
        string wlan_ip;
        string freq; // in MHz
        string bandwidth; // in MHz
        bool lara_debug = default(false);
        bool use_udp = default(true);
        int numEvaluationBytes;
        string experimentName = default("StaticScenario");

        string SimInterfaceTxPower = default("2"); // in dBm
        string DutTxPower = default("20"); // in dBm
        bool SimInterfaceModifyMac = default(true);
}
