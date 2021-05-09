# This file is part of SimbaR.
#
# SimbaR is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# SimbaR is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with SimbaR. If not, see <http://www.gnu.org/licenses/>.
#
# Authors:
# Mario Franke <research@m-franke.net>

#!/bin/bash

#kill all runing processes of this script
trap killgroup SIGINT
killgroup(){
    echo "SIGINT, killing..."
    kill 0
}

SIMBAR_ROOT=../..

apuSIMINT="10.0.197.103"
apuDUT="10.0.197.104"
ALIX5="10.0.197.105"
apuINTERFERENCE="10.0.197.106"

# set pfifo queue for tc
sudo tc qdisc replace dev enp0s3 root handle 1: pfifo

# test whether apuSIMINT is online
while ! ping -c 1 $apuSIMINT > /dev/null
do
    echo "apuSIMINT offline, waiting..."
    sleep 1s
done
echo "apuSIMINT is online!"

# test whether apuDUT is online
while ! ping -c 1 $apuDUT > /dev/null
do
    echo "apuDUT offline, waiting..."
    sleep 1s
done
echo "apuDUT is online!"

# test whether ALIX5 is online
while ! ping -c 1 $ALIX5 > /dev/null
do
    echo "ALIX5 offline, waiting..."
    sleep 1s
done
echo "ALIX5 is online!"

# test whether apuINTERFERENCE is online
while ! ping -c 1 $apuINTERFERENCE > /dev/null
do
    echo "apuINTERFERENCE offline, waiting..."
    sleep 1s
done
echo "apuINTERFERENCE is online!"
# to be sure everything runs on the alix boxes.
sleep 1s

echo "Configure wlan interfaces"
# configure lanradio simInt/apuSIMINT transmit by packet injection interface
ssh root@$apuSIMINT iw reg set DE
ssh root@$apuSIMINT ip link set wlan1 down
ssh root@$apuSIMINT iw dev wlan1 set type monitor
ssh root@$apuSIMINT iw phy phy1 set antenna 1 1
ssh root@$apuSIMINT ip link set wlan1 up
ssh root@$apuSIMINT iw dev wlan1 set freq 5900 10Mhz

# configure lanradio simInt/apuSIMINT receive interface
ssh root@$apuSIMINT ip link set wlan0 down
ssh root@$apuSIMINT iw dev wlan0 set type ocb
ssh root@$apuSIMINT iw phy phy0 set antenna 1 1
ssh root@$apuSIMINT ip link set wlan0 up
ssh root@$apuSIMINT iw dev wlan0 set bitrates legacy-5 12
ssh root@$apuSIMINT iw dev wlan0 ocb join 5900 10Mhz
ssh root@$apuSIMINT ifconfig wlan0 192.168.55.2
ssh root@$apuSIMINT iw dev wlan0 set txpower fixed 300

# configure lanradio interface apuDUT
ssh root@$apuDUT iw reg set DE
ssh root@$apuDUT ip link set wlan1 down
ssh root@$apuDUT iw dev wlan1 set type monitor
ssh root@$apuDUT iw phy phy1 set antenna 1 1
ssh root@$apuDUT ip link set wlan1 up
ssh root@$apuDUT iw dev wlan1 set freq 5900 10Mhz

# configure chanload interface alix5
ssh root@$ALIX5 iw reg set DE
ssh root@$ALIX5 ip link set wlan1 down
ssh root@$ALIX5 iw dev wlan1 set type ocb
ssh root@$ALIX5 iw phy phy1 set antenna 1 1
ssh root@$ALIX5 ip link set wlan1 up
ssh root@$ALIX5 iw dev wlan1 set bitrates legacy-5 12
ssh root@$ALIX5 iw dev wlan1 ocb join 5900 10MHz
ssh root@$ALIX5 ifconfig wlan1 192.168.55.1
ssh root@$ALIX5 iw dev wlan1 set txpower fixed 300

# configure chanload interface apuINTERFERENCE
ssh root@$apuINTERFERENCE iw reg set DE
ssh root@$apuINTERFERENCE ip link set wlan1 down
ssh root@$apuINTERFERENCE iw dev wlan1 set type monitor
ssh root@$apuINTERFERENCE iw phy phy1 set antenna 1 1
ssh root@$apuINTERFERENCE ip link set wlan1 up
ssh root@$apuINTERFERENCE iw dev wlan1 set freq 5900 10MHz

# configure ssh on alix boxes
touch /home/franke/.ssh/authorized_keys
ssh root@$apuSIMINT "ssh-keygen -t rsa -N \"\" -f /tmp/id_rsa"
ssh root@$apuDUT "ssh-keygen -t rsa -N \"\" -f /tmp/id_rsa"
ssh root@$ALIX5 "ssh-keygen -t rsa -N \"\" -f /tmp/id_rsa"
ssh root@$apuSIMINT cat /tmp/id_rsa.pub >> /home/$USER/.ssh/authorized_keys
ssh root@$apuDUT cat /tmp/id_rsa.pub >> /home/$USER/.ssh/authorized_keys
ssh root@$ALIX5 cat /tmp/id_rsa.pub >> /home/$USER/.ssh/authorized_keys

# start simulation
echo "Start simulating."
cd $SIMBAR_ROOT/src/SimbaR/examples/StaticScenario
touch runs.log
./generateRunsFile_ini.pl omnetpp.ini StaticScenario_DisableCS_10Vehicles_400ByteOnAir_useCaptureEffect > runs.txt
./runmaker4.py -j 1 -l runs.log -n 100 runs.txt
