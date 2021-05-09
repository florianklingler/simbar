#!/bin/bash
# This file is part of Lanradio.
#
# Lanradio is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# Lanradio is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with Lanradio.  If not, see <http://www.gnu.org/licenses/>.
#
# Authors:
# Florian Klingler <klingler@ccs-labs.org>
# Mario Franke <research@m-franke.net>

set -x
#set -e

if [ -z "$1" ]; then
	device="10.0.197.103"
else
	device="$1"
fi
ssh -o StrictHostKeyChecking=no root@${device} "iw phy ${phy} interface add mon0 type monitor"
ssh -o StrictHostKeyChecking=no root@${device} "ifconfig mon0 up"

ssh -o StrictHostKeyChecking=no root@${device} "tcpdump -U -i mon0 --immediate-mode -w -" | wireshark -k -l -i -
