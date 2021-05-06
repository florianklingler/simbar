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

#ifndef __VEINS_LARA_LANRADIOMACINTERFACE_H
#define __VEINS_LARA_LANRADIOMACINTERFACE_H

#include <omnetpp.h>

using namespace omnetpp;

namespace veins {
class LanradioMacInterface {
public:
    virtual void sendMacPacketDirect(cMessage* msg) = 0;
    virtual void toApplicationLayer(cMessage* msg) = 0;
    virtual void setAsLanradioMac(bool isNewLanradioMac) = 0;

    virtual ~LanradioMacInterface() = default;
};

} // namespace veins

#endif
