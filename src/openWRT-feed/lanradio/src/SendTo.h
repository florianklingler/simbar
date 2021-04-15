// This file is part of OpenC2X.
//
// OpenC2X is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// OpenC2X is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with OpenC2X.  If not, see <http://www.gnu.org/licenses/>.
//
// Authors:
// Sven Laux <slaux@mail.uni-paderborn.de>
// Gurjashan Singh Pannu <gurjashan.pannu@ccs-labs.org>
// Stefan Schneider <stefan.schneider@ccs-labs.org>
// Jan Tiemann <janhentie@web.de>


#ifndef SENDTO_H_
#define SENDTO_H_

/**
 * @addtogroup dccHardware
 * @{
 */

// #include <common/utility/LoggingUtility.h>
// #include <common/utility/Constants.h>
#include <string.h>
#include "wlan.pb.h"

/**
 * Sends messages to anything to be broadcasted on the MAC layer. 
 *
 *
 * @nonStandard serialization is not standard conform. Uses protobuffer instead.
 */
class SendTo{
public:

    /**
     * Sends standard compliant packet to the hardware queue with the corresponding priority.
     * @param msg
     * @param priority
     * @param type
     */
    // virtual void sendWithGeoNet(dataPackage::DATA& data, int priority) = 0;
    virtual void sendWithGeoNet(wlanPackage::WLAN& msg, int priority) = 0;
    
    std::string mOwnMac;


};

/**
 * @}
 */

#endif /* SENDTO_H_ */
