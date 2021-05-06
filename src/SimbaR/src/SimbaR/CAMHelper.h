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
// Dominik S. Buse <buse@ccs-labs.org>

#ifndef __VEINS_LARA_CAMHELPER_H
#define __VEINS_LARA_CAMHELPER_H

#include "CAM.h"

#include <stdexcept>
#include <chrono>

namespace veins {

/*
 * Helper class for generating ASN.1-based CAM messages
 *
 * Features:
 *   header:
 *     - stationID
 *     - messageID (alwas CAM)
 *     - protocolVersion (always 1)
 *   cam:
 *     generationDeltaTime
 *     camParameters:
 *       basicContainer:
 *         - station type (passengar car or RSU)
 *         - referencePosition
 *         - stationType
 *       highFrequencyContainer:
 *         rsu:
 *           - presence
 *         baseVehicle:
 *           - heading
 *           - speed
 *           - driveDirection
 *           - vehicleLength
 *           - vehicleWidth
 *           - longitudinalAcceleration
 *           - curvature
 *           - yawRate
 *       lowFrequencyContainer:
 *
 * Missing:
 *  - confidence makers
 *  - calculation modes
 *  - values greater than the allowed maximum
 *
 * Notes:
 *  - it is currenty not possible to unset configuration values
 *    or set them back to unavailable.
 */
class CAMHelper {
public:
    enum class StationType {
        unknown = 0,
        base_vehicle = 1,
        rsu = 2,
    };

public:
    /*
     * Set the numerical identifier for this station (may change over time and/or space)
     */
    CAMHelper& stationId(unsigned long newStationId);

    /*
     * Set sending time of previous CAM from this station in nanoseconds.
     */
    CAMHelper& lastSendingTime(int64_t lastSendingTime_ns);

    /*
     * Set type of sending station (see StationType for available types)
     */
    CAMHelper& stationType(StationType newType);

    /*
     * Set geo-coordinates in degrees (latitude/longitude) or meters (altitude).
     */
    CAMHelper& currentPosition(double latitude_deg, double longutude_deg, double altitude_m);

    /*
     * Set the heading of the vehicle in degrees with 0 and 360 meaning north
     */
    CAMHelper& heading(double heading_deg);

    /*
     * Set the driving speed of the vehicles in meters per second and whether it is forward or backward
     */
    CAMHelper& speed(double speed_mps, bool forward = true);

    /*
     * Set the vehicle dimensions in meters (<=0 meaning size unavailable)
     */
    CAMHelper& vehicleSize(double length_m = -1, double width_m = -1);

    /*
     * Set the vehicle's longitudinal acceleration in meters per second^2, negative numbers meaning breaking.
     */
    CAMHelper& acceleration(double acc_mpss);

    /*
     * Set the reciprocal radius of the curve driven by the vehicle in meters,
     * 0 means driving straight, negative means turning to the right, positiv means turning to the left.
     */
    CAMHelper& curvature(double reciprocalRadius_m);

    /*
     * Set the vehicle's yaw rate in degrees per second
     */
    CAMHelper& yawRate(double yawRate_degps);

    /*
     * Build an ASN.1-conform CAM object out of the configuration of this object.
     */
    CAM_t build() const;

private:
    // header values
    StationID_t myStationID;

    // last message's data
    GenerationDeltaTime_t myLastSendingTimeNS = 0;

    // basic container values
    StationType_t myStationType = StationType_unknown;

    // geo-coordinates
    // globe position in 0.1 micro-degrees or 10e-7 * degrees
    Latitude_t myLatitude = Latitude_unavailable;
    Longitude_t myLongitude = Longitude_unavailable;
    // altitude in centimeters
    AltitudeValue_t myAltitude = AltitudeValue_unavailable;

    // high frequency container config
    StationType myHighFrequencyType = StationType::unknown;
    // high frequency container values for basic vehicles
    HeadingValue_t myHeading = HeadingValue_unavailable;
    SpeedValue_t mySpeed = SpeedValue_unavailable;
    DriveDirection_t myDriveDirection = DriveDirection_unavailable;
    VehicleLengthValue_t myVehicleLength = VehicleLengthValue_unavailable;
    VehicleWidth_t myVehicleWidth = VehicleWidth_unavailable;
    LongitudinalAccelerationValue_t myLongitudinalAcceleration = LongitudinalAccelerationValue_unavailable;
    CurvatureValue_t myCurvature = CurvatureValue_unavailable;
    YawRateValue_t myYawRate = YawRateValue_unavailable;

private:
    GenerationDeltaTime_t buildGenerationDeltaTime() const;
    ReferencePosition_t buildReferencePosition() const;
    HighFrequencyContainer_t buildHighFrequencyContainer() const;
};

} // namespace veins

#endif
