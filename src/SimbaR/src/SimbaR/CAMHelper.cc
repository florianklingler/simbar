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
// Mario Franke <research@m-franke.net>

#include "SimbaR/CAMHelper.h"

namespace veins {

CAMHelper& CAMHelper::stationId(unsigned long newStationId)
{
    myStationID = newStationId;
    return *this;
}

CAMHelper& CAMHelper::lastSendingTime(int64_t lastSendingTime_ns)
{
    myLastSendingTimeNS = lastSendingTime_ns;
    return *this;
}

CAMHelper& CAMHelper::stationType(StationType newType)
{
    switch (newType) {
    case StationType::base_vehicle:
        myStationType = StationType_passengerCar;
        break;
    case StationType::rsu:
        myStationType = StationType_roadSideUnit;
        break;
    case StationType::unknown:
        myStationType = StationType_unknown;
        break;
    default:
        throw std::runtime_error("Unkown station type given to CAMHelper::stationType");
    }
    myHighFrequencyType = newType;
    return *this;
}

CAMHelper& CAMHelper::currentPosition(double latitude_deg, double longitude_deg, double altitude_m)
{
    // from degree to one tenth of a microdegree
    myLatitude = static_cast<Latitude_t>(latitude_deg * 10e6);
    myLongitude = static_cast<Longitude_t>(longitude_deg * 10e6);
    myAltitude = static_cast<AltitudeValue_t>(altitude_m * 10e2);
    return *this;
}

CAMHelper& CAMHelper::heading(double heading_deg)
{
    // convert to 0.1 degrees in int
    myHeading = int(heading_deg * 10) % 3600;
    return *this;
}

CAMHelper& CAMHelper::speed(double speed_mps, bool forward)
{
    // convert to centimeters per second
    mySpeed = int(speed_mps * 100);
    myDriveDirection = forward ? DriveDirection_forward : DriveDirection_backward;
    return *this;
}

CAMHelper& CAMHelper::vehicleSize(double length_m, double width_m)
{
    if (length_m > 0) {
        // convert to decimeters
        myVehicleLength = int(length_m * 10);
    }
    else {
        myVehicleLength = VehicleLengthValue_unavailable;
    }
    if (width_m > 0) {
        // convert to decimeters
        myVehicleWidth = int(width_m * 10);
    }
    else {
        myVehicleWidth = VehicleWidth_unavailable;
    }
    return *this;
}

CAMHelper& CAMHelper::acceleration(double acc_mpss)
{
    // convert to 0.1 m/s^2
    myLongitudinalAcceleration = int(acc_mpss * 10);
    return *this;
}

CAMHelper& CAMHelper::curvature(double reciprocalRadius_m)
{
    // convert so that a 1m reciprocal radius equals a value of 30 000
    myCurvature = int(reciprocalRadius_m * 30000);
    return *this;
}

CAMHelper& CAMHelper::yawRate(double yawRate_degps)
{
    // convert to 0.01 degree per second
    myYawRate = int(yawRate_degps / 100);
    return *this;
}

CAM_t CAMHelper::build() const
{
    CAM_t cam;
    memset(&cam, 0, sizeof(CAM_t)); // make sure cam is initialize to all zeros

    // header
    auto& header = cam.header;
    // static configuration of the CAM header
    header.messageID = messageID_cam;
    header.protocolVersion = 1; // TODO: check if this is correct
    // configured part of the CAM header
    header.stationID = myStationID;

    // camGenerationDeltaTime
    cam.cam.generationDeltaTime = buildGenerationDeltaTime();

    // basic container
    auto& basicContainer = cam.cam.camParameters.basicContainer;
    basicContainer.stationType = myStationType;
    basicContainer.referencePosition = buildReferencePosition();

    // high frequency container
    cam.cam.camParameters.highFrequencyContainer = buildHighFrequencyContainer();

    // finish
    return cam;
}

GenerationDeltaTime_t CAMHelper::buildGenerationDeltaTime() const
{
    if (myLastSendingTimeNS == 0)
        return 0;

    auto now = std::chrono::high_resolution_clock::now().time_since_epoch();
    int64_t nowNS = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
    return (nowNS - myLastSendingTimeNS) / (1000000);
}

ReferencePosition_t CAMHelper::buildReferencePosition() const
{
    ReferencePosition_t referencePosition;
    memset(&referencePosition, 0, sizeof(ReferencePosition_t)); // make sure cam is initialize to all zeros
    referencePosition.latitude = myLatitude;
    referencePosition.longitude = myLongitude;
    referencePosition.altitude.altitudeValue = myAltitude;
    // MISSING: altitude confidence
    referencePosition.altitude.altitudeConfidence = AltitudeConfidence_unavailable;
    return referencePosition;
}

HighFrequencyContainer_t CAMHelper::buildHighFrequencyContainer() const
{
    HighFrequencyContainer_t result;
    memset(&result, 0, sizeof(HighFrequencyContainer_t)); // make sure cam is initialize to all zeros
    if (myHighFrequencyType == StationType::rsu) {
        result.present = HighFrequencyContainer_PR_rsuContainerHighFrequency;
        return result;
    }

    result.present = HighFrequencyContainer_PR_basicVehicleContainerHighFrequency;
    auto& container = result.choice.basicVehicleContainerHighFrequency;

    container.heading.headingValue = myHeading;
    container.heading.headingConfidence = HeadingConfidence_unavailable;
    container.speed.speedValue = mySpeed;
    container.speed.speedConfidence = SpeedConfidence_unavailable;
    container.driveDirection = myDriveDirection;
    container.vehicleLength.vehicleLengthValue = myVehicleLength;
    container.vehicleLength.vehicleLengthConfidenceIndication = VehicleLengthConfidenceIndication_unavailable;
    container.vehicleWidth = myVehicleWidth;
    container.longitudinalAcceleration.longitudinalAccelerationValue = myLongitudinalAcceleration;
    container.longitudinalAcceleration.longitudinalAccelerationConfidence = AccelerationConfidence_unavailable;
    container.curvature.curvatureValue = myCurvature;
    container.curvature.curvatureConfidence = CurvatureConfidence_unavailable;
    container.curvatureCalculationMode = CurvatureCalculationMode_unavailable;
    container.yawRate.yawRateValue = myYawRate;
    container.yawRate.yawRateConfidence = YawRateConfidence_unavailable;

    return result;
}

} // namespace veins
