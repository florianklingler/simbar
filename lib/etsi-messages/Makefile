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
# Dominik S. Buse <buse@ccs-labs.org>

# ASN.1 Compiler - make sure this is the right version
ASN1C ?= ../../asn1c/asn1c/asn1c

# Generated header and implementation source files
ASN_MODULE_SOURCES=	\
	CAM.c	\
	CoopAwareness.c	\
	CamParameters.c	\
	HighFrequencyContainer.c	\
	LowFrequencyContainer.c	\
	SpecialVehicleContainer.c	\
	BasicContainer.c	\
	BasicVehicleContainerHighFrequency.c	\
	BasicVehicleContainerLowFrequency.c	\
	PublicTransportContainer.c	\
	SpecialTransportContainer.c	\
	DangerousGoodsContainer.c	\
	RoadWorksContainerBasic.c	\
	RescueContainer.c	\
	EmergencyContainer.c	\
	SafetyCarContainer.c	\
	RSUContainerHighFrequency.c	\
	GenerationDeltaTime.c	\
	DENM.c	\
	DecentralizedEnvironmentalNotificationMessage.c	\
	ManagementContainer.c	\
	SituationContainer.c	\
	LocationContainer.c	\
	ImpactReductionContainer.c	\
	RoadWorksContainerExtended.c	\
	StationaryVehicleContainer.c	\
	AlacarteContainer.c	\
	Termination.c	\
	ReferenceDenms.c	\
	MapData.c	\
	SPAT.c	\
	AdvisorySpeed.c	\
	AdvisorySpeedList.c	\
	ComputedLane.c	\
	ConnectingLane.c	\
	Connection.c	\
	ConnectionManeuverAssist.c	\
	ConnectsToList.c	\
	DataParameters.c	\
	EnabledLaneList.c	\
	GenericLane.c	\
	IntersectionGeometry.c	\
	IntersectionGeometryList.c	\
	IntersectionReferenceID.c	\
	IntersectionState.c	\
	IntersectionStateList.c	\
	LaneAttributes.c	\
	LaneDataAttribute.c	\
	LaneDataAttributeList.c	\
	LaneList.c	\
	LaneTypeAttributes.c	\
	ManeuverAssistList.c	\
	MovementEvent.c	\
	MovementEventList.c	\
	MovementList.c	\
	MovementState.c	\
	Node.c	\
	NodeAttributeList.c	\
	NodeAttributeSet.c	\
	NodeList2.c	\
	Node-LLmD-64b.c	\
	NodeOffsetPoint.c	\
	NodeSet.c	\
	Node-XY-20b.c	\
	Node-XY-22b.c	\
	Node-XY-24b.c	\
	Node-XY-26b.c	\
	Node-XY-28b.c	\
	Node-XY-32b.c	\
	OverlayLaneList.c	\
	Position3D-2.c	\
	PreemptPriorityList.c	\
	RegionalAdvisorySpeed.c	\
	RegionalComputedLane.c	\
	RegionalConnectionManeuverAssist.c	\
	RegionalGenericLane.c	\
	RegionalIntersection.c	\
	RegionalIntersectionState.c	\
	RegionalLaneAttributes.c	\
	RegionalLaneDataAttribute.c	\
	RegionalMapData.c	\
	RegionalMovementEvent.c	\
	RegionalMovementState.c	\
	RegionalNodeAttribute.c	\
	RegionalNodeAttributeList.c	\
	RegionalNodeOffsetPoint.c	\
	RegionalPosition3D.c	\
	RegionalRestrictionUserType.c	\
	RegionalRoadSegment.c	\
	RegionalSignalControlZone.c	\
	RegionalSPAT.c	\
	RegulatorySpeedLimit.c	\
	RestrictionClassAssignment.c	\
	RestrictionClassList.c	\
	RestrictionUserType.c	\
	RestrictionUserTypeList.c	\
	RoadLaneSetList.c	\
	RoadSegment.c	\
	RoadSegmentList.c	\
	RoadSegmentReferenceID.c	\
	SegmentAttributeList.c	\
	SpeedLimitList.c	\
	TimeChangeDetails.c	\
	AdvisorySpeedType.c	\
	AllowedManeuvers.c	\
	Angle.c	\
	ApproachID.c	\
	DeltaAngle.c	\
	DescriptiveName.c	\
	DrivenLineOffsetLg.c	\
	DrivenLineOffsetSm.c	\
	DSecond2.c	\
	DSRCmsgID2.c	\
	DSRCmsgSubID.c	\
	Elevation2.c	\
	IntersectionID.c	\
	IntersectionStatusObject.c	\
	LaneAttributes-Barrier.c	\
	LaneAttributes-Bike.c	\
	LaneAttributes-Crosswalk.c	\
	LaneAttributes-Parking.c	\
	LaneAttributes-Sidewalk.c	\
	LaneAttributes-Striping.c	\
	LaneAttributes-TrackedVehicle.c	\
	LaneAttributes-Vehicle.c	\
	LaneConnectionID.c	\
	LaneDirection.c	\
	LaneID.c	\
	LaneSharing.c	\
	LaneWidth.c	\
	LayerID.c	\
	LayerType.c	\
	MergeDivergeNodeAngle.c	\
	MinuteOfTheYear.c	\
	MovementPhaseState.c	\
	MsgCount.c	\
	MsgCRC.c	\
	NodeAttribute.c	\
	Offset-B10.c	\
	Offset-B11.c	\
	Offset-B12.c	\
	Offset-B13.c	\
	Offset-B14.c	\
	Offset-B16.c	\
	PedestrianBicycleDetect.c	\
	PrioritizationResponseStatus.c	\
	RestrictionAppliesTo.c	\
	RestrictionClassID.c	\
	RoadRegulatorID.c	\
	RoadSegmentID.c	\
	RoadwayCrownAngle.c	\
	Scale-B12.c	\
	SegmentAttribute.c	\
	SignalControlState.c	\
	SignalGroupID.c	\
	SpeedAdvice.c	\
	SpeedLimitType.c	\
	TimeIntervalConfidence.c	\
	TimeMark.c	\
	Velocity.c	\
	WaitOnStopline.c	\
	ZoneLength.c	\
	Reg-AdvisorySpeed.c	\
	Reg-ComputedLane.c	\
	Reg-GenericLane.c	\
	Reg-Intersection.c	\
	Reg-LaneAttributes.c	\
	Reg-LaneDataAttribute.c	\
	Reg-MovementEvent.c	\
	Reg-MovementState.c	\
	Reg-NodeAttribute.c	\
	Reg-NodeOffsetPoint.c	\
	Reg-RoadSegment.c	\
	Reg-SignalControlZone.c	\
	Reg-SPAT.c	\
	ItsPduHeader.c	\
	StationID.c	\
	ReferencePosition.c	\
	DeltaReferencePosition.c	\
	Longitude.c	\
	Latitude.c	\
	Altitude.c	\
	AltitudeValue.c	\
	AltitudeConfidence.c	\
	DeltaLongitude.c	\
	DeltaLatitude.c	\
	DeltaAltitude.c	\
	PosConfidenceEllipse.c	\
	PathPoint.c	\
	PathDeltaTime.c	\
	PtActivation.c	\
	PtActivationType.c	\
	PtActivationData.c	\
	AccelerationControl.c	\
	SemiAxisLength.c	\
	CauseCode.c	\
	CauseCodeType.c	\
	SubCauseCodeType.c	\
	TrafficConditionSubCauseCode.c	\
	AccidentSubCauseCode.c	\
	RoadworksSubCauseCode.c	\
	HumanPresenceOnTheRoadSubCauseCode.c	\
	WrongWayDrivingSubCauseCode.c	\
	AdverseWeatherCondition-ExtremeWeatherConditionSubCauseCode.c	\
	AdverseWeatherCondition-AdhesionSubCauseCode.c	\
	AdverseWeatherCondition-VisibilitySubCauseCode.c	\
	AdverseWeatherCondition-PrecipitationSubCauseCode.c	\
	SlowVehicleSubCauseCode.c	\
	StationaryVehicleSubCauseCode.c	\
	HumanProblemSubCauseCode.c	\
	EmergencyVehicleApproachingSubCauseCode.c	\
	HazardousLocation-DangerousCurveSubCauseCode.c	\
	HazardousLocation-SurfaceConditionSubCauseCode.c	\
	HazardousLocation-ObstacleOnTheRoadSubCauseCode.c	\
	HazardousLocation-AnimalOnTheRoadSubCauseCode.c	\
	CollisionRiskSubCauseCode.c	\
	SignalViolationSubCauseCode.c	\
	RescueAndRecoveryWorkInProgressSubCauseCode.c	\
	DangerousEndOfQueueSubCauseCode.c	\
	DangerousSituationSubCauseCode.c	\
	VehicleBreakdownSubCauseCode.c	\
	PostCrashSubCauseCode.c	\
	Curvature.c	\
	CurvatureValue.c	\
	CurvatureConfidence.c	\
	CurvatureCalculationMode.c	\
	Heading.c	\
	HeadingValue.c	\
	HeadingConfidence.c	\
	LanePosition.c	\
	ClosedLanes.c	\
	HardShoulderStatus.c	\
	DrivingLaneStatus.c	\
	PerformanceClass.c	\
	SpeedValue.c	\
	SpeedConfidence.c	\
	VehicleMass.c	\
	Speed.c	\
	DriveDirection.c	\
	EmbarkationStatus.c	\
	LongitudinalAcceleration.c	\
	LongitudinalAccelerationValue.c	\
	AccelerationConfidence.c	\
	LateralAcceleration.c	\
	LateralAccelerationValue.c	\
	VerticalAcceleration.c	\
	VerticalAccelerationValue.c	\
	StationType.c	\
	ExteriorLights.c	\
	DangerousGoodsBasic.c	\
	DangerousGoodsExtended.c	\
	SpecialTransportType.c	\
	LightBarSirenInUse.c	\
	HeightLonCarr.c	\
	PosLonCarr.c	\
	PosPillar.c	\
	PosCentMass.c	\
	RequestResponseIndication.c	\
	SpeedLimit.c	\
	StationarySince.c	\
	Temperature.c	\
	TrafficRule.c	\
	WheelBaseVehicle.c	\
	TurningRadius.c	\
	PosFrontAx.c	\
	PositionOfOccupants.c	\
	PositioningSolutionType.c	\
	VehicleIdentification.c	\
	WMInumber.c	\
	VDS.c	\
	EnergyStorageType.c	\
	VehicleLength.c	\
	VehicleLengthValue.c	\
	VehicleLengthConfidenceIndication.c	\
	VehicleWidth.c	\
	PathHistory.c	\
	EmergencyPriority.c	\
	InformationQuality.c	\
	RoadType.c	\
	SteeringWheelAngle.c	\
	SteeringWheelAngleValue.c	\
	SteeringWheelAngleConfidence.c	\
	TimestampIts.c	\
	VehicleRole.c	\
	YawRate.c	\
	YawRateValue.c	\
	YawRateConfidence.c	\
	ProtectedZoneType.c	\
	RelevanceDistance.c	\
	RelevanceTrafficDirection.c	\
	TransmissionInterval.c	\
	ValidityDuration.c	\
	ActionID.c	\
	ItineraryPath.c	\
	ProtectedCommunicationZone.c	\
	Traces.c	\
	NumberOfOccupants.c	\
	SequenceNumber.c	\
	PositionOfPillars.c	\
	RestrictedTypes.c	\
	EventHistory.c	\
	EventPoint.c	\
	ProtectedCommunicationZonesRSU.c	\
	CenDsrcTollingZone.c	\
	ProtectedZoneRadius.c	\
	ProtectedZoneID.c	\
	CenDsrcTollingZoneID.c	\
	MAP-PDU.c	\
	SPAT-PDU.c	\
	EmissionType.c	\
	PrioritizationResponse.c	\
	PrioritizationResponseList.c	\
	Reg-ConnectionManeuverAssist.c	\
	Reg-IntersectionState.c	\
	Reg-MapData.c	\
	Reg-Position3D.c	\
	Reg-RestrictionUserType.c	\
	SignalHeadLocation.c	\
	SignalHeadLocationList.c	\
	VehicleToLanePosition.c	\
	VehicleToLanePositionList.c

ASN_MODULE_HEADERS=	\
	CAM.h	\
	CoopAwareness.h	\
	CamParameters.h	\
	HighFrequencyContainer.h	\
	LowFrequencyContainer.h	\
	SpecialVehicleContainer.h	\
	BasicContainer.h	\
	BasicVehicleContainerHighFrequency.h	\
	BasicVehicleContainerLowFrequency.h	\
	PublicTransportContainer.h	\
	SpecialTransportContainer.h	\
	DangerousGoodsContainer.h	\
	RoadWorksContainerBasic.h	\
	RescueContainer.h	\
	EmergencyContainer.h	\
	SafetyCarContainer.h	\
	RSUContainerHighFrequency.h	\
	GenerationDeltaTime.h	\
	DENM.h	\
	DecentralizedEnvironmentalNotificationMessage.h	\
	ManagementContainer.h	\
	SituationContainer.h	\
	LocationContainer.h	\
	ImpactReductionContainer.h	\
	RoadWorksContainerExtended.h	\
	StationaryVehicleContainer.h	\
	AlacarteContainer.h	\
	Termination.h	\
	ReferenceDenms.h	\
	MapData.h	\
	SPAT.h	\
	AdvisorySpeed.h	\
	AdvisorySpeedList.h	\
	ComputedLane.h	\
	ConnectingLane.h	\
	Connection.h	\
	ConnectionManeuverAssist.h	\
	ConnectsToList.h	\
	DataParameters.h	\
	EnabledLaneList.h	\
	GenericLane.h	\
	IntersectionGeometry.h	\
	IntersectionGeometryList.h	\
	IntersectionReferenceID.h	\
	IntersectionState.h	\
	IntersectionStateList.h	\
	LaneAttributes.h	\
	LaneDataAttribute.h	\
	LaneDataAttributeList.h	\
	LaneList.h	\
	LaneTypeAttributes.h	\
	ManeuverAssistList.h	\
	MovementEvent.h	\
	MovementEventList.h	\
	MovementList.h	\
	MovementState.h	\
	Node.h	\
	NodeAttributeList.h	\
	NodeAttributeSet.h	\
	NodeList2.h	\
	Node-LLmD-64b.h	\
	NodeOffsetPoint.h	\
	NodeSet.h	\
	Node-XY-20b.h	\
	Node-XY-22b.h	\
	Node-XY-24b.h	\
	Node-XY-26b.h	\
	Node-XY-28b.h	\
	Node-XY-32b.h	\
	OverlayLaneList.h	\
	Position3D-2.h	\
	PreemptPriorityList.h	\
	RegionalAdvisorySpeed.h	\
	RegionalComputedLane.h	\
	RegionalConnectionManeuverAssist.h	\
	RegionalGenericLane.h	\
	RegionalIntersection.h	\
	RegionalIntersectionState.h	\
	RegionalLaneAttributes.h	\
	RegionalLaneDataAttribute.h	\
	RegionalMapData.h	\
	RegionalMovementEvent.h	\
	RegionalMovementState.h	\
	RegionalNodeAttribute.h	\
	RegionalNodeAttributeList.h	\
	RegionalNodeOffsetPoint.h	\
	RegionalPosition3D.h	\
	RegionalRestrictionUserType.h	\
	RegionalRoadSegment.h	\
	RegionalSignalControlZone.h	\
	RegionalSPAT.h	\
	RegulatorySpeedLimit.h	\
	RestrictionClassAssignment.h	\
	RestrictionClassList.h	\
	RestrictionUserType.h	\
	RestrictionUserTypeList.h	\
	RoadLaneSetList.h	\
	RoadSegment.h	\
	RoadSegmentList.h	\
	RoadSegmentReferenceID.h	\
	SegmentAttributeList.h	\
	SpeedLimitList.h	\
	TimeChangeDetails.h	\
	AdvisorySpeedType.h	\
	AllowedManeuvers.h	\
	Angle.h	\
	ApproachID.h	\
	DeltaAngle.h	\
	DescriptiveName.h	\
	DrivenLineOffsetLg.h	\
	DrivenLineOffsetSm.h	\
	DSecond2.h	\
	DSRCmsgID2.h	\
	DSRCmsgSubID.h	\
	Elevation2.h	\
	IntersectionID.h	\
	IntersectionStatusObject.h	\
	LaneAttributes-Barrier.h	\
	LaneAttributes-Bike.h	\
	LaneAttributes-Crosswalk.h	\
	LaneAttributes-Parking.h	\
	LaneAttributes-Sidewalk.h	\
	LaneAttributes-Striping.h	\
	LaneAttributes-TrackedVehicle.h	\
	LaneAttributes-Vehicle.h	\
	LaneConnectionID.h	\
	LaneDirection.h	\
	LaneID.h	\
	LaneSharing.h	\
	LaneWidth.h	\
	LayerID.h	\
	LayerType.h	\
	MergeDivergeNodeAngle.h	\
	MinuteOfTheYear.h	\
	MovementPhaseState.h	\
	MsgCount.h	\
	MsgCRC.h	\
	NodeAttribute.h	\
	Offset-B10.h	\
	Offset-B11.h	\
	Offset-B12.h	\
	Offset-B13.h	\
	Offset-B14.h	\
	Offset-B16.h	\
	PedestrianBicycleDetect.h	\
	PrioritizationResponseStatus.h	\
	RestrictionAppliesTo.h	\
	RestrictionClassID.h	\
	RoadRegulatorID.h	\
	RoadSegmentID.h	\
	RoadwayCrownAngle.h	\
	Scale-B12.h	\
	SegmentAttribute.h	\
	SignalControlState.h	\
	SignalGroupID.h	\
	SpeedAdvice.h	\
	SpeedLimitType.h	\
	TimeIntervalConfidence.h	\
	TimeMark.h	\
	Velocity.h	\
	WaitOnStopline.h	\
	ZoneLength.h	\
	Reg-AdvisorySpeed.h	\
	Reg-ComputedLane.h	\
	Reg-GenericLane.h	\
	Reg-Intersection.h	\
	Reg-LaneAttributes.h	\
	Reg-LaneDataAttribute.h	\
	Reg-MovementEvent.h	\
	Reg-MovementState.h	\
	Reg-NodeAttribute.h	\
	Reg-NodeOffsetPoint.h	\
	Reg-RoadSegment.h	\
	Reg-SignalControlZone.h	\
	Reg-SPAT.h	\
	ItsPduHeader.h	\
	StationID.h	\
	ReferencePosition.h	\
	DeltaReferencePosition.h	\
	Longitude.h	\
	Latitude.h	\
	Altitude.h	\
	AltitudeValue.h	\
	AltitudeConfidence.h	\
	DeltaLongitude.h	\
	DeltaLatitude.h	\
	DeltaAltitude.h	\
	PosConfidenceEllipse.h	\
	PathPoint.h	\
	PathDeltaTime.h	\
	PtActivation.h	\
	PtActivationType.h	\
	PtActivationData.h	\
	AccelerationControl.h	\
	SemiAxisLength.h	\
	CauseCode.h	\
	CauseCodeType.h	\
	SubCauseCodeType.h	\
	TrafficConditionSubCauseCode.h	\
	AccidentSubCauseCode.h	\
	RoadworksSubCauseCode.h	\
	HumanPresenceOnTheRoadSubCauseCode.h	\
	WrongWayDrivingSubCauseCode.h	\
	AdverseWeatherCondition-ExtremeWeatherConditionSubCauseCode.h	\
	AdverseWeatherCondition-AdhesionSubCauseCode.h	\
	AdverseWeatherCondition-VisibilitySubCauseCode.h	\
	AdverseWeatherCondition-PrecipitationSubCauseCode.h	\
	SlowVehicleSubCauseCode.h	\
	StationaryVehicleSubCauseCode.h	\
	HumanProblemSubCauseCode.h	\
	EmergencyVehicleApproachingSubCauseCode.h	\
	HazardousLocation-DangerousCurveSubCauseCode.h	\
	HazardousLocation-SurfaceConditionSubCauseCode.h	\
	HazardousLocation-ObstacleOnTheRoadSubCauseCode.h	\
	HazardousLocation-AnimalOnTheRoadSubCauseCode.h	\
	CollisionRiskSubCauseCode.h	\
	SignalViolationSubCauseCode.h	\
	RescueAndRecoveryWorkInProgressSubCauseCode.h	\
	DangerousEndOfQueueSubCauseCode.h	\
	DangerousSituationSubCauseCode.h	\
	VehicleBreakdownSubCauseCode.h	\
	PostCrashSubCauseCode.h	\
	Curvature.h	\
	CurvatureValue.h	\
	CurvatureConfidence.h	\
	CurvatureCalculationMode.h	\
	Heading.h	\
	HeadingValue.h	\
	HeadingConfidence.h	\
	LanePosition.h	\
	ClosedLanes.h	\
	HardShoulderStatus.h	\
	DrivingLaneStatus.h	\
	PerformanceClass.h	\
	SpeedValue.h	\
	SpeedConfidence.h	\
	VehicleMass.h	\
	Speed.h	\
	DriveDirection.h	\
	EmbarkationStatus.h	\
	LongitudinalAcceleration.h	\
	LongitudinalAccelerationValue.h	\
	AccelerationConfidence.h	\
	LateralAcceleration.h	\
	LateralAccelerationValue.h	\
	VerticalAcceleration.h	\
	VerticalAccelerationValue.h	\
	StationType.h	\
	ExteriorLights.h	\
	DangerousGoodsBasic.h	\
	DangerousGoodsExtended.h	\
	SpecialTransportType.h	\
	LightBarSirenInUse.h	\
	HeightLonCarr.h	\
	PosLonCarr.h	\
	PosPillar.h	\
	PosCentMass.h	\
	RequestResponseIndication.h	\
	SpeedLimit.h	\
	StationarySince.h	\
	Temperature.h	\
	TrafficRule.h	\
	WheelBaseVehicle.h	\
	TurningRadius.h	\
	PosFrontAx.h	\
	PositionOfOccupants.h	\
	PositioningSolutionType.h	\
	VehicleIdentification.h	\
	WMInumber.h	\
	VDS.h	\
	EnergyStorageType.h	\
	VehicleLength.h	\
	VehicleLengthValue.h	\
	VehicleLengthConfidenceIndication.h	\
	VehicleWidth.h	\
	PathHistory.h	\
	EmergencyPriority.h	\
	InformationQuality.h	\
	RoadType.h	\
	SteeringWheelAngle.h	\
	SteeringWheelAngleValue.h	\
	SteeringWheelAngleConfidence.h	\
	TimestampIts.h	\
	VehicleRole.h	\
	YawRate.h	\
	YawRateValue.h	\
	YawRateConfidence.h	\
	ProtectedZoneType.h	\
	RelevanceDistance.h	\
	RelevanceTrafficDirection.h	\
	TransmissionInterval.h	\
	ValidityDuration.h	\
	ActionID.h	\
	ItineraryPath.h	\
	ProtectedCommunicationZone.h	\
	Traces.h	\
	NumberOfOccupants.h	\
	SequenceNumber.h	\
	PositionOfPillars.h	\
	RestrictedTypes.h	\
	EventHistory.h	\
	EventPoint.h	\
	ProtectedCommunicationZonesRSU.h	\
	CenDsrcTollingZone.h	\
	ProtectedZoneRadius.h	\
	ProtectedZoneID.h	\
	CenDsrcTollingZoneID.h	\
	MAP-PDU.h	\
	SPAT-PDU.h	\
	EmissionType.h	\
	PrioritizationResponse.h	\
	PrioritizationResponseList.h	\
	Reg-ConnectionManeuverAssist.h	\
	Reg-IntersectionState.h	\
	Reg-MapData.h	\
	Reg-Position3D.h	\
	Reg-RestrictionUserType.h	\
	SignalHeadLocation.h	\
	SignalHeadLocationList.h	\
	VehicleToLanePosition.h	\
	VehicleToLanePositionList.h

# asn1c-provided header and source files
ASN_MODULE_HEADERS+=BIT_STRING.h
ASN_MODULE_HEADERS+=BOOLEAN.h
ASN_MODULE_HEADERS+=IA5String.h
ASN_MODULE_HEADERS+=INTEGER.h
ASN_MODULE_HEADERS+=NativeEnumerated.h
ASN_MODULE_HEADERS+=NativeInteger.h
ASN_MODULE_HEADERS+=OCTET_STRING.h
ASN_MODULE_HEADERS+=UTF8String.h
ASN_MODULE_HEADERS+=asn_SEQUENCE_OF.h
ASN_MODULE_HEADERS+=asn_SET_OF.h
ASN_MODULE_HEADERS+=asn_application.h
ASN_MODULE_HEADERS+=asn_codecs.h
ASN_MODULE_HEADERS+=asn_codecs_prim.h
ASN_MODULE_HEADERS+=asn_internal.h
ASN_MODULE_HEADERS+=asn_system.h
ASN_MODULE_HEADERS+=ber_decoder.h
ASN_MODULE_HEADERS+=ber_tlv_length.h
ASN_MODULE_HEADERS+=ber_tlv_tag.h
ASN_MODULE_HEADERS+=constr_CHOICE.h
ASN_MODULE_HEADERS+=constr_SEQUENCE.h
ASN_MODULE_HEADERS+=constr_SEQUENCE_OF.h
ASN_MODULE_HEADERS+=constr_SET_OF.h
ASN_MODULE_HEADERS+=constr_TYPE.h
ASN_MODULE_HEADERS+=constraints.h
ASN_MODULE_HEADERS+=der_encoder.h
ASN_MODULE_HEADERS+=per_decoder.h
ASN_MODULE_HEADERS+=per_encoder.h
ASN_MODULE_HEADERS+=per_opentype.h
ASN_MODULE_HEADERS+=per_support.h
ASN_MODULE_HEADERS+=xer_decoder.h
ASN_MODULE_HEADERS+=xer_encoder.h
ASN_MODULE_HEADERS+=xer_support.h

ASN_MODULE_SOURCES+=BIT_STRING.c
ASN_MODULE_SOURCES+=BOOLEAN.c
ASN_MODULE_SOURCES+=IA5String.c
ASN_MODULE_SOURCES+=INTEGER.c
ASN_MODULE_SOURCES+=NativeEnumerated.c
ASN_MODULE_SOURCES+=NativeInteger.c
ASN_MODULE_SOURCES+=OCTET_STRING.c
ASN_MODULE_SOURCES+=UTF8String.c
ASN_MODULE_SOURCES+=asn_SEQUENCE_OF.c
ASN_MODULE_SOURCES+=asn_SET_OF.c
ASN_MODULE_SOURCES+=asn_codecs_prim.c
ASN_MODULE_SOURCES+=ber_decoder.c
ASN_MODULE_SOURCES+=ber_tlv_length.c
ASN_MODULE_SOURCES+=ber_tlv_tag.c
ASN_MODULE_SOURCES+=constr_CHOICE.c
ASN_MODULE_SOURCES+=constr_SEQUENCE.c
ASN_MODULE_SOURCES+=constr_SEQUENCE_OF.c
ASN_MODULE_SOURCES+=constr_SET_OF.c
ASN_MODULE_SOURCES+=constr_TYPE.c
ASN_MODULE_SOURCES+=constraints.c
ASN_MODULE_SOURCES+=der_encoder.c
ASN_MODULE_SOURCES+=per_decoder.c
ASN_MODULE_SOURCES+=per_encoder.c
ASN_MODULE_SOURCES+=per_opentype.c
ASN_MODULE_SOURCES+=per_support.c
ASN_MODULE_SOURCES+=xer_decoder.c
ASN_MODULE_SOURCES+=xer_encoder.c
ASN_MODULE_SOURCES+=xer_support.c

ASN_SOURCE_FILES = \
	$(ASN1DIR)ITS_ASN1-CAM.asn \
	$(ASN1DIR)ITS_ASN1-DENM.asn \
	$(ASN1DIR)ITS_ASN1-DSRC_REG_D.asn \
	$(ASN1DIR)ITS_ASN1-ITS-Container.asn \
	$(ASN1DIR)ITS_ASN1-MAP_SPAT_ETSI.asn \
	$(ASN1DIR)ITS_ASN1-MAP_SPAT_REG_D.asn

# build configuration
BUILDDIR = build/
INCLDIR = include/
ASN1DIR = asn1/
TGTLIB = libetsimsg.so
CFLAGS += -DASN_PDU_COLLECTION -I$(INCLDIR) -fPIC
LDFLAGS += -L.
ASN_BUILD_SOURCES = ${addprefix $(BUILDDIR),$(ASN_MODULE_SOURCES)}
ASN_BUILD_HEADERS = ${addprefix $(INCLDIR),$(ASN_MODULE_HEADERS)}
OBJS=${ASN_BUILD_SOURCES:.c=.o}
WITNESS=$(BUILDDIR)/witness.tmp

WGET = wget -q

.PHONEY: all clean buildclean regen

# commands
all: $(TGTLIB)

buildclean:
	rm -rf $(BUILDDIR)

clean: buildclean
	rm -rf $(INCLDIR)
	rm -f $(TGTLIB)
	rm -rf $(ASN1DIR)

regen: $(WITNESS)

print-%  : ; @echo $* = $($*)

# final target library linking
$(TGTLIB): $(OBJS)
	$(CC) -shared $(CFLAGS) -o $@ $^ $(LDFLAGS) $(LIBS)

# binary compilation
%.o: %.c $(ASN_BUILD_SOURCES) $(ASN_BUILD_HEADERS)
	$(CC) $(CFLAGS) -o $@ -c $<

# c source file generation
$(ASN_BUILD_SOURCES) $(ASN_BUILD_HEADERS): $(WITNESS)

$(BUILDDIR)/witness.tmp: $(ASN_SOURCE_FILES) | directories
	cd $(BUILDDIR);	$(ASN1C) -gen-PER -pdu=auto ${addprefix ../,$(ASN_SOURCE_FILES)}
	rm $(BUILDDIR)Makefile.am.sample
	mv $(BUILDDIR)*.h $(INCLDIR)
	touch $@

directories: $(BUILDDIR) $(INCLDIR)

$(BUILDDIR) $(INCLDIR) $(ASN1DIR):
	mkdir $@

# downloads
$(ASN1DIR)ITS_ASN1-ITS-Container.asn: | $(ASN1DIR)
	$(WGET) "http://oldforge.etsi.org/websvn/dl.php?repname=ITS.ITS_ASN1&path=%2Ftrunk%2FCDD_TS102894-2%2FITS-Container.asn&rev=18&peg=18" -O $@

$(ASN1DIR)ITS_ASN1-CAM.asn: | $(ASN1DIR)
	$(WGET) "http://oldforge.etsi.org/websvn/dl.php?repname=ITS.ITS_ASN1&path=%2Ftrunk%2FCAM_EN302637-2%2FCAM.asn&rev=2&peg=2" -O $@

$(ASN1DIR)ITS_ASN1-DENM.asn: | $(ASN1DIR)
	$(WGET) "http://oldforge.etsi.org/websvn/dl.php?repname=ITS.ITS_ASN1&path=%2Ftrunk%2FDENM_EN302637-3%2FDENM.asn&rev=5&peg=5" -O $@

$(ASN1DIR)ITS_ASN1-DSRC_REG_D.asn: | $(ASN1DIR)
	$(WGET) "http://oldforge.etsi.org/websvn/dl.php?repname=ITS.ITS_ASN1&path=%2Ftags%2FMAP_SPAT_SAEJ2735%2F20141020%2FDSRC_REG_D.asn&rev=9&peg=9" -O $@

$(ASN1DIR)ITS_ASN1-MAP_SPAT_ETSI.asn: | $(ASN1DIR)
	$(WGET) "http://oldforge.etsi.org/websvn/dl.php?repname=ITS.ITS_ASN1&path=%2Ftags%2FMAP_SPAT_SAEJ2735%2F20141020%2FMAP_SPAT_ETSI.asn&rev=9&peg=9" -O $@

$(ASN1DIR)ITS_ASN1-MAP_SPAT_REG_D.asn: | $(ASN1DIR)
	$(WGET) "http://oldforge.etsi.org/websvn/dl.php?repname=ITS.ITS_ASN1&path=%2Ftags%2FMAP_SPAT_SAEJ2735%2F20141020%2FMAP_SPAT_REG_D.asn&rev=9&peg=9" -O $@
