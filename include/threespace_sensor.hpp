#pragma once

#include "threespace_device.hpp"
#include "yost_fair_mutex.hpp"
#include <thread>

class TssDongle;

#define TSS_NUM_STREAMING_SLOTS 8

class TssSensor : public TssDevice
{
public:
	TssSensor();
	TssSensor(std::string port);
	TssSensor(const TssSensor& other);

	void operator =(const TssSensor& other);

	//Port control functions
	TSS_ERROR openPort(std::string port);
	TSS_ERROR closePort();

	//Header control functions
	TSS_ERROR enableTimestampsWired();
	TSS_ERROR disableTimestampsWired();

	//Streaming functions
	TSS_ERROR enableStreamingWireless(U32 data_flags, U32 interval, U32 duration, U32 delay = 0);
	TSS_ERROR disableStreamingWireless();
	TSS_ERROR startStreamingWired(U32 data_flags, U32 interval, U32 duration, U32 delay = 0);
	TSS_ERROR getFirstStreamingPacket(TSS_Stream_Packet* packet);
	TSS_ERROR getLastStreamingPacket(TSS_Stream_Packet* packet);
	TSS_ERROR stopStreamingWired();
	U32 getStreamingPacketsInWaiting();
	bool didStreamingOverflow();

	//Call and response functions
	void setCommandRetries(U8 retries);
	//0x00
	TSS_ERROR getTaredOrientationAsQuaternion(Orient* quat, U32* timestamp);
	//0x01
	TSS_ERROR getTaredOrientationAsEulerAngles(float* euler, U32* timestamp);
	//0x02
	TSS_ERROR getTaredOrientationAsRotationMatrix(Matrix3x3* mat, U32* timestamp);
	//0x03
	TSS_ERROR getTaredOrientationAsAxisAngle(Vector3* vec, float* angle, U32* timestamp);
	//0x04
	TSS_ERROR getTaredOrientationAsTwoVector(Vector3* forward, Vector3* down, U32* timestamp);
    //0x06
	TSS_ERROR getUntaredOrientationAsQuaternion(Orient* quat, U32* timestamp);
	//0x07
	TSS_ERROR getUntaredOrientationAsEulerAngles(float* euler, U32* timestamp);
	//0x08
	TSS_ERROR getUntaredOrientationAsRotationMatrix(Matrix3x3* mat, U32* timestamp);
	//0x09
	TSS_ERROR getUntaredOrientationAsAxisAngle(Vector3* vec, float* angle, U32* timestamp);
	//0x0A
	TSS_ERROR getUntaredOrientationAsTwoVector(Vector3* north, Vector3* gravity, U32* timestamp);
	//0x60
	TSS_ERROR tareWithCurrentOrientation(U32* timestamp);
	//0x20
	TSS_ERROR getNormalizedSensorData(Vector3* gyroscope3, Vector3* accelerometer3, Vector3* magnetometer3, U32* timestamp);
	//0x21
	TSS_ERROR getNormalizedGyroscope(Vector3* gyroscope3, U32* timestamp);
	//0x22
	TSS_ERROR getNormalizedAccelerometer(Vector3* accelerometer3, U32* timestamp);
	//0x23
	TSS_ERROR getNormalizedMagnetometer(Vector3* magnetometer3, U32* timestamp);
	//0x25
	TSS_ERROR getCorrectedSensorData(Vector3* gyroscope3, Vector3* accelerometer3, Vector3* magnetometer3, U32* timestamp);
	//0x26
	TSS_ERROR getCorrectedGyroscope(Vector3* gyroscope3, U32* timestamp);
	//0x27
	TSS_ERROR getCorrectedAccelerometer(Vector3* accelerometer3, U32* timestamp);
	//0x28
	TSS_ERROR getCorrectedMagnetometer(Vector3* magnetometer3, U32* timestamp);
	//0x29
	TSS_ERROR getCorrectedLinearAccelerationInGlobalSpace(Vector3* accelerometer3, U32* timestamp);
	//0x40
	TSS_ERROR getRawComponentSensorData(Vector3* gyroscope3, Vector3* accelerometer3, Vector3* magnetometer3, U32* timestamp);
	//0x41
	TSS_ERROR getRawGyroscope(Vector3* gyroscope3, U32* timestamp);
	//0x42
	TSS_ERROR getRawAccelerometer(Vector3* accelerometer3, U32* timestamp);
	//0x43
	TSS_ERROR getRawMagnetometer(Vector3* magnetometer3, U32* timestamp);
	//0x2B
	TSS_ERROR getTemperatureC(float* temp, U32* timestamp);
	//0x2C
	TSS_ERROR getTemperatureF(float* temp, U32* timestamp);
	//0x2D
	TSS_ERROR getConfidenceFactor(float* confindence, U32* timestamp);

	//Data-Logging Commands
	//  57(0x39)magnetometer3
	TSS_ERROR turnOnMassStorage(U32* timestamp);
	//  58(0x3a)
	TSS_ERROR turnOffMassStorage(U32* timestamp);
	//  59(0x3b)
	TSS_ERROR formatAndInitializeSDCard(U32* timestamp);
	//  60(0x3c)
	TSS_ERROR beginDataLoggingSession(U32* timestamp);
	//  61(0x3d)
	TSS_ERROR endDataLoggingSession(U32* timestamp);
	//  62(0x3e)
	TSS_ERROR setClockValues(U8 month, U8 day, U8 year, U8 hour, U8 minute,	U8 second, U32* timestamp);
	//  63(0x3f)
	TSS_ERROR getClockValues(U8* month, U8* day, U8* year,	U8* hour, U8* minute, U8* second, U32* timestamp);


	//Streaming Commands
	//  95(0x5f)
	TSS_ERROR updateCurrentTimestamp(U32 set_timestamp, U32* timestamp);
	//  80(0x50)
	TSS_ERROR setStreamingSlots(U8 slot1, U8 slot2, U8 slot3, U8 slot4, U8 slot5, U8 slot6, U8 slot7, U8 slot8, U32* timestamp);
	//  81(0x51)
	TSS_ERROR getStreamingSlots(U8* slot1, U8* slot2, U8* slot3, U8* slot4, U8* slot5, U8* slot6, U8* slot7, U8* slot8, U32* timestamp);
	//  82(0x52)
	TSS_ERROR setStreamingTiming(U32 interval, U32 duration, U32 delay, U32* timestamp);
	//  83(0x53)
	TSS_ERROR getStreamingTiming(U32* interval, U32* duration, U32* delay, U32* timestamp);

	//Configuration Write Commands
	//  16(0x10)
	TSS_ERROR setEulerAngleDecompositionOrder(U8 order, U32* timestamp);
	//  17(0x11)
	TSS_ERROR setMagnetoresistiveThreshold(float threshold, U32 trust_frames, float lockout_decay, float perturbation_detection_value, U32* timestamp);
	//  18(0x12)
	TSS_ERROR setAccelerometerResistanceThreshold(float threshold, U32 lockout_frames, U32* timestamp);
	//  19(0x13)
	TSS_ERROR offsetWithCurrentOrientation(U32* timestamp);
	//  20(0x14)
	TSS_ERROR resetBaseOffset(U32* timestamp);
	//  21(0x15)
	TSS_ERROR offsetWithQuaternion(Orient* quat4, U32* timestamp);
	//  97(0x61)
	TSS_ERROR tareWithQuaternion(Orient* quat4, U32* timestamp);
	//  98(0x62)
	TSS_ERROR tareWithRotationMatrix(float* matrix9, U32* timestamp);
	//  99(0x63)
	TSS_ERROR setStaticAccelerometerTrustValue(float trust_value, U32* timestamp);
	// 100(0x64)
	TSS_ERROR setConfidenceAccelerometerTrustValues(float min_trust_value, float max_trust_value, U32* timestamp);
	// 101(0x65)
	TSS_ERROR setStaticCompassTrustValue(float trust_value, U32* timestamp);
	// 102(0x66)
	TSS_ERROR setConfidenceCompassTrustValues(float min_trust_value, float max_trust_value, U32* timestamp);
	// 103(0x67)
	TSS_ERROR setDesiredUpdateRate(U32 update_rate, U32* timestamp);
	// 105(0x69)
	TSS_ERROR setReferenceVectorMode(U8 mode, U32* timestamp);
	// 106(0x6a)
	TSS_ERROR setOversampleRate(U16 gyro_sample_rate, U16 accel_sample_rate, U16 compass_sample_rate, U32* timestamp);
	// 107(0x6b)
	TSS_ERROR setGyroscopeEnabled(U8 enabled, U32* timestamp);
	// 108(0x6c)
	TSS_ERROR setAccelerometerEnabled(U8 enabled, U32* timestamp);
	// 109(0x6d)
	TSS_ERROR setCompassEnabled(U8 enabled, U32* timestamp);
	// 110(0x6e)
	TSS_ERROR setOrientationSmoothing(float enabled, float compass_max_smooth_factor, float compass_min_smooth_factor, float accelerometer_max_smooth_factor, float accelerometer_min_smooth_factor, float smoothing_magnification, float smoothing_offset, U32* timestamp);
	// 111(0x6f)
	TSS_ERROR getOrientationSmoothing(U8* enabled, float* compass_max_smooth_factor, float* compass_min_smooth_factor, float* accelerometer_max_smooth_factor, float* accelerometer_min_smooth_factor, float* smoothing_magnification, float* smoothing_offset, U32* timestamp);
	// 116(0x74)
	TSS_ERROR setAxisDirections(U8 axis_direction_byte, U32* timestamp);
	// 117(0x75)
	TSS_ERROR setRunningAveragePercent(float gyro_running_average_percent, float accel_running_average_percent, float mag_running_average_percent, float orient_running_average_percent, U32* timestamp);
	// 118(0x76)
	TSS_ERROR setCompassReferenceVector(Vector3* reference_vector3, U32* timestamp);
	// 119(0x77)
	TSS_ERROR setAccelerometerReferenceVector(Vector3* reference_vector3, U32* timestamp);
	// 120(0x78)
	TSS_ERROR resetKalmanFilter(U32* timestamp);
	// 121(0x79)
	TSS_ERROR setAccelerometerRange(U8 accelerometer_range_setting, U32* timestamp);
	// 123(0x7b)
	TSS_ERROR setFilterMode(U8 mode, U32* timestamp);
	// 124(0x7c)
	TSS_ERROR setRunningAverageMode(U8 mode, U32* timestamp);
	// 125(0x7d)
	TSS_ERROR setGyroscopeRange(U8 gyroscope_range_setting, U32* timestamp);
	// 126(0x7e)
	TSS_ERROR setCompassRange(U8 compass_range_setting, U32* timestamp);

	// Configuration Read Commands
	// 128(0x80)  Get tare orientation as quaternion
	TSS_ERROR getTareAsQuaternion(Orient* quat4, U32* timestamp);
	// 129(0x81)  Get tare orientation as rotation matrix
	TSS_ERROR getTareAsRotationMatrix(float* matrix9, U32* timestamp);
	// 130(0x82)
	TSS_ERROR getAccelerometerTrustValues(float* min_trust_value, float* max_trust_value, U32* timestamp);
	// 131(0x83)
	TSS_ERROR getCompassTrustValues(float* min_trust_value, float* max_trust_value, U32* timestamp);
	// 132(0x84)
	TSS_ERROR getCurrentUpdateRate(U32* last_update, U32* timestamp);
	// 133(0x85)
	TSS_ERROR getCompassReferenceVector(float* reference_vector, U32* timestamp);
	// 134(0x86)
	TSS_ERROR getAccelerometerReferenceVector(float* reference_vector, U32* timestamp);
	// 135(0x87)
	TSS_ERROR getReferenceVectorMode(U8* mode, U32* timestamp);
	// 140(0x8c)
	TSS_ERROR getGyroscopeEnabledState(U8* enabled, U32* timestamp);
	// 141(0x8d)
	TSS_ERROR getAccelerometerEnabledState(U8* enabled, U32* timestamp);
	// 142(0x8e)
	TSS_ERROR getCompassEnabledState(U8 * enabled, U32* timestamp);
	// 143(0x8f)
	TSS_ERROR getAxisDirections(U8* axis_directions, U32* timestamp);
	// 144(0x90)
	TSS_ERROR getOversampleRate(U16* gyro_sample_rate, U16* accel_sample_rate, U16* compass_sample_rate, U32* timestamp);
	// 145(0x91)
	TSS_ERROR getRunningAveragePercent(float* gyro_running_average_percent, float* accel_running_average_percent, float* mag_running_average_percent, float* orient_running_average_percent, U32* timestamp);
	// 146(0x92)
	TSS_ERROR getDesiredUpdateRate(U32* update_rate, U32* timestamp);
	// 148(0x94)
	TSS_ERROR getAccelerometerRange(U8* accelerometer_range_setting, U32* timestamp);
	// 152(0x98)
	TSS_ERROR getFilterMode(U8* mode, U32* timestamp);
	// 153(0x99)
	TSS_ERROR getRunningAverageMode(U8* mode, U32* timestamp);
	// 154(0x9a)
	TSS_ERROR getGyroscopeRange(U8* gyroscope_range_setting, U32* timestamp);
	// 155(0x9b)
	TSS_ERROR getCompassRange(U8* compass_range_setting, U32* timestamp);
	// 156(0x9c)
	TSS_ERROR getEulerAngleDecompositionOrder(U8* order, U32* timestamp);
	// 157(0x9d)
	TSS_ERROR getMagnetoresistiveThreshold(float* threshold, U32* trust_frames, float* lockout_decay, float* perturbation_detection_value, U32* timestamp);
	// 158(0x9e)
	TSS_ERROR getAccelerometerResistanceThreshold(float* threshold, U32* lockout_frames, U32* timestamp);
	// 159(0x9f)
	TSS_ERROR getOffsetOrientationAsQuaternion(Orient* quat4, U32* timestamp);

	//Calibration Commands
	// 160(0xa0)
	TSS_ERROR setCompassCalibrationCoefficients(float* matrix9, Vector3* bias3, U32* timestamp);
	// 161(0xa1)
	TSS_ERROR setAccelerometerCalibrationCoefficients(float* matrix9, Vector3* bias3, U32* timestamp);
	// 162(0xa2)
	TSS_ERROR getCompassCalibrationCoefficients(float* matrix9, Vector3* bias3, U32* timestamp);
	// 163(0xa3)
	TSS_ERROR getAccelerometerCalibrationCoefficients(float* matrix9, Vector3* bias3, U32* timestamp);
	// 164(0xa4)
	TSS_ERROR getGyroscopeCalibrationCoefficients(float* matrix9, Vector3* bias3, U32* timestamp);
	// 165(0xa5)
	TSS_ERROR beginGyroscopeAutoCalibration(U32* timestamp);
	// 166(0xa6)
	TSS_ERROR setGyroscopeCalibrationCoefficients(float* matrix9, Vector3* bias3, U32* timestamp);
	// 169(0xa9)
	TSS_ERROR setCalibrationMode(U8 mode, U32* timestamp);
	// 170(0xaa)
	TSS_ERROR getCalibrationMode(U8 * mode, U32* timestamp);
	// 171(0xab)
	//TSS_ERROR setOrthoCalibrationDataPointFromCurrentOrientation(U32* timestamp);
	// 172(0xac)
	//TSS_ERROR setOrthoCalibrationDataPointFromVector(U8 type, U8 index, Vector3* vector3, U32* timestamp);
	// 173(0xad)
	//TSS_ERROR getOrthoCalibrationDataPoint(U8 type, U8 index, Vector3* vector3, U32* timestamp);
	// 174(0xae)
	//TSS_ERROR performOrthoCalibration(U32* timestamp);
	// 175(0xaf)
	//TSS_ERROR clearOrthoCalibrationData(U32* timestamp);

	// 171(0xab)
	TSS_ERROR setAutoCompassCalibrationMode(U8 mode, U32* timestamp);
	// 172(0xac)
	TSS_ERROR getAutoCompassCalibrationMode(U8* mode, U32* timestamp);
	// 173(0xad)
	TSS_ERROR setAutoCompassCalibrationSettings(float too_close_angle, float bias_movement_percentage, float coplanarity_tolerance, U8 max_averages, U8 max_bad_deviations, U32* timestamp);
	// 174(0xae)
	TSS_ERROR getAutoCompassCalibrationSettings(float* too_close_angle, float* bias_movement_percentage, float* coplanarity_tolerance, U8* max_averages, U8* max_bad_deviations, U32* timestamp);
	// 175(0xaf)
	TSS_ERROR getAutoCompassCaibrationCount(U8* count, U32* timestamp);


	//Wireless Sensor & Dongle Commands
	// 192(0xc0)
	TSS_ERROR getWirelessPanID(U16* panid, U32* timestamp);
	// 193(0xc1)
	TSS_ERROR setWirelessPanID(U16 panid, U32* tinestamp);
	// 194(0xc2)
	TSS_ERROR getWirelessChannel(U8* channel, U32* timestamp);
	// 195(0xc3)
	TSS_ERROR setWirelessChannel(U8 channel, U32* timestamp);
	// 197(0xc5)
	TSS_ERROR commitWirelessSettings(U32* timestamp);
	// 198(0xc6)
	TSS_ERROR getWirelessAddress(U16* address, U32* timestamp);

	//Battery Commands
	// 201(0xc9)
	TSS_ERROR getBatteryVoltage(float* battery_voltage, U32* timestamp);
	// 202(0xca)
	TSS_ERROR getBatteryPercentRemaining(U8* battery_percent, U32* timestamp);
	// 203(0xcb)
	TSS_ERROR getBatteryStatus(U8* battery_charge_status, U32* timestamp);

	//General Commands
	// 196(0xc4)
	TSS_ERROR setLEDMode(U8 mode, U32* timestamp);
	// 200(0xc8)
	TSS_ERROR getLEDMode(U8* mode, U32* timestamp);
	// 221(0xdd)
	TSS_ERROR getWiredResponseHeaderBitfield(U32* bitfield, U32* timestamp);
	// 222(0xde)
	TSS_ERROR setWiredResponseHeaderBitfield(U32 bitfield, U32* timestamp);
	// 224(0xe0)
	TSS_ERROR restoreFactorySettings(U32* timestamp);
	// 225(0xe1)
	TSS_ERROR commitSettings(U32* timestamp);
	// 226(0xe2)
	TSS_ERROR softwareReset();
	// 227(0xe3)
	TSS_ERROR setSleepMode(U8 mode, U32* timestamp);
	// 228(0xe4)
	TSS_ERROR getSleepMode(U8* mode, U32* timestamp);		
	// 231(0xe7)
	TSS_ERROR setUARTBaudRate(U32 baud_rate, U32* timestamp);
	// 232(0xe8)
	TSS_ERROR getUARTBaudRate(U32* baud_rate, U32* timestamp);
	// 233(0xe9)
	TSS_ERROR setUSBMode(U8 mode, U32* timestamp);
	// 234(0xea)
	TSS_ERROR getUSBMode(U8* mode, U32* timestamp);
	// 238(0xee)
	TSS_ERROR setLEDColor(Vector3* rgb_color3, U32* timestamp);
	// 239(0xef)
	TSS_ERROR getLEDColor(Vector3* rgb_color3, U32* timestamp);

	//Wired HID Commands
	// 240(0xf0)
	TSS_ERROR setJoystickEnabled(U8 enabled_state, U32* timestamp);
	// 241(0xf1)
	TSS_ERROR setMouseEnabled(U8 enabled_state, U32* timestamp);
	// 242(0xf2)
	TSS_ERROR getJoystickEnabled(U8* enabled_state, U32* timestamp);
	// 243(0xf3)
	TSS_ERROR getMouseEnabled(U8* enabled_state, U32* timestamp);

	//General HID Commands
	// 244(0xf4)
	TSS_ERROR setControlMode(U8 control_class, U8 control_index, U8 handler_index, U32* timestamp);
	// 245(0xf5)
	TSS_ERROR setControlData(U8 control_class, U8 control_index, U8 data_point_index, float data_point, U32* timestamp);
	// 246(0xf6)
	TSS_ERROR getControlMode(U8 control_class, U8 control_index, U8* handler_index, U32* timestamp);
	// 247(0xf7)
	TSS_ERROR getControlData(U8 control_class, U8 control_index, U8 data_point_index, float * data_point, U32* timestamp);
	// 248(0xf8)
	TSS_ERROR setButtonGyroDisableLength(U8 number_of_frames, U32* timestamp);
	// 249(0xf9)
	TSS_ERROR getButtonGyroDisableLength(U8* number_of_frames, U32* timestamp);
	// 250(0xfa)
	TSS_ERROR getButtonState(U8* button_state, U32* timestamp);
	// 251(0xfb)
	TSS_ERROR setMouseAbsoluteRelativeMode(U8 mode, U32* timestamp);
	// 252(0xfc)
	TSS_ERROR getMouseAbsoluteRelativeMode(U8* mode, U32* timestamp);
	// 253(0xfd)
	TSS_ERROR setJoystickAndMousePresentRemoved(U8 joystick, U8 mouse, U32* timestamp);
	// 254(0xfe)
	TSS_ERROR getJoystickAndMousePresentRemoved(U8* joystick, U8* mouse, U32* timestamp);

	//Pedestrian Tracking Set Commands
	// 52(0x34),0(0x00)
	TSS_ERROR setPedestrianTrackingState(U8 state, U32* timestamp);
	// 52(0x34),1(0x01)
	TSS_ERROR setSelectedStepIndex(U16 index, U32* timestamp);
	// 52(0x34),2(0x02)
	TSS_ERROR setStepRatioMinimum(float min_ratio, U32* timestamp);
	// 52(0x34),3(0x03)
	TSS_ERROR setStepRatioMaximum(float max_ratio, U32* timestamp);
	// 52(0x34),4(0x04)
	TSS_ERROR setStepDutyCycleMinimum(float min_duty_cycle, U32* timestamp);
	// 52(0x34),5(0x05)
	TSS_ERROR setStepDutyCycleMaximum(float max_duty_cycle, U32* timestamp);
	// 52(0x34),6(0x06)
	TSS_ERROR setStepAmplitudeMinimum(float min_amplitude, U32* timestamp);
	// 52(0x34),7(0x07)
	TSS_ERROR setStepAmplitudeMaximum(float max_amplitude, U32* timestamp);
	// 52(0x34),8(0x08)
	TSS_ERROR setStepDurationMinimum(float min_step_duration, U32* timestamp);
	// 52(0x34),9(0x09)
	TSS_ERROR setStepDurationMaximum(float max_step_duration, U32* timestamp);
	// 52(0x34),10(0x0A)
	TSS_ERROR setStepStrideSlope(float slope, U32* timestamp);
	// 52(0x34),11(0x0B)
	TSS_ERROR setStepStrideOffset(float offset, U32* timestamp);
	// 52(0x34),12(0x0C)
	TSS_ERROR setPedestrianTrackingUnits(U8 unit_select, U32* timestamp);
	// 52(0x34),13(0x0D)
	TSS_ERROR setBarometerAltitudeOffset(float offset, U32* timestamp);
	// 52(0x34),14(0x0E)
	TSS_ERROR autoSetBarometerAltitudeOffset(float known_height, U32* timestamp);
	// 52(0x34),15(0x0F)
	TSS_ERROR resetSteps(U32* timestamp);

	//Pedestrian Tracking Get Commands
	// 53(0x35),0(0x00)
	TSS_ERROR getPedestrianTrackingState(U8* state, U32* timestamp);
	// 53(0x35),1(0x01)
	TSS_ERROR getSelectedStepIndex(U16* index, U32* timestamp);
	// 53(0x35),2(0x02)
	TSS_ERROR getStepRatioMinimum(float* min_ratio, U32* timestamp);
	// 53(0x35),3(0x03)
	TSS_ERROR getStepRatioMaximum(float* max_ratio, U32* timestamp);
	// 53(0x35),4(0x04)
	TSS_ERROR getStepDutyCycleMinimum(float* min_duty_cycle, U32* timestamp);
	// 53(0x35),5(0x05)
	TSS_ERROR getStepDutyCycleMaximum(float* max_duty_cycle, U32* timestamp);
	// 53(0x35),6(0x06)
	TSS_ERROR getStepAmplitudeMinimum(float* min_amplitude, U32* timestamp);
	// 53(0x35),7(0x07)
	TSS_ERROR getStepAmplitudeMaximum(float* max_amplitude, U32* timestamp);
	// 53(0x35),8(0x08)
	TSS_ERROR getStepDurationMinimum(float* min_step_duration, U32* timestamp);
	// 53(0x35),9(0x09)
	TSS_ERROR getStepDurationMaximum(float* max_step_duration, U32* timestamp);
	// 53(0x35),10(0x0A)
	TSS_ERROR getStepStrideSlope(float* slope, U32* timestamp);
	// 53(0x35),11(0x0B)
	TSS_ERROR getStepStrideOffset(float* offset, U32* timestamp);
	// 53(0x35),12(0x0C)
	TSS_ERROR getPedestrianTrackingUnits(U8* unit_select, U32* timestamp);
	// 53(0x35),13(0x0D)
	TSS_ERROR getAltitudeOffset(float* offset, U32* timestamp);
	// 53(0x35),14(0x0E)
	TSS_ERROR getCurrentAltitude(float* altitude, U32* timestamp);
	// 53(0x35),15(0x0F)
	TSS_ERROR getFirstAltitude(float* altitude, U32* timestamp);
	// 53(0x35),16(0x10)
	TSS_ERROR getAltitudeDifference(float* difference, U32* timestamp);
	// 53(0x35),17(0x11)
	TSS_ERROR getCurrentPressure(float* pressure, U32* timestamp);
	// 53(0x35),18(0x12)
	TSS_ERROR getCurrentHeading(float* heading, U32* timestamp);
	// 53(0x35),19(0x13)
	TSS_ERROR getStepBufferLength(U16* step_buffer_length, U32* timestamp);
	// 53(0x35),20(0x14)
	TSS_ERROR getTotalDistanceTraveled(float* distance, U32* timestamp);
	// 53(0x35),21(0x15)
	TSS_ERROR getStepSessionConfidenceValue(float* confidence, U32* timestamp);
	// 53(0x35),22(0x16)
	TSS_ERROR getLatestStep(float* step, U32* timestamp);
	// 53(0x35),23(0x17)
	TSS_ERROR getStepAtSelectedIndex(float* step, U32* timestamp);

	TSS_ERROR manualWrite(U8* bytes, U16 amount, U32* timestamp);
	TSS_ERROR manualRead(U8* bytes, U16 amount, U16* read_count, U32* timestamp);
	TSS_ERROR manualFlush(U32* timestamp);

	//not private, but not meant to be called by end-users
	void _parseStreamingPacketItem(U8* dest, U8* src, U16& src_index, U8 nbytes);
	void _parseStreamingPacketItemFloats(float* dest, U8* src, U16& src_index, U8 nfloats);
	TSS_ERROR _parseStreamingPacket(TSS_Stream_Packet* packet);
	void _addToStreamingSlots(U8 command, U8 nbytes, U8* command_buff, U8& curr_slot);

	TSS_ERROR _prepareStreaming(U32 data_flags, U32 interval, U32 duration, U32 delay = 0);
	TSS_ERROR _prepareStreamingWireless(U32 data_flags, U32 interval, U32 duration, U32 delay = 0);
	TSS_ERROR _triggerStreaming();

	U8 _streamingSlots[8];
	U8 _streamingSlotIndex[8];
	U8 _streamDataSize;
	U32 _streamInterval;
	TssStreamPacketCircularBuffer _streamBuffer;
	
	vector<U8> _unparsedStreamData;

	TssDongle* _ownerDongle;
};