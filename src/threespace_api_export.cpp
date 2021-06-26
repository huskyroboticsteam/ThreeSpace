#include "threespace_api_export.h"
#include "threespace_api.hpp"
#include <iostream>
#include <fstream>

vector<shared_ptr<TssSensor>> stored_sensors;
vector<shared_ptr<TssDongle>> stored_dongles;
vector<TssComPort> stored_tss_ports;
U8 stored_port_index = 0;

#define SENSOR_RANGE_CHECK() if (sensor_id >= stored_sensors.size() || !stored_sensors[sensor_id]){return TSS_ERROR_INVALID_ID;}
#define DONGLE_RANGE_CHECK() if (dongle_id >= stored_dongles.size() || !stored_dongles[dongle_id]){return TSS_ERROR_INVALID_ID;}

TSS_EXPORT TSS_ERROR tss_initAPI()
{
	gAPI.init();

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_deinitAPI()
{
	stored_sensors.clear();
	stored_dongles.clear();

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_createSensor(const char* port_name, tss_device_id* out_id)
{
	TssSensor* sensor = new TssSensor(port_name);

	if (!sensor->isConnected())
	{
		delete sensor;
		return TSS_ERROR_CANT_OPEN_PORT;
	}

	*out_id = stored_sensors.size();
	stored_sensors.push_back(shared_ptr<TssSensor>(sensor));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_removeSensor(tss_device_id sensor_id)
{
	SENSOR_RANGE_CHECK();

	if (stored_sensors[sensor_id]->_isStreaming)
	{
		gAPI.unregisterStreamingDevice(stored_sensors[sensor_id].get());
	}

	stored_sensors[sensor_id].reset();

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_isConnected(tss_device_id sensor_id)
{
	SENSOR_RANGE_CHECK();

	if (!stored_sensors[sensor_id]->isConnected())
	{
		return TSS_ERROR_NOT_CONNECTED;
	}

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_isWireless(tss_device_id sensor_id, U8* is_wireless)
{
	SENSOR_RANGE_CHECK();

	*is_wireless = (U8)stored_sensors[sensor_id]->_isWireless;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getLastSensorTimestamp(tss_device_id sensor_id, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	*timestamp = stored_sensors[sensor_id]->getLastHeader()->SensorTimestamp;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getLastSystemTimestamp(tss_device_id sensor_id, float* timestamp)
{
	SENSOR_RANGE_CHECK();

	*timestamp = stored_sensors[sensor_id]->getLastHeader()->SystemTimestamp;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_inBootloader(tss_device_id sensor_id, U8* in_bootloader)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->_bootloaderCheck(in_bootloader));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getSerialNumber(tss_device_id sensor_id, U32* serial_number, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->_readSerialNumber(timestamp));
	*serial_number = stored_sensors[sensor_id]->getSerialNumber();

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getHardwareVersionString(tss_device_id sensor_id, char* version_string, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->_readVersionString(timestamp));
	string tmp = stored_sensors[sensor_id]->getHardwareVersionString();		
	for (int i = 0; i < 32; i++) version_string[i] = '\0';
	memcpy(version_string, tmp.c_str(), tmp.length());

	if (tmp.length())
		version_string[tmp.length() - 1] = '\0';

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getFirmwareVersionString(tss_device_id sensor_id, char* version_string, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->_readFirmwareString(timestamp));
	string tmp = stored_sensors[sensor_id]->getFirmwareVersionString();	
	for (int i = 0; i < 32; i++) version_string[i] = '\0';
	memcpy(version_string, tmp.c_str(), tmp.length());
	
	if (tmp.length())
		version_string[tmp.length() - 1] = '\0';

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getSensorType(tss_device_id sensor_id, U16* sensor_type)
{
	SENSOR_RANGE_CHECK();

	*sensor_type = stored_sensors[sensor_id]->getSensorType();

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_openPort(tss_device_id sensor_id, const char* port_name)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->openPort(port_name));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_closePort(tss_device_id sensor_id)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->closePort());

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_enableTimestampsWired(tss_device_id sensor_id)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->enableTimestampsWired());

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_disableTimestampsWired(tss_device_id sensor_id)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->disableTimestampsWired());

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_enableStreamingWireless(tss_device_id sensor_id, U32 data_flags, U32 interval, U32 duration, U32 delay)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->enableStreamingWireless(data_flags, interval, duration, delay));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_disableStreamingWireless(tss_device_id sensor_id)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->disableStreamingWireless());

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_startStreamingWired(tss_device_id sensor_id, U32 data_flags, U32 interval, U32 duration, U32 delay)
{
	SENSOR_RANGE_CHECK();

	return stored_sensors[sensor_id]->startStreamingWired(data_flags, interval, duration, delay);
}

TSS_EXPORT TSS_ERROR tss_sensor_getFirstStreamingPacket(tss_device_id sensor_id, TSS_Stream_Packet* stream_packet)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getFirstStreamingPacket(stream_packet));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getFirstStreamingPacketRaw(tss_device_id sensor_id, U8* stream_packet)
{
	SENSOR_RANGE_CHECK();

	TSS_Stream_Packet packet;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getFirstStreamingPacket(&packet));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getLastStreamingPacket(tss_device_id sensor_id, TSS_Stream_Packet* stream_packet)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getLastStreamingPacket(stream_packet));
	
	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getLastStreamingPacketRaw(tss_device_id sensor_id, U8* stream_packet)
{
	SENSOR_RANGE_CHECK();

	TSS_Stream_Packet packet;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getLastStreamingPacket(&packet));

	// Copy the header
	memcpy(stream_packet, &packet.header.Flags, 1);
	memcpy(stream_packet+1, &packet.header.Success, 1);
	memcpy(stream_packet+2, &packet.header.SensorTimestamp, 4);
	memcpy(stream_packet+6, &packet.header.SystemTimestamp, 4);
	memcpy(stream_packet+10, &packet.header.CommandEcho, 1);
	memcpy(stream_packet+11, &packet.header.Checksum, 1);
	memcpy(stream_packet+12, &packet.header.LogicalId, 1);
	memcpy(stream_packet+13, &packet.header.SerialNumber, 4);
	memcpy(stream_packet+17, &packet.header.DataLength, 1);
	memcpy(stream_packet+18, &packet.rawDataSize, 1);
	memcpy(stream_packet+19, &packet.rawData, packet.rawDataSize);

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_stopStreamingWired(tss_device_id sensor_id)
{
	SENSOR_RANGE_CHECK();

	return stored_sensors[sensor_id]->stopStreamingWired();
}

TSS_EXPORT TSS_ERROR tss_sensor_getStreamingPacketsInWaiting(tss_device_id sensor_id, U32* in_waiting)
{
	SENSOR_RANGE_CHECK();

	*in_waiting = stored_sensors[sensor_id]->getStreamingPacketsInWaiting();

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_didStreamingOverflow(tss_device_id sensor_id, U8* overflow)
{
	SENSOR_RANGE_CHECK();

	*overflow = stored_sensors[sensor_id]->didStreamingOverflow();

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getStreamingSlots(tss_device_id sensor_id, U8* slot1, U8* slot2, U8* slot3, U8* slot4, U8* slot5, U8* slot6, U8* slot7, U8* slot8, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getStreamingSlots(slot1, slot2, slot3, slot4, slot5, slot6, slot7, slot8, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setStreamingSlots(tss_device_id sensor_id, U8 slot1, U8 slot2, U8 slot3, U8 slot4, U8 slot5, U8 slot6, U8 slot7, U8 slot8, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setStreamingSlots(slot1, slot2, slot3, slot4, slot5, slot6, slot7, slot8, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getStreamingTiming(tss_device_id sensor_id, U32* interval, U32* duration, U32* delay, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getStreamingTiming(interval, duration, delay, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setStreamingTiming(tss_device_id sensor_id, U32 interval, U32 duration, U32 delay, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setStreamingTiming(interval, duration, delay, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getWiredResponseHeaderBitfield(tss_device_id sensor_id, U32* bitfield, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getWiredResponseHeaderBitfield(bitfield, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setWiredResponseHeaderBitfield(tss_device_id sensor_id, U32 bitfield, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setWiredResponseHeaderBitfield(bitfield, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setCommandRetries(tss_device_id sensor_id, U8 retries)
{
	SENSOR_RANGE_CHECK();

	stored_sensors[sensor_id]->setCommandRetries(retries);

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getTaredOrientationAsQuaternion(tss_device_id sensor_id, float* quat, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	Orient vquat;
	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getTaredOrientationAsQuaternion(&vquat, timestamp));

	memcpy(quat, vquat.data, 16);

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getTaredOrientationAsEulerAngles(tss_device_id sensor_id, float* euler, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getTaredOrientationAsEulerAngles(euler, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getTaredOrientationAsRotationMatrix(tss_device_id sensor_id, float* mat, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	Matrix3x3 vmat;
	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getTaredOrientationAsRotationMatrix(&vmat, timestamp));

	memcpy(mat, vmat.data, 36);

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getTaredOrientationAsAxisAngle(tss_device_id sensor_id, float* vec, float* angle, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	Vector3 vvec;
	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getTaredOrientationAsAxisAngle(&vvec, angle, timestamp));

	memcpy(vec, vvec.data, 12);

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getTaredOrientationAsTwoVector(tss_device_id sensor_id, float* forward, float* down, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	Vector3 vforward;
	Vector3 vdown;
	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getTaredOrientationAsTwoVector(&vforward, &vdown, timestamp));

	memcpy(forward, vforward.data, 12);
	memcpy(down, vdown.data, 12);

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getUntaredOrientationAsQuaternion(tss_device_id sensor_id, float* orient_quat, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	Orient vquat;
	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getUntaredOrientationAsQuaternion(&vquat, timestamp));

	memcpy(orient_quat, vquat.data, 16);

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getUntaredOrientationAsEulerAngles(tss_device_id sensor_id, float* euler, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getUntaredOrientationAsEulerAngles(euler, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getUntaredOrientationAsRotationMatrix(tss_device_id sensor_id, float* mat, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	Matrix3x3 vmat;
	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getUntaredOrientationAsRotationMatrix(&vmat, timestamp));

	memcpy(mat, vmat.data, 36);

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getUntaredOrientationAsAxisAngle(tss_device_id sensor_id, float* vec, float* angle, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	Vector3 vvec;
	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getUntaredOrientationAsAxisAngle(&vvec, angle, timestamp));

	memcpy(vec, vvec.data, 12);

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getUntaredOrientationAsTwoVector(tss_device_id sensor_id, float* north, float* gravity, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	Vector3 vnorth;
	Vector3 vgravity;
	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getUntaredOrientationAsTwoVector(&vnorth, &vgravity, timestamp));

	memcpy(north, vnorth.data, 12);
	memcpy(gravity, vgravity.data, 12);

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_tareWithCurrentOrientation(tss_device_id sensor_id, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->tareWithCurrentOrientation(timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getWirelessChannel(tss_device_id sensor_id, U8* channel, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getWirelessChannel(channel, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setWirelessChannel(tss_device_id sensor_id, U8 channel, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setWirelessChannel(channel, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getWirelessPanID(tss_device_id sensor_id, U16* pan_id, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getWirelessPanID(pan_id, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setWirelessPanID(tss_device_id sensor_id, U16 pan_id, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setWirelessPanID(pan_id, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_commitSettings(tss_device_id sensor_id, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->commitSettings(timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_commitWirelessSettings(tss_device_id sensor_id, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->commitWirelessSettings(timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getNormalizedSensorData(tss_device_id sensor_id, float* gyroscope3, float* accelerometer3, float* magnetometer3, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	Vector3 gyroscope;
	Vector3 accelerometer;
	Vector3 magnetometer;
	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getNormalizedSensorData(&gyroscope, &accelerometer, &magnetometer, timestamp));

	memcpy(gyroscope3, gyroscope.data, 12);
	memcpy(accelerometer3, accelerometer.data, 12);
	memcpy(magnetometer3, magnetometer.data, 12);

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getNormalizedGyroscope(tss_device_id sensor_id, float* gyroscope3, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	Vector3 gyroscope;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getNormalizedGyroscope(&gyroscope, timestamp));

	memcpy(gyroscope3, gyroscope.data, 12);

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getNormalizedAccelerometer(tss_device_id sensor_id, float* accelerometer3, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	Vector3 accelerometer;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getNormalizedGyroscope(&accelerometer, timestamp));

	memcpy(accelerometer3, accelerometer.data, 12);

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getNormalizedMagnetometer(tss_device_id sensor_id, float* magnetometer3, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	Vector3 magnetometer;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getNormalizedMagnetometer(&magnetometer, timestamp));

	memcpy(magnetometer3, magnetometer.data, 12);

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getCorrectedSensorData(tss_device_id sensor_id, float* gyroscope3, float* accelerometer3, float* magnetometer3, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	Vector3 gyroscope;
	Vector3 accelerometer;
	Vector3 magnetometer;
	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getCorrectedSensorData(&gyroscope, &accelerometer, &magnetometer, timestamp));

	memcpy(gyroscope3, gyroscope.data, 12);
	memcpy(accelerometer3, accelerometer.data, 12);
	memcpy(magnetometer3, magnetometer.data, 12);

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getCorrectedGyroscope(tss_device_id sensor_id, float* gyroscope3, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	Vector3 gyroscope;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getCorrectedGyroscope(&gyroscope, timestamp));

	memcpy(gyroscope3, gyroscope.data, 12);

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getCorrectedAccelerometer(tss_device_id sensor_id, float* accelerometer3, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	Vector3 accelerometer;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getCorrectedAccelerometer(&accelerometer, timestamp));

	memcpy(accelerometer3, accelerometer.data, 12);

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getCorrectedMagnetometer(tss_device_id sensor_id, float* magnetometer3, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	Vector3 magnetometer;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getCorrectedMagnetometer(&magnetometer, timestamp));

	memcpy(magnetometer3, magnetometer.data, 12);

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getCorrectedLinearAccelerationInGlobalSpace(tss_device_id sensor_id, float* accelerometer3, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	Vector3 accelerometer;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getCorrectedLinearAccelerationInGlobalSpace(&accelerometer, timestamp));

	memcpy(accelerometer3, accelerometer.data, 12);

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getRawComponentSensorData(tss_device_id sensor_id, float* gyroscope3, float* accelerometer3, float* magnetometer3, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	Vector3 gyroscope;
	Vector3 accelerometer;
	Vector3 magnetometer;
	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getRawComponentSensorData(&gyroscope, &accelerometer, &magnetometer, timestamp));

	memcpy(gyroscope3, gyroscope.data, 12);
	memcpy(accelerometer3, accelerometer.data, 12);
	memcpy(magnetometer3, magnetometer.data, 12);

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getRawGyroscope(tss_device_id sensor_id, float* gyroscope3, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	Vector3 gyroscope;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getRawGyroscope(&gyroscope, timestamp));

	memcpy(gyroscope3, gyroscope.data, 12);

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getRawAccelerometer(tss_device_id sensor_id, float* accelerometer3, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	Vector3 accelerometer;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getRawAccelerometer(&accelerometer, timestamp));

	memcpy(accelerometer3, accelerometer.data, 12);

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getRawMagnetometer(tss_device_id sensor_id, float* magnetometer3, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	Vector3 magnetometer;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getRawMagnetometer(&magnetometer, timestamp));

	memcpy(magnetometer3, magnetometer.data, 12);

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getTemperatureC(tss_device_id sensor_id, float* temp, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	float tempature;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getTemperatureC(&tempature, timestamp));

	*temp = tempature;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getTemperatureF(tss_device_id sensor_id, float* temp, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	float tempature;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getTemperatureF(&tempature, timestamp));

	*temp = tempature;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getConfidenceFactor(tss_device_id sensor_id, float* confindence, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	float confind;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getConfidenceFactor(&confind, timestamp));

	*confindence = confind;

	return TSS_NO_ERROR;
}

//Data-Logging Commands
TSS_EXPORT TSS_ERROR tss_sensor_turnOnMassStorage(tss_device_id sensor_id, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->turnOnMassStorage(timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_turnOffMassStorage(tss_device_id sensor_id, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->turnOffMassStorage(timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_formatAndInitializeSDCard(tss_device_id sensor_id, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->formatAndInitializeSDCard(timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_beginDataLoggingSession(tss_device_id sensor_id, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->beginDataLoggingSession(timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_endDataLoggingSession(tss_device_id sensor_id, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->endDataLoggingSession(timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setClockValues(tss_device_id sensor_id, U8 month, U8 day, U8 year, U8 hour, U8 minute, U8 second, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setClockValues(month, day, year, hour, minute, second, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getClockValues(tss_device_id sensor_id, U8* month, U8* day, U8* year, U8* hour, U8* minute, U8* second, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	U8 mon,d,yr,hr,min,sec;


	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getClockValues(&mon, &d, &yr, &hr, &min, &sec, timestamp));

	*month = mon;
	*day = d;
	*year = yr;
	*hour = hr;
	*minute = min;
	*second = sec;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_updateCurrentTimestamp(tss_device_id sensor_id, U32 set_timestamp, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->updateCurrentTimestamp(set_timestamp, timestamp));

	return TSS_NO_ERROR;
}

//Configuration Write Commands
TSS_EXPORT TSS_ERROR tss_sensor_setEulerAngleDecompositionOrder(tss_device_id sensor_id, U8 order, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setEulerAngleDecompositionOrder(order, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setMagnetoresistiveThreshold(tss_device_id sensor_id, float threshold, U32 trust_frames, float lockout_decay, float perturbation_detection_value, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setMagnetoresistiveThreshold(threshold, trust_frames, lockout_decay, perturbation_detection_value, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setAccelerometerResistanceThreshold(tss_device_id sensor_id, float threshold, U32 lockout_frames, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setAccelerometerResistanceThreshold(threshold, lockout_frames, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_offsetWithCurrentOrientation(tss_device_id sensor_id, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->offsetWithCurrentOrientation(timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_resetBaseOffset(tss_device_id sensor_id, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->resetBaseOffset(timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_offsetWithQuaternion(tss_device_id sensor_id, const float* quat4, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	Orient quat;
	memcpy(quat.data, quat4, 16);

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->offsetWithQuaternion(&quat, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_tareWithQuaternion(tss_device_id sensor_id, const float* quat4, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	Orient quat;
	memcpy(quat.data, quat4, 16);
	TSS_ERROR_CHECK(stored_sensors[sensor_id]->tareWithQuaternion(&quat, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_tareWithRotationMatrix(tss_device_id sensor_id, const float* matrix9, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->tareWithRotationMatrix((float*)matrix9, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setStaticAccelerometerTrustValue(tss_device_id sensor_id, float trust_value, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setStaticAccelerometerTrustValue(trust_value, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setConfidenceAccelerometerTrustValues(tss_device_id sensor_id, float min_trust_value, float max_trust_value, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setConfidenceAccelerometerTrustValues(min_trust_value, max_trust_value, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setStaticCompassTrustValue(tss_device_id sensor_id, float trust_value, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setStaticCompassTrustValue(trust_value, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setConfidenceCompassTrustValues(tss_device_id sensor_id, float min_trust_value, float max_trust_value, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setConfidenceCompassTrustValues(min_trust_value, max_trust_value, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setDesiredUpdateRate(tss_device_id sensor_id, U32 update_rate, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setDesiredUpdateRate(update_rate, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setReferenceVectorMode(tss_device_id sensor_id, U8 mode, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setReferenceVectorMode(mode, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setOversampleRate(tss_device_id sensor_id, U16 gyro_sample_rate, U16 accel_sample_rate, U16 compass_sample_rate, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setOversampleRate(gyro_sample_rate, accel_sample_rate, compass_sample_rate, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setGyroscopeEnabled(tss_device_id sensor_id, U8 enabled, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setGyroscopeEnabled(enabled, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setAccelerometerEnabled(tss_device_id sensor_id, U8 enabled, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setAccelerometerEnabled(enabled, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setCompassEnabled(tss_device_id sensor_id, U8 enabled, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setCompassEnabled(enabled, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setOrientationSmoothing(tss_device_id sensor_id, float enabled, float compass_max_smooth_factor, float compass_min_smooth_factor, float accelerometer_max_smooth_factor, float accelerometer_min_smooth_factor, float smoothing_magnification, float smoothing_offset, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setOrientationSmoothing(enabled, compass_max_smooth_factor, compass_min_smooth_factor, accelerometer_max_smooth_factor, accelerometer_min_smooth_factor, smoothing_magnification, smoothing_offset, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getOrientationSmoothing(tss_device_id sensor_id, U8* enabled, float* compass_max_smooth_factor, float* compass_min_smooth_factor, float* accelerometer_max_smooth_factor, float* accelerometer_min_smooth_factor, float* smoothing_magnification, float* smoothing_offset, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	U8 en;
	float comMax;
	float comMin;
	float accelMax;
	float accelMin;
	float smoothMag;
	float smoothOff;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getOrientationSmoothing(&en, &comMax, &comMin, &accelMax, &accelMin, &smoothMag, &smoothOff, timestamp));

	*enabled = en;
	*compass_max_smooth_factor = comMax;
	*compass_min_smooth_factor = comMin;
	*accelerometer_max_smooth_factor = accelMax;
	*accelerometer_min_smooth_factor = accelMin;
	*smoothing_magnification = smoothMag;
	*smoothing_offset = smoothOff;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setAxisDirections(tss_device_id sensor_id, U8 axis_direction_byte, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setAxisDirections(axis_direction_byte, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setRunningAveragePercent(tss_device_id sensor_id, float gyro_running_average_percent, float accel_running_average_percent, float mag_running_average_percent, float orient_running_average_percent, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setRunningAveragePercent(gyro_running_average_percent, accel_running_average_percent, mag_running_average_percent, orient_running_average_percent, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setCompassReferenceVector(tss_device_id sensor_id, const float* reference_vector3, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	Vector3 ref_vector;
	memcpy(ref_vector.data, reference_vector3, 12);
	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setCompassReferenceVector(&ref_vector, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setAccelerometerReferenceVector(tss_device_id sensor_id, const float* reference_vector3, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	Vector3 ref_vector;
	memcpy(ref_vector.data, reference_vector3, 12);
	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setAccelerometerReferenceVector(&ref_vector, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_resetKalmanFilter(tss_device_id sensor_id, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->resetKalmanFilter(timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setAccelerometerRange(tss_device_id sensor_id, U8 accelerometer_range_setting, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setAccelerometerRange(accelerometer_range_setting, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setFilterMode(tss_device_id sensor_id, U8 mode, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setFilterMode(mode, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setRunningAverageMode(tss_device_id sensor_id, U8 mode, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setRunningAverageMode(mode, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setGyroscopeRange(tss_device_id sensor_id, U8 gyroscope_range_setting, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setGyroscopeRange(gyroscope_range_setting, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setCompassRange(tss_device_id sensor_id, U8 compass_range_setting, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setCompassRange(compass_range_setting, timestamp));

	return TSS_NO_ERROR;
}

// Configuration Read Commands
TSS_EXPORT TSS_ERROR tss_sensor_getTareAsQuaternion(tss_device_id sensor_id, float* quat4, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	Orient quat;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getTareAsQuaternion(&quat, timestamp));

	memcpy(quat4, quat.data, 16);

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getTareAsRotationMatrix(tss_device_id sensor_id, float* matrix9, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	float mat[9];

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getTareAsRotationMatrix(mat, timestamp));

	memcpy(matrix9, mat, 36);

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getAccelerometerTrustValues(tss_device_id sensor_id, float* min_trust_value, float* max_trust_value, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	float min;
	float max;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getAccelerometerTrustValues(&min, &max, timestamp));

	*min_trust_value = min;
	*max_trust_value = max;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getCompassTrustValues(tss_device_id sensor_id, float* min_trust_value, float* max_trust_value, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	float min;
	float max;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getCompassTrustValues(&min, &max, timestamp));

	*min_trust_value = min;
	*max_trust_value = max;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getCurrentUpdateRate(tss_device_id sensor_id, U32* last_update, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	U32 update;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getCurrentUpdateRate(&update, timestamp));

	*last_update = update;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getCompassReferenceVector(tss_device_id sensor_id, float* reference_vector, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	float ref_vector[3];

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getCompassReferenceVector(ref_vector, timestamp));

	memcpy(reference_vector, ref_vector, 12);

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getAccelerometerReferenceVector(tss_device_id sensor_id, float* reference_vector, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	float ref_vector[3];

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getAccelerometerReferenceVector(ref_vector, timestamp));

	memcpy(reference_vector, ref_vector, 12);

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getReferenceVectorMode(tss_device_id sensor_id, U8* mode, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	U8 m;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getReferenceVectorMode(&m, timestamp));

	*mode = m;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getGyroscopeEnabledState(tss_device_id sensor_id, U8* enabled, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	U8 e;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getGyroscopeEnabledState(&e, timestamp));

	*enabled = e;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getAccelerometerEnabledState(tss_device_id sensor_id, U8* enabled, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	U8 e;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getAccelerometerEnabledState(&e, timestamp));

	*enabled = e;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getCompassEnabledState(tss_device_id sensor_id, U8* enabled, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	U8 e;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getCompassEnabledState(&e, timestamp));

	*enabled = e;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getAxisDirections(tss_device_id sensor_id, U8* axis_directions, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	U8 axis_dir;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getAxisDirections(&axis_dir, timestamp));

	*axis_directions = axis_dir;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getOversampleRate(tss_device_id sensor_id, U16* gyro_sample_rate, U16* accel_sample_rate, U16* compass_sample_rate, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	U16 gRate;
	U16 aRate;
	U16 cRate;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getOversampleRate(&gRate, &aRate, &cRate, timestamp));

	*gyro_sample_rate = gRate;
	*accel_sample_rate = aRate;
	*compass_sample_rate = cRate;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getRunningAveragePercent(tss_device_id sensor_id, float* gyro_running_average_percent, float* accel_running_average_percent, float* mag_running_average_percent, float* orient_running_average_percent, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	float gyro_average, accel_average, mag_average, orient_average;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getRunningAveragePercent(&gyro_average, &accel_average, &mag_average, &orient_average, timestamp));

	*gyro_running_average_percent = gyro_average;
	*accel_running_average_percent = accel_average;
	*mag_running_average_percent = mag_average;
	*orient_running_average_percent = orient_average;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getDesiredUpdateRate(tss_device_id sensor_id, U32* update_rate, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	U32 rate;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getDesiredUpdateRate(&rate, timestamp));

	*update_rate = rate;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getAccelerometerRange(tss_device_id sensor_id, U8* accelerometer_range_setting, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	U8 range;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getAccelerometerRange(&range, timestamp));

	*accelerometer_range_setting = range;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getFilterMode(tss_device_id sensor_id, U8* mode, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	U8 m;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getFilterMode(&m, timestamp));

	*mode = m;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getRunningAverageMode(tss_device_id sensor_id, U8* mode, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	U8 m;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getRunningAverageMode(&m, timestamp));

	*mode = m;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getGyroscopeRange(tss_device_id sensor_id, U8* gyroscope_range_setting, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	U8 range;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getGyroscopeRange(&range, timestamp));

	*gyroscope_range_setting = range;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getCompassRange(tss_device_id sensor_id, U8* compass_range_setting, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	U8 range;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getCompassRange(&range, timestamp));

	*compass_range_setting = range;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getEulerAngleDecompositionOrder(tss_device_id sensor_id, U8* order, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	U8 o;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getEulerAngleDecompositionOrder(&o, timestamp));

	*order = o;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getMagnetoresistiveThreshold(tss_device_id sensor_id, float* threshold, U32* trust_frames, float* lockout_decay, float* perturbation_detection_value, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	float thres;
	U32 trust;
	float lockout;
	float perturbation;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getMagnetoresistiveThreshold(&thres, &trust, &lockout, &perturbation, timestamp));

	*threshold = thres;
	*trust_frames = trust;
	*lockout_decay = lockout;
	*perturbation_detection_value = perturbation;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getAccelerometerResistanceThreshold(tss_device_id sensor_id, float* threshold, U32* lockout_frames, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	float thres;
	U32 lockout;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getAccelerometerResistanceThreshold(&thres, &lockout, timestamp));

	*threshold = thres;
	*lockout_frames = lockout;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getOffsetOrientationAsQuaternion(tss_device_id sensor_id, float* quat4, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	Orient quat;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getOffsetOrientationAsQuaternion(&quat, timestamp));

	memcpy(quat4, quat.data, 16);

	return TSS_NO_ERROR;
}

//Calibration Commands
TSS_EXPORT TSS_ERROR tss_sensor_setCompassCalibrationCoefficients(tss_device_id sensor_id, float* matrix9, float* bias3, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	Vector3 bias;
	memcpy(bias.data, bias3, 12);

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setCompassCalibrationCoefficients((float*)matrix9, &bias, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setAccelerometerCalibrationCoefficients(tss_device_id sensor_id, const float* matrix9, const float* bias3, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	Vector3 bias;
	memcpy(bias.data, bias3, 12);

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setAccelerometerCalibrationCoefficients((float*)matrix9, &bias, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getCompassCalibrationCoefficients(tss_device_id sensor_id, float* matrix9, float* bias3, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	float matrix[9];
	Vector3 bias;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getCompassCalibrationCoefficients(matrix, &bias, timestamp));

	memcpy(matrix9, matrix, 36);
	memcpy(bias3, bias.data, 12);


	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getAccelerometerCalibrationCoefficients(tss_device_id sensor_id, float* matrix9, float* bias3, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	float matrix[9];
	Vector3 bias;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getAccelerometerCalibrationCoefficients(matrix, &bias, timestamp));

	memcpy(matrix9, matrix, 36);
	memcpy(bias3, bias.data, 12);


	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getGyroscopeCalibrationCoefficients(tss_device_id sensor_id, float* matrix9, float* bias3, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	float matrix[9];
	Vector3 bias;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getGyroscopeCalibrationCoefficients(matrix, &bias, timestamp));

	memcpy(matrix9, matrix, 36);
	memcpy(bias3, bias.data, 12);


	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_beginGyroscopeAutoCalibration(tss_device_id sensor_id, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->beginGyroscopeAutoCalibration(timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setGyroscopeCalibrationCoefficients(tss_device_id sensor_id, const float* matrix9, const float* bias3, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	Vector3 bias;
	memcpy(bias.data, bias3, 12);

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setGyroscopeCalibrationCoefficients((float*)matrix9, &bias, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setCalibrationMode(tss_device_id sensor_id, U8 mode, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setCalibrationMode(mode, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getCalibrationMode(tss_device_id sensor_id, U8* mode, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	U8 m;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getCalibrationMode(&m, timestamp));

	*mode = m;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setAutoCompassCalibrationMode(tss_device_id sensor_id, U8 mode, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setAutoCompassCalibrationMode(mode, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getAutoCompassCalibrationMode(tss_device_id sensor_id, U8* mode, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	U8 l_mode;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getAutoCompassCalibrationMode(&l_mode, timestamp));

	*mode = l_mode;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setAutoCompassCalibrationSettings(tss_device_id sensor_id, float too_close_angle, float bias_movement_percentage, float coplanarity_tolerance, U8 max_averages, U8 max_bad_deviations, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setAutoCompassCalibrationSettings(too_close_angle, bias_movement_percentage, coplanarity_tolerance, max_averages, max_bad_deviations, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getAutoCompassCalibrationSettings(tss_device_id sensor_id, float* too_close_angle, float* bias_movement_percentage, float* coplanarity_tolerance, U8* max_averages, U8* max_bad_deviations, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	float l_too_close_angle;
	float l_bias_movement_percentage;
	float l_coplanarity_tolerance;	
	U8 l_max_averages;
	U8 l_max_bad_deviations;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getAutoCompassCalibrationSettings(&l_too_close_angle, &l_bias_movement_percentage, &l_coplanarity_tolerance, &l_max_averages, &l_max_bad_deviations, timestamp));
	
	*too_close_angle = l_too_close_angle;
	*bias_movement_percentage = l_bias_movement_percentage;
	*coplanarity_tolerance = l_coplanarity_tolerance;	
	*max_averages = l_max_averages;
	*max_bad_deviations = l_max_bad_deviations;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getAutoCompassCalibrationCount(tss_device_id sensor_id, U8* count, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	U8 l_count;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getAutoCompassCaibrationCount(&l_count, timestamp));

	*count = l_count;

	return TSS_NO_ERROR;
}

/*
TSS_EXPORT TSS_ERROR tss_sensor_setOrthoCalibrationDataPointFromCurrentOrientation(tss_device_id sensor_id, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setOrthoCalibrationDataPointFromCurrentOrientation(timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setOrthoCalibrationDataPointFromVector(tss_device_id sensor_id, U8 type, U8 index, const float* vector3, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	Vector3 vec;
	memcpy(vec.data, vector3, 12);

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setOrthoCalibrationDataPointFromVector(type, index, &vec, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getOrthoCalibrationDataPoint(tss_device_id sensor_id, U8 type, U8 index, float* vector3, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	Vector3 vec;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getOrthoCalibrationDataPoint(type, index, &vec, timestamp));

	memcpy(vector3, vec.data, 16);

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_performOrthoCalibration(tss_device_id sensor_id, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->performOrthoCalibration(timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_clearOrthoCalibrationData(tss_device_id sensor_id, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->clearOrthoCalibrationData(timestamp));

	return TSS_NO_ERROR;
}
*/

TSS_EXPORT TSS_ERROR tss_sensor_getWirelessAddress(tss_device_id sensor_id, U16* address, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	U16 add;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getWirelessAddress(&add, timestamp));

	*address = add;

	return TSS_NO_ERROR;
}

//Battery Commands
TSS_EXPORT TSS_ERROR tss_sensor_getBatteryVoltage(tss_device_id sensor_id, float* battery_voltage, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	float battery;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getBatteryVoltage(&battery, timestamp));

	*battery_voltage = battery;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getBatteryPercentRemaining(tss_device_id sensor_id, U8* battery_percent, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	U8 battery;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getBatteryPercentRemaining(&battery, timestamp));

	*battery_percent = battery;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getBatteryStatus(tss_device_id sensor_id, U8* battery_charge_status, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	U8 battery;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getBatteryStatus(&battery, timestamp));

	*battery_charge_status = battery;

	return TSS_NO_ERROR;
}

//General Commands
TSS_EXPORT TSS_ERROR tss_sensor_setLEDMode(tss_device_id sensor_id, U8 mode, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getBatteryPercentRemaining(&mode, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getLEDMode(tss_device_id sensor_id, U8* mode, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	U8 m;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getLEDMode(&m, timestamp));

	*mode = m;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_restoreFactorySettings(tss_device_id sensor_id, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->restoreFactorySettings(timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_softwareReset(tss_device_id sensor_id)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->softwareReset());

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setSleepMode(tss_device_id sensor_id, U8 mode, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setSleepMode(mode, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getSleepMode(tss_device_id sensor_id, U8* mode, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	U8 m;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getSleepMode(&m, timestamp));

	*mode = m;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_enterBootloaderMode(tss_device_id sensor_id, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->_enterBootloader(timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setUARTBaudRate(tss_device_id sensor_id, U32 baud_rate, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setUARTBaudRate(baud_rate, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getUARTBaudRate(tss_device_id sensor_id, U32* baud_rate, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	U32 baud;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getUARTBaudRate(&baud, timestamp));

	*baud_rate = baud;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setUSBMode(tss_device_id sensor_id, U8 mode, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setUSBMode(mode, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getUSBMode(tss_device_id sensor_id, U8* mode, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	U8 m;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getUSBMode(&m, timestamp));

	*mode = m;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setLEDColor(tss_device_id sensor_id, const float* rgb_color3, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	Vector3 rgb;
	
	memcpy(rgb.data, rgb_color3, 12);

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setLEDColor(&rgb, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getLEDColor(tss_device_id sensor_id, float* rgb_color3, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	Vector3 rgb;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getLEDColor(&rgb, timestamp));

	memcpy(rgb_color3, &rgb.data, 12);

	return TSS_NO_ERROR;
}

//Wired HID Commands
TSS_EXPORT TSS_ERROR tss_sensor_setJoystickEnabled(tss_device_id sensor_id, U8 enabled_state, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setJoystickEnabled(enabled_state, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setMouseEnabled(tss_device_id sensor_id, U8 enabled_state, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setMouseEnabled(enabled_state, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getJoystickEnabled(tss_device_id sensor_id, U8* enabled_state, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	U8 enabled;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getJoystickEnabled(&enabled, timestamp));

	*enabled_state = enabled;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getMouseEnabled(tss_device_id sensor_id, U8* enabled_state, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	U8 enabled;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getMouseEnabled(&enabled, timestamp));

	*enabled_state = enabled;

	return TSS_NO_ERROR;
}

//General HID Commands
TSS_EXPORT TSS_ERROR tss_sensor_setControlMode(tss_device_id sensor_id, U8 control_class, U8 control_index, U8 handler_index, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setControlMode( control_class, control_index, handler_index, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setControlData(tss_device_id sensor_id, U8 control_class, U8 control_index, U8 data_point_index, float data_point, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setControlData(control_class, control_index, data_point_index, data_point, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getControlMode(tss_device_id sensor_id, U8 control_class, U8 control_index, U8* handler_index, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	U8 handler;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getControlMode(control_class, control_index, &handler, timestamp));

	*handler_index = handler;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getControlData(tss_device_id sensor_id, U8 control_class, U8 control_index, U8 data_point_index, float* data_point, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	float data;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getControlData(control_class, control_index, data_point_index, &data, timestamp));

	*data_point = data;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setButtonGyroDisableLength(tss_device_id sensor_id, U8 number_of_frames, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setButtonGyroDisableLength(number_of_frames, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getButtonGyroDisableLength(tss_device_id sensor_id, U8* number_of_frames, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	U8 number;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getButtonGyroDisableLength(&number, timestamp));

	*number_of_frames = number;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getButtonState(tss_device_id sensor_id, U8* button_state, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	U8 button;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getButtonState(&button, timestamp));

	*button_state = button;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setMouseAbsoluteRelativeMode(tss_device_id sensor_id, U8 mode, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setMouseAbsoluteRelativeMode(mode, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getMouseAbsoluteRelativeMode(tss_device_id sensor_id, U8* mode, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	U8 m;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getMouseAbsoluteRelativeMode(&m, timestamp));

	*mode = m;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setJoystickAndMousePresentRemoved(tss_device_id sensor_id, U8 joystick, U8 mouse, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setJoystickAndMousePresentRemoved(joystick, mouse, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getJoystickAndMousePresentRemoved(tss_device_id sensor_id, U8* joystick, U8* mouse, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	U8 joy, m;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getJoystickAndMousePresentRemoved(&joy, &m, timestamp));

	*joystick = joy;
	*mouse = m;

	return TSS_NO_ERROR;
}


TSS_EXPORT TSS_ERROR tss_sensor_enablePedestrianTracking(tss_device_id sensor_id, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setPedestrianTrackingState(1, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_disablePedestrianTracking(tss_device_id sensor_id, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setPedestrianTrackingState(0, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getPedestrianTrackingEnabledState(tss_device_id sensor_id, U8* enabled, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	U8 e;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getPedestrianTrackingState(&e, timestamp));

	*enabled = e;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setSelectedStepIndex(tss_device_id sensor_id, U16 index, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setSelectedStepIndex(index, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setStepRatioMinimum(tss_device_id sensor_id, float min_ratio, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setStepRatioMinimum(min_ratio, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getStepRatioMinimum(tss_device_id sensor_id, float* min_ratio, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	float min;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getStepRatioMinimum(&min, timestamp));

	*min_ratio = min;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setStepRatioMaximum(tss_device_id sensor_id, float max_ratio, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setStepRatioMaximum(max_ratio, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getStepRatioMaximum(tss_device_id sensor_id, float* max_ratio, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	float max;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getStepRatioMaximum(&max, timestamp));

	*max_ratio = max;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setStepDutyCycleMinimum(tss_device_id sensor_id, float min_duty_cycle, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setStepDutyCycleMinimum(min_duty_cycle, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getStepDutyCycleMinimum(tss_device_id sensor_id, float* min_duty_cycle, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	float min;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getStepDutyCycleMinimum(&min, timestamp));

	*min_duty_cycle = min;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setStepDutyCycleMaximum(tss_device_id sensor_id, float max_duty_cycle, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setStepDutyCycleMaximum(max_duty_cycle, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getStepDutyCycleMaximum(tss_device_id sensor_id, float* max_duty_cycle, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	float max;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getStepDutyCycleMaximum(&max, timestamp));

	*max_duty_cycle = max;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setStepAmplitudeMinimum(tss_device_id sensor_id, float min_amplitude, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setStepAmplitudeMinimum(min_amplitude, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getStepAmplitudeMinimum(tss_device_id sensor_id, float* min_amplitude, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	float min;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getStepAmplitudeMinimum(&min, timestamp));

	*min_amplitude = min;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setStepAmplitudeMaximum(tss_device_id sensor_id, float max_amplitude, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setStepAmplitudeMaximum(max_amplitude, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getStepAmplitudeMaximum(tss_device_id sensor_id, float* max_amplitude, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	float max;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getStepAmplitudeMaximum(&max, timestamp));

	*max_amplitude = max;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setStepDurationMinimum(tss_device_id sensor_id, float min_step_duration, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setStepDurationMinimum(min_step_duration, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getStepDurationMinimum(tss_device_id sensor_id, float* min_step_duration, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	float min;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getStepDurationMinimum(&min, timestamp));

	*min_step_duration = min;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setStepDurationMaximum(tss_device_id sensor_id, float max_step_duration, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setStepDurationMaximum(max_step_duration, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getStepDurationMaximum(tss_device_id sensor_id, float* max_step_duration, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	float max;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getStepDurationMaximum(&max, timestamp));

	*max_step_duration = max;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setStepStrideSlope(tss_device_id sensor_id, float slope, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setStepStrideSlope(slope, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getStepStrideSlope(tss_device_id sensor_id, float* slope, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	float s;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getStepStrideSlope(&s, timestamp));

	*slope = s;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setStepStrideOffset(tss_device_id sensor_id, float offset, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setStepStrideOffset(offset, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getStepStrideOffset(tss_device_id sensor_id, float* offset, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	float off;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getStepStrideOffset(&off, timestamp));

	*offset = off;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setPedestrianTrackingUnits(tss_device_id sensor_id, U8 unit_select, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setPedestrianTrackingUnits(unit_select, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getPedestrianTrackingUnits(tss_device_id sensor_id, U8* unit_select, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	U8 unit;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getPedestrianTrackingUnits(&unit, timestamp));

	*unit_select = unit;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_setAltitudeOffset(tss_device_id sensor_id, float offset, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->setBarometerAltitudeOffset(offset, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_autoSetAltitudeOffset(tss_device_id sensor_id, float known_height, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->autoSetBarometerAltitudeOffset(known_height, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getAltitudeOffset(tss_device_id sensor_id, float* offset, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	float off;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getAltitudeOffset(&off, timestamp));

	*offset = off;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getCurrentAltitude(tss_device_id sensor_id, float* altitude, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	float alt;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getCurrentAltitude(&alt, timestamp));

	*altitude = alt;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getFirstAltitude(tss_device_id sensor_id, float* first_altitude, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	float first;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getFirstAltitude(&first, timestamp));

	*first_altitude = first;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getAltitudeDifference(tss_device_id sensor_id, float* altitude_difference, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	float diff;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getAltitudeDifference(&diff, timestamp));

	*altitude_difference = diff;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getCurrentPressure(tss_device_id sensor_id, float* pressure, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	float p;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getCurrentPressure(&p, timestamp));

	*pressure = p;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getStepSessionConfidenceValue(tss_device_id sensor_id, float* confidence, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	float con;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getStepSessionConfidenceValue(&con, timestamp));

	*confidence = con;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getSelectedStepIndex(tss_device_id sensor_id, U16* index, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	U16 ind;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getSelectedStepIndex(&ind, timestamp));

	*index = ind;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getCurrentHeading(tss_device_id sensor_id, float* heading, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	float h;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getCurrentHeading(&h, timestamp));

	*heading = h;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getStepBufferLength(tss_device_id sensor_id, U16* step_buffer_length, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	U16 length;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getStepBufferLength(&length, timestamp));

	*step_buffer_length = length;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getTotalDistanceTraveled(tss_device_id sensor_id, float* distance, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	float dist;

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getTotalDistanceTraveled(&dist, timestamp));

	*distance = dist;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getLatestStep(tss_device_id sensor_id, float* step_data, U32* timestamp)
{
	SENSOR_RANGE_CHECK();
	
	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getLatestStep(step_data, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getStepAtSelectedIndex(tss_device_id sensor_id, float* step_data, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->getStepAtSelectedIndex(step_data, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_resetSteps(tss_device_id sensor_id, U32* timestamp)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->resetSteps(timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_resetPort(tss_device_id sensor_id, U8* port_open)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->_portReset(port_open));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_writePage(tss_device_id sensor_id, const char* page_data)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->_writePage(page_data));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_enterFirmware(tss_device_id sensor_id)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->_enterFirmware());

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_getPageSize(tss_device_id sensor_id, U16* page_size)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->_getPageSize(page_size));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_startFirmwareUpdate(tss_device_id sensor_id, const char* address)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->_startFirmwareUpdate(address));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_stopFirmwareUpdate(tss_device_id sensor_id)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->_stopFirmwareUpdate());

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_autoFirmwareUpdateFromFile(tss_device_id sensor_id, const char* file_name)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->_autoFirmwareUpdateFromFile(file_name));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_autoFirmwareUpdateFromString(tss_device_id sensor_id, const char* update_contents)
{
	SENSOR_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_sensors[sensor_id]->_autoFirmwareUpdateFromString(update_contents));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_manualWrite(tss_device_id sensor_id, U8* bytes, U16 amount, U32* timestamp)
{
	SENSOR_RANGE_CHECK();	
		
	TSS_ERROR_CHECK(stored_sensors[sensor_id]->manualWrite(bytes, amount, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_manualRead(tss_device_id sensor_id, U8* bytes, U16 amount, U16* read_count, U32* timestamp)
{
	SENSOR_RANGE_CHECK();	
	
	TSS_ERROR_CHECK(stored_sensors[sensor_id]->manualRead(bytes, amount, read_count, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_sensor_manualFlush(tss_device_id sensor_id, U32* timestamp)
{
	SENSOR_RANGE_CHECK();
	
	TSS_ERROR_CHECK(stored_sensors[sensor_id]->manualFlush(timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_createDongle(const char* port_name, tss_device_id* out_id)
{
	TssDongle* dongle = new TssDongle(port_name);

	if (!dongle->isConnected())
	{
		delete dongle;
		return TSS_ERROR_CANT_OPEN_PORT;
	}

	*out_id = stored_dongles.size();
	stored_dongles.push_back(unique_ptr<TssDongle>(dongle));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_removeDongle(tss_device_id dongle_id)
{
	DONGLE_RANGE_CHECK();

	if (stored_dongles[dongle_id]->_isStreaming)
	{
		gAPI.unregisterStreamingDevice(stored_dongles[dongle_id].get());
	}

	stored_dongles[dongle_id].reset();

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_isConnected(tss_device_id dongle_id)
{
	DONGLE_RANGE_CHECK();

	if (!stored_dongles[dongle_id]->isConnected())
	{
		return TSS_ERROR_NOT_CONNECTED;
	}

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_inBootloader(tss_device_id dongle_id, U8* in_bootloader)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->_bootloaderCheck(in_bootloader));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_enterBootloaderMode(tss_device_id dongle_id, U32* timestamp)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->_enterBootloader(timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_getLastSensorTimestamp(tss_device_id dongle_id, U32* timestamp)
{
	DONGLE_RANGE_CHECK();

	*timestamp = stored_dongles[dongle_id]->getLastHeader()->SensorTimestamp;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_getLastSystemTimestamp(tss_device_id dongle_id, float* timestamp)
{
	DONGLE_RANGE_CHECK();

	*timestamp = stored_dongles[dongle_id]->getLastHeader()->SystemTimestamp;

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_getSerialNumber(tss_device_id dongle_id, U32* serial_number, U32* timestamp)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->_readSerialNumber(timestamp));
	*serial_number = stored_dongles[dongle_id]->getSerialNumber();

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_getHardwareVersionString(tss_device_id dongle_id, char* version_string)
{
	DONGLE_RANGE_CHECK();

	string tmp = stored_dongles[dongle_id]->getHardwareVersionString();	
	for (int i = 0; i < 32; i++) version_string[i] = '\0';
	memcpy(version_string, tmp.c_str(), tmp.length());

	if (tmp.length())
		version_string[tmp.length() - 1] = '\0';

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_getFirmwareVersionString(tss_device_id dongle_id, char* version_string)
{
	DONGLE_RANGE_CHECK();

	U32 timestamp;

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->_readFirmwareString(&timestamp));
	string tmp = stored_dongles[dongle_id]->getFirmwareVersionString();	
	for (int i = 0; i < 32; i++) version_string[i] = '\0';
	memcpy(version_string, tmp.c_str(), tmp.length());
	
	if(tmp.length())
		version_string[tmp.length() - 1] = '\0';

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_openPort(tss_device_id dongle_id, const char* port_name)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->openPort(port_name));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_closePort(tss_device_id dongle_id)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->closePort());

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_enableTimestampsWireless(tss_device_id dongle_id)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->enableTimestampsWireless());

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_disableTimestampsWireless(tss_device_id dongle_id)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->disableTimestampsWireless());

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_manualWrite(tss_device_id dongle_id, U8* bytes, U16 amount, U32* timestamp)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->manualWrite(bytes, amount, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_manualRead(tss_device_id dongle_id, U8* bytes, U16 amount, U16* read_count, U32* timestamp)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->manualRead(bytes, amount, read_count, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_manualFlush(tss_device_id dongle_id, U32* timestamp)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->manualFlush(timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_resetPort(tss_device_id dongle_id, U8* port_open)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->_portReset(port_open));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_writePage(tss_device_id dongle_id, const char* page_data)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->_writePage(page_data));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_enterFirmware(tss_device_id dongle_id)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->_enterFirmware());

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_getPageSize(tss_device_id dongle_id, U16* page_size)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->_getPageSize(page_size));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_startFirmwareUpdate(tss_device_id dongle_id, const char* address)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->_startFirmwareUpdate(address));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_stopFirmwareUpdate(tss_device_id dongle_id)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->_stopFirmwareUpdate());

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_autoFirmwareUpdateFromFile(tss_device_id dongle_id, const char* file_name)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->_autoFirmwareUpdateFromFile(file_name));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_autoFirmwareUpdateFromString(tss_device_id dongle_id, const char* update_contents)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->_autoFirmwareUpdateFromString(update_contents));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_getWirelessSensor(tss_device_id dongle_id, U8 logical_id, tss_device_id* out_id)
{
	DONGLE_RANGE_CHECK();

	shared_ptr<TssSensor> sensor;

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->getWirelessSensor(logical_id, sensor));

	if (!sensor->isConnected())
	{
		return TSS_ERROR_NOT_CONNECTED;
	}

	*out_id = stored_sensors.size();
	stored_sensors.push_back(shared_ptr<TssSensor>(sensor));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_removeWirelessSensor(tss_device_id dongle_id, U8 logical_id)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->removeWirelessSensor(logical_id));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_enableAllSensorsAndStartStreaming(tss_device_id dongle_id, U32 data_flags, U32 interval, U32 duration)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->enableAllSensorsAndStartStreaming(data_flags, interval, duration));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_startStreaming(tss_device_id dongle_id)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->startStreaming());

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_stopStreaming(tss_device_id dongle_id)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->stopStreaming());

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_softwareReset(tss_device_id dongle_id)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->softwareReset());

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_setLEDMode(tss_device_id dongle_id, U8 mode, U32* timestamp)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->setLEDMode(mode, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_getLEDMode(tss_device_id dongle_id, U8* mode, U32* timestamp)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->getLEDMode(mode, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_setLEDColor(tss_device_id dongle_id, const float* rgb_color3, U32* timestamp)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->setLEDColor(rgb_color3, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_getLEDColor(tss_device_id dongle_id, float* rgb_color3, U32* timestamp)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->getLEDColor(rgb_color3, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_getWiredResponseHeaderBitfield(tss_device_id dongle_id, U32* bitfield, U32* timestamp)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->getWiredResponseHeaderBitfield(bitfield, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_setWiredResponseHeaderBitfield(tss_device_id dongle_id, U32 bitfield, U32* timestamp)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->setWiredResponseHeaderBitfield(bitfield, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_getWirelessChannel(tss_device_id dongle_id, U8* channel, U32* timestamp)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->getWirelessChannel(channel, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_setWirelessChannel(tss_device_id dongle_id, U8 channel, U32* timestamp)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->setWirelessChannel(channel, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_getWirelessPanID(tss_device_id dongle_id, U16* pan_id, U32* timestamp)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->getWirelessPanID(pan_id, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_setWirelessPanID(tss_device_id dongle_id, U16 pan_id, U32* timestamp)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->setWirelessPanID(pan_id, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_getSerialNumberAtLogicalID(tss_device_id dongle_id, U8 logical_id, U32* serial_number, U32* timestamp)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->getSerialNumberAtLogicalID(logical_id, serial_number, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_setSerialNumberAtLogicalID(tss_device_id dongle_id, U8 logical_id, U32 serial_number, U32* timestamp)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->setSerialNumberAtLogicalID(logical_id, serial_number, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_commitWirelessSettings(tss_device_id dongle_id, U32* timestamp)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->commitWirelessSettings(timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_getWirelessAddress(tss_device_id dongle_id, U16* address, U32* timestamp)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->getWirelessAddress(address, timestamp));

	return TSS_NO_ERROR;
}

//Dongle Commands
TSS_EXPORT TSS_ERROR tss_dongle_setWirelessStreamingAutoFlushMode(tss_device_id dongle_id, U8 mode, U32* timestamp)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->setWirelessStreamingAutoFlushMode(mode, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_getWirelessStreamingAutoFlushMode(tss_device_id dongle_id, U8* mode, U32* timestamp)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->getWirelessStreamingAutoFlushMode(mode, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_setWirelessStreamingManualFlushBitfield(tss_device_id dongle_id, U16 manual_flush_bitfield, U32* timestamp)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->setWirelessStreamingManualFlushBitfield(manual_flush_bitfield, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_getWirelessStreamingManualFlushBitfield(tss_device_id dongle_id, U16* manual_flush_bitfield, U32* timestamp)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->getWirelessStreamingManualFlushBitfield(manual_flush_bitfield, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_broadcastSynchronizationPulse(tss_device_id dongle_id, U32* timestamp)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->broadcastSynchronizationPulse(timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_getWirelessChannelNoiseLevels(tss_device_id dongle_id, U8* channel_strengths16, U32* timestamp)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->getWirelessChannelNoiseLevels(channel_strengths16, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_setWirelessRetries(tss_device_id dongle_id, U8 retries, U32* timestamp)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->setWirelessRetries(retries, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_getWirelessRetries(tss_device_id dongle_id, U8* retries, U32* timestamp)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->getWirelessRetries(retries, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_getWirelessSlotsOpen(tss_device_id dongle_id, U8* slots_open, U32* timestamp)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->getWirelessSlotsOpen(slots_open, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_getSignalStrength(tss_device_id dongle_id, U8* last_packet_signal_strength, U32* timestamp)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->getSignalStrength(last_packet_signal_strength, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_setWirelessResponseHeaderBitfield(tss_device_id dongle_id, U32 header_bitfield, U32* timestamp)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->setWirelessResponseHeaderBitfield(header_bitfield, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_getWirelessResponseHeaderBitfield(tss_device_id dongle_id, U32* header_bitfield, U32* timestamp)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->getWirelessResponseHeaderBitfield(header_bitfield, timestamp));

	return TSS_NO_ERROR;
}

//Wireless HID Commands
TSS_EXPORT TSS_ERROR tss_dongle_setWirelessHIDUpdateRate(tss_device_id dongle_id, U8 HID_update_rate, U32* timestamp)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->setWirelessHIDUpdateRate(HID_update_rate, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_getWirelessHIDUpdateRate(tss_device_id dongle_id, U8* HID_update_rate, U32* timestamp)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->getWirelessHIDUpdateRate(HID_update_rate, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_setWirelessHIDAsynchronousMode(tss_device_id dongle_id, U8 HID_communication_mode, U32* timestamp)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->setWirelessHIDAsynchronousMode(HID_communication_mode, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_getWirelessHIDAsynchronousMode(tss_device_id dongle_id, U8* HID_communication_mode, U32* timestamp)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->getWirelessHIDAsynchronousMode(HID_communication_mode, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_setJoystickLogicalID(tss_device_id dongle_id, U8 logical_ID, U32* timestamp)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->setJoystickLogicalID(logical_ID, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_setMouseLogicalID(tss_device_id dongle_id, U8 logical_ID, U32* timestamp)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->setMouseLogicalID(logical_ID, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_getJoystickLogicalID(tss_device_id dongle_id, U8* logical_ID, U32* timestamp)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->getJoystickLogicalID(logical_ID, timestamp));

	return TSS_NO_ERROR;
}

TSS_EXPORT TSS_ERROR tss_dongle_getMouseLogicalID(tss_device_id dongle_id, U8* logical_ID, U32* timestamp)
{
	DONGLE_RANGE_CHECK();

	TSS_ERROR_CHECK(stored_dongles[dongle_id]->getMouseLogicalID(logical_ID, timestamp));

	return TSS_NO_ERROR;
}

void tss_findSensorPorts(U32 find_flags)
{		
	stored_tss_ports = tssFindSensorPorts(find_flags);

	stored_port_index = 0;
}

TSS_ERROR tss_getNextSensorPort(char* port_name, U16* sensor_type, U8* connection_type)
{
	if (stored_port_index >= stored_tss_ports.size())
		return TSS_ERROR_COMMAND_FAILURE;

	memcpy(port_name, stored_tss_ports[stored_port_index].port_name.c_str(), 64);
	*sensor_type = stored_tss_ports[stored_port_index].device_type;
	*connection_type = stored_tss_ports[stored_port_index].connection_type;

	stored_port_index++;

	return TSS_NO_ERROR;
}
