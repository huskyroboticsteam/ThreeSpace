#include "threespace_sensor.hpp"
#include "threespace_api.hpp"
#include <fstream>
#include <iostream>

TssSensor::TssSensor()
{
	_type = TSS_DEVICE_TYPE_SENSOR;

	_ownerDongle = NULL;
	_streamInterval = 0;
	_streamDataSize = 0;
}

TssSensor::TssSensor(std::string port)
{
	_type = TSS_DEVICE_TYPE_SENSOR;

	_ownerDongle = NULL;
	_streamInterval = 0;
	_streamDataSize = 0;

	result = _openPort(port);

	if (result != TSS_NO_ERROR)
	{
		closePort();
		return;
	}

	U32 timestamp;
	_sendCommand(86, &timestamp); // stop streaming

	U8 in_bootloader = 0;
	result = _bootloaderCheck(&in_bootloader);
	if (result != TSS_NO_ERROR)
	{
		closePort();
		return;
	}

	if (in_bootloader != 0)
	{
		return;
	}

	result = _readSerialNumber(&timestamp);

	if (result != TSS_NO_ERROR)
	{
		closePort();
		return;
	}

	result = _readVersionString(&timestamp);

	if (result != TSS_NO_ERROR)
	{
		closePort();
		return;
	}

	result = _readFirmwareString(&timestamp);

	if (result != TSS_NO_ERROR)
	{
		closePort();
		return;
	}

	if (_sensorType == TSS_DONGLE)
	{
		closePort();
		return;
	}
}

TssSensor::TssSensor(const TssSensor& other)
{
	throw runtime_error("Deep copies of sensor objects not allowed.");
}

void TssSensor::operator =(const TssSensor& other)
{
	throw runtime_error("Deep copies of sensor objects not allowed.");
}

TSS_ERROR TssSensor::enableStreamingWireless(U32 data_flags, U32 interval, U32 duration, U32 delay)
{
	if (!_isWireless)
	{
		return TSS_ERROR_WIRELESS_ONLY;
	}

	BASIC_CALL_CHECK_TSS();

	return _prepareStreamingWireless(data_flags, interval, duration, delay);
}

TSS_ERROR TssSensor::disableStreamingWireless()
{
	if (!_isWireless)
	{
		return TSS_ERROR_WIRELESS_ONLY;
	}

	BASIC_CALL_CHECK_TSS();

	_streamDataSize = 0; //the only variable the dongle checks to see if a sensor wants to stream, set to 0 to exclude it

	return TSS_NO_ERROR;
}

void TssSensor::_addToStreamingSlots(U8 command, U8 nbytes, U8* command_buff, U8& curr_slot)
{
	if (curr_slot < TSS_NUM_STREAMING_SLOTS) //only add command to a slot if slots remain
	{
		command_buff[curr_slot + 1] = command;
		_streamingSlots[curr_slot] = command;
		_streamDataSize += nbytes;
		curr_slot++;
	}
}

TSS_ERROR TssSensor::_prepareStreaming(U32 data_flags, U32 interval, U32 duration, U32 delay)
{
	if (_isStreaming)
	{
		return TSS_ERROR_ALREADY_STREAMING;
	}

	U8 curr_slot = 0;
	U8 buff[256];

	//initialize streaming slots based on data flags
	//also initialize the streaming data size so mass reads can be performed
	_streamDataSize = 0;
	buff[0] = 80; //set streaming slots command

	_setResponseHeader(_responseHeaderFlags | TSS_RESPONSE_HEADER_CHECKSUM | TSS_RESPONSE_HEADER_DATA_LENGTH);

	//go through the flags one by one in order and put them in slots
	//we rely on the numerical values of the flags to detemin_ratioe what order
	//they are assigned to slots in
	if (data_flags & TSS_STREAM_TARED_ORIENTATION_AS_QUATERNION)
	{
		_addToStreamingSlots(0x00, 16, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_TARED_ORIENTATION_AS_EULER_ANGLES)
	{
		_addToStreamingSlots(0x01, 12, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_TARED_ORIENTATION_AS_ROTATION_MATRIX)
	{
		_addToStreamingSlots(0x02, 36, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_TARED_ORIENTATION_AS_AXIS_ANGLE)
	{
		_addToStreamingSlots(0x03, 16, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_TARED_ORIENTATION_AS_TWO_VECTOR)
	{
		_addToStreamingSlots(0x04, 24, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_UNTARED_ORIENTATION_AS_QUATERNION)
	{
		_addToStreamingSlots(0x06, 16, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_UNTARED_ORIENTATION_AS_EULER_ANGLES)
	{
		_addToStreamingSlots(0x07, 12, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_UNTARED_ORIENTATION_AS_ROTATION_MATRIX)
	{
		_addToStreamingSlots(0x08, 36, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_UNTARED_ORIENTATION_AS_AXIS_ANGLE)
	{
		_addToStreamingSlots(0x09, 16, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_UNTARED_ORIENTATION_AS_TWO_VECTOR)
	{
		_addToStreamingSlots(0x0A, 24, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_RAW_SENSOR_DATA)
	{
		_addToStreamingSlots(0x40, 36, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_RAW_GYROSCOPE_DATA)
	{
		_addToStreamingSlots(0x41, 12, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_RAW_ACCELEROMETER_DATA)
	{
		_addToStreamingSlots(0x42, 12, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_RAW_MAGNETOMETER_DATA)
	{
		_addToStreamingSlots(0x43, 12, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_NORMALIZED_SENSOR_DATA)
	{
		_addToStreamingSlots(0x20, 36, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_NORMALIZED_GYROSCOPE_DATA)
	{
		_addToStreamingSlots(0x21, 12, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_NORMALIZED_ACCELEROMETER_DATA)
	{
		_addToStreamingSlots(0x22, 12, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_NORMALIZED_MAGNETOMETER_DATA)
	{
		_addToStreamingSlots(0x23, 12, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_CORRECTED_SENSOR_DATA)
	{
		_addToStreamingSlots(0x25, 36, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_CORRECTED_GYROSCOPE_DATA)
	{
		_addToStreamingSlots(0x26, 12, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_CORRECTED_ACCELEROMETER_DATA)
	{
		_addToStreamingSlots(0x27, 12, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_CORRECTED_MAGNETOMETER_DATA)
	{
		_addToStreamingSlots(0x28, 12, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_LINEAR_ACCELERATION)
	{
		_addToStreamingSlots(0x29, 12, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_BATTERY_VOLTAGE)
	{
		_addToStreamingSlots(0xC9, 4, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_BATTERY_LEVEL)
	{
		_addToStreamingSlots(0xCA, 1, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_BATTERY_STATUS)
	{
		_addToStreamingSlots(0xCB, 1, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_CELSIUS_TEMPERATURE)
	{
		_addToStreamingSlots(0x2B, 4, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_FAHRENHEIT_TEMPERATURE)
	{
		_addToStreamingSlots(0x2C, 4, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_BUTTON_STATE)
	{
		_addToStreamingSlots(0xFA, 1, buff, curr_slot);
	}

	//fill out any unused slots with blank commands
	while (curr_slot < 8)
	{
		buff[curr_slot + 1] = 0xff;
		_streamingSlots[curr_slot] = 0xff;
		curr_slot++;
	}

	U32 timestamp;
	TSS_RETRY(_sendCommandBytes(buff, 9, &timestamp), TSS_START_STREAM_RETRIES);

	if (result != TSS_NO_ERROR)
		return result;

	//record the streaming interval, or a guess at the interval
	if (interval == 0)
		_streamInterval = 5000;
	else
		if (interval < 1000)
			_streamInterval = 1000;
		else
			_streamInterval = interval;


	//send stream timing command to sensor
	buff[0] = 82;
	memcpy(buff + 1, &interval, 4);
	memcpy(buff + 5, &duration, 4);
	memcpy(buff + 9, &delay, 4);

	yost::swap32(buff + 1);
	yost::swap32(buff + 5);
	yost::swap32(buff + 9);

	TSS_RETRY(_sendCommandBytes(buff, 13, &timestamp), TSS_START_STREAM_RETRIES);

	if (result != TSS_NO_ERROR)
		return result;

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::_prepareStreamingWireless(U32 data_flags, U32 interval, U32 duration, U32 delay)
{
	if (_isStreaming)
	{
		return TSS_ERROR_ALREADY_STREAMING;
	}

	U8 curr_slot = 0;
	U8 buff[256];

	//initialize streaming slots based on data flags
	//also initialize the streaming data size so mass reads can be performed
	_streamDataSize = 0;
	buff[0] = 80; //set streaming slots command

	//go through the flags one by one in order and put them in slots
	//we rely on the numerical values of the flags to detemin_ratioe what order
	//they are assigned to slots in
	if (data_flags & TSS_STREAM_TARED_ORIENTATION_AS_QUATERNION)
	{
		_addToStreamingSlots(0x00, 16, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_TARED_ORIENTATION_AS_EULER_ANGLES)
	{
		_addToStreamingSlots(0x01, 12, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_TARED_ORIENTATION_AS_ROTATION_MATRIX)
	{
		_addToStreamingSlots(0x02, 36, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_TARED_ORIENTATION_AS_AXIS_ANGLE)
	{
		_addToStreamingSlots(0x03, 16, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_TARED_ORIENTATION_AS_TWO_VECTOR)
	{
		_addToStreamingSlots(0x04, 24, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_UNTARED_ORIENTATION_AS_QUATERNION)
	{
		_addToStreamingSlots(0x06, 16, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_UNTARED_ORIENTATION_AS_EULER_ANGLES)
	{
		_addToStreamingSlots(0x07, 12, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_UNTARED_ORIENTATION_AS_ROTATION_MATRIX)
	{
		_addToStreamingSlots(0x08, 36, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_UNTARED_ORIENTATION_AS_AXIS_ANGLE)
	{
		_addToStreamingSlots(0x09, 16, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_UNTARED_ORIENTATION_AS_TWO_VECTOR)
	{
		_addToStreamingSlots(0x0A, 24, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_RAW_SENSOR_DATA)
	{
		_addToStreamingSlots(0x40, 36, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_RAW_GYROSCOPE_DATA)
	{
		_addToStreamingSlots(0x41, 12, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_RAW_ACCELEROMETER_DATA)
	{
		_addToStreamingSlots(0x42, 12, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_RAW_MAGNETOMETER_DATA)
	{
		_addToStreamingSlots(0x43, 12, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_NORMALIZED_SENSOR_DATA)
	{
		_addToStreamingSlots(0x20, 36, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_NORMALIZED_GYROSCOPE_DATA)
	{
		_addToStreamingSlots(0x21, 12, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_NORMALIZED_ACCELEROMETER_DATA)
	{
		_addToStreamingSlots(0x22, 12, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_NORMALIZED_MAGNETOMETER_DATA)
	{
		_addToStreamingSlots(0x23, 12, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_CORRECTED_SENSOR_DATA)
	{
		_addToStreamingSlots(0x25, 36, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_CORRECTED_GYROSCOPE_DATA)
	{
		_addToStreamingSlots(0x26, 12, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_CORRECTED_ACCELEROMETER_DATA)
	{
		_addToStreamingSlots(0x27, 12, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_CORRECTED_MAGNETOMETER_DATA)
	{
		_addToStreamingSlots(0x28, 12, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_LINEAR_ACCELERATION)
	{
		_addToStreamingSlots(0x29, 12, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_BATTERY_VOLTAGE)
	{
		_addToStreamingSlots(0xC9, 4, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_BATTERY_LEVEL)
	{
		_addToStreamingSlots(0xCA, 1, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_BATTERY_STATUS)
	{
		_addToStreamingSlots(0xCB, 1, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_CELSIUS_TEMPERATURE)
	{
		_addToStreamingSlots(0x2B, 4, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_FAHRENHEIT_TEMPERATURE)
	{
		_addToStreamingSlots(0x2C, 4, buff, curr_slot);
	}
	if (data_flags & TSS_STREAM_BUTTON_STATE)
	{
		_addToStreamingSlots(0xFA, 1, buff, curr_slot);
	}

	//fill out any unused slots with blank commands
	while (curr_slot < 8)
	{
		buff[curr_slot + 1] = 0xff;
		_streamingSlots[curr_slot] = 0xff;
		curr_slot++;
	}

	U32 timestamp;
	TSS_RETRY(_sendCommandBytes(buff, 9, &timestamp), TSS_START_STREAM_RETRIES);

	if (result != TSS_NO_ERROR)
		return result;

	//record the streaming interval, or a guess at the interval
	if (interval == 0)
		_streamInterval = 5000;
	else
		if (interval < 1000)
			_streamInterval = 1000;
		else
			_streamInterval = interval;


	//send stream timing command to sensor
	buff[0] = 82;
	memcpy(buff + 1, &interval, 4);
	memcpy(buff + 5, &duration, 4);
	memcpy(buff + 9, &delay, 4);

	yost::swap32(buff + 1);
	yost::swap32(buff + 5);
	yost::swap32(buff + 9);

	TSS_RETRY(_sendCommandBytes(buff, 13, &timestamp), TSS_START_STREAM_RETRIES);

	if (result != TSS_NO_ERROR)
		return result;

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::_triggerStreaming()
{
	_streamBuffer.clear();

	//start streaming
	U32 timestamp;
	TSS_RETRY(_sendCommand(85, &timestamp), TSS_START_STREAM_RETRIES);

	if (result != TSS_NO_ERROR)
		return result;

	_isStreaming = true;

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::openPort(std::string port)
{
	if (_port)
	{
		return TSS_ERROR_ALREADY_CONNECTED;
	}

	_openPort(port);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::closePort()
{
	if (_isWireless)
	{
		return TSS_ERROR_CANT_CLOSE_WIRELESS_PORT;
	}

	BASIC_CALL_CHECK_TSS();

	try
	{
		_port->close();
	}
	catch (...)
	{
		// We are closing and trying to avoid a crash, so do nothing but catch.
	}

	_port.reset();

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::enableTimestampsWired()
{
	if (_isWireless)
	{
		return TSS_ERROR_NOT_AVAILABLE_WIRELESS;
	}

	BASIC_CALL_CHECK_TSS();

	_setResponseHeader(_responseHeaderFlags | TSS_RESPONSE_HEADER_TIMESTAMP);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::disableTimestampsWired()
{
	if (_isWireless)
	{
		return TSS_ERROR_NOT_AVAILABLE_WIRELESS;
	}

	BASIC_CALL_CHECK_TSS();

	_setResponseHeader(_responseHeaderFlags & ~TSS_RESPONSE_HEADER_TIMESTAMP);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::startStreamingWired(U32 data_flags, U32 interval, U32 duration, U32 delay)
{
	if (_isWireless)
	{
		return TSS_ERROR_NO_WIRELESS_STREAMING_FROM_SENSOR;
	}

	TSS_ERROR_CHECK(_prepareStreaming(data_flags, interval, duration, delay));

	TSS_ERROR_CHECK(_triggerStreaming());

	_unparsedStreamData.empty();

	//register this device with the overall module so it can assign threading to it
	gAPI.registerStreamingDevice(this);

	return TSS_NO_ERROR;
}

void TssSensor::_parseStreamingPacketItem(U8* dest, U8* src, U16& src_index, U8 nbytes)
{
	memcpy(dest, src + src_index, nbytes);
	src_index += nbytes;
}

void TssSensor::_parseStreamingPacketItemFloats(float* dest, U8* src, U16& src_index, U8 nfloats)
{
	_parseFloats((float*)(src + src_index), nfloats);
	_parseStreamingPacketItem((U8*)dest, src, src_index, nfloats * 4);
}

TSS_ERROR TssSensor::_parseStreamingPacket(TSS_Stream_Packet* packet)
{
	U16 raw_data_index = 0;

	memcpy(packet->streaming_slots, _streamingSlots, 8);

	U8 i;
	for (i = 0; i < 8; i++)
	{
		if (_streamingSlots[i] == 0xff)
			break;

		//check through the slots one by one and pull out the raw data that belongs to that slot
		if (_streamingSlots[i] == 0x00)
		{
			_parseStreamingPacketItemFloats(packet->taredOrientQuat, packet->rawData, raw_data_index, 4);
		}
		else if (_streamingSlots[i] == 0x01)
		{
			_parseStreamingPacketItemFloats(packet->taredOrientEuler, packet->rawData, raw_data_index, 3);
		}
		else if (_streamingSlots[i] == 0x02)
		{
			_parseStreamingPacketItemFloats(packet->taredOrientMatrix, packet->rawData, raw_data_index, 9);
		}
		else if (_streamingSlots[i] == 0x03)
		{
			float data[4];
			_parseStreamingPacketItemFloats(data, packet->rawData, raw_data_index, 4);
			memcpy(packet->taredOrientAxis, data, 12);
			packet->taredOrientAngle = data[3];
		}
		else if (_streamingSlots[i] == 0x04)
		{
			float data[6];
			_parseStreamingPacketItemFloats(data, packet->rawData, raw_data_index, 6);
			memcpy(packet->taredOrientForward, data, 12);
			memcpy(packet->taredOrientDown, data + 3, 12);
		}
		else if (_streamingSlots[i] == 0x06)
		{
			_parseStreamingPacketItemFloats(packet->untaredOrientQuat, packet->rawData, raw_data_index, 4);
		}
		else if (_streamingSlots[i] == 0x07)
		{
			_parseStreamingPacketItemFloats(packet->untaredOrientEuler, packet->rawData, raw_data_index, 3);
		}
		else if (_streamingSlots[i] == 0x08)
		{
			_parseStreamingPacketItemFloats(packet->untaredOrientMatrix, packet->rawData, raw_data_index, 9);
		}
		else if (_streamingSlots[i] == 0x09)
		{
			float data[4];
			_parseStreamingPacketItemFloats(data, packet->rawData, raw_data_index, 4);
			memcpy(packet->untaredOrientAxis, data, 12);
			packet->untaredOrientAngle = data[3];
		}
		else if (_streamingSlots[i] == 0x0A)
		{
			float data[6];
			_parseStreamingPacketItemFloats(data, packet->rawData, raw_data_index, 6);
			memcpy(packet->untaredOrientNorth, data, 12);
			memcpy(packet->untaredOrientGravity, data + 3, 12);
		}
		else if (_streamingSlots[i] == 0x40)
		{
			_parseStreamingPacketItemFloats(packet->rawSensorData, packet->rawData, raw_data_index, 9);
		}
		else if (_streamingSlots[i] == 0x41)
		{
			_parseStreamingPacketItemFloats(packet->rawGyroscopeData, packet->rawData, raw_data_index, 3);
		}
		else if (_streamingSlots[i] == 0x42)
		{
			_parseStreamingPacketItemFloats(packet->rawAccelerometerData, packet->rawData, raw_data_index, 3);
		}
		else if (_streamingSlots[i] == 0x43)
		{
			_parseStreamingPacketItemFloats(packet->rawMagnetometerData, packet->rawData, raw_data_index, 3);
		}
		else if (_streamingSlots[i] == 0x20)
		{
			_parseStreamingPacketItemFloats(packet->normalizedSensorData, packet->rawData, raw_data_index, 9);
		}
		else if (_streamingSlots[i] == 0x21)
		{
			_parseStreamingPacketItemFloats(packet->normalizedGyroscopeData, packet->rawData, raw_data_index, 3);
		}
		else if (_streamingSlots[i] == 0x22)
		{
			_parseStreamingPacketItemFloats(packet->normalizedAccelerometerData, packet->rawData, raw_data_index, 3);
		}
		else if (_streamingSlots[i] == 0x23)
		{
			_parseStreamingPacketItemFloats(packet->normalizedMagnetometerData, packet->rawData, raw_data_index, 3);
		}
		else if (_streamingSlots[i] == 0x25)
		{
			_parseStreamingPacketItemFloats(packet->correctedSensorData, packet->rawData, raw_data_index, 9);
		}
		else if (_streamingSlots[i] == 0x26)
		{
			_parseStreamingPacketItemFloats(packet->correctedGyroscopeData, packet->rawData, raw_data_index, 3);
		}
		else if (_streamingSlots[i] == 0x27)
		{
			_parseStreamingPacketItemFloats(packet->correctedAccelerometerData, packet->rawData, raw_data_index, 3);
		}
		else if (_streamingSlots[i] == 0x28)
		{
			_parseStreamingPacketItemFloats(packet->correctedMagnetometerData, packet->rawData, raw_data_index, 3);
		}
		else if (_streamingSlots[i] == 0x29)
		{
			_parseStreamingPacketItemFloats(packet->linearAcceleration, packet->rawData, raw_data_index, 3);
		}
		else if (_streamingSlots[i] == 0xC9)
		{
			_parseStreamingPacketItemFloats(&packet->batteryVoltage, packet->rawData, raw_data_index, 1);
		}
		else if (_streamingSlots[i] == 0xCA)
		{
			packet->batteryLevel = packet->rawData[raw_data_index];
			raw_data_index++;
		}
		else if (_streamingSlots[i] == 0xCB)
		{
			packet->batteryStatus = packet->rawData[raw_data_index];
			raw_data_index++;
		}
		else if (_streamingSlots[i] == 0x2B)
		{
			_parseStreamingPacketItemFloats(&packet->temperatureC, packet->rawData, raw_data_index, 1);
		}
		else if (_streamingSlots[i] == 0x2C)
		{
			_parseStreamingPacketItemFloats(&packet->temperatureF, packet->rawData, raw_data_index, 1);
		}
		else if (_streamingSlots[i] == 0xFA)
		{
			packet->buttonState = packet->rawData[raw_data_index];
			raw_data_index++;
		}
	}

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getFirstStreamingPacket(TSS_Stream_Packet* packet)
{	
	gAPI._readerThreadMutex.lock(); //lock the reading thread so it won't mess with our buffer
	if (_streamBuffer.len == 0)
	{
		gAPI._readerThreadMutex.unlock();
		return TSS_ERROR_NOT_ENOUGH_DATA;
	}

	//get index of last packet
	U32 packet_index = _streamBuffer.start;
	_streamBuffer.getPacket(packet, packet_index);

	//parse the streaming packet out into member variables
	result = _parseStreamingPacket(packet);
	if (result != TSS_NO_ERROR)
	{
		gAPI._readerThreadMutex.unlock();
		return result;
	}

	//remove the parsed packet from the buffer
	_streamBuffer.removeFirstPacket();

	gAPI._readerThreadMutex.unlock();

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getLastStreamingPacket(TSS_Stream_Packet* packet)
{	
	gAPI._readerThreadMutex.lock(); //lock the reading thread so it won't mess with our buffer
	
	if (_streamBuffer.len == 0)
	{
		gAPI._readerThreadMutex.unlock();
		
		return TSS_ERROR_NOT_ENOUGH_DATA;
	}

	//get index of last packet
	U32 packet_index = _streamBuffer.end;
	if (packet_index == 0)
		packet_index = TSS_STREAM_PACKET_CIRCULAR_BUFFER_SIZE - 1;
	else
		packet_index--;

	_streamBuffer.getPacket(packet, packet_index);

	//dump all packets in the buffer
	_streamBuffer.clear();

	gAPI._readerThreadMutex.unlock(); 
	result = _parseStreamingPacket(packet);

	return result;
}

TSS_ERROR TssSensor::stopStreamingWired()
{
	if (!_port)
	{
		return TSS_ERROR_NOT_CONNECTED;
	}

	if (_isWireless)
	{
		return TSS_ERROR_NOT_AVAILABLE_WIRELESS;
	}

	if (!_isStreaming)
	{
		return TSS_ERROR_NOT_STREAMING;
	}

	gAPI.unregisterStreamingDevice(this);

	U32 timestamp;
	TSS_ERROR_CHECK(_sendCommand(86, &timestamp)); //stop streaming command

	bool stillStreaming = true;
	for (unsigned i = 0; i < 50; ++i)
	{
		if (_available() != 0)
		{
			_flushInput();
			_flush();
		}
		else
		{
			stillStreaming = false;
			break;
		}

		yost::sleep_ms(100);
	}

	if(stillStreaming)
	{
		TSS_ERROR_CHECK(_sendCommand(86, &timestamp)); //stop streaming command
		yost::sleep_ms(100);
	}

	_flushInput();
	_flush();

	_unparsedStreamData.empty();

	_isStreaming = false;

	_setResponseHeader(_responseHeaderFlags & ~TSS_RESPONSE_HEADER_CHECKSUM & ~TSS_RESPONSE_HEADER_DATA_LENGTH);

	return result;
}

U32 TssSensor::getStreamingPacketsInWaiting()
{
	return _streamBuffer.len;
}

bool TssSensor::didStreamingOverflow()
{
	return _streamBuffer.len == TSS_STREAM_PACKET_CIRCULAR_BUFFER_SIZE;
}

void TssSensor::setCommandRetries(U8 retries)
{
	_commandRetries = retries;
}

TSS_ERROR TssSensor::getTaredOrientationAsQuaternion(Orient* quat, U32* timestamp)
{
	float buff[4];

	TSS_ERROR_CHECK(_checkedCommandWriteReadFloats(0x00, buff, 4, timestamp));

	memcpy(quat->data, buff, 16);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getTaredOrientationAsEulerAngles(float* euler, U32* timestamp)
{
	float buff[3];

	TSS_ERROR_CHECK(_checkedCommandWriteReadFloats(0x01, buff, 3, timestamp));

	memcpy(euler, buff, 12);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getTaredOrientationAsRotationMatrix(Matrix3x3* mat, U32* timestamp)
{
	TSS_ERROR_CHECK(_checkedCommandWriteReadFloats(0x02, mat->data, 9, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getTaredOrientationAsAxisAngle(Vector3* vec, float* angle, U32* timestamp)
{
	float data[4];

	TSS_ERROR_CHECK(_checkedCommandWriteReadFloats(0x03, data, 4, timestamp));

	memcpy(vec->data, data, 12);
	*angle = data[3];

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getTaredOrientationAsTwoVector(Vector3* forward, Vector3* down, U32* timestamp)
{
	float data[6];

	TSS_ERROR_CHECK(_checkedCommandWriteReadFloats(0x04, data, 6, timestamp));

	memcpy(forward->data, data, 12);
	memcpy(down->data, data, 12);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getUntaredOrientationAsQuaternion(Orient* quat, U32* timestamp)
{
	float buff[4];

	TSS_ERROR_CHECK(_checkedCommandWriteReadFloats(0x06, buff, 4, timestamp));

	memcpy(quat->data, buff, 16);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getUntaredOrientationAsEulerAngles(float* euler, U32* timestamp)
{
	float buff[3];

	TSS_ERROR_CHECK(_checkedCommandWriteReadFloats(0x07, buff, 3, timestamp));

	memcpy(euler, buff, 12);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getUntaredOrientationAsRotationMatrix(Matrix3x3* mat, U32* timestamp)
{
	float buff[9];

	TSS_ERROR_CHECK(_checkedCommandWriteReadFloats(0x08, buff, 9, timestamp));

	memcpy(mat->data, buff, 36);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getUntaredOrientationAsAxisAngle(Vector3* vec, float* angle, U32* timestamp)
{
	float data[4];

	TSS_ERROR_CHECK(_checkedCommandWriteReadFloats(0x09, data, 4, timestamp));

	memcpy(vec->data, data, 12);
	*angle = data[3];

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getUntaredOrientationAsTwoVector(Vector3* forward, Vector3* down, U32* timestamp)
{
	float data[6];

	TSS_ERROR_CHECK(_checkedCommandWriteReadFloats(0x0A, data, 6, timestamp));

	memcpy(forward->data, data, 12);
	memcpy(down->data, data+3, 12);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::tareWithCurrentOrientation(U32* timestamp)
{
	if (!isConnected() || _inErrorState)
		return TSS_ERROR_NOT_CONNECTED;

	if (_inBootloader)
		return TSS_ERROR_BOOTLOADER_MODE;

	TSS_ERROR_CHECK(_sendCommand(0x60, timestamp, true));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getNormalizedSensorData(Vector3* gyroscope3, Vector3* accelerometer3, Vector3* magnetometer3, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	float buff[9];

	TSS_ERROR_CHECK(_checkedCommandWriteReadFloats(0x20, buff, 9, timestamp));

	memcpy(gyroscope3->data, buff, 12);
	memcpy(accelerometer3->data, buff + 3, 12);
	memcpy(magnetometer3->data, buff + 6, 12);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getNormalizedGyroscope(Vector3* gyroscope3, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	float buff[3];

	TSS_ERROR_CHECK(_checkedCommandWriteReadFloats(0x21, buff, 3, timestamp));

	memcpy(gyroscope3->data, buff, 12);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getNormalizedAccelerometer(Vector3* accelerometer3, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	float buff[3];

	TSS_ERROR_CHECK(_checkedCommandWriteReadFloats(0x22, buff, 3, timestamp));

	memcpy(accelerometer3->data, buff, 12);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getNormalizedMagnetometer(Vector3* magnetometer3, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	float buff[3];

	TSS_ERROR_CHECK(_checkedCommandWriteReadFloats(0x23, buff, 3, timestamp));

	memcpy(magnetometer3->data, buff, 12);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getCorrectedSensorData(Vector3* gyroscope3, Vector3* accelerometer3, Vector3* magnetometer3, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	float buff[9];

	TSS_ERROR_CHECK(_checkedCommandWriteReadFloats(0x25, buff, 9, timestamp));

	memcpy(gyroscope3->data, buff, 12);
	memcpy(accelerometer3->data, buff + 3, 12);
	memcpy(magnetometer3->data, buff + 6, 12);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getCorrectedGyroscope(Vector3* gyroscope3, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	float buff[3];

	TSS_ERROR_CHECK(_checkedCommandWriteReadFloats(0x26, buff, 3, timestamp));

	memcpy(gyroscope3->data, buff, 12);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getCorrectedAccelerometer(Vector3* accelerometer3, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	float buff[3];

	TSS_ERROR_CHECK(_checkedCommandWriteReadFloats(0x27, buff, 3, timestamp));

	memcpy(accelerometer3->data, buff, 12);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getCorrectedMagnetometer(Vector3* magnetometer3, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	float buff[3];

	TSS_ERROR_CHECK(_checkedCommandWriteReadFloats(0x28, buff, 3, timestamp));

	memcpy(magnetometer3->data, buff, 12);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getCorrectedLinearAccelerationInGlobalSpace(Vector3* accelerometer3, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	float buff[3];

	TSS_ERROR_CHECK(_checkedCommandWriteReadFloats(0x29, buff, 3, timestamp));

	memcpy(accelerometer3->data, buff, 12);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getRawComponentSensorData(Vector3* gyroscope3, Vector3* accelerometer3, Vector3* magnetometer3, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	float buff[9];

	TSS_ERROR_CHECK(_checkedCommandWriteReadFloats(0x40, buff, 9, timestamp));

	memcpy(gyroscope3->data, buff, 12);
	memcpy(accelerometer3->data, buff + 3, 12);
	memcpy(magnetometer3->data, buff + 6, 12);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getRawGyroscope(Vector3* gyroscope3, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	float buff[3];

	TSS_ERROR_CHECK(_checkedCommandWriteReadFloats(0x41, buff, 3, timestamp));

	memcpy(gyroscope3->data, buff, 12);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getRawAccelerometer(Vector3* accelerometer3, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	float buff[3];

	TSS_ERROR_CHECK(_checkedCommandWriteReadFloats(0x42, buff, 3, timestamp));

	memcpy(accelerometer3->data, buff, 12);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getRawMagnetometer(Vector3* magnetometer3, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	float buff[3];

	TSS_ERROR_CHECK(_checkedCommandWriteReadFloats(0x43, buff, 3, timestamp));

	memcpy(magnetometer3->data, buff, 12);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getTemperatureC(float* temp, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	float buff[1];

	TSS_ERROR_CHECK(_checkedCommandWriteReadFloats(0x2B, buff, 1, timestamp));

	memcpy(temp, buff, 4);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getTemperatureF(float* temp, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	float buff[1];

	TSS_ERROR_CHECK(_checkedCommandWriteReadFloats(0x2C, buff, 1, timestamp));

	memcpy(temp, buff, 4);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getConfidenceFactor(float* confindence, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	float buff[1];

	TSS_ERROR_CHECK(_checkedCommandWriteReadFloats(0x2D, buff, 1, timestamp));

	memcpy(confindence, buff, 4);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getWirelessChannel(U8* channel, U32* timestamp)
{
	if (_sensorType != TSS_WIRELESS)
	{
		return TSS_ERROR_SENSOR_TYPE_MISMATCH;
	}

	U8 buff[1];

	TSS_ERROR_CHECK(_checkedCommandWriteRead(0xC2, buff, 1, timestamp));

	memcpy(channel, buff, 1);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setWirelessChannel(U8 channel, U32* timestamp)
{
	if (_sensorType != TSS_WIRELESS)
	{
		return TSS_ERROR_SENSOR_TYPE_MISMATCH;
	}

	BASIC_CALL_CHECK_TSS();

	U8 buff[2];
	buff[0] = 0xC3;
	buff[1] = channel;

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 2, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getWirelessPanID(U16* panid, U32* timestamp)
{
	if (_sensorType != TSS_WIRELESS)
	{
		return TSS_ERROR_SENSOR_TYPE_MISMATCH;
	}

	U8 buff[2];

	TSS_ERROR_CHECK(_checkedCommandWriteRead(0xC0, buff, 2, timestamp));

	memcpy(panid, buff, 2);

	yost::swap16((U8*)panid);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setWirelessPanID(U16 panid, U32* timestamp)
{
	if (_sensorType != TSS_WIRELESS)
	{
		return TSS_ERROR_SENSOR_TYPE_MISMATCH;
	}

	BASIC_CALL_CHECK_TSS();

	U8 buff[3];
	buff[0] = 0xC1;

	memcpy(buff + 1, &panid, 2);
	yost::swap16((U8*)buff + 1);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 3, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::commitSettings(U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	TSS_ERROR_CHECK(_sendCommand(0xE1, timestamp, false));

	if (_sensorType == TSS_LE)
	{
		yost::sleep_ms(1500);
	}

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::commitWirelessSettings(U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	TSS_ERROR_CHECK(_sendCommand(0xc5, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getWirelessAddress(U16* address, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[2];

	TSS_ERROR_CHECK(_checkedCommandWriteRead(0xC6, buff, 2, timestamp));

	yost::swap16(buff);

	memcpy(address, buff, 2);

	return TSS_NO_ERROR;
}

//Data-Logging Commands
TSS_ERROR TssSensor::turnOnMassStorage(U32* timestamp)
{
	if (_sensorType != TSS_DATALOGGER)
	{
		return TSS_ERROR_SENSOR_TYPE_MISMATCH;
	}

	BASIC_CALL_CHECK_TSS();

	TSS_ERROR_CHECK(_sendCommand(0x39, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::turnOffMassStorage(U32* timestamp)
{
	if (_sensorType != TSS_DATALOGGER)
	{
		return TSS_ERROR_SENSOR_TYPE_MISMATCH;
	}

	BASIC_CALL_CHECK_TSS();

	TSS_ERROR_CHECK(_sendCommand(0x3A, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::formatAndInitializeSDCard(U32* timestamp)
{
	if (_sensorType != TSS_DATALOGGER)
	{
		return TSS_ERROR_SENSOR_TYPE_MISMATCH;
	}

	BASIC_CALL_CHECK_TSS();

	TSS_ERROR_CHECK(_sendCommand(0x3B, timestamp, true));

	yost::sleep_ms(10000);

	_flush();

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::beginDataLoggingSession(U32* timestamp)
{
	if (_sensorType != TSS_DATALOGGER)
	{
		return TSS_ERROR_SENSOR_TYPE_MISMATCH;
	}

	BASIC_CALL_CHECK_TSS();

	TSS_ERROR_CHECK(_sendCommand(0x3C, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::endDataLoggingSession(U32* timestamp)
{
	if (_sensorType != TSS_DATALOGGER)
	{
		return TSS_ERROR_SENSOR_TYPE_MISMATCH;
	}

	BASIC_CALL_CHECK_TSS();

	TSS_ERROR_CHECK(_sendCommand(0x3D, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setClockValues(U8 month, U8 day, U8 year, U8 hour, U8 minute, U8 second, U32* timestamp)
{
	if (_sensorType != TSS_DATALOGGER)
	{
		return TSS_ERROR_SENSOR_TYPE_MISMATCH;
	}

	BASIC_CALL_CHECK_TSS();

	U8 buff[7];
	buff[0] = 0x3E;

	memcpy(buff + 1, &month, 1);
	memcpy(buff + 2, &day, 1);
	memcpy(buff + 3, &year, 1);
	memcpy(buff + 4, &hour, 1);
	memcpy(buff + 5, &minute, 1);
	memcpy(buff + 6, &second, 1);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 7, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getClockValues(U8* month, U8* day, U8* year, U8* hour, U8* minute, U8* second, U32* timestamp)
{
	if (_sensorType != TSS_DATALOGGER)
	{
		return TSS_ERROR_SENSOR_TYPE_MISMATCH;
	}

	BASIC_CALL_CHECK_TSS();

	U8 buff[6];

	TSS_ERROR_CHECK(_checkedCommandWriteRead(0x3F, buff, 6, timestamp));

	*month = buff[0];
	*day = buff[1];
	*year = buff[2];
	*hour = buff[3];
	*minute = buff[4];
	*second = buff[5];

	return TSS_NO_ERROR;
}

//Streaming Commands
TSS_ERROR TssSensor::updateCurrentTimestamp(U32 set_timestamp, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[5];
	buff[0] = 0x5F;

	memcpy(buff + 1, &set_timestamp, 4);
	yost::swap32(buff + 1);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 5, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setStreamingSlots(U8 slot1, U8 slot2, U8 slot3, U8 slot4, U8 slot5, U8 slot6, U8 slot7, U8 slot8, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[9];
	buff[0] = 0x50;
	buff[1] = slot1;
	buff[2] = slot2;
	buff[3] = slot3;
	buff[4] = slot4;
	buff[5] = slot5;
	buff[6] = slot6;
	buff[7] = slot7;
	buff[8] = slot8;

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 9, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getStreamingSlots(U8* slot1, U8* slot2, U8* slot3, U8* slot4, U8* slot5, U8* slot6, U8* slot7, U8* slot8, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[8];

	TSS_ERROR_CHECK(_checkedCommandWriteRead(0x51, buff, 8, timestamp));

	*slot1 = buff[0];
	*slot2 = buff[1];
	*slot3 = buff[2];
	*slot4 = buff[3];
	*slot5 = buff[4];
	*slot6 = buff[5];
	*slot7 = buff[6];
	*slot8 = buff[7];

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setStreamingTiming(U32 interval, U32 duration, U32 delay, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[13];
	buff[0] = 0x52;
	
	memcpy(buff + 1, &interval, 4);
	memcpy(buff + 5, &duration, 4);
	memcpy(buff + 9, &delay, 4);

	yost::swap32(buff + 1);
	yost::swap32(buff + 5);
	yost::swap32(buff + 9);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 13, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getStreamingTiming(U32* interval, U32* duration, U32* delay, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[12];
	TSS_ERROR_CHECK(_checkedCommandWriteRead(0x53, buff, 12, timestamp));

	yost::swap32(buff);
	yost::swap32(buff + 4);
	yost::swap32(buff + 8);

	memcpy(interval, buff, 4);
	memcpy(duration, buff + 4, 4);
	memcpy(delay, buff + 8, 4);

	return TSS_NO_ERROR;
}

//Configuration Write Commands
TSS_ERROR TssSensor::setEulerAngleDecompositionOrder(U8 order, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[2];
	buff[0] = 0x10;

	memcpy(buff + 1, &order, 1);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 2, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setMagnetoresistiveThreshold(float threshold, U32 trust_frames, float lockout_decay, float perturbation_detection_value, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[17];
	buff[0] = 0x11;

	memcpy(buff + 1, &threshold, 4);
	memcpy(buff + 5, &trust_frames, 4);
	memcpy(buff + 9, &lockout_decay, 4);
	memcpy(buff + 13, &perturbation_detection_value, 4);

	yost::swap32(buff + 1);
	yost::swap32(buff + 5);
	yost::swap32(buff + 9);
	yost::swap32(buff + 13);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 17, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setAccelerometerResistanceThreshold(float threshold, U32 lockout_frames, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[9];
	buff[0] = 0x12;

	memcpy(buff + 1, &threshold, 4);
	memcpy(buff + 5, &lockout_frames, 4);

	yost::swap32(buff + 1);
	yost::swap32(buff + 5);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 9, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::offsetWithCurrentOrientation(U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	TSS_ERROR_CHECK(_sendCommand(0x13, timestamp));
	
	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::resetBaseOffset(U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	TSS_ERROR_CHECK(_sendCommand(0x14, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::offsetWithQuaternion(Orient* quat4, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[17];
	buff[0] = 0x15;

	memcpy(buff + 1, &quat4->data, 16);

	yost::swap32(buff + 1);
	yost::swap32(buff + 5);
	yost::swap32(buff + 9);
	yost::swap32(buff + 13);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 17, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::tareWithQuaternion(Orient* quat4, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[17];
	buff[0] = 0x61;

	memcpy(buff + 1, &quat4->data, 16);

	yost::swap32(buff + 1);
	yost::swap32(buff + 5);
	yost::swap32(buff + 9);
	yost::swap32(buff + 13);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 17, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::tareWithRotationMatrix(float* matrix9, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[37];
	buff[0] = 0x62;

	memcpy(buff + 1, matrix9, 4);
	memcpy(buff + 5, matrix9 + 1, 4);
	memcpy(buff + 9, matrix9 + 2, 4);
	memcpy(buff + 13, matrix9 + 3, 4);
	memcpy(buff + 17, matrix9 + 4, 4);
	memcpy(buff + 21, matrix9 + 5, 4);
	memcpy(buff + 25, matrix9 + 6, 4);
	memcpy(buff + 29, matrix9 + 7, 4);
	memcpy(buff + 33, matrix9 + 8, 4);

	yost::swap32(buff + 1);
	yost::swap32(buff + 5);
	yost::swap32(buff + 9);
	yost::swap32(buff + 13);
	yost::swap32(buff + 17);
	yost::swap32(buff + 21);
	yost::swap32(buff + 25);
	yost::swap32(buff + 29);
	yost::swap32(buff + 33);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 17, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setStaticAccelerometerTrustValue(float trust_value, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[5];
	buff[0] = 0x63;

	memcpy(buff + 1, &trust_value, 4);

	yost::swap32(buff + 1);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 5, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setConfidenceAccelerometerTrustValues(float min_trust_value, float max_trust_value, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[9];
	buff[0] = 0x64;

	memcpy(buff + 1, &min_trust_value, 4);
	memcpy(buff + 5, &max_trust_value, 4);

	yost::swap32(buff + 1);
	yost::swap32(buff + 5);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 9, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setStaticCompassTrustValue(float trust_value, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[5];
	buff[0] = 0x65;

	memcpy(buff + 1, &trust_value, 4);

	yost::swap32(buff + 1);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 5, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setConfidenceCompassTrustValues(float min_trust_value, float max_trust_value, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[9];
	buff[0] = 0x66;

	memcpy(buff + 1, &min_trust_value, 4);
	memcpy(buff + 5, &max_trust_value, 4);

	yost::swap32(buff + 1);
	yost::swap32(buff + 5);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 9, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setDesiredUpdateRate(U32 update_rate, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[5];
	buff[0] = 0x67;

	memcpy(buff + 1, &update_rate, 4);

	yost::swap32(buff + 1);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 5, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setReferenceVectorMode(U8 mode, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[2];
	buff[0] = 0x69;

	memcpy(buff + 1, &mode, 1);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 2, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setOversampleRate(U16 gyro_sample_rate, U16 accel_sample_rate, U16 compass_sample_rate, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[7];
	buff[0] = 0x6A;

	memcpy(buff + 1, &gyro_sample_rate, 2);
	memcpy(buff + 3, &accel_sample_rate, 2);
	memcpy(buff + 5, &compass_sample_rate, 2);

	yost::swap16(buff + 1);
	yost::swap16(buff + 3);
	yost::swap16(buff + 5);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 7, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setGyroscopeEnabled(U8 enabled, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[2];
	buff[0] = 0x6B;

	memcpy(buff + 1, &enabled, 1);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 2, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setAccelerometerEnabled(U8 enabled, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[2];
	buff[0] = 0x6C;

	memcpy(buff + 1, &enabled, 1);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 2, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setCompassEnabled(U8 enabled, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[2];
	buff[0] = 0x6D;

	memcpy(buff + 1, &enabled, 1);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 2, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setOrientationSmoothing(float enabled, float compass_max_smooth_factor, float compass_min_smooth_factor, float accelerometer_max_smooth_factor, float accelerometer_min_smooth_factor, float smoothing_magnification, float smoothing_offset, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[29];
	buff[0] = 0x6E;

	memcpy(buff + 1, &enabled, 4);
	memcpy(buff + 5, &compass_max_smooth_factor, 4);
	memcpy(buff + 9, &compass_min_smooth_factor, 4);
	memcpy(buff + 13, &accelerometer_max_smooth_factor, 4);
	memcpy(buff + 17, &accelerometer_min_smooth_factor, 4);
	memcpy(buff + 21, &smoothing_magnification, 4);
	memcpy(buff + 25, &smoothing_offset, 4);

	yost::swap32(buff + 1);
	yost::swap32(buff + 5);
	yost::swap32(buff + 9);
	yost::swap32(buff + 13);
	yost::swap32(buff + 17);
	yost::swap32(buff + 21);
	yost::swap32(buff + 25);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 29, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getOrientationSmoothing(U8* enabled, float* compass_max_smooth_factor, float* compass_min_smooth_factor, float* accelerometer_max_smooth_factor, float* accelerometer_min_smooth_factor, float* smoothing_magnification, float* smoothing_offset, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[25];

	TSS_ERROR_CHECK(_checkedCommandWriteRead(0x6F, buff, 25, timestamp));

	*enabled = buff[0];

	yost::swap32(buff + 1);
	yost::swap32(buff + 5);
	yost::swap32(buff + 9);
	yost::swap32(buff + 13);
	yost::swap32(buff + 17);
	yost::swap32(buff + 21);

	memcpy(compass_max_smooth_factor, buff + 1, 4);
	memcpy(compass_min_smooth_factor, buff + 5, 4);
	memcpy(accelerometer_max_smooth_factor, buff + 9, 4);
	memcpy(accelerometer_min_smooth_factor, buff + 13, 4);
	memcpy(smoothing_magnification, buff + 17, 4);
	memcpy(smoothing_offset, buff + 21, 4);

	return TSS_NO_ERROR;
}
TSS_ERROR TssSensor::setAxisDirections(U8 axis_direction_byte, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[2];
	buff[0] = 0x74;

	memcpy(buff + 1, &axis_direction_byte, 1);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 2, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setRunningAveragePercent(float gyro_running_average_percent, float accel_running_average_percent, float mag_running_average_percent, float orient_running_average_percent, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[17];
	buff[0] = 0x75;

	memcpy(buff + 1, &gyro_running_average_percent, 4);
	memcpy(buff + 5, &accel_running_average_percent, 4);
	memcpy(buff + 9, &mag_running_average_percent, 4);
	memcpy(buff + 13, &orient_running_average_percent, 4);

	yost::swap32(buff + 1);
	yost::swap32(buff + 5);
	yost::swap32(buff + 9);
	yost::swap32(buff + 13);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 17, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setCompassReferenceVector(Vector3* reference_vector3, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[13];
	buff[0] = 0x76;

	memcpy(buff + 1, &reference_vector3->data, 12);

	yost::swap32(buff + 1);
	yost::swap32(buff + 5);
	yost::swap32(buff + 9);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 13, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setAccelerometerReferenceVector(Vector3* reference_vector3, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[13];
	buff[0] = 0x77;

	memcpy(buff + 1, &reference_vector3->data, 12);

	yost::swap32(buff + 1);
	yost::swap32(buff + 5);
	yost::swap32(buff + 9);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 13, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::resetKalmanFilter(U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	TSS_ERROR_CHECK(_sendCommand(0x78, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setAccelerometerRange(U8 accelerometer_range_setting, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[2];
	buff[0] = 0x79;

	memcpy(buff + 1, &accelerometer_range_setting, 1);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 2, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setFilterMode(U8 mode, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[2];
	buff[0] = 0x7B;

	memcpy(buff + 1, &mode, 1);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 2, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setRunningAverageMode(U8 mode, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[2];
	buff[0] = 0x7C;

	memcpy(buff + 1, &mode, 1);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 2, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setGyroscopeRange(U8 gyroscope_range_setting, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[2];
	buff[0] = 0x7D;

	memcpy(buff + 1, &gyroscope_range_setting, 1);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 2, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setCompassRange(U8 compass_range_setting, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[2];
	buff[0] = 0x7E;

	memcpy(buff + 1, &compass_range_setting, 1);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 2, timestamp));

	return TSS_NO_ERROR;
}

// Configuration Read Commands
TSS_ERROR TssSensor::getTareAsQuaternion(Orient* quat4, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	float buff[4];

	TSS_ERROR_CHECK(_checkedCommandWriteReadFloats(0x80, buff, 4, timestamp));

	memcpy(quat4->data, buff, 16);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getTareAsRotationMatrix(float* matrix9, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	float buff[9];

	TSS_ERROR_CHECK(_checkedCommandWriteReadFloats(0x81, buff, 9, timestamp));

	memcpy(matrix9, buff, 36);


	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getAccelerometerTrustValues(float* min_trust_value, float* max_trust_value, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	float buff[2];

	TSS_ERROR_CHECK(_checkedCommandWriteReadFloats(0x82, buff, 2, timestamp));

	*min_trust_value = buff[0];
	*max_trust_value = buff[1];

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getCompassTrustValues(float* min_trust_value, float* max_trust_value, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	float buff[2];

	TSS_ERROR_CHECK(_checkedCommandWriteReadFloats(0x83, buff, 2, timestamp));

	*min_trust_value = buff[0];
	*max_trust_value = buff[1];

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getCurrentUpdateRate(U32* last_update, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[4];

	TSS_ERROR_CHECK(_checkedCommandWriteRead(0x84, buff,4, timestamp));

	yost::swap32(buff);

	memcpy(last_update, buff, 4);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getCompassReferenceVector(float* reference_vector, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	float buff[3];

	TSS_ERROR_CHECK(_checkedCommandWriteReadFloats(0x85, buff, 3, timestamp));

	memcpy(reference_vector, buff, 12);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getAccelerometerReferenceVector(float* reference_vector, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	float buff[3];

	TSS_ERROR_CHECK(_checkedCommandWriteReadFloats(0x86, buff, 3, timestamp));

	memcpy(reference_vector, buff, 12);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getReferenceVectorMode(U8* mode, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[1];

	TSS_ERROR_CHECK(_checkedCommandWriteRead(0x87, buff, 1, timestamp));

	*mode = buff[0];

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getGyroscopeEnabledState(U8* enabled, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[1];

	TSS_ERROR_CHECK(_checkedCommandWriteRead(0x8C, buff, 1, timestamp));

	*enabled = buff[0];

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getAccelerometerEnabledState(U8* enabled, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[1];

	TSS_ERROR_CHECK(_checkedCommandWriteRead(0x8D, buff, 1, timestamp));

	*enabled = buff[0];

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getCompassEnabledState(U8* enabled, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[1];

	TSS_ERROR_CHECK(_checkedCommandWriteRead(0x8E, buff, 1, timestamp));

	*enabled = buff[0];

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getAxisDirections(U8* axis_directions, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[1];

	TSS_ERROR_CHECK(_checkedCommandWriteRead(0x8F, buff, 1, timestamp));

	*axis_directions = buff[0];

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getOversampleRate(U16* gyro_sample_rate, U16* accel_sample_rate, U16* compass_sample_rate, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[6];

	TSS_ERROR_CHECK(_checkedCommandWriteRead(0x90, buff, 6, timestamp));

	yost::swap16(buff);
	yost::swap16(buff + 2);
	yost::swap16(buff + 4);

	memcpy(gyro_sample_rate, buff, 2);
	memcpy(accel_sample_rate, buff + 2, 2);
	memcpy(compass_sample_rate, buff + 4, 2);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getRunningAveragePercent(float* gyro_running_average_percent, float* accel_running_average_percent, float* mag_running_average_percent, float* orient_running_average_percent, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	float buff[4];

	TSS_ERROR_CHECK(_checkedCommandWriteReadFloats(0x91, buff, 4, timestamp));

	*gyro_running_average_percent = buff[0];
	*accel_running_average_percent = buff[1];
	*mag_running_average_percent = buff[2];
	*orient_running_average_percent = buff[3];

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getDesiredUpdateRate(U32* update_rate, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[4];

	TSS_ERROR_CHECK(_checkedCommandWriteRead(0x92, buff, 4, timestamp));

	yost::swap32(buff);

	memcpy(update_rate, buff, 4);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getAccelerometerRange(U8* accelerometer_range_setting, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[1];

	TSS_ERROR_CHECK(_checkedCommandWriteRead(0x94, buff, 1, timestamp));

	*accelerometer_range_setting = buff[0];

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getFilterMode(U8* mode, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[1];

	TSS_ERROR_CHECK(_checkedCommandWriteRead(0x98, buff, 1, timestamp));

	*mode = buff[0];

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getRunningAverageMode(U8* mode, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[1];

	TSS_ERROR_CHECK(_checkedCommandWriteRead(0x99, buff, 1, timestamp));

	*mode = buff[0];

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getGyroscopeRange(U8* gyroscope_range_setting, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[1];

	TSS_ERROR_CHECK(_checkedCommandWriteRead(0x9A, buff, 1, timestamp));

	*gyroscope_range_setting = buff[0];

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getCompassRange(U8* compass_range_setting, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[1];

	TSS_ERROR_CHECK(_checkedCommandWriteRead(0x9B, buff, 1, timestamp));

	*compass_range_setting = buff[0];

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getEulerAngleDecompositionOrder(U8* order, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[1];

	TSS_ERROR_CHECK(_checkedCommandWriteRead(0x9C, buff, 1, timestamp));

	*order = buff[0];

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getMagnetoresistiveThreshold(float* threshold, U32* trust_frames, float* lockout_decay, float* perturbation_detection_value, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[16];

	TSS_ERROR_CHECK(_checkedCommandWriteRead(0x9D, buff, 16, timestamp));

	yost::swap32(buff);
	yost::swap32(buff + 4);
	yost::swap32(buff + 8);
	yost::swap32(buff + 12);

	memcpy(threshold, buff, 4);
	memcpy(trust_frames, buff + 4, 4);
	memcpy(lockout_decay, buff + 8, 4);
	memcpy(perturbation_detection_value, buff + 12, 4);


	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getAccelerometerResistanceThreshold(float* threshold, U32* lockout_frames, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[9];
	buff[0] = 0x9E;

	memcpy(buff + 1, &threshold, 4);
	memcpy(buff + 5, &lockout_frames, 4);

	yost::swap32(buff + 1);
	yost::swap32(buff + 5);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 9, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getOffsetOrientationAsQuaternion(Orient* quat4, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	float buff[4];

	TSS_ERROR_CHECK(_checkedCommandWriteReadFloats(0x9F, buff, 4, timestamp));

	memcpy(quat4, buff, 16);

	return TSS_NO_ERROR;
}

//Calibration Commands
TSS_ERROR TssSensor::setCompassCalibrationCoefficients(float* matrix9, Vector3* bias3, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[49];
	buff[0] = 0xA0;

	memcpy(buff + 1, matrix9, 36);
	memcpy(buff + 37, &bias3->data, 12);

	yost::swap32(buff + 1);
	yost::swap32(buff + 5);
	yost::swap32(buff + 9);
	yost::swap32(buff + 13);
	yost::swap32(buff + 17);
	yost::swap32(buff + 21);
	yost::swap32(buff + 25);
	yost::swap32(buff + 29);
	yost::swap32(buff + 33);
	yost::swap32(buff + 37);
	yost::swap32(buff + 41);
	yost::swap32(buff + 45);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 49, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setAccelerometerCalibrationCoefficients(float* matrix9, Vector3* bias3, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[49];
	buff[0] = 0xA1;

	memcpy(buff + 1, matrix9, 36);
	memcpy(buff + 37, &bias3->data, 12);

	yost::swap32(buff + 1);
	yost::swap32(buff + 5);
	yost::swap32(buff + 9);
	yost::swap32(buff + 13);
	yost::swap32(buff + 17);
	yost::swap32(buff + 21);
	yost::swap32(buff + 25);
	yost::swap32(buff + 29);
	yost::swap32(buff + 33);
	yost::swap32(buff + 37);
	yost::swap32(buff + 41);
	yost::swap32(buff + 45);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 49, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getCompassCalibrationCoefficients(float* matrix9, Vector3* bias3, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	float buff[12];

	TSS_ERROR_CHECK(_checkedCommandWriteReadFloats(0xA2, buff, 12, timestamp));

	memcpy(matrix9, buff, 36);
	memcpy(bias3->data, buff + 9, 12);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getAccelerometerCalibrationCoefficients(float* matrix9, Vector3* bias3, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	float buff[12];

	TSS_ERROR_CHECK(_checkedCommandWriteReadFloats(0xA3, buff, 12, timestamp));

	memcpy(matrix9, buff, 36);
	memcpy(bias3->data, buff + 9, 12);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getGyroscopeCalibrationCoefficients(float* matrix9, Vector3* bias3, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	float buff[12];

	TSS_ERROR_CHECK(_checkedCommandWriteReadFloats(0xA4, buff, 12, timestamp));

	memcpy(matrix9, buff, 36);
	memcpy(bias3->data, buff + 9, 12);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::beginGyroscopeAutoCalibration(U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	TSS_ERROR_CHECK(_sendCommand(0xA5, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setGyroscopeCalibrationCoefficients(float* matrix9, Vector3* bias3, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[49];
	buff[0] = 0xA6;

	memcpy(buff + 1, matrix9, 36);
	memcpy(buff + 37, &bias3->data, 12);

	yost::swap32(buff + 1);
	yost::swap32(buff + 5);
	yost::swap32(buff + 9);
	yost::swap32(buff + 13);
	yost::swap32(buff + 17);
	yost::swap32(buff + 21);
	yost::swap32(buff + 25);
	yost::swap32(buff + 29);
	yost::swap32(buff + 33);
	yost::swap32(buff + 37);
	yost::swap32(buff + 41);
	yost::swap32(buff + 45);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 49, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setCalibrationMode(U8 mode, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[2];
	buff[0] = 0xA9;

	memcpy(buff + 1, &mode, 1);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 2, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getCalibrationMode(U8* mode, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[1];

	TSS_ERROR_CHECK(_checkedCommandWriteRead(0xAA, buff, 1, timestamp));

	*mode = buff[0];

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setAutoCompassCalibrationMode(U8 mode, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[2];
	buff[0] = 0xAB;
	buff[1] = mode;
	
	TSS_ERROR_CHECK(_sendCommandBytes(buff, 2, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getAutoCompassCalibrationMode(U8* mode, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	TSS_ERROR_CHECK(_sendCommand(0xAC, timestamp));
	TSS_ERROR_CHECK(_readBytes(mode, 1));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setAutoCompassCalibrationSettings(float too_close_angle, float bias_movement_percentage, float coplanarity_tolerance, U8 max_averages, U8 max_bad_deviations, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[15];
	buff[0] = 0xAD;
	memcpy(buff + 1, &too_close_angle, 4);
	memcpy(buff + 5, &bias_movement_percentage, 4);
	memcpy(buff + 9, &coplanarity_tolerance, 4);	

	yost::swap32(buff + 1);
	yost::swap32(buff + 5);
	yost::swap32(buff + 9);	

	buff[13] = max_averages;
	buff[14] = max_bad_deviations;

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 15, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getAutoCompassCalibrationSettings(float* too_close_angle, float* bias_movement_percentage, float* coplanarity_tolerance, U8* max_averages, U8* max_bad_deviations, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[14];
	float* fl_buff = (float*)buff;

	TSS_ERROR_CHECK(_sendCommand(0xAE, timestamp));
	TSS_ERROR_CHECK(_readBytes(buff, 14));

	yost::swap32(buff);
	yost::swap32(buff + 4);
	yost::swap32(buff + 8);

	*too_close_angle = fl_buff[0];
	*bias_movement_percentage = fl_buff[1];
	*coplanarity_tolerance = fl_buff[2];	

	*max_averages = buff[12];
	*max_bad_deviations = buff[13];

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getAutoCompassCaibrationCount(U8* count, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	TSS_ERROR_CHECK(_sendCommand(0xAF, timestamp));
	TSS_ERROR_CHECK(_readBytes(count, 1));

	return TSS_NO_ERROR;
}

/*
TSS_ERROR TssSensor::setOrthoCalibrationDataPointFromCurrentOrientation(U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	TSS_ERROR_CHECK(_sendCommand(0xAB, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setOrthoCalibrationDataPointFromVector(U8 type, U8 index, Vector3* vector3, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[15];
	buff[0] = 0xAC;

	memcpy(buff + 1, &type, 1);
	memcpy(buff + 2, &index, 1);
	memcpy(buff + 3, &vector3->data, 12);

	yost::swap32(buff + 3);
	yost::swap32(buff + 7);
	yost::swap32(buff + 11);


	TSS_ERROR_CHECK(_sendCommandBytes(buff, 15, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getOrthoCalibrationDataPoint(U8 type, U8 index, Vector3* vector3, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[3];
	buff[0] = 0xAD;
	buff[1] = type;
	buff[2] = index;

	float returnBuff[3];

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 3, timestamp));

	TSS_ERROR_CHECK(_readBytes((U8*)returnBuff, 12));

	yost::swap32((U8*)returnBuff);
	yost::swap32((U8*)returnBuff+1);
	yost::swap32((U8*)returnBuff+2);

	memcpy(vector3->data, returnBuff, 12);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::performOrthoCalibration(U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	TSS_ERROR_CHECK(_sendCommand(0xAE, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::clearOrthoCalibrationData(U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	TSS_ERROR_CHECK(_sendCommand(0xAF, timestamp));

	return TSS_NO_ERROR;
}
*/

//Battery Commands
TSS_ERROR TssSensor::getBatteryVoltage(float* battery_voltage, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	float buff[1];

	TSS_ERROR_CHECK(_checkedCommandWriteReadFloats(0xC9, buff, 1, timestamp));

	*battery_voltage = buff[0];

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getBatteryPercentRemaining(U8* battery_percent, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[1];

	TSS_ERROR_CHECK(_checkedCommandWriteRead(0xCA, buff, 1, timestamp));

	*battery_percent = buff[0];

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getBatteryStatus(U8* battery_charge_status, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[1];

	TSS_ERROR_CHECK(_checkedCommandWriteRead(0xCB, buff, 1, timestamp));

	*battery_charge_status = buff[0];

	return TSS_NO_ERROR;
}

//General Commands
TSS_ERROR TssSensor::setLEDMode(U8 mode, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[2];
	buff[0] = 0xC4;

	memcpy(buff + 1, &mode, 1);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 2, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getLEDMode(U8* mode, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	TSS_ERROR_CHECK(_checkedCommandWriteRead(0xC8, mode, 1, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getWiredResponseHeaderBitfield(U32* bitfield, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[4];

	TSS_ERROR_CHECK(_checkedCommandWriteRead(0xDE, buff, 4, timestamp));

	yost::swap32(buff);

	memcpy(bitfield, buff, 4);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setWiredResponseHeaderBitfield(U32 bitfield, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[5];
	buff[0] = 0xDD;
	memcpy(buff + 1, &bitfield, 4);

	yost::swap32(buff + 1);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 5, timestamp));
	_setResponseHeader(bitfield);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::restoreFactorySettings(U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U32 responseHeaferFlagsBackup = _responseHeaderFlags;

	TSS_ERROR_CHECK(_sendCommand(0xE0, timestamp, true));
	_setResponseHeader(0);
	_setResponseHeader(responseHeaferFlagsBackup);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::softwareReset()
{
	if (!isConnected()) { return TSS_ERROR_NOT_CONNECTED; }
	if (_isStreaming) { return TSS_ERROR_COMMAND_CALLED_DURING_STREAMING; }
	if (_inErrorState) { return TSS_ERROR_NOT_CONNECTED; }	

	TSS_ERROR_CHECK(_softwareReset());

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setSleepMode(U8 mode, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[2];
	buff[0] = 0xE3;

	memcpy(buff + 1, &mode, 1);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 2, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getSleepMode(U8* mode, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[1];

	TSS_ERROR_CHECK(_checkedCommandWriteRead(0xE4, buff, 1, timestamp));

	*mode = buff[0];

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setUARTBaudRate(U32 baud_rate, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[5];
	buff[0] = 0xE7;

	memcpy(buff + 1, &baud_rate, 4);
	yost::swap32(buff + 1);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 5, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getUARTBaudRate(U32* baud_rate, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[4];

	TSS_ERROR_CHECK(_checkedCommandWriteRead(0xE8, buff, 4, timestamp));

	yost::swap32(buff);

	memcpy(baud_rate, buff, 4);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setUSBMode(U8 mode, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[2];
	buff[0] = 0xE9;

	memcpy(buff + 1, &mode, 1);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 2, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getUSBMode(U8* mode, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[1];

	TSS_ERROR_CHECK(_checkedCommandWriteRead(0x92, buff, 1, timestamp));

	memcpy(mode, buff, 1);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setLEDColor(Vector3* rgb_color3, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[13];
	buff[0] = 0xEE;

	memcpy(buff + 1, &rgb_color3->data, 12);

	yost::swap32(buff + 1);
	yost::swap32(buff + 5);
	yost::swap32(buff + 9);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 13, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getLEDColor(Vector3* rgb_color3, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	float buff[3];

	TSS_ERROR_CHECK(_checkedCommandWriteReadFloats(0xEF, buff, 3, timestamp));

	memcpy(rgb_color3->data, buff, 12);

	return TSS_NO_ERROR;
}

//Wired HID Commands
TSS_ERROR TssSensor::setJoystickEnabled(U8 enabled_state, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[2];
	buff[0] = 0xF0;

	memcpy(buff + 1, &enabled_state, 1);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 2, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setMouseEnabled(U8 enabled_state, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[2];
	buff[0] = 0xF1;

	memcpy(buff + 1, &enabled_state, 1);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 2, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getJoystickEnabled(U8* enabled_state, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[1];

	TSS_ERROR_CHECK(_checkedCommandWriteRead(0xF2, buff, 1, timestamp));

	*enabled_state = buff[0];

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getMouseEnabled(U8* enabled_state, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[1];

	TSS_ERROR_CHECK(_checkedCommandWriteRead(0xF3, buff, 1, timestamp));

	*enabled_state = buff[0];

	return TSS_NO_ERROR;
}

//General HID Commands
TSS_ERROR TssSensor::setControlMode(U8 control_class, U8 control_index, U8 handler_index, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[4];
	buff[0] = 0xF4;

	memcpy(buff + 1, &control_class, 1);
	memcpy(buff + 2, &control_index, 1);
	memcpy(buff + 3, &handler_index, 1);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 4, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setControlData(U8 control_class, U8 control_index, U8 data_point_index, float data_point, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[8];
	buff[0] = 0xF5;

	memcpy(buff + 1, &control_class, 1);
	memcpy(buff + 2, &control_index, 1);
	memcpy(buff + 3, &data_point_index, 1);
	yost::swap32((U8*)&data_point);
	memcpy(buff + 4, &data_point, 4);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 8, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getControlMode(U8 control_class, U8 control_index, U8* handler_index, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[3];
	buff[0] = 0xF6;
	buff[1] = control_class;
	buff[2] = control_index;

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 3, timestamp));

	TSS_ERROR_CHECK(_readBytes(handler_index, 1));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getControlData(U8 control_class, U8 control_index, U8 data_point_index, float* data_point, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[4];
	buff[0] = 0xF7;
	buff[1] = control_class;
	buff[2] = control_index;
	buff[3] = data_point_index;

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 4, timestamp));

	TSS_ERROR_CHECK(_readBytes((U8*)data_point, 4));

	yost::swap32((U8*)data_point);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setButtonGyroDisableLength(U8 number_of_frames, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[2];
	buff[0] = 0xF8;

	memcpy(buff + 1, &number_of_frames, 1);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 2, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getButtonGyroDisableLength(U8* number_of_frames, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[1];

	TSS_ERROR_CHECK(_checkedCommandWriteRead(0xF9, buff, 1, timestamp));

	*number_of_frames = buff[0];

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getButtonState(U8* button_state, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[1];

	TSS_ERROR_CHECK(_checkedCommandWriteRead(0xFA, buff, 1, timestamp));

	*button_state = buff[0];

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setMouseAbsoluteRelativeMode(U8 mode, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[2];
	buff[0] = 0xFB;

	memcpy(buff + 1, &mode, 1);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 2, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getMouseAbsoluteRelativeMode(U8* mode, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[1];

	TSS_ERROR_CHECK(_checkedCommandWriteRead(0xFC, buff, 1, timestamp));

	*mode = buff[0];

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setJoystickAndMousePresentRemoved(U8 joystick, U8 mouse, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[3];
	buff[0] = 0xFD;

	memcpy(buff + 1, &joystick, 1);
	memcpy(buff + 2, &joystick, 1);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 3, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getJoystickAndMousePresentRemoved(U8* joystick, U8* mouse, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[2];

	TSS_ERROR_CHECK(_checkedCommandWriteRead(0xFE, buff, 2, timestamp));

	*joystick = buff[0];
	*mouse = buff[1];

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setPedestrianTrackingState(U8 state, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	float state_info = state;

	U8 buff[6];
	buff[0] = 0x34;
	buff[1] = 0x00;
	memcpy(buff + 2, &state_info, 4);
	yost::swap32(buff + 2);
	
	TSS_RETRY_CHECK(_sendCommandBytes(buff, 6, timestamp), 5);	

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setSelectedStepIndex(U16 index, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	float index_info = index;

	U8 buff[6];
	buff[0] = 0x34;
	buff[1] = 0x01;
	memcpy(buff + 2, &index_info, 4);
	yost::swap32(buff + 2);

	TSS_RETRY_CHECK(_sendCommandBytes(buff, 6, timestamp), 5);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setStepRatioMinimum(float min_ratio, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[6];

	buff[0] = 0x34;
	buff[1] = 0x02;
	memcpy(buff + 2, &min_ratio, 4);
	yost::swap32(buff + 2);

	TSS_RETRY_CHECK(_sendCommandBytes(buff, 6, timestamp), 5);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setStepRatioMaximum(float max_ratio, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[6];

	buff[0] = 0x34;
	buff[1] = 0x03;
	memcpy(buff + 2, &max_ratio, 4);
	yost::swap32(buff + 2);

	TSS_RETRY_CHECK(_sendCommandBytes(buff, 6, timestamp), 5);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setStepDutyCycleMinimum(float min_duty_cycle, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[6];

	buff[0] = 0x34;
	buff[1] = 0x04;
	memcpy(buff + 2, &min_duty_cycle, 4);
	yost::swap32(buff + 2);

	TSS_RETRY_CHECK(_sendCommandBytes(buff, 6, timestamp), 5);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setStepDutyCycleMaximum(float max_duty_cycle, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[6];

	buff[0] = 0x34;
	buff[1] = 0x05;
	memcpy(buff + 2, &max_duty_cycle, 4);
	yost::swap32(buff + 2);

	TSS_RETRY_CHECK(_sendCommandBytes(buff, 6, timestamp), 5);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setStepAmplitudeMinimum(float min_amplitue, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[6];

	buff[0] = 0x34;
	buff[1] = 0x06;
	memcpy(buff + 2, &min_amplitue, 4);
	yost::swap32(buff + 2);

	TSS_RETRY_CHECK(_sendCommandBytes(buff, 6, timestamp), 5);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setStepAmplitudeMaximum(float max_amplitude, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[6];

	buff[0] = 0x34;
	buff[1] = 0x07;
	memcpy(buff + 2, &max_amplitude, 4);
	yost::swap32(buff + 2);

	TSS_RETRY_CHECK(_sendCommandBytes(buff, 6, timestamp), 5);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setStepDurationMinimum(float min_step_duration, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[6];

	buff[0] = 0x34;
	buff[1] = 0x08;
	memcpy(buff + 2, &min_step_duration, 4);
	yost::swap32(buff + 2);

	TSS_RETRY_CHECK(_sendCommandBytes(buff, 6, timestamp), 5);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setStepDurationMaximum(float max_step_duration, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[6];

	buff[0] = 0x34;
	buff[1] = 0x09;
	memcpy(buff + 2, &max_step_duration, 4);
	yost::swap32(buff + 2);

	TSS_RETRY_CHECK(_sendCommandBytes(buff, 6, timestamp), 6);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setStepStrideSlope(float slope, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[6];

	buff[0] = 0x34;
	buff[1] = 0x0A;
	memcpy(buff + 2, &slope, 4);
	yost::swap32(buff + 2);

	TSS_RETRY_CHECK(_sendCommandBytes(buff, 6, timestamp), 5);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setStepStrideOffset(float offset, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[6];

	buff[0] = 0x34;
	buff[1] = 0x0B;
	memcpy(buff + 2, &offset, 4);
	yost::swap32(buff + 2);

	TSS_RETRY_CHECK(_sendCommandBytes(buff, 6, timestamp), 5);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setPedestrianTrackingUnits(U8 unit_select, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	float unit_data = unit_select;
	U8 buff[6];

	buff[0] = 0x34;
	buff[1] = 0x0C;
	memcpy(buff + 2, &unit_data, 4);
	yost::swap32(buff + 2);

	TSS_RETRY_CHECK(_sendCommandBytes(buff, 6, timestamp), 5);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::setBarometerAltitudeOffset(float offset, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[6];

	buff[0] = 0x34;
	buff[1] = 0x0D;
	memcpy(buff + 2, &offset, 4);
	yost::swap32(buff + 2);

	TSS_RETRY_CHECK(_sendCommandBytes(buff, 6, timestamp), 5);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::autoSetBarometerAltitudeOffset(float known_height, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[6];

	buff[0] = 0x34;
	buff[1] = 0x0E;
	memcpy(buff + 2, &known_height, 4);
	yost::swap32(buff + 2);

	TSS_RETRY_CHECK(_sendCommandBytes(buff, 6, timestamp), 5);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::resetSteps(U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	float dummy = 0;
	
	U8 buff[6];
	buff[0] = 0x34;
	buff[1] = 0x0F;
	memcpy(buff + 2, &dummy, 4);
	// no need to swap32 a dummy that will always be zero

	TSS_RETRY_CHECK(_sendCommandBytes(buff, 6, timestamp), 1);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getPedestrianTrackingState(U8* state, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[4];
	buff[0] = 0x35;
	buff[1] = 0x00;

	TSS_RETRY_CHECK(_sendCommandBytes(buff, 2, timestamp), 5);

	TSS_ERROR_CHECK(_readBytes(buff, 4));

	yost::swap32(buff);

	float f_state = *(float*)buff;
	*state = (U8)f_state;	

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getSelectedStepIndex(U16* index, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[4];
	buff[0] = 0x35;
	buff[1] = 0x01;

	TSS_RETRY_CHECK(_sendCommandBytes(buff, 2, timestamp), 5);

	TSS_ERROR_CHECK(_readBytes(buff, 4));	

	yost::swap32(buff);

	float f_state = *(float*)buff;
	*index = (U16)f_state;

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getStepRatioMinimum(float* min_ratio, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[4];
	buff[0] = 0x35;
	buff[1] = 0x02;

	TSS_RETRY_CHECK(_sendCommandBytes(buff, 2, timestamp), 5);

	TSS_ERROR_CHECK(_readBytes(buff, 4));

	yost::swap32(buff);

	memcpy(min_ratio, buff, 4);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getStepRatioMaximum(float* max_ratio, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[4];
	buff[0] = 0x35;
	buff[1] = 0x03;

	TSS_RETRY_CHECK(_sendCommandBytes(buff, 2, timestamp), 5);

	TSS_ERROR_CHECK(_readBytes(buff, 4));

	yost::swap32(buff);

	memcpy(max_ratio, buff, 4);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getStepDutyCycleMinimum(float* min_duty_cycle, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[4];
	buff[0] = 0x35;
	buff[1] = 0x04;

	TSS_RETRY_CHECK(_sendCommandBytes(buff, 2, timestamp), 5);

	TSS_ERROR_CHECK(_readBytes(buff, 4));

	yost::swap32(buff);

	memcpy(min_duty_cycle, buff, 4);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getStepDutyCycleMaximum(float* max_duty_cycle, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[4];
	buff[0] = 0x35;
	buff[1] = 0x05;

	TSS_RETRY_CHECK(_sendCommandBytes(buff, 2, timestamp), 5);

	TSS_ERROR_CHECK(_readBytes(buff, 4));

	yost::swap32(buff);

	memcpy(max_duty_cycle, buff, 4);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getStepAmplitudeMinimum(float* min_amplitude, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[4];
	buff[0] = 0x35;
	buff[1] = 0x06;

	TSS_RETRY_CHECK(_sendCommandBytes(buff, 2, timestamp), 5);

	TSS_ERROR_CHECK(_readBytes(buff, 4));

	yost::swap32(buff);

	memcpy(min_amplitude, buff, 4);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getStepAmplitudeMaximum(float* max_amplitude, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[4];
	buff[0] = 0x35;
	buff[1] = 0x07;

	TSS_RETRY_CHECK(_sendCommandBytes(buff, 2, timestamp), 5);

	TSS_ERROR_CHECK(_readBytes(buff, 4));

	yost::swap32(buff);

	memcpy(max_amplitude, buff, 4);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getStepDurationMinimum(float* min_step_duration, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[4];
	buff[0] = 0x35;
	buff[1] = 0x08;

	TSS_RETRY_CHECK(_sendCommandBytes(buff, 2, timestamp), 5);

	TSS_ERROR_CHECK(_readBytes(buff, 4));

	yost::swap32(buff);

	memcpy(min_step_duration, buff, 4);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getStepDurationMaximum(float* max_step_duration, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[4];
	buff[0] = 0x35;
	buff[1] = 0x09;

	TSS_RETRY_CHECK(_sendCommandBytes(buff, 2, timestamp), 5);

	TSS_ERROR_CHECK(_readBytes(buff, 4));

	yost::swap32(buff);

	memcpy(max_step_duration, buff, 4);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getStepStrideSlope(float* slope, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[4];
	buff[0] = 0x35;
	buff[1] = 0x0A;

	TSS_RETRY_CHECK(_sendCommandBytes(buff, 2, timestamp), 5);

	TSS_ERROR_CHECK(_readBytes(buff, 4));

	yost::swap32(buff);

	memcpy(slope, buff, 4);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getStepStrideOffset(float* offset, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[4];
	buff[0] = 0x35;
	buff[1] = 0x0B;

	TSS_RETRY_CHECK(_sendCommandBytes(buff, 2, timestamp), 5);

	TSS_ERROR_CHECK(_readBytes(buff, 4));

	yost::swap32(buff);

	memcpy(offset, buff, 4);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getPedestrianTrackingUnits(U8* unit_select, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[4];
	buff[0] = 0x35;
	buff[1] = 0x0C;

	TSS_RETRY_CHECK(_sendCommandBytes(buff, 2, timestamp), 5);

	TSS_ERROR_CHECK(_readBytes(buff, 4));
	
	yost::swap32(buff);

	float f_state = *(float*)buff;
	*unit_select = (U8)f_state;

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getAltitudeOffset(float* offset, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[4];
	buff[0] = 0x35;
	buff[1] = 0x0D;

	TSS_RETRY_CHECK(_sendCommandBytes(buff, 2, timestamp), 5);

	TSS_ERROR_CHECK(_readBytes(buff, 4));

	yost::swap32(buff);

	memcpy(offset, buff, 4);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getCurrentAltitude(float* altitude, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[4];
	buff[0] = 0x35;
	buff[1] = 0x0E;

	TSS_RETRY_CHECK(_sendCommandBytes(buff, 2, timestamp), 5);

	TSS_ERROR_CHECK(_readBytes(buff, 4));

	yost::swap32(buff);

	memcpy(altitude, buff, 4);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getFirstAltitude(float* altitude, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[4];
	buff[0] = 0x35;
	buff[1] = 0x0F;

	TSS_RETRY_CHECK(_sendCommandBytes(buff, 2, timestamp), 5);

	TSS_ERROR_CHECK(_readBytes(buff, 4));

	yost::swap32(buff);

	memcpy(altitude, buff, 4);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getAltitudeDifference(float* difference, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[4];
	buff[0] = 0x35;
	buff[1] = 0x10;

	TSS_RETRY_CHECK(_sendCommandBytes(buff, 2, timestamp), 5);

	TSS_ERROR_CHECK(_readBytes(buff, 4));

	yost::swap32(buff);

	memcpy(difference, buff, 4);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getCurrentPressure(float* pressure, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[4];
	buff[0] = 0x35;
	buff[1] = 0x11;

	TSS_RETRY_CHECK(_sendCommandBytes(buff, 2, timestamp), 5);

	TSS_ERROR_CHECK(_readBytes(buff, 4));

	yost::swap32(buff);

	memcpy(pressure, buff, 4);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getCurrentHeading(float* heading, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[4];
	buff[0] = 0x35;
	buff[1] = 0x12;

	TSS_RETRY_CHECK(_sendCommandBytes(buff, 2, timestamp), 5);

	TSS_ERROR_CHECK(_readBytes(buff, 4));

	yost::swap32(buff);

	memcpy(heading, buff, 4);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getStepBufferLength(U16* step_buffer_length, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[4];
	buff[0] = 0x35;
	buff[1] = 0x13;

	TSS_RETRY_CHECK(_sendCommandBytes(buff, 2, timestamp), 5);

	TSS_ERROR_CHECK(_readBytes(buff, 4));

	yost::swap32(buff);

	float f_state = *(float*)buff;
	*step_buffer_length = (U16)f_state;	
	
	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getTotalDistanceTraveled(float* distance, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[4];
	buff[0] = 0x35;
	buff[1] = 0x14;

	TSS_RETRY_CHECK(_sendCommandBytes(buff, 2, timestamp), 5);

	TSS_ERROR_CHECK(_readBytes(buff, 4));

	yost::swap32(buff);

	memcpy(distance, buff, 4);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getStepSessionConfidenceValue(float* confidence, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[4];
	buff[0] = 0x35;
	buff[1] = 0x15;

	TSS_RETRY_CHECK(_sendCommandBytes(buff, 2, timestamp), 5);

	TSS_ERROR_CHECK(_readBytes(buff, 4));

	yost::swap32(buff);

	memcpy(confidence, buff, 4);

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getLatestStep(float* step, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	step[0] = -1.0f;
	memset(step + 1, 0, sizeof(float) * 11);

	U8 buff[2];
	buff[0] = 0x35;
	buff[1] = 0x16;

	TSS_RETRY_CHECK(_sendCommandBytes(buff, 2, timestamp), 5);

	TSS_ERROR_CHECK(_readFloats(step, 1));

	if (step[0] == -1.0f)
	{				
		return TSS_NO_ERROR;
	}
	
	TSS_ERROR_CHECK(_readFloats(step + 1, 11));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::getStepAtSelectedIndex(float* step, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	step[0] = -1.0f;
	memset(step + 1, 0, sizeof(float) * 11);

	U8 buff[2];
	buff[0] = 0x35;
	buff[1] = 0x17;

	TSS_RETRY_CHECK(_sendCommandBytes(buff, 2, timestamp), 5);

	TSS_ERROR_CHECK(_readFloats(step, 1));

	if (step[0] == -1.0f)
	{		
		return TSS_NO_ERROR;
	}

	TSS_ERROR_CHECK(_readFloats(step + 1, 11));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::manualWrite(U8* bytes, U16 amount, U32* timestamp)
{
	// No Bootloader Check so users can still manually talk to the bootloader if they want to.
	if (!isConnected()) { return TSS_ERROR_NOT_CONNECTED; }
	if (_isStreaming) { return TSS_ERROR_COMMAND_CALLED_DURING_STREAMING; }
	if (_inErrorState) { return TSS_ERROR_NOT_CONNECTED; }
	
	TSS_ERROR_CHECK(_writeBytes(bytes, amount));

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::manualRead(U8* bytes, U16 amount, U16* read_count, U32* timestamp)
{
	// No Bootloader Check so users can still manually talk to the bootloader if they want to.
	if (!isConnected()) { return TSS_ERROR_NOT_CONNECTED; }
	if (_isStreaming) { return TSS_ERROR_COMMAND_CALLED_DURING_STREAMING; }
	if (_inErrorState) { return TSS_ERROR_NOT_CONNECTED; }

	size_t read_amount = 0;
	size_t available = _available();

	if (amount > available)
		read_amount = available;
	else
		read_amount = amount;

	if (read_amount > 65536)
		read_amount = 65536;

	unsigned long read = 0;
	try
	{		
		read = _port->read(bytes, read_amount);
	}
	catch (...)
	{
		_inErrorState = true;
		return TSS_ERROR_NOT_CONNECTED;
	}
	
	*read_count = (U16)read;

	return TSS_NO_ERROR;
}

TSS_ERROR TssSensor::manualFlush(U32* timestamp)
{
	// No Bootloader Check so users can still manually talk to the bootloader if they want to.
	if (!isConnected()) { return TSS_ERROR_NOT_CONNECTED; }
	if (_isStreaming) { return TSS_ERROR_COMMAND_CALLED_DURING_STREAMING; }
	if (_inErrorState) { return TSS_ERROR_NOT_CONNECTED; }

	if (this->_port.get() != nullptr && this->_port->isOpen())
	{
		_flushInput();
		_flush();
	}

	return TSS_NO_ERROR;
}
