#include "threespace_dongle.hpp"
#include "threespace_api.hpp"

TssDongle::TssDongle(std::string port)
{
    _type = TSS_DEVICE_TYPE_DONGLE;

    result = _openPort(port);

    if (result != TSS_NO_ERROR)
    {
        closePort();
        return;
    }

	U32 timestamp;
    _sendCommand(85, &timestamp); // pause streaming output

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

    if (_sensorType != TSS_DONGLE)
    {
        closePort();
        return;
    }

    _childCount = 0;

    _responseHeaderWirelessFlags = 0;

    _setWirelessResponseHeader(TSS_RESPONSE_HEADER_SUCCESS | TSS_RESPONSE_HEADER_LOGICAL_ID | TSS_RESPONSE_HEADER_DATA_LENGTH);

    _maxStreamInterval = 0;

    _children.resize(TSS_DONGLE_NUM_CHILDREN, NULL);
}

TssDongle::TssDongle(const TssDongle& other)
{
    throw runtime_error("Deep copies of dongle objects not allowed.");
}

void TssDongle::operator =(const TssDongle& other)
{
    throw runtime_error("Deep copies of dongle objects not allowed.");
}

TSS_ERROR TssDongle::openPort(std::string port)
{
    if (_port)
    {
        return TSS_ERROR_ALREADY_CONNECTED;
    }

    _openPort(port);

    return TSS_NO_ERROR;
}

TSS_ERROR TssDongle::closePort()
{
    BASIC_CALL_CHECK_TSS();

	try
	{
		_port->close();
	}
	catch (...)
	{
		// We are closing and trying to avoid a crash, so do nothing but catch.
	}

    for (U8 i = 0; i < _children.size(); i++)
    {
        if (_children[i] != nullptr)
        {
            _children[i]->_ownerDongle = NULL;
            _children[i]->_port = NULL;
            _children[i].reset();
        }
    }

    _port.reset();

    return TSS_NO_ERROR;
}

TSS_ERROR TssDongle::enableTimestampsWireless()
{
    BASIC_CALL_CHECK_TSS();

    _setWirelessResponseHeader(_responseHeaderWirelessFlags | TSS_RESPONSE_HEADER_TIMESTAMP);

    return TSS_NO_ERROR;
}

TSS_ERROR TssDongle::disableTimestampsWireless()
{
    BASIC_CALL_CHECK_TSS();

    _setWirelessResponseHeader(_responseHeaderWirelessFlags & ~TSS_RESPONSE_HEADER_TIMESTAMP);

    return TSS_NO_ERROR;
}

TSS_ERROR TssDongle::getWirelessSensor(U8 logical_id, shared_ptr<TssSensor>& sensor)
{
    BASIC_CALL_CHECK_TSS();

    if (logical_id >= TSS_DONGLE_NUM_CHILDREN)
    {
        return TSS_ERROR_LOGICAL_ID_OUT_OF_RANGE;
    }

    if (_children.size() > logical_id && _children[logical_id])
    {
        return TSS_ERROR_CHILD_EXISTS;
    }

    sensor = shared_ptr<TssSensor>(new TssSensor());

    sensor->_port = _port;
    sensor->_ownerDongle = this;
    sensor->_isWireless = true;
    sensor->_logicalId = logical_id;
    sensor->_commandRetries = TSS_DEFAULT_WIRELESS_COMMAND_RETRIES;

    sensor->_setResponseHeader(_responseHeaderWirelessFlags);

	U32 timestamp;
    //in an ideal case, it would be better to be able to tell when it had stopped streaming
    //and stop sending the commands at that point, but attempts to use the streaming bitfield
    //here yielded poor results.  we should revisit this and find a better way to do this
    //when we get the chance.
    for (U8 i = 0; i < TSS_STOP_STREAM_RETRIES; i++)
    {
        sensor->_sendCommand(86, &timestamp); //stop streaming command, [wait for results, otherwise weird stuff happens - Keith]
    }

    //sleep for a long time to make sure the dongle has processed all the commands and responded to each
    yost::sleep_ms(500);

    //dump data from all the stop streaming commands
    _purgeInput();

    #define TSS_WIRELESS_SENSOR_SERIAL_RETRIES 50

    bool success = false;
    for (U32 i = 0; i < TSS_WIRELESS_SENSOR_SERIAL_RETRIES; i++)
    {
        result = sensor->_readSerialNumber(&timestamp);

        if (result == TSS_NO_ERROR)
        {
            success = true;
            break;
        }
    }

    if (!success)
    {
        sensor.reset();
        return TSS_ERROR_CANT_READ_INIT_DATA;
    }

    success = false;
    for (U32 i = 0; i < TSS_WIRELESS_SENSOR_SERIAL_RETRIES; i++)
    {
        result = sensor->_readVersionString(&timestamp);

        if (result == TSS_NO_ERROR)
        {
            success = true;
            break;
        }
    }

    if (!success)
    {
        sensor.reset();
        return TSS_ERROR_CANT_READ_INIT_DATA;
    }


    _children[logical_id] = sensor;
    _childCount++;

    return TSS_NO_ERROR;
}

TSS_ERROR TssDongle::removeWirelessSensor(U8 logical_id)
{
    BASIC_CALL_CHECK_TSS();

    if (logical_id >= TSS_DONGLE_NUM_CHILDREN)
    {
        return TSS_ERROR_LOGICAL_ID_OUT_OF_RANGE;
    }

    if (!_children[logical_id])
    {
        return TSS_ERROR_CHILD_DOESNT_EXIST;
    }

    _childCount--;
    _children[logical_id].reset();

    return TSS_NO_ERROR;
}

TSS_ERROR TssDongle::_setWirelessResponseHeader(U32 flags)
{
	U32 timestamp;
    U8 buff[256];
    buff[0] = 219; // set wireless response header command
    memcpy(buff + 1, &flags, 4);
    yost::swap32(buff + 1);
    _sendCommandBytes(buff,5, &timestamp);

    _sendCommand(220, &timestamp); // get wireless response header command
    _readBytes(buff, 4);
	yost::swap32(buff);
	U32 read_flags = *((U32*)buff);

	flags = read_flags;

    for (vector<shared_ptr<TssSensor>>::iterator it = _children.begin(); it != _children.end(); ++it)
    {
        if ((*it))
            (*it)->_setResponseHeader(flags);
    }

    _responseHeaderWirelessFlags = flags;
    _responseHeaderWirelessSize = 0;

    if (_responseHeaderWirelessFlags & TSS_RESPONSE_HEADER_SUCCESS)
    {
        _responseHeaderWirelessSize += 1;
    }

    if (_responseHeaderWirelessFlags & TSS_RESPONSE_HEADER_TIMESTAMP)
    {
        _responseHeaderWirelessSize += 4;
    }

    if (_responseHeaderWirelessFlags & TSS_RESPONSE_HEADER_COMMAND_ECHO)
    {
        _responseHeaderWirelessSize += 1;
    }

    if (_responseHeaderWirelessFlags & TSS_RESPONSE_HEADER_CHECKSUM)
    {
        _responseHeaderWirelessSize += 1;
    }

    if (_responseHeaderWirelessFlags & TSS_RESPONSE_HEADER_LOGICAL_ID)
    {
        _responseHeaderWirelessSize += 1;
    }

    if (_responseHeaderWirelessFlags & TSS_RESPONSE_HEADER_SERIAL_NUMBER)
    {
        _responseHeaderWirelessSize += 4;
    }

    if (_responseHeaderWirelessFlags & TSS_RESPONSE_HEADER_DATA_LENGTH)
    {
        _responseHeaderWirelessSize += 1;
    }

    return TSS_NO_ERROR;
}

TSS_ERROR TssDongle::enableAllSensorsAndStartStreaming(U32 data_flags, U32 interval, U32 duration)
{
    BASIC_CALL_CHECK_TSS();

    U8 count = 0;
    for (auto& child : _children)
    {
        if (!child)
        {
            continue;
        }

        //this automatically handles the delays for all children while starting to stream
        TSS_ERROR result = child->enableStreamingWireless(data_flags, interval, TSS_STREAM_DURATION_INFINITE, 1000);
        //yost::sleep_ms(20);

        if (result != TSS_NO_ERROR)
        {
            return result;
        }
        count++;
    }

    return startStreaming();
}

TSS_ERROR TssDongle::startStreaming()
{
    BASIC_CALL_CHECK_TSS();

	U32 timestamp;
    U8 failures;

    _maxStreamInterval = 0;

    for (int i = 0; i < TSS_START_STREAM_RETRIES; i++)
    {
        failures = 0;

        for (vector<shared_ptr<TssSensor>>::iterator it = _children.begin(); it != _children.end(); ++it)
        {
            if ((*it) && (*it)->_streamDataSize && !(*it)->_isStreaming)
            {
                (*it)->_streamBuffer.clear();

                if ((*it)->_streamInterval > _maxStreamInterval)
                {
                    _maxStreamInterval = (*it)->_streamInterval;
                }
                yost::sleep_ms(20);
                //call start streaming command
                if ((*it)->_sendCommand(85, &timestamp) == TSS_NO_ERROR)
                {
                    (*it)->_isStreaming = true;
                }
                else
                {
                    failures++;
                }
            }
        }

        if (failures == 0)
            break;
    }


    if (failures > 0)
        return TSS_ERROR_NOT_STREAMING;

    TSS_ERROR_CHECK(_sendCommand(85, &timestamp)); //open the floodgates, here it comes

    _isStreaming = true;

    _responseHeaderWirelessFlagsPreStream = _responseHeaderWirelessFlags;

    //always use the logical id header flag so we know what data belongs to
    _setWirelessResponseHeader(_responseHeaderWirelessFlags | TSS_RESPONSE_HEADER_LOGICAL_ID);

    //register this device, since there is now something to stream
    gAPI.registerStreamingDevice(this);

    return TSS_NO_ERROR;
}

U16 TssDongle::_detectStreaming(U32 interval_us)
{
    U16 response;
	U32 timestamp;

    _sendCommand(183, &timestamp);

    result = _readBytes((U8*)&response, 2);


    //the 15 is a fudge factor meant to guarantee any streaming data will be in during the waiting period
    yost::sleep_ms(interval_us / 1000 * 15);


    _sendCommand(183, &timestamp);

    result = _readBytes((U8*)&response, 2);

    if (result == TSS_NO_ERROR)
    {
        return response;
    }

    return 0;
}

TSS_ERROR TssDongle::stopStreaming()
{
	U32 timestamp;
    //in this function, we specifically don't make sure the dongle thinks it is streaming before
    //telling sensors to stop, as we want this to be able to catch any stray streaming sensors

    if (!isConnected())
    {
        return TSS_ERROR_NOT_CONNECTED;
    }

    if (_isStreaming)
    {
        _isStreaming = false;

        gAPI.unregisterStreamingDevice(this);

        _sendCommand(86, &timestamp); //pause streaming output

        _setWirelessResponseHeader(_responseHeaderWirelessFlagsPreStream);
    }

    //dump any extra streaming data
    _purgeInput();

    for (int i = 0; i < TSS_STOP_STREAM_RETRIES; i++)
    {
        for (vector<shared_ptr<TssSensor>>::iterator it = _children.begin(); it != _children.end(); ++it)
        {
            if ((*it))
            {
                (*it)->_sendCommand(86, &timestamp); //stop streaming command
                yost::sleep_ms(20);
            }
        }

        if (_detectStreaming(_maxStreamInterval) == 0x0000)
            break;
    }

    for (vector<shared_ptr<TssSensor>>::iterator it = _children.begin(); it != _children.end(); ++it)
    {
        if ((*it))
        {
            (*it)->_isStreaming = false;
        }
    }


    if (_detectStreaming(_maxStreamInterval) != 0x0000)
    {
        _maxStreamInterval = 0;
        return TSS_ERROR_CANT_STOP_STREAMING;
    }

    _maxStreamInterval = 0;

    return TSS_NO_ERROR;
}

TSS_ERROR TssDongle::softwareReset()
{
	if (!isConnected()) { return TSS_ERROR_NOT_CONNECTED; }
	if (_isStreaming) { return TSS_ERROR_COMMAND_CALLED_DURING_STREAMING; }
	if (_inErrorState) { return TSS_ERROR_NOT_CONNECTED; }

	TSS_ERROR_CHECK(_softwareReset());

	return TSS_NO_ERROR;
}

TSS_ERROR TssDongle::setLEDMode(U8 led_mode, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[2];
	buff[0] = 0xC4;
	buff[1] = led_mode;

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 2, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssDongle::getLEDMode(U8* led_mode, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[1];

	TSS_ERROR_CHECK(_checkedCommandWriteRead(0xC8, buff, 1, timestamp));

	memcpy(led_mode, buff, 1);

	return TSS_NO_ERROR;
}

TSS_ERROR TssDongle::setLEDColor(const float* rgb_color3, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[13];
	buff[0] = 0xEE;
	
	memcpy(buff + 1, rgb_color3, 12);

	yost::swap32(buff + 1);
	yost::swap32(buff + 5);
	yost::swap32(buff + 9);



	TSS_ERROR_CHECK(_sendCommandBytes(buff, 13, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssDongle::getLEDColor(float* rgb_color3, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[12];

	TSS_ERROR_CHECK(_checkedCommandWriteRead(0xEF, buff, 12, timestamp));

	yost::swap32(buff);
	yost::swap32(buff + 4);
	yost::swap32(buff + 8);

	memcpy(rgb_color3, buff, 12);

	return TSS_NO_ERROR;
}

TSS_ERROR TssDongle::setWiredResponseHeaderBitfield(U32 bitfield, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[5];
	buff[0] = 0xDD;
	memcpy(buff + 1, &bitfield, 4);

	yost::swap32(buff + 1);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 5, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssDongle::getWiredResponseHeaderBitfield(U32* bitfield, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[4];

	TSS_ERROR_CHECK(_checkedCommandWriteRead(0xDE, buff, 4, timestamp));

	yost::swap32(buff);
	memcpy(bitfield, buff, 4);

	return TSS_NO_ERROR;
}

TSS_ERROR TssDongle::getWirelessChannel(U8* channel, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[1];

    TSS_ERROR_CHECK(_checkedCommandWriteRead(0xC2, buff, 1, timestamp));

	memcpy(channel, buff, 1);

    return TSS_NO_ERROR;
}

TSS_ERROR TssDongle::setWirelessChannel(U8 channel, U32* timestamp)
{
    BASIC_CALL_CHECK_TSS();

    U8 buff[2];
    buff[0] = 0xC3;
    buff[1] = channel;

    TSS_ERROR_CHECK(_sendCommandBytes(buff, 2, timestamp));

    return TSS_NO_ERROR;
}

TSS_ERROR TssDongle::getWirelessPanID(U16* panid, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[2];

    TSS_ERROR_CHECK(_checkedCommandWriteRead(0xC0, buff, 2, timestamp));

    yost::swap16(buff);

	memcpy(panid, buff, 2);

    return TSS_NO_ERROR;
}

TSS_ERROR TssDongle::setWirelessPanID(U16 panid, U32* timestamp)
{
    BASIC_CALL_CHECK_TSS();

    U8 buff[3];
    buff[0] = 0xC1;

    memcpy(buff + 1, &panid, 2);
    yost::swap16((U8*)buff + 1);

    TSS_ERROR_CHECK(_sendCommandBytes(buff, 3, timestamp));

    return TSS_NO_ERROR;
}

TSS_ERROR TssDongle::getSerialNumberAtLogicalID(U8 logical_id, U32* serial_number, U32* timestamp)
{
    BASIC_CALL_CHECK_TSS();

    U8 buff[4];
    buff[0] = 0xD0;
    buff[1] = logical_id;

    TSS_ERROR_CHECK(_sendCommandBytes(buff, 2, timestamp));

    TSS_ERROR_CHECK(_readBytes(buff, 4));

    yost::swap32(buff);

	memcpy(serial_number, buff, 4);

    return TSS_NO_ERROR;
}

TSS_ERROR TssDongle::setSerialNumberAtLogicalID(U8 logical_id, U32 serial_number, U32* timestamp)
{
    BASIC_CALL_CHECK_TSS();

    U8 buff[6];
    buff[0] = 0xD1;
    buff[1] = logical_id;

    memcpy(buff + 2, &serial_number, 4);
    yost::swap32(buff + 2);

    TSS_ERROR_CHECK(_sendCommandBytes(buff, 6, timestamp));

    return TSS_NO_ERROR;
}

TSS_ERROR TssDongle::commitWirelessSettings(U32* timestamp)
{
    BASIC_CALL_CHECK_TSS();

    TSS_ERROR_CHECK(_sendCommand(0xc5, timestamp));

    return TSS_NO_ERROR;
}

TSS_ERROR TssDongle::getWirelessAddress(U16* address, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[2];

	TSS_ERROR_CHECK(_checkedCommandWriteRead(0xC6, buff, 2, timestamp));

	yost::swap16(buff);

	memcpy(address, buff, 2);

	return TSS_NO_ERROR;
}

TSS_ERROR TssDongle::setWirelessStreamingAutoFlushMode(U8 mode, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[2];
	buff[0] = 0xB0;

	memcpy(buff + 1, &mode, 1);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 2, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssDongle::getWirelessStreamingAutoFlushMode(U8* mode, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[1];

	TSS_ERROR_CHECK(_checkedCommandWriteRead(0xB1, buff, 1, timestamp));

	memcpy(mode, buff, 1);

	return TSS_NO_ERROR;
}

TSS_ERROR TssDongle::setWirelessStreamingManualFlushBitfield(U16 manual_flush_bitfield, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[3];
	buff[0] = 0xB2;

	memcpy(buff + 1, &manual_flush_bitfield, 2);

	yost::swap16(buff + 1);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 3, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssDongle::getWirelessStreamingManualFlushBitfield(U16* manual_flush_bitfield, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[2];

	TSS_ERROR_CHECK(_checkedCommandWriteRead(0xB3, buff, 2, timestamp));

	yost::swap16(buff);

	memcpy(manual_flush_bitfield, buff, 2);

	return TSS_NO_ERROR;
}

TSS_ERROR TssDongle::broadcastSynchronizationPulse(U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	TSS_ERROR_CHECK(_sendCommand(0xB6, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssDongle::getWirelessChannelNoiseLevels(U8* channel_strengths16, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[1];

	TSS_ERROR_CHECK(_checkedCommandWriteRead(0xD2, buff, 1, timestamp));

	memcpy(channel_strengths16, buff, 1);

	return TSS_NO_ERROR;
}

TSS_ERROR TssDongle::setWirelessRetries(U8 retries, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[2];
	buff[0] = 0xD3;

	memcpy(buff + 1, &retries, 1);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 2, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssDongle::getWirelessRetries(U8* retries, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[1];

	TSS_ERROR_CHECK(_checkedCommandWriteRead(0xD4, buff, 1, timestamp));

	memcpy(retries, buff, 1);

	return TSS_NO_ERROR;
}

TSS_ERROR TssDongle::getWirelessSlotsOpen(U8* slots_open, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[1];

	TSS_ERROR_CHECK(_checkedCommandWriteRead(0xD5, buff, 1, timestamp));

	memcpy(slots_open, buff, 1);

	return TSS_NO_ERROR;
}

TSS_ERROR TssDongle::getSignalStrength(U8* last_packet_signal_strength, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[1];

	TSS_ERROR_CHECK(_checkedCommandWriteRead(0xD6, buff, 1, timestamp));

	memcpy(last_packet_signal_strength, buff, 1);

	return TSS_NO_ERROR;
}

TSS_ERROR TssDongle::setWirelessResponseHeaderBitfield(U32 header_bitfield, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[5];
	buff[0] = 0xDB;

	memcpy(buff + 1, &header_bitfield, 4);
	yost::swap32(buff + 1);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 5, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssDongle::getWirelessResponseHeaderBitfield(U32* header_bitfield, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[4];

	TSS_ERROR_CHECK(_checkedCommandWriteRead(0xDC, buff, 4, timestamp));

	yost::swap32(buff);

	memcpy(header_bitfield, buff, 4);

	return TSS_NO_ERROR;
}

TSS_ERROR TssDongle::manualWrite(U8* bytes, U16 amount, U32* timestamp)
{
	// No Bootloader Check so users can still manually talk to the bootloader if they want to.
	if (!isConnected()) { return TSS_ERROR_NOT_CONNECTED; }
	if (_isStreaming) { return TSS_ERROR_COMMAND_CALLED_DURING_STREAMING; }
	if (_inErrorState) { return TSS_ERROR_NOT_CONNECTED; }

	TSS_ERROR_CHECK(_writeBytes(bytes, amount));

	return TSS_NO_ERROR;
}

TSS_ERROR TssDongle::manualRead(U8* bytes, U16 amount, U16* read_count, U32* timestamp)
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

TSS_ERROR TssDongle::manualFlush(U32* timestamp)
{
	// No Bootloader Check so users can still manually talk to the bootloader if they want to.
	if (!isConnected()) { return TSS_ERROR_NOT_CONNECTED; }
	if (_isStreaming) { return TSS_ERROR_COMMAND_CALLED_DURING_STREAMING; }
	if (_inErrorState) { return TSS_ERROR_NOT_CONNECTED; }

	if (this->_port.get() != nullptr && this->_port->isOpen())
	{
		this->_flushInput();
		this->_flush();
	}

	return TSS_NO_ERROR;
}

//Wireless HID Commands
TSS_ERROR TssDongle::setWirelessHIDUpdateRate(U8 HID_update_rate, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[2];
	buff[0] = 0xD7;

	memcpy(buff + 1, &HID_update_rate, 1);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 2, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssDongle::getWirelessHIDUpdateRate(U8* HID_update_rate, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[1];

	TSS_ERROR_CHECK(_checkedCommandWriteRead(0xD8, buff, 1, timestamp));

	memcpy(HID_update_rate, buff, 1);

	return TSS_NO_ERROR;
}

TSS_ERROR TssDongle::setWirelessHIDAsynchronousMode(U8 HID_communication_mode, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[2];
	buff[0] = 0xD9;

	memcpy(buff + 1, &HID_communication_mode, 1);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 2, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssDongle::getWirelessHIDAsynchronousMode(U8* HID_communication_mode, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[1];

	TSS_ERROR_CHECK(_checkedCommandWriteRead(0xDA, buff, 1, timestamp));

	memcpy(HID_communication_mode, buff, 1);

	return TSS_NO_ERROR;
}

TSS_ERROR TssDongle::setJoystickLogicalID(U8 logical_ID, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[2];
	buff[0] = 0xF0;

	memcpy(buff + 1, &logical_ID, 1);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 2, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssDongle::setMouseLogicalID(U8 logical_ID, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[2];
	buff[0] = 0xF1;

	memcpy(buff + 1, &logical_ID, 1);

	TSS_ERROR_CHECK(_sendCommandBytes(buff, 2, timestamp));

	return TSS_NO_ERROR;
}

TSS_ERROR TssDongle::getJoystickLogicalID(U8* logical_ID, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[1];

	TSS_ERROR_CHECK(_checkedCommandWriteRead(0xF2, buff, 1, timestamp));

	memcpy(logical_ID, buff, 1);

	return TSS_NO_ERROR;
}

TSS_ERROR TssDongle::getMouseLogicalID(U8* logical_ID, U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

	U8 buff[1];

	TSS_ERROR_CHECK(_checkedCommandWriteRead(0xF3, buff, 1, timestamp));

	memcpy(logical_ID, buff, 1);


	return TSS_NO_ERROR;
}