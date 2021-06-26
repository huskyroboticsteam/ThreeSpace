#include "threespace_device.hpp"
#include <iostream>
#include <thread>
#include <algorithm>
#include <time.h>
#include <fstream>
#include "rapidxml/rapidxml.hpp"

TSS_ERROR result;
U32 ntries;

void TssStreamPacketCircularBuffer::clear()
{
    start = 0;
    end = 0;
    len = 0;
}

void TssStreamPacketCircularBuffer::removeFirstPacket()
{
    if (len == 0)
        return;

    //remove the start element from the buffer, wrapping if needed
    start++;
    if (start == TSS_STREAM_PACKET_CIRCULAR_BUFFER_SIZE)
        start = 0;
    len--;
}

void TssStreamPacketCircularBuffer::addPacket(TSS_Header* header, U8* data, U32 nbytes)
{
    //put the element at the end of the buffer, updating the appropriate indexing variables
    memcpy(&buff[end].header, header, sizeof(TSS_Header));
    memcpy(buff[end].rawData, data, nbytes);
    buff[end].rawDataSize = nbytes;

    len++;
    end++;
    if (end == TSS_STREAM_PACKET_CIRCULAR_BUFFER_SIZE)
        end = 0;

    //if we have too many elements in the list, delete the first one
    if (len > TSS_STREAM_PACKET_CIRCULAR_BUFFER_SIZE)
    {
        start++;
        if (start == TSS_STREAM_PACKET_CIRCULAR_BUFFER_SIZE)
            start = 0;
        len = TSS_STREAM_PACKET_CIRCULAR_BUFFER_SIZE;
    }
}

void TssStreamPacketCircularBuffer::getPacket(TSS_Stream_Packet* packet, U32 index)
{
    memcpy(&packet->header, &buff[index].header, sizeof(TSS_Header));
    memcpy(packet->rawData, buff[index].rawData, buff[index].rawDataSize);
    packet->rawDataSize = buff[index].rawDataSize;
}

void _tssParseResponseHeader(U8* data, U8 flags, TSS_Header* header)
{
    memset(header, 0xff, sizeof(TSS_Header));

    if (flags == 0)
        return;

	header->Flags = flags;

    U8 index = 0;

    if (flags & TSS_RESPONSE_HEADER_SUCCESS)
    {
        memcpy(&header->Success, data + index, 1);
        header->Success = !header->Success; //the header data has true as failure when it comes in, flip it here
        index += 1;
    }

    if (flags & TSS_RESPONSE_HEADER_TIMESTAMP)
    {
        memcpy(&header->SensorTimestamp, data + index, 4);
        yost::swap32((U8*)&header->SensorTimestamp);
        index += 4;
    }

    if (flags & TSS_RESPONSE_HEADER_COMMAND_ECHO)
    {
        memcpy(&header->CommandEcho, data + index, 1);
        index += 1;
    }

    if (flags & TSS_RESPONSE_HEADER_CHECKSUM)
    {
        memcpy(&header->Checksum, data + index, 1);
        index += 1;
    }

    if (flags & TSS_RESPONSE_HEADER_LOGICAL_ID)
    {
        memcpy(&header->LogicalId, data + index, 1);
        index += 1;
    }

    if (flags & TSS_RESPONSE_HEADER_SERIAL_NUMBER)
    {
        memcpy(&header->SerialNumber, data + index, 4);
        yost::swap32((U8*)&header->SerialNumber);
        index += 4;
    }

    if (flags & TSS_RESPONSE_HEADER_DATA_LENGTH)
    {
        memcpy(&header->DataLength, data + index, 1);
        index += 1;
    }

    header->SystemTimestamp = yost::get_time();
}

TssDevice::TssDevice()
{
    _isWireless = false;
    _logicalId = 0xff;
    _isportOwner = false;
    _responseHeaderFlags = 0;
    _responseHeaderSize = 0;
    _commandRetries = 10;
    _isStreaming = false;
}

TssDevice::~TssDevice()
{
    if (_port && _isportOwner)
    {

		try
		{
			_port->close();
		}
		catch (...)
		{
			// We are closing and trying to avoid a crash, so do nothing but catch.
		}
		_port = NULL;
    }
}

bool TssDevice::isConnected()
{
    return _port && _port->isOpen() && !_inErrorState;
}

TSS_Header* TssDevice::getLastHeader()
{
    return &_lastHeader;
}

U32 TssDevice::getSerialNumber()
{
    return _serialNumber;
}

TSS_TYPE TssDevice::getSensorType()
{
    return _sensorType;
}

string TssDevice::getHardwareVersionString()
{
	return _versionString;
}

string TssDevice::getFirmwareVersionString()
{
	return _firmwareString;
}

TSS_ERROR TssDevice::_setResponseHeader(U32 flags)
{
    // send the set response header command, pick wired or wireless based on wireless flag
    if (!_isWireless)
    {
		U32 timestamp;
        U8 buff[5];
        buff[0] = 221;
        memcpy(buff + 1, &flags, 4);
        yost::swap32(buff + 1);
        _sendCommandBytes(buff, 5, &timestamp);
    }

    _responseHeaderFlags = flags;
    _responseHeaderSize = 0;

    if (_responseHeaderFlags & TSS_RESPONSE_HEADER_SUCCESS)
    {
        _responseHeaderSize += 1;
    }

    if (_responseHeaderFlags & TSS_RESPONSE_HEADER_TIMESTAMP)
    {
        _responseHeaderSize += 4;
    }

    if (_responseHeaderFlags & TSS_RESPONSE_HEADER_COMMAND_ECHO)
    {
        _responseHeaderSize += 1;
    }

    if (_responseHeaderFlags & TSS_RESPONSE_HEADER_CHECKSUM)
    {
        _responseHeaderSize += 1;
    }

    if (_responseHeaderFlags & TSS_RESPONSE_HEADER_LOGICAL_ID)
    {
        _responseHeaderSize += 1;
    }

    if (_responseHeaderFlags & TSS_RESPONSE_HEADER_SERIAL_NUMBER)
    {
        _responseHeaderSize += 4;
    }

    if (_responseHeaderFlags & TSS_RESPONSE_HEADER_DATA_LENGTH)
    {
        _responseHeaderSize += 1;
    }

    return TSS_NO_ERROR;
}

TSS_ERROR TssDevice::_openPort(std::string name)
{
    _portName = name;
    Timeout t;

	t.inter_byte_timeout = 154;
    t.read_timeout_multiplier = 100;
    t.read_timeout_constant = 100;
    t.write_timeout_multiplier = 3;
    t.write_timeout_constant = 2;

    try
    {
        _port = shared_ptr<Serial>(new Serial(name, 115200, t));
    }
    catch (...)
    {
        _port.reset();
        return TSS_ERROR_CANT_OPEN_PORT;
    }

    if (!_port->isOpen())
    {
        _port.reset();
        return TSS_ERROR_CANT_OPEN_PORT;
    }

    _isportOwner = true;

    return TSS_NO_ERROR;
}

TSS_ERROR TssDevice::_writeBytes(U8* data, U32 n_bytes, size_t* write_count)
{
	if (_inErrorState)
		return TSS_ERROR_NOT_CONNECTED;

	size_t written = 0;
	try
	{
		written = _port->write(data, n_bytes);
	}
	catch (...)
	{
		_inErrorState = true;
		return TSS_ERROR_NOT_CONNECTED;
	}

	if (write_count != NULL)
	{
		*write_count = written;
	}

	if (written != n_bytes)
	{		
		return TSS_ERROR_NOT_ENOUGH_DATA;
	}

    return TSS_NO_ERROR;
}

TSS_ERROR TssDevice::_readBytes(U8* data, U32 n_bytes, size_t* read_count)
{
	if (_inErrorState)
		return TSS_ERROR_NOT_CONNECTED;

    if (n_bytes == 0)
        return TSS_NO_ERROR;

	size_t read = 0;
	try
	{	
		read = _port->read(data, n_bytes);
	}
	catch (...)
	{
		_inErrorState = true;
		return TSS_ERROR_NOT_CONNECTED;
	}

	if (read_count != NULL)
	{
		*read_count = read;
	}

	if (read != n_bytes)
	{		
		return TSS_ERROR_NOT_ENOUGH_DATA;
	}

    return TSS_NO_ERROR;
}

TSS_ERROR TssDevice::_readResponseHeader()
{
    U8 buff[256];
    TSS_ERROR_CHECK(_readBytes(buff, _responseHeaderSize));
    _tssParseResponseHeader(buff, _responseHeaderFlags, &_lastHeader);
    return TSS_NO_ERROR;
}

TSS_ERROR TssDevice::_sendCommandBytes(const U8* data, U8 n_bytes, U32* timestamp, bool ignore_response)
{
    U8 buff[256];

    U8 index;
    U8 checksum;
    TSS_ERROR result;

    //dump any pre-existing data
	_flushInput();


    //select the corrent command type byte based on wirelessness and header flags
    if (_isWireless)
    {
        checksum = _logicalId;

        if (_responseHeaderFlags == 0)
        {
            buff[0] = 0xf8;
        }
        else
        {
            buff[0] = 0xfa;
        }

        buff[1] = _logicalId;
        index = 2;
    }
    else
    {
        checksum = 0;

        if (_responseHeaderFlags == 0)
        {
            buff[0] = 0xf7;
        }
        else
        {
            buff[0] = 0xf9;
        }

        index = 1;
    }

    //move command data over into the final call buffer and calculate the checksum
    U8 i;
    for (i = 0; i < n_bytes; i++)
    {
        buff[index] = data[i];
        checksum += data[i];
        index++;
    }

    buff[index] = checksum;
    index++;

    TSS_ERROR_CHECK(_writeBytes(buff, index));

    if (ignore_response)
    {
        //do nothing here, caller is expected to purge data once it is done
    }
    else
    {
        //read out header and determine success if possible, but do not read the command data yet
        TSS_ERROR_CHECK(_readResponseHeader());

		if (timestamp != NULL)
		{
			*timestamp = _lastHeader.SensorTimestamp;
		}
        if (_responseHeaderFlags & TSS_RESPONSE_HEADER_SUCCESS)
        {
            if (!_lastHeader.Success)
            {
                return TSS_ERROR_COMMAND_FAILURE;
            }
        }
    }

    return TSS_NO_ERROR;
}

TSS_ERROR TssDevice::_sendCommand(U8 command, U32* timestamp,  bool ignore_response)
{
    U8 buff[1];
    buff[0] = command;

    TSS_ERROR_CHECK(_sendCommandBytes(buff, 1, timestamp, ignore_response));

    return TSS_NO_ERROR;
}

TSS_ERROR TssDevice::_readFloats(float* data, U8 n_floats)
{
    TSS_ERROR_CHECK(_readBytes((U8*)data, n_floats * 4));

    _parseFloats(data, n_floats);

    return TSS_NO_ERROR;
}

TSS_ERROR TssDevice::_parseFloats(float* data, U8 n_floats)
{
    U8 i;
    for (i = 0; i < n_floats; i++)
    {
        yost::swap32((U8*)&(data[i]));
    }

    return TSS_NO_ERROR;
}

TSS_ERROR TssDevice::_checkedCommandWriteRead(U8 command, U8* data, U8 n_bytes, U32* timestamp)
{
    BASIC_CALL_CHECK_TSS();

    for (U8 i = 0; i < _commandRetries; i++)
    {
        result = _sendCommand(command, timestamp);

        if (result != TSS_NO_ERROR)
            continue;

        result = _readBytes(data, n_bytes);

        if (result != TSS_NO_ERROR)
            continue;

        break;
    }

    return result;
}

TSS_ERROR TssDevice::_checkedCommandWriteReadFloats(U8 command, float* data, U8 n_floats, U32* timestamp)
{
    BASIC_CALL_CHECK_TSS();

    for (U8 i = 0; i < _commandRetries; i++)
    {
        result = _sendCommand(command, timestamp);

        if (result != TSS_NO_ERROR)
            continue;

        result = _readFloats(data, n_floats);

		if (result != TSS_NO_ERROR)
		{
			_flushInput();
			_flush();
			continue;
		}

        break;
    }

    return result;
}

void TssDevice::_purgeInput()
{
    yost::sleep_ms(TSS_MAX_COMMAND_RETURN_TIME_MS);

	_flushInput();
}

void TssDevice::_flush()
{
	if (_inErrorState)
		return;

	try
	{		
		_port->flush();
	}
	catch (...)
	{
		_inErrorState = true;
	}
}

void TssDevice::_flushInput()
{
	if (_inErrorState)
		return;

	try
	{
		_port->flushInput();		
	}
	catch (...)
	{
		_inErrorState = true;
	}
}

size_t TssDevice::_available()
{
	if (_inErrorState)
		return 0;

	size_t available = 0;
	try
	{
		available = _port->available();
	}
	catch (...)
	{
		_inErrorState = true;
		return 0;
	}
	return available;
}

TSS_ERROR TssDevice::_readSerialNumber(U32* timestamp)
{
	BASIC_CALL_CHECK_TSS();

    TSS_ERROR_CHECK(_sendCommand(237, timestamp));

    TSS_ERROR_CHECK(_readBytes((U8*)&_serialNumber, 4));

    yost::swap32((U8*)&_serialNumber);

    return TSS_NO_ERROR;
}

TSS_ERROR TssDevice::_readVersionString(U32* timestamp)
{
    TSS_ERROR_CHECK(_sendCommand(230, timestamp));

    U8 buff[33];
    TSS_ERROR_CHECK(_readBytes(buff, 32));
    buff[32] = '\0';

    _versionString = (char*)buff;

	if (_versionString.size() < 4)
	{
		return TSS_ERROR_COMMAND_FAILURE;
	}

    string version_sub = _versionString.substr(4);
    if (yost::startsWith(version_sub, "USB"))
    {
        _sensorType = TSS_USB;
    }
	else if (yost::startsWith(version_sub, "MUSB"))
	{
		_sensorType = TSS_USB;
	}
    else if (yost::startsWith(version_sub, "EM"))
    {
        _sensorType = TSS_EMBEDDED;
    }
    else if (yost::startsWith(version_sub, "WL"))
    {
        _sensorType = TSS_WIRELESS;
    }
    else if (yost::startsWith(version_sub, "DNG"))
    {
        _sensorType = TSS_DONGLE;
    }
    else if (yost::startsWith(version_sub, "DL"))
    {
        _sensorType = TSS_DATALOGGER;
    }
    else if (yost::startsWith(version_sub, "BT"))
    {
        _sensorType = TSS_BLUETOOTH;
    }
	else if (yost::startsWith(version_sub, "LE"))
	{
		_sensorType = TSS_LE;
	}
	else if (yost::startsWith(version_sub, "LX"))
	{
		_sensorType = TSS_LX;
	}
	else if (yost::startsWith(version_sub, "Nano"))
	{
		_sensorType = TSS_NANO;
	}
	else if (yost::startsWith(version_sub, "MBT"))
	{
		_sensorType = TSS_MINIBT;
	}

    return TSS_NO_ERROR;
}

TSS_ERROR TssDevice::_readFirmwareString(U32* timestamp)
{
	TSS_ERROR_CHECK(_sendCommand(223, timestamp));

	U8 buff[13];
	TSS_ERROR_CHECK(_readBytes(buff, 12));
	buff[12] = '\0';

	_firmwareString = (char*)buff;

	if (_firmwareString.size() < 4)
	{
		return TSS_ERROR_COMMAND_FAILURE;
	}

	return TSS_NO_ERROR;
}

TSS_ERROR TssDevice::_softwareReset()
{
	if (_inBootloader)
	{
		// There is no way to reset a bootloader other than power cycling it.
	}
	else
	{
		U8 buffer[4] = { 0xF7, 0xE2, 0xE2, 0xE2 };
		if (_isWireless)
		{
			buffer[0] = 0xFA;
			buffer[1] = _logicalId;
			buffer[3] = (buffer[1] + buffer[2]) % 255;
			_writeBytes(buffer, 4);
		}
		else
		{
			_writeBytes(buffer, 3);
		}
	}

	try
	{
		_port->close();
	}
	catch (...)
	{
		// Don't care, just keep going.
	}

	yost::sleep_ms(2000);

	try
	{
		_port->open();
	}
	catch (...)
	{
		_inErrorState = true;
		return TSS_ERROR_COMMAND_FAILURE;
	}

	_flush();
	_flushInput();

	return TSS_NO_ERROR;
}

TSS_ERROR TssDevice::_portReset(U8* port_open)
{
	_flush();
	_flushInput();

	try
	{
		_port->close();
	}
	catch (...)
	{
		// Don't care, just keep going.
	}

	yost::sleep_ms(100);

	try
	{
		_port->open();
	}
	catch (...)
	{
		*port_open = 0;
		_inErrorState = true;
		return TSS_ERROR_COMMAND_FAILURE;
	}

	_flush();
	_flushInput();

	_inErrorState = false;
	*port_open = 1;
	return TSS_NO_ERROR;
}

U32 TssDevice::_hexCharToInt(U8 c)
{
	const std::string chrs("0123456789ABCDEF");
	return chrs.find(toupper(c));
}

U16 TssDevice::_bytesToU16(U8 byte1, U8 byte2)
{
	U16 dest = 0;
	U8* dest_pointer = (U8*)&dest;

	dest_pointer[0] = byte1;
	dest_pointer[1] = byte2;

	return dest;
}

void TssDevice::_U16ToBytes(U16 val, U8* byte1, U8* byte2)
{
	U8* src = (U8*)&val;

	*byte1 = src[0];
	*byte2 = src[1];
}

I32 TssDevice::_byteStringToInt(std::string info)
{
	I32 dest = 0;
	U8* dest_pointer = (U8*)&dest;

	for (U32 i = 0; i < info.size(); i++)
	{
		dest_pointer[i] = info[i];
	}

	std::reverse(dest_pointer, dest_pointer + info.size());

	return dest;
}

std::string TssDevice::_intToByteString(I32 value, int override_size)
{
	int size = (override_size == -1) ? sizeof(value) : override_size;

	std::string dest;
	U8* src_pointer = (U8*)&value;

	for (I32 i = 0; i < size; i++)
	{
		dest.push_back((unsigned char)src_pointer[i]);
	}

	std::reverse(dest.begin(), dest.end());

	return dest;
}

I32 TssDevice::_hexStringToInt(std::string hexstr)
{
	return (I32)strtol(hexstr.c_str(), nullptr, 16);
}

void TssDevice::_hexStringToByteString(std::string hex_string, std::string* rtn_info, I32* rtn_cdc)
{
	std::string info;
	I32 cdc = 0;

	for (U32 i = 0; i < hex_string.size() / 2; i++)
	{
		I32 int_value = _hexStringToInt(hex_string.substr(i * 2, (i * 2 + 2) - (i * 2)));

		cdc += int_value;
		info += (U8)int_value;
	}

	*rtn_cdc = cdc;
	*rtn_info = info;
}

TSS_ERROR TssDevice::_bootloaderCheck(U8* in_bootloader)
{
	if (!isConnected()) { return TSS_ERROR_NOT_CONNECTED; }
	if (_isStreaming) { return TSS_ERROR_COMMAND_CALLED_DURING_STREAMING; }
	if (_inErrorState) { return TSS_ERROR_NOT_CONNECTED; }

	_flush();
	_flushInput();

	U8 btlr_check[2] = { '?', ' ' };
	TSS_ERROR_CHECK(_writeBytes(btlr_check, 1));

	yost::sleep_ms(100);

	// This should be unchecked, we are expecting that it might not return data.
	_readBytes(btlr_check, 2);

	if (btlr_check[0] == 'O' && btlr_check[1] == 'K')
	{
		_inBootloader = true;
		*in_bootloader = true;
	}
	else
	{
		_inBootloader = false;
		*in_bootloader = false;
	}

	_flush();
	_flushInput();

	return TSS_NO_ERROR;
}

TSS_ERROR TssDevice::_enterBootloader(U32* timestamp)
{	
	BASIC_CALL_CHECK_TSS();

	if (_isWireless)
		return TSS_ERROR_NOT_AVAILABLE_WIRELESS;
	
	_sendCommand(0xE5, timestamp, true);
	_inBootloader = true;

	_flush();
	_flushInput();

	try
	{
		_port->close();
	}
	catch (...)
	{
		return TSS_ERROR_COMMAND_FAILURE;
	}

	yost::sleep_ms(5000);

	try
	{
		_port->open();
	}
	catch (...)
	{
		return TSS_ERROR_CANT_OPEN_PORT;
	}

	return TSS_NO_ERROR;
}

TSS_ERROR TssDevice::_writePage(const char* page_data)
{
	if (!_inBootloader)
		return TSS_ERROR_BOOTLOADER_ONLY;

	U8 buffer[5];
	memset(buffer, 0, 5);
	
	buffer[0] = 'C';
	TSS_ERROR_CHECK(_writeBytes(buffer, 1));	

	std::string page = _intToByteString(strlen(page_data) / 2, 2);
	buffer[0] = page[0];
	buffer[1] = page[1];
	TSS_ERROR_CHECK(_writeBytes(buffer, 2));	

	std::string byte_string;
	int cdc;

	_hexStringToByteString(page_data, &byte_string, &cdc);
	
	for (unsigned i = 0; i < byte_string.size(); i++)
	{
		byte_string[i] = byte_string[i];
	}

	TSS_ERROR_CHECK(_writeBytes((U8*)byte_string.c_str(), byte_string.size()));

	std::string cdc_str = _intToByteString(cdc, 2);
	buffer[0] = cdc_str[0];
	buffer[1] = cdc_str[1];
	TSS_ERROR_CHECK(_writeBytes(buffer, 2));

	if (_sensorType == TSS_BOOTLOADER || _sensorType == TSS_LE)
	{
		yost::sleep_ms(100);
	}

	buffer[0] = 0;
	TSS_ERROR result = _readBytes(buffer, 1);
	if (result == TSS_ERROR_NOT_ENOUGH_DATA)
	{
		// Device did not respond
		return TSS_ERROR_COMMAND_FAILURE;
	}
	
	std::string status((char*)buffer);
	if (_byteStringToInt(status))
	{
		// Device denied data
		return TSS_ERROR_COMMAND_FAILURE;
	}

	return TSS_NO_ERROR;
}

TSS_ERROR TssDevice::_enterFirmware()
{
	if (!_inBootloader)
		return TSS_ERROR_BOOTLOADER_ONLY;

	U8 buffer = 'B';
	TSS_ERROR_CHECK(_writeBytes(&buffer, 1));

	try
	{
		_port->close();		
	}
	catch (...)
	{
		return TSS_ERROR_COMMAND_FAILURE;
	}

	// Devices that have failed components take longer to become ready.
	yost::sleep_ms(10000); // But it feels uncomfortably long.

	try
	{		
		_port->open();
	}
	catch (...)
	{
		return TSS_ERROR_CANT_OPEN_PORT;
	}

	_inBootloader = false;

	_flush();
	_flushInput();

	_setResponseHeader(0);

	_flush();
	_flushInput();

	return TSS_NO_ERROR;
}

TSS_ERROR TssDevice::_getPageSize(U16* page_size)
{
	if (!_inBootloader)
		return TSS_ERROR_BOOTLOADER_ONLY;

	U8 buffer[24];
	memset(buffer, 0, 24);
	buffer[0] = 'I';

	TSS_ERROR_CHECK(_writeBytes(buffer, 1));
	buffer[0] = 0;
	TSS_ERROR_CHECK(_readBytes(buffer, 24));
	
	*page_size = _bytesToU16(buffer[13], buffer[12]);

	return TSS_NO_ERROR;
}

TSS_ERROR TssDevice::_startFirmwareUpdate(const char* address)
{
	if (!_inBootloader)
		return TSS_ERROR_BOOTLOADER_ONLY;
	
	I32 decimal_address = _hexStringToInt(address);
	std::string cmd = _intToByteString(decimal_address, 4);

	U8 buffer[5];
	buffer[0] = 'S';
	buffer[1] = cmd[0];
	buffer[2] = cmd[1];
	buffer[3] = cmd[2];
	buffer[4] = cmd[3];

	TSS_ERROR_CHECK(_writeBytes(buffer, 1));
	yost::sleep_ms(100);
	TSS_ERROR_CHECK(_writeBytes(buffer + 1, 4));

	// This sleep is 100% essential as the device takes some time after recieving the command.
	yost::sleep_ms(5000);
	memset(buffer, 0, 5);
	
	TSS_ERROR result = _readBytes(buffer, 1);
	if (result == TSS_ERROR_NOT_ENOUGH_DATA)
	{
		// Device did not respond
		return TSS_ERROR_COMMAND_FAILURE;
	}

	std::string out((char*)buffer);
	if (_byteStringToInt(out))
	{
		// Device denied command success
		return TSS_ERROR_COMMAND_FAILURE;
	}

	return TSS_NO_ERROR;
}

TSS_ERROR TssDevice::_stopFirmwareUpdate()
{
	if (!_inBootloader)
		return TSS_ERROR_BOOTLOADER_ONLY;

	U8 buffer[1] = { 'F' };
	TSS_ERROR_CHECK(_writeBytes(buffer, 1));

	yost::sleep_ms(100);

	buffer[0] = 0;
	TSS_ERROR result = _readBytes(buffer, 1);
	if (result == TSS_ERROR_NOT_ENOUGH_DATA)
	{
		// Device did not respond
		return TSS_ERROR_COMMAND_FAILURE;
	}

	std::string out((char*)buffer);
	if (_byteStringToInt(out))
	{
		// Device denied command success
		return TSS_ERROR_COMMAND_FAILURE;
	}

	return TSS_NO_ERROR;
}

TSS_ERROR TssDevice::_autoFirmwareUpdateFromFile(const char* file_name)
{
	if (!_inBootloader)
	{
		U32 tm;
		TSS_ERROR_CHECK(_enterBootloader(&tm));
	}

	rapidxml::xml_document<> doc;
	std::ifstream file(file_name);
	if (!file.is_open())
	{
		return TSS_ERROR_COMMAND_FAILURE;
	}

	// Parse File With RapidXML
	std::stringstream buffer;
	buffer << file.rdbuf();
	file.close();
	std::string content(buffer.str());

	try
	{
		doc.parse<0>(&content[0]);
	}
	catch (...)
	{		
		return TSS_ERROR_COMMAND_FAILURE;
	}

	// Values For The Tags We Need
	std::vector<std::string> supported_firmwares;
	std::string firmware_memprogc_value;	
	std::string firmware_setaddr_value;
	std::string firmware_version_value;

	// Get Tag Info We Need
	rapidxml::xml_node<> *root = doc.first_node();
	for (rapidxml::xml_node<> *node = root->first_node(); node; node = node->next_sibling())
	{
		if (node->type() == rapidxml::node_type::node_element)
		{
			std::string name = node->name();
			std::string value = node->value();

			if (name == "FirmwareType")
				supported_firmwares.push_back(value);
			else if (name == "MemProgC")
				firmware_memprogc_value = value;			
			else if (name == "SetAddr")
				firmware_setaddr_value = value;
			else if (name == "MajorVersion")
				firmware_version_value = value;
		}
	}

	// Confirm There Is Data In The MemProgC Tag
	if (firmware_memprogc_value.size() == 0)
	{
		return TSS_ERROR_NOT_ENOUGH_DATA;
	}

	U16 page_size = 0;
	TSS_ERROR_CHECK(_getPageSize(&page_size));

	// Variable To Keep Track of Where We Are In The Update
	unsigned long update_string_index = 0;

	TSS_ERROR_CHECK(_startFirmwareUpdate((char*)firmware_setaddr_value.c_str()));

	while (update_string_index < firmware_memprogc_value.size())
	{
		std::string page = firmware_memprogc_value.substr(update_string_index, (unsigned long)(page_size * 2));
		TSS_ERROR_CHECK(_writePage((char*)page.c_str()));
		update_string_index += page_size * 2;
	}

	TSS_ERROR_CHECK(_stopFirmwareUpdate());

	TSS_ERROR_CHECK(_enterFirmware());

	return TSS_NO_ERROR;
}

TSS_ERROR TssDevice::_autoFirmwareUpdateFromString(const char* update_contents)
{
	if (!_inBootloader)
	{
		U32 tm;
		TSS_ERROR_CHECK(_enterBootloader(&tm));
	}

	rapidxml::xml_document<> doc;

	// Parse File With RapidXML
	std::string content(update_contents);

	try
	{
		doc.parse<0>(&content[0]);
	}
	catch (...)
	{
		return TSS_ERROR_COMMAND_FAILURE;
	}

	// Values For The Tags We Need
	std::vector<std::string> supported_firmwares;
	std::string firmware_memprogc_value;
	std::string firmware_setaddr_value;
	std::string firmware_version_value;

	// Get Tag Info We Need
	rapidxml::xml_node<> *root = doc.first_node();
	for (rapidxml::xml_node<> *node = root->first_node(); node; node = node->next_sibling())
	{
		if (node->type() == rapidxml::node_type::node_element)
		{
			std::string name = node->name();
			std::string value = node->value();

			if (name == "FirmwareType")
				supported_firmwares.push_back(value);
			else if (name == "MemProgC")
				firmware_memprogc_value = value;
			else if (name == "SetAddr")
				firmware_setaddr_value = value;
			else if (name == "MajorVersion")
				firmware_version_value = value;
		}
	}

	// Confirm There Is Data In The MemProgC Tag
	if (firmware_memprogc_value.size() == 0)
	{
		return TSS_ERROR_NOT_ENOUGH_DATA;
	}

	U16 page_size = 0;
	TSS_ERROR_CHECK(_getPageSize(&page_size));

	// Variable To Keep Track of Where We Are In The Update
	unsigned long update_string_index = 0;

	TSS_ERROR_CHECK(_startFirmwareUpdate((char*)firmware_setaddr_value.c_str()));

	while (update_string_index < firmware_memprogc_value.size())
	{
		std::string page = firmware_memprogc_value.substr(update_string_index, (unsigned long)(page_size * 2));
		TSS_ERROR_CHECK(_writePage((char*)page.c_str()));
		update_string_index += page_size * 2;
	}

	TSS_ERROR_CHECK(_stopFirmwareUpdate());

	TSS_ERROR_CHECK(_enterFirmware());

	return TSS_NO_ERROR;
}
