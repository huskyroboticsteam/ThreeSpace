#pragma once

#include <memory>
#include <string>
#include <vector>
#include "yost_math.hpp"
#include "serial.h"
#include "threespace_api_export.h"
#include "yost_core_api.hpp"

using namespace std;
using namespace serial;

extern TSS_ERROR result;
extern U32 ntries;
#define TSS_ERROR_CHECK(x) result=x;if(result != TSS_NO_ERROR)return result
#define TSS_RETRY(x, tries) ntries=tries;while(ntries>0){result=x;if(result==TSS_NO_ERROR)break;ntries--;}
#define TSS_RETRY_CHECK(x, tries) ntries=tries;while(ntries>0){result=x;if(result==TSS_NO_ERROR)break;yost::sleep_ms(10);ntries--;}if(result != TSS_NO_ERROR)return result

//#define _swap(a,b,c) c=a;a=b;b=c

#define TSS_START_STREAM_RETRIES 200
#define TSS_STOP_STREAM_RETRIES 100

#define TSS_MAX_COMMAND_RETURN_TIME_MS 500

#define TSS_MAX_STREAM_PACKET_RAW_DATA_SIZE 128

//note about this struct: there was a design tradeoff here between memory and speed.
//this is the speed form of this struct, as all data that was in a streaming packet
//is present and ready for use, though memory is sacrificed for this.  The memory
//form would involve only having the rawData array here, and a set of functions to
//pull out desired pieces of data.  I can't say at this point which would be better
//in the long run, so we may want to try it the other way at some point.

#define TSS_STREAM_PACKET_CIRCULAR_BUFFER_SIZE 1024

struct TssStreamPacketCircularBuffer
{
	TSS_Stream_Packet buff[TSS_STREAM_PACKET_CIRCULAR_BUFFER_SIZE];
    U32 start;
    U32 end;
    U32 len;

    void clear();
    void removeFirstPacket();
    void addPacket(TSS_Header* header, U8* data, U32 nbytes);
    void getPacket(TSS_Stream_Packet* packet, U32 index);
};

#define TSS_DEVICE_TYPE_SENSOR 0
#define TSS_DEVICE_TYPE_DONGLE 1

void _tssParseResponseHeader(U8* data, U8 flags, TSS_Header* header);

class TssDevice
{
public:
    TssDevice();
    virtual ~TssDevice();

    bool isConnected();
    TSS_Header* getLastHeader();
    U32 getSerialNumber();
    TSS_TYPE getSensorType();
	string getHardwareVersionString();
	string getFirmwareVersionString();

    //Port functions
    TSS_ERROR _openPort(std::string name);
    TSS_ERROR _writeBytes(U8* data, U32 n_bytes, size_t* write_count = NULL);
    TSS_ERROR _readBytes(U8* data, U32 n_bytes, size_t* read_count = NULL);
	TSS_ERROR _portReset(U8* port_open);
    void _purgeInput();
	void _flushInput();
	void _flush();
	size_t _available();

    //Response header functions
    TSS_ERROR _readResponseHeader();
    TSS_ERROR _setResponseHeader(U32 flags);

    //Command sending support functions
    TSS_ERROR _sendCommandBytes(const U8* data, U8 n_bytes, U32* timestamp, bool ignore_response = false);
    TSS_ERROR _sendCommand(U8 command, U32* timestamp, bool ignore_response = false);
    TSS_ERROR _readFloats(float* data, U8 n_floats);
    TSS_ERROR _parseFloats(float* data, U8 n_floats);
    TSS_ERROR _checkedCommandWriteRead(U8 command, U8* data, U8 n_bytes, U32* timestamp);
    TSS_ERROR _checkedCommandWriteReadFloats(U8 command, float* data, U8 n_floats, U32* timestamp);

    //Lower level command functions
    TSS_ERROR _readSerialNumber(U32* timestamp);
    TSS_ERROR _readVersionString(U32* timestamp);
	TSS_ERROR _readFirmwareString(U32* timestamp);
	TSS_ERROR _softwareReset();

	//Bootloader functions
	TSS_ERROR _bootloaderCheck(U8* in_bootloader);
	TSS_ERROR _enterBootloader(U32* timestamp);
	TSS_ERROR _writePage(const char* page_data);
	TSS_ERROR _enterFirmware();
	TSS_ERROR _getPageSize(U16* page_size);
	TSS_ERROR _startFirmwareUpdate(const char* address);
	TSS_ERROR _stopFirmwareUpdate();
	TSS_ERROR _autoFirmwareUpdateFromFile(const char* file_name);
	TSS_ERROR _autoFirmwareUpdateFromString(const char* update_contents);

	static U32 _hexCharToInt(U8 c);
	static U16 _bytesToU16(U8 byte1, U8 byte2);
	static void _U16ToBytes(U16 val, U8* byte1, U8* byte2);
	static I32 _byteStringToInt(std::string info);
	static std::string _intToByteString(I32 value, int override_size = -1);
	static I32 _hexStringToInt(std::string hexstr);
	static void _hexStringToByteString(std::string hex_string, std::string* rtn_info, I32* rtn_cdc);
	
    U8 _type;
    bool _isportOwner;
    bool _isWireless;
	bool _inErrorState = false;
	bool _inBootloader = false;
    U32 _responseHeaderFlags;
    U8 _responseHeaderSize;
    string _portName;
    shared_ptr<Serial> _port;
    U8 _logicalId;
    TSS_Header _lastHeader;
    U32 _serialNumber;
    std::string _versionString;
	std::string _firmwareString;
	std::string _hardwareString;
    TSS_TYPE _sensorType;
    U8 _commandRetries;
    bool _isStreaming;
};


#define BASIC_CALL_CHECK_TSS() if (!isConnected()){return TSS_ERROR_NOT_CONNECTED;}if (_isStreaming){return TSS_ERROR_COMMAND_CALLED_DURING_STREAMING;}if(_inErrorState){return TSS_ERROR_NOT_CONNECTED;}if(_inBootloader){return TSS_ERROR_BOOTLOADER_MODE;}
