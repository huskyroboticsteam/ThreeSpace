#pragma once

#include "threespace_sensor.hpp"

#define TSS_DONGLE_NUM_CHILDREN 15
#define TSS_DEFAULT_WIRELESS_COMMAND_RETRIES 5

class TssDongle : public TssDevice
{
public:
	TssDongle(string port);
	TssDongle(const TssDongle& other);

	void operator =(const TssDongle& other);

	//Port control functions
	TSS_ERROR openPort(std::string port);
	TSS_ERROR closePort();

	//Header control functions
	TSS_ERROR enableTimestampsWireless();
	TSS_ERROR disableTimestampsWireless();

	//Wireless sensor functions
	TSS_ERROR getWirelessSensor(U8 logical_id, shared_ptr<TssSensor>& sensor);
	TSS_ERROR removeWirelessSensor(U8 logical_id);

	//Streaming functions
	TSS_ERROR enableAllSensorsAndStartStreaming(U32 data_flags, U32 interval, U32 duration);
	TSS_ERROR startStreaming();
	TSS_ERROR stopStreaming();

	//Wireless Sensor & Dongle Commands
	// 226(0xe2)
	TSS_ERROR softwareReset();
	// 196(0xc4)
	TSS_ERROR setLEDMode(U8 led_mode, U32* timestamp);
	// 197(0xc5)
	TSS_ERROR getLEDMode(U8* led_mode, U32* timestamp);
	// 238(0xee)
	TSS_ERROR setLEDColor(const float* rgb_color3, U32* timestamp);
	// 239(0xef)
	TSS_ERROR getLEDColor(float* rgb_color3, U32* timestamp);
	// 221(0xdd)
	TSS_ERROR setWiredResponseHeaderBitfield(U32 bitfield, U32* timestamp);
	// 222(0xde)
	TSS_ERROR getWiredResponseHeaderBitfield(U32* bitfield, U32* timestamp);
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

	//Dongle Commands
	// 176(0xb0)
	TSS_ERROR setWirelessStreamingAutoFlushMode(U8 mode, U32* timestamp);
	// 177(0xb1)
	TSS_ERROR getWirelessStreamingAutoFlushMode(U8* mode, U32* timestamp);
	// 178(0xb2)
	TSS_ERROR setWirelessStreamingManualFlushBitfield(U16 manual_flush_bitfield, U32* timestamp);
	// 179(0xb3)
	TSS_ERROR getWirelessStreamingManualFlushBitfield(U16* manual_flush_bitfield, U32* timestamp);
	// 182(0xb6)
	TSS_ERROR broadcastSynchronizationPulse(U32* timestamp);
	// 208(0xd0)
	TSS_ERROR getSerialNumberAtLogicalID(U8 logical_id, U32* serial_number, U32* timestamp);
	// 209(0xd1)
	TSS_ERROR setSerialNumberAtLogicalID(U8 logical_id, U32 serial_number, U32* timestamp);
	// 210(0xd2)
	TSS_ERROR getWirelessChannelNoiseLevels(U8* channel_strengths16, U32* timestamp);
	// 211(0xd3)
	TSS_ERROR setWirelessRetries(U8 retries, U32* timestamp);
	// 212(0xd4)
	TSS_ERROR getWirelessRetries(U8* retries, U32* timestamp);
	// 213(0xd5)
	TSS_ERROR getWirelessSlotsOpen(U8* slots_open, U32* timestamp);
	// 214(0xd6)
	TSS_ERROR getSignalStrength(U8* last_packet_signal_strength, U32* timestamp);
	// 219(0xdb)
	TSS_ERROR setWirelessResponseHeaderBitfield(U32 header_bitfield, U32* timestamp);
	// 220(0xdc)
	TSS_ERROR getWirelessResponseHeaderBitfield(U32* header_bitfield, U32* timestamp);

	TSS_ERROR manualWrite(U8* bytes, U16 amount, U32* timestamp);
	TSS_ERROR manualRead(U8* bytes, U16 amount, U16* read_count, U32* timestamp);
	TSS_ERROR manualFlush(U32* timestamp);

	//Wireless HID Commands
	// 215(0xd7)
	TSS_ERROR setWirelessHIDUpdateRate(U8 HID_update_rate, U32* timestamp);
	// 216(0xd8)
	TSS_ERROR getWirelessHIDUpdateRate(U8* HID_update_rate, U32* timestamp);
	// 217(0xd9)
	TSS_ERROR setWirelessHIDAsynchronousMode(U8 HID_communication_mode, U32* timestamp);
	// 218(0xda)
	TSS_ERROR getWirelessHIDAsynchronousMode(U8* HID_communication_mode, U32* timestamp);
	// 240(0xf0)
	TSS_ERROR setJoystickLogicalID(U8 logical_ID, U32* timestamp);
	// 241(0xf1)
	TSS_ERROR setMouseLogicalID(U8 logical_ID, U32* timestamp);
	// 242(0xf2)
	TSS_ERROR getJoystickLogicalID(U8* logical_ID, U32* timestamp);
	// 243(0xf3)
	TSS_ERROR getMouseLogicalID(U8* logical_ID, U32* timestamp);

	TSS_ERROR _setWirelessResponseHeader(U32 flags);	
	U16 _detectStreaming(U32 interval_us);

	vector<shared_ptr<TssSensor>> _children; //always has TSS_DONGLE_NUM_CHILDREN elements, unused elements are null
	U32 _responseHeaderWirelessFlags;
	U8 _responseHeaderWirelessSize;
	U32 _responseHeaderWirelessFlagsPreStream;
	U32 _maxStreamInterval;
	U8 _childCount;
};