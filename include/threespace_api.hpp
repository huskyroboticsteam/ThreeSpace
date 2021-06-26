#pragma once

#include "threespace_device.hpp"
#include "threespace_sensor.hpp"
#include "threespace_dongle.hpp"

#include "yost_fair_mutex.hpp"

// #include <mutex>
#include <thread>

#include "serial_enumerator.hpp"

struct TssComPort
{
	string port_name;                     /**< The system name for the serial port. */
	TSS_TYPE device_type;                 /**< The type of ThreeSpace device connected through the serial port. */
	TSS_CONNECTION_TYPE connection_type;  /**< The type of connection the ThreeSpace device is using. */
};

class TssAPI
{
public:
	TssAPI();
	~TssAPI();
	void init();
	void deinit();
	void registerStreamingDevice(TssDevice* device);
	void unregisterStreamingDevice(TssDevice* device);

	yost::FairMutex _readerThreadDeviceMutex;
	yost::FairMutex _readerThreadPortMutex;
	yost::FairMutex _readerThreadMutex;

	std::thread _readerThread;
	vector<shared_ptr<Serial>> _readPorts;
	vector<TssDevice*> _readDevices;
	bool _breakReadThread;
	SerialEnumerator _serialEnumerator;
};

extern TssAPI gAPI;

vector<TssComPort> tssFindSensorPorts(U32 find_flags = 0xffffffff);
