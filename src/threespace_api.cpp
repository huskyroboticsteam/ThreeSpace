#include "threespace_api.hpp"
#include "serial_enumerator.hpp"
#include <iostream>
#include <vector>
#include <algorithm>
#include <iostream>
#include <fstream>

using namespace std;

TssAPI gAPI;

#define TSS_READER_THREAD_BUFF_SIZE 4096
#define TSS_READER_DONGLE_PROCESS_LIMIT 4096

 void tssHandleStreamingDataSensor(TssSensor* sensor)
 {
	U8 buff[TSS_READER_THREAD_BUFF_SIZE];
	U32 bytes_to_read = sensor->_available();

	if (bytes_to_read < (size_t)TSS_READER_THREAD_BUFF_SIZE)
	{
		size_t read_data = 0;
		sensor->_readBytes(buff, bytes_to_read, &read_data);

		for (unsigned i = 0; i < read_data; i++)
		{
			sensor->_unparsedStreamData.push_back(buff[i]);
		}
	}
	else
	{
		size_t read_data = 0;
		sensor->_readBytes(buff, TSS_READER_THREAD_BUFF_SIZE, &read_data);

		for (unsigned i = 0; i < read_data; i++)
		{
			sensor->_unparsedStreamData.push_back(buff[i]);
		}
	}

	TSS_Header header;
	U32 read_index = 0;
	U32 current_index = 0;
	U32 last_packet_index = 0;

	while (read_index + sensor->_responseHeaderSize + sensor->_streamDataSize < sensor->_unparsedStreamData.size())
	{
		_tssParseResponseHeader(&sensor->_unparsedStreamData[0] + read_index, sensor->_responseHeaderFlags, &header);

		current_index = read_index + sensor->_responseHeaderSize;

		if (header.DataLength == sensor->_streamDataSize)
		{
			U8 packetChecksum = 0;
			for (int i = 0; i < sensor->_streamDataSize; i++)
			{
				packetChecksum += sensor->_unparsedStreamData[current_index + i];
			}

			if (packetChecksum == header.Checksum)
			{
				gAPI._readerThreadMutex.lock();
				sensor->_streamBuffer.addPacket(&header, &sensor->_unparsedStreamData[0] + current_index, sensor->_streamDataSize);
				gAPI._readerThreadMutex.unlock();

				read_index += sensor->_responseHeaderSize + sensor->_streamDataSize;
				last_packet_index = read_index;
			}
			else
			{
				read_index++;
			}
		}
		else
		{
			read_index++;
		}
	}

	vector<U8>::const_iterator last = sensor->_unparsedStreamData.begin() + last_packet_index;
	sensor->_unparsedStreamData.erase(sensor->_unparsedStreamData.begin(), last);
	
	return;
 }

void tssHandleStreamingDataDongle(TssDongle* dongle)
{
    int max_bytes_to_read = dongle->_available(); //hope this is a complete packet for the time being, read as though it is
    if (max_bytes_to_read > TSS_READER_DONGLE_PROCESS_LIMIT)
        max_bytes_to_read = TSS_READER_DONGLE_PROCESS_LIMIT;

    U8 buff[TSS_READER_THREAD_BUFF_SIZE];
    while (max_bytes_to_read > 0)
    {        
        dongle->_readBytes(buff, dongle->_responseHeaderWirelessSize);
        TSS_Header header;
        _tssParseResponseHeader(buff, dongle->_responseHeaderWirelessFlags, &header);
        max_bytes_to_read -= dongle->_responseHeaderWirelessSize;

        shared_ptr<TssSensor> child;

        //if the logical id is present, we can use that to find the appropriate sensor...
        if (header.LogicalId < TSS_DONGLE_NUM_CHILDREN)
        {
            child = dongle->_children[header.LogicalId];
        }
        else
        {
            //no logical id?  bail
            break;
        }

        if (child)
        {
            dongle->_readBytes(buff, child->_streamDataSize);
			gAPI._readerThreadMutex.lock();
            child->_streamBuffer.addPacket(&header, buff, child->_streamDataSize);
			gAPI._readerThreadMutex.unlock();
            max_bytes_to_read -= child->_streamDataSize;
        }
        else
        {
            //uh...didn't find a child?  bail
            break;
        }

    }
}

void tssHandleStreamingData(TssDevice* device)
{
    if (device->_type == TSS_DEVICE_TYPE_SENSOR)
    {
        tssHandleStreamingDataSensor((TssSensor*)device);
    }
    else
    {
        tssHandleStreamingDataDongle((TssDongle*)device);
    }
}

void tssReaderThread()
{
    for(;;)
    {
		if (gAPI._breakReadThread)
		{
			return;
		}

		gAPI._readerThreadDeviceMutex.lock();
		if (gAPI._readDevices.size() > 0)
		{
				
			WaitForMultiplePorts(gAPI._readPorts);

			if (gAPI._readDevices.size() > 0)
			{			
				for (vector<TssDevice*>::iterator it = gAPI._readDevices.begin(); it != gAPI._readDevices.end(); ++it)
				{
					if(*it == nullptr)
						continue;
					if ((*it)->_inErrorState)
						continue;

					tssHandleStreamingData(*it);
				}
			}			
		}
		gAPI._readerThreadDeviceMutex.unlock();
    }
}

TssAPI::TssAPI()
{
    init();
}

TssAPI::~TssAPI()
{
    deinit();
}

void TssAPI::init()
{
}

void TssAPI::deinit()
{
    if (_readerThread.joinable())
    {        
		_breakReadThread = true;
        _readerThread.join();
    }
}

void TssAPI::registerStreamingDevice(TssDevice* device)
{
	_readerThreadDeviceMutex.lock();
    _readerThreadMutex.lock();

    _readPorts.push_back(device->_port);
    _readDevices.push_back(device);

    if (_readDevices.size() == 1)
    {
        _breakReadThread = false;
        _readerThread = std::thread(tssReaderThread);
    }

	_readerThreadMutex.unlock();
	_readerThreadDeviceMutex.unlock();
}

void TssAPI::unregisterStreamingDevice(TssDevice* device)
{
	_readerThreadDeviceMutex.lock();
    _readerThreadMutex.lock();

    _readPorts.erase(std::remove(_readPorts.begin(), _readPorts.end(), device->_port), _readPorts.end());
    _readDevices.erase(std::remove(_readDevices.begin(), _readDevices.end(), device), _readDevices.end());

    U32 size = _readDevices.size();

	_readerThreadMutex.unlock();
	_readerThreadDeviceMutex.unlock();

    if (size == 0)
    {
        _breakReadThread = true;
		if (_readerThread.joinable())
		{
			_readerThread.join();
		}        
    }
}

#define TSS_VID 0x2476

vector<TssComPort> tssFindSensorPorts(U32 find_flags)
{    
    vector<BasicPortInfo> in_ports = gAPI._serialEnumerator.getPorts(); 

	vector<TssComPort> out_ports;

	for (auto& port : in_ports)
    {
        TSS_TYPE dev_type = TSS_UNKNOWN;
		TSS_CONNECTION_TYPE conn_type = TSS_USB_CT;

        if (port.vendor_id == TSS_VID)
        {
			if (port.port_type == 0)
			{
				conn_type = TSS_USB_CT;
			}
			else if (port.port_type == 1)
			{
				conn_type = TSS_BT_CT;
			}

            if (port.product_id == DNG_PID)
            {
                dev_type = TSS_DONGLE;
            }
            else if (port.product_id == WL_PID)
            {
                dev_type = TSS_WIRELESS;
            }
            else if (port.product_id == USB_PID)
            {
                dev_type = TSS_USB;
            }
            else if (port.product_id == EM_PID)
            {
                dev_type = TSS_EMBEDDED;
            }
            else if (port.product_id == BT_PID)
            {
                dev_type = TSS_BLUETOOTH;
            }
            else if (port.product_id == DL_PID)
            {
                dev_type = TSS_DATALOGGER;
            }
            else if (port.product_id == BTL_PID)
            {
                dev_type = TSS_BOOTLOADER;
            }
			else if (port.product_id == LE_PID)
			{
				dev_type = TSS_LE;
			}
			else if (port.product_id == LX_PID)
			{
				dev_type = TSS_LX;
			}
			else if (port.product_id == MBT_PID)
			{
				dev_type = TSS_MINIBT;
			}
        }
		else if(port.vendor_id == 0x0403)
		{
			if(port.product_id == 0x6015)
			{
				dev_type = TSS_NANO;
			}
		}

        if (dev_type & find_flags)
        {
            TssComPort new_port;
            new_port.port_name = port.port_name;
            new_port.device_type = dev_type;
			new_port.connection_type = conn_type;
            out_ports.push_back(new_port);
        }
    }

    return out_ports;
}
