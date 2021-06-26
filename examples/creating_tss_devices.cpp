/********************************************//**
* This example finds and creates the device and returns a TSS_ID which is needed to call many functions in the API
***********************************************/
#include <stdio.h>
#include "threespace_api_export.h"

using namespace std;

int main()
{
	TSS_ComPort port;
	port.port_name = new char[64];
	tss_device_id device_id;
	TSS_ERROR error = TSS_NO_ERROR;


	// Find all Bluetooth devices can be changed to any TSS device 
	printf("====Creating a Three Space Devices from Search====\n");
	tss_findSensorPorts(TSS_BLUETOOTH);

	// Gets the next comport of a device
	// port.port_name Name of the com port used ie: COM1
	// port.device_type Type of 3-Space device.
	// port.connection_type Connection type of the 3-Space device either USB or Bluetooth
	error = tss_getNextSensorPort(port.port_name, &port.device_type, &port.connection_type);

	if (error == TSS_NO_ERROR)
	{
		// Create the 3-Space device on the given comport
		error = tss_createSensor(port.port_name, &device_id);

		if (error)
		{
			printf("Failed to create TSS Sensor on %s!\n", port.port_name);
		}
		else
		{
			// Gather the serial number, hardware verion and firmware version of the device
			U32 serial_number;
			tss_sensor_getSerialNumber(device_id, &serial_number, NULL);
			char hardware_string[32];
			tss_sensor_getHardwareVersionString(device_id, hardware_string, NULL);
			char firmware_string[32];
			tss_sensor_getFirmwareVersionString(device_id, firmware_string, NULL);
			printf("Device Type: %d\nSerial: %08X\nHardware Version: %s\nFirmware Version: %s\n\n", port.device_type, serial_number, hardware_string, firmware_string);

			tss_removeSensor(device_id);
		}
	}
	else
	{
		printf("Failed to get the port!\n");
	}

	printf("====Creating all Three Space Devices from Search====\n");
	// Find all devices known to be 3-Space devices
	tss_findSensorPorts(TSS_FIND_ALL_KNOWN);

	while (error == TSS_NO_ERROR)
	{
		// Gets the next comport of a device
		// port.port_name Name of the com port used ie: COM1
		// port.device_type Type of 3-Space device.
		// port.connection_type Connection type of the 3-Space device either USB or Bluetooth
		error = tss_getNextSensorPort(port.port_name, &port.device_type, &port.connection_type);
		if (error == TSS_NO_ERROR)
		{
			// Create the 3-Space device on the given comport
			error = tss_createSensor(port.port_name, &device_id);

			if (error)
			{
				printf("Failed to create TSS Sensor on %s!\n", port.port_name);
				error = TSS_NO_ERROR; // Reset the error flag
			}
			else
			{
				// Gather the serial number, hardware verion and firmware version of the device
				U32 serial_number;
				tss_sensor_getSerialNumber(device_id, &serial_number, NULL);
				char hardware_string[32];
				tss_sensor_getHardwareVersionString(device_id, hardware_string, NULL);
				char firmware_string[32];
				tss_sensor_getFirmwareVersionString(device_id, firmware_string, NULL);
				printf("Device Type: %d\nSerial: %08X\nHardware Version: %s\nFirmware Version: %s\n\n", port.device_type, serial_number, hardware_string, firmware_string);

				tss_removeSensor(device_id);
			}
		}
		else
		{
			printf("Failed to get the port!\n");
		}
	}

	error = TSS_NO_ERROR; // Reset the error flag

	// This will write bytes to serial ports that are not recognized as 3-Space virtual comports. 
	// If the 3-Space sensors is connected via a serial to usb adapter or a physical serial port this will allow you to get information on what kind of sensor is connected and other useful information
	printf("====Creating all Unknown Devices from Search====\n");
	tss_findSensorPorts(TSS_UNKNOWN);

	while (error == TSS_NO_ERROR)
	{
		// Gets the next comport of a device
		// port.port_name Name of the com port used ie: COM1
		// port.device_type Type of 3-Space device.
		// port.connection_type Connection type of the 3-Space device either USB or Bluetooth
		error = tss_getNextSensorPort(port.port_name, &port.device_type, &port.connection_type);
		if (error == TSS_NO_ERROR)
		{
			// Create the 3-Space device on the given comport
			error = tss_createSensor(port.port_name, &device_id);

			if (error)
			{
				printf("Failed to create TSS Sensor on %s!\n", port.port_name);
				error = TSS_NO_ERROR; // Reset the error flag
			}
			else
			{
				// Gather the serial number, hardware verion and firmware version of the device
				U32 serial_number;
				tss_sensor_getSerialNumber(device_id, &serial_number, NULL);
				char hardware_string[32];
				tss_sensor_getHardwareVersionString(device_id, hardware_string, NULL);
				char firmware_string[32];
				tss_sensor_getFirmwareVersionString(device_id, firmware_string, NULL);
				printf("Device Type: %d\nSerial: %08X\nHardware Version: %s\nFirmware Version: %s\n\n", port.device_type, serial_number, hardware_string, firmware_string);

				tss_removeSensor(device_id);
			}
		}
		else
		{
			printf("Failed to get the port!\n");
		}
	}

	tss_deinitAPI();

	printf("Finished press Enter to continue");
	getchar();
	return 1;
}