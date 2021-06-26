/********************************************//**
* This is example requires a 3-Space Wireless and Dongle
* Dongle must be plugged into the computer and wireless sensor turned on
* This example gets and sets data on a wireless sensor
* If using bluetooth see getting_information and setting_information
* A wireless sensor must be paired on in Logical id 0 for this example
***********************************************/
#include <stdio.h>
#include "threespace_api_export.h"

using namespace std;

int main()
{
	TSS_ComPort port;
	port.port_name = new char[64];
	tss_device_id dongle_id;
	tss_device_id sensor_id;
	tss_device_id wireless_sensor_id;
	TSS_ERROR error = TSS_NO_ERROR;

	printf("====Creating a Three Space Dongle from Search====\n");
	tss_findSensorPorts(TSS_DONGLE);
	error = tss_getNextSensorPort(port.port_name, &port.device_type, &port.connection_type);
	if (error == TSS_NO_ERROR)
	{
		error = tss_createDongle(port.port_name, &dongle_id);

		if (error != TSS_NO_ERROR)
		{
			printf("Failed to create TSS Dongle on %s!\n", port.port_name);
			tss_deinitAPI();
			printf("Finished press Enter to continue");
			getchar();
			return 0;
		}
	}
	else
	{
		printf("Failed to get the port!\n");
		tss_deinitAPI();
		printf("Finished press Enter to continue");
		getchar();
		return 0;
	}

	printf("====Creating a Three Space Sensor from Search====\n");
	tss_findSensorPorts(TSS_WIRELESS);
	error = tss_getNextSensorPort(port.port_name, &port.device_type, &port.connection_type);

	if (error == TSS_NO_ERROR)
	{
		error = tss_createSensor(port.port_name, &sensor_id);

		if (error != TSS_NO_ERROR)
		{
			printf("Failed to create TSS Sensor on %s!\n", port.port_name);
			tss_deinitAPI();
			printf("Finished press Enter to continue");
			getchar();
			return 0;
		}
	}
	else
	{
		printf("Failed to get the port!\n");
		tss_deinitAPI();
		printf("Finished press Enter to continue");
		getchar();
		return 0;
	}

	// Pairing requires 3 things, the dongle and wireless sensor pan_id, 
	// channel much match and the wireless sensor serial need
	unsigned short pan_id = 1234;
	unsigned char channel = 25; // Must be an integer value between 11 and 26
	unsigned char logical_id = 0;
	unsigned int serial_number;

	printf("Setting the Pan id to: %d, and channel to: %d\n", pan_id, channel);
	// To simplify the example functions are not validated look at setting_data
	error = tss_sensor_getSerialNumber(sensor_id, &serial_number, NULL);
	if (error)
	{
		printf("TSS_Error: %s\nFinished press Enter to continue", tss_error_string[error]);
		getchar();
		return 0;
	}

	error = tss_sensor_setWirelessPanID(sensor_id, pan_id, NULL);
	error = tss_sensor_setWirelessChannel(sensor_id, channel, NULL);

	error = tss_dongle_setWirelessPanID(dongle_id, pan_id, NULL);
	error = tss_dongle_setWirelessChannel(dongle_id, channel, NULL);
	error = tss_dongle_getWirelessSensor(dongle_id, logical_id, &wireless_sensor_id);

	if (error == TSS_NO_ERROR)
	{
		printf("Sensor pair successful!!!\n");
		printf("Commiting settings\n");
		tss_sensor_commitWirelessSettings(sensor_id, NULL);
		tss_dongle_commitWirelessSettings(dongle_id, NULL);
	}
	else
	{
		printf("Failed to pair sensor\n");
	}

	tss_deinitAPI();

	printf("Finished press Enter to continue");
	getchar();
	return 1;
}