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
	TSS_ERROR error = TSS_NO_ERROR;

	printf("====Creating a Three Space Dongle from Search====\n");
	tss_findSensorPorts(TSS_DONGLE);
	error = tss_getNextSensorPort(port.port_name, &port.device_type, &port.connection_type);
	if (error == TSS_NO_ERROR)
	{
		error = tss_createDongle(port.port_name, &dongle_id);

		if (error)
		{
			printf("Failed to create TSS Dongle on %s!\n", port.port_name);
			tss_deinitAPI();
			printf("Finished press Enter to continue");
			getchar();
			return 0;
		}
		else
		{
			error = tss_dongle_getWirelessSensor(dongle_id, 0, &sensor_id);

			if (error)
			{
				printf("Failed to create TSS Sensor on 0!\n");
				tss_deinitAPI();
				printf("Finished press Enter to continue");
				getchar();
				return 0;
			}
			else
			{
				printf("==================================================\n");
				printf("Getting the filtered tared quaternion orientation.(xyzw)\n");
				float quat[4];
				U32 timestamp;
				error = tss_sensor_getTaredOrientationAsQuaternion(sensor_id, quat, &timestamp);

				if (!error)
				{
					printf("Quaternion: %f, %f, %f, %f Timestamp=%u\n", quat[0], quat[1], quat[2], quat[3], timestamp);
				}
				else
				{
					printf("TSS_Error: %s\n", tss_error_string[error]);
				}

				printf("==================================================\n");
				printf("Getting the Corrected Component Sensor Data.\n");
				float gyro[3];
				float accel[3];
				float compass[3];
				error = tss_sensor_getCorrectedSensorData(sensor_id, gyro, accel, compass, NULL);
				if (!error)
				{
					printf("Gyro:  %f, %f, %f\n", gyro[0], gyro[1], gyro[2]);
					printf("Accel: %f, %f, %f\n", accel[0], accel[1], accel[2]);
					printf("Comp:  %f, %f, %f\n", compass[0], compass[1], compass[2]);
				}
				else
				{
					printf("TSS_Error: %s\n", tss_error_string[error]);
				}
				printf("==================================================\n");
				printf("Getting the LED color  of the device.\n");
				float color[3];
				error = tss_sensor_getLEDColor(sensor_id, color, NULL);
				if (!error)
				{
					printf("Color: %f, %f, %f\n", color[0], color[1], color[2]);
				}
				else
				{
					printf("TSS_Error: %s\n", tss_error_string[error]);
				}
				printf("==================================================\n");
				tss_removeSensor(sensor_id);
			}
			
			tss_removeDongle(dongle_id);
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

	tss_deinitAPI();

	printf("Finished press Enter to continue");
	getchar();
	return 1;
}