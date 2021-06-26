/********************************************//**
* This example demonstrates getting basic data from the sensor
* This is compatible with all sensors plugged in via USB or Bluetooth
* Will not work with the dongle or wireless sensor wirelessly see
* getting_information_wireless for a wireless example
* For Sensors plugged in with RS232 see creating_class_instances
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

	printf("====Creating a Three Space Device from Search====\n");
	tss_findSensorPorts(TSS_FIND_ALL_KNOWN ^ TSS_DONGLE);

	error = tss_getNextSensorPort(port.port_name, &port.device_type, &port.connection_type);
	if (error == TSS_NO_ERROR)
	{
		error = tss_createSensor(port.port_name, &device_id);

		if (error)
		{
			printf("Failed to create TSS Sensor on %s!\n", port.port_name);
			tss_deinitAPI();
			printf("Finished press Enter to continue");
			getchar();
			return 0;
		}
		else
		{
			printf("==================================================\n");
			printf("Setting the tared data of the device to an arbitrary quaternion.\n");
			float in_quat[4];
			in_quat[0] = 0.7071f;
			in_quat[1] = 0.0f;
			in_quat[2] = 0.7071f;
			in_quat[3] = 0.0f;
			error = tss_sensor_tareWithQuaternion(device_id, in_quat, NULL);
			//Checking the error returned from the function, typicly succeed as long as the
			//parameters are valid on a plugged in sensor, functions sent to a sensor
			//wirelessly should allways be checked
			if (!error)
			{
				float out_quat[4];
				out_quat[0] = 666;
				//Note this is the only function in ther API that doesnt match the description in the docs due to the 
				//simularity to GetTaredOrientationAsQuaternion causing frequent auto complete mistakes.
				error = tss_sensor_getTareAsQuaternion(device_id, out_quat, NULL);
				if (!error)
				{
					printf("TareQuat:%f, %f, %f, %f\n", out_quat[0],
						out_quat[1],
						out_quat[2],
						out_quat[3]);
				}
				else
				{
					printf("TSS_Error: %s\n", tss_error_string[error]);
				}
			}
			else
			{
				printf("TSS_Error: %s\n", tss_error_string[error]);
			}
			printf("==================================================\n");
			printf("Setting the LED color of the device to RED.\n");
			float red[3] = { 1.0f, 0.0f, 0.0f };
			error = tss_sensor_setLEDColor(device_id, red, NULL);
			if (!error)
			{
				printf("LED should be be RED\n");
			}
			else
			{
				printf("TSS_Error: %s\n", tss_error_string[error]);
			}

			printf("Press enter to change LED color to Blue.\n");
			getchar();
			printf("==================================================\n");
			printf("Setting the LED color of the device to BLUE.\n");
			float blue[3] = { 0.0f, 0.0f, 1.0f };
			error = tss_sensor_setLEDColor(device_id, blue, NULL);
			if (!error)
			{
				printf("LED should be be BLUE\n");
			}
			else
			{
				printf("TSS_Error: %s\n", tss_error_string[error]);
			}
			printf("==================================================\n");
			tss_removeSensor(device_id);
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