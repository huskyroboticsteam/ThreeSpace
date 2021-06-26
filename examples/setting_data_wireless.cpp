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
				printf("Setting the tared data of the device to an arbitrary quaternion.\n");
				float in_quat[4];
				in_quat[0] = 0.7071f;
				in_quat[1] = 0.0f;
				in_quat[2] = 0.7071f;
				in_quat[3] = 0.0f;
				error = tss_sensor_tareWithQuaternion(sensor_id, in_quat, NULL);
				//Checking the error returned from the function, typicly succeed as long as the
				//parameters are valid on a plugged in sensor, functions sent to a sensor
				//wirelessly should allways be checked
				if (!error)
				{
					float out_quat[4];
					out_quat[0] = 666;
					//Note this is the only function in ther API that doesnt match the description in the docs due to the 
					//simularity to GetTaredOrientationAsQuaternion causing frequent auto complete mistakes.
					error = tss_sensor_getTareAsQuaternion(sensor_id, out_quat, NULL);
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
				error = tss_sensor_setLEDColor(sensor_id, red, NULL);
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
				error = tss_sensor_setLEDColor(sensor_id, blue, NULL);
				if (!error)
				{
					printf("LED should be be BLUE\n");
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