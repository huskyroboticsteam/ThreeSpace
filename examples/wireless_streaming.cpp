/********************************************//**
* This is example requires a 3-Space Wireless and Dongle
* Dongle must be plugged into the computer and wireless sensor turned on
* This example streams basic data from a wireless sensor
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

				printf("====Starting Streaming====\n");
				error = tss_dongle_enableAllSensorsAndStartStreaming(dongle_id, TSS_STREAM_TARED_ORIENTATION_AS_QUATERNION, 1000, TSS_STREAM_DURATION_INFINITE);
				if (error)
				{
					printf("TSS_Error: %s\n", tss_error_string[error]);
				}
				else
				{
					TSS_Stream_Packet packet;
					for (int i = 0; i < 10; i++)
					{
						printf("Press Enter to get next packet.\n");
						getchar();
						error = tss_sensor_getLastStreamingPacket(sensor_id, &packet);

						if (error)
						{
							continue;
						}
						else
						{
							printf("Quaternion: (%.03f,%.03f,%.03f,%.03f)\n", packet.taredOrientQuat[0], packet.taredOrientQuat[1], packet.taredOrientQuat[2], packet.taredOrientQuat[3]);
						}
					}

					printf("====Stoping Streaming====\n");
					tss_dongle_stopStreaming(dongle_id);
				}
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