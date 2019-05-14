#include <stdio.h>
#include <stdlib.h>

/* Include files needed to use VnEzAsyncData. */
#include "ezasyncdata.h"
#include "vectornav.h"
int processErrorReceived(char* errorMessage, VnError errorCode);
VnEzAsyncData ez;
VnError error = E_NONE;
size_t i = 0;
char strConversions[50];
float data;
int imu_init(void){

	char strConversions[50];
	
	const char SENSOR_PORT[] = "/dev/ttyUSB0"; /**/ /* Linux format for virtual (USB) serial port. */
	/*const char SENSOR_PORT[] = "/dev/tty.usbserial-FTXXXXXX"; */ /* Mac OS X format for virtual (USB) serial port. */
	/*const char SENSOR_PORT[] = "/dev/ttyS0"; */ /* CYGWIN format. Usually the Windows COM port number minus 1. This would connect to COM1. */
	const uint32_t SENSOR_BAUDRATE = 115200;
	printf("Did we get into init");
	/* We call the initialize and connect method to connect with our VectorNav sensor. */
	if ((error = VnEzAsyncData_initializeAndConnect(&ez, SENSOR_PORT, SENSOR_BAUDRATE)) != E_NONE)
		return 1;/*processErrorReceived("Error connecting to sensor.", error);*/
	
  return 0; /*Here 0 means its okay*/
}

float imu_update(void)
{
	VnCompositeData cd;

	cd = VnEzAsyncData_currentData(&ez);
	data = cd.yawPitchRoll.c[0];
	/*float fdata = atof(strConversions);*/
	/*str_vec3f(strConversions, cd.yawPitchRoll);
	printf("yaw %f", data);
	
	printf(strConversions[0]);
	printf("Current YPR: %s\n", strConversions);*/
	return data;

}

void imu_quit(void)
{
	printf ("app_quit() in vectornav.c called\n");
}



/*int main(void)
{
	VnEzAsyncData ez;
	VnError error = E_NONE;
	size_t i = 0;
	char strConversions[50];

	/* This example walks through using the VnEzAsyncData structure to easily access
	 * asynchronous data from a VectorNav sensor at a slight performance hit which is
	 * acceptable for many applications, especially simple data logging. */

	/* First determine which COM port your sensor is attached to and update the
	 * constant below. Also, if you have changed your sensor from the factory
	 * default baudrate of 115200, you will need to update the baudrate
	 * constant below as well. */
	/*const char SENSOR_PORT[] = "COM1"; 	 Windows format for physical and virtual (USB) serial port. */
	/*const char SENSOR_PORT[] = "/dev/ttyS1"; */ /* Linux format for physical serial port. 
	const char SENSOR_PORT[] = "/dev/ttyUSB0"; *//**/ /* Linux format for virtual (USB) serial port. */
	/*const char SENSOR_PORT[] = "/dev/tty.usbserial-FTXXXXXX"; */ /* Mac OS X format for virtual (USB) serial port. */
	/*const char SENSOR_PORT[] = "/dev/ttyS0"; */ /* CYGWIN format. Usually the Windows COM port number minus 1. This would connect to COM1. 
	const uint32_t SENSOR_BAUDRATE = 115200;
*/
	/* We call the initialize and connect method to connect with our VectorNav sensor. 
	if ((error = VnEzAsyncData_initializeAndConnect(&ez, SENSOR_PORT, SENSOR_BAUDRATE)) != E_NONE)
		return processErrorReceived("Error connecting to sensor.", error);*/

	/* Now let's display the latest yaw, pitch, roll data at 5 Hz for 5 seconds.
	printf("Displaying yaw, pitch, roll at 5 Hz for 5 seconds.\n");
	for (i = 0; i < 25; i++)
	{
		VnCompositeData cd;

		VnThread_sleepMs(200);

		cd = VnEzAsyncData_currentData(&ez);

		str_vec3f(strConversions, cd.yawPitchRoll);
		printf("Current YPR: %s\n", strConversions);
	}
 */
	/* Most of the asynchronous data handling is done by VnEzAsyncData but there are times
	 * when we wish to configure the sensor directly while still having VnEzAsyncData do
	 * most of the grunt work. This is easily accomplished and we show changing the ASCII
	 * asynchronous data output type here. 
	if ((error = VnSensor_writeAsyncDataOutputType(VnEzAsyncData_sensor(&ez), VNYPR, true)) != E_NONE)
		return processErrorReceived("Error setting async data output type.", error);
*/
	/* We can now display yaw, pitch, roll data from the new ASCII asynchronous data type. 
	printf("Displaying yaw, pitch, roll from new ASCII async type.\n");*/
/*	for (i = 0; i < 2500; i++)
	{
		VnCompositeData cd;

		VnThread_sleepMs(5);

		cd = VnEzAsyncData_currentData(&ez);

		str_vec3f(strConversions, cd.yawPitchRoll);
		printf("Current YPR: %s\n", strConversions);
	}
*/
	/* Now disconnect from the sensor since we are finished. */
/*	if ((error = VnEzAsyncData_disconnectAndUninitialize(&ez)) != E_NONE)
		return processErrorReceived("Error disconnecting from sensor.", error);

	return 0;
}

int processErrorReceived(char* errorMessage, VnError errorCode)
{
	char errorCodeStr[100];
	strFromVnError(errorCodeStr, errorCode);
	printf("%s\nERROR: %s\n", errorMessage, errorCodeStr);
	return -1;
}
*/

