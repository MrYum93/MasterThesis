#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

/* Include files needed to use VnEzAsyncData. */
#include "vn/sensors/ezasyncdata.h"
#include "vectornav.h"
#include "vn/sensors.h"


int processErrorReceived(char* errorMessage, VnError errorCode);
void asciiAsyncMessageReceived(void *userData, VnUartPacket *packet, size_t runningIndex);

VnEzAsyncData ez;
VnSensor vs;
VnError error;

size_t i = 0;
size_t update_cnt_vec = 0;
char strConversions[50];
char modelNumber[30];
float data;
vec3f ypr;
uint32_t oldHz, newHz;


int imu_init(void){

	char strConversions[50];
	
	const char SENSOR_PORT[] = "/dev/ttyUSB0"; /**/ /* Linux format for virtual (USB) serial port. */
	/*const char SENSOR_PORT[] = "/dev/tty.usbserial-FTXXXXXX"; */ /* Mac OS X format for virtual (USB) serial port. */
	/*const char SENSOR_PORT[] = "/dev/ttyS0"; */ /* CYGWIN format. Usually the Windows COM port number minus 1. This would connect to COM1. */
	const uint32_t SENSOR_BAUDRATE = 115200;
<<<<<<< HEAD
	
	/* We first need to initialize our VnSensor structure. */
	VnSensor_initialize(&vs);
	
	//* Now connect to our sensor. */
	if ((error = VnSensor_connect(&vs, SENSOR_PORT, SENSOR_BAUDRATE)) != E_NONE)
		return processErrorReceived("Error connecting to sensor.", error);

	/* Let's query the sensor's model number. */
	if ((error = VnSensor_readModelNumber(&vs, modelNumber, sizeof(modelNumber))) != E_NONE)
		return processErrorReceived("Error reading model number.", error);
	printf("Model Number: %s\n", modelNumber);

	// /* Get the initial orientation data from the sensor. */
	if ((error = VnSensor_readYawPitchRoll(&vs, &ypr)) != E_NONE)
		return processErrorReceived("Error reading yaw pitch roll.", error);
	str_vec3f(strConversions, ypr);
	printf("Init YPR: %s\n", strConversions);
	
	/* Let's do some simple reconfiguration of the sensor. As it comes from the
	 * factory, the sensor outputs asynchronous data at 40 Hz. For the DS we need 
	 * will change this to 100 Hz for safety purposes. */
	if ((error = VnSensor_readAsyncDataOutputFrequency(&vs, &oldHz)) != E_NONE)
		return processErrorReceived("Error reading async data output frequency.", error);
	printf("Old Async Frequency: %d Hz\n", oldHz);
	if ((error = VnSensor_writeAsyncDataOutputFrequency(&vs, 50, true)) != E_NONE)
		return processErrorReceived("Error writing async data output frequency.", error);
	if ((error = VnSensor_readAsyncDataOutputFrequency(&vs, &newHz)) != E_NONE)
		return processErrorReceived("Error reading async data output frequency.", error);
	printf("New Async Frequency: %d Hz\n", newHz);
			
	/* You will need to define a method which has the appropriate
	* signature for receiving notifications. This is implemented with the
	* method asciiAsyncMessageReceived. Now we register the method with the
	* VnSensor structure. */
	VnSensor_registerAsyncPacketReceivedHandler(&vs, asciiAsyncMessageReceived, NULL);

	/* setup complete now get the async data */
	
	return 0; /*Here 0 means its okay*/
=======
	printf("Did we get into init");
	/* We call the initialize and connect method to connect with our VectorNav sensor. */
	if ((error = VnEzAsyncData_initializeAndConnect(&ez, SENSOR_PORT, SENSOR_BAUDRATE)) != E_NONE)
		return 1;/*processErrorReceived("Error connecting to sensor.", error);*/
	
  return 0; /*Here 0 means its okay*/
>>>>>>> ff11f49c434522dceb063cc1e0a38c3b354efa15
}

float imu_update(void)
{
	update_cnt_vec += 1;

	/* The sched runs at 10.000 Hz if we mod it with 100 the method runs
	* with 100 Hz */
	if((update_cnt_vec % 100) == 0)
	{
		/* nothing really happens here... */
	}

	return data;
	
	// VnCompositeData cd;

	// cd = VnEzAsyncData_currentData(&ez);
	// data = cd.yawPitchRoll.c[0];

	// str_vec3f(strConversions, cd.yawPitchRoll);
	// printf("from vectornav: %s\n", strConversions);
	// /*float fdata = atof(strConversions);*/
	// /*str_vec3f(strConversions, cd.yawPitchRoll);
	// printf("yaw %f", data);
	
	// printf(strConversions[0]);
	// printf("Current YPR: %s\n", strConversions);*/
	// return data;
	
}

void imu_quit(void)
{
	/* unregister the sensor */
	VnSensor_unregisterAsyncPacketReceivedHandler(&vs);
	printf ("app_quit() in vectornav.c called\n");
}

int processErrorReceived(char* errorMessage, VnError errorCode)
{
	char errorCodeStr[100];
	strFromVnError(errorCodeStr, errorCode);
	printf("%s\nERROR: %s\n", errorMessage, errorCodeStr);
	return -1;
}

void asciiAsyncMessageReceived(void *userData, VnUartPacket *packet, size_t runningIndex)
{
	vec3f ypr;
	char strConversions[50];

	/* Silence 'unreferenced formal parameters' warning in Visual Studio. */
	(userData);
	(runningIndex);

	/* Make sure we have an ASCII packet and not a binary packet. */
	if (VnUartPacket_type(packet) != PACKETTYPE_ASCII)
		return;

	/* Make sure we have a VNYPR data packet. */
	if (VnUartPacket_determineAsciiAsyncType(packet) != VNYPR)
		return;

	/* We now need to parse out the yaw, pitch, roll data. */
	VnUartPacket_parseVNYPR(packet, &ypr);

	/* then output the yaw data */
	data = ypr.c[0];
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


*/

