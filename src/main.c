/******************************************************************************
* Basic Example of using lib_i2c on the CH32V003 Microcontroller
*
* Connections:
* 	SDA -> PC1
* 	SCL -> PC2
*
* Demo Version 2.0    03 May 2025 
* See GitHub Repo for more information: 
* https://github.com/
*
* Released under the MIT Licence
* Copyright ADBeta (c) 2024 - 2025
******************************************************************************/
#include <stdio.h>
#include <float.h>

#include "ch32fun.h"
#include "scd41.h"


int main() 
{
	SystemInit();

	// Initialise the SCD41 Device (CO2, Temperature, Humidity Sensor)
	scd41Init(); 
	
	// Fetch the serial number of the SCD41 sensor (thus proving that the i2c connection is working)
	uint64_t serialNumber = 0;
	if (!scd41GetSerialNumber(&serialNumber)) {
		printf("Failed to get SCD41 serial number.\n");
	} else {
		printf("SCD41 Serial Number: %04X-%04X-%04X\n",
			(uint16_t)(serialNumber >> 32), 
			(uint16_t)(serialNumber >> 16), 
			(uint16_t)(serialNumber & 0xFFFF));
	}
	
	// Enable periodic measurement, where the module will product readings every 5 seconds.
	scd41startPeriodicMeasurement();

	while(1) // Loop forever
	{
		// Read the measurement data
		float co2, temperature, humidity;
		if (scd41readMeasurement(&co2, &temperature, &humidity)) {
				printf("Measurement data collected OK.\n"); 
				printf("CO2: "); printFloat(co2);
				printf("ppm,  Temperature: "); printFloat(temperature);
				printf("°C, Humidity: "); printFloat(humidity);
				printf("%%\n");
			}
			else { printf("Failed to read measurement data.\n"); }

		// Sleep until the next sensor reading is available.
		Delay_Ms(5000);
	}
}