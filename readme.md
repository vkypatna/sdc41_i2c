# SCD41 Driver for CH32V003 Microcontrollers

A lightweight and robust I²C driver for the Sensirion SCD41 CO2, temperature, and humidity sensor, designed specifically for CH32V003 microcontrollers. This library provides a comprehensive set of functions to interact with the SCD41, including measurement, configuration, calibration, and diagnostic features.

## Features

* **I2C Communication:** Utilizes the CH32V003's hardware I²C bus.
* **Comprehensive Command Set:** Supports all major SCD41 commands, including:
    * Periodic and single-shot measurements (CO2, Temperature, Humidity)
    * Temperature offset and sensor altitude compensation
    * Ambient pressure compensation
    * Forced recalibration (FRC)
    * Automatic Self-Calibration (ASC) control
    * Persisting settings to non-volatile memory
    * Self-test, factory reset, and reinitialization
    * Serial number reading
    * Data ready status check
* **CRC Validation:** Implements Sensirion's CRC8 algorithm for robust data integrity checking on all read operations and for writing data with CRC.
* **Float Printing Workaround:** Includes a utility function (`printFloat`) for systems with without `printf` float support.
* **Low Power Modes:** Supports starting low-power periodic measurements.
* **Small Footprint:** Without debug the compiled example code uses **13.1\%** of the available flash memory, with debug it's **39.3\%** (of the 16Kb flash memory of the CH32V003)

## Supported Hardware & Software

* **Microcontroller:** CH32V003 (tested with specific I²C pinouts, see `lib_i2c` documentation for details).
* **Framework:** [ch32fun](https://github.com/cnlohr/ch32v003fun) with the I2C_LIB addon active.
* **Sensor:** Sensirion SCD41 (CO2, Temperature, Humidity Sensor Module).  Likely also works for SCD40 sensor but without low power mode.
* **I2C Address:** 0x62 (standard for SCD41).
* **Platform.io:** [platform.io](https://platform.io/) Framework for build and deployment.

## Usage

1. Clone this repository into your ch32fun project.  Make sure that the files `funconfig.h`, `scd41.h` and `scd41_i2c.h` files are in the `include`
directory and that the `main.c`, `scd41.c` and `scd41_i2cc.c` are in the `src` directory.
2.  **Ensure Dependencies:** Make sure `ch32fun.h` and `lib_i2c.h` (and their respective `.c` files) are correctly integrated into your build environment.
3.  **I2C Initialization:** Call `scd41Init()` to initialize the I2C bus and the sensor.
4.  **Start Measurement:** Call `startPeriodicMeasurement()` or `startLowPowerPeriodicMeasurement()` to begin data acquisition.
5.  **Read Data:** Periodically call `readMeasurement()` to get the latest CO2, temperature, and humidity readings.
6.  **Connect Sensor:** Connect the SCD41 sensor to the I²C pins of your CH32V003 (default pins C1 & C2).
7.  **Usage:** Use the provided API to initialize and read data from the sensor.

```c
#include "ch32fun.h"
#include "scd41.h"
#include <stdio.h> // For printf (if supported or if you have a wrapper)

// Assume ch32fun.h provides Delay_Ms
// Assume lib_i2c.h provides i2c_init, i2c_device_t, i2c_err_t, etc.

int main(void) {
    float co2, temperature, humidity;
    uint64_t serial_number;

    // 1. Initialize the SCD41
    if (!scd41Init()) {
        printf("SCD41 Initialization Failed!\n");
        while(1); // Halt on error
    }
    printf("SCD41 Initialized.\n");

    // Optional: Get and print serial number
    if (scd41GetSerialNumber(&serial_number)) {
        printf("SCD41 Serial Number: ");
        // You would print the 64-bit serial number using your specific method (e.g., in 3 words)
        // For example: printf("%04X%04X%04X\n", (uint16_t)(serial_number >> 32), (uint16_t)(serial_number >> 16), (uint16_t)serial_number);
    }

    // 2. Start Periodic Measurement
    if (!startPeriodicMeasurement()) {
        printf("Failed to start periodic measurement.\n");
        while(1);
    }
    printf("Periodic measurement started.\n");

    // Main loop for reading data
    while (1) {
        // Check if new data is ready (recommended before reading)
        if (isDataReady()) {
            if (readMeasurement(&co2, &temperature, &humidity)) {
                // readMeasurement already prints values using your printFloat
            } else {
                printf("Failed to read measurement.\n");
            }
        } else {
            // printf("Data not ready yet...\n");
        }

        Delay_Ms(5000); // Wait for 5 seconds (measurements update every 5s in standard mode)
    }

    return 0;
}
```

## License

MIT License

## API References ##

**scd41Init()**

Initialize the I²C bus and the SCD41 devicce and makes them available for further API calls.  Returns true on success or false on failure.

### Triggering Measurement ###
All these functions return true for success or false for failure.

**scd41startPeriodicMeasurement()**

Puts the SCD41 sensor in periodic measurement mode. This means it will automatically refresh its readings every 5 seconds.  This doesn't actually
return data, it trigger the sensor to go collect data.  The scd41getMeasurement function can be called when there is data available to be read.
The function will run for approximately 6 seconds to ensure a scd41getMeasurement can be called immediately and will have data available.

**scd41startLowPowerPeriodicMeasurement()**

Puts the SCD41 sensor in low power periodic measurement mode.  This means it will automatically refresh its readinggs every 30 seconds.  This doesn't actually return data, it trigger the sensor to go collect data.  The scd41getMeasurement function can be called when there is data available to be read.  This mode uses less power than the standard periodic measurement mode and only operates the on-chip heater intermitently.

**scd41stopPeriodicMeasurement()**

Puts the SCD41 sensor into idle mode.  This means measurements will only be performed when a single-shot measurement request is made.  When in idle
mode, the sensor reduces its power usage to a minimum.

**scd41measureSingleShot()**

Requests that the SCD41 sensor performs a one off quick CO2, temperature and humidity measurement.  This takes 1.35 seconds, and then the measurement
data will be retrievable with the scd41getMeasurement function.  This mode is useful where measurements are required infrequently and keeps the sensor
in low power mode until a single shot measurement is requested.

**scd41readSingleShotOnlyRHT()**

Requests that the SCD41 sensor performs a one off quick temperature and humidity measurement.  This is much faster and lower power than a full measurement.  When the measurement is done, the sensor will return to idle mode and the data will be readable using the scd41getMeasurement function.
This is helpful to collect temperature and pressure data only before calibration settings are used.

### Getting Measurement Data ###

**scd41isDataReady()**

This function checks the sensor to see if measurement data is available to be read.  If data is available it will return true, false if not.
If data is not available, then one or more of the following:
1. Periodic Measurement (normal or low power) is disabled.
2. Periodic Measurement is enabled but the last measurement has already been read - you'll need to wait for the next periodic measurement to be
taken (5 seconds in normal mode, 30 seconds in low power mode).
3. The sensor is in idle mode (with periodic measurement disabled), and no single-shot reading has been requested.  You'll need to request the 
sensor performs a measurement before checking again whether data is ready.

**scd41readMeasurement(float \*co2, float \*temperature, float \*humidity)**

This function gets full data from the sensor.  Once data is collected through one of the triggering methods, calling this function will get the results.  It returns true on success or false on failure and the data in the three pointer variables will be populated with the data from the 
sensor.  This includes performing a CRC check on the data and converting it to appropriate units as specified in the sensor manual.

### Sensor Calibration ###
All these functions return true for success or false for failure.

**scd41getTemperatureOffset(float \*temperatureOffset)**

This function gets the current temperature offset used by the sensor to calculate measurement data.  The temperature offset doesn't directly impact
co2 measurements but does impact both humidity and temperature measurements.  Refer to the datasheet for full details.

**scd41setTemperatureOffset(float offsetTemperatureCelsius)** [^1]

This function sets the temperature offset used by the sensor to calculate measurement data. The temperature offset doesn't directly impact
co2 measurements but does impact both humidity and temperature measurements.  Refer to the datasheet for full details.

**scd41getSensorAltitude(int16_t \*altitude)**

This function gets the current altitude setting used by the sensor to calculate measurement data.  This function is intended to be used rarely, when
the device is moved to a new location.  It improves the accuracy of co2 measurements.  Refer to the datasheet for full details.

**scd41setSensorAltitude(int16_t altitude)** [^1]

This function getsetss the current altitude setting used by the sensor to calculate measurement data.  This function is intended to be used rarely, when the device is moved to a new location.  It improves the accuracy of co2 measurements.  Refer to the datasheet for full details.

**scd41getAutomaticSelfCalibrationEnabled(uint8_t \*enabled)**

The SCD41 sensor provides an automatic self calibration function which self-adjusts for long term accuracy.  This function gets the status of the
ASC (Automatic Self Calibration) function; true is enabled, false is disabled (or failure to make the I²C call).  Refer to the datasheet for more
information on ASC, its operation and requirements.

**scd41setAutomaticSelfCalibrationEnabled(uint8_t enabled)** [^1]

This function turns ASC on or off.  If enabled is passed as true, ASC is enabled, if it is passed as false, ASC is disabled.  Returns true on success, false on failure.

**scd41setAmbientPressure(uint16_t pressure)** [^1]

This function sets the ambient pressure (in Pascals) for the location where the sensor is operating.  If not directly set it is calculated by the 
sensor from the altitude data.

**scd41performForcedRecalibration()** [^1]

This function forces a complete self-calibration and stores the settings in the volatile memory.  Use of this command is complex and has some 
requirements for the calibration to work.  Refer to the datasheet or the comments in the code for full details.


### Utility ###
All these functions return true for success or false for failure.

**scd41getSerialNumber(\*serialNumber)**

This will return a 4 word (8 byte) serial number which will uniquely identify the connected sensor.  It is also a useful call to prove that
both the sensor and the bus are working correctly.  It has no requirements in terms of which mode or calibration status the sensor is in.
It returns a uint64_t value, although it's really a 48 bit number.

**scd41persistSettings()**

Changes made during manual or automatic calibration will be written to volatile memory.  This configuration data will be lost when the chip is 
re-initialized or powered off.  This function takes the current configuration and writes it to non-volatile memory, thus making it the default
configuration every time the chip is powered on or initialized.

**scd41performSelfTest()**

This function will instruct the sensor to perform a self-test and will return the result.  This function returns false if the self-test can't be
completed, because of a bus error or that the self-test itself failed.  If it returns true then everything, including the sensor is working OK. 
Note that this function takes 5.5 seconds to complete and the function won't return until it is complete and a result is available.

**scd41factoryReset()**

This function resets the sensor to its factory setting and removes any calibration data stored in the non-volatile memory.  It takes about 1.2
seconds to complete and the function will wait until this is complete before returning true or false (depending on the outcome of the reset).

**scd41reInit**

This function resets the sensor to its normal power-up state.  Calibration is read from non-volatile memory and the sensor will reset to idle mode
and await further commands.

**printFloat(float value)**

Although not required for the operation of the SCD41, as the CH32V003 doesn't support printf of float data types, this very simple function prints
a floating number to 2 digits precision to help printing out the values returned by the sensor readings.

[^1]: Any change in calibration settings are temporary unless the sdc41persistSettings() function is called.

## References

- [Sensirion SCD41 Datasheet](https://sensirion.com/products/catalog/SCD41/)
- [ch32fun Documentation](https://github.com/cnlohr/ch32v003fun)
- [ch32fun I²C Library](https://github.com/ADBeta/CH32V003_lib_i2c)

## TODO
- When the CH32FUN extension I2C_LIB supports multi byte commands, as used by the SCD41 sensor, remove the duplicative functions 
(i2c_init|i2c_start|i2c_stop|...) from the intermediate driver file (scd41_i2c.c) and deprecate the scd41_i2c_read_sequence and
scd41_i2c_write_sequence.
- Test on SCD40 sensor (it should work, minus a couple of the features).
- Test on other WCH CH32x0x microprocessors. 