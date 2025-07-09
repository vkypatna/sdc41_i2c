/******************************************************************************
* This library enables I2C communication with the Sensiron SCD41 (and probably
* SCD40) sensors.  It uses an enhanced version of the CH32FUN LIB_I2C which 
* provides multi-byte commands and data to be passed back & forth.
*
* It is dependent on v4.3 of I2C_LIB and there is some cross-contamination of
* private functions for basic I2C management that are copied into the mapper. 
* Ideally these would be merged into the I2C_LIB.
*
* This library provides functions to init, read and write to the hardware I2C
* Bus - in Default, and Alternative Pinout Modes.
* Default:	SCL = PC2		SDA = PC1
* Alt 1:	SCL = PD1		SDA = PD0
* Alt 2:	SCL = PC5		SDA = PC6
*
*
* See GitHub Repo for more information: 
* https://github.com/HardwareHarry/sdc41_i2c
* Released under the MIT Licence
* Copyright HardwareHarry (c) 2025
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to
* deal in the Software without restriction, including without limitation the 
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or 
* sell copies of the Software, and to permit persons to whom the Software is 
* furnished to do so, subject to the following conditions:
* The above copyright notice and this permission notice shall be included in 
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, 
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF 
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
* DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR 
* OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE 
* USE OR OTHER DEALINGS IN THE SOFTWARE.
******************************************************************************/

#ifndef CH32_LIB_SCD41_H
#define CH32_LIB_SCD41_H

#include <stdint.h>

// #define DEBUG

#ifdef DEBUG
#include <stdio.h>
#endif // DEBUG

// I2C Address for SCD41
#define I2C_ADDR_SDC41                                          0x62

// Basic Commands
#define I2C_SCD41_COMMAND_START_PERIODIC_MEASUREMENT            0x21, 0xb1  // start_periodic_measurement 
#define I2C_SCD41_READ_MEASUREMENT               		        0xec, 0x05  // read_measurement
#define I2C_SCD41_COMMAND_STOP_PERIODIC_MEASUREMENT             0x3f, 0x86  // stop_periodic_measurement
// On-chip output signal compensation
#define I2C_SCD41_WRITE_SET_TEMPERATURE_OFFSET                  0x24, 0x1d  // set_temperature_offset
#define I2C_SCD41_READ_GET_TEMPERATURE_OFFSET                   0x23, 0x18  // get_temperature_offset
#define I2C_SCD41_WRITE_SET_SENSOR_ALTITUDE                     0x24, 0x27  // set_sensor_altitude
#define I2C_SCD41_READ_GET_SENSOR_ALTITUDE                      0x23, 0x22  // get_sensor_altitude
#define I2C_SCD41_WRITE_SET_AMBIENT_PRESSURE                    0xE0, 0x00  // set_ambient_pressure
// Field calibration
#define I2C_SCD41_WRITE_READ_FORCE_RECALIBRATION                0x36, 0x2f  // perform_forced_recalibration
#define I2C_SCD41_WRITE_SET_AUTO_SELF_CALIBRATION_EN            0x24, 0x16  // set_automatic_self_calibration_enabled
#define I2C_SCD41_READ_GET_AUTO_SELF_CALIBRATION_EN             0x23, 0x13  // get_automatic_self_calibration_enabled
// Low power
#define I2C_SCD41_COMMAND_START_LOW_POWER_PERIODIC_MEASUREMENT  0x21, 0xac  // start_low_power_periodic_measurement
#define I2C_SCD41_READ_GET_DATA_READY_STATUS                    0xe4, 0xb8  // get_data_ready_status
// Advanced features
#define I2C_SCD41_COMMAND_PERSIST_SETTINGS                      0x36, 0x15  // persist_settings
#define I2C_SCD41_READ_GET_SERIAL_NUMBER                        0x36, 0x82  // get_serial_number
#define I2C_SCD41_READ_PERFORM_SELF_TEST                        0x36, 0x39  // perform_self_test
#define I2C_SCD41_COMMAND_FACTORY_RESET                         0x36, 0x32  // perform_factory_reset
#define I2C_SCD41_COMMAND_REINITIALIZE                          0x36, 0x46  // reinit
// SCD41 Specific low power single shot measurements (not available on SCD40)
#define I2C_SCD41_COMMAND_MEASURE_SINGLE_SHOT                   0x21, 0x9d  // measure_single_shot
#define I2C_SCD41_COMMAND_MEASURE_SINGLE_SHOT_RHT_ONLY          0x21, 0x96  // measure_single_shot_rht_only

#define CRC8_POLYNOMIAL                                         0x31    // Polynomial used for CRC8 calculation 
#define CRC8_INIT                                               0xFF    // Initial value for CRC8 calculation

#define SCD41_CONVERSION_FACTOR                                 65535.0f / 175.0f // (2^16 - 1) / 175 for Celsius to ticks
#define SCD41_INVERSE_CONVERSION_FACTOR                         175.0f / 65535.0f // 175 / (2^16 - 1) for ticks to Celsius
#define SCD41_DATA_READY_MASK                                   0x0000011111111111 // Mask for data ready status (the lowest 11 bits are relevant)

// Commands (integer return true/false)

/// @brief initialize the i2c bus and send command to device to initialize the SCD41 sensor.
/// @return uint8_t true on success, false otherwise.
uint8_t scd41Init(void);

/// @brief send command to device to start periodic measurement (every 5 seconds).
/// @return uint8_t true on success, false otherwise.
uint8_t scd41startPeriodicMeasurement(void);

/// @brief send command to device to stop periodic measurement.  when idle the device will only measure when single shot
///        measurement is requested, or until periodic measurement is started again.
/// @return uint8_t true on success, false otherwise.
uint8_t scd41stopPeriodicMeasurement(void);

/// @brief send command to device to initialize low power periodic measurement (every 30 seconds).  this uses less power and reduces heater use.
/// @return uint8_t true on success, false otherwise.
uint8_t scd41startLowPowerPeriodicMeasurement(void);

/// @brief send command to device to persist current settings to non-volatile memory.
/// @return uint8_t true on success, false otherwise.
uint8_t scd41persistSettings(void);

/// @brief send command to device to perform a self-test and report the result.
/// @return uint8_t true on success, false otherwise.
uint8_t scd41performSelfTest(void);

/// @brief send command to device to perform a factory reset.
/// @return uint8_t true on success, false otherwise.
uint8_t scd41factoryReset(void);

/// @brief send command to device to reinitialize the sensor and reload settings from non-volatile memory.
/// @return uint8_t true on success, false otherwise.
uint8_t scd41reInit(void);

/// @brief send command to device to measure single shot (CO2, temperature, humidity).
/// @return uint8_t true on success, false otherwise.
uint8_t scd41measureSingleShot(void);

/// @brief send command to device to measure single shot (temperature, humidity only).
/// @return uint8_t true on success, false otherwise.
uint8_t scd41readSingleShotOnlyRHT(void);

// Read data into memory (returns true/false)

/// @brief send command to device to provide current readings (CO2, temperature, humidity) if one is available, NACK if not.
/// @param co2, pointer to float where CO2 concentration will be stored.
/// @param temperature, pointer to float where temperature will be stored.
/// @param humidity, pointer to float where humidity will be stored.
/// @note The values are in the following ranges:
///        - CO2: 0 to 65535 ppm (parts per million)
///        - Temperature: -45.0 to 130.0 degrees Celsius
///        - Humidity: 0 to 100.0% relative humidity
/// @return uint8_t true on success, false otherwise.
uint8_t scd41readMeasurement(float *co2, float *temperature, float *humidity);

/// @brief send command to device to return the current configuration temperature offset.
/// @note The temperature offset is a float value in degrees Celsius that can be used to adjust the temperature reading from the sensor.
/// @param temperatureOffset, pointer to float where the temperature offset will be stored.
/// @return uint8_t true on success, false otherwise.
uint8_t scd41getTemperatureOffset(float *temperatureOffset);

/// @brief send command to device to return the current configuration sensor altitide.
/// @param altitude, pointer to int16_t where the altitude in meters will be stored.
/// @note The altitude is an integer value representing the altitude in meters. The sensor uses this value to adjust the CO2 concentration readings based on the altitude.
///        The altitude can be set to a value between -500 and 5000 meters, where negative values represent below sea level and positive values represent above sea level.
/// @return uint8_t true on success, false otherwise.
uint8_t scd41getSensorAltitude(int16_t *altitude);

/// @brief send command to device to return the its serial number.
/// @note The serial number is a 64-bit unsigned integer that uniquely identifies the sensor.
/// @param serialNumber, pointer to uint64_t where the serial number will be stored.
/// @return uint8_t true on success, false otherwise.
uint8_t scd41GetSerialNumber(uint64_t *serialNumber);

/// @brief Forces a recalibration of the SCD41 sensor.
/// @param co2Concentration, co2 concentration to use during recalibration (pointer to uint16_t).
/// @return i2c_err_t. I2C_OK On Success.
/// @note Important Notes:
/// To successfully conduct an accurate forced recalibration, the following steps need to be carried out:   
///  1. Operate the SCD4x in a periodic measurement mode for > 3 minutes in an environment with homogenous and constant 
///    CO2 concentration.  
///  2. Stop periodic measurement. Wait 500 ms.  
///  3. Subsequently issue the perform_forced_recalibration command and optionally read out the FRC correction (i.e. the magnitude of the correction). 
/// A return value of 0xffff indicates that the forced recalibration failed  
/// Notes: The sensor will fail to perform a forced recalibration if it was not operated before sending the command.
///        Please make sure that the sensor is operated at the voltage desired for the application when applying the forced 
///          recalibration sequence.  
uint8_t scd41performForcedRecalibration(uint16_t *co2Concentration);

/// @brief send command to device to get the current configuration status of the automatic self calibration is enabled.
/// @param enabled, true if automatic self calibration is turned on, false otherwise.
/// @return uint8_t true on success, false otherwise.
uint8_t scd41getAutomaticSelfCalibrationEnabled(uint8_t *enabled);

// Write data (returns true/false)

/// @brief Sets the temperature offset for the SCD41 sensor.
/// @param offsetTemperatureCelsius, Temperature offset in Celsius
/// @return uint8_t true on success, false otherwise.
/// @note Important Notes:
/// 1. Setting the temperature offset requires automatic periodic measurement to be stopped
///    before the command can be sent, or a NACK will be returned. 
/// 2. The temperature offset is stored in the sensor's volatile memory and will be lost when the sensor is powered off
///    unless the persist_settings command is issued after setting the temperature offset.
uint8_t scd41setTemperatureOffset(float offsetTemperatureCelsius);

/// @brief sets the altitude in the current configuration of the SCD41 sensor. 
/// @param altitude, in meters above sea level (negative numbers are possible)
/// @return uint8_t, true on success
/// @note Important Notes:
/// 1. Setting the temperature offset requires automatic periodic measurement to be stopped
///    before the command can be sent, or a NACK will be returned.  Altitude will usually be called once when the device is installed.
/// 2. The temperature offset is stored in the sensor's volatile memory and will be lost when the sensor is powered off
///    unless the persist_settings command is issued after setting the temperature offset.
uint8_t scd41setSensorAltitude(int16_t altitude);

/// @brief sets the ambient pressure in the current configuration of the SCD41 sensor.
/// @param pressure, in pascals.
/// @return uint8_t true on success, false otherwise.
/// @note can be sent during periodic measurements to enable continuous pressure compensation. note that setting an ambient pressure
///   on the sensor using set_ambient_pressure overrides any pressure compensation based on a previously set sensor altitude. 
uint8_t scd41setAmbientPressure(uint16_t pressure);

// Query data (returns actual data)

/// @brief check whether data is available to be read by the readMeasurement (binary, true - data available, false, no data available)
/// @return uint8_t, true if data is available to read, false if not.
uint8_t scd41isDataReady(void);

// Utilities
/// @brief utility to print float values on systems that can't support printf of float variables as standard.
/// @param value, the value to print
/// @return n/a
void printFloat(float value);

#endif // CH32_LIB_SCD41_H

