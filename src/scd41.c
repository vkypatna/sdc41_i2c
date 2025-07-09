#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>

#include "ch32fun.h"
#include "scd41_i2c.h"
#include "scd41.h"

i2c_err_t i2c_stat;
i2c_device_t dev = {
    .type = I2C_ADDR_7BIT,
    .addr = I2C_ADDR_SDC41
};

// I2C Scan Callback example function. Prints the address which responded
static inline void i2c_scan_callback(const uint8_t addr)
{
	printf("Address: 0x%02X Responded.\n", addr);
}

// Code taken directly from Sensirion's SCD4x datasheet to calculate CRC bytes.
uint8_t sensirion_common_generate_crc(const uint8_t* data, uint16_t count) { 
    uint16_t current_byte; 
    uint8_t crc = CRC8_INIT; 
    uint8_t crc_bit; 
    /* calculates 8-Bit checksum with given polynomial */ 
    for (current_byte = 0; current_byte < count; ++current_byte) { 
        crc ^= (data[current_byte]);
        for (crc_bit = 8; crc_bit > 0; --crc_bit) {
            if (crc & 0x80)
                crc = (crc << 1) ^ CRC8_POLYNOMIAL; 
            else 
                crc = (crc << 1);
        }
    } 
    return crc; 
} 

void printFloat(float value)
{
    // This function works around the limitation of printf not supporting float formatting directly.
    int32_t int_part = (int32_t)value;
    int32_t dec_part = (int32_t)((value - int_part) * 100.0f); // Get two decimal places
    if (dec_part < 0) dec_part = -dec_part; // Handle negative numbers correctly for decimal part
    printf("%ld.%02ld", int_part, dec_part);
}

uint8_t scd41GetSerialNumber(uint64_t *serialNumber)
{
    // Command: Read the Serial Number Register (Reg 0x3682, returns three words)
    uint8_t writeData[] = {I2C_SCD41_READ_GET_SERIAL_NUMBER};
    uint8_t readData[9] = {0}; // Buffer to hold the serial number data

    // Call the I2C function to send the command and read the serial number data
    // The SCD41 returns 9 bytes of data, which includes the serial number and CRC bytes.
    i2c_stat = scd41_i2c_read_sequence(&dev, &writeData[0], sizeof(writeData), &readData[0], sizeof(readData), 1);
    if (i2c_stat != I2C_OK) {
#ifdef DEBUG
        printf("Error %u using the I2C Bus\n", i2c_stat);
#endif // DEBUG
        return false; // Return false if reading the serial number failed
    }

    // Verify the CRC of the temperature offset data
    uint8_t crc1 = sensirion_common_generate_crc(&readData[0], 2);
    uint8_t crc2 = sensirion_common_generate_crc(&readData[3], 2);
    uint8_t crc3 = sensirion_common_generate_crc(&readData[6], 2);
    if ((readData[2] != crc1 ) || (readData[5] != crc2 ) || (readData[8] != crc3 )) {
#ifdef DEBUG
        printf("CRC check failed for serial number data.\n");
        printf("Word 1 - read CRC: %u, calculated CRC: %u\n", readData[2], crc1);
        printf("Word 1 - read CRC: %u, calculated CRC: %u\n", readData[5], crc2);
        printf("Word 1 - read CRC: %u, calculated CRC: %u\n", readData[8], crc3);
#endif // DEBUG
        return false; // Return false if CRC check fails
    } else { 
#ifdef DEBUG
        printf("CRC check passed for serial number data.\n"); 
#endif // DEBUG
    }

    // Extract the serial number from the data
    // The serial number is in the the three words of the data array byte shifted.
    uint16_t word1 = ((uint16_t)readData[0] << 8) | readData[1]; // First two bytes
    uint16_t word2 = ((uint16_t)readData[3] << 8) | readData[4]; // Next two bytes
    uint16_t word3 = ((uint16_t)readData[6] << 8) | readData[7]; // Last two bytes

    *serialNumber = 0ULL; // Initialize the serial number to 0
    *serialNumber = ((uint64_t)word1 << 32) |
                     ((uint64_t)word2 << 16) |
                     ((uint64_t)word3);
#ifdef DEBUG
    // Print it as three consecutive words as printf can't handle 64 bit integers directly
    printf("SCD41 Serial Number: 0x%04X-%04X-%04X\n", word1, word2, word3);
#endif // DEBUG`
    return true; // Return true for success
}

uint8_t scd41startPeriodicMeasurement(void)
{
    // Start Periodic Measurement Command (Reg 0x21B1, no data)
    uint8_t writeData[] = {I2C_SCD41_COMMAND_START_PERIODIC_MEASUREMENT};
    i2c_stat = scd41_i2c_write_sequence(&dev,  &writeData[0], sizeof(writeData));
    if (i2c_stat != I2C_OK) {
#ifdef DEBUG
        printf("Error %u starting periodic measurement\n", i2c_stat);
#endif // DEBUG
        return false; // Return false if starting the measurement failed
    }
#ifdef DEBUG
    printf("Periodic measurement started successfully.\n");
#endif
    Delay_Ms(6000); // Wait for the first measurement to complete so it can be queried.
    return true; // Return true if successful
}

uint8_t scd41stopPeriodicMeasurement(void)
{
    // Stop Periodic Measurement Command (Reg 0x3F86, no data)
    uint8_t writeData[] = {I2C_SCD41_COMMAND_STOP_PERIODIC_MEASUREMENT};
    i2c_stat = scd41_i2c_write_sequence(&dev, &writeData[0], sizeof(writeData));
    if (i2c_stat != I2C_OK) {
#ifdef DEBUG
        printf("Error %u stopping periodic measurement\n", i2c_stat);
#endif // DEBUG
        return false; // Return false if stopping the measurement failed
    }
    // Can take up to 500ms to complete
    Delay_Ms(500); // Wait for the command to complete

#ifdef DEBUG
    printf("Periodic measurement stopped successfully.\n");
#endif
    return true; // Return true if successful
}

uint8_t scd41readMeasurement(float *co2, float *temperature, float *humidity)
{
    printf("Starting read measurement...\n");
    // Read Measurement Command (Reg 0xEC05, 9 bytes of data received)
    const uint8_t writeBuffer[] = {I2C_SCD41_READ_MEASUREMENT};
    uint8_t measurementData[9] = {0};
    i2c_stat = scd41_i2c_read_sequence(&dev, &writeBuffer[0], sizeof(writeBuffer), &measurementData[0], sizeof(measurementData), 1);
    if (i2c_stat != I2C_OK) {
#ifdef DEBUG
        printf("Error %u starting read measurement\n", i2c_stat);
#endif // DEBUG
        return false; // Return false if starting the read failed
    }

    // CO2 - request measurement data, check the CRC, calculate the CO2 value and prepare it for output
    uint16_t co2_raw = (measurementData[0] << 8) | measurementData[1];
    uint8_t co2_crc = measurementData[2];
    if (sensirion_common_generate_crc(&measurementData[0], 2) != co2_crc) {
#ifdef DEBUG
        printf("SCD41 Error: CO2 CRC mismatch! Received 0x%02X, Calculated 0x%02X\n", co2_crc, sensirion_common_generate_crc(&measurementData[0], 2));
#endif // DEBUG
        return false;
    } 
    *co2 = co2_raw;
    
    // Temperature request measurement data, check the CRC, calculate the temperature value and prepare it for output
    uint16_t temp_raw = (measurementData[3] << 8) | measurementData[4];
    uint8_t temp_crc = measurementData[5];
    if (sensirion_common_generate_crc(&measurementData[3], 2) != temp_crc) {
#ifdef DEBUG
        printf("SCD41 Error: Temperature CRC mismatch! Received 0x%02X, Calculated 0x%02X\n", temp_crc, sensirion_common_generate_crc(&measurementData[3], 2));
#endif // DEBUG
        return false;
    }
    *temperature = -45.0f + 175.0f * ((float)temp_raw / 65536.0f);
    
    // Humidity request measurement data, check the CRC, calculate the humidity value and prepare it for output
    uint16_t hum_raw = (measurementData[6] << 8) | measurementData[7];
    uint8_t hum_crc = measurementData[8];
    if (sensirion_common_generate_crc(&measurementData[6], 2) != hum_crc) {
#ifdef DEBUG
        printf("SCD41 Error: Humidity CRC mismatch! Received 0x%02X, Calculated 0x%02X\n", hum_crc, sensirion_common_generate_crc(&measurementData[6], 2));
    #endif // DEBUG
        return false;
    }
    *humidity = 100.0f * (float)hum_raw / 65536.0f;
    
#ifdef DEBUG
    printf("CO2: ");
    printFloat(*co2);
    printf(" ppm, Temperature: ");
    printFloat(*temperature);
    printf(" °C, Humidity: ");
    printFloat(*humidity);
    printf("%%\n");
#endif // DEBUG

    return true; // Return true if successful
}

uint8_t scd41Init(void)
{
    // Initialise the I2C Interface on the selected pins, at the specified Hz.
	// Enter a clock speed in Hz (Weirdness happens below 10,000), or use one
	// of the pre-defined clock speeds:
	// I2C_CLK_10KHZ    I2C_CLK_50KHZ    I2C_CLK_100KHZ    I2C_CLK_400KHZ
	// I2C_CLK_500KHZ   I2C_CLK_600KHZ   I2C_CLK_750KHZ    I2C_CLK_1MHZ
    i2c_stat = i2c_init(I2C_CLK_400KHZ);
    if (i2c_stat != I2C_OK) {
#ifdef DEBUG
        printf("Failed to init the I2C Bus\n");
#endif // DEBUG
        return false; // Return false if initialization failed
    } else {
        // Initialising I2C causes the pins to transition from LOW to HIGH.
        // Wait 100ms to allow the I2C Device to timeout and ignore the transition.
        // Otherwise, an extra 1-bit will be added to the next transmission
        Delay_Ms(100);

        // Optional i2c scan to find devices on the bus.  The scan should show the SCD41 device at address 0x62.    
        // i2c_scan(i2c_scan_callback);
    }
    
    // A non-distructive read of the serial number could be performed to ensure the device is present and functioning.
    // uint64_t serialNumber = 0;
    // if (!scd41GetSerialNumber(&serialNumber)) {
#ifdef DEBUG
        // printf("Failed to get SCD41 serial number.\n");
#endif // DEBUG
    //     printf("Failed to get SCD41 serial number.\n");
    //     return false; // Return false if getting the serial number failed
    // }

    return true; // Initialization successful
}

uint8_t scd41setTemperatureOffset(float offsetTemperatureCelsius) {
    // Set Temperature Offset Command (Reg 0x241D, two command bytes, two data bytes + CRC)
    // Important Notes: 
    // 1. Setting the temperature offset requires automatic periodic measurement to be stopped 
    //    before the command can be sent, or a NACK will be returned.
    // 2. The temperature offset is stored in the sensors volatile memory and will be lost when the sensor is powered off 
    //    unless the persist_settings command is issued after setting the temperature offset.
    
    // Calculate the "ticks" value for the temperature offset.
    uint16_t offset_raw = (uint16_t)roundf(offsetTemperatureCelsius * SCD41_CONVERSION_FACTOR);

    // Prepare the data buffer with the command and the offset value
    uint8_t wData[5] = {I2C_SCD41_WRITE_SET_TEMPERATURE_OFFSET, (offset_raw >> 8) & 0xFF, offset_raw & 0xFF, 0};
    // Calculate CRC for the data
    wData[4] = sensirion_common_generate_crc(&wData[2], 2);
    
    i2c_stat = scd41_i2c_write_sequence(&dev, &wData[0], sizeof(wData));
    if (i2c_stat != I2C_OK) {
#ifdef DEBUG
        printf("Error %u setting temperature offset\n", i2c_stat);
#endif // DEBUG
        return false; // Return false if setting the offset failed
    }
    
#ifdef DEBUG
    printf("Temperature offset set to ");
    printFloat(offsetTemperatureCelsius);
    printf(".\n");
#endif // DEBUG
    return true; // Return true if successful
}

uint8_t scd41getTemperatureOffset(float *temperatureOffset) {
    // Get Temperature Offset Command (Reg 0x2318, no data)
    uint8_t wData[2] = {I2C_SCD41_READ_GET_TEMPERATURE_OFFSET};
    uint8_t rData[3] = {0};
    i2c_stat = scd41_i2c_read_sequence(&dev, wData, sizeof(wData), rData, sizeof(rData), 1);
    if (i2c_stat != I2C_OK) {
#ifdef DEBUG
        printf("Error %u starting get temperature offset\n", i2c_stat);
#endif // DEBUG
        return false; // Return false if starting the read failed
    }

    // Verify the CRC of the temperature offset data
    uint8_t crc = sensirion_common_generate_crc(&rData[0], 2);
    if (rData[2] != crc) {
#ifdef DEBUG
        printf("CRC check failed for temperature offset data.\n");
        printf("Read CRC: %u, calculated CRC: %u\n", rData[2], crc);
#endif // DEBUG
        return false; // Return false if CRC check fails
    }

    // Convert the raw data to int16_t "ticks" into a float Celcius value.
    uint16_t tempBuffer = ((rData[0] << 8) | rData[1]);
    *temperatureOffset = tempBuffer * SCD41_INVERSE_CONVERSION_FACTOR; // Convert ticks to Celsius
#ifdef DEBUG
    printf("Temperature Offset:");
    printFloat( *temperatureOffset);
    printf("\n");
#endif // DEBUG
    return true; // Return true if successful
}

uint8_t scd41setSensorAltitude(int16_t altitude) {
    // Set Sensor Altitude Command (Reg 0x2427, two bytes + CRC)
    // Prepare the data buffer with the command and the offset value
    uint8_t wData[5] = {I2C_SCD41_WRITE_SET_SENSOR_ALTITUDE, (altitude >> 8) & 0xFF, altitude & 0xFF, 0};
    // Calculate CRC for the data
    wData[4] = sensirion_common_generate_crc(&wData[2], 2);

    i2c_stat = scd41_i2c_write_sequence(&dev, &wData[0], sizeof(wData));
    if (i2c_stat != I2C_OK) {
#ifdef DEBUG
        printf("Error %u setting sensor altitude\n", i2c_stat);
#endif // DEBUG
        return false; // Return false if setting the altitude failed
    }

#ifdef DEBUG
    printf("Sensor altitude set to %d meters.\n", altitude);
#endif // DEBUG
    return true; // Return true if successful
}

uint8_t scd41getSensorAltitude(int16_t *altitude) {
    // Get Sensor Altitude Command (Reg 0x2322, no data)
    // Prepare the data buffer with the command and the offset value
    uint8_t wData[2] = {I2C_SCD41_READ_GET_SENSOR_ALTITUDE};
    // Prepare the read data buffer to receive the altitude data
    uint8_t rData[3] = {0};
    i2c_stat = scd41_i2c_read_sequence(&dev, &wData[0], sizeof(wData), &rData[0], sizeof(rData), 1);
    if (i2c_stat != I2C_OK) {
#ifdef DEBUG
        printf("Error %u requesting get sensor altitude\n", i2c_stat);
#endif
        return false; // Return false if starting the read failed
    }

    // Verify the CRC of the sensor altitude data
    uint8_t crc = sensirion_common_generate_crc(&rData[0], 2);
    if (rData[2] != crc) {
#ifdef DEBUG
        printf("CRC check failed for sensor altitude data.\n");
        printf("Read CRC: %u, calculated CRC: %u\n", rData[2], crc);
#endif // DEBUG
        return false; // Return false if CRC check fails
    }

    // Convert the raw data to int32_t value
    *altitude = ((rData[0] << 8) | rData[1]);
#ifdef DEBUG
    printf("Sensor Altitude: %u meters\n", *altitude);
#endif // DEBUG
    return true; // Return true if successful
}

uint8_t scd41setAmbientPressure(uint16_t pressure) {
    // Set Ambient Pressure Command (Reg 0xE000, 2 bytes command, 2 bytes data + CRC)
    // Prepare the data buffer with the command and the pressure value
    uint8_t wData[5] = {I2C_SCD41_WRITE_SET_AMBIENT_PRESSURE, (pressure >> 8) & 0xFF, pressure & 0xFF, 0};
    // Calculate CRC for the data
    wData[4] = sensirion_common_generate_crc(&wData[2], 2);

    i2c_stat = scd41_i2c_write_sequence(&dev, &wData[0], sizeof(wData));
    if (i2c_stat != I2C_OK) {
#ifdef DEBUG
        printf("Error %u setting ambient pressure\n", i2c_stat);
#endif // DEBUG
        return false; // Return false if setting the pressure failed
    }

#ifdef DEBUG
    printf("Ambient pressure set to %u Pa.\n", pressure);
#endif
    return true; // Return true if successful
}

uint8_t scd41performForcedRecalibration(uint16_t *co2Concentration) {
    // To successfully conduct an accurate forced recalibration, the following steps need to be carried out:   
    //  1. Operate the SCD4x in a periodic measurement mode for > 3 minutes in an environment with homogenous and constant 
    //    CO2 concentration.  
    //  2. Stop periodic measurement. Wait 500 ms.  
    //  3. Subsequently issue the perform_forced_recalibration command and optionally read out the FRC correction (i.e. the magnitude of the correction). 
    // A return value of 0xffff indicates that the forced recalibration failed  
    // Notes: The sensor will fail to perform a forced recalibration if it was not operated before sending the command.
    //        Please make sure that the sensor is operated at the voltage desired for the application when applying the forced 
    //          recalibration sequence.  

    // Perform Forced Recalibration Command (Reg 0x362F, 2 bytes + CRC)
    // Prepare the data buffer with the command and the pressure value
    uint8_t wData[5] = {I2C_SCD41_WRITE_READ_FORCE_RECALIBRATION, (*co2Concentration >> 8) & 0xFF, *co2Concentration & 0xFF, 0};
    wData[4] = sensirion_common_generate_crc(&wData[2], 2);

    // Prepare the data buffer to recceive the FRC correction data
    uint8_t rData[3] = {0};

    i2c_stat = scd41_i2c_read_sequence(&dev, &wData[0], sizeof(wData), &rData[0], sizeof(rData), 400);
    if (i2c_stat != I2C_OK) {
#ifdef DEBUG
        printf("Error %u performing forced recalibration\n", i2c_stat);
#endif // DEBUG

        return false; // Return false if performing the recalibration failed
    }

    // Verify the CRC of the FRC correction data
    uint8_t crc = sensirion_common_generate_crc(&rData[0], 2);
    if (rData[2] != crc) {
#ifdef DEBUG
        printf("CRC check failed for FRC correction data.\n");
        printf("Read CRC: %u, calculated CRC: %u\n", rData[2], crc);
#endif
        return false; // Return false if CRC check fails
    }

    // Check if the FRC calibration request worked OK
    if (rData[0] == 0xFF && rData[1] == 0xFF) {
#ifdef DEBUG
        printf("Forced recalibration failed.\n");
#endif // DEBUG
        return false; // Return false if the recalibration failed
    }

    // The forced recalibration was successful, return the FRC correction value
    *co2Concentration = (rData[0] << 8) | rData[1]; // Combine high and low bytes
#ifdef DEBUG
    printf("Forced recalibration performed with CO2 concentration: %u ppm.\n", *co2Concentration);
#endif // DEBUG
    return true; // Return true if successful
}

uint8_t scd41setAutomaticSelfCalibrationEnabled(uint8_t enable) {
    // Set Automatic Self Calibration (ASC) Enabled Command (Reg 0x2416, 2 bytes + CRC)
    // Prepare the data buffer with the command and the enable value
    uint8_t wData[5] = {I2C_SCD41_WRITE_SET_AUTO_SELF_CALIBRATION_EN, 0x00, enable ? 1 : 0, 0};
    wData[4] = sensirion_common_generate_crc(&wData[2], 2);
    
    i2c_stat = scd41_i2c_write_sequence(&dev, &wData[0], sizeof(wData));
    if (i2c_stat != I2C_OK) {
#ifdef DEBUG
        printf("Error %u setting automatic self calibration\n", i2c_stat);
#endif // DEBUG
        return false; // Return false if setting ASC failed
    }

#ifdef DEBUG
    printf("Automatic Self Calibration %s.\n", enable ? "enabled" : "disabled");
#endif // DEBUG
    return true; // Return true if successful
}

uint8_t scd41getAutomaticSelfCalibrationEnabled(uint8_t *enabled) {
    // Get Automatic Self Calibration (ASC) Enabled Command (Reg 0x2313, no data)
    // Prepare the data buffer with the command
    uint8_t wData[2] = {I2C_SCD41_READ_GET_AUTO_SELF_CALIBRATION_EN};
    // Prepare the data buffer to receive the ASC enabled data
    uint8_t rData[3] = {0};
    i2c_stat = scd41_i2c_read_sequence(&dev, &wData[0], sizeof(wData), &rData[0], sizeof(rData), 1);
    if (i2c_stat != I2C_OK) {
#ifdef DEBUG
        printf("Error %u starting get automatic self calibration\n", i2c_stat);
#endif // DEBUG
        return false; // Return false if starting the read failed
    }

    // Verify the CRC of the ASC enabled data
    uint8_t crc = sensirion_common_generate_crc(&rData[0], 2);
    if (rData[2] != crc) {
#ifdef DEBUG
        printf("CRC check failed for get automatic self calibration data.\n");
        printf("Read CRC: %u, calculated CRC: %u\n", rData[2], crc);
#endif
        return false; // Return false if CRC check fails
    }

    // Convert the raw data to uint8_t value
    *enabled = rData[1];
#ifdef DEBUG
    printf("Automatic Self Calibration is %s.\n", *enabled ? "enabled" : "disabled");
#endif // DEBUG
    return true; // Return true if successful
}

uint8_t scd41startLowPowerPeriodicMeasurement(void) {
    // Start Low Power Periodic Measurement Command (Reg 0x21AC, no data)
    // Prepare the data buffer with the command
    uint8_t wData[2] = {I2C_SCD41_COMMAND_START_LOW_POWER_PERIODIC_MEASUREMENT};
    i2c_stat = scd41_i2c_write_sequence(&dev, &wData[0], sizeof(wData));
    if (i2c_stat != I2C_OK) {
#ifdef DEBUG
        printf("Error %u starting low power periodic measurement\n", i2c_stat);
#endif // DEBUG
        return false; // Return false if starting the low power measurement failed
    }
#ifdef DEBUG
    printf("Low power periodic measurement started successfully.\n");
    #endif // DEBUG
    return true; // Return true if successful
}

uint8_t scd41isDataReady(void)
{
    // Get data ready status (Reg 0xe4b8, no data)
    // Prepare the data buffer with the command
    uint8_t wData[2] = {I2C_SCD41_READ_GET_DATA_READY_STATUS};
    // Prepare the data buffer to receive the data ready status
    uint8_t rData[3] = {0};

    i2c_stat = scd41_i2c_read_sequence(&dev, &wData[0], sizeof(wData), &rData[0], sizeof(rData), 1);
    if (i2c_stat != I2C_OK) {
#ifdef DEBUG
        printf("Error %u requesting data ready status\n", i2c_stat);
#endif // DEBUG
        return false; // Return false if starting the read failed
    }

    // Verify the CRC of the ASC enabled data
    uint8_t crc = sensirion_common_generate_crc(&rData[0], 2);
    if (rData[2] != crc) {
#ifdef DEBUG
        printf("CRC check failed for automatic self calibration data.\n");
        printf("Read CRC: %u, calculated CRC: %u\n", rData[2], crc);
#endif // DEBUG
        return false; // Return false if CRC check fails
    }

    uint16_t dataReady = (rData[0] << 8) | rData[1]; // Combine high and low bytes
    if ((dataReady & SCD41_DATA_READY_MASK) != 0)  {
#ifdef DEBUG
        printf("Data is ready.\n");
#endif // DEBUG
        return true; // Return true if data is ready
    } else {
#ifdef DEBUG
        printf("Data is not ready.\n");
#endif // DEBUG
        return false; // Return false if data is not ready
    }
}

uint8_t scd41persistSettings(void)
{
    // Persist Settings Command (Reg 0x3615, no data)
    // Prepare the data buffer with the command
    uint8_t wData[2] = {I2C_SCD41_COMMAND_PERSIST_SETTINGS};
    i2c_stat = scd41_i2c_write_sequence(&dev, &wData[0], sizeof(wData));
    if (i2c_stat != I2C_OK) {
#ifdef DEBUG
        printf("Error %u persisting settings\n", i2c_stat);
#endif
        return false; // Return false if persisting settings failed
    }
    // Wait for the command to complete
    Delay_Ms(800);

#ifdef DEBUG
    printf("Settings persisted successfully.\n");
#endif // DEBUG
    return true; // Return true if successful
}

uint8_t scd41performSelfTest(void)
{
    // Perform Self Test Read (Reg 0x3639, no data)
    // Prepare the data buffer with the command
    uint8_t wData[2] = {I2C_SCD41_READ_PERFORM_SELF_TEST};
    // Prepare the data buffer to receive the self test response
    uint8_t rData[3] = {0};
    i2c_stat = scd41_i2c_read_sequence(&dev, &wData[0], sizeof(wData), &rData[0], sizeof(rData), 5500);
    if (i2c_stat != I2C_OK) {
#ifdef DEBUG
        printf("Error %u performing self test\n", i2c_stat);
#endif // DEBUG
        return false; // Return false if performing the self test failed
    }
    
    // Verify the CRC of the ASC enabled data
    uint8_t crc = sensirion_common_generate_crc(&rData[0], 2);
    if (rData[2] != crc) {
#ifdef DEBUG
        printf("CRC check failed for automatic self calibration data.\n");
        printf("Read CRC: %u, calculated CRC: %u\n", rData[2], crc);
#endif
        return false; // Return false if CRC check fails
    }

    // Convert the raw data to uint8_t value
    if (rData[0] != 0x00 && rData[1] != 0x00) {
#ifdef DEBUG
        printf("Self-test failed.\n");
#endif // DEBUG
        return false; // Return false if data is not ready
    } else {
#ifdef DEBUG
        printf("Self-test passed.\n");
#endif // DEBUG
    }   return true; // Return true if self-test passed
}

uint8_t scd41factoryReset(void)
{
    // Factory Reset Command (Reg 0x3632, no data)
     // Prepare the data buffer with the command
    uint8_t wData[2] = {I2C_SCD41_COMMAND_FACTORY_RESET};
    i2c_stat = scd41_i2c_write_sequence(&dev, &wData[0], sizeof(wData));
    if (i2c_stat != I2C_OK) {
#ifdef DEBUG
        printf("Error %u performing factory reset\n", i2c_stat);
#endif // DEBUG
        return false; // Return false if performing the factory reset failed
    }

    // Wait for the command to complete
    Delay_Ms(1200); // Wait for 1.2 seconds as per the datasheet
#ifdef DEBUG
    printf("Factory reset performed successfully.\n");
#endif // DEBUG
    return true; // Return true if successful
}

uint8_t scd41reInit(void)
{
    // ReInit Command (Reg 0x3646, no data)
    // Prepare the data buffer with the command
    uint8_t wData[2] = {I2C_SCD41_COMMAND_REINITIALIZE};
    i2c_stat = scd41_i2c_write_sequence(&dev, &wData[0], sizeof(wData));
    if (i2c_stat != I2C_OK) {
#ifdef DEBUG
        printf("Error %u performing reinitialization\n", i2c_stat);
#endif // DEBUG
        return false; // Return false if performing the factory reset failed
    }

    // Wait for the command to complete
    Delay_Ms(20); 
#ifdef DEBUG
    printf("Reinitialization performed successfully.\n");
#endif // DEBUG
    return true; // Return true if successful}
}

uint8_t scd41measureSingleShot(void)
{
    // Measure Single Shot Command (Reg 0x219D, no data)
    // Prepare the data buffer with the command
    uint8_t wData[2] = {I2C_SCD41_COMMAND_MEASURE_SINGLE_SHOT};
    i2c_stat = scd41_i2c_write_sequence(&dev, &wData[0], sizeof(wData));
    if (i2c_stat != I2C_OK) {
#ifdef DEBUG
        printf("Error %u starting single shot measurement request\n", i2c_stat);
#endif // DEBUG
        return false; // Return false if starting the single shot measurement failed
    }
    
    // Wait for the command to complete
    Delay_Ms(1350); 

#ifdef DEBUG
    printf("Single shot measurement requested successfully.\n");
#endif // DEBUG
    return true; // Return true if successful
}

uint8_t scd41readSingleShotOnlyRHT(void)
{
    // This function is to request a one-off reading of the humidity and temperature only.
    // After this command, the data can be read using the readMeasurement function.
    
    // Measure Single Shot Command (Reg 0x219D, no data)
    // Prepare the data buffer with the command
    uint8_t wData[2] = {I2C_SCD41_COMMAND_MEASURE_SINGLE_SHOT};
    i2c_stat = scd41_i2c_write_sequence(&dev, &wData[0], sizeof(wData));
    if (i2c_stat != I2C_OK) {
#ifdef DEBUG
        printf("Error %u starting single shot measurement (only humitidy & temperature)\n", i2c_stat);
#endif // DEBUG
        return false; // Return false if starting the single shot measurement failed
    }
    
    // Wait for the command to complete
    Delay_Ms(50); 

#ifdef DEBUG
    printf("Single shot measurement (only humitidy & temperature) requested successfully.\n");
#endif // DEBUG
    return true; // Return true if successful
}