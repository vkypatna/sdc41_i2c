/******************************************************************************
* Wrapper for the CH32FUN I2C_LIB library which offers 16 bit commands used
* for the Sensiron SCD41 co2, temperature and humidity sensor.
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

#ifndef SCD41_I2C_H
#define SCD41_I2C_H

#include "ch32fun.h"
#include "lib_i2c.h"

extern i2c_err_t i2c_err;
extern i2c_device_t i2c_device;

/// @brief send a command to the device to read some specific data.  
/// @param dev i2c_device_t, the i2c device that has been initialized and is waiting for commands
/// @param wbuf pointer to write buffer which contains the command and data to be send to the device (commands are 2 bytes).
/// @param wlen uint8_t, the size of the write buffer.
/// @param rbuf pointer to read buffer which will contain the data returned from the device. typically the device returns byte triplets 
///             with 2 bytes and a CRC check byte.
/// @param rlen uint8_8, the size of the read buffer and therefore the number of bytes that will be read.
/// @param delayBeforeRead uint32_t if a delay is needed between writing the write buffer and reading the read buffer, it can be specified
///                        in micro-seconds.  this allows some commands to take time before they are ready to return results.
/// @return i2c_err, which is a status code with 0 being success and >0 being an error (see definition for values).
i2c_err_t scd41_i2c_read_sequence(const i2c_device_t *dev,	const uint8_t *wbuf, const uint8_t wlen,
						       uint8_t *rbuf, const uint8_t rlen, const uint32_t delayBeforeRead);

/// @brief send a command to the device, which optionally may have data.  a command is generally a 16 bit integer which identifies which 
///        particular function is needed.  a write sequence is a command plus some data (for example to set some configuration).  the data
///        is typically 3 bytes; a 2 byte word and a CRC checksum.
/// @param dev i2c_device_t, the i2c device that has been initialized and is waiting for commands
/// @param wbuf pointer to write buffer which contains the command, plus optionally some data.  this is typically a 2 byte command (for example
///             0x3639 for perform self-test) and an optional 2 byte data word plus a CRC checksum).  this buffer is generally 2 bytes long or
///             5 bytes long
/// @param wlen uint8_t, the size of the write buffer.
/// @return i2c_err, which is a status code with 0 being success and >0 being an error (see definition for values).
i2c_err_t scd41_i2c_write_sequence(const i2c_device_t *dev, const uint8_t *wbuf, const uint8_t wlen);

#endif // SCD41_I2C_H