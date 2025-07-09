/******************************************************************************
* Lightweight and simple CH32V003 I2C Library.
*
* This library provides functions to init, read and write to the hardware I2C
* Bus - in Default, and Alternative Pinout Modes.
* Default:	SCL = PC2		SDA = PC1
* Alt 1:	SCL = PD1		SDA = PD0
* Alt 2:	SCL = PC5		SDA = PC6
*
* Version 4.3    03 May 2025
*
* See GitHub Repo for more information: 
* https://github.com/ADBeta/CH32V003_lib_i2c
* Released under the MIT Licence
* Copyright ADBeta (c) 2024 - 2025
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
#include "scd41_i2c.h"

///////////////////////////////////////////////////////////////////////////////
//                FUNCTIONALITY "BORROWED"" FROM CH32FUN I2C_LIB             //
///////////////////////////////////////////////////////////////////////////////

/*** Static Variables ********************************************************/
/// @brief Timeout variable. Set and decrimented in functions
static int32_t _i2c_timeout = 1000;


/*** Macro Functions *********************************************************/
#define I2C_TIMEOUT_WAIT_FOR(condition, err_var) \
do { \
	_i2c_timeout = I2C_TIMEOUT; \
	while((condition)) \
		if(--_i2c_timeout <= 0) {(err_var) = i2c_get_busy_error(); break;} \
} while(0)


/*** Static Functions ********************************************************/
/// @brief Checks the I2C Status against a mask value, returns 1 if it matches
/// @param Status To match to
/// @return uint32_t masked status value: 1 if mask and status match
__attribute__((always_inline))
static inline uint32_t i2c_status(const uint32_t status_mask)
{
	uint32_t status = (uint32_t)I2C1->STAR1 | (uint32_t)(I2C1->STAR2 << 16);
	return (status & status_mask) == status_mask; 
}


/// @brief Gets and returns any error state on the I2C Interface, and resets
/// the bit flags
/// @param none
/// @return i2c_err_t error value
static inline i2c_err_t i2c_error(void)
{
	if(I2C1->STAR1 & I2C_STAR1_BERR)  {I2C1->STAR1 &= ~I2C_STAR1_BERR;  return I2C_ERR_BERR;}
	if(I2C1->STAR1 & I2C_STAR1_AF)    {I2C1->STAR1 &= ~I2C_STAR1_AF;    return I2C_ERR_NACK;}
	if(I2C1->STAR1 & I2C_STAR1_ARLO)  {I2C1->STAR1 &= ~I2C_STAR1_ARLO;  return I2C_ERR_ARLO;}
	if(I2C1->STAR1 & I2C_STAR1_OVR)   {I2C1->STAR1 &= ~I2C_STAR1_OVR;   return I2C_ERR_OVR;}

	return I2C_OK;
}

/// @brief Called when the I2C Bus Timesout - Returns any known error code
/// if applicable - returns generic I2C_ERR_BUSY if not
/// @param None
/// @return i2c_err_t error value
__attribute__((always_inline))
static inline uint32_t i2c_get_busy_error(void)
{

	i2c_err_t i2c_err = i2c_error();
	if(i2c_err == I2C_OK) i2c_err = I2C_ERR_BUSY;
	return i2c_err;
}

/// @brief Waits for the I2C Bus to be ready
/// @param None
/// @return i2c_err_t, I2C_OK if the bus is ready
__attribute__((always_inline))
static inline i2c_err_t i2c_wait()
{
	i2c_err_t i2c_ret = I2C_OK;
	I2C_TIMEOUT_WAIT_FOR((I2C1->STAR2 & I2C_STAR2_BUSY), i2c_ret);

	return i2c_ret;
}

/// @brief Starts the I2C Bus for communications
/// @param None
/// @return None
__attribute__((always_inline))
static inline void i2c_start()
{
	// Send a START Signal and wait for it to assert
	I2C1->CTLR1 |= I2C_CTLR1_START;
	while(!i2c_status(I2C_EVENT_MASTER_MODE_SELECT));
}

/// @brief Stops the I2C Bus
/// @param None
/// @return None
__attribute__((always_inline))
static inline void i2c_stop()
{
	I2C1->CTLR1 |= I2C_CTLR1_STOP;
}

/// @brief Sends the Address Byte(s) to the I2C Device in Write mode
/// @param i2c_dev_t device to address
/// @return i2c_err_r error status. I2C_OK on success
static inline i2c_err_t i2c_send_addr_write(const i2c_device_t *dev)
{
	i2c_err_t i2c_ret = I2C_OK;
	
	if(dev->type == I2C_ADDR_7BIT)
	{
		// Send the Address and wait for it to finish transmitting
		I2C1->DATAR = (dev->addr << 1) & 0xFE;
		I2C_TIMEOUT_WAIT_FOR(!i2c_status(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED), i2c_ret);
	}

/*
	if(dev->type == I2C_ADDR_10BIT)
	{
		uint8_t upper = 0xF0 | ((dev->addr & 0x0300) >> 7);
		uint8_t lower = dev->addr & 0xFF;
	}
*/
	return i2c_ret;
}

/// @brief Sends the Address Byte(s) to the I2C Device in Read mode
/// @param i2c_dev_t device to address
/// @return i2c_err_r error status. I2C_OK on success
static inline i2c_err_t i2c_send_addr_read(const i2c_device_t *dev)
{
	i2c_err_t i2c_ret = I2C_OK;

	if(dev->type == I2C_ADDR_7BIT)
	{
		// Send the Address and wait for it to finish transmitting
		I2C1->DATAR = (dev->addr << 1) | 0x01;
		I2C_TIMEOUT_WAIT_FOR(!i2c_status(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED), i2c_ret);
	}
	return i2c_ret;
}

///////////////////////////////////////////////////////////////////////////////
//                      END OF "BORROWED"" FUNCTIONALITY                     //
///////////////////////////////////////////////////////////////////////////////


uint8_t checkEvent(uint32_t event_mask)
{
	/* read order matters here! STAR1 before STAR2!! */
	uint32_t status = I2C1->STAR1 | (I2C1->STAR2<<16);
	return (status & event_mask) == event_mask;
}

i2c_err_t scd41_i2c_write_sequence(const i2c_device_t *dev, const uint8_t *wbuf, const uint8_t wlen)
{
	// The write buffer should contain a sequence of bytes to write to the device:
	//   The first two bytes are the address of the command.
	//   The remaining bytes are the data to be written.
	
	i2c_err_t i2c_ret = I2C_OK;

	// Wait for the I2C Bus the be Available
	I2C_TIMEOUT_WAIT_FOR(I2C1->STAR2 & I2C_STAR2_BUSY, i2c_ret);
	
	// Start the I2C Bus and send the Write Address byte
	// If the device is busy, return an error
	i2c_start(); 
	i2c_ret = i2c_send_addr_write(dev);
	
	// Wait for the bus to confirm the address was ACK'd
	I2C_TIMEOUT_WAIT_FOR(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED, i2c_ret);

	// Send data to the bus one byte at a time
	uint8_t cbyte = 0;
	uint16_t timeout = TIMEOUT_MAX;
	while((cbyte < wlen) & (timeout > 0))
	{
		// If the data TX register is empty, send the next byte
		if (I2C1->STAR1 & I2C_STAR1_TXE) {
			I2C1->DATAR = wbuf[cbyte];
			++cbyte; 
		}
		timeout--;
	}
	if (timeout == 0) {
		// Signal a STOP
		i2c_stop();	
		// If we timed out, return an error
		i2c_ret = i2c_get_busy_error();
		return i2c_ret;
	}

	// Wait for the bus to finish transmitting
	I2C_TIMEOUT_WAIT_FOR(I2C_EVENT_MASTER_BYTE_TRANSMITTED, i2c_ret);
	Delay_Us(100); // Wait for the device to process the command

	// Signal a STOP
	i2c_stop();
	// i2c_ret = i2c_error();

	return I2C_OK;
}

i2c_err_t scd41_i2c_read_sequence(const i2c_device_t *dev,	const uint8_t *wbuf, const uint8_t wlen,
						    uint8_t *rbuf, const uint8_t rlen, uint32_t delayBeforeRead)
{
	// The write buffer should contain a sequence of bytes to write to the device:
	//   The first two bytes are the address of the command.
	
	i2c_err_t i2c_ret = I2C_OK;

	// Wait for the I2C Bus the be Available
	I2C_TIMEOUT_WAIT_FOR(I2C1->STAR2 & I2C_STAR2_BUSY, i2c_ret);
	
	// Start the I2C Bus and send the Write Address byte
	// If the device is busy, return an error
	i2c_start(); 
	i2c_ret = i2c_send_addr_write(dev);
	
	// Wait for the bus to confirm the address was ACK'd
	// I2C_TIMEOUT_WAIT_FOR(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED, i2c_ret);  <-- event changed by google ai suggestion!
	I2C_TIMEOUT_WAIT_FOR(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED , i2c_ret);

	// Send data to the bus one byte at a time
	uint8_t cbyte = 0;
	uint16_t timeout = TIMEOUT_MAX;
	while((cbyte < wlen) & (timeout > 0))
	{
		// If the data TX register is empty, send the next byte
		if (I2C1->STAR1 & I2C_STAR1_TXE) {
			I2C1->DATAR = wbuf[cbyte];
			++cbyte; 
		}
	}
	if (timeout == 0) {
		// Signal a STOP
		i2c_stop();
		// If we timed out, return an error
		i2c_ret = i2c_get_busy_error();
		return i2c_ret;
	}

	// Wait for the bus to finish transmitting
	I2C_TIMEOUT_WAIT_FOR(I2C_EVENT_MASTER_BYTE_TRANSMITTED, i2c_ret);
	
	// Wait for the device to perform its operation (the processing time)
	Delay_Ms(delayBeforeRead);
	
	// If the message is long enough, enable ACK messages (otherwise we will only read one byte)
	if(rlen > 1) I2C1->CTLR1 |= I2C_CTLR1_ACK;

	// Send a repeated START and the Read Address
	i2c_start(); 
	i2c_ret = i2c_send_addr_read(dev);

	// Wait for the bus to confirm the address was ACK'd
	I2C_TIMEOUT_WAIT_FOR(I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED, i2c_ret);
	
	// Recieve the data from the bus
	cbyte = 0;
	timeout = TIMEOUT_MAX;
	while((cbyte < rlen) & (timeout > 0))
	{
		// If this is the last byte, send the NACK Bit
		if(cbyte == rlen - 1) I2C1->CTLR1 &= ~I2C_CTLR1_ACK;

		// If there is data in the RX buffer, read it
		if ((I2C1->STAR1 & I2C_STAR1_RXNE) != 0) {
			rbuf[cbyte] = I2C1->DATAR;
			++cbyte; 
		}
		timeout--;
	}
		if (timeout == 0) {
		// If we timed out, return an error
		// Signal a STOP
		i2c_stop();
		i2c_ret = i2c_get_busy_error();
		return i2c_ret;
	}


	// Signal a STOP
	i2c_stop();
	i2c_ret = i2c_error();

	return i2c_ret;
}