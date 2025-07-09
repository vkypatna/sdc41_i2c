
#ifndef CH32_LIB_I2C_H
#define CH32_LIB_I2C_H

#include "ch32fun.h"

// I2C Timeout count
#define TIMEOUT_MAX 10000U

// TESTED: DEFAULT OK	ALT_1 OK
#define I2C_PINOUT_DEFAULT
//#define I2C_PINOUT_ALT_1
//#define I2C_PINOUT_ALT_2

/*** Hardware Definitions ****************************************************/
// Predefined Clock Speeds
#define I2C_CLK_10KHZ     10000
#define I2C_CLK_50KHZ     50000
#define I2C_CLK_100KHZ    100000
#define I2C_CLK_400KHZ    400000
#define I2C_CLK_500KHZ    500000
#define I2C_CLK_600KHZ    600000
#define I2C_CLK_750KHZ    750000
#define I2C_CLK_1MHZ      1000000

// Hardware CLK Prerate and timeout
#define I2C_PRERATE       1000000
#define I2C_TIMEOUT       2000

// Default Pinout
#ifdef I2C_PINOUT_DEFAULT
	#define I2C_AFIO_REG	((uint32_t)0x00000000)
	#define I2C_PORT_RCC	RCC_APB2Periph_GPIOC
	#define I2C_PORT		GPIOC
	#define I2C_PIN_SCL 	2
	#define I2C_PIN_SDA 	1
#endif

// Alternate 1 Pinout
#ifdef I2C_PINOUT_ALT_1
	#define I2C_AFIO_REG	((uint32_t)0x04000002)
	#define I2C_PORT_RCC	RCC_APB2Periph_GPIOD
	#define I2C_PORT		GPIOD
	#define I2C_PIN_SCL 	1
	#define I2C_PIN_SDA 	0
#endif

// Alternate 2 Pinout
#ifdef I2C_PINOUT_ALT_2
	#define I2C_AFIO_REG	((uint32_t)0x00400002)
	#define I2C_PORT_RCC	RCC_APB2Periph_GPIOC
	#define I2C_PORT		GPIOC
	#define I2C_PIN_SCL 	5
	#define I2C_PIN_SDA 	6
#endif

/*** Types and Enums *********************************************************/
/// @brief I2C Error Codes
typedef enum {
	I2C_OK	  = 0,   // No Error. All OK
	I2C_ERR_BERR,	 // Bus Error
	I2C_ERR_NACK,	 // ACK Bit failed
	I2C_ERR_ARLO,	 // Arbitration Lost
	I2C_ERR_OVR,	 // Overun/underrun condition
	I2C_ERR_BUSY,	 // Bus was busy and timed out
} i2c_err_t;

/// @brief I2C Address Mode Types:
/// 7bit is the standard I2C Address mode. ADDR[LSB] is the read/write bit
/// 10bit ADDR is 2 bytes, the ADDR[0][LSB] is the read/write bit.
/// 16bit Mode sends the Address raw as 2 bytes to support some devices
typedef enum {
	I2C_ADDR_7BIT     = 0,
//	I2C_ADDR_10BIT,
//	I2C_ADDR_16BIT
} i2c_addr_t;


typedef struct {
	i2c_addr_t    type;  // Address Type - Determines address behaviour
	uint16_t      addr;  // Address Value. Default is WRITE in 7 and 10bit
} i2c_device_t;

/*** Functions ***************************************************************/
/// @brief Initialise the I2C Peripheral on the default pins, in Master Mode
/// @param clk_rate that the I2C Bus should use in Hz
/// @return i2c_err_t, I2C_OK On success
i2c_err_t i2c_init(const uint32_t clk_rate);

/// @brief Pings a specific I2C Address, and returns a i2c_err_t status
/// @param addr I2C Device Address,                    NOTE: 7BIT ADDRESS ONLY
/// @return i2c_err_t, I2C_OK if the device responds
i2c_err_t i2c_ping(const uint8_t addr);

/// @brief Scans through all 7 Bit addresses, prints any that respond
/// @param callback function - returns void, takes uint8_t
///                                                    NOTE: 7BIT ADDRESS ONLY
/// @return None
void i2c_scan(void (*callback)(const uint8_t));



// TODO: read/write raw

/// @brief reads [len] bytes from [addr]s [reg] register into [buf]
/// @param dev, I2C Device to Read from
/// @param reg, register to read from
/// @param buf, buffer to read to
/// @param len, number of bytes to read
/// @return 12c_err_t. I2C_OK on Success
i2c_err_t i2c_read_reg(const i2c_device_t *dev,		const uint8_t reg,
													uint8_t *buf,
													const uint8_t len);


/// @brief writes [len] bytes from [buf], to the [reg] of [addr]
/// @param dev, I2C Device to Write to
/// @param reg, register to write to
/// @param buf, Buffer to write from
/// @param len, number of bytes to read
/// @return i2c_err_t. I2C_OK On Success.
i2c_err_t i2c_write_reg(const i2c_device_t *dev,	const uint8_t reg,
													const uint8_t *buf,
													const uint8_t len);



#endif