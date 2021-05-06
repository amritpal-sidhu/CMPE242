#pragma once

#include <stdint.h>

typedef struct {
    uint8_t mode;
    uint8_t mux_config;
    uint8_t data_rate;
    uint8_t pga_fsr;
} ADS1015_adc_config;

typedef struct {
    uint8_t mode;
    uint8_t polarity;
    uint8_t latching;
    uint8_t queue;
} ADS1015_comp_config;

/**
 * Jetson nano I2C0 on J41 pin header:
 *   I2C0_SDA - pin 27 - PB.05
 *   I2C0_SCL - pin 28 - PC.02
 * This program is using the linux device drivers to interface
 * with the LSM303DLHC
 */
#define JETSON_NANO_I2C_BUS                  "/dev/i2c-1"


/**
 * Possible modes
 */
#define ADS1015_CONTINUOUS              0x01
#define ADS1015_SINGLE_SHOT             0x02

/**
 * Register mapping.
 * All are 16-bit R/W Registers
 */
#define ADS1015_CONV_REG                0x00 /* Data read */
#define ADS1015_CONFIG_REG              0x01
#define ADS1015_LO_THRES_REG            0x02
#define ADS1015_HI_THRES_REG            0x03

#define I2C_ADDRESS                     0x32

/**
 * Possible multiplexer configurations
 */
#define ADS1015_MUX_AIN0_AIN1_DIFF      0x00
#define ADS1015_MUX_AIN0_AIN3_DIFF      0x01
#define ADS1015_MUX_AIN1_AIN3_DIFF      0x02
#define ADS1015_MUX_AIN2_AIN3_DIFF      0x03
#define ADS1015_MUX_AIN0_SINGLE         0x04
#define ADS1015_MUX_AIN1_SINGLE         0x05
#define ADS1015_MUX_AIN2_SINGLE         0x06
#define ADS1015_MUX_AIN3_SINGLE         0x07

/**
 * Possible programmable gain amplifier full scale range configurations
 */
#define ADS1015_PGA_6_144               0x00
#define ADS1015_PGA_4_096               0x01
#define ADS1015_PGA_2_048               0x02
#define ADS1015_PGA_1_024               0x03
#define ADS1015_PGA_0_512               0x04
#define ADS1015_PGA_0_256               0x05

/**
 * Possible data rate configurations
 */
#define ADS1015_RATE_128SPS             0x00
#define ADS1015_RATE_250SPS             0x01
#define ADS1015_RATE_490SPS             0x02
#define ADS1015_RATE_920SPS             0x03
#define ADS1015_RATE_1600SPS            0x04
#define ADS1015_RATE_2400SPS            0x05
#define ADS1015_RATE_3300SPS            0x06

/**
 * Possible comparator modes
 */
#define ADS1015_COMP_NORMAL_MODE        0x00
#define ADS1015_COMP_NORMAL_MODE        0x01

/**
 * Possible comparator polarity
 */
#define ADS1015_COMP_ACTIVE_LOW        0x00
#define ADS1015_COMP_ACTIVE_HIGH       0x01

/**
 * Possible comparator latching
 */
#define ADS1015_COMP_NONLATCHING        0x00
#define ADS1015_COMP_LATCHING           0x01

/**
 * Possible comparator queue config
 */
#define ADS1015_COMP_ALERT_1            0x00
#define ADS1015_COMP_ALERT_2            0x01
#define ADS1015_COMP_ALERT_3            0x02
#define ADS1015_COMP_ALERT_DISABLE      0x03

/**
 *  Jetson Nano Linux I2C function  prototypes
 */
/**
 * Open Linux I2C file descriptor.
 */
int jetson_nano_i2c_init(void);

/**
 * Close Linux I2C file descriptor.
 */
void jetson_nano_i2c_deinit(void);

/**
 * ADS1015 function  prototypes
 */
void ADS1015_config(void);
void ADS1015_start_conversion(void);

/**
 * Read/Write funtion prototypes
 */

/**
  * @brief  Writes one byte to the ADS1015.  Adapted to use ioctrl and work with Linux
  *         I2C device driver for the Nvidia Jetson Nano.
  * @param  DeviceAddr : specifies the slave address to be programmed.
  * @param  RegAddr : specifies the LSM303DLHC register to be written.
  * @param  pBuffer : pointer to the buffer containing the data to be written to the LSM303DLHC.
  *                   It is assumed to be statically allocated memory.
  * @retval 1 on success, 0 on failure.
  */
uint16_t ADS1015_Write(uint8_t DeviceAddr, uint8_t RegAddr, uint8_t* pBuffer);

/**
  * @brief  Reads a block of data from the ADS1015.  Adapted to use ioctrl and work with Linux
  *         I2C device driver for the Nvidia Jetson Nano.
  * @param  DeviceAddr : specifies the slave address to be programmed(ACC_I2C_ADDRESS or MAG_I2C_ADDRESS).
  * @param  RegAddr : specifies the LSM303DLHC internal address register to read from.
  * @param  pBuffer : pointer to the buffer that receives the data read from the LSM303DLH.
  * @param  NumByteToRead : number of bytes to read from the LSM303DLH ( NumByteToRead >1  only for the Mgnetometer readinf).
  * @retval 1 on success, 0 on failure.
  */
uint16_t ADS1015_Read(uint8_t DeviceAddr, uint8_t RegAddr,uint8_t* pBuffer, uint16_t NumByteToRead);
