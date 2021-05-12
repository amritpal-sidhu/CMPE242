#include "ads1015.h"

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>


/**
 * Local global variables
 */
static int i2c_fd = -1;

/**
 * Function definitions
 */

int ADS1015_jetson_nano_i2c_init(void) {
    int retval = 0;
    unsigned long func;

    if ((i2c_fd = open(ADS1015_JETSON_NANO_I2C_BUS, O_RDWR)) < 0) {
        retval = -1;
    }
    else {
        ioctl(i2c_fd, I2C_FUNCS, &func);
        if (!(func & I2C_FUNC_I2C)) {
        printf("Adapter %li does not have I2C_FUNC_I2C\n", func);
        }
    }

    return retval;
}

void ADS1015_jetson_nano_i2c_deinit(void) {
    close(i2c_fd);
}

void ADS1015_config_adc(ADS1015_adc_config config_data) {
    uint8_t reg_data[2];

    ADS1015_Read(ADS1015_I2C_ADDRESS, ADS1015_CONFIG_REG, reg_data);

    // upper byte is written first
    reg_data[0] &= 0x00;
    reg_data[0] |= (config_data.mux_config << 4) | (config_data.pga_fsr << 1) | config_data.mode;
    reg_data[1] &= 0x1F; // clear data rate bits
    reg_data[1] |= config_data.data_rate << 5;

    ADS1015_Write(ADS1015_I2C_ADDRESS, ADS1015_CONFIG_REG, reg_data);
}

void ADS1015_start_conversion(void) {
    uint8_t reg_data[2];

    ADS1015_Read(ADS1015_I2C_ADDRESS, ADS1015_CONFIG_REG, reg_data);

    if (reg_data[0] & 0x01) {

        reg_data[0] |= 0x80;
        ADS1015_Write(ADS1015_I2C_ADDRESS, ADS1015_CONFIG_REG, reg_data);
    }
    else {
        printf("ADS1015 is not in single shot mode\n");
    }
}

void ADS1015_config_comp(ADS1015_comp_config config_data) {
    uint8_t reg_data[2];

    ADS1015_Read(ADS1015_I2C_ADDRESS, ADS1015_CONFIG_REG, reg_data);

    reg_data[1] &= ~0x1F; // clear data rate bits
    reg_data[1] |= (config_data.mode << 4) | (config_data.polarity << 3) | (config_data.latching << 2) | config_data.queue;

    ADS1015_Write(ADS1015_I2C_ADDRESS, ADS1015_CONFIG_REG, reg_data);
}

int16_t ADS1015_get_data(void) {
    uint8_t adc_raw_data[2];
    int16_t adc_data;

    ADS1015_Read(ADS1015_I2C_ADDRESS, ADS1015_CONV_REG, adc_raw_data);

    adc_data = (((int16_t)adc_raw_data[0] << 8) | adc_raw_data[1]) >> 4;

    return adc_data;
}

/* read write funtions */
uint16_t ADS1015_Write(uint8_t DeviceAddr, uint8_t RegAddr, uint8_t* pBuffer) {
    
    struct i2c_msg ioctl_msg;
    struct i2c_rdwr_ioctl_data ioctl_data;
    uint8_t write_buffer[3] = {RegAddr, pBuffer[0], pBuffer[1]};
    uint16_t write_status = 1;

    // Write data to device
    ioctl_msg.addr = DeviceAddr;
    ioctl_msg.buf = write_buffer;
    ioctl_msg.len = 3;
    ioctl_msg.flags = 0;

    ioctl_data.msgs = &ioctl_msg;
    ioctl_data.nmsgs = 1;

    if (ioctl(i2c_fd, I2C_RDWR, &ioctl_data) < 0) {
        printf("Writing ADS1015 failed\n");
        write_status = 0;
    }
    
    return write_status;
}

uint16_t ADS1015_Read(uint8_t DeviceAddr, uint8_t RegAddr,uint8_t* pBuffer) {
    
    struct i2c_msg ioctl_msg[2];
    struct i2c_rdwr_ioctl_data ioctl_data;
    uint16_t read_status = 1;

    // Writing SUB address to read
    ioctl_msg[0].addr = DeviceAddr;
    ioctl_msg[0].buf = &RegAddr;
    ioctl_msg[0].len = 1;
    ioctl_msg[0].flags = 0;

    // Reading from SUB address
    ioctl_msg[1].addr = DeviceAddr;
    ioctl_msg[1].buf = pBuffer;
    ioctl_msg[1].len = 2;
    ioctl_msg[1].flags = I2C_M_RD;

    ioctl_data.msgs = ioctl_msg;
    ioctl_data.nmsgs = 2;

    if (ioctl(i2c_fd, I2C_RDWR, &ioctl_data) < 0) {
        printf("Reading ADS1015 failed\n");
        read_status = 0;
    }
    
    return read_status;
}
