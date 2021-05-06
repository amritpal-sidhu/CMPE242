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

int jetson_nano_i2c_init(void) {
    int retval = 0;
    unsigned long func;

    if ((i2c_fd = open(JETSON_NANO_I2C_BUS, O_RDWR)) < 0) {
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

void jetson_nano_i2c_deinit(void) {
    close(i2c_fd);
}

void ADS1015_config(void) {

}

void ADS1015_start_conversion(void) {

}

/* read write funtions */
uint16_t ADS1015_Write(uint8_t DeviceAddr, uint8_t RegAddr, uint8_t* pBuffer) {
    
    struct i2c_msg ioctl_msg;
    struct i2c_rdwr_ioctl_data ioctl_data;
    uint8_t write_buffer[2] = {RegAddr, *pBuffer};
    uint16_t write_status = 1;

    // needed to work with ioctl
    DeviceAddr >>= 1;

    // Write data to device
    ioctl_msg.addr = DeviceAddr;
    ioctl_msg.buf = write_buffer;
    ioctl_msg.len = 2;
    ioctl_msg.flags = 0;

    ioctl_data.msgs = &ioctl_msg;
    ioctl_data.nmsgs = 1;

    if (ioctl(i2c_fd, I2C_RDWR, &ioctl_data) < 0) {
        printf("Writing ADS1015 failed\n");
        write_status = 0;
    }
    
    return write_status;
}

uint16_t ADS1015_Read(uint8_t DeviceAddr, uint8_t RegAddr,uint8_t* pBuffer, uint16_t NumByteToRead) {
    
    struct i2c_msg ioctl_msg[2];
    struct i2c_rdwr_ioctl_data ioctl_data;
    uint16_t read_status = 1;
    
    if(NumByteToRead>1)
        RegAddr |= 0x80;

    // needed to work with ioctl
    DeviceAddr >>= 1;

    // Writing SUB address to read
    ioctl_msg[0].addr = DeviceAddr;
    ioctl_msg[0].buf = &RegAddr;
    ioctl_msg[0].len = 1;
    ioctl_msg[0].flags = 0;

    // Reading from SUB address
    ioctl_msg[1].addr = DeviceAddr;
    ioctl_msg[1].buf = pBuffer;
    ioctl_msg[1].len = NumByteToRead;
    ioctl_msg[1].flags = I2C_M_RD;

    ioctl_data.msgs = ioctl_msg;
    ioctl_data.nmsgs = 2;

    if (ioctl(i2c_fd, I2C_RDWR, &ioctl_data) < 0) {
        printf("Reading ADS1015 failed\n");
        read_status = 0;
    }
    
    return read_status;
}
