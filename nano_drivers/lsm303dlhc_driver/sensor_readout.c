#include <stdio.h>
#include <sys/select.h>
#include "lsm303dlhc.h"

int main(void) {

    LSM303DLHCAcc_InitTypeDef acc_init;
    // LSM303DLHCAcc_FilterConfigTypeDef acc_filter_config;
    LSM303DLHCMag_InitTypeDef mag_init;
    
    uint16_t acc_x, acc_y, acc_z;
    uint16_t mag_x, mag_y, mag_z;
    uint8_t data_buf[2];
    
    fd_set rfd;
    int select_retval;

    if (jetson_nano_i2c_init() == -1) {
        printf("I2C initialization failed, exiting program\n");
        return -1;
    }

    /* Initialize accelerometer */
    acc_init.Power_Mode = LSM303DLHC_NORMAL_MODE;
    acc_init.AccOutput_DataRate = LSM303DLHC_ODR_400_HZ;
    acc_init.Axes_Enable = LSM303DLHC_AXES_ENABLE;
    acc_init.High_Resolution = LSM303DLHC_HR_DISABLE;
    acc_init.BlockData_Update = LSM303DLHC_BlockUpdate_Continous;
    acc_init.Endianness = LSM303DLHC_BLE_LSB;
    acc_init.AccFull_Scale = LSM303DLHC_FULLSCALE_8G;

    LSM303DLHC_AccInit(&acc_init);

    /* Initialize magnetometer */
    mag_init.Temperature_Sensor = LSM303DLHC_TEMPSENSOR_DISABLE;
    mag_init.MagOutput_DataRate = LSM303DLHC_ODR_75_HZ;
    mag_init.Working_Mode = LSM303DLHC_CONTINUOS_CONVERSION;
    mag_init.MagFull_Scale = LSM303DLHC_FS_5_6_GA;

    LSM303DLHC_MagInit(&mag_init);

    // using sys/select to check for stdin updates
    FD_ZERO(&rfd);
    FD_SET(0, &rfd);

    printf("Press any key to quit program\n");

    for (;;) {
        
        LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_OUT_X_L_A, &data_buf[0], 1);
        LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_OUT_X_H_A, &data_buf[1], 1);
        acc_x = (((uint16_t)data_buf[1] << 8) + data_buf[0]);

        LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_OUT_Y_L_A, &data_buf[0], 1);
        LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_OUT_Y_H_A, &data_buf[1], 1);
        acc_y = (((uint16_t)data_buf[1] << 8) + data_buf[0]);

        LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_OUT_Z_L_A, &data_buf[0], 1);
        LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_OUT_Z_H_A, &data_buf[1], 1);
        acc_z = (((uint16_t)data_buf[1] << 8) + data_buf[0]);

        LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_X_L_M, &data_buf[0], 1);
        LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_X_H_M, &data_buf[1], 1);
        mag_x = (((uint16_t)data_buf[1] << 8) + data_buf[0]);

        LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Y_L_M, &data_buf[0], 1);
        LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Y_H_M, &data_buf[1], 1);
        mag_y = (((uint16_t)data_buf[1] << 8) + data_buf[0]);

        LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Z_L_M, &data_buf[0], 1);
        LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Z_H_M, &data_buf[1], 1);
        mag_z = (((uint16_t)data_buf[1] << 8) + data_buf[0]);

        printf("acc x = %i, y = %i, z = %i\n", acc_x, acc_y, acc_z);
        printf("mag x = %i, y = %i, z = %i\n\n", mag_x, mag_y, mag_z);

        select_retval = select(1, &rfd, NULL, NULL, NULL);
        if (select_retval == -1) {
            printf("Error with select()\n");
        }
        else if (select_retval) {
            break;
        }
    }

    jetson_nano_i2c_deinit();

    return 0;
}
