#include <stdio.h>
#include <signal.h>

#include "lsm303dlhc.h"

sig_atomic_t stop_signal = 0;

void stop_handler(int param) {
    if (param) {
        /* ignore param */
    }
    stop_signal = 1;
}

int main(void) {

    LSM303DLHCAcc_InitTypeDef acc_init;
    LSM303DLHCMag_InitTypeDef mag_init;
    
    int16_t accX, accY, accZ;
    float magX, magY, magZ;
    uint8_t data_buf[6];

    if (LSM303DLHC_jetson_nano_i2c_init() == -1) {
        printf("I2C initialization failed, exiting program\n");
        return -1;
    }

    /* Initialize accelerometer */
    acc_init.Power_Mode = LSM303DLHC_NORMAL_MODE;
    acc_init.AccOutput_DataRate = LSM303DLHC_ODR_400_HZ;
    acc_init.Axes_Enable = LSM303DLHC_AXES_ENABLE;
    acc_init.High_Resolution = LSM303DLHC_HR_ENABLE;
    acc_init.BlockData_Update = LSM303DLHC_BlockUpdate_Continous;
    acc_init.Endianness = LSM303DLHC_BLE_LSB;
    acc_init.AccFull_Scale = LSM303DLHC_FULLSCALE_2G;
    LSM303DLHC_AccInit(&acc_init);

    /* Initialize magnetometer */
    mag_init.Temperature_Sensor = LSM303DLHC_TEMPSENSOR_DISABLE;
    mag_init.MagOutput_DataRate = LSM303DLHC_ODR_75_HZ;
    mag_init.Working_Mode = LSM303DLHC_CONTINUOS_CONVERSION;
    mag_init.MagFull_Scale = LSM303DLHC_FS_1_3_GA;
    LSM303DLHC_MagInit(&mag_init);
    
    signal(SIGINT, stop_handler);

    while (!stop_signal) {
        
        accX = LSM303DLHC_AccGetDataX(&acc_init);
        accY = LSM303DLHC_AccGetDataY(&acc_init);
        accZ = LSM303DLHC_AccGetDataZ(&acc_init);
        magX = LSM303DLHC_MagGetDataX(&mag_init);
        magY = LSM303DLHC_MagGetDataY(&mag_init);
        magZ = LSM303DLHC_MagGetDataZ(&mag_init);
        printf("\racc x = %6img, y = %6img, z = %6img\tmag x = %6.3fGa, y = %6.3fGa, z = %6.3fGa ", accX, accY, accZ, magX, magY, magZ);
        
        fflush(stdout);
    }

    printf("\nClosing I2C and terminating program\n");
    LSM303DLHC_jetson_nano_i2c_deinit();

    return 0;
}
