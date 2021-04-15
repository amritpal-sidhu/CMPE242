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
    LSM303DLHCAcc_FilterConfigTypeDef acc_filter_config;
    
    int16_t acc[3];
    float mag[3];
    uint8_t data_buf[6];

    if (jetson_nano_i2c_init() == -1) {
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
    
    /* Initialize filter */
    acc_filter_config.HighPassFilter_Mode_Selection = LSM303DLHC_HPM_NORMAL_MODE;
    acc_filter_config.HighPassFilter_CutOff_Frequency = LSM303DLHC_HPFCF_16;
    acc_filter_config.HighPassFilter_AOI1 = LSM303DLHC_HPF_AOI1_DISABLE;
    acc_filter_config.HighPassFilter_AOI2 = LSM303DLHC_HPF_AOI2_DISABLE;
    LSM303DLHC_AccFilterConfig(&acc_filter_config);
    LSM303DLHC_AccFilterCmd(LSM303DLHC_HIGHPASSFILTER_ENABLE);

    printf("Printing acc config register values for debugging\n");
    LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG1_A, data_buf, 6);
    for (size_t i = 0; i < 6; ++i) {
        printf("reg%li_a val = 0x%x\n", i+1, data_buf[i]);
    }

    printf("\nPrinting mag config register values for debugging\n");
    LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_CRA_REG_M, data_buf, 3);
    for (size_t i = 0; i < 3; ++i) {
        printf("reg%li_m val = 0x%x\n", i+1, data_buf[i]);
    }
    printf("\n");
    
    signal(SIGINT, stop_handler);

    while (!stop_signal) {
        
        LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_OUT_X_L_A, data_buf, 6);
        for (size_t i = 0; i < 3; ++i) {
        	acc[i] = ((int16_t)(((uint16_t)data_buf[2*i+1] << 8) | data_buf[2*i])>>4) * LSM303DLHC_A_SENSITIVITY_2G;
        }
        
        LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_X_H_M, data_buf, 6);
        for (size_t i = 0; i < 3; ++i) {
        	if (i == 1)
        		mag[i] = (float)((int16_t)(((uint16_t)data_buf[2*i] << 8) | data_buf[2*i+1])) / LSM303DLHC_M_SENSITIVITY_Z_1_3Ga;
        	else
        		mag[i] = (float)((int16_t)(((uint16_t)data_buf[2*i] << 8) | data_buf[2*i+1])) / LSM303DLHC_M_SENSITIVITY_XY_1_3Ga;
        }

        printf("\racc x = %6img, y = %6img, z = %6img\tmag x = %5.3fGa, y = %5.3fGa, z = %5.3fGa", acc[0], acc[1], acc[2], mag[0], mag[2], mag[1]);
        fflush(stdout);
    }

    printf("\nClosing I2C and terminating program\n");
    jetson_nano_i2c_deinit();

    return 0;
}
