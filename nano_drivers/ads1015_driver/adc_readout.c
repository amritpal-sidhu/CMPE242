#include <stdio.h>
#include <signal.h>

#include "ads1015.h"

sig_atomic_t stop_signal = 0;

void stop_handler(int param) {
    if (param) {
        /* ignore param */
    }
    stop_signal = 1;
}

int main(void) {

    int16_t adc_data;
    uint8_t reg_data[2];

    ADS1015_adc_config adc_config = {
        .mode = ADS1015_CONTINUOUS,
        .mux_config = ADS1015_MUX_AIN0_SINGLE,
        .data_rate = ADS1015_RATE_1600SPS,
        .pga_fsr = ADS1015_PGA_4_096
    };

    if (ADS1015_jetson_nano_i2c_init() == -1) {
        printf("I2C initialization failed, exiting program\n");
        return -1;
    }

    ADS1015_config_adc(adc_config);

    // reading out config register for debugging
    ADS1015_Read(ADS1015_I2C_ADDRESS, ADS1015_CONFIG_REG, reg_data);
    printf("config reg low = 0x%x, high = 0x%x\n", reg_data[1], reg_data[0]);

    signal(SIGINT, stop_handler);

    while (!stop_signal) {

        adc_data = ADS1015_get_data();

        printf("\radc data = %6i", adc_data);
        fflush(stdout);
    }

    printf("\nClosing I2C and terminating program\n");
    ADS1015_jetson_nano_i2c_deinit();

    return 0;
}
