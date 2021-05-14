#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <time.h>
#include <math.h>

#include "ads1015.h"

#define MAX_LOGGING_POINTS      1024

/**
 * The FFT function implementation is taken
 * from https://github.com/hualili/CMPE242-Embedded-Systems-/blob/master/2019S/13-2018S-26-fft.c
 */
struct Complex
{	double real;
	double imag;
}          X[MAX_LOGGING_POINTS], U, W, T, Tmp;

void FFT(void);

sig_atomic_t stop_signal = 0;

void stop_handler(int param) {
    if (param) {
        /* ignore param */
    }
    stop_signal = 1;
}

int main(void) {

    FILE *fd;
    char str_buf[16];
	struct timespec ts = {.tv_sec = 0, .tv_nsec = 1000000};

    ADS1015_adc_config adc_config = {
        .mode = ADS1015_CONTINUOUS,
        .mux_config = ADS1015_MUX_AIN0_SINGLE,
        .data_rate = ADS1015_RATE_920SPS,
        .pga_fsr = ADS1015_PGA_4_096
    };

    if (ADS1015_jetson_nano_i2c_init() == -1) {
        printf("I2C initialization failed, exiting program\n");
        return -1;
    }

    ADS1015_config_adc(adc_config);
    signal(SIGINT, stop_handler);
    fd = fopen("adc_log_data.csv", "w");

    if (fd == NULL) {
        printf("Log file could not be opened\n");
        return -1;
    }

    for (int i = 0; i < MAX_LOGGING_POINTS; ++i) {

        snprintf(str_buf, sizeof(str_buf), "%i\n", ADS1015_get_data());
        fwrite(str_buf, strlen(str_buf), 1, fd);
		nanosleep(&ts, NULL);
    }

    printf("\nClosing I2C and terminating program\n");
    ADS1015_jetson_nano_i2c_deinit();
    fclose(fd);

    return 0;
}

void FFT(void) {
	int M = 2;
	int N = pow(2, M);

	int i = 1, j = 1, k = 1;
	int LE = 0, LE1 = 0;
	int IP = 0;

	for (k = 1; k <= M; k++)
	{
		LE = pow(2, M + 1 - k);
		LE1 = LE / 2;

		U.real = 1.0;
		U.imag = 0.0;

		W.real = cos(M_PI / (double)LE1);
		W.imag = -sin(M_PI/ (double)LE1);

		for (j = 1; j <= LE1; j++)
		{
			for (i = j; i <= N; i = i + LE)
			{
				IP = i + LE1;
				T.real = X[i].real + X[IP].real;
				T.imag = X[i].imag + X[IP].imag;
				Tmp.real = X[i].real - X[IP].real;
				Tmp.imag = X[i].imag - X[IP].imag;
				X[IP].real = (Tmp.real * U.real) - (Tmp.imag * U.imag);
				X[IP].imag = (Tmp.real * U.imag) + (Tmp.imag * U.real);
				X[i].real = T.real;
				X[i].imag = T.imag;
			}
			Tmp.real = (U.real * W.real) - (U.imag * W.imag);
			Tmp.imag = (U.real * W.imag) + (U.imag * W.real);
			U.real = Tmp.real;
			U.imag = Tmp.imag;
		}
	}

	int NV2 = N / 2;
	int NM1 = N - 1;
	int K = 0;

	j = 1;
	for (i = 1; i <= NM1; i++)
	{
		if (i >= j) goto TAG25;
		T.real = X[j].real;
		T.imag = X[j].imag;

		X[j].real = X[i].real;
		X[j].imag = X[i].imag;
		X[i].real = T.real;
		X[i].imag = T.imag;
TAG25:	K = NV2;
TAG26:	if (K >= j) goto TAG30;
		j = j - K;
		K = K / 2;
		goto TAG26;
TAG30:	j = j + K;
	}
}
