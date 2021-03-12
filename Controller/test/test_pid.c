#include <stdio.h>
#include <math.h>

#include "pid.h"

static void test_derivatives(float e0, float e1, float e2, float std) {
    
    printf("TESTING derivatives with [%0.1f, %0.1f, %0.1f] and std = %0.1f\n", e2, e1, e0, std);
    printf("central: %0.3f\n", derivative_c(e0, e2));
    printf("forward: %0.3f\n", derivative_f(e0, e1));
    printf("backward: %0.3f\n", derivative_b(e1, e2));
    printf("LoG: %0.3f\n\n", derivative_LoG(e0, e2, std));
}

int main(void) {

    // history buffer moves up, slows down, goes back fast with an abrubt stop
    const float test_buffer[] = {0.0, 0.2, 0.4, 0.8, 1.2, 1.4, 1.4, 1.4, 1.0, 0.5, 0.1, -0.4, -1.2, -1.6, -1.8, -1.8, -1.8, -1.8};
    const unsigned int size = sizeof(test_buffer) / sizeof(float);

    // calculate mean
    float mean = 0, std = 0;
    for (unsigned i = 0; i < size; ++i)
        mean += test_buffer[i];
    mean /= size;

    // calculate standard deviation
    for (unsigned i = 0; i < size; ++i)
        std += pow(test_buffer[i]-mean, 2);
    std = sqrt(std/size);

    // run tests
    for (unsigned i = 1; i < size-1; ++i)
        test_derivatives(test_buffer[i+1], test_buffer[i], test_buffer[i-1], std);

    return 0;
}