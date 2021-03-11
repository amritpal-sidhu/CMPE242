#pragma once

// TODO: set up interrupt to sample sensor
const unsigned buffer_size = 6;
float history_buffer[buffer_size] = {0}; // the 0th entry is the current sample
float kernel[buffer_size] = {0};

/**
 * @brief  Construct Kernel based on central difference.
 * @retval None
 */
void construct_kernel_central(void);

/**
 * @brief  Construct Kernel based on forward difference.
 * @retval None
 */
void construct_kernel_forward(void);

/**
 * @brief  Construct Kernel based on backward difference.
 * @retval None
 */
void construct_kernel_backward(void);

/**
 * @brief  Construct Kernel based on the Laplacian of Gaussian,
 *         using central difference.
 * @param  std: Standard deviation
 * @retval None
 */
void construct_kernel_LoG(float std);

// TODO: use the above in convolution
