/**
 * @date Modified on 06-April-2021
 * @note This c source is adapted from the driver provided by STMicroelectronics
 *       for the STM32F3 Discovery development board owned by the adapter,
 *       Amritpal Sidhu.  The file is adapted to work with the Nvidia Jetson
 *       Nano solely for purpose of education.
 * 
 *       See below for a copy of the original c source comment.
 * 
 ******************************************************************************
 * @file    stm32f3_discovery_lsm303dlhc.c
 * @author  MCD Application Team
 * @version V1.1.0
 * @date    20-September-2012
 * @brief   This file provides a set of functions needed to manage the lsm303dlhc
 *          MEMS accelerometer available on STM32F3-Discovery Kit.
 ******************************************************************************
 * @attention
 *
 *                    COPYRIGHT 2012 STMicroelectronics
 *
 * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *        http://www.st.com/software_license_agreement_liberty_v2
 *
 * Unless required by applicable law or agreed to in writing, software 
 * distributed under the License is distributed on an "AS IS" BASIS, 
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************
 */
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include "lsm303dlhc.h"

static int i2c_fd = -1;

static int16_t LSM303DLHC_AccDataConversion(const LSM303DLHCAcc_InitTypeDef *LSM303DLHC_InitStruct, const uint8_t *data_reg_values);
static float LSM303DLHC_MagDataConversion(const LSM303DLHCMag_InitTypeDef *LSM303DLHC_InitStruct, const uint8_t *data_reg_values, const uint8_t isZData);


/**
 * @brief  Open Linux I2C file descriptor.
 * @param  None
 * @retval -1 on error, file descriptor on success.
 */
int LSM303DLHC_jetson_nano_i2c_init(void) {

  int retval = 0;
  unsigned long func;

  if ((i2c_fd = open(LSM303DLHC_JETSON_NANO_I2C_BUS, O_RDWR)) < 0) {
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

/**
 * @brief  Close Linux I2C file descriptor.
 * @param  None
 * @retval None
 */
void LSM303DLHC_jetson_nano_i2c_deinit(void) {

  close(i2c_fd);
}

/**
  * @brief  Set LSM303DLHC Initialization.
  * @param  LSM303DLHC_InitStruct: pointer to a LSM303DLHC_InitTypeDef structure 
  *         that contains the configuration setting for the LSM303DLHC.
  * @retval None
  */
void LSM303DLHC_AccInit(const LSM303DLHCAcc_InitTypeDef *LSM303DLHC_InitStruct)
{  
  uint8_t ctrl1 = 0x00, ctrl4 = 0x00;
  
  /* Configure MEMS: data rate, power mode, full scale and axes */
  ctrl1 |= (uint8_t) (LSM303DLHC_InitStruct->Power_Mode | LSM303DLHC_InitStruct->AccOutput_DataRate | \
                    LSM303DLHC_InitStruct->Axes_Enable);
  
  ctrl4 |= (uint8_t) (LSM303DLHC_InitStruct->BlockData_Update | LSM303DLHC_InitStruct->Endianness | \
                    LSM303DLHC_InitStruct->AccFull_Scale|LSM303DLHC_InitStruct->High_Resolution);
                    
  /* Write value to ACC MEMS CTRL_REG1 regsister */
  LSM303DLHC_Write(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG1_A, &ctrl1);
  
  /* Write value to ACC MEMS CTRL_REG4 regsister */
  LSM303DLHC_Write(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG4_A, &ctrl4);
}

/**
  * @brief  Reboot memory content of LSM303DLHC
  * @param  None
  * @retval None
  */
void LSM303DLHC_AccRebootCmd(void)
{
  uint8_t tmpreg;
  
  /* Read CTRL_REG5 register */
  LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG5_A, &tmpreg, 1);
  
  /* Enable or Disable the reboot memory */
  tmpreg |= LSM303DLHC_BOOT_REBOOTMEMORY;
  
  /* Write value to ACC MEMS CTRL_REG5 regsister */
  LSM303DLHC_Write(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG5_A, &tmpreg);
}

/**
  * @brief  Set High Pass Filter Modality
  * @param  LSM303DLHC_FilterStruct: pointer to a LSM303DLHC_FilterConfigTypeDef structure 
  *         that contains the configuration setting for the LSM303DLHC.        
  * @retval None
  */
void LSM303DLHC_AccFilterConfig(const LSM303DLHCAcc_FilterConfigTypeDef *LSM303DLHC_FilterStruct) 
{
  uint8_t tmpreg;
  
  /* Read CTRL_REG2 register */
  LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG2_A, &tmpreg, 1);
  
  tmpreg &= 0x0C;
  
  /* Configure MEMS: mode, cutoff frquency, Filter status, Click, AOI1 and AOI2 */
  tmpreg |= (uint8_t) (LSM303DLHC_FilterStruct->HighPassFilter_Mode_Selection |\
                      LSM303DLHC_FilterStruct->HighPassFilter_CutOff_Frequency|\
                      LSM303DLHC_FilterStruct->HighPassFilter_AOI1|\
                      LSM303DLHC_FilterStruct->HighPassFilter_AOI2);                             
  
  /* Write value to ACC MEMS CTRL_REG2 regsister */
  LSM303DLHC_Write(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG2_A, &tmpreg);
}

/**
  * @brief  Enable or Disable High Pass Filter
  * @param  HighPassFilterState: new state of the High Pass Filter feature.
  *      This parameter can be: 
  *         @arg: LSM303DLHC_HighPassFilter_DISABLE 
  *         @arg: LSM303DLHC_HighPassFilter_ENABLE          
  * @retval None
  */
void LSM303DLHC_AccFilterCmd(const uint8_t HighPassFilterState)
 {
  uint8_t tmpreg;
  
  /* Read CTRL_REG2 register */
  LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG2_A, &tmpreg, 1);
                  
  tmpreg &= 0xF7;

  tmpreg |= HighPassFilterState;

  /* Write value to ACC MEMS CTRL_REG2 regsister */
  LSM303DLHC_Write(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG2_A, &tmpreg);
}

/**
  * @brief  Enable or Disable High Pass Filter on CLick
  * @param  HighPassFilterState: new state of the High Pass Filter feature.
  *      This parameter can be: 
  *         @arg: LSM303DLHC_HighPassFilter_DISABLE 
  *         @arg: LSM303DLHC_HighPassFilter_ENABLE          
  * @retval None
  */
void LSM303DLHC_AccFilterClickCmd(const uint8_t HighPassFilterClickState)
 {
  uint8_t tmpreg;
  
  /* Read CTRL_REG2 register */
  LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG2_A, &tmpreg, 1);
                  
  tmpreg &= 0xFB;

  tmpreg |= HighPassFilterClickState;

  /* Write value to ACC MEMS CTRL_REG2 regsister */
  LSM303DLHC_Write(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG2_A, &tmpreg);
}

/**
  * @brief Set LSM303DLHC Interrupt1 configuration
  * @param  LSM303DLHC_IT: specifies the LSM303DLHC interrupt source to be enabled.
  *           This parameter can be any combination of the following values: 
  *         @arg   LSM303DLHC_IT1_CLICK
  *         @arg   LSM303DLHC_IT1_AOI1
  *         @arg   LSM303DLHC_IT1_AOI2
  *         @arg   LSM303DLHC_IT1_DRY1
  *         @arg   LSM303DLHC_IT1_WTM
  *         @arg   LSM303DLHC_IT1_OVERRUN              
  * @param  NewState: new state of the selected LSM303DLHC interrupts.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void LSM303DLHC_AccIT1Config(const uint8_t LSM303DLHC_IT, const FunctionalState NewState)
{
  uint8_t tmpval = 0x00;
  
  /* Read CTRL_REG3 register */
  LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG3_A, &tmpval, 1 );
    
  tmpval &= ~LSM303DLHC_IT;
  
  if (NewState != DISABLE)
  {
    tmpval |= LSM303DLHC_IT;
  }
  else
  {
    /* Disable the selected interrupt */
    tmpval =~ LSM303DLHC_IT;
  }
      
  /* Write value to MEMS CTRL_REG3 register */
  LSM303DLHC_Write(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG3_A, &tmpval);  
}

/**
  * @brief Set LSM303DLHC Interrupt2 configuration
  * @param  LSM303DLHC_IT: specifies the LSM303DLHC interrupt source to be enabled.
  *           This parameter can be any combination of the following values: 
  *         @arg   LSM303DLHC_IT2_CLICK2
  *         @arg   LSM303DLHC_IT2_INT1
  *         @arg   LSM303DLHC_IT2_INT2
  *         @arg   LSM303DLHC_IT2_BOOT
  *         @arg   LSM303DLHC_IT2_ACT
  *         @arg   LSM303DLHC_IT2_HLACTIVE              
  * @param  NewState: new state of the selected LSM303DLHC interrupts.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void LSM303DLHC_AccIT2Config(const uint8_t LSM303DLHC_IT, const FunctionalState NewState)
{
  uint8_t tmpval = 0x00;
  
  /* Read CTRL_REG3 register */
  LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG6_A, &tmpval, 1);
    
  tmpval &= ~LSM303DLHC_IT;
  
  if (NewState != DISABLE)
  {
    tmpval |= LSM303DLHC_IT;
  }
  else
  {
    /* Disable the selected interrupt */
    tmpval =~ LSM303DLHC_IT;
  }
      
  /* Write value to MEMS CTRL_REG3 register */
  LSM303DLHC_Write(ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG6_A, &tmpval);  
}

/**
  * @brief  INT1 interrupt config
  * @param  ITCombination: Or or And combination
  *         ITAxes: axes to be enabled 
  *         NewState: Enable or Disable    
  * @retval None
  */
void LSM303DLHC_AccINT1InterruptConfig(const uint8_t ITCombination, const uint8_t ITAxes, const FunctionalState NewState )
{  
  uint8_t tmpval = ITCombination;
  
  /* Read INT1_CFR register */
  LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_INT1_CFG_A, &tmpval, 1);
  
  if (NewState != DISABLE)
  {
    tmpval |= ITAxes;
  }
  else
  {
    /* Disable the selected interrupt */
    tmpval =(~ITAxes)|ITCombination;
  }
      
  /* Write value to MEMS INT1_CFR register */
  LSM303DLHC_Write(ACC_I2C_ADDRESS, LSM303DLHC_INT1_CFG_A, &tmpval);  
}

/**
  * @brief  INT1 interrupt config
  * @param  ITCombination: Or or And combination
  *         ITAxes: axes to be enabled 
  *         NewState: Enable or Disable    
  * @retval None
  */
void LSM303DLHC_AccINT2InterruptConfig(const uint8_t ITCombination, const uint8_t ITAxes, const FunctionalState NewState )
{  
  uint8_t tmpval = ITCombination;
  
  /* Read INT2_CFR register */
  LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_INT2_CFG_A, &tmpval, 1);
  
  if (NewState != DISABLE)
  {
    tmpval |= ITAxes;
  }
  else
  {
    /* Disable the selected interrupt */
    tmpval =(~ITAxes)|ITCombination;
  }
      
  /* Write value to MEMS INT2_CFR register */
  LSM303DLHC_Write(ACC_I2C_ADDRESS, LSM303DLHC_INT2_CFG_A, &tmpval);  
}

/**
  * @brief  INT1 interrupt config
  * @param  ITCombination: Or or And combination
  *         ITAxes: axes to be enabled 
  *         NewState: Enable or Disable    
  * @retval None
  */
void LSM303DLHC_AccClickITConfig(const uint8_t ITClick, const FunctionalState NewState)
{  
  uint8_t tmpval;
  
  /* Read CLICK_CFR register */
  LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_CLICK_CFG_A, &tmpval, 1);
  
  if (NewState != DISABLE)
  {
    tmpval |= ITClick;
  }
  else
  {
    /* Disable the selected interrupt */
    tmpval =~ITClick;
  }
      
  /* Write value to MEMS CLICK_CFR register */
  LSM303DLHC_Write(ACC_I2C_ADDRESS, LSM303DLHC_CLICK_CFG_A, &tmpval);  
}

/**
  * @brief  Get status for Acc LSM303DLHC data
  * @param  None         
  * @retval Data status in a LSM303DLHC Data register
  */
uint8_t LSM303DLHC_AccGetDataStatus(void)
{
  uint8_t tmpreg;
  
  /* Read Mag STATUS register */
  LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_STATUS_REG_A, &tmpreg, 1);
                  
  return tmpreg;
}

int16_t LSM303DLHC_AccGetDataX(const LSM303DLHCAcc_InitTypeDef *LSM303DLHC_InitStruct) {
  uint8_t data_buf[2];

  LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_OUT_X_L_A, data_buf, 2);

  return LSM303DLHC_AccDataConversion(LSM303DLHC_InitStruct, data_buf);
}

int16_t LSM303DLHC_AccGetDataY(const LSM303DLHCAcc_InitTypeDef *LSM303DLHC_InitStruct) {
  uint8_t data_buf[2];

  LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_OUT_Y_L_A, data_buf, 2);

  return LSM303DLHC_AccDataConversion(LSM303DLHC_InitStruct, data_buf);
}

int16_t LSM303DLHC_AccGetDataZ(const LSM303DLHCAcc_InitTypeDef *LSM303DLHC_InitStruct) {
  uint8_t data_buf[2];

  LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_OUT_Z_L_A, data_buf, 2);

  return LSM303DLHC_AccDataConversion(LSM303DLHC_InitStruct, data_buf);
}

/**
  * @brief  Set LSM303DLHC Mag Initialization.
  * @param  LSM303DLHC_InitStruct: pointer to a LSM303DLHC_MagInitTypeDef structure 
  *         that contains the configuration setting for the LSM303DLHC.
  * @retval None
  */
void LSM303DLHC_MagInit(const LSM303DLHCMag_InitTypeDef *LSM303DLHC_InitStruct)
{  
  uint8_t cra_regm = 0x00, crb_regm = 0x00, mr_regm = 0x00;
  
  /* Configure MEMS: temp and Data rate */
  cra_regm |= (uint8_t) (LSM303DLHC_InitStruct->Temperature_Sensor | LSM303DLHC_InitStruct->MagOutput_DataRate);
    
  /* Configure MEMS: full Scale */
  crb_regm |= (uint8_t) (LSM303DLHC_InitStruct->MagFull_Scale);
      
  /* Configure MEMS: working mode */
  mr_regm |= (uint8_t) (LSM303DLHC_InitStruct->Working_Mode);
                    
  /* Write value to Mag MEMS CRA_REG regsister */
  LSM303DLHC_Write(MAG_I2C_ADDRESS, LSM303DLHC_CRA_REG_M, &cra_regm);
  
  /* Write value to Mag MEMS CRB_REG regsister */
  LSM303DLHC_Write(MAG_I2C_ADDRESS, LSM303DLHC_CRB_REG_M, &crb_regm);

  /* Write value to Mag MEMS MR_REG regsister */
  LSM303DLHC_Write(MAG_I2C_ADDRESS, LSM303DLHC_MR_REG_M, &mr_regm);
}

/**
  * @brief  Get status for Mag LSM303DLHC data
  * @param  None         
  * @retval Data status in a LSM303DLHC Data register
  */
uint8_t LSM303DLHC_MagGetDataStatus(void)
{
  uint8_t tmpreg;
  
  /* Read Mag STATUS register */
  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_SR_REG_M, &tmpreg, 1 );
                  
  return tmpreg;
}

float LSM303DLHC_MagGetDataX(const LSM303DLHCMag_InitTypeDef *LSM303DLHC_InitStruct) {
  uint8_t data_buf[2];

  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_X_H_M, data_buf, 2);

  return LSM303DLHC_MagDataConversion(LSM303DLHC_InitStruct, data_buf, 0);
}

float LSM303DLHC_MagGetDataY(const LSM303DLHCMag_InitTypeDef *LSM303DLHC_InitStruct) {
  uint8_t data_buf[2];

  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Y_H_M, data_buf, 2);

  return LSM303DLHC_MagDataConversion(LSM303DLHC_InitStruct, data_buf, 0);
}

float LSM303DLHC_MagGetDataZ(const LSM303DLHCMag_InitTypeDef *LSM303DLHC_InitStruct) {
  uint8_t data_buf[2];

  LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Z_H_M, data_buf, 2);

  return LSM303DLHC_MagDataConversion(LSM303DLHC_InitStruct, data_buf, 1);
}

/**
  * @brief  Writes one byte to the LSM303DLHC.  Adapted to use ioctrl and work with Linux
  *         I2C device driver for the Nvidia Jetson Nano.
  * @param  DeviceAddr : specifies the slave address to be programmed.
  * @param  RegAddr : specifies the LSM303DLHC register to be written.
  * @param  pBuffer : pointer to the buffer containing the data to be written to the LSM303DLHC.
  *                   It is assumed to be statically allocated memory.
  * @retval 1 on success, 0 on failure.
  */
uint16_t LSM303DLHC_Write(uint8_t DeviceAddr, const uint8_t RegAddr, const uint8_t* pBuffer)
{  
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
      printf("Writing LSM303DLHC failed\n");
      write_status = 0;
  }
  
  return write_status;
}

/**
  * @brief  Reads a block of data from the LSM303DLHC.  Adapted to use ioctrl and work with Linux
  *         I2C device driver for the Nvidia Jetson Nano.
  * @param  DeviceAddr : specifies the slave address to be programmed(ACC_I2C_ADDRESS or MAG_I2C_ADDRESS).
  * @param  RegAddr : specifies the LSM303DLHC internal address register to read from.
  * @param  pBuffer : pointer to the buffer that receives the data read from the LSM303DLH.
  * @param  NumByteToRead : number of bytes to read from the LSM303DLH ( NumByteToRead >1  only for the Mgnetometer readinf).
  * @retval 1 on success, 0 on failure.
  */
uint16_t LSM303DLHC_Read(uint8_t DeviceAddr, uint8_t RegAddr, uint8_t* pBuffer, const uint16_t NumByteToRead)
{    
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
      printf("Reading LSM303DLHC failed\n");
      read_status = 0;
  }
  
  return read_status;
}

/**
 * @brief  Local helper function used to convert low & high accelerometer bytes into 16-bit value in milli Gs
 * @param  LSM303DLHC_InitStruct: Pointer to accelerometer initialization structure
 * @param  data_reg_values: Pointer to base address of array holding low and high register values.
 * @retval 16-bit acceleration value
 */
static int16_t LSM303DLHC_AccDataConversion(const LSM303DLHCAcc_InitTypeDef *LSM303DLHC_InitStruct, const uint8_t *data_reg_values) {
  int16_t result = 0;

  if (LSM303DLHC_InitStruct->Endianness == LSM303DLHC_BLE_MSB) {
    result = (int16_t)((uint16_t)data_reg_values[0] << 8 | data_reg_values[1]) >> 4;
  }
  else {
    result = (int16_t)((uint16_t)data_reg_values[1] << 8 | data_reg_values[0]) >> 4;
  }

  switch (LSM303DLHC_InitStruct->AccFull_Scale)
  {
  case LSM303DLHC_FULLSCALE_2G:
    result *= LSM303DLHC_A_SENSITIVITY_2G;
    break;
  
  case LSM303DLHC_FULLSCALE_4G:
    result *= LSM303DLHC_A_SENSITIVITY_4G;
    break;

  case LSM303DLHC_FULLSCALE_8G:
    result *= LSM303DLHC_A_SENSITIVITY_8G;
    break;

  case LSM303DLHC_FULLSCALE_16G:
    result *= LSM303DLHC_A_SENSITIVITY_16G;
    break;

  default:
    result = 0;
    break;
  }

  return result;
}

/**
 * @brief  Local helper function used to convert low & high magnetometer bytes into float value in 
 * @param  LSM303DLHC_InitStruct: Pointer to magnetometer initialization structure
 * @param  data_reg_values: Pointer to base address of array holding low and high register values.
 * @param  isZData: A non-zero value if the data is for the Z axis
 * @retval single precision floating point magnetometer value
 */
static float LSM303DLHC_MagDataConversion(const LSM303DLHCMag_InitTypeDef *LSM303DLHC_InitStruct, const uint8_t *data_reg_values, const uint8_t isZData) {
  
  float result = (float)((uint16_t)data_reg_values[0] << 8 | data_reg_values[1]);

  switch (LSM303DLHC_InitStruct->MagFull_Scale)
  {
  case LSM303DLHC_FS_1_3_GA:
    result /= isZData ? LSM303DLHC_M_SENSITIVITY_Z_1_3Ga : LSM303DLHC_M_SENSITIVITY_XY_1_3Ga;
    break;
  
  case LSM303DLHC_FS_1_9_GA:
    result /= isZData ? LSM303DLHC_M_SENSITIVITY_Z_1_9Ga : LSM303DLHC_M_SENSITIVITY_XY_1_9Ga;
    break;

  case LSM303DLHC_FS_2_5_GA:
    result /= isZData ? LSM303DLHC_M_SENSITIVITY_Z_2_5Ga : LSM303DLHC_M_SENSITIVITY_XY_2_5Ga;
    break;

  case LSM303DLHC_FS_4_0_GA:
    result /= isZData ? LSM303DLHC_M_SENSITIVITY_Z_4Ga : LSM303DLHC_M_SENSITIVITY_XY_4Ga;
    break;

  case LSM303DLHC_FS_4_7_GA:
    result /= isZData ? LSM303DLHC_M_SENSITIVITY_Z_4_7Ga : LSM303DLHC_M_SENSITIVITY_XY_4_7Ga;
    break;

  case LSM303DLHC_FS_5_6_GA:
    result /= isZData ? LSM303DLHC_M_SENSITIVITY_Z_5_6Ga : LSM303DLHC_M_SENSITIVITY_XY_5_6Ga;
    break;

  case LSM303DLHC_FS_8_1_GA:
    result /= isZData ? LSM303DLHC_M_SENSITIVITY_Z_8_1Ga : LSM303DLHC_M_SENSITIVITY_XY_8_1Ga;
    break;

  default:
    result = 0;
    break;
  }

  return result;
}
