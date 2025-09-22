/*******************************************************************************
* File Name        : mtb_bmi270.c
*
* Description      : This file contains the functions for interacting with the
*                    motion sensor.
*
* Related Document : See README.md
*
*******************************************************************************
* Copyright 2023-2025, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/


/*******************************************************************************
* Header Files
*******************************************************************************/

#include "mtb_bmi270.h"

/*******************************************************************************
* Macros
*******************************************************************************/
#define SELECT_SINGLE_SENSOR     (1u)
#define SELECT_BOTH_SENSOR       (2u)
#define I2C_TIMEOUT_MS           (1u)
#define I2C_SEND_DATA_SIZE       (1u)
/*******************************************************************************
* Global variables
*******************************************************************************/

static uint8_t dev_addr = BMI2_I2C_PRIM_ADDR;

static mtb_hal_i2c_t* _bmi270_i2c = NULL;

/*******************************************************************************
* Function name: mtb_bmi2_i2c_read
********************************************************************************
* Summary:
* This internal function reads data from the register of the BMI270 sensor
*
* Parameters:
*  reg_addr    8bit register address of the sensor
*  reg_data    Data from the specified address
*  len         Length of the reg_data array
*  intf_ptr    Void pointer that can enable the linking of descriptors for
*              interface related callback.
*
* Return:
*  BMI2_INTF_RETURN_TYPE     Status of execution
*
*******************************************************************************/
static BMI2_INTF_RETURN_TYPE mtb_bmi2_i2c_read(uint8_t reg_addr,
                                                uint8_t *reg_data,
                                                uint32_t len, void *intf_ptr)
{
    uint8_t device_addr = *(uint8_t*)intf_ptr;

    cy_rslt_t status = mtb_hal_i2c_controller_write(_bmi270_i2c,
                                                    device_addr,
                                                    &reg_addr,
                                                    I2C_SEND_DATA_SIZE,
                                                    I2C_TIMEOUT_MS,
                                                    false);
    if (status == CY_RSLT_SUCCESS)
    {
        status = mtb_hal_i2c_controller_read(_bmi270_i2c,
                                            device_addr,
                                            reg_data,
                                            len,
                                            I2C_TIMEOUT_MS,
                                            true);
    }

    return (BMI2_INTF_RETURN_TYPE)status;

}

/*******************************************************************************
* Function name: mtb_bmi2_i2c_write
********************************************************************************
* Summary:
* This internal function writes data to register of the BMI270 sensor
*
* Parameters:
*  reg_addr    8bit register address of the sensor
*  reg_data    Data from the specified address
*  len         Length of the reg_data array
*  intf_ptr    Void pointer that can enable the linking of descriptors for
*              interface related callback.
*
* Return:
*  BMI2_INTF_RETURN_TYPE     Status of execution
*
*******************************************************************************/
static BMI2_INTF_RETURN_TYPE mtb_bmi2_i2c_write(uint8_t reg_addr,
                                                const uint8_t *reg_data,
                                                uint32_t len, 
                                                void *intf_ptr)
{
    uint8_t device_addr = *(uint8_t*)intf_ptr;

    cy_rslt_t status = mtb_hal_i2c_controller_write(_bmi270_i2c,
                                                    device_addr,
                                                    &reg_addr,
                                                    I2C_SEND_DATA_SIZE,
                                                    I2C_TIMEOUT_MS,
                                                    false);

    if (status == CY_RSLT_SUCCESS)
    {
        while (len > 0)
        {
            status = Cy_SCB_I2C_MasterWriteByte(_bmi270_i2c->base,
                                                *reg_data,
                                                I2C_TIMEOUT_MS,
                                                _bmi270_i2c->context);

            if (status != CY_SCB_I2C_SUCCESS)
            {
                break;
            }
            --len;
            ++reg_data;
        }
        /* SCB in I2C mode is very time sensitive. In practice we have to 
         * request STOP after each block, otherwise it may break the 
         * transmission */
        Cy_SCB_I2C_MasterSendStop(_bmi270_i2c->base,
                                    I2C_TIMEOUT_MS,
                                    _bmi270_i2c->context);
    }

    return (BMI2_INTF_RETURN_TYPE)status;

}

/*******************************************************************************
* Function name: mtb_bmi2_delay_us
********************************************************************************
* Summary:
* This internal function maps platform dependent delay function
*
* Parameters:
*  period    The time period in microseconds
*  intf_ptr  Void pointer that can enable the linking of descriptors for
*            interface related callback.
*
* Return:
*  void
*
*******************************************************************************/
static void mtb_bmi2_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;

    mtb_hal_system_delay_us(period);
}

/*******************************************************************************
* Function name: mtb_bmi270_init
********************************************************************************
* Summary:
* This function initializes the I2C instance, configures the BMI270, and sets
* platform-dependent function pointers.
*
* Parameters:
*  dev           Structure instance of bmi2_dev
*  i2c_instance  Instance of i2c peripheral
*
* Return:
* cy_rslt_t Status of execution
*
*******************************************************************************/
cy_rslt_t mtb_bmi270_init(struct bmi2_dev *dev, mtb_hal_i2c_t* i2c_instance)
{
    cy_rslt_t rslt;
    struct bmi2_sens_config config = {0};

    CY_ASSERT(NULL != i2c_instance);
    CY_ASSERT(NULL != dev);

    _bmi270_i2c  = i2c_instance;

    dev->intf = BMI2_I2C_INTF;
    dev->read = mtb_bmi2_i2c_read;
    dev->write = mtb_bmi2_i2c_write;
    dev->delay_us = mtb_bmi2_delay_us;
    dev->intf_ptr = &dev_addr;
    dev->read_write_len = I2C_SENSOR_READ_WRITE_LEN;
    dev->config_file_ptr = NULL;

    bmi270_init(dev);

    rslt = bmi270_get_sensor_config(&config, SELECT_SINGLE_SENSOR, dev);

    return rslt;
}

/*******************************************************************************
* Function name: mtb_bmi270_config
********************************************************************************
* Summary:
* This function configures the accelerometer and gyroscope with a 100 Hz
* output data rate
*
* Parameters:
*  dev      Structure instance of bmi2_dev
*
* Return:
* cy_rslt_t Status of execution
*
*******************************************************************************/
cy_rslt_t mtb_bmi270_config(struct bmi2_dev *dev)
{
    cy_rslt_t rslt;
    uint8_t sens_list[SELECT_BOTH_SENSOR] = {BMI2_ACCEL, BMI2_GYRO};
    struct bmi2_sens_config config = {0};

    rslt = bmi2_get_sensor_config(&config, SELECT_SINGLE_SENSOR, dev);
    config.type = BMI2_ACCEL;
    config.cfg.acc.odr = BMI2_ACC_ODR_100HZ;
    config.cfg.acc.range = BMI2_ACC_RANGE_2G;
    config.cfg.acc.bwp = BMI2_ACC_OSR4_AVG1;
    config.cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

    rslt = bmi2_set_sensor_config(&config, SELECT_SINGLE_SENSOR, dev);
    rslt = bmi2_get_sensor_config(&config, SELECT_SINGLE_SENSOR, dev);

    config.type = BMI2_GYRO;
    config.cfg.gyr.odr = BMI2_GYR_ODR_100HZ;
    config.cfg.gyr.range = BMI2_GYR_RANGE_2000;
    config.cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;
    config.cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE;
    config.cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

    rslt = bmi2_set_sensor_config(&config, SELECT_SINGLE_SENSOR, dev);
    rslt = bmi2_get_sensor_config(&config, SELECT_SINGLE_SENSOR, dev);

    rslt = bmi2_sensor_enable(sens_list, SELECT_BOTH_SENSOR, dev);

    return rslt;
}

/*******************************************************************************
* Function name: mtb_bmi270_get_sensor_data
********************************************************************************
* Summary:
* This function gets the sensor data
*
* Parameters:
*  dev            Structure instance of bmi2_dev
*  sensor_data    Structure instance of bmi2_sens_data
*
* Return:
* cy_rslt_t     Status of execution
*
*******************************************************************************/
cy_rslt_t mtb_bmi270_get_sensor_data(struct bmi2_dev *dev,
                                    struct bmi2_sens_data *sensor_data)
{
    cy_rslt_t rslt;
    rslt = bmi2_get_sensor_data(sensor_data, dev);
    return rslt;
}

/* [] END OF FILE */
