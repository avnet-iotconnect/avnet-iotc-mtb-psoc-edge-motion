/******************************************************************************
* File Name:   config.h
*
* Description: This file contains the configuration for running the IMU based
*              model.
*
* Related Document: See README.md
*
********************************************************************************
* Copyright 2024-2025, Cypress Semiconductor Corporation (an Infineon company) or
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
#ifndef CONFIG_H
#define CONFIG_H

#include "bmi2_defs.h"
/******************************************************************************
 * Constants
 *****************************************************************************/

/* Set IMU_SAMPLE_RATE to one of the following
 * BMI2_ACC_ODR_400HZ
 * BMI2_ACC_ODR_200HZ
 * BMI2_ACC_ODR_100HZ
 * BMI2_ACC_ODR_50HZ */
#define IMU_SAMPLE_RATE BMI2_ACC_ODR_50HZ

/* Set IMU_SAMPLE_RANGE to one of the following
 * BMI2_ACC_RANGE_16G
 * BMI2_ACC_RANGE_8G
 * BMI2_ACC_RANGE_4G
 * BMI2_ACC_RANGE_2G */
#define IMU_SAMPLE_RANGE BMI2_ACC_RANGE_8G

/* Buffer size allocated to store raw FIFO data */
#define BMI2_FIFO_RAW_DATA_BUFFER_SIZE  UINT16_C(100)

/* Accelerometer Frame Count for each interrupt
 *  Calculation:
 *    IMU Sample Rate =  50 Hz
 *    Desired IMU interrupt interval = 200 ms = 0.2 sec
 *    Accelerometer Frame Count = (0.2 sec * 50 Hz) = 10 frames
 */
#define BMI2_FIFO_ACCEL_FRAME_COUNT     UINT8_C(10)

/* Watermark level for the IMU HW FIFO
 *  Calculation:
 *    Accelerometer Frame Count = 10 frames
 *    Length of 1 Accelerometer Frame = (3 * 2 bytes) = 6 bytes
 *    FIFO Watermark Level = (10 frames * 6 bytes) = 60 bytes
 */
#define BMI2_FIFO_WATERMARK_LEVEL       UINT16_C(60)

#endif /* CONFIG_H */

/* [] END OF FILE */
