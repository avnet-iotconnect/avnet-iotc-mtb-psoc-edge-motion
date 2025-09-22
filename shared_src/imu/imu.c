/******************************************************************************
* File Name:   imu.c
*
* Description: This file implements the interface with the BMI270 motion sensor.
*              The configurations of the motion sensor are specified in the
*              config.h file.
*
* Related Document: See README.md
*
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
#include "cybsp.h"
#include "mtb_bmi270.h"
#include "bmi2.h"

#include "config.h"
#include "imu.h"
#include "human_activity.h"

/*******************************************************************************
* Macros
*******************************************************************************/
#if IMU_SAMPLE_RANGE == BMI2_ACC_RANGE_2G
#define IMU_DIVIDE                  0x4000
#elif IMU_SAMPLE_RANGE == BMI2_ACC_RANGE_4G
#define IMU_DIVIDE                  0x2000
#elif IMU_SAMPLE_RANGE == BMI2_ACC_RANGE_8G
#define IMU_DIVIDE                  0x1000
#elif IMU_SAMPLE_RANGE == BMI2_ACC_RANGE_16G
#define IMU_DIVIDE                  0x800
#endif

#define IMU_NUM_AXIS                (3U)
#define IMU_INTERRUPT_PRIORITY      (2U)
#define IMU_INTR_MASK               (0x00000001UL << CYBSP_I2C_INT1_PORT_NUM)
#define MASKED_TRUE                 (1U)

/*******************************************************************************
* Global Variables
*******************************************************************************/
static mtb_hal_i2c_t CYBSP_I2C_CONTROLLER_0_hal_obj;
static cy_stc_scb_i2c_context_t CYBSP_I2C_CONTROLLER_0_context;

/* Instance of BMI270 sensor structure */
struct bmi2_dev dev;

/* Number of bytes of FIFO data */
static uint8_t fifo_data[BMI2_FIFO_RAW_DATA_BUFFER_SIZE] = { 0 };

/* Array of accelerometer frames */
static struct bmi2_sens_axes_data fifo_accel_data[BMI2_FIFO_ACCEL_FRAME_COUNT] = { { 0 } };

/* Initialize FIFO frame structure. */
struct bmi2_fifo_frame fifoframe = { 0 };

/* Flag to check if the data from IMU is ready for processing. */
static volatile bool imu_flag;

/*ipc*/
extern uint8_t data_refresh_flag;
extern uint32_t motion_data;

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
static void imu_interrupt_handler(void);
static cy_rslt_t imu_fifo_init(void);

/*******************************************************************************
* Function Definitions
*******************************************************************************/
/*******************************************************************************
* Function Name: imu_init
********************************************************************************
* Summary:
*  A function used to initialize the IMU (BMI270). Starts a timer that
*  triggers an interrupt at at the sample rate specified in config.h.
*
* Parameters:
*  None
*
* Return:
*  The status of the initialization.
*
*******************************************************************************/
cy_rslt_t imu_init(void)
{
    cy_rslt_t result;

    /* Initialize the I2C master interface for BMI270 motion sensor */
    result = Cy_SCB_I2C_Init(CYBSP_I2C_CONTROLLER_0_HW,
                             &CYBSP_I2C_CONTROLLER_0_config,
                             &CYBSP_I2C_CONTROLLER_0_context);

    if(CY_RSLT_SUCCESS != result)
    {
        printf(" Error : I2C initialization failed !!\r\n");
        CY_ASSERT(0);
    }

    Cy_SCB_I2C_Enable(CYBSP_I2C_CONTROLLER_0_HW);

    /* Configure the I2C master interface with the desired clock frequency */
    result = mtb_hal_i2c_setup(&CYBSP_I2C_CONTROLLER_0_hal_obj,
                               &CYBSP_I2C_CONTROLLER_0_hal_config,
                               &CYBSP_I2C_CONTROLLER_0_context,
                              NULL);

    if(CY_RSLT_SUCCESS != result)
    {
        printf(" Error : I2C setup failed !!\r\n");
        CY_ASSERT(0);
    }

    /* Initialize the BMI270 motion sensor */
    result = mtb_bmi270_init(&dev, &CYBSP_I2C_CONTROLLER_0_hal_obj);
    if(CY_RSLT_SUCCESS != result)
    {
        printf(" Error : IMU sensor init failed !!\r\n");
        CY_ASSERT(0);
    }

    result = mtb_bmi270_config(&dev);
    if(CY_RSLT_SUCCESS != result)
    {
        printf(" Error : IMU sensor config failed !!\r\n");
        CY_ASSERT(0);
    }

    /* Get the default IMU configuration and update it based on config.h */
    struct bmi2_sens_config config = {0};
    result = bmi2_get_sensor_config(&config, 1, &dev);
    if (BMI2_OK != result)
    {
        printf(" Error: Failed to get sensor config\n");
        CY_ASSERT(0);
    }

    config.type = BMI2_ACCEL;
    config.cfg.acc.odr = IMU_SAMPLE_RATE;
    config.cfg.acc.range = IMU_SAMPLE_RANGE;
    config.cfg.acc.bwp = BMI2_ACC_OSR4_AVG1;
    config.cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
    result = bmi2_set_sensor_config(&config, 1, &dev);
    if (BMI2_OK != result)
    {
        printf(" Error: Failed to set sensor config\n");
        CY_ASSERT(0);
    }

    result = imu_fifo_init();
    if (BMI2_OK != result)
    {
        CY_ASSERT(0);
    }

    imu_flag = false;

    return result;
}

/*******************************************************************************
* Function Name: imu_fifo_init
********************************************************************************
* Summary:
*  A function used to initialize the FIFO related configurations for the
*  BMI270 IMU.
*
* Parameters:
*  None
*
* Return:
*  The status of the initialization.
*
*******************************************************************************/
static cy_rslt_t imu_fifo_init(void)
{
    cy_rslt_t result;

    /* Accel and gyro sensor are listed in array. */
    uint8_t sensor_sel[] = { BMI2_ACCEL };

    /* Interrupt config structure for IMU interrupt */
    static cy_stc_sysint_t intrCfg =
    {
        CYBSP_I2C_INT1_IRQ,       /* Interrupt source */
        IMU_INTERRUPT_PRIORITY    /* Interrupt priority */
    };

    /* Interrupt pin configuration */
    static struct bmi2_int_pin_config pin_config = { 0 };


    /* Accelerometer must be enabled after setting configurations */
    result = bmi270_sensor_enable(sensor_sel, sizeof(sensor_sel), &dev);
    if (BMI2_OK != result)
    {
        printf(" Error: Failed to enable sensor\n");
        return result;
    }

    /* Before setting FIFO, disable the advance power save mode. */
    result = bmi2_set_adv_power_save(BMI2_DISABLE, &dev);
    if (BMI2_OK != result)
    {
        printf(" Error: Failed to disable advance power save mode\n");
        return result;
    }

    /* Initially disable all configurations in fifo. */
    result = bmi2_set_fifo_config(BMI2_FIFO_ALL_EN, BMI2_DISABLE, &dev);
    if (BMI2_OK != result)
    {
        printf(" Error: Failed to disable all FIFO configurations\n");
        return result;
    }

    /* Mapping the buffer to store the fifo data. */
    fifoframe.data = fifo_data;

    /* Length of FIFO frame. */
    fifoframe.length = BMI2_FIFO_RAW_DATA_BUFFER_SIZE;

    /* Set FIFO configuration by enabling accelerometer */
    result = bmi2_set_fifo_config(BMI2_FIFO_ACC_EN, BMI2_ENABLE, &dev);
    if (BMI2_OK != result)
    {
        printf(" Error: Failed to enable FIFO configuration\n");
        return result;
    }

    /* To enable headerless mode, disable the header. */
    result = bmi2_set_fifo_config(BMI2_FIFO_HEADER_EN, BMI2_DISABLE, &dev);
    if (BMI2_OK != result)
    {
        printf(" Error: Failed to disable FIFO header\n");
        return result;
    }

    /* Interrupt pin configuration */
    pin_config.pin_type = BMI2_INT1;
    pin_config.pin_cfg[0].input_en = BMI2_INT_INPUT_DISABLE;
    pin_config.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE;
    pin_config.pin_cfg[0].lvl = BMI2_INT_ACTIVE_LOW;
    pin_config.pin_cfg[0].od = BMI2_INT_PUSH_PULL;
    pin_config.int_latch = BMI2_INT_NON_LATCH;

    /* Set Hardware interrupt pin configuration */
    result = bmi2_set_int_pin_config(&pin_config, &dev);
    if (BMI2_OK != result)
    {
        printf(" Error: Failed to set interrupt pin configuration\n");
        return result;
    }

    /* FIFO water-mark interrupt is enabled. */
    fifoframe.data_int_map = BMI2_FWM_INT;

    /* Map water-mark interrupt to the required interrupt pin. */
    result = bmi2_map_data_int(fifoframe.data_int_map, BMI2_INT1, &dev);
    if (BMI2_OK != result)
    {
        printf(" Error: Failed to map data interrupt\n");
        return result;
    }

    /* Flush the FIFO before setting the watermark level. */
    result = bmi2_set_command_register(BMI2_FIFO_FLUSH_CMD, &dev);
    if (BMI2_OK != result)
    {
        printf(" Error: Failed to flush FIFO\n");
        return result;
    }

    /* Set the watermark level. */
    fifoframe.wm_lvl = BMI2_FIFO_WATERMARK_LEVEL;
    fifoframe.length = BMI2_FIFO_RAW_DATA_BUFFER_SIZE;

    /* Set the water-mark level if water-mark interrupt is mapped. */
    result = bmi2_set_fifo_wm(fifoframe.wm_lvl, &dev);
    if (BMI2_OK != result)
    {
        printf(" Error: Failed to set FIFO watermark level\n");
        return result;
    }

    /* Clear GPIO and NVIC interrupt before initializing to avoid false
     * triggering.
     */
    Cy_GPIO_ClearInterrupt(CYBSP_I2C_INT1_PORT, CYBSP_I2C_INT1_PIN);
    NVIC_ClearPendingIRQ(CYBSP_I2C_INT1_IRQ);

    /* Initialize the interrupt and register interrupt callback */
    Cy_SysInt_Init(&intrCfg, &imu_interrupt_handler);

    /* Enable the interrupt in the NVIC */
    NVIC_EnableIRQ(intrCfg.intrSrc);

    return result;
}


/*******************************************************************************
* Function Name: imu_interrupt_handler
********************************************************************************
* Summary:
*  Handler for the IMU interrupt. Sets the flag for the processing in the main
*  loop.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void imu_interrupt_handler(void)
{
    /* Get interrupt cause */
    uint32_t intrSrc = Cy_GPIO_GetInterruptCause0();

    /* Check if the interrupt was from the user button's port and pin */
    if((IMU_INTR_MASK == (intrSrc & IMU_INTR_MASK)) &&
       (MASKED_TRUE == Cy_GPIO_GetInterruptStatusMasked(CYBSP_I2C_INT1_PORT,
               CYBSP_I2C_INT1_PIN)))
    {
        /* Clear the interrupt */
        Cy_GPIO_ClearInterrupt(CYBSP_I2C_INT1_PORT, CYBSP_I2C_INT1_PIN);
        NVIC_ClearPendingIRQ(CYBSP_I2C_INT1_IRQ);

        /* The flag is set to true, meaning data is ready to be processed */
        imu_flag = true;
    }
}


/*******************************************************************************
* Function Name: imu_data_process
********************************************************************************
* Summary:
*  This function checks for the IMU interrupt status, reads the FIFO data from
*  the IMU, feeds the data to the DEEPCRAFT pre-processor, and prints the
*  processed results.
*
* Parameters:
*  None
*
* Return:
*  CY_RSLT_SUCCESS if successful, otherwise an error code indicating failure.
*******************************************************************************/
cy_rslt_t imu_data_process(void)
{
    /* Variables required to get the IMU data and process the results. */
    cy_rslt_t result = CY_RSLT_SUCCESS;
    float label_scores[IMAI_DATA_OUT_COUNT];
    char *label_text[] = IMAI_DATA_OUT_SYMBOLS;
    int16_t best_label;
    float max_score;
    uint16_t fifo_length = 0;
    uint16_t accel_frame_length;
    uint16_t int_status = 0;
    uint16_t watermark = 0;
    uint16_t index = 0;

    /* Check if IMU Data is ready to be processed */
    if (!imu_flag)
    {
        result = IMU_DATA_NOT_READY;
        return result;
    }

    /* Reset the flag to false, indicating that the data is being processed */
    imu_flag = false;

    /* Read FIFO data on interrupt. */
    result = bmi2_get_int_status(&int_status, &dev);
    if (BMI2_OK != result)
    {
        printf(" Error: Failed to get interrupt status\n");
        return result;
    }

    /* To check the status of FIFO watermark interrupt. */
    if (int_status & BMI2_FWM_INT_STATUS_MASK)
    {
        /* Move cursor home */
        //printf("\033[H\n");

#ifdef COMPONENT_CM33
        //printf("DEEPCRAFT Studio Deploy Motion Example - CM33\r\n\n");
#else
        //printf("DEEPCRAFT Studio Deploy Motion Example - CM55\r\n\n");
#endif /* COMPONENT_CM33 */

        /* Turn user led on.*/
        Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN, CYBSP_LED_STATE_ON);

        result = bmi2_get_fifo_wm(&watermark, &dev);
        if (BMI2_OK != result)
        {
            printf(" Error: Failed to get FIFO watermark level\n");
            return result;
        }

        result = bmi2_get_fifo_length(&fifo_length, &dev);
        if (BMI2_OK != result)
        {
            printf(" Error: Failed to get FIFO length\n");
            return result;
        }

        /* Updating FIFO length to be read based on available length and dummy byte updation */
        fifoframe.length = fifo_length + dev.dummy_byte;

        /* Read FIFO data. */
        result = bmi2_read_fifo_data(&fifoframe, &dev);
        if (BMI2_OK != result)
        {
            printf(" Error: Failed to read FIFO data\n");
            return result;
        }

        /* Read FIFO data on interrupt. */
        result = bmi2_get_int_status(&int_status, &dev);
        if (BMI2_OK != result)
        {
            printf(" Error: Failed to get interrupt status\n");
            return result;
        }

        /* Parse the FIFO data to extract accelerometer data from the FIFO buffer. */
        accel_frame_length = BMI2_FIFO_ACCEL_FRAME_COUNT;
        (void)bmi2_extract_accel(fifo_accel_data, &accel_frame_length, &fifoframe, &dev);

        if (BMI2_FIFO_ACCEL_FRAME_COUNT != accel_frame_length)
        {
            printf("Error: Expected number of Accelerometer frames not received\n");
            result = IMU_FIFO_INSUFFICIENT_DATA;
            return result;
        }

        for (index = 0; index < accel_frame_length; index++)
        {
            /* Store the IMU values in a buffer. Change the orientation to
             * match training data.
             */
            float imu_buffer[IMU_NUM_AXIS] =
            {
                fifo_accel_data[index].x / (float)IMU_DIVIDE,
                fifo_accel_data[index].y / (float)IMU_DIVIDE,
                fifo_accel_data[index].z / (float)IMU_DIVIDE,
            };

            /* Feed the DEEPCRAFT pre-processor */
            result = IMAI_enqueue(imu_buffer);
            CY_ASSERT(IMAI_RET_SUCCESS == result);

            /* Reset the scores and best label values. */
            best_label = 0;
            max_score = -1000.0f;

            /* Check if there is any model output */
            switch(IMAI_dequeue(label_scores))
            {
                /* On success, print the labels with their scores */
                case IMAI_RET_SUCCESS:
                {
                    for(int i = 0; i < IMAI_DATA_OUT_COUNT; i++)
                    {
                        //printf("label: %-10s: score: %f\r\n", label_text[i], label_scores[i]);
                        if (label_scores[i] > max_score)
                        {
                            max_score = label_scores[i];
                            best_label = i;
                        }
                    }
                    //testing code
                    if (strcmp(label_text[best_label], "standing") == 0) {
						motion_data = 0x1111;
					} else if (strcmp(label_text[best_label], "sitting") == 0) {
						motion_data = 0x2222;
					} else {
						motion_data = 0x0;
					}
					data_refresh_flag = 1;
					//end testing code
					
                    //printf("\r\n");
                    //printf("Output: %-30s\r\n", label_text[best_label]);
                    break;
                }

                /* No new output, continue with sampling */
                case IMAI_RET_NODATA:
                {
                    break;
                }

                /* Abort on error */
                case IMAI_RET_ERROR:
                {
                    printf("Error: IMAI_dequeue failed !\r\n");
                    CY_ASSERT(0);
                    break;
                }
            }
        }

        /* Turn user led off.*/
        Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN, CYBSP_LED_STATE_OFF);
    }

    return result;
}


/* [] END OF FILE */
