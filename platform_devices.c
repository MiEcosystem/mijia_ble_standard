/**
 ****************************************************************************************
 *
 * @file platform_devices.c
 *
 * @brief Configuration of devices connected to board
 *
 * Copyright (C) 2016-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#include <stdio.h>
#include <stdbool.h>
#include "ad_gpadc.h"

#ifdef __cplusplus
extern "C" {
#endif

#if (dg_configGPADC_ADAPTER == 1)

/*
 * Define sources connected to GPADC
 */


const ad_gpadc_driver_conf_t battery_level_driver = {
        .clock                  = HW_GPADC_CLOCK_INTERNAL,
        .input_mode             = HW_GPADC_INPUT_MODE_SINGLE_ENDED,
        .input                  = HW_GPADC_INPUT_SE_VBAT,
        .temp_sensor            = HW_GPADC_NO_TEMP_SENSOR,
        .sample_time            = 15,
        .chopping               = true,
        .oversampling           = HW_GPADC_OVERSAMPLING_4_SAMPLES,
        .input_attenuator       = HW_GPADC_INPUT_VOLTAGE_UP_TO_1V2,
};

const ad_gpadc_controller_conf_t BATTERY_LEVEL = {
        HW_GPADC_1,
        NULL,
        &battery_level_driver
};


#endif /* dg_configGPADC_ADAPTER */
#ifdef __cplusplus
}
#endif
