/**
 ****************************************************************************************
 *
 * @file custom_socf_battery_profile.h
 *
 * @brief a battery profile data for State Of Charge
 *
 * Copyright (C) 2016-2017 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef CUSTOM_SOCF_BATTERY_PROFILE_H_
#define CUSTOM_SOCF_BATTERY_PROFILE_H_

#define VOL2SOC_LUT_SIZE 11
/* Battery profile data for 190mA battery PH12YT */
#define SOCF_BATT_LOW_CURRENT 1000
#define SOCF_BATT_HIGH_CURRENT 50000
#define SOCF_BATTERY_CAPACITANCE 190
#define SOCF_CHARGING_CURRENT 120
static const int16_t vol_dis_low_0[VOL2SOC_LUT_SIZE] = {
        3691, 3730, 3781, 3828, 3864, 3895, 3939, 3989, 4040, 4097, 4183
};
static const int16_t vol_dis_high_0[VOL2SOC_LUT_SIZE] = {
        3633, 3676, 3730, 3779, 3818, 3850, 3891, 3939, 3988, 4042, 4123
};
static const int16_t vol_chg_0_0 = 3767; //120mA charging

#endif /* CUSTOM_SOCF_BATTERY_PROFILE_H_ */
