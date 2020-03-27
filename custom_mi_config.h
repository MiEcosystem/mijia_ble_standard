/***************************************************************************//**
 * @file
 * @brief custom_mi_config.h
 *
 ******************************************************************************/
#ifndef CUSTOM_MI_CONFIG_H
#define CUSTOM_MI_CONFIG_H

/**
 * @note Device firmware version. It'll be filled in Version character appended to
 * mijia ble libs version.
 */
#define DEVELOPER_VERSION       0001


/**
 * @note Product identification got from xiaomi IoT developer platform.
 */
#define PRODUCT_ID              156   // xiaomi BLE devboard


/**
 * @note Device access method : BLE or Mesh.
 */
#define MI_BLE_ENABLED


/**
 * @note Device side has RESET button or not.
 */
#define HAVE_RESET_BUTTON      0
#define HAVE_CONFIRM_BUTTON    0

/**
 * @note Process mi scheduler in main loop (non-interrupt context).
 */
#define MI_SCHD_PROCESS_IN_MAIN_LOOP    1


#define MAX_ATT_MTU            247

/* DEBUG */
#define DEBUG_MIBLE            1
#define TIME_PROFILE           0
#define RXFER_VERBOSE          0

#endif
