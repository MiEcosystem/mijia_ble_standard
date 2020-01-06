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
 * @note To support Secure Auth procedure MUST have mijia secure chip (MSC).
 * If device use Standard auth or Mesh Auth, it should be 0.
 *      NONE   : 0
 *      MJSC   : 1
 *      MJA1   : 2
 */
#define HAVE_MSC               0


/**
 * @note Device side has RESET button or not.
 */
#define HAVE_RESET_BUTTON      0


/**
 * @note Which OOB methods device used in authentication procedure.
 *       OOB_USE_NUMPAD_INPUT  : device has a numeric keypad to enter the pairing code.
 *       OOB_USE_QR_CODE_OUT   : device provided with a QR code label.
 *       OOB_USE_DISPLAT_OUT   : device displayed a six digit number.
 */
#define OOB_USE_NUMPAD_INPUT   0
#define OOB_USE_QR_CODE_OUT    0
#define OOB_USE_DISPLAT_OUT    0


/**
 * @note Process mi scheduler in main loop (non-interrupt context).
 */
#define MI_SCHD_PROCESS_IN_MAIN_LOOP    0

/* DEBUG */
#ifndef DEBUG_MIBLE
#define DEBUG_MIBLE            0
#define TIME_PROFILE           0
#endif


#endif
