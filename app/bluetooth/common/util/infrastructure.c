/***************************************************************************//**
 * @file
 * @brief infrastructure.c
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#include <stdint.h>
#include <stdio.h>
#include "infrastructure.h"

#if defined(DEBUG_APP)
void appAssertDebug(uint32_t val, const char* file, int line)
{
  while (1) {
//    printf("F: %s, L: %u; E: %u\n", file, line, (unsigned int)(val));
  }
}
#endif // DEBUG_APP

#if defined(LOG_APP)
void appLog(char* msg)
{
//  printf("%s", msg);
}
#endif // LOG_APP
