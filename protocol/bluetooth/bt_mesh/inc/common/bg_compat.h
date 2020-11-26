/***************************************************************************//**
 * @brief Compatibility macros for bgcommon
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
#ifndef COMMON_INCLUDE_BG_COMPAT_H_
#define COMMON_INCLUDE_BG_COMPAT_H_
#include <stdint.h>

/*
   IAR requires that macros are given before variable definition.
   For example:
   WEAK void func() {}
   PACKSTRUCT(struct foo { int bar; });
   ALIGNED(256) struct foo bar;
 */

#if defined(__IAR_SYSTEMS_ICC__)
/* IAR ICC */
  #define __STRINGIFY(a) #a
  #ifndef ALIGNED
  #define ALIGNED(alignment)    _Pragma(__STRINGIFY(data_alignment = alignment))
  #endif
  #ifndef PACKSTRUCT
  #define PACKSTRUCT(decl)      __packed decl
  #endif
  #define WEAK                  __weak
  #define KEEP_SYMBOL           __root
#ifdef NDEBUG
  #define OPTIMIZE_SPEED        _Pragma(__STRINGIFY(optimize = speed high))
#else
/* Increasing optimization is not allowed */
  #define OPTIMIZE_SPEED
#endif
#elif defined(__GNUC__)
/* GNU GCC */
  #ifndef ALIGNED
  #define ALIGNED(alignment)    __attribute__((aligned(alignment)))
  #endif
  #ifndef PACKSTRUCT
  #if defined(_WIN32)
    #define PACKSTRUCT(decl)    decl __attribute__((packed, gcc_struct))
  #else
    #define PACKSTRUCT(decl)    decl __attribute__((packed))
  #endif
  #endif
  #define WEAK                  __attribute__((weak))
  #define KEEP_SYMBOL           __attribute__((used))
  #define OPTIMIZE_SPEED        __attribute__((optimize("O3")))
#else
/* Unknown */
#ifndef PACKSTRUCT
  #define PACKSTRUCT(decl) decl
#endif
  #define WEAK
  #define KEEP_SYMBOL
#endif

#ifndef EFR32
uint32_t bg_compat_reverse32(uint32_t x);

uint32_t bg_compat_reverse32_bytes(uint32_t x);

//declare interrupt state
#define BG_INT_DECLARE_STATE
//disable interrupts
#define BG_INT_DISABLE()
//enable interrupts
#define BG_INT_ENABLE()

//Stub implementations, mainly for running utests
#define BG_COUNT_LEADING_ZEROS(bits) __builtin_clz(bits)

#define BG_INVERT_WORD(bits) bg_compat_reverse32(bits)

#define BG_REVERSE_BYTES32(bits) bg_compat_reverse32_bytes(bits)

#define BG_ATOMIC_BIT_SET(bitmask, bit) { *bitmask |= BIT(bit); } while (0)

#define BG_ATOMIC_BIT_CLR(bitmask, bit) do { *bitmask &= ~BIT(bit); } while (0)

#define BG_BIT_SET_EXCLUSIVE(bitmask, bitset) do { *bitmask |= bitset; } while (0)

#define BG_BIT_CLR_EXCLUSIVE(bitmask, bitset) do { *bitmask &= ~(bitset); } while (0)

#define BG_SWAP_EXCLUSIVE(bitmask, newbits) unsigned long oldbits; oldbits = *(bitmask); *(bitmask) = newbits; return oldbits;

#else

//TODO: assumes EM device

#include "em_bus.h"

#if BGCOMMON_USE_CORE
#include "em_core.h"
#define BG_INT_DECLARE_STATE CORE_DECLARE_IRQ_STATE
#define BG_INT_DISABLE() CORE_ENTER_ATOMIC()
#define BG_INT_ENABLE() CORE_EXIT_ATOMIC()
#else
#pragma diag_suppress=Pe1105 // supress deprecation warning
#include "em_int.h"
#pragma diag_default=Pe1105
#define BG_INT_DECLARE_STATE
#define BG_INT_DISABLE() INT_Disable()
#define BG_INT_ENABLE() INT_Enable()
#endif

#define BG_COUNT_LEADING_ZEROS(bits) __CLZ(bits)

#define BG_INVERT_WORD(bits) __RBIT(bits)

#define BG_REVERSE_BYTES32(bytes) __REV(bytes)

#define BG_ATOMIC_BIT_SET(bitmask, bit) \
  BG_BIT_SET_EXCLUSIVE(bitmask, BIT(bit))

#define BG_ATOMIC_BIT_CLR(bitmask, bit) \
  BG_BIT_CLR_EXCLUSIVE(bitmask, BIT(bit))

#define BG_BIT_SET_EXCLUSIVE(bitmask, setbits) \
  do {} while (__STREXW(__LDREXW(bitmask) | (setbits), bitmask))

#define BG_BIT_CLR_EXCLUSIVE(bitmask, setbits) \
  do {} while (__STREXW(__LDREXW(bitmask) & ~(setbits), bitmask))

#define BG_SWAP_EXCLUSIVE(bitmask, newbits) \
  unsigned long oldbits;                    \
  do {                                      \
    oldbits = __LDREXW(bitmask);            \
  } while (__STREXW(newbits, bitmask));     \
  return oldbits;

#endif

#ifndef PACKED
    #ifdef __GNUC__ //GNU Packed definition
        #define PACKED __attribute__((packed))
    #elif defined(__CWCC__)   //Codewarrior
        #define PACKED
        #pragma options align=packed
    #elif defined(__IAR_SYSTEMS_ICC__)
        #define PACKED
    #else
        #define PACKED
    #endif
#endif

#ifndef ALIGNED_WORD_SIZE
    #define ALIGNED_WORD_SIZE   (4)
#endif

#ifndef PACKSTRUCT
    #ifdef __GNUC__
        #ifdef _WIN32
            #define PACKSTRUCT(decl) decl __attribute__((__packed__, gcc_struct))
        #else
            #define PACKSTRUCT(decl) decl __attribute__((__packed__))
        #endif
        #define ALIGNED __attribute__((aligned(ALIGNED_WORD_SIZE)))
    #elif defined(__IAR_SYSTEMS_ICC__)
        #define PACKSTRUCT(decl) __packed decl
        #define ALIGNED
    #elif _MSC_VER  //msvc
        #define PACKSTRUCT(decl) __pragma(pack(push, 1) ) decl __pragma(pack(pop) )
        #define ALIGNED
    #else
        #define PACKSTRUCT(a) a PACKED
    #endif
#endif

#ifndef WEAK
    #if defined(__IAR_SYSTEMS_ICC__)
/* IAR ICC*/
        #define WEAK __weak
    #elif defined(__GNUC__)
/* GNU GCC */
        #define WEAK __attribute__ ((weak))
    #else
/* Unknown */
        #define WEAK
    #endif
#endif

#endif /* COMMON_INCLUDE_BG_COMPAT_H_ */
