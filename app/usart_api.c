/*
 * usart_api.c
 *
 *  Created on: 2020Äê10ÔÂ28ÈÕ
 *      Author: mi
 */
#include "usart_api.h"

#if (defined(MCU_OTA_DEMO) && MCU_OTA_DEMO)
#include "native_gecko.h"
#include "em_gpio.h"
#include "em_usart.h"
#include "em_ldma.h"
#include "em_bus.h"
#include "em_timer.h"
#include "mible_log.h"

// User options for plugin UART
#define BTL_DRIVER_UART_RX_BUFFER_SIZE      128
#define BTL_DRIVER_UART_TX_BUFFER_SIZE      128

#define BTL_DRIVER_UART                     USART1
#define BTL_DRIVER_USART_NUM                1
#define BTL_DRIVER_UART_CLOCK               cmuClock_USART1
#define BTL_DRIVER_UART_LDMA_TXBL_SIGNAL    (ldmaPeripheralSignal_USART1_TXBL)
#define BTL_DRIVER_UART_LDMA_RXDATAV_SIGNAL (ldmaPeripheralSignal_USART1_RXDATAV)

#define BTL_DRIVER_UART_LDMA_RX_CHANNEL     0
#define BTL_DRIVER_UART_LDMA_TX_CHANNEL     1

/// LDMA will pingpong between two halves of this buffer.
static uint8_t rxBuffer[BTL_DRIVER_UART_RX_BUFFER_SIZE];
/// Transmit buffer for LDMA use.
static uint8_t txBuffer[BTL_DRIVER_UART_TX_BUFFER_SIZE];
/// Amount of bytes in the current transmit operation
static size_t  txLength;

/// Index into the receive buffer indicating which byte is due to be read next.
static size_t  rxHead;

/// Flag to indicate hardware is up and running
static bool    initialized = false;
/// LDMA channel configuration triggering on free space in UART transmit FIFO
static const LDMA_TransferCfg_t ldmaTxTransfer = LDMA_TRANSFER_CFG_PERIPHERAL(
  BTL_DRIVER_UART_LDMA_TXBL_SIGNAL
  );
/// LDMA channel configuration triggering on available byte in UART receive FIFO
static const LDMA_TransferCfg_t ldmaRxTransfer = LDMA_TRANSFER_CFG_PERIPHERAL(
  BTL_DRIVER_UART_LDMA_RXDATAV_SIGNAL
  );

/// LDMA transfer for copying transmit buffer to UART TX FIFO
static LDMA_Descriptor_t ldmaTxDesc = LDMA_DESCRIPTOR_SINGLE_M2P_BYTE(
  txBuffer,
  &(BTL_DRIVER_UART->TXDATA),
  0
  );

static const LDMA_Descriptor_t ldmaRxDesc[4] = {
  /// First half of receive pingpong configuration
  LDMA_DESCRIPTOR_LINKREL_P2M_BYTE(
    &(BTL_DRIVER_UART->RXDATA),
    &(rxBuffer[0]),
    BTL_DRIVER_UART_RX_BUFFER_SIZE / 2,
    1
    ),
  // Sync structure waiting for SYNC[1], clearing SYNC[0]
  LDMA_DESCRIPTOR_LINKREL_SYNC(
    0,
    (1 << 0),
    (1 << 1),
    (1 << 1),
    1
    ),
  /// Second half of receive pingpong configuration
  LDMA_DESCRIPTOR_LINKREL_P2M_BYTE(
    &(BTL_DRIVER_UART->RXDATA),
    &(rxBuffer[BTL_DRIVER_UART_RX_BUFFER_SIZE / 2]),
    BTL_DRIVER_UART_RX_BUFFER_SIZE / 2,
    1
    ),
  /// Sync structure waiting for SYNC[0], clearing SYNC[1]
  LDMA_DESCRIPTOR_LINKREL_SYNC(
    0,
    (1 << 1),
    (1 << 0),
    (1 << 0),
    -3
    )
};

uint32_t util_getClockFreq(void)
{
#if defined(_SILICON_LABS_32B_SERIES_2)
  const uint8_t frequencies[] = { 4, 0, 0, 7, 0, 0, 13, 16, 19, 0, 26, 32, 38, 48, 56, 64, 80 };
#else
  const uint8_t frequencies[] = { 4, 0, 0, 7, 0, 0, 13, 16, 19, 0, 26, 32, 38, 48, 56, 64, 72 };
#endif
  uint32_t clockFreq;
#if defined(_SILICON_LABS_32B_SERIES_2)
  if ((CMU->SYSCLKCTRL & _CMU_SYSCLKCTRL_CLKSEL_MASK) == CMU_SYSCLKCTRL_CLKSEL_HFXO) {
    #if defined(BSP_CLK_HFXO_FREQ)
    clockFreq = BSP_CLK_HFXO_FREQ;
    #else
    clockFreq = 38400000UL;
    #endif
  } else {
#if defined(_CMU_CLKEN0_MASK)
    CMU->CLKEN0_SET = CMU_CLKEN0_HFRCO0;
#endif
    clockFreq = (HFRCO0->CAL & _HFRCO_CAL_FREQRANGE_MASK) >> _HFRCO_CAL_FREQRANGE_SHIFT;
    if (clockFreq > 16) {
      clockFreq = 19000000UL;
    } else {
      clockFreq = frequencies[clockFreq] * 1000000UL;
    }
    if (clockFreq == 4000000UL) {
      clockFreq /= (0x1 << ((HFRCO0->CAL & _HFRCO_CAL_CLKDIV_MASK) >> _HFRCO_CAL_CLKDIV_SHIFT));
    }
  }
  clockFreq /= (1U + ((CMU->SYSCLKCTRL & _CMU_SYSCLKCTRL_HCLKPRESC_MASK)
                      >> _CMU_SYSCLKCTRL_HCLKPRESC_SHIFT));
#else
  if ((CMU->HFCLKSTATUS & _CMU_HFCLKSTATUS_SELECTED_MASK) == CMU_HFCLKSTATUS_SELECTED_HFXO) {
    #if defined(BSP_CLK_HFXO_FREQ)
    clockFreq = BSP_CLK_HFXO_FREQ;
    #else
    clockFreq = 38400000UL;
    #endif
  } else {
    clockFreq = (CMU->HFRCOCTRL & _CMU_HFRCOCTRL_FREQRANGE_MASK) >> _CMU_HFRCOCTRL_FREQRANGE_SHIFT;
    if (clockFreq > 16) {
      clockFreq = 19000000UL;
    } else {
      clockFreq = frequencies[clockFreq] * 1000000UL;
    }
    if (clockFreq == 4000000UL) {
      clockFreq /= (0x1 << ((CMU->HFRCOCTRL & _CMU_HFRCOCTRL_CLKDIV_MASK) >> _CMU_HFRCOCTRL_CLKDIV_SHIFT));
    }
  }
  clockFreq /= (1U + ((CMU->HFPRESC & _CMU_HFPRESC_PRESC_MASK)
                      >> _CMU_HFPRESC_PRESC_SHIFT));
#endif
  return clockFreq;
}

mible_status_t mible_usart_init(void)
{
    uint32_t refFreq, clkdiv;

    // Clock peripherals
#if defined(_SILICON_LABS_32B_SERIES_1)
    CMU_ClockEnable(cmuClock_HFPER, true);
    CMU_ClockEnable(cmuClock_GPIO, true);
    CMU_ClockEnable(cmuClock_LDMA, true);
    CMU_ClockEnable(BTL_DRIVER_UART_CLOCK, true);
#endif

#if defined(_CMU_CLKEN0_MASK)
    CMU->CLKEN0_SET = CMU_CLKEN0_GPIO;
#if BSP_SERIAL_APP_PORT == HAL_SERIAL_PORT_USART0
    CMU->CLKEN0_SET = CMU_CLKEN0_USART0;
#elif BSP_SERIAL_APP_PORT == HAL_SERIAL_PORT_USART1
    CMU->CLKEN0_SET = CMU_CLKEN0_USART1;
#else
#error "Invalid BSP_EXTFLASH_USART"
#endif
#endif

#if defined(USART_EN_EN)
    BTL_DRIVER_UART->EN_SET = USART_EN_EN;
#endif

    // Set up USART
    BTL_DRIVER_UART->CMD = USART_CMD_RXDIS
                           | USART_CMD_TXDIS
                           | USART_CMD_MASTERDIS
                           | USART_CMD_RXBLOCKDIS
                           | USART_CMD_TXTRIDIS
                           | USART_CMD_CLEARTX
                           | USART_CMD_CLEARRX;
    BTL_DRIVER_UART->CTRL = _USART_CTRL_RESETVALUE;
    BTL_DRIVER_UART->CTRLX = _USART_CTRLX_RESETVALUE;
    BTL_DRIVER_UART->FRAME = _USART_FRAME_RESETVALUE;
    BTL_DRIVER_UART->TRIGCTRL = _USART_TRIGCTRL_RESETVALUE;
    BTL_DRIVER_UART->CLKDIV = _USART_CLKDIV_RESETVALUE;
    BTL_DRIVER_UART->IEN = _USART_IEN_RESETVALUE;
#if defined(_USART_IFC_MASK)
    BTL_DRIVER_UART->IFC = _USART_IFC_MASK;
#else
    BTL_DRIVER_UART->IF_CLR = _USART_IF_MASK;
#endif

#if defined(_USART_ROUTEPEN_RESETVALUE)
BTL_DRIVER_UART->ROUTEPEN = _USART_ROUTEPEN_RESETVALUE;
BTL_DRIVER_UART->ROUTELOC0 = _USART_ROUTELOC0_RESETVALUE;
BTL_DRIVER_UART->ROUTELOC1 = _USART_ROUTELOC1_RESETVALUE;
#else
#if defined(BTL_DRIVER_USART_NUM)
    GPIO->USARTROUTE[BTL_DRIVER_USART_NUM].ROUTEEN = _GPIO_USART_ROUTEEN_RESETVALUE;
    GPIO->USARTROUTE[BTL_DRIVER_USART_NUM].TXROUTE = _GPIO_USART_TXROUTE_RESETVALUE;
    GPIO->USARTROUTE[BTL_DRIVER_USART_NUM].RXROUTE = _GPIO_USART_RXROUTE_RESETVALUE;
    GPIO->USARTROUTE[BTL_DRIVER_USART_NUM].CTSROUTE = _GPIO_USART_CTSROUTE_RESETVALUE;
    GPIO->USARTROUTE[BTL_DRIVER_USART_NUM].RTSROUTE = _GPIO_USART_RTSROUTE_RESETVALUE;
#else
#error "No USART number given"
#endif // BTL_DRIVER_USART_NUM
#endif

    // Configure databits, stopbits and parity
    BTL_DRIVER_UART->FRAME = USART_FRAME_DATABITS_EIGHT
                             | USART_FRAME_STOPBITS_ONE
                             | USART_FRAME_PARITY_NONE;

    // Configure oversampling and baudrate
    BTL_DRIVER_UART->CTRL |= USART_CTRL_OVS_X16;
    refFreq = util_getClockFreq();
#if defined(_SILICON_LABS_32B_SERIES_2)
    refFreq = refFreq / (1U + ((CMU->SYSCLKCTRL & _CMU_SYSCLKCTRL_PCLKPRESC_MASK)
                               >> _CMU_SYSCLKCTRL_PCLKPRESC_SHIFT));
#endif
    clkdiv = 32 * refFreq + (16 * HAL_SERIAL_APP_BAUD_RATE) / 2;
    clkdiv /= (16 * HAL_SERIAL_APP_BAUD_RATE);
    clkdiv -= 32;
    clkdiv *= 8;

    // Verify that resulting clock divider is within limits
    //BTL_ASSERT(clkdiv <= _USART_CLKDIV_DIV_MASK);

    // If asserts are not enabled, make sure we don't write to reserved bits
    clkdiv &= _USART_CLKDIV_DIV_MASK;

    BTL_DRIVER_UART->CLKDIV = clkdiv;

    GPIO_PinModeSet(BSP_SERIAL_APP_TX_PORT,
                    BSP_SERIAL_APP_TX_PIN,
                    gpioModePushPull,
                    1);
    GPIO_PinModeSet(BSP_SERIAL_APP_RX_PORT,
                    BSP_SERIAL_APP_RX_PIN,
                    gpioModeInput,
                    1);

// Configure route
#if defined(_USART_ROUTEPEN_RESETVALUE)
    BTL_DRIVER_UART->ROUTELOC0 = (BSP_SERIAL_APP_TX_LOC
                                  << _USART_ROUTELOC0_TXLOC_SHIFT)
                                 | (BSP_SERIAL_APP_RX_LOC
                                    << _USART_ROUTELOC0_RXLOC_SHIFT);
    BTL_DRIVER_UART->ROUTEPEN = USART_ROUTEPEN_TXPEN | USART_ROUTEPEN_RXPEN;
#else
    GPIO->USARTROUTE[BTL_DRIVER_USART_NUM].ROUTEEN = GPIO_USART_ROUTEEN_TXPEN
                                                     | GPIO_USART_ROUTEEN_RXPEN;
    GPIO->USARTROUTE[BTL_DRIVER_USART_NUM].TXROUTE = 0
                                                     | (BSP_SERIAL_APP_TX_PORT << _GPIO_USART_TXROUTE_PORT_SHIFT)
                                                     | (BSP_SERIAL_APP_TX_PIN << _GPIO_USART_TXROUTE_PIN_SHIFT);
    GPIO->USARTROUTE[BTL_DRIVER_USART_NUM].RXROUTE = 0
                                                     | (BSP_SERIAL_APP_RX_PORT << _GPIO_USART_RXROUTE_PORT_SHIFT)
                                                     | (BSP_SERIAL_APP_RX_PIN << _GPIO_USART_RXROUTE_PIN_SHIFT);
#endif

// Configure CTS/RTS in case of flow control
#if (HAL_SERIAL_APP_FLOW_CONTROL == HAL_USART_FLOW_CONTROL_HW)      \
    || (HAL_SERIAL_APP_FLOW_CONTROL == HAL_USART_FLOW_CONTROL_HWUART) \
    || (HAL_SERIAL_APP_FLOW_CONTROL == HAL_UART_FLOW_CONTROL_HW)      \
    || (HAL_SERIAL_APP_FLOW_CONTROL == HAL_UART_FLOW_CONTROL_HWUART)
    GPIO_PinModeSet(BSP_SERIAL_APP_RTS_PORT,
                    BSP_SERIAL_APP_RTS_PIN,
                    gpioModePushPull,
                    1);
    GPIO_PinModeSet(BSP_SERIAL_APP_CTS_PORT,
                    BSP_SERIAL_APP_CTS_PIN,
                    gpioModeInput,
                    1);

    // Configure CTS/RTS route
#if defined(_USART_ROUTEPEN_RESETVALUE)
    BTL_DRIVER_UART->ROUTELOC1 = (BSP_SERIAL_APP_RTS_LOC
                                  << _USART_ROUTELOC1_RTSLOC_SHIFT)
                                 | (BSP_SERIAL_APP_CTS_LOC
                                    << _USART_ROUTELOC1_CTSLOC_SHIFT);
    BTL_DRIVER_UART->ROUTEPEN |= USART_ROUTEPEN_RTSPEN | USART_ROUTEPEN_CTSPEN;
#else
    GPIO->USARTROUTE_SET[BTL_DRIVER_USART_NUM].ROUTEEN = GPIO_USART_ROUTEEN_RTSPEN;
    GPIO->USARTROUTE[BTL_DRIVER_USART_NUM].CTSROUTE =
      (BSP_SERIAL_APP_CTS_PORT << _GPIO_USART_CTSROUTE_PORT_SHIFT)
      | (BSP_SERIAL_APP_CTS_PIN << _GPIO_USART_CTSROUTE_PIN_SHIFT);
    GPIO->USARTROUTE[BTL_DRIVER_USART_NUM].RTSROUTE =
      (BSP_SERIAL_APP_RTS_PORT << _GPIO_USART_RTSROUTE_PORT_SHIFT)
      | (BSP_SERIAL_APP_RTS_PIN << _GPIO_USART_RTSROUTE_PIN_SHIFT);

#endif

    // Configure USART for flow control
    BTL_DRIVER_UART->CTRLX |= USART_CTRLX_CTSEN;
#endif

#if (HAL_VCOM_ENABLE == 1) && defined(BSP_VCOM_ENABLE_PORT)
    GPIO_PinModeSet(BSP_VCOM_ENABLE_PORT,
                    BSP_VCOM_ENABLE_PIN,
                    gpioModePushPull,
                    1);
#endif

    // Enable TX/RX
    BTL_DRIVER_UART->CMD = USART_CMD_RXEN
                           | USART_CMD_TXEN;

#if defined(_CMU_CLKEN0_MASK)
    CMU->CLKEN0_SET = (CMU_CLKEN0_LDMA | CMU_CLKEN0_LDMAXBAR);
#endif

#if defined(LDMA_EN_EN)
    LDMA->EN = LDMA_EN_EN;
#endif

// Reset LDMA
    LDMA->CTRL = _LDMA_CTRL_RESETVALUE;
    LDMA->CHEN = _LDMA_CHEN_RESETVALUE;
    LDMA->DBGHALT = _LDMA_DBGHALT_RESETVALUE;
    LDMA->REQDIS = _LDMA_REQDIS_RESETVALUE;
    LDMA->IEN = _LDMA_IEN_RESETVALUE;

// Set up channel 0 as RX transfer
#if defined(LDMAXBAR)
    LDMAXBAR->CH[BTL_DRIVER_UART_LDMA_RX_CHANNEL].REQSEL = ldmaRxTransfer.ldmaReqSel;
#else
    LDMA->CH[BTL_DRIVER_UART_LDMA_RX_CHANNEL].REQSEL = ldmaRxTransfer.ldmaReqSel;
#endif
    LDMA->CH[BTL_DRIVER_UART_LDMA_RX_CHANNEL].LOOP
      = (ldmaRxTransfer.ldmaLoopCnt << _LDMA_CH_LOOP_LOOPCNT_SHIFT);
    LDMA->CH[BTL_DRIVER_UART_LDMA_RX_CHANNEL].CFG
      = (ldmaRxTransfer.ldmaCfgArbSlots << _LDMA_CH_CFG_ARBSLOTS_SHIFT)
        | (ldmaRxTransfer.ldmaCfgSrcIncSign << _LDMA_CH_CFG_SRCINCSIGN_SHIFT)
        | (ldmaRxTransfer.ldmaCfgDstIncSign << _LDMA_CH_CFG_DSTINCSIGN_SHIFT);

    LDMA->CH[BTL_DRIVER_UART_LDMA_RX_CHANNEL].LINK
      = (uint32_t)(&ldmaRxDesc[0]) & _LDMA_CH_LINK_LINKADDR_MASK;

// Set up channel 1 as TX transfer
#if defined(LDMAXBAR)
    LDMAXBAR->CH[BTL_DRIVER_UART_LDMA_TX_CHANNEL].REQSEL = ldmaTxTransfer.ldmaReqSel;
#else
    LDMA->CH[BTL_DRIVER_UART_LDMA_TX_CHANNEL].REQSEL = ldmaTxTransfer.ldmaReqSel;
#endif
    LDMA->CH[BTL_DRIVER_UART_LDMA_TX_CHANNEL].LOOP
      = (ldmaTxTransfer.ldmaLoopCnt << _LDMA_CH_LOOP_LOOPCNT_SHIFT);
    LDMA->CH[BTL_DRIVER_UART_LDMA_TX_CHANNEL].CFG
      = (ldmaTxTransfer.ldmaCfgArbSlots << _LDMA_CH_CFG_ARBSLOTS_SHIFT)
        | (ldmaTxTransfer.ldmaCfgSrcIncSign << _LDMA_CH_CFG_SRCINCSIGN_SHIFT)
        | (ldmaTxTransfer.ldmaCfgDstIncSign << _LDMA_CH_CFG_DSTINCSIGN_SHIFT);

    // Clear DONE flag on both RX and TX channels
    BUS_RegMaskedClear(&LDMA->CHDONE,
                       ((1 << BTL_DRIVER_UART_LDMA_RX_CHANNEL)
                        | (1 << BTL_DRIVER_UART_LDMA_TX_CHANNEL)));

    // Kick off background RX
    LDMA->LINKLOAD = (1 << BTL_DRIVER_UART_LDMA_RX_CHANNEL);

// Mark second half of RX buffer as ready
#if defined(_LDMA_SYNCSWSET_MASK)
    LDMA->SYNCSWSET_SET = 1 << 1;
#else
    BUS_RegMaskedSet(&LDMA->SYNC, 1 << 1);
#endif

    initialized = true;

    return MI_SUCCESS;
}

bool uart_isTxIdle(void)
{
  if (LDMA->CHDONE & (1 << BTL_DRIVER_UART_LDMA_TX_CHANNEL)) {
#if defined(_LDMA_CHDIS_MASK)
    LDMA->CHDIS = (1 << BTL_DRIVER_UART_LDMA_TX_CHANNEL);
#else
    BUS_RegMaskedClear(&LDMA->CHEN, 1 << BTL_DRIVER_UART_LDMA_TX_CHANNEL);
#endif
    BUS_RegMaskedClear(&LDMA->CHDONE, 1 << BTL_DRIVER_UART_LDMA_TX_CHANNEL);
    txLength = 0;
    return true;
#if defined(_LDMA_CHSTATUS_MASK)
  } else if ((LDMA->CHSTATUS & (1 << BTL_DRIVER_UART_LDMA_TX_CHANNEL)) == 0) {
#else
  } else if ((LDMA->CHEN & (1 << BTL_DRIVER_UART_LDMA_TX_CHANNEL)) == 0) {
#endif
    BUS_RegMaskedClear(&LDMA->CHDONE, 1 << BTL_DRIVER_UART_LDMA_TX_CHANNEL);
    txLength = 0;
    return true;
  }

  return false;
}

mible_status_t mible_usart_send_buffer(const char* buffer, size_t length, bool blocking)
{
    if (length >= BTL_DRIVER_UART_TX_BUFFER_SIZE) {
        return MI_ERR_DATA_SIZE;
    }

    if (!uart_isTxIdle()) {
        return MI_ERR_BUSY;
    }

    // Copy buffer
    txLength = length;
    length = 0;
    for (; length < txLength; length++) {
        txBuffer[length] = buffer[length];
    }

    // Populate descriptor
    ldmaTxDesc.xfer.xferCnt = txLength - 1;

    // Kick off transfer
    LDMA->CH[BTL_DRIVER_UART_LDMA_TX_CHANNEL].LINK
      = (uint32_t)(&ldmaTxDesc) & _LDMA_CH_LINK_LINKADDR_MASK;
    LDMA->LINKLOAD = 1 << BTL_DRIVER_UART_LDMA_TX_CHANNEL;

    // Optional wait for completion
    if (blocking) {
          while (uart_isTxIdle() == false) {
            // Do nothing
          }
          while (!(BTL_DRIVER_UART->STATUS & USART_STATUS_TXC)) {
            // Do nothing
          }
    }

    return MI_SUCCESS;
}

mible_status_t mible_usart_send_byte(uint8_t byte)
{
    // Wait until previous LDMA transfer is done
    while (!uart_isTxIdle()) {
    // Do nothing
    }

    // Wait until there is room for one more byte
    while (!(BTL_DRIVER_UART->STATUS & USART_STATUS_TXBL)) {
    // Do nothing
    }

    // Send byte
    BTL_DRIVER_UART->TXDATA = byte;

    // Wait until byte has been fully sent out
    while (!(BTL_DRIVER_UART->STATUS & USART_STATUS_TXC)) {
    // Do nothing
    }

    return MI_SUCCESS;
}

size_t  uart_getRxAvailableBytes(void)
{
  size_t ldmaHead;
  size_t dst;

  // Get destination address for next transfer
  dst = LDMA->CH[BTL_DRIVER_UART_LDMA_RX_CHANNEL].DST;

  if (dst == 0x0101) {
    // SYNC descriptor with bit 0 of MATCHEN and MATCHVAL set
    ldmaHead = 0;
  } else if (dst == 0x0202) {
    // SYNC descriptor with bit 1 of MATCHEN and MATCHVAL set
    ldmaHead = BTL_DRIVER_UART_RX_BUFFER_SIZE / 2;
  } else {
    // XFER descriptor with absolute address in buffer
    ldmaHead = dst - (uint32_t)(rxBuffer);
  }

  // Return difference between received head and LDMA head
  if (rxHead == ldmaHead) {
    return 0;
  } else if (rxHead < ldmaHead) {
    return ldmaHead - rxHead;
  } else {
    return BTL_DRIVER_UART_RX_BUFFER_SIZE - (rxHead - ldmaHead);
  }
}

static uint16_t delayTarget = 0;
static bool expectOverflow;
static uint32_t ticksPerMillisecond = 0;
static uint32_t reqLength = 0;
void delay_init(void)
{
  // Enable clocks to TIMER0
#if defined(CMU_CTRL_HFPERCLKEN)
  CMU->CTRL |= CMU_CTRL_HFPERCLKEN;
  CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_TIMER0;
#endif
#if defined(_CMU_CLKEN0_MASK)
  CMU->CLKEN0_SET = CMU_CLKEN0_TIMER0;
#endif

  // Calculate the length of a tick
  ticksPerMillisecond = (util_getClockFreq() / 1000UL) / 1024UL;

  // Initialize timer
  TIMER_Init_TypeDef init = TIMER_INIT_DEFAULT;
  init.prescale = timerPrescale1024;
  TIMER_Init(TIMER0, &init);
}

void delay_milliseconds(uint32_t msecs, bool blocking)
{
  // TODO: Assert msecs within a single TIMER0 overflow (approx. 3300 ms)

  uint16_t count = TIMER0->CNT;
  delayTarget = count + (msecs * ticksPerMillisecond);
  expectOverflow = (delayTarget < count);

  if (blocking) {
    while (TIMER0->CNT != delayTarget) {
      // Do nothing
    }
  }
#if defined(TIMER_IFC_OF)
  TIMER0->IFC = TIMER_IFC_OF;
#else
  TIMER0->IF_CLR = TIMER_IF_OF;
#endif
  TIMER0->IEN = TIMER_IEN_OF;
}

bool delay_expired(void)
{
  // Expecting overflow, but it hasn't happened yet
  if (expectOverflow && !(TIMER0->IF & TIMER_IF_OF)) {
    return false;
  }
  // Not expecting overflow, but it still happened
  if (!expectOverflow && (TIMER0->IF & TIMER_IF_OF)) {
    return true;
  }

  // Return true if CNT has passed the target
  return TIMER0->CNT >= delayTarget;
}

mible_status_t mible_usart_receive_delay_init(uint16_t length, uint32_t timeout)
{
    delay_init();
    delay_milliseconds(timeout, false);
    reqLength = length;
    return MI_SUCCESS;
}

mible_status_t mible_usart_receive_delay_expired(void)
{
    if((uart_getRxAvailableBytes() >= reqLength) || delay_expired())
        return MI_SUCCESS;
    else
        return MI_ERR_BUSY;
}

mible_status_t mible_usart_receive_buffer(uint8_t  * buffer,
                           size_t   requestedLength,
                           size_t   * receivedLength,
                           bool     blocking,
                           uint32_t timeout)
{
  size_t copyBytes = 0;
  size_t copiedBytes = 0;

  // Check whether we have enough data
  // Optional spin for timeout cycles
  if (blocking) {
    if (timeout != 0) {
      delay_init();
      delay_milliseconds(timeout, false);
    }

    while (uart_getRxAvailableBytes() < requestedLength) {
      if ((timeout != 0) && delay_expired()) {
          MI_LOG_WARNING("mible_usart_receive_buffer delay timeout %d\n", delayTarget);
        break;
      }
    }
  }

  copyBytes = uart_getRxAvailableBytes();
  if (requestedLength < copyBytes) {
    copyBytes = requestedLength;
  }

  // Copy up to requested bytes to given buffer
  while (copiedBytes < copyBytes) {
    buffer[copiedBytes] = rxBuffer[rxHead];
    copiedBytes++;
    rxHead++;

    if (rxHead == BTL_DRIVER_UART_RX_BUFFER_SIZE) {
      rxHead = 0;
      // Completed processing of second half of the buffer, mark it as available
      // for LDMA again by setting SYNC[1]
#if defined(_LDMA_SYNCSWSET_MASK)
      LDMA->SYNCSWSET_SET = 1 << 1;
#else
      BUS_RegMaskedSet(&LDMA->SYNC, 1 << 1);
#endif
    } else if (rxHead == BTL_DRIVER_UART_RX_BUFFER_SIZE / 2) {
      // Completed processing of first half of the buffer, mark it as available
      // for LDMA again by setting SYNC[0]
#if defined(_LDMA_SYNCSWSET_MASK)
      LDMA->SYNCSWSET_SET = 1 << 0;
#else
      BUS_RegMaskedSet(&LDMA->SYNC, 1 << 0);
#endif
    }

    // TODO: Check overflow by checking both SYNC[0] and SYNC[1]
  }

  if ((uint32_t)receivedLength != 0UL) {
    *receivedLength = copiedBytes;
  }

  if (copiedBytes < requestedLength) {
    return MI_ERR_TIMEOUT;
  } else {
    return MI_SUCCESS;
  }
}

mible_status_t mible_usart_receive_byte(uint8_t* byte)
{
  return mible_usart_receive_buffer(byte, 1, (size_t*)0UL, false, 0);
}

mible_status_t mible_usart_receive_byte_timeout(uint8_t* byte, uint32_t timeout)
{
  return mible_usart_receive_buffer(byte, 1, (size_t *)0UL, true, timeout);
}

mible_status_t mible_usart_flush(bool flushTx, bool flushRx)
{
  if (flushTx) {
#if defined(_LDMA_CHDIS_MASK)
    LDMA->CHDIS = (1 << BTL_DRIVER_UART_LDMA_TX_CHANNEL);
#else
    BUS_RegMaskedClear(&LDMA->CHEN, 1 << BTL_DRIVER_UART_LDMA_TX_CHANNEL);
#endif
    txLength = 0;
  }

  if (flushRx) {
#if defined(_LDMA_CHDIS_MASK)
    LDMA->CHDIS = (1 << BTL_DRIVER_UART_LDMA_RX_CHANNEL);
#else
    BUS_RegMaskedClear(&LDMA->CHEN, 1 << BTL_DRIVER_UART_LDMA_RX_CHANNEL);
#endif
    BUS_RegMaskedClear(&LDMA->CHDONE, 1 << BTL_DRIVER_UART_LDMA_RX_CHANNEL);
    LDMA->CH[BTL_DRIVER_UART_LDMA_RX_CHANNEL].LINK
      = (uint32_t)(&ldmaRxDesc[0]) & _LDMA_CH_LINK_LINKADDR_MASK;
    rxHead = 0;
    LDMA->LINKLOAD = (1 << BTL_DRIVER_UART_LDMA_RX_CHANNEL);

    // Mark second half of RX buffer as ready
#if defined(_LDMA_SYNCSWSET_MASK)
    BUS_RegMaskedSet(&LDMA->SYNCSWSET, 1 << 1);
#else
    BUS_RegMaskedSet(&LDMA->SYNC, 1 << 1);
#endif
  }

  return MI_SUCCESS;
}
#endif
