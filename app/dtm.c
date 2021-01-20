#include <stdio.h>
#include "dtm.h"
#include "em_usart.h"
#include "em_cmu.h"
#include "em_device.h"
#if defined(CRYOTIMER_PRESENT)
#include "em_cryotimer.h"
#elif defined(BURTC_PRESENT)
#include "em_burtc.h"
#endif

#include "cryptography/mi_mesh_otp.h"
#include "cryptography/mi_crypto.h"
#include "mi_config.h"

#define DTM_UART           USART0
#define DTM_UART_CLK       cmuClock_USART0
#define DTM_UART_IRQn      USART0_RX_IRQn
#define DTM_UART_IRQ       USART0_RX_IRQHandler

#define DTM_UART_RX_PIN    BSP_USART0_RX_PIN
#define DTM_UART_RX_PORT   BSP_USART0_RX_PORT
#define DTM_UART_RX_LOC    BSP_USART0_RX_LOC

#define DTM_UART_TX_PIN    BSP_USART0_TX_PIN
#define DTM_UART_TX_PORT   BSP_USART0_TX_PORT
#define DTM_UART_TX_LOC    BSP_USART0_TX_LOC

#define SUPPORTED_FEATURES             (FEATURE_PACKET_EXTENSION | FEATURE_2M_PHY)
#define MAX_PDU_OCTETS                  255
#define T_MIN                           6  // Inter byte timeout is 5 msec in spec, add 1 msec for tolerance
#define TRANSCEIVER_LENGTH_HIGH_MAX     3

#define BIT(x) (1 << (x))
#define CALC_MAX_PDU_TIME(octet_time, packet_overhead_time) (MAX_PDU_OCTETS * (octet_time) + (packet_overhead_time))

static bd_addr customer_bt_addr;
static void uart_tx(uint8_t data);
static const testmode_config_t cfg = {
        .write_response_byte = uart_tx,
        .get_ticks = BURTC_CounterGet,
        .ticks_per_second = 32768,
        .command_ready_signal = 1,
};

enum phy {
    PHY_NONE = 0,
    PHY_1M = 1,
    PHY_2M = 2,
    PHY_CODED_S_8 = 3,
    PHY_CODED_S_2 = 4,
    PHY_MAX
};

static const uint16_t MAX_PDU_TIME[] = {
    [PHY_1M] = CALC_MAX_PDU_TIME(8, 80),
    [PHY_2M] = CALC_MAX_PDU_TIME(4, 44),
    [PHY_CODED_S_8] = CALC_MAX_PDU_TIME(64, 720),
    [PHY_CODED_S_2] = CALC_MAX_PDU_TIME(16, 462),
};

static struct {
    uint8_t data[8];
    uint8_t len;
    uint32_t last_byte_time;
} cmd_buffer;

struct setup {
    uint8_t transceiver_length_high;
    uint8_t phy;
};

static const struct setup default_setup = {
    .transceiver_length_high = 0,
    .phy = PHY_1M,
};

static struct setup setup;

enum cmd_type {
    CMD_TYPE_SETUP     = 0,
    CMD_TYPE_RXTEST    = 1,
    CMD_TYPE_TXTEST    = 2,
    CMD_TYPE_TESTEND   = 3,
    CMD_TYPE_MAX
};

enum feature {
    FEATURE_PACKET_EXTENSION        = BIT(0),
    FEATURE_2M_PHY                  = BIT(1),
    FEATURE_STABLE_MODULATION_INDEX = BIT(2),
};

struct setup_cmd_packet {
    uint8_t control;
    uint8_t parameter;
    uint8_t dc;
};

struct transceiver_cmd_packet {
    uint8_t frequency;
    uint8_t length;
    uint8_t pkt;
};

struct cmd_packet {
    enum cmd_type cmd_type;
    union {
        struct setup_cmd_packet setup;
        struct transceiver_cmd_packet transceiver;
    } cmd;
};

static struct {
    enum cmd_type transceiver_cmd;
} test_state;

enum setup_cmd {
    SETUP_CMD_RESET                   = 0,
    SETUP_CMD_TRANSCEIVER_LENGTH_HIGH = 1,
    SETUP_CMD_PHY                     = 2,
    SETUP_CMD_TX_MODULATION_INDEX     = 3,
    SETUP_CMD_READ_SUPPORTED_FEATURES = 4,
    SETUP_CMD_READ_PDU_PARAMETERS     = 5,
	SYSTEM_INFORMATION                = 0x3C,
    SETUP_CMD_CERTS_VERIFY            = 0x3E,
    SETUP_CMD_CUSTOMIZED              = 0x3F,
};

enum setup_parameter {
    STANDARD_MODULATION_INDEX = 0,
};

enum pdu_parameter {
    PDU_PARAMETER_MAX_TX_OCTETS    = 0,
    PDU_PARAMETER_MAX_TX_TIME      = 1,
    PDU_PARAMETER_MAX_RX_OCTETS    = 2,
    PDU_PARAMETER_MAX_RX_TIME      = 3,
};

enum test_status {
    TEST_STATUS_SUCCESS  = 0,
    TEST_STATUS_ERROR    = 1,
};

enum setup_module_cmd {
    READ_BT_ADDRESS         = 0,
    HARDWARE_IO_TEST        = 1,
    WRITE_BT_ADDRESS        = 2,
    PROGRAM_CERTS           = 3,
};

enum system_infomation_parameter {
    SYSTEM_REBOOT           = 0,
    PRINT_SYSINFO           = 1,
    SET_PA_LEVEL            = 2,
};

static int uart_block_send(uint8_t const * const out, uint16_t len)
{
    for (int i = 0; i < len; i++) {
        USART_Tx(DTM_UART, out[i]);
    }

    return 0;
}

static int uart_block_recv(uint8_t * const in, uint16_t ilen)
{
    int i;
    for (i = 0; i < ilen; i++) {
        in[i] = USART_Rx(DTM_UART);
    }
    return i;
}

static void UartSendStr(char *str)
{
    while (*str) {
        cfg.write_response_byte(*str++);
    }
}

static void reset_setup()
{
    setup = default_setup;
}

static void reset_cmd_buffer()
{
    cmd_buffer.len = 0;
}

static void reset_transceiver_test_state()
{
    test_state.transceiver_cmd = CMD_TYPE_MAX;
}

static void set_transceiver_test_state(enum cmd_type cmd)
{
    test_state.transceiver_cmd = cmd;
}

static enum cmd_type get_transceiver_test_state()
{
    return test_state.transceiver_cmd;
}

static inline uint32_t t_min_in_ticks()
{
    return T_MIN * cfg.ticks_per_second / 1000;
}

static void send_test_status(uint8_t status, uint16_t response)
{
    response &= 0x3fff;

    uint8_t response_high = response >> 7;
    uint8_t response_low = response & 0x7f;

    cfg.write_response_byte(response_high);
    cfg.write_response_byte((response_low << 1) | (status ? TEST_STATUS_ERROR : TEST_STATUS_SUCCESS));
}

static void send_packet_counter(uint16 counter)
{
    counter |= 0x8000;  // EV bit on
    cfg.write_response_byte(counter >> 8);
    cfg.write_response_byte(counter & 0xff);
}

static void parse_cmd_buffer(struct cmd_packet *result)
{
    result->cmd_type = (enum cmd_type) (cmd_buffer.data[0] >> 6);
    uint8_t field_1 = cmd_buffer.data[0] & 63;
    uint8_t field_2 = cmd_buffer.data[1] >> 2;
    uint8_t field_3 = cmd_buffer.data[1] & 0x3;

    switch (result->cmd_type) {
    case CMD_TYPE_SETUP:
        if (cmd_buffer.data[0] == 0x3F && cmd_buffer.data[1] == 0x08) {
            customer_bt_addr.addr[0] = cmd_buffer.data[7];
            customer_bt_addr.addr[1] = cmd_buffer.data[6];
            customer_bt_addr.addr[2] = cmd_buffer.data[5];
            customer_bt_addr.addr[3] = cmd_buffer.data[4];
            customer_bt_addr.addr[4] = cmd_buffer.data[3];
            customer_bt_addr.addr[5] = cmd_buffer.data[2];
        }
    case CMD_TYPE_TESTEND:
        result->cmd.setup.control   = field_1;
        result->cmd.setup.parameter = field_2;
        result->cmd.setup.dc        = field_3;
        break;

    case CMD_TYPE_RXTEST:
    case CMD_TYPE_TXTEST:
        result->cmd.transceiver.frequency = field_1;
        result->cmd.transceiver.length    = field_2;
        result->cmd.transceiver.pkt       = field_3;
        break;

    default:
        break;
    }
}

static void hardware_io_test(GPIO_Port_TypeDef port_out, uint8_t pin_out,
                             GPIO_Port_TypeDef port_in,  uint8_t pin_in)
{
    GPIO_PinModeSet(port_out, pin_out, gpioModePushPull, 1);
    GPIO_PinModeSet(port_in, pin_in, gpioModeInputPull, 0);

    cfg.write_response_byte((port_out + 0xA) <<4 | pin_out);
    cfg.write_response_byte((port_in + 0xA) <<4 | pin_in);
    cfg.write_response_byte(!GPIO_PinInGet(port_in, pin_in));
}

static int arch_u64toa(uint64_t data, char *c)
{
    int num = 0, index;
    char tmp;

    while(data > 0) {
        c[num++] = data%10 + '0';
        data /= 10;
    }
    c[num] = '\0';
    for(index = 0; index < (num >> 1); index++) {
        tmp = c[index];
        c[index] = c[num - 1 - index];
        c[num - 1 - index] = tmp;
    }

    return num;
}

static int get_device_id(char * p_out)
{
    uint8_t  did_le[8] = {0};
    uint64_t did_be    = 0;
    uint8_t  buf[512];
    int16_t  buf_len;
    msc_crt_t   crt;

    buf_len = mi_mesh_otp_read(OTP_DEV_CERT, buf, sizeof(buf));
    if (buf_len > 0) {
        mi_crypto_crt_parse_der(buf, buf_len, NULL, &crt);
        /* Cert SN big endian convert to little endian  */
        for (int i = 0; i < crt.sn.len; i++) did_le[crt.sn.len-1-i] = crt.sn.p[i];
        memcpy(&did_be, did_le, 8);
    } else {
        did_be = 0;
    }
    return arch_u64toa(did_be, p_out);
}

static void process_setup_command(enum cmd_type cmd_type,
        struct setup_cmd_packet *cmd)
{
    uint16_t result;
    struct gecko_msg_system_get_bt_address_rsp_t* bt_address;
    int8_t status = TEST_STATUS_SUCCESS;
    uint16_t response = 0;
    int16 pa_level=0;
    char szBuf[64];

    switch (cmd->control) {
    case SETUP_CMD_RESET:
        reset_setup();
        gecko_cmd_test_dtm_end();
        /* Don't send test status here, because when DTM end is processed, a DTM
         * completed event is emitted, and the test status is sent in the event
         * handler. */
        return;
        break;

    case SETUP_CMD_PHY:
        if (cmd->parameter == PHY_NONE || cmd->parameter >= PHY_MAX) {
            status = TEST_STATUS_ERROR;
        } else {
            setup.phy = cmd->parameter;
        }
        break;

    case SETUP_CMD_TRANSCEIVER_LENGTH_HIGH:
        if (cmd->parameter > TRANSCEIVER_LENGTH_HIGH_MAX) {
            status = TEST_STATUS_ERROR;
        } else {
            setup.transceiver_length_high = cmd->parameter;
        }
        break;

    case SETUP_CMD_TX_MODULATION_INDEX:
        // only standard modulation index is supported.
        if (cmd->parameter != STANDARD_MODULATION_INDEX) {
            status = TEST_STATUS_ERROR;
        }
        break;

    case SETUP_CMD_READ_SUPPORTED_FEATURES:
        if (cmd->parameter != 0) {
            status = TEST_STATUS_ERROR;
        } else {
            response = SUPPORTED_FEATURES;
        }
        break;

    case SETUP_CMD_READ_PDU_PARAMETERS:
        switch (cmd->parameter) {
        case PDU_PARAMETER_MAX_TX_OCTETS:
        case PDU_PARAMETER_MAX_RX_OCTETS:
            response = MAX_PDU_OCTETS;
            break;

        case PDU_PARAMETER_MAX_TX_TIME:
        case PDU_PARAMETER_MAX_RX_TIME:
            response = MAX_PDU_TIME[setup.phy] / 2;
            break;
        default:
            status = TEST_STATUS_ERROR;
            break;
        }
        break;
		
    case SYSTEM_INFORMATION:
        switch (cmd->parameter) {
        case SYSTEM_REBOOT:
            gecko_cmd_system_reset(0);
            break;

        case PRINT_SYSINFO:
            snprintf(szBuf, 64, "PRODUCT_ID : %d \r\n", PRODUCT_ID);
            UartSendStr(szBuf);

            //snprintf(szBuf, 64, "FW_VERSION : %s \r\n", MIBLE_LIB_AND_DEVELOPER_VERSION);
            UartSendStr(szBuf);

            bt_address = gecko_cmd_system_get_bt_address();
            snprintf(szBuf, 64, "MAC : %02X %02X %02X %02X %02X %02X \r\n",
                                bt_address->address.addr[5],
                                bt_address->address.addr[4],
                                bt_address->address.addr[3],
                                bt_address->address.addr[2],
                                bt_address->address.addr[1],
                                bt_address->address.addr[0]);
            UartSendStr(szBuf);
#if HAVE_OTP_PKI
            char did_string[21];
            get_device_id(did_string);
            snprintf(szBuf, 64, "DID : %s \r\n", did_string);
            UartSendStr(szBuf);
#else
            UartSendStr("NO CERTIFICATES\r\n");
#endif
            break;

        case SET_PA_LEVEL:
            pa_level = cmd_buffer.data[3] << 8 | cmd_buffer.data[2];
            pa_level = gecko_cmd_system_set_tx_power(pa_level)->set_power;
            snprintf(szBuf, 64, "PA LEVEL : %0d.%01d dBm \r\n", pa_level/10, pa_level%10);
            UartSendStr(szBuf);
            break;

        default:
            status = TEST_STATUS_ERROR;
            break;
        }
        break;
		
    case SETUP_CMD_CERTS_VERIFY:
#if HAVE_OTP_PKI
        status = mi_mesh_otp_verify();
#else
        status = TEST_STATUS_ERROR;
#endif
        break;
		
    case SETUP_CMD_CUSTOMIZED:
        switch (cmd->parameter) {
        case READ_BT_ADDRESS:
            bt_address = gecko_cmd_system_get_bt_address();
            cfg.write_response_byte(bt_address->address.addr[5]);
            cfg.write_response_byte(bt_address->address.addr[4]);
            cfg.write_response_byte(bt_address->address.addr[3]);
            cfg.write_response_byte(bt_address->address.addr[2]);
            cfg.write_response_byte(bt_address->address.addr[1]);
            cfg.write_response_byte(bt_address->address.addr[0]);
            break;

        case WRITE_BT_ADDRESS:
            result = gecko_cmd_system_set_identity_address(customer_bt_addr, 0)->result;
            if (result) {
                status = TEST_STATUS_ERROR;
            } else {
                gecko_cmd_system_reset(0);
            }
            break;

        case HARDWARE_IO_TEST:
            /* pin PD14 and Pin PF6 connected from external line
             * Set PD14 is High and Check PF6 is High or not.
             * if error happen, two bytes will transmit, high half of type
             * index is fail or success and low half index port.
             * second type index is pin number*/
            hardware_io_test(gpioPortB, 2, gpioPortC, 3);
            hardware_io_test(gpioPortB, 1, gpioPortC, 3);
            hardware_io_test(gpioPortB, 0, gpioPortC, 1);
            hardware_io_test(gpioPortA, 0, gpioPortC, 0);
            hardware_io_test(gpioPortA, 3, gpioPortC, 5);
            hardware_io_test(gpioPortA, 4, gpioPortD, 1);
            hardware_io_test(gpioPortC, 4, gpioPortD, 0);
            break;

        case PROGRAM_CERTS:
            USART_IntDisable(DTM_UART, USART_IF_RXDATAV);
#if HAVE_OTP_PKI
            status = mi_mesh_otp_program();
#else
            status = TEST_STATUS_ERROR;
#endif
            USART_IntClear(DTM_UART, USART_IF_RXDATAV);
            USART_IntEnable(DTM_UART, USART_IF_RXDATAV);
            break;

        default:
            status = TEST_STATUS_ERROR;
            break;
        }
        break;
		
    default:
        /* Unsupported setup command */
        status = TEST_STATUS_ERROR;
        break;
    }

    if ((cmd->control == SYSTEM_INFORMATION) ||
        (cmd->control == SETUP_CMD_CERTS_VERIFY) ||
        (cmd->control == SETUP_CMD_CUSTOMIZED)) {
        cfg.write_response_byte(0x00);
        cfg.write_response_byte(status);
    } else {
        send_test_status(status, response);
    }
    reset_cmd_buffer();
}

static void process_transceiver_command(enum cmd_type cmd_type,
        struct transceiver_cmd_packet *cmd)
{
    uint16_t length = (setup.transceiver_length_high << 6) | cmd->length;

    switch (cmd_type) {
    case CMD_TYPE_RXTEST:
        gecko_cmd_test_dtm_rx(cmd->frequency, setup.phy);
        break;

    case CMD_TYPE_TXTEST:
        gecko_cmd_test_dtm_tx(cmd->pkt, length, cmd->frequency, setup.phy);
        break;

    default:
        break;
    }
}

static void process_command()
{
    struct cmd_packet cmd_packet;
    parse_cmd_buffer(&cmd_packet);

    switch (cmd_packet.cmd_type) {
    case CMD_TYPE_SETUP:
        process_setup_command(cmd_packet.cmd_type, &cmd_packet.cmd.setup);
        break;

    case CMD_TYPE_RXTEST:
    case CMD_TYPE_TXTEST:
      process_transceiver_command(cmd_packet.cmd_type, &cmd_packet.cmd.transceiver);
      set_transceiver_test_state(cmd_packet.cmd_type);
      break;

    case CMD_TYPE_TESTEND:
        gecko_cmd_test_dtm_end();
        break;
    default:
        break;
    }
}

static void handle_dtm_completed(struct gecko_cmd_packet *evt)
{
    struct cmd_packet cmd_packet;
    parse_cmd_buffer(&cmd_packet);

    if (get_transceiver_test_state() == CMD_TYPE_RXTEST
            && cmd_packet.cmd_type == CMD_TYPE_TESTEND) {
        send_packet_counter(evt->data.evt_test_dtm_completed.number_of_packets);
    } else {
        send_test_status(evt->data.evt_test_dtm_completed.result, 0);
    }

    // Command is executed and the response is sent => reset command buffer for next command
    reset_cmd_buffer();
    if (cmd_packet.cmd_type == CMD_TYPE_TESTEND) {
        reset_transceiver_test_state();
    }
}

void DTM_UART_IRQ(void)
{
    testmode_process_command_byte(USART_Rx(DTM_UART));
}

static void uart_tx(uint8_t data)
{
    USART_Tx(DTM_UART, data);
}

static void dtm_uart_init(void)
{
    /* Enable peripheral clocks */
    CMU_ClockEnable(cmuClock_PCLK, true);
    CMU_ClockEnable(cmuClock_GPIO, true);
    CMU_ClockEnable(DTM_UART_CLK, true);

    USART_InitAsync_TypeDef init_uart   = USART_INITASYNC_DEFAULT;
    init_uart.enable                    = usartDisable;
#if defined(USART_INPUT_RXPRS) && defined(USART_CTRL_MVDIS)
    init_uart.mvdis                = 0;
    init_uart.prsRxEnable          = 0;
    init_uart.prsRxCh              = 0;
#endif
    USART_InitAsync(DTM_UART, &init_uart);

    USART_PrsTriggerInit_TypeDef initprs = USART_INITPRSTRIGGER_DEFAULT;
    initprs.prsTriggerChannel      = usartPrsTriggerCh0;
    USART_InitPrsTrigger(DTM_UART, &initprs);

#if defined(_SILICON_LABS_32B_SERIES_2)
    GPIO->USARTROUTE[USART_NUM(DTM_UART)].ROUTEEN = GPIO_USART_ROUTEEN_TXPEN
                                                         | GPIO_USART_ROUTEEN_RXPEN;
    GPIO->USARTROUTE[USART_NUM(DTM_UART)].TXROUTE =
      (DTM_UART_TX_PORT << _GPIO_USART_TXROUTE_PORT_SHIFT)
      | (DTM_UART_TX_PIN << _GPIO_USART_TXROUTE_PIN_SHIFT);
    GPIO->USARTROUTE[USART_NUM(DTM_UART)].RXROUTE =
      (DTM_UART_RX_PORT << _GPIO_USART_RXROUTE_PORT_SHIFT)
      | (DTM_UART_RX_PIN << _GPIO_USART_RXROUTE_PIN_SHIFT);
#else
    /* Enable pins at correct UART/USART location. */
    DTM_UART->ROUTEPEN = USART_ROUTEPEN_RXPEN | USART_ROUTEPEN_TXPEN;
    DTM_UART->ROUTELOC0 = (DTM_UART->ROUTELOC0
                 & (~(_USART_ROUTELOC0_RXLOC_MASK|_USART_ROUTELOC0_TXLOC_MASK)))
                 | (((uint32_t) DTM_UART_RX_LOC) << _USART_ROUTELOC0_RXLOC_SHIFT)
                 | (((uint32_t) DTM_UART_TX_LOC) << _USART_ROUTELOC0_TXLOC_SHIFT);
#endif

    /* Disable CTS */
    DTM_UART->CTRLX   = DTM_UART->CTRLX & (~USART_CTRLX_CTSEN);
    /* Set CTS active low */
    DTM_UART->CTRLX   = DTM_UART->CTRLX & (~USART_CTRLX_CTSINV);
    /* Set RTS active low */
    DTM_UART->CTRLX   = DTM_UART->CTRLX & (~USART_CTRLX_RTSINV);
    /* Set CS active low */
    DTM_UART->CTRL    = DTM_UART->CTRL & (~USART_CTRL_CSINV);
    /* Set TX active high */
    DTM_UART->CTRL    = DTM_UART->CTRL & (~USART_CTRL_TXINV);
    /* Set RX active high */
    DTM_UART->CTRL    = DTM_UART->CTRL & (~USART_CTRL_RXINV);

    /* Finally enable it */
    USART_Enable(DTM_UART, usartEnable);

    /* To avoid false start, configure output as high */
    GPIO_PinModeSet(DTM_UART_TX_PORT, DTM_UART_TX_PIN, gpioModePushPull, 1);
    GPIO_PinModeSet(DTM_UART_RX_PORT, DTM_UART_RX_PIN, gpioModeInput, 0);

    /* Enable RX interrupts */
    USART_IntEnable(DTM_UART, USART_IF_RXDATAV);
    NVIC_EnableIRQ(DTM_UART_IRQn);
}

void testmode_init(void)
{
#if defined(CRYOTIMER_PRESENT)
  CRYOTIMER_Init_TypeDef init = CRYOTIMER_INIT_DEFAULT;
  init.osc = cryotimerOscLFXO;
  init.presc = cryotimerPresc_1;
  init.period = cryotimerPeriod_4096m;
  CRYOTIMER_Init(&init);
#elif defined(BURTC_PRESENT)
  CMU_ClockEnable(cmuClock_BURTC, true);
  BURTC_Init_TypeDef burtcInit = BURTC_INIT_DEFAULT;
  BURTC_Init(&burtcInit);
#endif

#if HAVE_OTP_PKI
    mi_mesh_otp_manufacture_init(uart_block_send, uart_block_recv, NULL, NULL);
#endif

    dtm_uart_init();

    reset_setup();
    reset_cmd_buffer();
    reset_transceiver_test_state();
}

#include "mible_log.h"

void testmode_process_command_byte(uint8_t byte)
{
    if (cmd_buffer.len >= sizeof(cmd_buffer.data)) {
        // Processing previous command => ignore byte
        return;
    }

    uint32_t current_byte_time = cfg.get_ticks();

    if (cmd_buffer.len
        && current_byte_time - cmd_buffer.last_byte_time > t_min_in_ticks()) {
        // Enter byte timeout occurred
        reset_cmd_buffer();
    }

    cmd_buffer.last_byte_time = current_byte_time;
    cmd_buffer.data[cmd_buffer.len++] = byte;

    if (cmd_buffer.len == sizeof(cmd_buffer.data)) {
        gecko_external_signal(cfg.command_ready_signal);
    }
}

int testmode_handle_gecko_event(struct gecko_cmd_packet *evt)
{
    switch (BGLIB_MSG_ID(evt->header)) {
    case gecko_evt_system_external_signal_id:
      if (evt->data.evt_system_external_signal.extsignals & cfg.command_ready_signal) {
        process_command();
      }
      break;

    case gecko_evt_test_dtm_completed_id:
        handle_dtm_completed(evt);
        break;

    case gecko_evt_system_boot_id:
        gecko_cmd_system_set_tx_power(100);
        cfg.write_response_byte(0x00);
        cfg.write_response_byte(0x00);
        break;
    }

    return 0;
}

