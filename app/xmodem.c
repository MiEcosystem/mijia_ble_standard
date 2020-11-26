/**
 * @file	: xmodem.c
 * @author	: songyu5@xiaomi.com
 * @brief	:
 * @version : 0.1
 * @date	: 2019-09-10
 *
 * @copyright Copyright (c) 2019
 *
 */

#include "xmodem.h"

#if (defined(MCU_OTA_DEMO) && MCU_OTA_DEMO)
#include "usart_api.h"
#include <stddef.h>
#include <string.h>
#include "mible_type.h"
#include "mible_log.h"
#include "efr32_api.h"

/* for ota debug */
#define DEBUG_OTA 0
/* Xmodem flag definition */
#define SOH		(0x01)
#define STX		(0x02)
#define EOT		(0x04)
#define ACK		(0x06)
#define NAK		(0x15)
#define CAN		(0x18)
#define CTRLZ	(0x1A)

#define DLY_1S 200
#define MAXRETRANS 25
//#define TRANSMIT_XMODEM_1K

/* Xmodem definition */
#define XMODEM_MTU              (128)
#define XMODEM_HEADER_POS       (0)
#define XMODEM_PACKNO_POS       (1)
#define XMODEM_DATA_POS         (3)
#define XMODEM_CRC_POS          (131)
/* Xmodem-1k definition */
#define XMODEM_1K_MTU           (1024)
#define XMODEM_1K_HEADER_POS    (0)
#define XMODEM_1K_PACKNO_POS    (1)
#define XMODEM_1K_DATA_POS      (3)
#define XMODEM_1K_CRC_POS       (1027)

#define XMODEM_RECV_TIMEOUT     (1 * 100)//100ms
#define XMODEM_INTERVAL_MS      (100)
#define TRANS_RETRY_MAX         (20)
#define PACK_RETRY_MAX          (10)
#define CRC_RETRY_MAX           (10)

/* =================================================================== */
/* ========================= miio xmodem start ======================= */
/* =================================================================== */

#define XMODEM_OK			MI_SUCCESS
#define XMODEM_ERR			MI_ERR_INTERNAL
#define XMODEM_PARAM_ERR	MI_ERR_INVALID_PARAM

static unsigned short crc16_ccitt(const unsigned char* buf, int len)
{
	unsigned short crc = 0;
	while (len--) {
		int i;
		crc ^= *buf++ << 8;

		for (i = 0; i < 8; ++i) {
			if (crc & 0x8000)
				crc = (crc << 1) ^ 0x1021;
			else
				crc = crc << 1;
		}
	}
	return crc;
}
/*
static int check(int crc, const unsigned char *buf, int sz)
{
    if (crc) {
        unsigned short crc = crc16_ccitt(buf, sz);
        unsigned short tcrc = (buf[sz]<<8)+buf[sz+1];
        if (crc == tcrc)
            return 1;
    }
    else {
        int i;
        unsigned char cks = 0;
        for (i = 0; i < sz; ++i) {
            cks += buf[i];
        }
        if (cks == buf[sz])
        return 1;
    }

    return 0;
}*/

int _inbyte(unsigned short timeout)
{
    uint8_t c;
    if(MI_SUCCESS == mible_usart_receive_byte_timeout(&c, timeout))
        return c;
    else
        return -1;
}


void _outbyte(int c)
{
    mible_usart_send_byte((uint8_t)c);
}

static void flushinput(void)
{
    //while (_inbyte(((DLY_1S)*3)>>1) >= 0)
    //    ;
    mible_usart_flush(false,true);
}

int xmodem_transfer_data(miio_xmodem_t *x, unsigned char *src, int srcsz)
{
    //unsigned char xbuff[1030]; // 1024 for XModem 1k + 3 head chars + 2 crc + nul */
    unsigned char *xbuff = x->xbuff;
    int bufsz, crc = -1;
    unsigned char packetno = 1;
    int i, c, len = 0;
    int retry;

    for(;;) {
        for( retry = 0; retry < 16; ++retry) {
            if ((c = _inbyte((DLY_1S)<<1)) >= 0) {
                switch (c) {
                case 'C':
                    crc = 1;
                    goto start_trans;
                case NAK:
                    crc = 0;
                    goto start_trans;
                case CAN:
                    if ((c = _inbyte(DLY_1S)) == CAN) {
                        _outbyte(ACK);
                        flushinput();
                        return -1; /* canceled by remote */
                    }
                    break;
                default:
                    break;
                }
            }
        }
        _outbyte(CAN);
        _outbyte(CAN);
        _outbyte(CAN);
        flushinput();
        return -2; /* no sync */

        for(;;) {
        start_trans:
#ifdef TRANSMIT_XMODEM_1K
            xbuff[0] = STX; bufsz = 1024;
#else
            xbuff[0] = SOH; bufsz = 128;
#endif
            xbuff[1] = packetno;
            xbuff[2] = ~packetno;
            c = srcsz - len;
            MI_LOG_DEBUG("XMODEM start_trans packetno = %d, c = %d, len = %d\n", packetno,c,len);
            if (c > bufsz) c = bufsz;
            if (c > 0) {
                memset (&xbuff[3], 0, bufsz);
                memcpy (&xbuff[3], &src[len], c);
                if (c < bufsz) xbuff[3+c] = CTRLZ;
                if (crc) {
                    unsigned short ccrc = crc16_ccitt(&xbuff[3], bufsz);
                    xbuff[bufsz+3] = (ccrc>>8) & 0xFF;
                    xbuff[bufsz+4] = ccrc & 0xFF;
                }
                else {
                    unsigned char ccks = 0;
                    for (i = 3; i < bufsz+3; ++i) {
                        ccks += xbuff[i];
                    }
                    xbuff[bufsz+3] = ccks;
                }
                for (retry = 0; retry < MAXRETRANS; ++retry) {
                    MI_LOG_DEBUG("XMODEM send byte retry = %d\n", retry);
                    MI_LOG_HEXDUMP(x->xbuff, bufsz+4+(crc?1:0));
                    for (i = 0; i < bufsz+4+(crc?1:0); ++i) {
                        _outbyte(xbuff[i]);
                    }
                    if ((c = _inbyte(DLY_1S)) >= 0 ) {
                        switch (c) {
                        case ACK:
                            ++packetno;
                            len += bufsz;
                            goto start_trans;
                        case CAN:
                            if ((c = _inbyte(DLY_1S)) == CAN) {
                                _outbyte(ACK);
                                flushinput();
                                return -1; /* canceled by remote */
                            }
                            break;
                        case NAK:
                        default:
                            break;
                        }
                    }
                }
                _outbyte(CAN);
                _outbyte(CAN);
                _outbyte(CAN);
                flushinput();
                return -4; /* xmit error */
            }
            else {
                for (retry = 0; retry < 10; ++retry) {
                    MI_LOG_DEBUG("XMODEM send EOT retry %d\n", retry);
                    _outbyte(EOT);
                    if ((c = _inbyte((DLY_1S)<<1)) == ACK) break;
                }
                flushinput();
                MI_LOG_DEBUG("XMODEM return %d\n", (c == ACK)?len:-5);
                return (c == ACK)?len:-5;
            }
        }
    }
    return -6;
}

int miio_xmodem_create_instance(miio_xmodem_t *xmodem)
{
	int ret = XMODEM_OK;

	if(NULL == xmodem){
		MI_LOG_ERROR("uart or xmodem pointer is NULL");
		return XMODEM_PARAM_ERR;
	}

	xmodem->type = XMODEM;

	return ret;
}

void miio_xmodem_destroy(miio_xmodem_t *x)
{
	MI_LOG_INFO("destroy xmodem instance");
	return;
}

#endif /* (defined(USER_OTA_ENABLE) && USER_OTA_ENABLE) */
