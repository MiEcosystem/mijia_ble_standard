/*
 * key.h
 *
 *  Created on: 2020Äê9ÔÂ9ÈÕ
 *      Author: mi
 */

#ifndef PERIPHERAL_KEY_H_
#define PERIPHERAL_KEY_H_

// Number of ticks after which press is considered to be long (1s)
#define LONG_PRESS_TIME_TICKS           (32768)
#define EXT_SIGNAL_PB0_SHORT_PRESS      0x01
#define EXT_SIGNAL_PB0_LONG_PRESS       0x02

void button_init(void);

#endif /* PERIPHERAL_KEY_H_ */
