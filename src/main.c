/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "ch.h"
#include "hal.h"
#include "usbcfg.h"

/*
 * Internal loopback mode, 500KBaud, automatic wakeup, automatic recover
 * from abort mode.
 */
static const CANConfig cancfg =
{
    CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_NART | CAN_MCR_TXFP,
	CAN_BTR_LBKM |
	0x001c0002
};

/*
 * Transmitter thread.
 */
static THD_WORKING_AREA(can_tx_wa, 256);
static THD_FUNCTION(can_tx, p) {
    CANTxFrame txmsg;

    (void)p;
    chRegSetThreadName("transmitter");
    txmsg.IDE = CAN_IDE_EXT;
    txmsg.EID = 0x01234567;
    txmsg.RTR = CAN_RTR_DATA;
    txmsg.DLC = 8;
    txmsg.data32[0] = 0x11223344;
    txmsg.data32[1] = 0x55667788;

    while (true) {
        canTransmit(&CAND1, CAN_ANY_MAILBOX, &txmsg, TIME_MS2I(100));
        chThdSleepMilliseconds(500);
    }
}

inline char nibtoa(uint8_t nibble)
{
	if(nibble >= 0xA)
	{
		return nibble - 0xA + 'a';
	}
	else
	{
		return nibble + '0';
	}
}

inline void u8toa(uint8_t value, char* str)
{
	uint8_t hi = value >> 4;
	uint8_t lo = value & 0x0F;

	*str = nibtoa(hi);
	*(str + 1) = nibtoa(lo);
}

inline void u16toa(uint16_t value, char* str)
{
	uint8_t hi = value >> 8;
	uint8_t lo = value & 0xFF;

	// A uint16 is just two uint8s
	u8toa(hi, str);
	u8toa(lo, str + 2);
}

inline void u32toa(uint32_t value, char* str)
{
	uint16_t hi = value >> 16;
	uint16_t lo = value & 0xffff;

	u16toa(hi, str);
	u16toa(lo, str + 4);
}

/*
 * Application entry point.
 */
int main(void) {

    /*
    * System initializations.
    * - HAL initialization, this also initializes the configured device drivers
    *   and performs the board-specific initializations.
    * - Kernel initialization, the main() function becomes a thread and the
    *   RTOS is active.
    */
    halInit();
    chSysInit();

    canStart(&CAND1, &cancfg);

    palSetPadMode(GPIOB, 8, PAL_MODE_ALTERNATE(4));
    palSetPadMode(GPIOB, 9, PAL_MODE_ALTERNATE(4));

    /*
    * Initializes a serial-over-USB CDC driver.
    */
    sduObjectInit(&SDU1);
    sduStart(&SDU1, &serusbcfg);

    /*
    * Activates the USB driver and then the USB bus pull-up on D+.
    * Note, a delay is inserted in order to not have to disconnect the cable
    * after a reset.
    */
    usbDisconnectBus(serusbcfg.usbp);
    chThdSleepMilliseconds(1500);
    usbStart(serusbcfg.usbp, &usbcfg);
    usbConnectBus(serusbcfg.usbp);

    palSetPadMode(GPIOB, 13, PAL_MODE_OUTPUT_PUSHPULL);

    event_listener_t el;
    CANRxFrame rxmsg;

    chThdCreateStatic(can_tx_wa, sizeof(can_tx_wa), NORMALPRIO + 7, can_tx, NULL);


    chEvtRegister(&CAND1.rxfull_event, &el, 0);
    while (true) {
        if(chEvtWaitAny(ALL_EVENTS) == 0)
            continue;
        while(canReceive(&CAND1, CAN_ANY_MAILBOX, &rxmsg, TIME_IMMEDIATE) == MSG_OK)
        {
            palTogglePad(GPIOB, 13);

            char buf[26];
            memset(buf, 0, 22);

            u32toa(rxmsg.EID, buf);

            buf[8] = ':';

            for(int i = 0; i < 8; i++)
            {
                u8toa(rxmsg.data8[i], &buf[i * 2 + 9]);
            }

            buf[25] = '\n';

            chnWrite(&SDU1, buf, 26);
        }
    }
}
