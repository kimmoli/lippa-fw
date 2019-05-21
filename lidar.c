#include <string.h>
#include "hal.h"
#include "chprintf.h"
#include "helpers.h"
#include "lidar.h"

static void lidarRxEnd(UARTDriver *uartp);
static uint8_t rxBuf[42];
event_source_t lidarEvent;

static UARTConfig lidarConfig =
{
    NULL,
    NULL,
    lidarRxEnd,
    NULL,
    NULL,
    /* speed */ 230400,
    /* CR1 */ 0,
    /* CR2 */ USART_CR2_LINEN,
    /* CR3 */ 0
};

static void lidarRxEnd(UARTDriver *uartp)
{
    chSysLockFromISR();
    chEvtBroadcastI(&lidarEvent);
    chSysUnlockFromISR();
}

void lidarTransmit(int count, uint8_t * buf)
{
    uartStartSend(&UARTD1, count, buf);
}

static THD_FUNCTION(lidarThread, arg)
{
    (void) arg;

    event_listener_t elLidar;
    chEvtRegister(&lidarEvent, &elLidar, 8);

    while (!chThdShouldTerminateX())
    {
        chEvtWaitOne(EVENT_MASK(8));

        palToggleLine(LINE_LED_DEBUG);

        if (rxBuf[0] == 0xFA) // SYNC
        {
            if (rxBuf[1] == 0xA0) // Angle index 0
            {
                PRINT("%5d ", (rxBuf[7] << 8) | rxBuf[6]);
            }
            if (rxBuf[1] == 0xAF) // Angle index 15
            {
                PRINT("%5d ", (rxBuf[7] << 8) | rxBuf[6]);
            }
            if (rxBuf[1] == 0xBE) // Angle index 30
            {
                PRINT("%5d ", (rxBuf[7] << 8) | rxBuf[6]);
            }
            if (rxBuf[1] == 0xCD) // Angle index 45
            {
                PRINT("%5d\r\n", (rxBuf[7] << 8) | rxBuf[6]);
            }
        }

        uartStartReceive(&UARTD1, 42, rxBuf);
    }

    chThdExit(MSG_OK);
}

void initLidar(void)
{
    uartStart(&UARTD1, &lidarConfig);

    chEvtObjectInit(&lidarEvent);

    chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(2048), "lidar", NORMALPRIO+1, lidarThread, NULL);

    uartStartReceive(&UARTD1, 42, rxBuf);
}

