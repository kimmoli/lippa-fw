#include <string.h>
#include "hal.h"
#include "chprintf.h"
#include "helpers.h"
#include "lidar.h"

LidarDriver LIDARD1;

static void lidarRxEnd(UARTDriver *uartp);
static uint8_t rxBuf[42];
event_source_t lidarEvent;
static bool lidarRunning = false;

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

        if (rxBuf[0] == 0xFA) // SYNC
        {
            uint8_t index = rxBuf[1] - 0xA0;

            if (index == 0x00) // Angle index 0
            {
                palToggleLine(LINE_LED_DEBUG);
            }

            uint16_t intensity = (rxBuf[5] << 8) | rxBuf[4];
            uint16_t distance = (rxBuf[7] << 8) | rxBuf[6];

            LIDARD1.intensity[index] = intensity;

            if (intensity > 10)
                LIDARD1.distance[index] = distance;
            else
                LIDARD1.distance[index] = 0xffff;
        }

        if (lidarRunning)
            uartStartReceive(&UARTD1, 42, rxBuf);
    }

    chThdExit(MSG_OK);
}

void controlLidar(bool enable)
{
    uint8_t buf[1];

    if (enable)
        buf[0] = 'b';
    else
        buf[0] = 'e';

    lidarTransmit(1, buf);

    if (enable)
        uartStartReceive(&UARTD1, 42, rxBuf);
    else
        uartStopReceive(&UARTD1);

    lidarRunning = enable;
}

void initLidar(void)
{
    for (int i=0 ; i < 60 ; i++)
    {
        LIDARD1.intensity[i] = 0x0;
        LIDARD1.distance[i] = 0xffff;
    }

    uartStart(&UARTD1, &lidarConfig);

    chEvtObjectInit(&lidarEvent);

    chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(2048), "lidar", NORMALPRIO+1, lidarThread, NULL);

}

int shortestBearingBetween(uint8_t min, uint8_t max)
{
    int ret = -1;
    int shortest = 0xffff;

    for (int i = min ; i<max ; i++)
    {
        if (LIDARD1.distance[i] < shortest)
        {
            ret = i;
            shortest = LIDARD1.distance[i];
        }
    }

    return ret;
}
