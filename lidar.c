#include <string.h>
#include "hal.h"
#include "chprintf.h"
#include "helpers.h"
#include "lidar.h"


static SerialConfig lidarConfig =
{
    /* speed */ 230400,
    /* CR1 */ 0,
    /* CR2 */ 0,
    /* CR3 */ 0
};

void lidarTransmit(int count, uint8_t * buf)
{
    int i;

    for (i=0 ; i<count ; i++)
    {
        chnPutTimeout(&SD1, *(buf+i), MS2ST(10));
        chThdSleepMicroseconds(10);
    }
}

void initLidar(void)
{
    sdStart(&SD1, &lidarConfig);
}

