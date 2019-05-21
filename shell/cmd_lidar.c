#include <stdlib.h>
#include <string.h>
#include "hal.h"
#include "chprintf.h"
#include "shellcommands.h"
#include "lidar.h"

void cmd_lidar(BaseSequentialStream *chp, int argc, char *argv[])
{
    uint8_t buf[1];

    if (argc == 1)
    {
        if (strcmp(argv[0], "ena") == 0)
        {
            buf[0] = 'b';
            lidarTransmit(1, buf);
            return;
        }
        else if (strcmp(argv[0], "dis") == 0)
        {
            buf[0] = 'e';
            lidarTransmit(1, buf);
            return;
        }
    }
    chprintf(chp, "lidar <ena|dis>\n\r");
}

