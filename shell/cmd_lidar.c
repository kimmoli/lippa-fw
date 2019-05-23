#include <stdlib.h>
#include <string.h>
#include "hal.h"
#include "chprintf.h"
#include "shellcommands.h"
#include "lidar.h"

void cmd_lidar(BaseSequentialStream *chp, int argc, char *argv[])
{
    if (argc == 1)
    {
        if (strcmp(argv[0], "ena") == 0)
        {
            controlLidar(true);
            return;
        }
        else if (strcmp(argv[0], "dis") == 0)
        {
            controlLidar(false);
            return;
        }
    }
    chprintf(chp, "lidar <ena|dis>\n\r");
}

