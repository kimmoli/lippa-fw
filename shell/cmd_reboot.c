#include <stdlib.h>
#include <string.h>
#include "hal.h"
#include "chprintf.h"
#include "shellcommands.h"

void cmd_reboot(BaseSequentialStream *chp, int argc, char *argv[])
{
    if (argc == 1)
    {
        if (strcmp(argv[0], "halt") == 0)
        {
            /*
             * This is to test watchdog
             */
            chprintf(chp, "Halting system...\n\r");
            chThdSleepMilliseconds(200);
            chSysHalt("die for me");
        }
        else
        {
            chprintf(chp, "reboot halt\n\r");
        }
        return;
    }

    chprintf(chp, "Rebooting...\n\r");
    chThdSleepMilliseconds(200);

    NVIC_SystemReset();
}

