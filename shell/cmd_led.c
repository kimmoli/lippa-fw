#include <stdlib.h>
#include <string.h>
#include "hal.h"
#include "chprintf.h"
#include "shellcommands.h"

void cmd_led(BaseSequentialStream *chp, int argc, char *argv[])
{
    if (argc == 1)
    {
        if (strcmp(argv[0], "err") == 0)
        {
            palToggleLine(LINE_LED_ERR);
            chprintf(chp, "Error led toggled\n\r");
            return;
        }
        else if (strcmp(argv[0], "dbg") == 0)
        {
            palToggleLine(LINE_LED_DEBUG);
            chprintf(chp, "Debug led toggled\n\r");
            return;
        }
    }
    chprintf(chp, "led <err|dbg>\n\r");
}

