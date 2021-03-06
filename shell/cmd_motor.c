#include <stdlib.h>
#include <string.h>
#include "hal.h"
#include "chprintf.h"
#include "shellcommands.h"
#include "stepper.h"

void cmd_motor(BaseSequentialStream *chp, int argc, char *argv[])
{
    uint8_t motor = 0;

    if (argc == 1)
    {
        if (strcmp(argv[0], "sleep") == 0)
        {
            palToggleLine(LINE_SLEEP_N);
        }
    }
    else if (argc >= 2)
    {
        motor = strtol(argv[0], NULL, 0);

        if (strcmp(argv[1], "en") == 0)
        {
            if (motor == 1 || motor == 0)
                palToggleLine(LINE_ENA_1_N);
            if (motor == 2 || motor == 0)
                palToggleLine(LINE_ENA_2_N);
            if (motor == 3 || motor == 0)
                palToggleLine(LINE_ENA_3_N);
            if (motor == 4 || motor == 0)
                palToggleLine(LINE_ENA_4_N);
        }
        else if (strcmp(argv[1], "dir") == 0)
        {
            if (motor == 1 || motor == 0)
                setStepperDirection(&STEPPERD1, DIR_TOGGLE);
            if (motor == 2 || motor == 0)
                setStepperDirection(&STEPPERD2, DIR_TOGGLE);
            if (motor == 3 || motor == 0)
                setStepperDirection(&STEPPERD3, DIR_TOGGLE);
            if (motor == 4 || motor == 0)
                setStepperDirection(&STEPPERD4, DIR_TOGGLE);
        }
        else if (strcmp(argv[1], "mode") == 0)
        {
            if (motor == 1 || motor == 0)
                palToggleLine(LINE_MODE_1);
            if (motor == 2 || motor == 0)
                palToggleLine(LINE_MODE_2);
            if (motor == 3 || motor == 0)
                palToggleLine(LINE_MODE_3);
            if (motor == 4 || motor == 0)
                palToggleLine(LINE_MODE_4);
        }
        else if (strcmp(argv[1], "set") == 0 && argc == 3)
        {
            if (motor == 1 || motor == 0)
                setStepper(&STEPPERD1, strtol(argv[2], NULL, 10), DIR_RETAIN);
            if (motor == 2 || motor == 0)
                setStepper(&STEPPERD2, strtol(argv[2], NULL, 10), DIR_RETAIN);
            if (motor == 3 || motor == 0)
                setStepper(&STEPPERD3, strtol(argv[2], NULL, 10), DIR_RETAIN);
            if (motor == 4 || motor == 0)
                setStepper(&STEPPERD4, strtol(argv[2], NULL, 10), DIR_RETAIN);
        }
    }
    else
    {
        chprintf(chp, "mot <#|sleep> <en|dir|mode>\n\n\r");
    }

    chprintf(chp, "SLEEP %d, FAULT %d\n\n\r", palReadLine(LINE_SLEEP_N), palReadLine(LINE_FAULT_N));
    chprintf(chp, "M1 EN %d, DIR %d, MODE %d\n\r", palReadLine(LINE_ENA_1_N),
                                                 palReadLine(LINE_DIR_1),
                                                 palReadLine(LINE_MODE_1));
    chprintf(chp, "M2 EN %d, DIR %d, MODE %d\n\r", palReadLine(LINE_ENA_2_N),
                                                 palReadLine(LINE_DIR_2),
                                                 palReadLine(LINE_MODE_2));
    chprintf(chp, "M3 EN %d, DIR %d, MODE %d\n\r", palReadLine(LINE_ENA_3_N),
                                                 palReadLine(LINE_DIR_3),
                                                 palReadLine(LINE_MODE_3));
    chprintf(chp, "M4 EN %d, DIR %d, MODE %d\n\r", palReadLine(LINE_ENA_4_N),
                                                 palReadLine(LINE_DIR_4),
                                                 palReadLine(LINE_MODE_4));
}

