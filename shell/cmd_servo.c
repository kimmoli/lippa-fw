#include <stdlib.h>
#include <string.h>
#include "hal.h"
#include "chprintf.h"
#include "shellcommands.h"
#include "servo.h"

void cmd_servo(BaseSequentialStream *chp, int argc, char *argv[])
{
    (void) chp;

    if (argc >= 2)
    {
        if (strtol(argv[0], NULL, 10) == 1)
        {
            setServo(&SERVOD1, strtol(argv[1], NULL, 10));
        }
        else if (strtol(argv[0], NULL, 10) == 2)
        {
            setServo(&SERVOD2, strtol(argv[1], NULL, 10));
        }
        else if (strtol(argv[0], NULL, 10) == 3)
        {
            setServo(&SERVOD3, strtol(argv[1], NULL, 10));
        }
    }
    else
    {
        chprintf(chp, "1: %d\n\r", SERVOD1.value);
        chprintf(chp, "2: %d\n\r", SERVOD2.value);
        chprintf(chp, "3: %d\n\r", SERVOD3.value);
        chprintf(chp, "servo [channel] [value in us %d...%d (1500 = center)]\n\r", SERVO_MIN, SERVO_MAX);
    }
}

