#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "shell.h"

#include "helpers.h"
#include "shellcommands.h"
#include "stepper.h"
#include "servo.h"
#include "spi.h"
#include "ps2.h"

int main(void)
{
    halInit();
    chSysInit();

//    startWdogKick(WDOG_KICK_INTERVAL);

    sdStart(&SD3, NULL);  /* Serial console in USART3, 115200 */

    consoleStream = (BaseSequentialStream *) &SD3;

    PRINT("\n\r");
    PRINT("\n\rLIPPA");
    PRINT("\n\r-----");
    PRINT("\n\r\n\r");

    initSpi();
    initPS2();
    initStepper();
    initServo();

    shellInit();
    chThdCreateFromHeap(NULL, SHELL_WA_SIZE, "shell", NORMALPRIO + 1, shellThread, (void *)&shell_cfg_uart);

    palClearLine(LINE_LED_ERR);

    while (true)
    {
        chThdSleepMilliseconds(200);
        palToggleLine(LINE_LED_RUN);
    }
}
