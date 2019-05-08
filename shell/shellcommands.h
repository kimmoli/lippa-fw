#ifndef TK_SHELLCOMMANDS_H
#define TK_SHELLCOMMANDS_H

#include "hal.h"
#include "shell.h"

#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(2048)

extern void cmd_reboot(BaseSequentialStream *chp, int argc, char *argv[]);
extern void cmd_led(BaseSequentialStream *chp, int argc, char *argv[]);
extern void cmd_motor(BaseSequentialStream *chp, int argc, char *argv[]);

extern const ShellCommand commands[];
extern const ShellConfig shell_cfg_uart;

#endif // TK_SHELLCOMMANDS_H
