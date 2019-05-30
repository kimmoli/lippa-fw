#include "hal.h"
#include "ps2.h"
#include "stepper.h"
#include "servo.h"
#include "lidar.h"
#include <stdlib.h>
#include "helpers.h"
#include "autodrive.h"

#define RATIO_STEPPER 25
#define RATIO_SERVO 2

PS2Values_t *PS2Values;
event_source_t PS2Poll;
static uint8_t rx[100] = {0};
static bool lidarState = false;
static bool autodrive = false;

//uint8_t poll[]={0x01,0x42,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint8_t poll[]={0x01,0x42,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

uint8_t enter_config[]={0x01,0x43,0x00,0x01,0x00};
uint8_t set_mode[]={0x01,0x44,0x00,0x01,0x03,0x00,0x00,0x00,0x00};
uint8_t set_bytes_large[]={0x01,0x4F,0x00,0xFF,0xFF,0x03,0x00,0x00,0x00};
uint8_t exit_config[]={0x01,0x43,0x00,0x00,0x5A,0x5A,0x5A,0x5A,0x5A};
uint8_t enable_rumble[]={0x01,0x4D,0x00,0x00,0x01};

static void gptps2cb(GPTDriver *gpt_ptr);

const GPTConfig gptps2cfg =
{
    1000,      // timer clock: 1 kHz ~ 1 ms
    gptps2cb,  // Timer callback function
    0,
    0
};

void sendPS2(uint8_t *tx, int count, uint8_t *rx)
{
    int i;

    palClearLine(LINE_SPI_CS_PS2_N);

    for (i=0 ; i < count; i++)
    {
        spiExchange(&SPID1, 1, tx+i, rx+i);
    }

    palSetLine(LINE_SPI_CS_PS2_N);
}

void sendPS2Config(void)
{
    sendPS2(enter_config, sizeof(enter_config), rx);
    chThdSleepMilliseconds(100);
    sendPS2(set_mode, sizeof(set_mode), rx);
    chThdSleepMilliseconds(100);
    sendPS2(set_bytes_large, sizeof(set_bytes_large), rx);
    chThdSleepMilliseconds(100);
    sendPS2(enable_rumble, sizeof(enable_rumble), rx);
    chThdSleepMilliseconds(100);
    sendPS2(exit_config, sizeof(exit_config), rx);
    chThdSleepMilliseconds(10);

    PS2Values->reconfigCount++;
}

static THD_FUNCTION(PS2Thread, arg)
{
    (void)arg;
    event_listener_t elPS2;
    bool sel = false;
    bool start = false;
    bool l1 = false;

    chEvtRegister(&PS2Poll, &elPS2, 9);

    while (!chThdShouldTerminateX())
    {
        chEvtWaitOne(EVENT_MASK(9));

        if (PS2Values->reconfig)
        {
            sendPS2Config();
            PS2Values->reconfig = 0;
            continue;
        }

        poll[3] = PS2Values->motor1;
        poll[4] = PS2Values->motor2;

        sendPS2(poll, sizeof(poll), rx);

        if (rx[1] == 0x79)
        {
            PS2Values->buttons = ~((rx[3] << 8) | rx[4]);
            PS2Values->analog_right_hor = (int)rx[5] - 128;
            PS2Values->analog_right_ver = (int)rx[6] - 128;
            PS2Values->analog_left_hor = (int)rx[7] - 128;
            PS2Values->analog_left_ver = (int)rx[8] - 128;
            PS2Values->pressure_right = (int)rx[9];
            PS2Values->pressure_left = (int)rx[10];
            PS2Values->pressure_up = (int)rx[11];
            PS2Values->pressure_down = (int)rx[12];
            PS2Values->pressure_triangle = (int)rx[13];
            PS2Values->pressure_circle = (int)rx[14];
            PS2Values->pressure_x = (int)rx[15];
            PS2Values->pressure_square = (int)rx[16];
        }
        else
        {
            PS2Values->reconfig = 1;
        }

        PS2Values->count++;

        if (PS2Values->buttons & BUTTON_START)
        {
            PS2Values->motor2 = 255;

            if (!start)
            {
                start = true;

                palToggleLine(LINE_ENA_1_N);
                palToggleLine(LINE_ENA_2_N);
                palToggleLine(LINE_ENA_3_N);
                palToggleLine(LINE_ENA_4_N);
                palToggleLine(LINE_SLEEP_N);

                SERVOD1.value = 1500;
                SERVOD2.value = 1500;

                updateServo(&SERVOD1);
                updateServo(&SERVOD2);

                chThdSleepMilliseconds(100);

                palToggleLine(LINE_ENABLE_PWM_N);
            }
        }
        else
        {
            PS2Values->motor2 = 0;
            start = false;
        }

        if (PS2Values->buttons & BUTTON_SELECT)
        {
            if (!sel)
            {
                sel = true;

                lidarState = !lidarState;

                controlLidar(lidarState);
            }
        }
        else
        {
            sel = false;
        }

        if (PS2Values->buttons & BUTTON_L1)
        {
            if (!l1)
            {
                l1 = true;
                autodrive = !autodrive;

                if (autodrive)
                    autodriveStart();
                else
                    autodriveStop();
            }
        }
        else
        {
            l1 = false;
        }

        if (autodrive)
            continue;

        if (PS2Values->buttons & BUTTON_UP && LIDARD1.distance[30] > 200)
        {
            setStepper(&STEPPERD1, RATIO_STEPPER * PS2Values->pressure_up + 2*MAX(1500 - SERVOD1.value, 0), DIR_CCW);
            setStepper(&STEPPERD2, RATIO_STEPPER * PS2Values->pressure_up + 2*MAX(1500 - SERVOD1.value, 0), DIR_CCW);
            setStepper(&STEPPERD3, RATIO_STEPPER * PS2Values->pressure_up + 2*MAX(SERVOD1.value - 1500, 0), DIR_CCW);
            setStepper(&STEPPERD4, RATIO_STEPPER * PS2Values->pressure_up + 2*MAX(SERVOD1.value - 1500, 0), DIR_CCW);
        }
        else if (PS2Values->buttons & BUTTON_DOWN && LIDARD1.distance[0] > 200)
        {
            setStepper(&STEPPERD1, RATIO_STEPPER * PS2Values->pressure_down + 2*MAX(1500 - SERVOD1.value, 0), DIR_CW);
            setStepper(&STEPPERD2, RATIO_STEPPER * PS2Values->pressure_down + 2*MAX(1500 - SERVOD1.value, 0), DIR_CW);
            setStepper(&STEPPERD3, RATIO_STEPPER * PS2Values->pressure_down + 2*MAX(SERVOD1.value - 1500, 0), DIR_CW);
            setStepper(&STEPPERD4, RATIO_STEPPER * PS2Values->pressure_down + 2*MAX(SERVOD1.value - 1500, 0), DIR_CW);
        }
        else
        {
            setStepper(&STEPPERD1, 0, DIR_RETAIN);
            setStepper(&STEPPERD2, 0, DIR_RETAIN);
            setStepper(&STEPPERD3, 0, DIR_RETAIN);
            setStepper(&STEPPERD4, 0, DIR_RETAIN);
        }

        if (PS2Values->buttons & BUTTON_SQUARE)
        {
            SERVOD1.value = 1500 - MAX(0, PS2Values->pressure_square - 100) * RATIO_SERVO;
            SERVOD2.value = 1500 + MAX(0, PS2Values->pressure_square - 100)* RATIO_SERVO;
        }
        else if (PS2Values->buttons & BUTTON_CIRCLE)
        {
            SERVOD1.value = 1500 + MAX(0, PS2Values->pressure_circle - 100) * RATIO_SERVO;
            SERVOD2.value = 1500 - MAX(0, PS2Values->pressure_circle - 100) * RATIO_SERVO;
        }
        else
        {
            SERVOD1.value = 1500;
            SERVOD2.value = 1500;
        }

        updateServo(&SERVOD1);
        updateServo(&SERVOD2);
    }

    chThdExit(MSG_OK);
}


void gptps2cb(GPTDriver *gpt_ptr)
{
    (void) gpt_ptr;

    osalSysLockFromISR();
    chEvtBroadcastI(&PS2Poll);
    osalSysUnlockFromISR();
}

void initPS2(void)
{
    chEvtObjectInit(&PS2Poll);

    PS2Values = chHeapAlloc(NULL, sizeof(PS2Values_t));

    PS2Values->buttons = 0;
    PS2Values->reconfig = 0;
    PS2Values->analog_left_hor = 0;
    PS2Values->analog_left_ver = 0;
    PS2Values->analog_right_hor = 0;
    PS2Values->analog_right_ver = 0;
    PS2Values->pressure_right = 0;
    PS2Values->pressure_left = 0;
    PS2Values->pressure_up = 0;
    PS2Values->pressure_down = 0;
    PS2Values->pressure_triangle = 0;
    PS2Values->pressure_circle = 0;
    PS2Values->pressure_x = 0;
    PS2Values->pressure_square = 0;
    PS2Values->motor1 = 0;
    PS2Values->motor2 = 0;
    PS2Values->count = 0;
    PS2Values->reconfigCount = 0;

    sendPS2Config();

    chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(512), "ps2", NORMALPRIO+1, PS2Thread, NULL);

    gptStart(&GPTD12, &gptps2cfg);
    gptStartContinuous(&GPTD12, PS2_UPDATE_INTERVAL);
}

