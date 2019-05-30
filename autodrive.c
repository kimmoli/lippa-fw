#include "autodrive.h"
#include "hal.h"
#include "servo.h"
#include "stepper.h"
#include "helpers.h"
#include "lidar.h"

#define SPEED 500

#define DIR_STOP      0
#define DIR_FWD       0x10
#define DIR_REV       0x20
#define DIR_LEFT      0x01
#define DIR_RIGHT     0x02

#define TURN_POINT 300
#define STOP_LIMIT 200
#define ADJ_TOL 50

virtual_timer_t adVt;
event_source_t autodrivePoll;
static uint8_t currentDir = DIR_STOP;


static void adVtCb(void *p)
{
    (void) p;

    chSysLockFromISR();
    chVTResetI(&adVt);
    chVTSetI(&adVt, MS2ST(250), adVtCb, NULL);
    chEvtBroadcastI(&autodrivePoll);
    chSysUnlockFromISR();
}

static void drive(uint8_t direction)
{
    int turn = 0;

    switch (direction & 0x30)
    {
        case DIR_FWD:
            setStepper(&STEPPERD1, SPEED, DIR_CW);
            setStepper(&STEPPERD2, SPEED, DIR_CW);
            setStepper(&STEPPERD3, SPEED, DIR_CW);
            setStepper(&STEPPERD4, SPEED, DIR_CW);
            turn = 500;
            break;
        case DIR_REV:
            setStepper(&STEPPERD1, SPEED, DIR_CCW);
            setStepper(&STEPPERD2, SPEED, DIR_CCW);
            setStepper(&STEPPERD3, SPEED, DIR_CCW);
            setStepper(&STEPPERD4, SPEED, DIR_CCW);
            turn = -500;
            break;
        default:
            setStepper(&STEPPERD1, 0, DIR_RETAIN);
            setStepper(&STEPPERD2, 0, DIR_RETAIN);
            setStepper(&STEPPERD3, 0, DIR_RETAIN);
            setStepper(&STEPPERD4, 0, DIR_RETAIN);
            turn = 0;
            break;
    }

    currentDir = direction;

    switch (direction & 0x03)
    {
        case DIR_LEFT:
            SERVOD1.value = 1500 + turn;
            SERVOD2.value = 1500 - turn;
            break;
        case DIR_RIGHT:
            SERVOD1.value = 1500 - turn;
            SERVOD2.value = 1500 + turn;
            break;
        default:
            SERVOD1.value = 1500;
            SERVOD2.value = 1500;
            break;
    }

    updateServo(&SERVOD1);
    updateServo(&SERVOD2);
}

static THD_FUNCTION(autodriveThread, arg)
{
    (void) arg;

    event_listener_t elAd;
    chEvtRegister(&autodrivePoll, &elAd, 8);

    while (!chThdShouldTerminateX())
    {
        chEvtWaitOne(EVENT_MASK(8));

        PRINT("dir %d d0=%d i0=%d d180=%d i180=%d\n\r", currentDir, LIDARD1.distance[0], LIDARD1.intensity[0],
                                                                    LIDARD1.distance[180/6], LIDARD1.intensity[180/6]);

        /* start moving to direction where is more free space, forward|backward */
        if (currentDir == DIR_STOP)
        {
            if (LIDARD1.distance[0] > STOP_LIMIT && LIDARD1.distance[0] > LIDARD1.distance[180/6])
            {
                drive(DIR_FWD);
            }
            else if (LIDARD1.distance[180/6] > STOP_LIMIT && LIDARD1.distance[180/6] > LIDARD1.distance[0])
            {
                drive(DIR_REV);
            }
            continue;
        }

        if (currentDir & DIR_FWD && LIDARD1.distance[0] > STOP_LIMIT)
        {
            if (LIDARD1.distance[60/6] > (LIDARD1.distance[300/6] + ADJ_TOL))
            {
                drive(DIR_FWD | DIR_RIGHT);
            }
            else if (LIDARD1.distance[300/6] > (LIDARD1.distance[60/6] + ADJ_TOL))
            {
                drive(DIR_FWD | DIR_LEFT);
            }
            else
            {
                drive(DIR_FWD);
            }
            continue;
        }

        if (currentDir & DIR_REV && LIDARD1.distance[180/6] > STOP_LIMIT)
        {
            if (LIDARD1.distance[120/6] > (LIDARD1.distance[240/6] + ADJ_TOL))
            {
                drive(DIR_REV | DIR_LEFT);
            }
            else if (LIDARD1.distance[240/6] > (LIDARD1.distance[120/6] + ADJ_TOL))
            {
                drive(DIR_REV | DIR_RIGHT);
            }
            else
            {
                drive(DIR_REV);
            }
            continue;
        }

        if (currentDir & DIR_FWD && LIDARD1.distance[0] < STOP_LIMIT)
        {
            drive(DIR_STOP);
        }
        if (currentDir & DIR_REV && LIDARD1.distance[180/6] < STOP_LIMIT)
        {
            drive(DIR_STOP);
        }

    }

    chThdExit(MSG_OK);
}


void initAutodrive(void)
{
    chEvtObjectInit(&autodrivePoll);
    chThdCreateFromHeap(NULL, THD_WORKING_AREA_SIZE(2048), "autodrive", NORMALPRIO+1, autodriveThread, NULL);
}

void autodriveStart(void)
{
    PRINT("Autodrive started\n");
    chVTSet(&adVt, MS2ST(250), adVtCb, NULL);
}

void autodriveStop(void)
{
    drive(DIR_STOP);
    chVTReset(&adVt);
}
