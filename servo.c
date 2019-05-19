#include "hal.h"
#include "servo.h"
#include "helpers.h"

ServoDriver SERVOD1;
ServoDriver SERVOD2;
ServoDriver SERVOD3;

const PWMConfig servoConfig =
{
    1000000,
    20000,
    NULL,
    {
        {PWM_OUTPUT_ACTIVE_HIGH, NULL},
        {PWM_OUTPUT_ACTIVE_HIGH, NULL},
        {PWM_OUTPUT_ACTIVE_HIGH, NULL},
        {PWM_OUTPUT_DISABLED, NULL},
    },
    0,
    0
};

/* Set servo output in us */
void setServo(ServoDriver *servop, uint32_t value)
{
    servop->value = value;
    updateServo(servop);
}

void updateServo(ServoDriver *servop)
{
    if (servop->value > SERVO_MAX)
    {
        servop->value = SERVO_MAX;
    }

    if (servop->value < SERVO_MIN)
    {
        servop->value = SERVO_MIN;
    }

    pwmEnableChannelI(servop->pwmp, servop->channel, servop->value);
}

void initServo(void)
{
    pwmStart(&PWMD3, &servoConfig);

    SERVOD1.pwmp = &PWMD3;
    SERVOD1.channel = 0;
    SERVOD1.value = 1500;

    SERVOD2.pwmp = &PWMD3;
    SERVOD2.channel = 1;
    SERVOD2.value = 1500;

    SERVOD3.pwmp = &PWMD3;
    SERVOD3.channel = 2;
    SERVOD3.value = 1500;

    palClearLine(LINE_ENABLE_PWM_N);

    updateServo(&SERVOD1);
    updateServo(&SERVOD2);
    updateServo(&SERVOD3);
}


