#ifndef _SERVO_H
#define _SERVO_H

struct ServoDriver
{
    PWMDriver *pwmp;
    uint32_t channel;
    uint32_t value;
};

#define SERVO_MAX 2000
#define SERVO_MIN 1000

typedef struct ServoDriver ServoDriver;

extern ServoDriver SERVOD1;
extern ServoDriver SERVOD2;
extern ServoDriver SERVOD3;

extern void initServo(void);
extern void setServo(ServoDriver *stepp, uint32_t value);
extern void updateServo(ServoDriver *servop);

#endif

