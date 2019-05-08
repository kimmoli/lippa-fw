#ifndef _STEPPER_H
#define _STEPPER_H

struct StepperDriver
{
    PWMDriver *pwmp;
    uint32_t directionLine;
    int32_t currentFrequency;
    int32_t setFrequency;
    uint32_t currentDirection;
    uint32_t setDirection;
    bool noAccel;
};

typedef struct StepperDriver StepperDriver;

extern StepperDriver STEPPERD1;
extern StepperDriver STEPPERD2;
extern StepperDriver STEPPERD3;
extern StepperDriver STEPPERD4;

#define DIR_CW     PAL_LOW
#define DIR_CCW    PAL_HIGH
#define DIR_RETAIN 9U

#define RATIOD1 15
#define RATIOD2 15
#define RATIOD3 15
#define RATIOD4 15

extern StepperDriver *steppers[4];

extern void initStepper(void);
extern void setStepper(StepperDriver *stepp, int32_t frequency, uint32_t direction);

#endif
