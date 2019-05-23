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
    bool invertDirection;
};

typedef struct StepperDriver StepperDriver;

extern StepperDriver STEPPERD1;
extern StepperDriver STEPPERD2;
extern StepperDriver STEPPERD3;
extern StepperDriver STEPPERD4;

#define DIR_CW     PAL_LOW
#define DIR_CCW    PAL_HIGH
#define DIR_TOGGLE 8U
#define DIR_RETAIN 9U

#define RATIOD1 25
#define RATIOD2 25
#define RATIOD3 25
#define RATIOD4 25

#define STEPPER_UPDATE_INTERVAL 10
#define STEPPER_ACCEL 30
#define STEPPER_DECEL 300

extern StepperDriver *steppers[4];

extern void initStepper(void);
extern void setStepper(StepperDriver *stepp, int32_t frequency, uint32_t direction);
extern void setStepperDirection(StepperDriver *stepp, uint32_t direction);

#endif

