#ifndef _PS2_H
#define _PS2_H

#define BUTTON_LEFT     0x8000
#define BUTTON_DOWN     0x4000
#define BUTTON_RIGHT    0x2000
#define BUTTON_UP       0x1000
#define BUTTON_START    0x0800
#define BUTTON_RANALOG  0x0400
#define BUTTON_LANALOG  0x0200
#define BUTTON_SELECT   0x0100
#define BUTTON_SQUARE   0x0080
#define BUTTON_X        0x0040
#define BUTTON_CIRCLE   0x0020
#define BUTTON_TRIANGLE 0x0010
#define BUTTON_R1       0x0008
#define BUTTON_L1       0x0004
#define BUTTON_R2       0x0002
#define BUTTON_L2       0x0001

#define PS2_UPDATE_INTERVAL 10

typedef struct
{
    uint32_t count;
    uint32_t reconfigCount;
    uint16_t buttons;
    int analog_left_hor;
    int analog_left_ver;
    int analog_right_hor;
    int analog_right_ver;
    uint8_t reconfig;
    uint8_t motor1;
    uint8_t motor2;
    int pressure_up;
    int pressure_down;
    int pressure_left;
    int pressure_right;
    int pressure_triangle;
    int pressure_x;
    int pressure_square;
    int pressure_circle;
} PS2Values_t;

extern PS2Values_t *PS2Values;

void initPS2(void);

#endif // _PS2_H
