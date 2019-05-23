#ifndef LIDAR_H
#define LIDAR_H

struct LidarDriver
{
    uint16_t distance[4];
};

typedef struct LidarDriver LidarDriver;

extern LidarDriver LIDARD1;

extern void lidarTransmit(int count, uint8_t * buf);
extern void controlLidar(bool enable);
extern void initLidar(void);

#endif

