#ifndef IMU_DATA_H
#define IMU_DATA_H

struct ImuData {
    double roll;
    double pitch;
    double yaw;
};

extern ImuData imuData;

#endif // IMU_DATA_H