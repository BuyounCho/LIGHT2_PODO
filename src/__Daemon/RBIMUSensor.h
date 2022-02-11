#ifndef RBIMUSENSOR_H
#define RBIMUSENSOR_H

#include "RBDataType.h"
#include "RBCAN.h"
#include <cmath>

class RBIMUSensor
{
public:
    RBIMUSensor();

    // from DB----
    int     BOARD_ID;
    QString BOARD_NAME;
    int     CAN_CHANNEL;
    int     ID_SEND_DATA;
    int     ID_RCV_GENERAL;
    int     ID_RCV_DATA_QUAT;
    int     ID_RCV_DATA_LOCAL_X;
    int     ID_RCV_DATA_LOCAL_Y;
    int     ID_RCV_DATA_LOCAL_Z;

    // not from DB----
    int     ConnectionStatus;

    // sensor data----    
    // Current raw data   
    int     ROLL_DIGIT;
    int     PITCH_DIGIT;
    int     YAW_DIGIT;
    int     ROLL_VEL_DIGIT;
    int     PITCH_VEL_DIGIT;
    int     YAW_VEL_DIGIT;

    // Currnet data
    double   ROLL;
    double   PITCH;
    double   YAW;
    double   ROLL_VEL;
    double   PITCH_VEL;
    double   YAW_VEL;
    double   ACC_X;
    double   ACC_Y;
    double   ACC_Z;

    // Offset value for angles
    double   ROLL_OFFSET;
    double   PITCH_OFFSET;
    double   YAW_OFFSET;

    double   FOG_ROLL_OFFSET;
    double   FOG_PITCH_OFFSET;
    double   FOG_YAW_OFFSET;

    double   FOG_ROLL_NULL;
    double   FOG_PITCH_NULL;
    double   FOG_YAW_NULL;

    // Hyobin added
//    double   delX;
//    double   delY;
//    double   delZ;
//    double   tempX;
//    double   tempY;
//    double   tempZ;

//    double   Quat_IMU0;
//    double   Quat_IMU1;
//    double   Quat_IMU2;
//    double   Quat_IMU3;



    void    RBIMU_AddCANMailBox();
    void    RBBoard_GetDBData(DB_IMU db);

    int     CANCheck();
    int     CANChannel_Arrange();
    int     RBIMU_RequestONOFF(int ONOFF);
    int     RBIMU_Reset(void);
    int     RBIMU_Nulling(void);


};


#endif // RBIMUSENSOR_H
