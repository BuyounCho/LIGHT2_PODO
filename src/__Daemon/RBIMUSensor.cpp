#include "RBIMUSensor.h"
#include <iostream>
using namespace std;

extern int     _NO_OF_COMM_CH;

RBIMUSensor::RBIMUSensor()
{
    ROLL_OFFSET = 0.0;
    PITCH_OFFSET = 0.0;
    YAW_OFFSET = 0.0;
}

void RBIMUSensor::RBIMU_AddCANMailBox(){
    canHandler->RBCAN_AddMailBox(ID_RCV_GENERAL);
    canHandler->RBCAN_AddMailBox(ID_RCV_DATA_QUAT);
    canHandler->RBCAN_AddMailBox(ID_RCV_DATA_LOCAL_X);
    canHandler->RBCAN_AddMailBox(ID_RCV_DATA_LOCAL_Y);
    canHandler->RBCAN_AddMailBox(ID_RCV_DATA_LOCAL_Z);
}

void RBIMUSensor::RBBoard_GetDBData(DB_IMU db){
    BOARD_ID            = db.BOARD_ID;
    BOARD_NAME          = db.BOARD_NAME;
    CAN_CHANNEL         = db.CAN_CHANNEL;
    ID_SEND_DATA        = db.ID_SEND_DATA;
    ID_RCV_GENERAL      = db.ID_RCV_GENERAL;
    ID_RCV_DATA_QUAT    = db.ID_RCV_DATA_QUAT;
    ID_RCV_DATA_LOCAL_X = db.ID_RCV_DATA_LOCAL_X;
    ID_RCV_DATA_LOCAL_Y = db.ID_RCV_DATA_LOCAL_Y;
    ID_RCV_DATA_LOCAL_Z = db.ID_RCV_DATA_LOCAL_Z;
}

int RBIMUSensor::CANCheck(){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 0;
    mb.dlc = 1;
    mb.id = ID_SEND_DATA;

    if( canHandler->RBCAN_WriteData(mb) == true ){
        usleep(10*1000);
        mb.channel = CAN_CHANNEL;
        mb.id = ID_RCV_GENERAL;
        canHandler->RBCAN_ReadData(&mb);

        if(mb.status != RBCAN_NODATA){
            std::cout << ">>> RMIMU: Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is \033[32minitialized.\033[0m[ch " << CAN_CHANNEL << "]\n";
            ConnectionStatus = true;
            mb.status = RBCAN_NODATA;
            return true;
        }else{
            std::cout << ">>> RMIMU: Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is \033[31mfailed \033[0mto initialize.[ch " << CAN_CHANNEL << "]\n";
            ConnectionStatus = false;
            return false;
        }
    }
    else return false;

}

int  RBIMUSensor::CANChannel_Arrange(){
    RBCAN_MB mb;
    std::cout << " ======== Board(" << BOARD_ID << ": IMU) is checking channels... ========\n";

    int NUM_CHECK_CAN_CH = min(MAX_HCB_CAN_CHANNEL,_NO_OF_COMM_CH);
    for(int idx = 0; idx<NUM_CHECK_CAN_CH; idx++){
        mb.data[0] = 0; //Ask Board status(information)
        mb.dlc = 1;
        mb.id = ID_SEND_DATA;
        mb.channel = idx;
        if(canHandler->RBCAN_WriteData(mb)){
            usleep(100*1000);
            mb.channel = idx;
            mb.id = ID_RCV_GENERAL;
            canHandler->RBCAN_ReadData(&mb);

            if(mb.status != RBCAN_NODATA && mb.data[0]==0){
                std::cout << ">>> Board(" << BOARD_ID << ": IMU) is \033[32mconnected \033[0mto channel [" << idx << "]\n";
                std::cout << " ========================================================\n\n";

                CAN_CHANNEL = idx;
                ConnectionStatus = true;
                mb.status = RBCAN_NODATA;
                return true;
            }else{
                std::cout << "\033[31m >>> [ Channel " << idx << " ] is not connected... \033[0m " << std::endl;
            }
        } else {
            std::cout << "\033[31m >>> [ Channel " << idx << " ] is not connected... \033[0m " << std::endl;
        }
    }

    std::cout << ">>> Board(" << BOARD_ID << ": IMU) is \033[31mnot connected.\033[0m \n";
    std::cout << " ========================================================\n\n";
    ConnectionStatus = false;
    mb.status = RBCAN_NODATA;
    return true;
}

int RBIMUSensor::RBIMU_RequestONOFF(int ONOFF) {
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 1; 			// board no.
    mb.data[1] = ONOFF;			// command
    mb.dlc = 2;
    mb.id = ID_SEND_DATA;

    if(ONOFF) {
        FILE_LOG(logINFO) << "CMD: DAEMON_SENSOR_IMU_ENABLE";
    } else {
        FILE_LOG(logINFO) << "CMD: DAEMON_SENSOR_IMU_DISABLE";
    }

    return canHandler->RBCAN_WriteData(mb);
}

int RBIMUSensor::RBIMU_Reset(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 2;
    mb.data[1] = 0;
    mb.dlc = 2;
    mb.id = ID_SEND_DATA;

    return canHandler->RBCAN_WriteData(mb);
}

int RBIMUSensor::RBIMU_Nulling(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 3;
    mb.dlc = 1;
    mb.id = ID_SEND_DATA;

    return canHandler->RBCAN_WriteData(mb);
}
