#include "HydraulicActuatorController.h"
#include <iostream>
using namespace std;

#include <chrono>
using namespace std::chrono;

extern int     _NO_OF_COMM_CH;

HydraulicActuatorController::HydraulicActuatorController()
{
    for(int i=0; i<MAX_JOINT; i++){
        MoveJoints[i].MoveFlag = false;
    }
}

void HydraulicActuatorController::HCB_GetDBData(DB_MC db){
    BOARD_NAME = db.BOARD_NAME;
    BOARD_ID = db.BOARD_ID;
    CAN_CHANNEL = db.CAN_CHANNEL;

    ACTUATOR_TYPE = db.ACTUATOR_TYPE;
    PULSE_PER_POSITION = db.PULSE_PER_POSITION;
    PULSE_PER_FORCETORQUE = db.PULSE_PER_FORCETORQUE;
    PULSE_PER_PRESSURE = db.PULSE_PER_PRESSURE;

    ID_SEND_GENERAL = db.ID_SEND_GENERAL;
    ID_SEND_POSVEL = db.ID_SEND_POSVEL;
    ID_SEND_VALVEPOS = db.ID_SEND_VALVEPOS;

    ID_RCV_GENERAL = db.ID_RCV_GENERAL;
    ID_RCV_POSVEL = db.ID_RCV_POSVEL;
    ID_RCV_VALVEPOS = db.ID_RCV_VALVEPOS;
    ID_RCV_OTHERINFO = db.ID_RCV_OTHERINFO;
}

void HydraulicActuatorController::HCB_AddCANMailBox_RCVDATA(){
    canHandler->RBCAN_AddMailBox(ID_RCV_GENERAL);
    canHandler->RBCAN_AddMailBox(ID_RCV_POSVEL);
    canHandler->RBCAN_AddMailBox(ID_RCV_VALVEPOS);
    canHandler->RBCAN_AddMailBox(ID_RCV_OTHERINFO);
//    std::cout<<"MailBoxID : "<<ID_RCV_GENERAL <<" "<<ID_RCV_POSVEL <<" "<<ID_RCV_FORCE <<" "<<ID_RCV_PRES <<" "<<ID_RCV_PWM <<" "<<ID_RCV_VALVEPOS <<" "<<std::endl;
}

// ============================================================================
// Board Operation Setting START

int  HydraulicActuatorController::HCB_CANCheck(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 0; //Ask Board status(information)
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;

//    system_clock::time_point StartTime = system_clock::now();

    if(canHandler->RBCAN_WriteData(mb)){
        usleep(3*1000);
        mb.channel = CAN_CHANNEL;
        mb.id = ID_RCV_GENERAL;
        canHandler->RBCAN_ReadData(&mb);

        if(mb.status != RBCAN_NODATA && mb.data[0]==0){
//            system_clock::time_point EndTime = system_clock::now();
//            microseconds t_calc_usec = duration_cast<std::chrono::microseconds>(EndTime - StartTime);
//            FILE_LOG(logDEBUG) << "CAN CHECK TIME : "<< t_calc_usec.count() <<" usecond(s).";

            std::cout << ">>> MC: Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is \033[32mconnected.\033[0m[ch " << CAN_CHANNEL << "]\n";
            ConnectionStatus = true;
            mb.status = RBCAN_NODATA;
            return true;
        }else{
            std::cout << ">>> MC: Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is \033[31mfailed \033[0mto connect.[ch " << CAN_CHANNEL << "]\n";
            ConnectionStatus = false;
            return false;
        }
    }
    else return false;
}

int  HydraulicActuatorController::HCB_CANChannel_Arrange(void){
    RBCAN_MB mb;
    std::cout << " ======== Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is checking channels... ========\n";

    int NUM_CHECK_CAN_CH = min(MAX_HCB_CAN_CHANNEL,_NO_OF_COMM_CH);
    for(int idx = 0; idx<NUM_CHECK_CAN_CH; idx++){
        mb.data[0] = 0; //Ask Board status(information)
        mb.dlc = 1;
        mb.id = ID_SEND_GENERAL;
        mb.channel = idx;
        if(canHandler->RBCAN_WriteData(mb)){
            usleep(100*1000);
            mb.channel = idx;
            mb.id = ID_RCV_GENERAL;
            canHandler->RBCAN_ReadData(&mb);

            if(mb.status != RBCAN_NODATA && mb.data[0]==0){
                std::cout << ">>> Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is \033[32mconnected \033[0mto channel [" << idx << "]\n";
                std::cout << " ========================================================\n\n";

                CAN_CHANNEL = idx;
                ConnectionStatus = true;
                mb.status = RBCAN_NODATA;
                return true;
            }else{
                std::cout << "\033[31m >>> [ Channel " << idx << " ] \is not connected... \033[0m " << std::endl;
            }
        } else {
            std::cout << "\033[31m >>> [ Channel " << idx << " ] is not connected... \033[0m " << std::endl;
        }
    }

    std::cout << ">>> Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is \033[31mnot connected.\033[0m \n";
    std::cout << " ========================================================\n\n";
    ConnectionStatus = false;
    mb.status = RBCAN_NODATA;
    return true;
}

int HydraulicActuatorController::HCB_InformationCheck(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 0; //Ask Board status(information)
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;

    if(canHandler->RBCAN_WriteData(mb)){
        usleep(35*1000);
        mb.channel = CAN_CHANNEL;
        mb.id = ID_RCV_GENERAL;
        canHandler->RBCAN_ReadData(&mb);
        if(mb.status != RBCAN_NODATA && mb.data[0]==0 && mb.data[1]==BOARD_ID) {
            Joints[0].HCB_Info.CAN_FREQ           = (int)((mb.data[2])|(mb.data[3]<<8));
            Joints[0].HCB_Info.FET_ONOFF          = (mb.data[4]&0b10000000);
            Joints[0].HCB_Info.BIGERROR_ONOFF     = (mb.data[4]&0b01000000);
            Joints[0].HCB_Info.ENCERROR_ONOFF     = (mb.data[4]&0b00100000);
            Joints[0].HCB_Info.CANERROR_ONOFF     = (mb.data[4]&0b00010000);
            Joints[0].HCB_Info.HOMEERROR_ONOFF    = (mb.data[4]&0b00001000);
            Joints[0].HCB_Info.PLIMITERROR_ONOFF  = (mb.data[4]&0b00000100);
            Joints[0].HCB_Info.LOGICERROR_ONOFF   = (mb.data[4]&0b00000010);
            Joints[0].HCB_Info.INPUTERROR_ONOFF   = (mb.data[4]&0b00000001);
            switch(mb.data[5]){
            case 0:
                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_NOACT; break;
            case 1:
                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_VALVE_OPENLOOP; break;
            case 2:
                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_VALVE_POS; break;
            case 3:
                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_POS_FORCE_PWM; break;
            case 4:
                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_POS_FORCE_VALVE; break;
            case 5:
                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_POS_FORCE_LEARN; break;
            case 6:
                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_POS_PRES_PWM; break;
            case 7:
                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_POS_PRES_VALVE; break;
            case 8:
                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_POS_PRES_LEARN; break;
            case 9:
                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_PWM_SINE_OPENLOOL; break;
            case 20:
                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_FORCE_NULL; break;
            case 21:
                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_VALVE_NULL; break;
            case 22:
                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_FINDHOME; break;
            case 23:
                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_FLOWGAIN_TUNE; break;
            default:
                break;
            }
            if(mb.data[6]==0){
                Joints[0].HCB_Info.OPERATION_MODE = HCB_OPER_MODE_SW;
            } else {
                Joints[0].HCB_Info.OPERATION_MODE = HCB_OPER_MODE_SH;
            }
            switch(mb.data[7]){
            }

            std::cout << "=================================== " << std::endl;
            std::cout << "CAN FREQ : "  << Joints[0].HCB_Info.CAN_FREQ << std::endl;
            std::cout << "CAN FET ERROR : " << Joints[0].HCB_Info.FET_ONOFF << std::endl;
            std::cout << "CAN BIG ERROR : " << Joints[0].HCB_Info.BIGERROR_ONOFF << std::endl;
            std::cout << "CAN ENC ERROR : " << Joints[0].HCB_Info.ENCERROR_ONOFF << std::endl;
            std::cout << "CAN ENC ERROR : " << Joints[0].HCB_Info.CANERROR_ONOFF << std::endl;
            std::cout << "CAN ENC ERROR : " << Joints[0].HCB_Info.HOMEERROR_ONOFF << std::endl;
            std::cout << "CAN ENC ERROR : " << Joints[0].HCB_Info.PLIMITERROR_ONOFF << std::endl;
            std::cout << "CAN ENC ERROR : " << Joints[0].HCB_Info.LOGICERROR_ONOFF << std::endl;
            std::cout << "CAN ENC ERROR : " << Joints[0].HCB_Info.INPUTERROR_ONOFF << std::endl;
            std::cout << "CAN Control Mode : " << Joints[0].HCB_Info.CONTROL_MODE << std::endl;
            std::cout << "CAN Operation Mode : " << Joints[0].HCB_Info.OPERATION_MODE << std::endl;
            std::cout << "=================================== " << std::endl;

            ConnectionStatus = true;
            mb.status = RBCAN_NODATA;
            return true;
        } else {
            ConnectionStatus = false;
            return false;
        }
    }
    else return false;

}

void HydraulicActuatorController::HCB_ASK_BoardNumber(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 1;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;

    if(canHandler->RBCAN_WriteData(mb)){
        usleep(35*1000);
        mb.channel = CAN_CHANNEL;
        mb.id = ID_RCV_GENERAL;
        canHandler->RBCAN_ReadData(&mb);
        if(mb.status != RBCAN_NODATA && mb.data[0]==1 && mb.data[1]==BOARD_ID){
            std::cout << "=================================== " << std::endl;
            std::cout << "Board ID : " << (int)(mb.data[1]) << std::endl;
            std::cout << "=================================== " << std::endl;
            ConnectionStatus = true;
            mb.status = RBCAN_NODATA;
        }else{
            std::cout << ">>> ASK 1 - Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is not connected. \n";
            ConnectionStatus = false;
        }
    }
}
int HydraulicActuatorController::HCB_CMD_BoardNumber(int BNO){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 101;
    mb.data[1] = BNO;
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}

int HydraulicActuatorController::HCB_CMD_EncZero(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 103;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}

int HydraulicActuatorController::HCB_CMD_HomePos(int settingswitch_all){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 114;
    mb.data[1] = settingswitch_all;
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}

bool HydraulicActuatorController::HCB_Read_FindHomeDone(void)
{
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.id = ID_RCV_GENERAL;
    canHandler->RBCAN_ReadData(&mb);
    if(mb.status != RBCAN_NODATA && mb.data[0]==3){
        std::cout << "Board (" << this->BOARD_ID << ") : Find Home is Done!! " << std::endl;
        mb.status = RBCAN_NODATA;
        return true;
    }
    return false;
}


void HydraulicActuatorController::HCB_ASK_BoardOperationMode(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 2;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;

    if(canHandler->RBCAN_WriteData(mb)){
        usleep(35*1000);
        mb.channel = CAN_CHANNEL;
        mb.id = ID_RCV_GENERAL;
        canHandler->RBCAN_ReadData(&mb);
        if(mb.status != RBCAN_NODATA && mb.data[0]==2){
            Joints[0].HCB_Info.OPERATION_MODE = mb.data[1];
            Joints[0].HCB_Info.SENSING_MODE = mb.data[2];
            Joints[0].HCB_Info.CURRENT_CONTROL_MODE = mb.data[3];
            std::cout << "=================================== " << std::endl;
            std::cout << "Board Oper. Mode : " << (int)(Joints[0].HCB_Info.OPERATION_MODE) << std::endl;
            std::cout << "Board Sensing Mode : " << (int)(Joints[0].HCB_Info.SENSING_MODE) << std::endl;
            std::cout << "Board Current Ctrl. Mode : " << (int)(Joints[0].HCB_Info.CURRENT_CONTROL_MODE) << std::endl;
            std::cout << "=================================== " << std::endl;
        }else{
            std::cout << ">>> ASK 2 - Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is not connected. \n";
        }
    }
}
int HydraulicActuatorController::HCB_CMD_BoardOperationMode(int OperMode,int SenseMode, int CurCtrlMode){
    Joints[0].HCB_Info.OPERATION_MODE = OperMode;

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 102;
    mb.data[1] = OperMode; // (00 : Moog & Rot, 01 : Moog & Lin, 10 : KNR & Rot, 11 : KNR & Lin)
    mb.data[2] = SenseMode; // (0 : torque, 1: pressure)
    mb.data[3] = CurCtrlMode; // (0 : pwm, 1 : current control)
    mb.data[4] = 0; // (0 : not use mechanical deadzone, 1 : use)
    mb.dlc = 5;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}

int HydraulicActuatorController::HCB_CMD_EncoderZero(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 103;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}

int HydraulicActuatorController::HCB_CMD_FETOnOff(bool OnOff){
    Joints[0].HCB_Info.FET_ONOFF = OnOff;

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 104;
    if(OnOff) {
        mb.data[1] = 1; // FET On!
    } else {
        mb.data[1] = 0; // FET Off!
    }
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}

enum JointControlMethod {
    ForceControl = 1,
    PositionControl = 3,
};

int HydraulicActuatorController::HCB_CMD_ControlMethodTransition(bool On_TorqueControl){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 105;
    if(On_TorqueControl==true) { // Position Control
        mb.data[1] = ForceControl; // FET On!
    } else {
        mb.data[1] = PositionControl; // FET Off!
    }
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}

void HydraulicActuatorController::HCB_ASK_CANFrequency(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 6;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;

    if(canHandler->RBCAN_WriteData(mb)){
        usleep(35*1000);
        mb.channel = CAN_CHANNEL;
        mb.id = ID_RCV_GENERAL;
        canHandler->RBCAN_ReadData(&mb);
        if(mb.status != RBCAN_NODATA && mb.data[0]==6){
            Joints[0].HCB_Info.CAN_FREQ = (int16_t)((mb.data[1])|(mb.data[2]<<8));
            std::cout << "=================================== " << std::endl;
            std::cout << "Board CAN Frequency : " << (int)(Joints[0].HCB_Info.CAN_FREQ) << std::endl;
            std::cout << "=================================== " << std::endl;
            mb.status = RBCAN_NODATA;
        }else{
            std::cout << ">>> ASK 6 - Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is not connected. \n";
        }
    }
}
int HydraulicActuatorController::HCB_CMD_CANFrequency(int16_t CAN_FREQ_HZ){

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 106;
    mb.data[1] = (uint8_t)(CAN_FREQ_HZ & 0xFF);
    mb.data[2] = (uint8_t)((CAN_FREQ_HZ>>8) & 0xFF);
    mb.dlc = 3;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}

void HydraulicActuatorController::HCB_ASK_ControlMode(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 7;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;

    if(canHandler->RBCAN_WriteData(mb)){
        usleep(35*1000);
        mb.channel = CAN_CHANNEL;
        mb.id = ID_RCV_GENERAL;
        canHandler->RBCAN_ReadData(&mb);
        if(mb.status != RBCAN_NODATA && mb.data[0]==7){
            Joints[0].HCB_Info.CONTROL_MODE = (int)mb.data[1];
            std::cout << "=================================== " << std::endl;
            std::cout << "Board Control Mode : " << (int)(Joints[0].HCB_Info.CONTROL_MODE) << std::endl;
            std::cout << "=================================== " << std::endl;
            mb.status = RBCAN_NODATA;
        }else{
            std::cout << ">>> ASK 7 - Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is not connected. \n";
        }
    }
}

int HydraulicActuatorController::HCB_CMD_ControlMode(int mode){
    Joints[0].HCB_Info.CONTROL_MODE = mode;

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 107;
    mb.data[1] = mode;
    // data[1] = 0   : NO control mode
    // data[1] = 1   : valve open loop control mode
    // data[1] = 2   : valve position control mode
    // data[1] = 3   : joint position and force control based on PWM
    // data[1] = 4   : joint position and force control based on valve position
    // data[1] = 5   : joint position and force control based on learing
    // data[1] = 6   : joint position and load pressure control based on PWM
    // data[1] = 7   : joint position and load pressure control based on valve position
    // data[1] = 8   : joint position and load pressure control based on learning
    // data[1] = 9   : valve sine openloop
    // data[1] = 20  : (force sensor or pressure sensor) nulling
    // data[1] = 21  : valve nulling and dead zone setting
    // data[1] = 22  : find home
    // data[1] = 23  : flow rate gain tuning
    // data[1] = 24  : pressure sensor nulling
    // data[1] = 25  : pressure sensor calibration
    // data[1] = 26  : Friction tuning
    // data[1] = 27  : Valve identification with current control

    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}
int HydraulicActuatorController::HCB_CMD_Request_PosVel(bool OnOff){
    Joints[0].HCB_Info.REQUEST_ONOFF_PosVel = OnOff;

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 108;
    if(OnOff) {
        mb.data[1] = 1; // PosVel Request On!
    } else {
        mb.data[1] = 0; // PosVel Request Off!
    }
    mb.data[2] = 0;
    mb.dlc = 3;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}
int HydraulicActuatorController::HCB_CMD_Request_ValvePos(bool OnOff){
    Joints[0].HCB_Info.REQUEST_ONOFF_ValvePos = OnOff;

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 108;
    if(OnOff) {
        mb.data[1] = 1; // ValvePos Request On!
    } else {
        mb.data[1] = 0; // ValvePos Request Off!
    }
    mb.data[2] = 1;
    mb.dlc = 3;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}
int HydraulicActuatorController::HCB_CMD_Request_OtherInfo(bool OnOff){
    Joints[0].HCB_Info.REQUEST_ONOFF_OtherInfo = OnOff;

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 108;
    if(OnOff) {
        mb.data[1] = 1; // ValvePos Request On!
    } else {
        mb.data[1] = 0; // ValvePos Request Off!
    }
    mb.data[2] = 2;
    mb.dlc = 3;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}
void HydraulicActuatorController::HCB_ASK_JointEncDir(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 9;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;

    if(canHandler->RBCAN_WriteData(mb)){
        usleep(35*1000);
        mb.channel = CAN_CHANNEL;
        mb.id = ID_RCV_GENERAL;
        canHandler->RBCAN_ReadData(&mb);
        if(mb.status != RBCAN_NODATA && mb.data[0]==9){
            Joints[0].HCB_Info.JOINTENC_DIRECTION = (int16_t)((mb.data[1])|(mb.data[2]<<8)); // 1 : Positive / -1 : Negative
            std::cout << "=================================== " << std::endl;
            std::cout << "Board Joint Enc Dir. : " << (Joints[0].HCB_Info.JOINTENC_DIRECTION) << std::endl;
            std::cout << "=================================== " << std::endl;
            mb.status = RBCAN_NODATA;
        }else{
            std::cout << ">>> ASK 9 - Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is not connected. \n";
        }
    }
}
int HydraulicActuatorController::HCB_CMD_JointEncDir(int Dir){
    Joints[0].HCB_Info.JOINTENC_DIRECTION = Dir;

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 109;
    mb.data[1] = (unsigned char)(Dir & 0x000000FF);
    mb.data[2] = (unsigned char)((Dir>>8) & 0x000000FF);
    mb.dlc = 3;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}

void HydraulicActuatorController::HCB_ASK_ValveInputDir(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 10;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;

    if(canHandler->RBCAN_WriteData(mb)){
        usleep(35*1000);
        mb.channel = CAN_CHANNEL;
        mb.id = ID_RCV_GENERAL;
        canHandler->RBCAN_ReadData(&mb);
        if(mb.status != RBCAN_NODATA && mb.data[0]==10){
            Joints[0].HCB_Info.VALVEINPUT_DIRECTION = (int16_t)((mb.data[1])|(mb.data[2]<<8)); // 1 : Positive / 0 : Negative
            std::cout << "=================================== " << std::endl;
            std::cout << "Board Valve Input Dir. : " << (Joints[0].HCB_Info.VALVEINPUT_DIRECTION) << std::endl;
            std::cout << "=================================== " << std::endl;
            mb.status = RBCAN_NODATA;
        }else{
            std::cout << ">>> ASK 10 - Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is not connected. \n";
        }
    }
}
int HydraulicActuatorController::HCB_CMD_ValveInputDir(int Dir){
    Joints[0].HCB_Info.VALVEINPUT_DIRECTION = (int)Dir;

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 110;
    mb.data[1] = (Dir & 0x000000FF);
    mb.data[2] = ((Dir>>8) & 0x000000FF);
    mb.dlc = 3;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}

void HydraulicActuatorController::HCB_ASK_ValveEncDir(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 11;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;

    if(canHandler->RBCAN_WriteData(mb)){
        usleep(35*1000);
        mb.channel = CAN_CHANNEL;
        mb.id = ID_RCV_GENERAL;
        canHandler->RBCAN_ReadData(&mb);
        if(mb.status != RBCAN_NODATA && mb.data[0]==11){
            Joints[0].HCB_Info.VALVEENC_DIRECTION = (int16_t)((mb.data[1])|(mb.data[2]<<8)); // 1 : Positive / 0 : Negative
            std::cout << "=================================== " << std::endl;
            std::cout << "Board Valve Enc Dir. : " << (Joints[0].HCB_Info.VALVEENC_DIRECTION) << std::endl;
            std::cout << "=================================== " << std::endl;
            mb.status = RBCAN_NODATA;
        }else{
            std::cout << ">>> ASK 11 - Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is not connected. \n";
        }
    }
}
int HydraulicActuatorController::HCB_CMD_ValveEncDir(int Dir){
    Joints[0].HCB_Info.VALVEENC_DIRECTION = (int)Dir;

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 111;
    mb.data[1] = (Dir & 0x000000FF);
    mb.data[2] = ((Dir>>8) & 0x000000FF);
    mb.dlc = 3;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}

void HydraulicActuatorController::HCB_ASK_BoardInputVoltage(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 12;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;

    if(canHandler->RBCAN_WriteData(mb)){
        usleep(35*1000);
        mb.channel = CAN_CHANNEL;
        mb.id = ID_RCV_GENERAL;
        canHandler->RBCAN_ReadData(&mb);
        if(mb.status != RBCAN_NODATA && mb.data[0]==12){
            int temp_V = (int)((mb.data[1])|(mb.data[2]<<8));
            Joints[0].HCB_Info.BOARD_IN_VOLTAGE = ((double)temp_V)/10.0;
            std::cout << "=================================== " << std::endl;
            std::cout << "Board Supply Voltage : " << (Joints[0].HCB_Info.BOARD_IN_VOLTAGE) << std::endl;
            std::cout << "=================================== " << std::endl;
            mb.status = RBCAN_NODATA;
        }else{
            std::cout << ">>> ASK 12 - Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is not connected. \n";
        }
    }
}
int HydraulicActuatorController::HCB_CMD_BoardInputVoltage(double InV){
    Joints[0].HCB_Info.BOARD_IN_VOLTAGE = InV;

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 112;
    int temp = (int)(10.0*InV);
    mb.data[1] = (unsigned char)(temp & 0x000000FF);
    mb.data[2] = (unsigned char)((temp>>8) & 0x000000FF);
    mb.dlc = 3;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}

void HydraulicActuatorController::HCB_ASK_ValveOperationVoltage(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 13;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;

    if(canHandler->RBCAN_WriteData(mb)){
        usleep(35*1000);
        mb.channel = CAN_CHANNEL;
        mb.id = ID_RCV_GENERAL;
        canHandler->RBCAN_ReadData(&mb);
        if(mb.status != RBCAN_NODATA && mb.data[0]==13){
            int temp_V = (int)((mb.data[1])|(mb.data[2]<<8));
            Joints[0].HCB_Info.BOARD_OPER_VOLTAGE = ((double)temp_V)/10.0;
            std::cout << "=================================== " << std::endl;
            std::cout << "Board Valve Operation Voltage : " << (Joints[0].HCB_Info.BOARD_OPER_VOLTAGE) << std::endl;
            std::cout << "=================================== " << std::endl;
            mb.status = RBCAN_NODATA;
        }else{
            std::cout << ">>> ASK 13 - Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is not connected. \n";
        }
    }
}
int HydraulicActuatorController::HCB_CMD_ValveOperationVoltage(double OperV){
    Joints[0].HCB_Info.BOARD_OPER_VOLTAGE = OperV;

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 113;
    int temp = (int)(10.0*OperV);
    mb.data[1] = (unsigned char)(temp & 0x000000FF);
    mb.data[2] = (unsigned char)((temp>>8) & 0x000000FF);
    mb.dlc = 3;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}

void HydraulicActuatorController::HCB_ASK_VariableSupplyPressure(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 15;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;

    if(canHandler->RBCAN_WriteData(mb)){
        usleep(35*1000);
        mb.channel = CAN_CHANNEL;
        mb.id = ID_RCV_GENERAL;
        canHandler->RBCAN_ReadData(&mb);
        if(mb.status != RBCAN_NODATA && mb.data[0]==15){
            int temp = mb.data[1];
            Joints[0].HCB_Info.VARIABLE_SUPPLYPRES_ONOFF = temp;
            std::cout << "=================================== " << std::endl;
            std::cout << "Variable Supply Pressure Mode : " << (Joints[0].HCB_Info.VARIABLE_SUPPLYPRES_ONOFF) << std::endl;
            std::cout << "=================================== " << std::endl;
            mb.status = RBCAN_NODATA;
        }else{
            std::cout << ">>> ASK 15 - Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is not connected. \n";
        }
    }
}
int HydraulicActuatorController::HCB_CMD_VariableSupplyPressure(int OnOff){
    Joints[0].HCB_Info.VARIABLE_SUPPLYPRES_ONOFF = OnOff;

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 115;
    mb.data[1] = (unsigned char)(OnOff & 0x000000FF);
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}


// Board Operation Setting END
// ============================================================================
// Control Parameter Setting START

void HydraulicActuatorController::HCB_ASK_PIDGain(int mode){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 20;
    mb.data[1] = mode;
    // data[1] = 0 : Valve Position Gain
    // data[1] = 1 : Joint Position Gain
    // data[1] = 2 : Joint Force/torque Gain
    // data[1] = 3 : Joint Spring&Damper
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;

    if(canHandler->RBCAN_WriteData(mb)){
        usleep(35*1000);
        mb.channel = CAN_CHANNEL;
        mb.id = ID_RCV_GENERAL;
        canHandler->RBCAN_ReadData(&mb);
        if(mb.status != RBCAN_NODATA && mb.data[0]==20){
            if (mb.data[1]==0) {     // data[1] = 0 : Valve Position Gain
                Joints[0].HCB_Info.VALVE_P_KP = (int16_t)((mb.data[2])|(mb.data[3]<<8));
                Joints[0].HCB_Info.VALVE_P_KI = (int16_t)((mb.data[4])|(mb.data[5]<<8));
                Joints[0].HCB_Info.VALVE_P_KD = (int16_t)((mb.data[6])|(mb.data[7]<<8));
                std::cout << "=================================== " << std::endl;
                std::cout << "Valve Pos. P Gain : " << (Joints[0].HCB_Info.VALVE_P_KP) << std::endl;
                std::cout << "Valve Pos. I Gain : " << (Joints[0].HCB_Info.VALVE_P_KI) << std::endl;
                std::cout << "Valve Pos. D Gain : " << (Joints[0].HCB_Info.VALVE_P_KD) << std::endl;
                std::cout << "=================================== " << std::endl;
            } else if (mb.data[1]==1) {     // data[1] = 1 : Joint Position Gain
                Joints[0].HCB_Info.JOINT_P_KP = (int16_t)((mb.data[2])|(mb.data[3]<<8));
                Joints[0].HCB_Info.JOINT_P_KI = (int16_t)((mb.data[4])|(mb.data[5]<<8));
                Joints[0].HCB_Info.JOINT_P_KD = (int16_t)((mb.data[6])|(mb.data[7]<<8));
                std::cout << "=================================== " << std::endl;
                std::cout << "Joint Pos. P Gain : " << (Joints[0].HCB_Info.JOINT_P_KP) << std::endl;
                std::cout << "Joint Pos. I Gain : " << (Joints[0].HCB_Info.JOINT_P_KI) << std::endl;
                std::cout << "Joint Pos. D Gain : " << (Joints[0].HCB_Info.JOINT_P_KD) << std::endl;
                std::cout << "=================================== " << std::endl;

            } else if (mb.data[1]==2) {     // data[1] = 2 : Joint Force/torque Gain
                Joints[0].HCB_Info.JOINT_F_KP = (int16_t)((mb.data[2])|(mb.data[3]<<8));
                Joints[0].HCB_Info.JOINT_F_KI = (int16_t)((mb.data[4])|(mb.data[5]<<8));
                Joints[0].HCB_Info.JOINT_F_KD = (int16_t)((mb.data[6])|(mb.data[7]<<8));
                std::cout << "=================================== " << std::endl;
                std::cout << "Joint Force P Gain : " << (Joints[0].HCB_Info.JOINT_F_KP) << std::endl;
                std::cout << "Joint Force I Gain : " << (Joints[0].HCB_Info.JOINT_F_KI) << std::endl;
                std::cout << "Joint Force D Gain : " << (Joints[0].HCB_Info.JOINT_F_KD) << std::endl;
                std::cout << "=================================== " << std::endl;
            } else if (mb.data[1]==3) {     // data[1] = 3 : Joint Spring&Damper
                int16_t temp_SPRING = (int16_t)((mb.data[2])|(mb.data[3]<<8));
                int16_t temp_DAMPER = (int16_t)((mb.data[4])|(mb.data[5]<<8));
                Joints[0].HCB_Info.JOINT_SPRING = ((double)temp_SPRING)/10.0;
                Joints[0].HCB_Info.JOINT_DAMPER = ((double)temp_DAMPER)/100.0;
                std::cout << "=================================== " << std::endl;
                std::cout << "Joint Spring Constant : " << (Joints[0].HCB_Info.JOINT_SPRING) << std::endl;
                std::cout << "Joint Damper Constant : " << (Joints[0].HCB_Info.JOINT_DAMPER) << std::endl;
                std::cout << "=================================== " << std::endl;
            }
            mb.status = RBCAN_NODATA;
        }else{
            std::cout << ">>> ASK 20 - Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is not connected. \n";
        }
    }
}
int HydraulicActuatorController::HCB_CMD_PIDGain(int mode, int Pgain, int Igain, int Dgain){
    if (mode==0) {     // data[1] = 0 : Valve Position Gain
        Joints[0].HCB_Info.VALVE_P_KP = Pgain;
        Joints[0].HCB_Info.VALVE_P_KI = Igain;
        Joints[0].HCB_Info.VALVE_P_KD = Dgain;
    } else if (mode==1) {     // data[1] = 1 : Joint Position Gain
        Joints[0].HCB_Info.JOINT_P_KP = Pgain;
        Joints[0].HCB_Info.JOINT_P_KI = Igain;
        Joints[0].HCB_Info.JOINT_P_KD = Dgain;
    } else if (mode==2) {     // data[1] = 2 : Joint Force/torque Gain
        Joints[0].HCB_Info.JOINT_F_KP = Pgain;
        Joints[0].HCB_Info.JOINT_F_KI = Igain;
        Joints[0].HCB_Info.JOINT_F_KD = Dgain;
    } else if (mode==3) {     // data[1] = 3 : Joint Spring&Damper
        Joints[0].HCB_Info.JOINT_SPRING = Pgain;
        Joints[0].HCB_Info.JOINT_DAMPER = Igain;
    }

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 120;
    mb.data[1] = mode;
    mb.data[2] = (unsigned char)(Pgain & 0x000000FF);
    mb.data[3] = (unsigned char)((Pgain>>8) & 0x000000FF);
    mb.data[4] = (unsigned char)(Igain & 0x000000FF);
    mb.data[5] = (unsigned char)((Igain>>8) & 0x000000FF);
    mb.data[6] = (unsigned char)(Dgain & 0x000000FF);
    mb.data[7] = (unsigned char)((Dgain>>8) & 0x000000FF);
    mb.dlc = 8;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}

int HydraulicActuatorController::HCB_CMD_JointSpringDamper(double Spring, double Damper)
{
    Joints[0].HCB_Info.JOINT_SPRING = Spring;
    Joints[0].HCB_Info.JOINT_DAMPER = Damper;

    int16_t K = (int16_t)(Spring*10.0);
    int16_t D = (int16_t)(Damper*100.0);

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 120;
    mb.data[1] = 3;
    mb.data[2] = (unsigned char)(K & 0x000000FF);
    mb.data[3] = (unsigned char)((K>>8) & 0x000000FF);
    mb.data[4] = (unsigned char)(D & 0x000000FF);
    mb.data[5] = (unsigned char)((D>>8) & 0x000000FF);
    mb.data[6] = 0;
    mb.data[7] = 0;
    mb.dlc = 8;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}

void HydraulicActuatorController::HCB_ASK_ValveDeadzoneNCenterPos(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 21;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;

    if(canHandler->RBCAN_WriteData(mb)){
        usleep(35*1000);
        mb.channel = CAN_CHANNEL;
        mb.id = ID_RCV_GENERAL;
        canHandler->RBCAN_ReadData(&mb);
        if(mb.status != RBCAN_NODATA && mb.data[0]==21){
            Joints[0].HCB_Info.VALVE_CENTER_POS = (int16_t)((mb.data[1])|(mb.data[2]<<8));
            Joints[0].HCB_Info.VALVE_DZ_PLUS    = (int16_t)((mb.data[3])|(mb.data[4]<<8));
            Joints[0].HCB_Info.VALVE_DZ_MINUS   = (int16_t)((mb.data[5])|(mb.data[6]<<8));
            std::cout << "=================================== " << std::endl;
            std::cout << "Valve Center Pos. : " << (Joints[0].HCB_Info.VALVE_CENTER_POS) << std::endl;
            std::cout << "Valve Positive DeadZone Pos : " << (Joints[0].HCB_Info.VALVE_DZ_PLUS) << std::endl;
            std::cout << "Valve Negative DeadZone Pos : " << (Joints[0].HCB_Info.VALVE_DZ_MINUS) << std::endl;
            std::cout << "=================================== " << std::endl;
            mb.status = RBCAN_NODATA;
        }else{
            std::cout << ">>> ASK 21 - Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is not connected. \n";
        }
    }

}
int HydraulicActuatorController::HCB_CMD_ValveDeadzoneNCenterPos(int CenterPos, int PositiveDeadzone, int NegativeDeadZone){
    // Unit : PWM(two-stage valve) / Pulse(DDV)
    Joints[0].HCB_Info.VALVE_CENTER_POS = CenterPos;
    Joints[0].HCB_Info.VALVE_DZ_PLUS = PositiveDeadzone;
    Joints[0].HCB_Info.VALVE_DZ_MINUS = NegativeDeadZone;

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 121;
    mb.data[1] = (unsigned char)(CenterPos & 0x000000FF);
    mb.data[2] = (unsigned char)((CenterPos>>8) & 0x000000FF);
    mb.data[3] = (unsigned char)(PositiveDeadzone & 0x000000FF);
    mb.data[4] = (unsigned char)((PositiveDeadzone>>8) & 0x000000FF);
    mb.data[5] = (unsigned char)(NegativeDeadZone & 0x000000FF);
    mb.data[6] = (unsigned char)((NegativeDeadZone>>8) & 0x000000FF);
    mb.dlc = 7;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}

void HydraulicActuatorController::HCB_ASK_VelCompensationGain(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 22;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;

    if(canHandler->RBCAN_WriteData(mb)){
        usleep(35*1000);
        mb.channel = CAN_CHANNEL;
        mb.id = ID_RCV_GENERAL;
        canHandler->RBCAN_ReadData(&mb);
        if(mb.status != RBCAN_NODATA && mb.data[0]==22){
            Joints[0].HCB_Info.VEL_COMPENSATION_K = (int)((mb.data[1])|(mb.data[2]<<8));
            std::cout << "=================================== " << std::endl;
            std::cout << "Velocity Compensation Gain : " << (Joints[0].HCB_Info.VEL_COMPENSATION_K) << std::endl;
            std::cout << "=================================== " << std::endl;
            mb.status = RBCAN_NODATA;
        }else{
            std::cout << ">>> ASK 22 - Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is not connected. \n";
        }
    }
}
int HydraulicActuatorController::HCB_CMD_VelCompensationGain(int CompensationPercent){
    // Unit : % (0~100(?))
    Joints[0].HCB_Info.VEL_COMPENSATION_K = CompensationPercent;

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 122;
    mb.data[1] = (unsigned char)(CompensationPercent & 0x000000FF);
    mb.data[2] = (unsigned char)((CompensationPercent>>8) & 0x000000FF);
    mb.dlc = 3;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}

void HydraulicActuatorController::HCB_ASK_ComplianceGain(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 23;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;

    if(canHandler->RBCAN_WriteData(mb)){
        usleep(35*1000);
        mb.channel = CAN_CHANNEL;
        mb.id = ID_RCV_GENERAL;
        canHandler->RBCAN_ReadData(&mb);
        if(mb.status != RBCAN_NODATA && mb.data[0]==23){
            Joints[0].HCB_Info.ACTUATOR_COMPLIANCE_K = (int)((mb.data[1])|(mb.data[2]<<8));
            std::cout << "=================================== " << std::endl;
            std::cout << "Actuator Compliance Gain : " << (Joints[0].HCB_Info.ACTUATOR_COMPLIANCE_K) << std::endl;
            std::cout << "=================================== " << std::endl;
            mb.status = RBCAN_NODATA;
        }else{
            std::cout << ">>> ASK 23 - Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is not connected. \n";
        }
    }
}
int HydraulicActuatorController::HCB_CMD_ComplianceGain(int KGain){
    // Unit : ?(Neet to test)
    Joints[0].HCB_Info.ACTUATOR_COMPLIANCE_K = KGain;

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 123;
    mb.data[1] = (unsigned char)(KGain & 0x000000FF);
    mb.data[2] = (unsigned char)((KGain>>8) & 0x000000FF);
    mb.dlc = 3;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}

void HydraulicActuatorController::HCB_ASK_ValveFeedforward(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 25;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;

    if(canHandler->RBCAN_WriteData(mb)){
        usleep(35*1000);
        mb.channel = CAN_CHANNEL;
        mb.id = ID_RCV_GENERAL;
        canHandler->RBCAN_ReadData(&mb);
        if(mb.status != RBCAN_NODATA && mb.data[0]==25){
            Joints[0].HCB_Info.VALVE_FEEDFORWARD = (int)((mb.data[1])|(mb.data[2]<<8));
            std::cout << "=================================== " << std::endl;
            std::cout << "Valve Feedforward : " << (Joints[0].HCB_Info.VALVE_FEEDFORWARD) << std::endl;
            std::cout << "=================================== " << std::endl;
            mb.status = RBCAN_NODATA;
        }else{
            std::cout << ">>> ASK 25 - Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is not connected. \n";
        }
    }
}
int HydraulicActuatorController::HCB_CMD_ValveFeedforward(int ff){
    // Unit : PWM(two-stage valve) / Pulse(DDV)
    Joints[0].HCB_Info.VALVE_FEEDFORWARD = ff;

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 125;
    mb.data[1] = (unsigned char)(ff & 0x000000FF);
    mb.data[2] = (unsigned char)((ff>>8) & 0x000000FF);
    mb.dlc = 3;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}

void HydraulicActuatorController::HCB_ASK_BulkModulus(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 26;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;

    if(canHandler->RBCAN_WriteData(mb)){
        usleep(35*1000);
        mb.channel = CAN_CHANNEL;
        mb.id = ID_RCV_GENERAL;
        canHandler->RBCAN_ReadData(&mb);
        if(mb.status != RBCAN_NODATA && mb.data[0]==26){
            Joints[0].HCB_Info.BULK_MODULUS = (int)((mb.data[1])|(mb.data[2]<<8));
            std::cout << "=================================== " << std::endl;
            std::cout << "Bulk Modulus : " << (Joints[0].HCB_Info.BULK_MODULUS) << std::endl;
            std::cout << "=================================== " << std::endl;
            mb.status = RBCAN_NODATA;
        }else{
            std::cout << ">>> ASK 26 - Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is not connected. \n";
        }
    }
}
int HydraulicActuatorController::HCB_CMD_BulkModulus(int B){
    // Unit : bar
    Joints[0].HCB_Info.BULK_MODULUS = B;

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 126;
    mb.data[1] = (unsigned char)(B & 0x000000FF);
    mb.data[2] = (unsigned char)((B>>8) & 0x000000FF);
    mb.dlc = 3;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}

void HydraulicActuatorController::HCB_ASK_ChamberVolume(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 27;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;

    if(canHandler->RBCAN_WriteData(mb)){
        usleep(35*1000);
        mb.channel = CAN_CHANNEL;
        mb.id = ID_RCV_GENERAL;
        canHandler->RBCAN_ReadData(&mb);
        if(mb.status != RBCAN_NODATA && mb.data[0]==27){
            Joints[0].HCB_Info.VOL_A = (int)((mb.data[1])|(mb.data[2]<<8));
            Joints[0].HCB_Info.VOL_B = (int)((mb.data[3])|(mb.data[4]<<8));
            std::cout << "=================================== " << std::endl;
            std::cout << "Chamber A Volume : " << (Joints[0].HCB_Info.VOL_A) << std::endl;
            std::cout << "Chamber B Volume : " << (Joints[0].HCB_Info.VOL_B) << std::endl;
            std::cout << "=================================== " << std::endl;
            mb.status = RBCAN_NODATA;
        }else{
            std::cout << ">>> ASK 27 - Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is not connected. \n";
        }
    }
}
int HydraulicActuatorController::HCB_CMD_ChamberVolume(int Vol_A, int Vol_B){
    // Unit : mm^3
    Joints[0].HCB_Info.VOL_A = Vol_A;
    Joints[0].HCB_Info.VOL_B = Vol_B;

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 127;
    mb.data[1] = (unsigned char)(Vol_A & 0x000000FF);
    mb.data[2] = (unsigned char)((Vol_A>>8) & 0x000000FF);
    mb.data[3] = (unsigned char)(Vol_B & 0x000000FF);
    mb.data[4] = (unsigned char)((Vol_B>>8) & 0x000000FF);
    mb.dlc = 5;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}

void HydraulicActuatorController::HCB_ASK_PistonArea(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 28;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;

    if(canHandler->RBCAN_WriteData(mb)){
        usleep(35*1000);
        mb.channel = CAN_CHANNEL;
        mb.id = ID_RCV_GENERAL;
        canHandler->RBCAN_ReadData(&mb);
        if(mb.status != RBCAN_NODATA && mb.data[0]==28){
            Joints[0].HCB_Info.PIS_AREA_A = (int)((mb.data[1])|(mb.data[2]<<8));
            Joints[0].HCB_Info.PIS_AREA_B = (int)((mb.data[3])|(mb.data[4]<<8));
            std::cout << "=================================== " << std::endl;
            std::cout << "Piston A Area : " << (Joints[0].HCB_Info.PIS_AREA_A) << std::endl;
            std::cout << "Piston B Area : " << (Joints[0].HCB_Info.PIS_AREA_B) << std::endl;
            std::cout << "=================================== " << std::endl;
            mb.status = RBCAN_NODATA;
        }else{
            std::cout << ">>> ASK 28 - Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is not connected. \n";
        }
    }
}
int HydraulicActuatorController::HCB_CMD_PistonArea(int Area_A, int Area_B){
    // Unit : mm^2
    Joints[0].HCB_Info.PIS_AREA_A = Area_A;
    Joints[0].HCB_Info.PIS_AREA_B = Area_B;

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 128;
    mb.data[1] = (unsigned char)(Area_A & 0x000000FF);
    mb.data[2] = (unsigned char)((Area_A>>8) & 0x000000FF);
    mb.data[3] = (unsigned char)(Area_B & 0x000000FF);
    mb.data[4] = (unsigned char)((Area_B>>8) & 0x000000FF);
    mb.dlc = 5;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}

void HydraulicActuatorController::HCB_ASK_SupplyAndReturnPressure(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 29;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;

    if(canHandler->RBCAN_WriteData(mb)){
        usleep(35*1000);
        mb.channel = CAN_CHANNEL;
        mb.id = ID_RCV_GENERAL;
        canHandler->RBCAN_ReadData(&mb);
        if(mb.status != RBCAN_NODATA && mb.data[0]==29){
            Joints[0].HCB_Info.SUP_PRES = (int)((mb.data[1])|(mb.data[2]<<8));
            Joints[0].HCB_Info.RET_PRES = (int)((mb.data[3])|(mb.data[4]<<8));
            std::cout << "=================================== " << std::endl;
            std::cout << "Supply Pressure : " << (Joints[0].HCB_Info.SUP_PRES) << std::endl;
            std::cout << "Return Pressure : " << (Joints[0].HCB_Info.RET_PRES) << std::endl;
            std::cout << "=================================== " << std::endl;
            mb.status = RBCAN_NODATA;
        }else{
            std::cout << ">>> ASK 29 - Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is not connected. \n";
        }
    }
}
int HydraulicActuatorController::HCB_CMD_SupplyAndReturnPressure(int P_sup, int P_ret){
    // Unit : bar
    Joints[0].HCB_Info.SUP_PRES = P_sup;
    Joints[0].HCB_Info.RET_PRES = P_ret;

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 129;
    mb.data[1] = (unsigned char)(P_sup & 0x000000FF);
    mb.data[2] = (unsigned char)((P_sup>>8) & 0x000000FF);
    mb.data[3] = (unsigned char)(P_ret & 0x000000FF);
    mb.data[4] = (unsigned char)((P_ret>>8) & 0x000000FF);
    mb.dlc = 5;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}

void HydraulicActuatorController::HCB_ASK_JointEncLimit(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 30;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;

    if(canHandler->RBCAN_WriteData(mb)){
        usleep(35*1000);
        mb.channel = CAN_CHANNEL;
        mb.id = ID_RCV_GENERAL;
        canHandler->RBCAN_ReadData(&mb);
        if(mb.status != RBCAN_NODATA && mb.data[0]==30){
            Joints[0].HCB_Info.JOINTENC_LIMIT_MINUS = (int16_t)((mb.data[1])|(mb.data[2]<<8));
            Joints[0].HCB_Info.JOINTENC_LIMIT_PLUS = (int16_t)((mb.data[3])|(mb.data[4]<<8));
            std::cout << "=================================== " << std::endl;
            std::cout << "Joint Enc. Negative Limit : " << (Joints[0].HCB_Info.JOINTENC_LIMIT_MINUS) << std::endl;
            std::cout << "Joint Enc. Positive Limit : " << (Joints[0].HCB_Info.JOINTENC_LIMIT_PLUS) << std::endl;
            std::cout << "=================================== " << std::endl;
            mb.status = RBCAN_NODATA;
        }else{
            std::cout << ">>> ASK 30 - Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is not connected. \n";
        }
    }
}
int HydraulicActuatorController::HCB_CMD_JointEncLimit(int LoLimit, int UpLimit){
    // Unit : pulse
    Joints[0].HCB_Info.JOINTENC_LIMIT_MINUS = LoLimit;
    Joints[0].HCB_Info.JOINTENC_LIMIT_PLUS = UpLimit;

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 130;
    mb.data[1] = (unsigned char)(LoLimit & 0x000000FF);
    mb.data[2] = (unsigned char)((LoLimit>>8) & 0x000000FF);
    mb.data[3] = (unsigned char)(UpLimit & 0x000000FF);
    mb.data[4] = (unsigned char)((UpLimit>>8) & 0x000000FF);
    mb.dlc = 5;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}

void HydraulicActuatorController::HCB_ASK_PistonStroke(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 31;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;

    if(canHandler->RBCAN_WriteData(mb)){
        usleep(35*1000);
        mb.channel = CAN_CHANNEL;
        mb.id = ID_RCV_GENERAL;
        canHandler->RBCAN_ReadData(&mb);
        if(mb.status != RBCAN_NODATA && mb.data[0]==31){
            Joints[0].HCB_Info.PIS_STROKE = (int)((mb.data[1])|(mb.data[2]<<8));
            std::cout << "=================================== " << std::endl;
            std::cout << "Piston Stroke : " << (Joints[0].HCB_Info.PIS_STROKE) << std::endl;
            std::cout << "=================================== " << std::endl;
            mb.status = RBCAN_NODATA;
        }else{
            std::cout << ">>> ASK 31 - Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is not connected. \n";
        }
    }
}
int HydraulicActuatorController::HCB_CMD_PistonStroke(int Stroke){
    // Unit : mm(linear) / deg(rotary)
    Joints[0].HCB_Info.PIS_STROKE = Stroke;

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 131;
    mb.data[1] = (unsigned char)(Stroke & 0x000000FF);
    mb.data[2] = (unsigned char)((Stroke>>8) & 0x000000FF);
    mb.dlc = 3;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}

void HydraulicActuatorController::HCB_ASK_ValvePositionLimit(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 32;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;

    if(canHandler->RBCAN_WriteData(mb)){
        usleep(35*1000);
        mb.channel = CAN_CHANNEL;
        mb.id = ID_RCV_GENERAL;
        canHandler->RBCAN_ReadData(&mb);
        if(mb.status != RBCAN_NODATA && mb.data[0]==32){
            Joints[0].HCB_Info.VALVEPOS_LIMIT_MINUS = (int16_t)((mb.data[1])|(mb.data[2]<<8));
            Joints[0].HCB_Info.VALVEPOS_LIMIT_PLUS = (int16_t)((mb.data[3])|(mb.data[4]<<8));
            std::cout << "=================================== " << std::endl;
            std::cout << "Valve Pos. Negative Limit : " << (Joints[0].HCB_Info.VALVEPOS_LIMIT_MINUS) << std::endl;
            std::cout << "Valve Pos. Positive Limit : " << (Joints[0].HCB_Info.VALVEPOS_LIMIT_PLUS) << std::endl;
            std::cout << "=================================== " << std::endl;
            mb.status = RBCAN_NODATA;
        }else{
            std::cout << ">>> ASK 32 - Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is not connected. \n";
        }
    }
}
int HydraulicActuatorController::HCB_CMD_ValvePositionLimit(int LoLimit, int UpLimit){
    // Unit : Pulse
    Joints[0].HCB_Info.VALVEPOS_LIMIT_MINUS = LoLimit;
    Joints[0].HCB_Info.VALVEPOS_LIMIT_PLUS = UpLimit;

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 132;
    mb.data[1] = (unsigned char)(LoLimit & 0x000000FF);
    mb.data[2] = (unsigned char)((LoLimit>>8) & 0x000000FF);
    mb.data[3] = (unsigned char)(UpLimit & 0x000000FF);
    mb.data[4] = (unsigned char)((UpLimit>>8) & 0x000000FF);
    mb.dlc = 5;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}

void HydraulicActuatorController::HCB_ASK_EncoderPulsePerPosition(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 33;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;

    if(canHandler->RBCAN_WriteData(mb)){
        usleep(35*1000);
        mb.channel = CAN_CHANNEL;
        mb.id = ID_RCV_GENERAL;
        canHandler->RBCAN_ReadData(&mb);
        if(mb.status != RBCAN_NODATA && mb.data[0]==33){
            int16_t temp = (int)((mb.data[1])|(mb.data[2]<<8));
            Joints[0].HCB_Info.JOINTENC_PPP = (double)temp;
            std::cout << "=================================== " << std::endl;
            std::cout << "Joint Encoder Pulse per Position : " << (Joints[0].HCB_Info.JOINTENC_PPP) << std::endl;
            std::cout << "=================================== " << std::endl;
            mb.status = RBCAN_NODATA;
        }else{
            std::cout << ">>> ASK 33 - Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is not connected. \n";
        }
    }
}
int HydraulicActuatorController::HCB_CMD_EncoderPulsePerPosition(double PPP){
    // Unit : Pulse/mm(Linear) / Pulse/deg(Rotary)
    Joints[0].HCB_Info.JOINTENC_PPP = PPP;

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 133;
    int16_t temp = (int16_t)(PPP);
    mb.data[1] = (unsigned char)(temp & 0x000000FF);
    mb.data[2] = (unsigned char)((temp>>8) & 0x000000FF);
    mb.dlc = 3;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}

void HydraulicActuatorController::HCB_ASK_SensorPulsePerForceTorque(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 34;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;

    if(canHandler->RBCAN_WriteData(mb)){
        usleep(35*1000);
        mb.channel = CAN_CHANNEL;
        mb.id = ID_RCV_GENERAL;
        canHandler->RBCAN_ReadData(&mb);
        if(mb.status != RBCAN_NODATA && mb.data[0]==34){
            int temp = (int)((mb.data[1])|(mb.data[2]<<8));
            Joints[0].HCB_Info.FORCESEN_PPF = (double)temp*0.001;
            std::cout << "=================================== " << std::endl;
            std::cout << "Sensor Pulse per Force/Torque : " << (Joints[0].HCB_Info.FORCESEN_PPF) << std::endl;
            std::cout << "=================================== " << std::endl;
            mb.status = RBCAN_NODATA;
        }else{
            std::cout << ">>> ASK 34 - Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is not connected. \n";
        }
    }
}
int HydraulicActuatorController::HCB_CMD_SensorPulsePerForceTorque(double PPF){
    // Unit : Pulse/mm(Linear) / Pulse/deg(Rotary)
    Joints[0].HCB_Info.FORCESEN_PPF = PPF;

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 134;
    int temp = (int)(PPF*1000.0);


    mb.data[1] = (unsigned char)(temp & 0x000000FF);
    mb.data[2] = (unsigned char)((temp>>8) & 0x000000FF);
    mb.dlc = 3;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}

void HydraulicActuatorController::HCB_ASK_SensorPulsePerPressure(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 35;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;

    if(canHandler->RBCAN_WriteData(mb)){
        usleep(35*1000);
        mb.channel = CAN_CHANNEL;
        mb.id = ID_RCV_GENERAL;
        canHandler->RBCAN_ReadData(&mb);
        if(mb.status != RBCAN_NODATA && mb.data[0]==35){
            int temp_A = (int)((mb.data[1])|(mb.data[2]<<8));
            int temp_B = (int)((mb.data[3])|(mb.data[4]<<8));
            Joints[0].HCB_Info.PRESSEN_PPP_A = (double)temp_A*0.01;
            Joints[0].HCB_Info.PRESSEN_PPP_B = (double)temp_B*0.01;
            std::cout << "=================================== " << std::endl;
            std::cout << "Sensor Pulse per Pressure A port : " << (Joints[0].HCB_Info.PRESSEN_PPP_A) << std::endl;
            std::cout << "Sensor Pulse per Pressure B port : " << (Joints[0].HCB_Info.PRESSEN_PPP_B) << std::endl;
            std::cout << "=================================== " << std::endl;
            mb.status = RBCAN_NODATA;
        }else{
            std::cout << ">>> ASK 35 - Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is not connected. \n";
        }
    }
}
int HydraulicActuatorController::HCB_CMD_SensorPulsePerPressure(double PPP_A, double PPP_B){
    // Unit : Pulse/mm(Linear) / Pulse/deg(Rotary)
    Joints[0].HCB_Info.PRESSEN_PPP_A = PPP_A;
    Joints[0].HCB_Info.PRESSEN_PPP_B = PPP_B;

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 135;
    int temp_A = PPP_A*100.0;
    int temp_B = PPP_B*100.0;
    mb.data[1] = (unsigned char)(temp_A & 0x000000FF);
    mb.data[2] = (unsigned char)((temp_A>>8) & 0x000000FF);
    mb.data[3] = (unsigned char)(temp_B & 0x000000FF);
    mb.data[4] = (unsigned char)((temp_B>>8) & 0x000000FF);
    mb.dlc = 5;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}

void HydraulicActuatorController::HCB_ASK_ConstantFriction(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 36;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;

    if(canHandler->RBCAN_WriteData(mb)){
        usleep(35*1000);
        mb.channel = CAN_CHANNEL;
        mb.id = ID_RCV_GENERAL;
        canHandler->RBCAN_ReadData(&mb);
        if(mb.status != RBCAN_NODATA && mb.data[0]==36){
            int temp_Fric = (int16_t)((mb.data[1])|(mb.data[2]<<8));
            Joints[0].HCB_Info.CONST_FRIC = ((double)temp_Fric)/10.0;
            std::cout << "=================================== " << std::endl;
            std::cout << "Constant Friction : " << (Joints[0].HCB_Info.CONST_FRIC) << std::endl;
            std::cout << "=================================== " << std::endl;
            mb.status = RBCAN_NODATA;
        }else{
            std::cout << ">>> ASK 36 - Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is not connected. \n";
        }
    }
}
int HydraulicActuatorController::HCB_CMD_ConstantFriction(double Fric){
    // Unit : N(Linear) / Nm(Rotary)
    Joints[0].HCB_Info.CONST_FRIC = Fric;

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 136;
    int temp = (int)(10.0*Fric);
    mb.data[1] = (unsigned char)(temp & 0x000000FF);
    mb.data[2] = (unsigned char)((temp>>8) & 0x000000FF);
    mb.dlc = 3;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}

void HydraulicActuatorController::HCB_ASK_HomeposOffset(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 40;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;

    if(canHandler->RBCAN_WriteData(mb)){
        usleep(35*1000);
        mb.channel = CAN_CHANNEL;
        mb.id = ID_RCV_GENERAL;
        canHandler->RBCAN_ReadData(&mb);
        if(mb.status != RBCAN_NODATA && mb.data[0]==40){
            int temp_HomeposOffset = (int16_t)((mb.data[1])|(mb.data[2]<<8));
            Joints[0].HCB_Info.HOMEPOS_OFFSET = temp_HomeposOffset;
            std::cout << "=================================== " << std::endl;
            std::cout << "HomePos Offset : " << (Joints[0].HCB_Info.HOMEPOS_OFFSET) << std::endl;
            std::cout << "=================================== " << std::endl;
            mb.status = RBCAN_NODATA;
        }else{
            std::cout << ">>> ASK 37 - Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is not connected. \n";
        }
    }
}
int HydraulicActuatorController::HCB_CMD_HomeposOffset(double homeposoffset){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 140;
    int temp = (int)(homeposoffset);
    mb.data[1] = (unsigned char)(temp & 0x000000FF);
    mb.data[2] = (unsigned char)((temp>>8) & 0x000000FF);
    mb.dlc = 3;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}

void HydraulicActuatorController::HCB_ASK_HomeposValveOpening(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 41;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;

    if(canHandler->RBCAN_WriteData(mb)){
        usleep(35*1000);
        mb.channel = CAN_CHANNEL;
        mb.id = ID_RCV_GENERAL;
        canHandler->RBCAN_ReadData(&mb);
        if(mb.status != RBCAN_NODATA && mb.data[0]==41){
            int temp_HomeposValveOpening = (int16_t)((mb.data[1])|(mb.data[2]<<8));
            Joints[0].HCB_Info.HOMEPOS_VALVE_OPENING = temp_HomeposValveOpening;
            std::cout << "=================================== " << std::endl;
            std::cout << "HomePos Valve Opening : " << (Joints[0].HCB_Info.HOMEPOS_VALVE_OPENING) << std::endl;
            std::cout << "=================================== " << std::endl;
            mb.status = RBCAN_NODATA;
        }else{
            std::cout << ">>> ASK 38 - Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is not connected. \n";
        }
    }
}
int HydraulicActuatorController::HCB_CMD_HomeposValveOpening(double homeposvalveopening){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 141;
    int temp = (int)(homeposvalveopening);
    mb.data[1] = (unsigned char)(temp & 0x000000FF);
    mb.data[2] = (unsigned char)((temp>>8) & 0x000000FF);
    mb.dlc = 3;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}

void HydraulicActuatorController::HCB_ASK_VALVE_GAIN_PLUS(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 37;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;

    if(canHandler->RBCAN_WriteData(mb)){
        usleep(35*1000);
        mb.channel = CAN_CHANNEL;
        mb.id = ID_RCV_GENERAL;
        canHandler->RBCAN_ReadData(&mb);
        if(mb.status != RBCAN_NODATA && mb.data[0]==37){
            double Valve_Gain_1 = (double)(mb.data[1]) * 0.02;
            double Valve_Gain_2 = (double)(mb.data[2]) * 0.02;
            double Valve_Gain_3 = (double)(mb.data[3]) * 0.02;
            double Valve_Gain_4 = (double)(mb.data[4]) * 0.02;
            double Valve_Gain_5 = (double)(mb.data[5]) * 0.02;
            Joints[0].HCB_Info.VALVE_GAIN_PLUS[0] = Valve_Gain_1;
            Joints[0].HCB_Info.VALVE_GAIN_PLUS[1] = Valve_Gain_2;
            Joints[0].HCB_Info.VALVE_GAIN_PLUS[2] = Valve_Gain_3;
            Joints[0].HCB_Info.VALVE_GAIN_PLUS[3] = Valve_Gain_4;
            Joints[0].HCB_Info.VALVE_GAIN_PLUS[4] = Valve_Gain_5;
            std::cout << "=================================== " << std::endl;
            std::cout << "Plus valve gain :" <<"   1V: "<< Valve_Gain_1 << "  2V: "<< Valve_Gain_2 << " 3V: "<< Valve_Gain_3 << "    4V: "<< Valve_Gain_4 << "   5V: "<< Valve_Gain_5 << std::endl;
            std::cout << "=================================== " << std::endl;
            mb.status = RBCAN_NODATA;
        }else{
            std::cout << ">>> ASK 36 - Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is not connected. \n";
        }
    }
}

void HydraulicActuatorController::HCB_ASK_VALVE_GAIN_MINUS(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 38;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;

    if(canHandler->RBCAN_WriteData(mb)){
        usleep(35*1000);
        mb.channel = CAN_CHANNEL;
        mb.id = ID_RCV_GENERAL;
        canHandler->RBCAN_ReadData(&mb);
        if(mb.status != RBCAN_NODATA && mb.data[0]==38){
            double Valve_Gain_1 = (double)(mb.data[1]) * 0.02;
            double Valve_Gain_2 = (double)(mb.data[2]) * 0.02;
            double Valve_Gain_3 = (double)(mb.data[3]) * 0.02;
            double Valve_Gain_4 = (double)(mb.data[4]) * 0.02;
            double Valve_Gain_5 = (double)(mb.data[5]) * 0.02;
            Joints[0].HCB_Info.VALVE_GAIN_MINUS[0] = Valve_Gain_1;
            Joints[0].HCB_Info.VALVE_GAIN_MINUS[1] = Valve_Gain_2;
            Joints[0].HCB_Info.VALVE_GAIN_MINUS[2] = Valve_Gain_3;
            Joints[0].HCB_Info.VALVE_GAIN_MINUS[3] = Valve_Gain_4;
            Joints[0].HCB_Info.VALVE_GAIN_MINUS[4] = Valve_Gain_5;
            std::cout << "=================================== " << std::endl;
            std::cout << "Plus valve gain :" <<"   1V: "<< Valve_Gain_1 << "  2V: "<< Valve_Gain_2 << "   3V: "<< Valve_Gain_3 << "   4V: "<< Valve_Gain_4 << "   5V: "<< Valve_Gain_5 << std::endl;
            std::cout << "=================================== " << std::endl;
            mb.status = RBCAN_NODATA;
        }else{
            std::cout << ">>> ASK 36 - Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is not connected. \n";
        }
    }
}
//int HydraulicActuatorController::HCB_CMD_ValveGain_Plus(double plus_valve_gain){
//    // Unit : N(Linear) / Nm(Rotary)
//    Joints[0].HCB_Info.CONST_FRIC = plus_valve_gain;

//    RBCAN_MB mb;
//    mb.channel = CAN_CHANNEL;
//    mb.data[0] = 136;
//    int temp = (int)(10.0*Fric);
//    mb.data[1] = (unsigned char)(temp & 0x000000FF);
//    mb.data[2] = (unsigned char)((temp>>8) & 0x000000FF);
//    mb.dlc = 3;
//    mb.id = ID_SEND_GENERAL;
//    return canHandler->RBCAN_WriteData(mb);
//}

int HydraulicActuatorController::HCB_CMD_ErrorClear(){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 150;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}

// Control Parameter Setting END
// ============================================================================
// Data&Info Read START

int HydraulicActuatorController::HCB_Read_Information(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.id = ID_RCV_GENERAL;
    if(canHandler->RBCAN_ReadData(&mb)) {
        if(mb.status != RBCAN_NODATA && mb.data[0]==0 && mb.data[1]==BOARD_ID) {
            Joints[0].HCB_Info.CAN_FREQ           = (int)((mb.data[2])|(mb.data[3]<<8));
            Joints[0].HCB_Info.FET_ONOFF          = (mb.data[4]&0b10000000);
            Joints[0].HCB_Info.BIGERROR_ONOFF     = (mb.data[4]&0b01000000);
            Joints[0].HCB_Info.ENCERROR_ONOFF     = (mb.data[4]&0b00100000);
            Joints[0].HCB_Info.CANERROR_ONOFF     = (mb.data[4]&0b00010000);
            Joints[0].HCB_Info.HOMEERROR_ONOFF    = (mb.data[4]&0b00001000);
            Joints[0].HCB_Info.PLIMITERROR_ONOFF  = (mb.data[4]&0b00000100);
            Joints[0].HCB_Info.LOGICERROR_ONOFF   = (mb.data[4]&0b00000010);
            Joints[0].HCB_Info.INPUTERROR_ONOFF   = (mb.data[4]&0b00000001);
            switch(mb.data[5]){
            case 0:
                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_NOACT; break;
            case 1:
                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_VALVE_OPENLOOP; break;
            case 2:
                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_VALVE_POS; break;
            case 3:
                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_POS_FORCE_PWM; break;
            case 4:
                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_POS_FORCE_VALVE; break;
            case 5:
                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_POS_FORCE_LEARN; break;
            case 6:
                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_POS_PRES_PWM; break;
            case 7:
                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_POS_PRES_VALVE; break;
            case 8:
                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_POS_PRES_LEARN; break;
            case 9:
                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_PWM_SINE_OPENLOOL; break;
            case 20:
                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_FORCE_NULL; break;
            case 21:
                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_VALVE_NULL; break;
            case 22:
                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_FINDHOME; break;
            case 23:
                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_FLOWGAIN_TUNE; break;
            default:
                break;
            }
            if(mb.data[6]==0){
                Joints[0].HCB_Info.OPERATION_MODE = HCB_OPER_MODE_SW;
            } else {
                Joints[0].HCB_Info.OPERATION_MODE = HCB_OPER_MODE_SH;
            }
            switch(mb.data[7]){
            }
//            std::cout << ">>> MC: Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is \033[31m connected!! \033[0m [ch " << CAN_CHANNEL << "]\n";
            ConnectionStatus = true;
            mb.status = RBCAN_NODATA;
            return true;
        } else {
//            std::cout << ">>> ASK 0 - MC: Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is \033[31mfailed \033[0mto connect.[ch " << CAN_CHANNEL << "]\n";
            ConnectionStatus = false;
            return false;
        }
    } else {
        return false;
    }
}

int HydraulicActuatorController::HCB_SEE_Information(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.id = ID_RCV_GENERAL;
    if(canHandler->RBCAN_SeeData(&mb)) {
        if(mb.status != RBCAN_NODATA && mb.data[0]==0 && mb.data[1]==BOARD_ID) {
            Joints[0].HCB_Info.CAN_FREQ           = (int)((mb.data[2])|(mb.data[3]<<8));
            Joints[0].HCB_Info.FET_ONOFF          = (mb.data[4]&0b10000000);
            Joints[0].HCB_Info.BIGERROR_ONOFF     = (mb.data[4]&0b01000000);
            Joints[0].HCB_Info.ENCERROR_ONOFF     = (mb.data[4]&0b00100000);
            Joints[0].HCB_Info.CANERROR_ONOFF     = (mb.data[4]&0b00010000);
            Joints[0].HCB_Info.HOMEERROR_ONOFF    = (mb.data[4]&0b00001000);
            Joints[0].HCB_Info.PLIMITERROR_ONOFF  = (mb.data[4]&0b00000100);
            Joints[0].HCB_Info.LOGICERROR_ONOFF   = (mb.data[4]&0b00000010);
            Joints[0].HCB_Info.INPUTERROR_ONOFF   = (mb.data[4]&0b00000001);
            switch(mb.data[5]){
            case 0:
                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_NOACT; break;
            case 1:
                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_VALVE_OPENLOOP; break;
            case 2:
                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_VALVE_POS; break;
            case 3:
                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_POS_FORCE_PWM; break;
            case 4:
                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_POS_FORCE_VALVE; break;
            case 5:
                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_POS_FORCE_LEARN; break;
            case 6:
                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_POS_PRES_PWM; break;
            case 7:
                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_POS_PRES_VALVE; break;
            case 8:
                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_POS_PRES_LEARN; break;
            case 9:
                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_PWM_SINE_OPENLOOL; break;
            case 20:
                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_FORCE_NULL; break;
            case 21:
                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_VALVE_NULL; break;
            case 22:
                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_FINDHOME; break;
            case 23:
                Joints[0].HCB_Info.CONTROL_MODE = HCB_CTRL_MODE_FLOWGAIN_TUNE; break;
            default:
                break;
            }
            if(mb.data[6]==0){
                Joints[0].HCB_Info.OPERATION_MODE = HCB_OPER_MODE_SW;
            } else {
                Joints[0].HCB_Info.OPERATION_MODE = HCB_OPER_MODE_SH;
            }
            switch(mb.data[7]){
            }

            std::cout <<  Joints[0].HCB_Info.FET_ONOFF
                      <<  Joints[0].HCB_Info.BIGERROR_ONOFF
                      <<  Joints[0].HCB_Info.ENCERROR_ONOFF
                      <<  Joints[0].HCB_Info.CANERROR_ONOFF
                      <<  Joints[0].HCB_Info.HOMEERROR_ONOFF
                      <<  Joints[0].HCB_Info.PLIMITERROR_ONOFF
                      <<  Joints[0].HCB_Info.LOGICERROR_ONOFF
                      <<  Joints[0].HCB_Info.INPUTERROR_ONOFF << std::endl;

            ConnectionStatus = true;
//            mb.status = RBCAN_NODATA;
            return true;
        } else {
//            std::cout << ">>> ASK 0 - MC: Board(" << BOARD_ID << ": " << BOARD_NAME.toStdString().data() << ") is \033[31mfailed \033[0mto connect.[ch " << CAN_CHANNEL << "]\n";
            ConnectionStatus = false;
            return false;
        }
    } else {
        return false;
    }
}

int HydraulicActuatorController::HCB_Read_EncoderData(void){
    int16_t temp_pos, temp_vel, temp_force_pulse;
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.id = ID_RCV_POSVEL;
    canHandler->RBCAN_ReadData(&mb);
    if(mb.status != RBCAN_NODATA){
        temp_pos = (int16_t)((mb.data[0])|(mb.data[1]<<8)); // Unit : deg or mm
        temp_vel = (int16_t)((mb.data[2])|(mb.data[3]<<8)); // Unit : deg/s or mm/s
        temp_force_pulse = (int16_t)((mb.data[4])|(mb.data[5]<<8)); // Unit : sensor pulse (0~4095)

        // @ joint : maximum 100deg(or mm) > multiply 200
        //           maximum 1000deg/s(or mm/s) multiply 20
        // @ force sensor : maximum 4096 > multiply 10
        Joints[0].HCB_Data.CurrentPosition = (double)temp_pos/200.0;
        Joints[0].HCB_Data.CurrentVelocity = (double)temp_vel/20.0;
        Joints[0].HCB_Data.CurrentForce = (double)temp_force_pulse/10.0/PULSE_PER_FORCETORQUE;
        mb.status = RBCAN_NODATA;

        return true;
    }
    return false;
}


int HydraulicActuatorController::HCB_Read_ValvePosData(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.id = ID_RCV_VALVEPOS;
    canHandler->RBCAN_ReadData(&mb);
    if(mb.status != RBCAN_NODATA){
        Joints[0].HCB_Data.CurrentValvePos = (int16_t)((mb.data[0])|(mb.data[1]<<8));
        mb.status = RBCAN_NODATA;
        return true;
    }
    return false;
}

int HydraulicActuatorController::HCB_Read_OtherInfoData(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.id = ID_RCV_OTHERINFO;
    canHandler->RBCAN_ReadData(&mb);
    if(mb.status != RBCAN_NODATA){
        int16_t temp_INFO1 = (int16_t)((mb.data[0])|(mb.data[1]<<8));
        int16_t temp_INFO2 = (int16_t)((mb.data[2])|(mb.data[3]<<8));
        int16_t temp_INFO3 = (int16_t)((mb.data[4])|(mb.data[5]<<8));
        int16_t temp_INFO4 = (int16_t)((mb.data[6])|(mb.data[7]<<8));

        Joints[0].HCB_Data.CurrentPressureA = ((double)temp_INFO1)/100.0;
        Joints[0].HCB_Data.CurrentPressureB = ((double)temp_INFO2)/100.0;
        Joints[0].HCB_Data.CurrentTempData1 = (double)temp_INFO3;
        Joints[0].HCB_Data.CurrentTempData2 = (double)temp_INFO4/10.0/PULSE_PER_FORCETORQUE;
        mb.status = RBCAN_NODATA;
        return true;
    }
    return false;
}

// Data Read END
// ============================================================================
// Reference Send START

int HydraulicActuatorController::HCB_ResetReference_PosVel(bool PsVar_OnOff)
{
    int16_t ref_pos,ref_vel,ref_force_pulse;
    int16_t ref_pressure;

    ref_pos = 0;
    ref_vel = 0;
    ref_force_pulse = 0;
    ref_pressure = 100;

    Joints[0].HCB_Ref.ReferencePosition_last = ref_pos;
    Joints[0].HCB_Ref.ReferenceVelocity_last = ref_vel;
    Joints[0].HCB_Ref.ReferenceForceTorque_last = ref_force_pulse;
    Joints[0].HCB_Ref.ReferencePumpPressure_last = ref_pressure;

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.id = ID_SEND_POSVEL; // Position and Velocity Reference
    if(PsVar_OnOff) {
        mb.dlc = 8;
        mb.data[0] = (unsigned char)(ref_pos & 0xFF);
        mb.data[1] = (unsigned char)((ref_pos>>8) & 0xFF);
        mb.data[2] = (unsigned char)(ref_vel & 0xFF);
        mb.data[3] = (unsigned char)((ref_vel>>8) & 0xFF);
        mb.data[4] = (unsigned char)(ref_force_pulse & 0xFF);
        mb.data[5] = (unsigned char)((ref_force_pulse>>8) & 0xFF);
        mb.data[6] = (unsigned char)(ref_pressure & 0xFF);
        mb.data[7] = (unsigned char)((ref_pressure>>8) & 0xFF);
    } else {
        mb.dlc = 6;
        mb.data[0] = (unsigned char)(ref_pos & 0xFF);
        mb.data[1] = (unsigned char)((ref_pos>>8) & 0xFF);
        mb.data[2] = (unsigned char)(ref_vel & 0xFF);
        mb.data[3] = (unsigned char)((ref_vel>>8) & 0xFF);
        mb.data[4] = (unsigned char)(ref_force_pulse & 0xFF);
        mb.data[5] = (unsigned char)((ref_force_pulse>>8) & 0xFF);
    }
    return canHandler->RBCAN_WriteDataDirectly(mb);
}

int HydraulicActuatorController::HCB_SendReference_PosVel(bool PsVar_OnOff)
{
    double ref1_n = Joints[0].HCB_Ref.ReferencePosition;
    double ref1_l = Joints[0].HCB_Ref.ReferencePosition_last;
    double ref2_n = Joints[0].HCB_Ref.ReferenceVelocity;
    double ref2_l = Joints[0].HCB_Ref.ReferenceVelocity_last;
    double ref3_n = Joints[0].HCB_Ref.ReferenceForceTorque;
    double ref3_l = Joints[0].HCB_Ref.ReferenceForceTorque_last;
    double ref4_n = Joints[0].HCB_Ref.ReferencePumpPressure;
    double ref4_l = Joints[0].HCB_Ref.ReferencePumpPressure_last;

    if(ref1_n!=ref1_l || ref2_n!=ref2_l || ref3_n!=ref3_l || ref4_n!=ref4_l) {
        int16_t ref_pos,ref_vel,ref_force_pulse;
        int16_t ref_pressure;

        // @ joint : maximum 100 deg(or mm) > multiply 200
        //           maximum 1000 deg/s(or mm/s) multiply 20
        // @ force sensor : maximum 4096 pulse > multiply 10
        // @ force sensor : maximum 210 bar > multiply 100

        ref_pos = (int16_t)(Joints[0].HCB_Ref.ReferencePosition*200.0);
        ref_vel = (int16_t)(Joints[0].HCB_Ref.ReferenceVelocity*20.0);
        ref_force_pulse = (int16_t)(Joints[0].HCB_Ref.ReferenceForceTorque*PULSE_PER_FORCETORQUE*10.0);
        ref_pressure = (int16_t)(Joints[0].HCB_Ref.ReferencePumpPressure*100.0);

        Joints[0].HCB_Ref.ReferencePosition_last = Joints[0].HCB_Ref.ReferencePosition;
        Joints[0].HCB_Ref.ReferenceVelocity_last = Joints[0].HCB_Ref.ReferenceVelocity;
        Joints[0].HCB_Ref.ReferenceForceTorque_last = Joints[0].HCB_Ref.ReferenceForceTorque;
        Joints[0].HCB_Ref.ReferencePumpPressure_last = Joints[0].HCB_Ref.ReferencePumpPressure;

        RBCAN_MB mb;
        mb.channel = CAN_CHANNEL;
        mb.id = ID_SEND_POSVEL; // Position and Velocity Reference
        if(PsVar_OnOff) {
            mb.dlc = 8;
            mb.data[0] = (unsigned char)(ref_pos & 0xFF);
            mb.data[1] = (unsigned char)((ref_pos>>8) & 0xFF);
            mb.data[2] = (unsigned char)(ref_vel & 0xFF);
            mb.data[3] = (unsigned char)((ref_vel>>8) & 0xFF);
            mb.data[4] = (unsigned char)(ref_force_pulse & 0xFF);
            mb.data[5] = (unsigned char)((ref_force_pulse>>8) & 0xFF);
            mb.data[6] = (unsigned char)(ref_pressure & 0xFF);
            mb.data[7] = (unsigned char)((ref_pressure>>8) & 0xFF);
        } else {
            mb.dlc = 6;
            mb.data[0] = (unsigned char)(ref_pos & 0xFF);
            mb.data[1] = (unsigned char)((ref_pos>>8) & 0xFF);
            mb.data[2] = (unsigned char)(ref_vel & 0xFF);
            mb.data[3] = (unsigned char)((ref_vel>>8) & 0xFF);
            mb.data[4] = (unsigned char)(ref_force_pulse & 0xFF);
            mb.data[5] = (unsigned char)((ref_force_pulse>>8) & 0xFF);
        }
        return canHandler->RBCAN_WriteDataDirectly(mb);
    }
    return false;
}

int HydraulicActuatorController::HCB_SendReference_ValvePos(){

    double ref1_n = Joints[0].HCB_Ref.ReferenceValvePos;
    double ref1_l = Joints[0].HCB_Ref.ReferenceValvePos_last;

    if(ref1_n!=ref1_l)
    {
        int ref_valvepos = (int16_t)(Joints[0].HCB_Ref.ReferenceValvePos);

        RBCAN_MB mb;
        mb.channel = CAN_CHANNEL;
        mb.id = ID_SEND_VALVEPOS; // Valve Position Reference
        mb.dlc = 2;
        mb.data[0] = (unsigned char)(ref_valvepos & 0x000000FF);
        mb.data[1] = (unsigned char)((ref_valvepos>>8) & 0x000000FF);

        Joints[0].HCB_Ref.ReferenceValvePos_last = Joints[0].HCB_Ref.ReferenceValvePos;

        return canHandler->RBCAN_WriteDataDirectly(mb);
    }
    return false;
}

// Reference Send END
// ============================================================================


////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
// Hydraulic Pump Controller


HydraulicPumpController::HydraulicPumpController() {
    ONOFF_SendRefSpeed = false;
    REQUEST_ONOFF_DATA = false;
    ReferencePumpVelocity = 0.0;
}

void HydraulicPumpController::GetDBData(DB_PC db) {
    BOARD_ID = db.BOARD_ID;
    CAN_CHANNEL = db.CAN_CHANNEL;

    ID_SEND_GENERAL = db.ID_SEND_GENERAL;
    ID_SEND_VELOCITY = db.ID_SEND_VELOCITY;

    ID_RCV_GENERAL = db.ID_RCV_GENERAL;
    ID_RCV_VELOCITY = db.ID_RCV_VELOCITY;
    ID_RCV_PRESSURE = db.ID_RCV_PRESSURE;
}

void HydraulicPumpController::AddCANMailBox_RCVDATA(){
    canHandler->RBCAN_AddMailBox(ID_RCV_GENERAL);
    canHandler->RBCAN_AddMailBox(ID_RCV_VELOCITY);
    canHandler->RBCAN_AddMailBox(ID_RCV_PRESSURE);
//    std::cout<<"MailBoxID : "<<ID_RCV_GENERAL <<" "<<ID_RCV_POSVEL <<" "<<ID_RCV_FORCE <<" "<<ID_RCV_PRES <<" "<<ID_RCV_PWM <<" "<<ID_RCV_VALVEPOS <<" "<<std::endl;
}

/****************************************************************
 *  PumpController - CAN DATA TX
 ****************************************************************/

int HydraulicPumpController::CANCheck(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 0; //Ask Board status(information)
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;

    if(canHandler->RBCAN_WriteData(mb)){
        usleep(10*1000);
        mb.channel = CAN_CHANNEL;
        mb.id = ID_RCV_GENERAL;
        canHandler->RBCAN_ReadData(&mb);

        if(mb.status != RBCAN_NODATA && mb.data[0]==0){
            std::cout << ">>> PC: Board(" << BOARD_ID << ") is \033[32mconnected.\033[0m[ch " << CAN_CHANNEL << "]\n";
            ConnectionStatus = true;
            mb.status = RBCAN_NODATA;
            return true;
        }else{
            std::cout << ">>> PC: Board(" << BOARD_ID << ") is \033[31mfailed \033[0mto connect.[ch " << CAN_CHANNEL << "]\n";
            ConnectionStatus = false;
            return false;
        }
    }
    else return false;
}

int  HydraulicPumpController::CANChannel_Arrange(void){
    RBCAN_MB mb;
    std::cout << " ======== Board(" << BOARD_ID << ": PUMP) is checking channels... ========\n";

    int NUM_CHECK_CAN_CH = min(MAX_HCB_CAN_CHANNEL,_NO_OF_COMM_CH);
    for(int idx = 0; idx<NUM_CHECK_CAN_CH; idx++){
        mb.data[0] = 0; //Ask Board status(information)
        mb.dlc = 1;
        mb.id = ID_SEND_GENERAL;
        mb.channel = idx;
        if(canHandler->RBCAN_WriteData(mb)){
            usleep(100*1000);
            mb.channel = idx;
            mb.id = ID_RCV_GENERAL;
            canHandler->RBCAN_ReadData(&mb);

            if(mb.status != RBCAN_NODATA && mb.data[0]==0){
                std::cout << ">>> Board(" << BOARD_ID << ": PUMP) is \033[32mconnected \033[0mto channel [" << idx << "]\n";
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

    std::cout << ">>> Board(" << BOARD_ID << ": PUMP) is \033[31mnot connected.\033[0m \n";
    std::cout << " ========================================================\n\n";
    ConnectionStatus = false;
    mb.status = RBCAN_NODATA;
    return true;
}

int HydraulicPumpController::ASK_SPEED_REF(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 81;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;

    if(canHandler->RBCAN_WriteData(mb)){
        usleep(20*1000);
        mb.channel = CAN_CHANNEL;
        mb.id = ID_RCV_GENERAL;
        canHandler->RBCAN_ReadData(&mb);
        if(mb.status != RBCAN_NODATA && mb.data[0]==81){
            int Pump_SpeedRef = (int)(mb.data[1]);
            std::cout << "=================================== " << std::endl;
            std::cout << "Pump Speed Reference : " << Pump_SpeedRef << std::endl;
            std::cout << "=================================== " << std::endl;
            mb.status = RBCAN_NODATA;
        }else{
            std::cout << ">>> ASK 81 - PumpBoard(" << BOARD_ID << ") is not connected. \n";
        }
    }
}

int HydraulicPumpController::ASK_CONTROL_MODE_ONOFF(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 82;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;

    if(canHandler->RBCAN_WriteData(mb)){
        usleep(20*1000);
        mb.channel = CAN_CHANNEL;
        mb.id = ID_RCV_GENERAL;
        canHandler->RBCAN_ReadData(&mb);
        if(mb.status != RBCAN_NODATA && mb.data[0]==82){
            int OnOff = (int)(mb.data[1]);
            std::cout << "=================================== " << std::endl;
            if (OnOff) std::cout << "Control Mode : On" << std::endl;
            else  std::cout << "Control Mode : Off" << std::endl;
            std::cout << "=================================== " << std::endl;
            mb.status = RBCAN_NODATA;
        }else{
            std::cout << ">>> ASK 82 - PumpBoard(" << BOARD_ID << ") is not connected. \n";
        }
    }
}


int HydraulicPumpController::ASK_PUMPDATA_REQUEST_FLAG(void) {
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 90;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;

    if(canHandler->RBCAN_WriteData(mb)){
        usleep(20*1000);
        mb.channel = CAN_CHANNEL;
        mb.id = ID_RCV_GENERAL;
        canHandler->RBCAN_ReadData(&mb);
        if(mb.status != RBCAN_NODATA && mb.data[0]==90){
            int ONOFF = (int)(mb.data[1]);
            std::cout << "=================================== " << std::endl;
            if(ONOFF == 0) std::cout << "Pressure Data Request : OFF" << std::endl;
            else if (ONOFF == 1) std::cout << "Pressure Data Request : ON" << std::endl;
            std::cout << "=================================== " << std::endl;
            mb.status = RBCAN_NODATA;
        }else{
            std::cout << ">>> ASK 90 - PumpBoard(" << BOARD_ID << ") is not connected. \n";
        }
    }
}


int HydraulicPumpController::CMD_SPEED_REF(int SPEED){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 181;
    mb.data[1] = (unsigned char)(SPEED & 0x000000FF);
    mb.data[2] = (unsigned char)((SPEED>>8) & 0x000000FF);
    mb.dlc = 3;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}

int HydraulicPumpController::CMD_CONTROL_MODE_ON(void){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 182;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}

int HydraulicPumpController::CMD_CONTROL_MODE_OFF(void){
    CMD_SPEED_REF(0);
    ONOFF_SendRefSpeed = false;
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 183;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}

int HydraulicPumpController::CMD_PRESSURE_NULL(){
    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 188;
    mb.dlc = 1;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}

int HydraulicPumpController::CMD_PUMPDATA_REQUEST_FLAG(int ONOFF){
    REQUEST_ONOFF_DATA = ONOFF;

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.data[0] = 190;
    mb.data[1] = ONOFF;
    mb.dlc = 2;
    mb.id = ID_SEND_GENERAL;
    return canHandler->RBCAN_WriteData(mb);
}


/****************************************************************
 *  PumpController - CAN DATA RX
 ****************************************************************/


int HydraulicPumpController::Read_PumpData(void){
    uint16_t temp_velocity_rpm, temp_temperature;
    int16_t temp_pressure;

    RBCAN_MB mb;
    mb.channel = CAN_CHANNEL;
    mb.id = ID_RCV_VELOCITY;
    canHandler->RBCAN_ReadData(&mb);
    if(mb.status != RBCAN_NODATA){
        temp_velocity_rpm = (uint16_t)((mb.data[0])|(mb.data[1]<<8));
        temp_pressure     = (int16_t)((mb.data[2])|(mb.data[3]<<8)); // Raw value (20210405)
        temp_temperature  = (uint16_t)((mb.data[4])|(mb.data[5]<<8));

        CurrentVelocity     = (double)temp_velocity_rpm/10.0;
        CurrentTemperature  = (double)temp_temperature/10.0;

        double in = (double)temp_pressure;
//        CurrentPressure = -0.0000034*in*in + 0.07*in + 14.0; // Seunghoon's pump board
//        CurrentPressure = 0.07*in + 17.0; // Seunghoon's pump board (LIGHT2)
        CurrentPressure = 0.07336*in - 5.0; // Seunghoon's 2nd ver. pump board (LIGHT2)
//        CurrentPressure = -0.0000034*in*in + 0.07*in - 9.1; // LIGHT's pump board

        mb.status = RBCAN_NODATA;
        return true;
    }
    return false;
}


/****************************************************************
 *  PumpController - CAN REFERENCE TX
 ****************************************************************/

void HydraulicPumpController::ActivePumpControl_ONOFF(int ONOFF){
    if(ONOFF==1){
        ONOFF_SendRefSpeed = true;
        CMD_CONTROL_MODE_ON();
    }else if(ONOFF==0){
        CMD_SPEED_REF(0);
        ONOFF_SendRefSpeed = false;
    }
}

int HydraulicPumpController::Send_ReferenceSpeed(void){
    if(ONOFF_SendRefSpeed == true)  {
        int16_t ref_n = (int16_t)ReferencePumpVelocity;
        int16_t ref_l = (int16_t)ReferencePumpVelocity_last;

        if(ref_n!=ref_l)
        {
            ReferencePumpVelocity_last = ReferencePumpVelocity;

            RBCAN_MB mb;
            mb.channel = CAN_CHANNEL;
            mb.data[0] = (unsigned char)(ref_n & 0x000000FF);
            mb.data[1] = (unsigned char)((ref_n>>8) & 0x000000FF);
            mb.dlc = 2;
            mb.id = ID_SEND_VELOCITY;
            return canHandler->RBCAN_WriteData(mb);
        }
        return true;
    } else {
        return false;
    }
}
