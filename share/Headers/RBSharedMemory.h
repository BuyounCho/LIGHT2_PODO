#ifndef RB_SHARED_MEMORY_H
#define RB_SHARED_MEMORY_H

#define RBCORE_SHM_NAME_REFERENCE       "RBCORE_SHARED_MEMORY_REFERENCE"
#define RBCORE_SHM_NAME_SENSOR          "RBCORE_SHARED_MEMORY_SENSOR"
#define RBCORE_SHM_NAME_COMMAND         "RBCORE_SHARED_MEMORY_COMMAND"

#define MAX_JOINT   1
#define MAX_MC      13
#define MAX_FT      2
#define MAX_IMU     1
#define MAX_SP      1
#define MAX_OF      2
#define MAX_AL      10
#define MAX_PC      1

#define MAX_PREVIEW             25

#define MAX_HCB_CAN_CHANNEL         7

#define MAX_MANUAL_CAN		30
#define MAX_COMMAND_DATA        30

#define MOTOR_1CH               1
#define MOTOR_2CH               2

#define COMMAND_CANID           0x01
#define SENSOR_REQUEST_CANID    0x03    //0x02
#define RT_TIMER_PERIOD_MS      4       // 1.0ms = 1000us >> 1000Hz
#define RT_TIMER_FREQ           250.0

#define RBCORE_PODO_NO          0
#define RBCORE_PODO_NO          0

#define ENABLE			1
#define DISABLE			0

#define RBCORE_PI               3.141592f

#include <stdint.h>

typedef enum{
    MANUALCAN_NEW = 0,
    MANUALCAN_EMPTY,
    MANUALCAN_WRITING
} MANUALCAN_STATUS;

typedef	struct _MANUAL_CAN_{
    unsigned char	channel;
    unsigned int        id;
    unsigned char	data[8];
    unsigned char	dlc;
    MANUALCAN_STATUS    status;
} MANUAL_CAN;

typedef union{
    struct{
        unsigned    HIP:1;	 	// Open loop(PWM) Control mode
        unsigned    RUN:1;		// Control ON
        unsigned    MOD:1;		// Control mode
        unsigned    LIM:1;		// Limit sensor
        unsigned    HME:4;		// Limit Sensor mode during Homing

        unsigned    JAM:1;		// JAM error
        unsigned    PWM:1;		// PWM saturation
        unsigned    BIG:1;		// Big Position error
        unsigned    INP:1;              // Big Input error
        unsigned    FLT:1;		// FET Driver Fault 1= Fault
        unsigned    ENC:1;              // encoder fail
        unsigned    CUR:1;		// big current difference <<== used for CAN connection check
        unsigned    TMP:1;		// Temperature warning

        unsigned    PS1:1;		// Position Limit 1
        unsigned    PS2:1;		// Position Limit 2
        unsigned    SWM:1;		// switching mode (1:complementary, 0:non-complementary)
        unsigned    GOV:1;		// gain override
        unsigned    FRC:1;		// friction compensation
        unsigned    REF:1;		// reference mode (1:incremental, 0:absolute)
        unsigned    CAN:1;		// CAN has been recovered

        unsigned    RPL:1;              // reply ok
    }b;
    unsigned char B[3];
}mSTAT;

struct COCOA_DATA
{
    double          P_KP, P_KI, P_KD;
    double          C_KP, C_KI, C_KD;
    double          FOC_P_KP, FOC_P_KI, FOC_P_KD;
    double          FOC_C_KP, FOC_C_KI, FOC_C_KD;
    double          PWM_RATE_LIMIT;
    double          CURRENT_LIMIT;
    uint16_t        FINDHOME_SEARCH_VEL;
    uint16_t        FINDHOME_OFFSET;
    bool            FINDHOME_DIRECTION;//true=>positive
    bool            FINDHOME_OFFSET_DIRECTION;//true=>positive
    bool            MOTOR_DIRECTION;//true=>positive
    bool            FET_ONOFF;
    int             BOARD_ACT;
    int             TORQUE_SENSOR1;
    int             TORQUE_SENSOR2;

    // Newly added by HSW
    double          EMF_KP;
    uint16_t        VEL_LIMIT;
    uint16_t        ACC_LIMIT;
    uint16_t        GAIN_OVER_VALUE;
    int             FINDHOME_LIM;
    uint16_t        ERROR_LIM;
    bool            BIGERROR_ONOFF;
    bool            CANERROR_ONOFF;
    bool            ENCERROR_ONOFF;
    bool            HOMEERROR_ONOFF;
    bool            PLIMITERROR_ONOFF;
    bool            LOGICERROR_ONOFF;
    bool            INPUTERROR_ONOFF;

    int             HOME_STATE;
    double          BEMF;

};

struct HCB_REF
{
    // Revised for Hydraulic Actuator (ActuatorLevel)
    double          ReferencePosition;      // LinearAct : mm, RotaryAct : deg
    double          ReferenceVelocity;      // LinearAct : mm/s, RotaryAct : deg/s
    double          ReferenceForceTorque;   // LinearAct : N, RotaryAct : Nm
    double          ReferenceLoadPressure;  // LinearAct : bar, RotaryAct : bar
    double          ReferenceValvePos;      // DDV : pulse, two-stage : uA
    double          ReferencePumpPressure;  // Pump Supply Pressure

    double          ReferencePosition_last;     // LinearAct : mm, RotaryAct : deg
    double          ReferenceVelocity_last;     // LinearAct : mm/s, RotaryAct : deg/s
    double          ReferenceForceTorque_last;  // LinearAct : N, RotaryAct : Nm
    double          ReferenceLoadPressure_last; // LinearAct : bar, RotaryAct : bar
    int             ReferencePWM_last;          // 1~1000
    double          ReferenceValvePos_last;     // DDV : pulse, two-stage : uA
    double          ReferencePumpPressure_last; // Pump Supply Pressure

    HCB_REF();
};

inline HCB_REF::HCB_REF(){
    ReferencePosition_last = 0;
    ReferenceVelocity_last = 0;
    ReferenceForceTorque_last = 0;
    ReferenceLoadPressure_last = 0;
    ReferencePWM_last = 0;
    ReferenceValvePos_last = 0;
    ReferencePumpPressure_last = 0;
}

struct HCB_DATA
{
    // Revised for Hydraulic Actuator (ActuatorLevel)
    double          CurrentPosition; // LinearAct : mm, RotaryAct : deg
    double          CurrentVelocity; // LinearAct : mm/s, RotaryAct : deg/s
    double          CurrentForce; // LinearAct : N, RotaryAct : Nm
    double          CurrentValvePos; // DDV : pulse, two-stage : N/A
    double          CurrentPressureA; // LinearAct : bar, RotaryAct : bar
    double          CurrentPressureB; // LinearAct : bar, RotaryAct : bar
    int16_t         CurrentTempData1; // -32756~32756
    int16_t         CurrentTempData2; // -32756~32756
};

struct HCB_INFO
{
    int             CAN_FREQ;
    bool            FET_ONOFF;
    bool            REQUEST_ONOFF_PosVel;
    bool            REQUEST_ONOFF_ValvePos;
    bool            REQUEST_ONOFF_OtherInfo;

    bool            BIGERROR_ONOFF;
    bool            ENCERROR_ONOFF;
    bool            CANERROR_ONOFF;
    bool            HOMEERROR_ONOFF;
    bool            PLIMITERROR_ONOFF;
    bool            LOGICERROR_ONOFF;
    bool            INPUTERROR_ONOFF;
    int             CONTROL_MODE;
    int             OPERATION_MODE;  // (00 : Moog & Rot, 01 : Moog & Lin, 10 : KNR & Rot, 11 : KNR & Lin)
    int             SENSING_MODE;    // (0 : torque, 1: pressure)
    int             CURRENT_CONTROL_MODE;   // (0 : pwm, 1 : current control)
    int             JOINTENC_DIRECTION;     // +1:Positive, -1:Negative
    int             VALVEINPUT_DIRECTION;   // +1:Positive, -1:Negative
    int             VALVEENC_DIRECTION;     // +1:Positive, -1:Negative
    double          BOARD_IN_VOLTAGE;
    double          BOARD_OPER_VOLTAGE;
    int             VARIABLE_SUPPLYPRES_ONOFF;    // (0 : OFF, 1: ON)

    int             VALVE_CENTER_POS;
    int             VALVE_DZ_PLUS, VALVE_DZ_MINUS;
    int             VEL_COMPENSATION_K;
    int             ACTUATOR_COMPLIANCE_K;
    int             VALVE_FEEDFORWARD;
    int             BULK_MODULUS;
    int             VOL_A, VOL_B;
    int             PIS_AREA_A, PIS_AREA_B;
    int             SUP_PRES, RET_PRES;
    int             JOINTENC_LIMIT_MINUS, JOINTENC_LIMIT_PLUS;
    int             PIS_STROKE;
    int             VALVEPOS_LIMIT_MINUS, VALVEPOS_LIMIT_PLUS;
    double          JOINTENC_PPP;
    double          FORCESEN_PPF;
    double          PRESSEN_PPP_A;
    double          PRESSEN_PPP_B;
    int             CONST_FRIC;
    int             HOMEPOS_OFFSET;
    int             HOMEPOS_VALVE_OPENING;
    double          VALVE_GAIN_PLUS[5];
    double          VALVE_GAIN_MINUS[5];

    int             VALVE_P_KP, VALVE_P_KI, VALVE_P_KD;
    int             JOINT_P_KP, JOINT_P_KI, JOINT_P_KD;
    int             JOINT_F_KP, JOINT_F_KI, JOINT_F_KD;
    double          JOINT_SPRING, JOINT_DAMPER;

    HCB_INFO(){
        PRESSEN_PPP_A = 15.00;
        PRESSEN_PPP_B = 15.00;
        FORCESEN_PPF = 0.2773;
    }
};

typedef struct _ENCODER_SENSOR_
{
    int     BoardConnection;

    // Revised for Hydraulic Actuator (JointLevel)
    double      CurrentRefAngle; // Angle reference
    double      CurrentRefAngVel; // Angular velocity reference
    double      CurrentRefActPos; // Cylinder linear position reference
    double      CurrentRefActVel; // Cylinder linear velocity reference
    double      CurrentRefTorque; // Angle reference
    double      CurrentRefActForce; // Cylinder linear force reference
    double      CurrentRefValvePos; // Valve pos reference

    double      CurrentAngle; // deg
    double      CurrentAngVel; // deg/s
    double      CurrentActPos; // mm or deg
    double      CurrentActVel; // mm/s or deg/s
    double      CurrentTorque; // Nm
    double      CurrentPressureA; // bar
    double      CurrentPressureB; // bar
    double      CurrentValvePos; // DDV : pulse, two-stage : N/A
    double      CurrentActForce; // N or Nm
    double      CurrentTempData1;
    double      CurrentTempData2;
    
    int         NO_RESPONSE_CNT;

    HCB_INFO    HCB_Info;

}ENCODER_SENSOR;

typedef struct _PUMP_SENSOR_
{
    int         BoardConnection;
    double      CurrentSettingVoltage; // Setting Voltage [V]

    // Revised for Hydraulic Actuator (JointLevel)
    double      CurrentRefPressure; // Pressure reference [bar]
    double      CurrentRefVelocity; // Velocity reference [rpm]

    double      CurrentPressure;    // Sensing pressure [bar]
    double      CurrentVelocity;    // Sensing velocity [rpm]
    double      CurrentTemperature; // Sensing stator temperature [deg]

    double      CurrentMechaPower;   // Mechanical Power [W]
    double      CurrentHydraPower;   // Hydraulic Power [W]

    int         NO_RESPONSE_CNT;
}PUMP_SENSOR;

typedef struct _FT_SENSOR_
{
    int     BoardConnection;
    float   Mx;
    float   My;
    float   Mz;
    float   Fx;
    float   Fy;
    float   Fz;
    float   RollVel;
    float   PitchVel;
    float   Roll;
    float   Pitch;
}FT_SENSOR;

typedef struct _IMU_SENSOR_
{
    int     BoardConnection;
    float   Roll;  // (Global >> IMU) X''-axis rotation
    float   Pitch;
    float   Yaw;
    float   Wx_B;  // IMU(Body)'s angular velocity w.r.t the body frame
    float   Wy_B;
    float   Wz_B;
    float   Ax_B;  // IMU's linear acceleration w.r.t the body frame
    float   Ay_B;
    float   Az_B;
    float   Wx_G;  // IMU's angular velocity w.r.t the global frame
    float   Wy_G;
    float   Wz_G;
    float   Ax_G;  // IMU's linear acceleration w.r.t the global frame
    float   Ay_G;
    float   Az_G;
    double  Q[4]; // Q[0]:w , Q[1~3]:x,y,z
    int     NO_RESPONSE_CNT;
}IMU_SENSOR;

typedef struct _OF_SENSOR_
{
    int     BoardConnection;
    int     AccumX[2];
    int     AccumY[2];
    int     DeltaX[2];
    int     DeltaY[2];
    float   Xmea;
    float   Ymea;
    float   thmea;
    float   Pitmea;
}OF_SENSOR;

typedef struct _SMART_POWER_
{
    int     BoardConnection;

    float   Voltage;
    float   Current;
}SMART_POWER;

typedef struct _FOG_SENSOR_
{
    float   Roll;
    float   Pitch;
    float   Yaw;
    float   RollVel;
    float   PitchVel;
    float   YawVel;
    int     Warning;
    double  quaternion[4];
    double  LocalW[3];
}FOG_SENSOR;

typedef struct _COMMAND_STRUCT_
{
    int     USER_COMMAND;
    char    JOYSTICK_COMMAND_ONOFF;
    char    USER_PARA_CHAR[MAX_COMMAND_DATA];
    int	    USER_PARA_INT[MAX_COMMAND_DATA];
    float   USER_PARA_FLOAT[MAX_COMMAND_DATA];
    double  USER_PARA_DOUBLE[MAX_COMMAND_DATA];

    HCB_INFO    HCB_Info;

} COMMAND_STRUCT, *pCOMMAND_STRUCT;

typedef struct _USER_COMMAND_
{
    int             COMMAND_TARGET;
    COMMAND_STRUCT  COMMAND_DATA;
} USER_COMMAND, *pUSER_COMMAND;

//======================================
// Sensor Shared Memory <Read Only>
typedef struct _RBCORE_SHM_SENSOR_
{
    char            PODO_AL_WORKING[MAX_AL];
    mSTAT           MCStatus[MAX_MC][MOTOR_2CH];
    float           MCTemperature[MAX_MC];
    float           MotorTemperature[MAX_MC][MOTOR_2CH];

    ENCODER_SENSOR  ENCODER[MAX_MC][MOTOR_2CH];
    PUMP_SENSOR     PUMP[MAX_PC];

    FT_SENSOR       FT[MAX_FT];
    IMU_SENSOR      IMU[MAX_IMU];
    FOG_SENSOR      FOG;
    OF_SENSOR       OF;
    SMART_POWER     SP[MAX_SP];

    int             CAN_Enabled;
    int             REF_PosVel_Enabled;
    int             REF_Force_Enabled;
    int             REF_LoadPres_Enabled;
    int             REF_PWM_Enabled;
    int             REF_ValvePos_Enabled;

    int             SEN_Enabled;
    int             ENC_Enabled;

    int             Sim_Time_sec;
    int             Sim_Time_nsec;

    int             TORQUE_SENSOR1;
    int             TORQUE_SENSOR2;

    int             CONTACT_SENSOR_POS[4];
    int             CONTACT_SENSOR_VEL[4];

}RBCORE_SHM_SENSOR, *pRBCORE_SHM_SENSOR;
//======================================

//======================================
// Reference Shared Memory <Write Only>
typedef struct _RBCORE_SHM_REFERENCE_
{
    // This Variables are for choreonoid simulation. (20190508, Buyoun) ////
    double          Simulation_AngleSensor[MAX_MC];  // sensor value from choreonoid
    double          Simulation_VelocitySensor[MAX_MC];  // sensor value from choreonoid
    double          Simulation_TorqueSensor[MAX_MC]; // sensor value from choreonoid
    FT_SENSOR       Simulation_FT[MAX_FT];
    IMU_SENSOR      Simulation_IMU[MAX_IMU];
    int             Simulation_HandlingMode;
    bool            Simulation_DataEnable;
    // /////////////////////////////////////////////////////////////////////

    double          AngleReference[MAX_AL][MAX_MC][MOTOR_2CH];     // Joint Angle, not Cylinder Position! (by BUYOUN)
    double          AngVelReference[MAX_AL][MAX_MC][MOTOR_2CH];    // Joint Angular Vel, not Cylinder Velocity! (by BUYOUN)
    double          TorqueReference[MAX_AL][MAX_MC][MOTOR_2CH];    // Joint Torque, not Cylinder Force! (by BUYOUN)
    double          ActPosReference[MAX_AL][MAX_MC][MOTOR_2CH];
    double          ActVelReference[MAX_AL][MAX_MC][MOTOR_2CH];
    double          ActForceReference[MAX_AL][MAX_MC][MOTOR_2CH];
    double          JointStiffness[MAX_MC];
    double          JointDamping[MAX_MC];
    double          ActuatorStiffness[MAX_MC];
    double          ActuatorDamping[MAX_MC];
    bool            StiffnDampChange[MAX_MC];

    double          ValvePosReference[MAX_AL][MAX_MC][MOTOR_2CH];

    int             ValveCtrlMode_Command[MAX_MC]; // 0 : Nothing, 1 : Position or Force, 2 : Valve Open (Openloop)
    int             ValveCtrlMode[MAX_MC];
    bool            PosOrFor_Selection[MAX_MC];  // 0(false):PositionControl, 1(true):TorqueControl
    bool            PosOrFor_Selection_last[MAX_MC];

    double          PumpVelocityReference[MAX_PC];
    double          PumpPressureReference[MAX_PC];
    bool            PumpSupplyPressureChange;

    // Future Reference for Model Predictive Control (Pump Control)
    int             N_PrevPump = 20;
    double          dT_PrevPump = 0.100;
    bool            Flag_PumpControlMPC;
    double          RequiredFlowrateReference_Future[MAX_PREVIEW+1]; // 0:Current Ref, 1~MAX_PREVIEW:Future Ref
    double          RequiredPressureReference_Future[MAX_PREVIEW+1]; // 0:Current Ref, 1~MAX_PREVIEW:Future Ref
    double          LoadPressureReference_Future[MAX_PREVIEW+1][MAX_MC]; // 0:Current Ref, 1~MAX_PREVIEW:Future Ref
    double          ActFlowrateReference_Future[MAX_PREVIEW+1][MAX_MC]; // 0:Current Ref, 1~MAX_PREVIEW:Future Ref

    MANUAL_CAN      ManualCAN[MAX_MANUAL_CAN];

    int             NO_RESPONSE_CNT[MAX_AL];

}RBCORE_SHM_REFERENCE, *pRBCORE_SHM_REFERENCE;
//======================================

//======================================
// Reference Shared Memory <Read Write>
typedef struct _RBCORE_SHM_COMMAND_
{
    int             SYNC_SIGNAL[MAX_AL];
    int             ACK_SIGNAL[MAX_MC][MOTOR_2CH];

    int             MotionOwner[MAX_MC][MOTOR_2CH];
    COMMAND_STRUCT  COMMAND[MAX_AL];
    int             CommandAccept[MAX_AL];
    long            ErrorInform2GUI;

    MANUAL_CAN      ManualCAN[MAX_MANUAL_CAN];
} RBCORE_SHM_COMMAND, *pRBCORE_SHM_COMMAND;
//======================================

typedef enum _DAEMON4LIGHT_COMMAND_SET_{
    DAEMON4LIGHT_NO_ACT = 0,
    // For process handle
    DAEMON4LIGHT_PROCESS_CREATE,
    DAEMON4LIGHT_PROCESS_KILL,

    // For initialize
    DAEMON4LIGHT_INIT_CHECK_DEVICE,
    DAEMON4LIGHT_CAN_CHANNEL_ARRANGE,
    DAEMON4LIGHT_INIT_CAN_RESET,

    // For LIGHT utility
    DAEMON4LIGHT_SAVEDATA,

    // For HCB(motion controller)
    DAEMON4LIGHT_MOTION_ENCODER_ZERO,
    DAEMON4LIGHT_MOTION_FET_ONOFF,
    DAEMON4LIGHT_MOTION_REF_ONOFF,
    DAEMON4LIGHT_MOTION_ERROR_CLEAR,
    DAEMON4LIGHT_MOTION_BOARDTEST,
    DAEMON4LIGHT_MOTION_BOARDTEST_ERRORRESET,
    DAEMON4LIGHT_MOTION_TORQUEFORCE_NULLING,
    DAEMON4LIGHT_MOTION_PRES_NULLING,
    DAEMON4LIGHT_MOTION_REF_RESET,
    DAEMON4LIGHT_ENC_ZERO,
    DAEMON4LIGHT_FINDHOME,

    // For MC
    DAEMON4LIGHT_MOTION_REQUEST_ENABLE_DISABLE,
    DAEMON4LIGHT_MOTION_ASK_EVERYTHING,
    DAEMON4LIGHT_MOTION_SET_EVERYTHING,
    DAEMON4LIGHT_MOTION_SET_BNO,
    DAEMON4LIGHT_MOTION_CHANGE_POSorTOR,
    DAEMON4LIGHT_MOTION_CHANGE_SUPPLYPRES_MODE,

    // For Pump Control
    DAEMON4LIGHT_PUMP_ASK_STATUS,
    DAEMON4LIGHT_PUMP_SEND_COMMAND,
    DAEMON4LIGHT_PUMP_MANUAL_REFSPEED_SET,
    DAEMON4LIGHT_PUMP_ACTIVE_VELOCITY_CONTROL_ONOFF,

    // For FT Sensor
    DAEMON4LIGHT_SENSOR_FT_ONOFF,
    DAEMON4LIGHT_SENSOR_FT_NULL,

    // For IMU
    DAEMON4LIGHT_SENSOR_IMU_ONOFF,
    DAEMON4LIGHT_SENSOR_IMU_NULL,
    DAEMON4LIGHT_SENSOR_IMU_FIND_OFFSET,

    // For Simulation Setting
    DAEMON4LIGHT_SIMLATION_MODE_ONOFF,

} COMMAND_SET;

typedef enum LIGHT_WALKING_COMMAND_SET {
    LIGHT_NO_ACT = 100,
    LIGHT_ALL_STOP,
    LIGHT_DATA_SAVE,
    LIGHT_DATA_SAVE_SUPPRES,
    LIGHT_DATA_SAVE_SYS_ID,

    LIGHT_PARAMETER_SETTING,
    LIGHT_SUPPLYPRESSURE_SETTING,

    // Joint space control
    LIGHT_GOTO_HOMEPOSE,

    // Task space control - Base is floating
    LIGHT_WORKSPACE_MOVE,
    LIGHT_AIRWALKING,
    LIGHT_TESTJUMPING,

    // Task space control - Contact to ground
    LIGHT_COM_MOVING_DSP,
    LIGHT_COM_MOVING_SSP,
    LIGHT_SUPPORT_TRANSITION,
    LIGHT_SYSID_COM,

    LIGHT_SQUAT,

    LIGHT_COM_MOVING_RDSP_SMOOTH,
    LIGHT_COM_MOVING_LDSP_SMOOTH,

    LIGHT_RFSWINGUP_DYNAMIC,
    LIGHT_RFSWINGDOWN_DYNAMIC,
    LIGHT_LFSWINGUP_DYNAMIC,
    LIGHT_LFSWINGDOWN_DYNAMIC,

    LIGHT_WALK,
    LIGHT_WALK_withJOYSTICK,

    LIGHT_FULLTASK

}LIGHT_WALKING_COMMAND;

typedef enum PUMP_CONTROL_COMMAND_SET{
    PUMP_CONTROL_NO_ACT = 100,
    PUMP_CONTROL_SAVE_DATA,

    PUMP_CONTROL_PRESREF_SELECTION,

    PUMP_ACTIVE_CONTROL_ONOFF,
    PUMP_ACTIVE_CONTROL_MPC_ONOFF,

    PUMP_ACTIVE_CONTROL_LINEAR,

}PUMP_CONTROL_COMMAND;

typedef enum SH_TASK_COMMAND_SET{
    SH_TASK_NO_ACT = 100,
    SH_TASK_SAVE_DATA,

    SH_TASK_VALVE_ID_POS,
    SH_TASK_VALVE_ID_NEG,

    // For process handle
    SH_TASK_CONST_OPENING,
    SH_TASK_SINEWAVE_OPENING,

    SH_TASK_POSITION_CONTROL_GOTO_POSE,
    SH_TASK_POSITION_CONTROL_RANDOM_MOTION,
    SH_TASK_POSITION_CONTROL_SINEWAVE,

    SH_TASK_TORQUE_CONTROL_GO,
    SH_TASK_TORQUE_CONTROL_STOP,

}SH_TASK_COMMAND;


typedef enum QUAD_TASK_COMMAND_SET{
    QUAD_TASK_NO_ACT = 100,
    QUAD_TASK_SAVE_DATA,

    // For process handle
    QUAD_TASK_ARMMOTION,

    QUAD_TASK_HOMEPOSE,
    QUAD_TASK_TESTJOINTS,
    QUAD_TASK_SQUAT,
    QUAD_TASK_SWING,

}QUAD_TASK_COMMAND;



enum RobotControlMode
{
    RobotControlMode_NoSignal = 0,
    RobotControlMode_Transition_PostionControl,
    RobotControlMode_Transition_TorqueControl,
};

enum ValveControlMode
{
    ValveControlMode_Noact = -1,
    ValveControlMode_Null = 0,
    ValveControlMode_Opening,
    ValveControlMode_PosOrFor,
    ValveControlMode_FindHome
};

enum JointControlMode
{
    JointControlMode_Position = 0,
    JointControlMode_Torque
};

#endif // RB_SHARED_MEMORY_H

