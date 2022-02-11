#ifndef HydraulicActuatorController_H
#define HydraulicActuatorController_H

#include "RBDataType.h"
#include "RBCAN.h"
#include <cmath>

#define HCB_CTRL_MODE_NOACT                 0   // NO control mode
#define HCB_CTRL_MODE_VALVE_OPENLOOP        1   // valve open loop control mode
#define HCB_CTRL_MODE_VALVE_POS             2   // valve position control mode
#define HCB_CTRL_MODE_POS_FORCE_PWM         3   // joint position and force control based on PWM
#define HCB_CTRL_MODE_POS_FORCE_VALVE       4   // joint position and force control based on valve position
#define HCB_CTRL_MODE_POS_FORCE_LEARN       5   // joint position and force control based on learing
#define HCB_CTRL_MODE_POS_PRES_PWM          6   // joint position and load pressure control based on PWM
#define HCB_CTRL_MODE_POS_PRES_VALVE        7   // joint position and load pressure control based on valve position
#define HCB_CTRL_MODE_POS_PRES_LEARN        8   // joint position and load pressure control based on learning
#define HCB_CTRL_MODE_PWM_SINE_OPENLOOL     9   // valve sine open loop mode

#define HCB_CTRL_MODE_FORCE_NULL            20  // force sensor nulling
#define HCB_CTRL_MODE_VALVE_NULL            21  // valve nulling and dead zone setting
#define HCB_CTRL_MODE_FINDHOME              22  // find home
#define HCB_CTRL_MODE_FLOWGAIN_TUNE         23  // flow rate gain tuning

#define HCB_OPER_MODE_SW                    0 // Sungwoo operation mode
#define HCB_OPER_MODE_SH                    1 // Seunghoon operation mode

#define HCB_REF_MODE_NOACT                  0
#define HCB_REF_COS                         1
#define HCB_REF_LINE                        2
#define HCB_REF_SIN_WAVE                    3


typedef struct _MOVE_JOINT_
{
    double			RefAngleCurrent;	// reference to move at this step
    double			RefAngleDelta;		// reference of the past step
    double			RefAngleToGo;		// goal position - initial position
    double			RefAngleInitial;	// initial position
    unsigned long	GoalTimeCount;		// the time at which the goal is reached
    unsigned long	CurrentTimeCount;	// current time count
    char			MoveFlag;			// move flag
} MOVE_JOINT, *pMOVE_JOINT;


class HydraulicActuatorController
{
public:
    HydraulicActuatorController();

    // from DB <-
    QString BOARD_NAME;
    int     BOARD_ID;
    int     CAN_CHANNEL;

    QString ACTUATOR_TYPE;
    double  PULSE_PER_POSITION;
    double  PULSE_PER_FORCETORQUE;
    double  PULSE_PER_PRESSURE;

    int     ID_SEND_GENERAL;
    int     ID_SEND_POSVEL;
    int     ID_SEND_VALVEPOS;

    int     ID_RCV_GENERAL;
    int     ID_RCV_POSVEL;
    int     ID_RCV_VALVEPOS;
    int     ID_RCV_OTHERINFO;

    // not from DB----
    int     ConnectionStatus;
    float   BoardTemperature;

    RB_JOINT    Joints[MAX_JOINT];
    MOVE_JOINT  MoveJoints[MAX_JOINT];

    void HCB_GetDBData(DB_MC db);
    void HCB_AddCANMailBox_RCVDATA();

    // Board Operation Setting ///////////////////////////////////////////////
    int HCB_CANCheck(void);
    int HCB_CANChannel_Arrange(void);
    int HCB_InformationCheck(void);

    void HCB_ASK_BoardNumber(void);
    int HCB_CMD_BoardNumber(int BNO);
    int HCB_CMD_EncZero(void);
    int HCB_CMD_HomePos(int settingswitch_all);
    bool HCB_Read_FindHomeDone(void);
    void HCB_ASK_BoardOperationMode(void);
    int HCB_CMD_BoardOperationMode(int OperMode, int SenseMode, int CurCtrlMode);
    int HCB_CMD_EncoderZero(void);
    int HCB_CMD_FETOnOff(bool OnOff);
    int HCB_CMD_ControlMethodTransition(bool On_TorqueControl);
    void HCB_ASK_CANFrequency(void);
    int HCB_CMD_CANFrequency(int16_t CAN_FREQ_HZ);
    void HCB_ASK_ControlMode(void);
    int HCB_CMD_ControlMode(int mode);
    int HCB_CMD_Request_PosVel(bool OnOff);
    int HCB_CMD_Request_ValvePos(bool OnOff);
    int HCB_CMD_Request_OtherInfo(bool OnOff);
    void HCB_ASK_JointEncDir(void);
    int HCB_CMD_JointEncDir(int Dir);
    void HCB_ASK_ValveInputDir(void);
    int HCB_CMD_ValveInputDir(int Dir);
    void HCB_ASK_ValveEncDir(void);
    int HCB_CMD_ValveEncDir(int Dir);
    void HCB_ASK_BoardInputVoltage(void);
    int HCB_CMD_BoardInputVoltage(double InV);
    void HCB_ASK_ValveOperationVoltage(void);
    int HCB_CMD_ValveOperationVoltage(double OperV);
    void HCB_ASK_VariableSupplyPressure(void);
    int HCB_CMD_VariableSupplyPressure(int OnOff);

    // Control Parameter Setting /////////////////////////////////////////////
    void HCB_ASK_PIDGain(int mode);
    int HCB_CMD_PIDGain(int mode, int Pgain, int Igain, int Dgain);
    void HCB_ASK_JointSpringDamper(void);
    int HCB_CMD_JointSpringDamper(double Spring, double Damper);
    void HCB_ASK_ValveDeadzoneNCenterPos(void);
    int HCB_CMD_ValveDeadzoneNCenterPos(int CenterPos, int PositiveDeadzone, int NegativeDeadZone);
    void HCB_ASK_VelCompensationGain(void);
    int HCB_CMD_VelCompensationGain(int CompensationPercent);
    void HCB_ASK_ComplianceGain(void);
    int HCB_CMD_ComplianceGain(int KGain);
    void HCB_ASK_ValveFeedforward(void);
    int HCB_CMD_ValveFeedforward(int ff);
    void HCB_ASK_BulkModulus(void);
    int HCB_CMD_BulkModulus(int B);
    void HCB_ASK_ChamberVolume(void);
    int HCB_CMD_ChamberVolume(int Vol_A, int Vol_B);
    void HCB_ASK_PistonArea(void);
    int HCB_CMD_PistonArea(int Area_A, int Area_B);
    void HCB_ASK_SupplyAndReturnPressure(void);
    int HCB_CMD_SupplyAndReturnPressure(int P_sup, int P_ret);
    void HCB_ASK_JointEncLimit(void);
    int HCB_CMD_JointEncLimit(int LoLimit, int UpLimit);
    void HCB_ASK_PistonStroke(void);
    int HCB_CMD_PistonStroke(int Stroke);
    void HCB_ASK_ValvePositionLimit(void);
    int HCB_CMD_ValvePositionLimit(int LoLimit, int UpLimit);
    void HCB_ASK_EncoderPulsePerPosition(void);
    int HCB_CMD_EncoderPulsePerPosition(double PPP);
    void HCB_ASK_SensorPulsePerForceTorque(void);
    int HCB_CMD_SensorPulsePerForceTorque(double PPF);
    void HCB_ASK_SensorPulsePerPressure(void);
    int HCB_CMD_SensorPulsePerPressure(double PPP_A, double PPP_B);
    void HCB_ASK_ConstantFriction(void);
    int HCB_CMD_ConstantFriction(double Fric);
    void HCB_ASK_HomeposOffset(void);
    int HCB_CMD_HomeposOffset(double homeposoffset);
    void HCB_ASK_HomeposValveOpening(void);
    int HCB_CMD_HomeposValveOpening(double homeposvalveopening);
    void HCB_ASK_VALVE_GAIN_PLUS(void);
    void HCB_ASK_VALVE_GAIN_MINUS(void);
    int HCB_CMD_ErrorClear(void);

    // Read Data&Info from HCB //////////////////////////////////////////////////////////////
    int HCB_Read_Information(void);
    int HCB_SEE_Information(void);
    int HCB_Read_EncoderData(void);
    int HCB_Read_ValvePosData(void);
    int HCB_Read_OtherInfoData(void);

    // Send Data&Info to HCB //////////////////////////////////////////////////////////////
    int HCB_ResetReference_PosVel(bool PsVar_OnOff);
    int HCB_SendReference_PosVel(bool PsVar_OnOff);
    int HCB_SendReference_ValvePos(void);



private:

};

class HydraulicPumpController
{
public:
    HydraulicPumpController();

    // from DB <-
    int     BOARD_ID;
    int     CAN_CHANNEL;
    bool    REQUEST_ONOFF_DATA;

    int     ID_SEND_GENERAL;
    int     ID_SEND_VELOCITY;

    int     ID_RCV_GENERAL;
    int     ID_RCV_VELOCITY;
    int     ID_RCV_PRESSURE;

    // not from DB----
    int     ConnectionStatus;
    double   CurrentVelocity;     // Pump Rotation Speed
    double   CurrentPressure;     // Supply Pressure
    double   CurrentTemperature;  // Oil(or Wire) Temperature

    void    GetDBData(DB_PC db);
    void    AddCANMailBox_RCVDATA();

    // Board Operation Setting ///////////////////////////////////////////////
    int CANCheck(void);
    int CANChannel_Arrange(void);
//    int HCB_InformationCheck(void);

    // Control Parameter Setting /////////////////////////////////////////////

    int ASK_SPEED_REF(void);
    int CMD_SPEED_REF(int VEL);

    int CMD_PRESSURE_NULL(void);
    int ASK_PUMPDATA_REQUEST_FLAG(void);
    int CMD_PUMPDATA_REQUEST_FLAG(int ONOFF);

    int ASK_CONTROL_MODE_ONOFF(void);
    int CMD_CONTROL_MODE_ON(void);
    int CMD_CONTROL_MODE_OFF(void);

    // Read Data&Info from HCB //////////////////////////////////////////////////////////////
    int Read_PumpData(void);

    // Send Data&Info to HCB //////////////////////////////////////////////////////////////

    bool    ONOFF_SendRefSpeed;
    double  ReferencePumpVelocity;
    double  ReferencePumpVelocity_last;

    void    ActivePumpControl_ONOFF(int ONOFF);
    int     Send_ReferenceSpeed(void);


private:

};

#endif // HydraulicActuatorController_H
