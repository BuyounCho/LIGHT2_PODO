#include "BasicFiles/BasicSetting.h"
#include "BasicFiles/BasicJoint.h"
#include "ManualCAN.h"

#include "LIGHT_commands.h"
#include "LIGHT_robotmodel.h"
#include "LIGHT_var_and_func.h"
#include "LIGHT_dynamics.h"
#include "LIGHT_kinematics.h"
#include "LIGHT_motion.h"
#include "LIGHT_savedata.h"

// for Rigid Body Dynamics Library
#include "rbdl/rbdl.h"

// for Quadratic Programming
#include "QP_BasicFiles/QuadProg++.hh"
#include "qpSWIFT_BasicFiles/include/Matrices.h"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
using namespace RigidBodyDynamics::Utils;

#define PODO_AL_NAME       "LIGHTWalking"

// Variables ===========================================
// Shared memory
pRBCORE_SHM_COMMAND     sharedCMD;
pRBCORE_SHM_REFERENCE   sharedREF;
pRBCORE_SHM_SENSOR      sharedSEN;
pUSER_SHM               userData;
JointControlClass       *joint;
JointControlClass       *jCon;


// Program variable
int isTerminated;
int     __IS_WORKING = false;
int     __AL_TERMINATE = false;
int     __IS_GAZEBO = false;
int     PODO_NO = -1;
int     _OPERATION_MODE = _OPERATION_NO;
int     _OPERATION_MODE_NEW = _OPERATION_NO;
bool    _OPERATION_MODE_CHANGED = false;

INFO_LIGHT LIGHT_Info;
LIGHTWholeBody LIGHT;
LIGHTWholeMotions LIGHT_WholeMotions;

LIGHT_InvKinematics_INFO INFO_InvKin;
LIGHT_InvKinematics_SUB_INFO INFO_InvKin_SUB;


extern void PrintHere(int n);
bool RefUpdateEnable = true;
bool Flag_ZMP_Control = false;

bool PelvisCompensation = false;

/********************************************
 * Operation Mode and Counter
 ********************************************/

double          _OPERATION_POS_SPRING   = 0.0;
double          _OPERATION_POS_DAMPING  = 0.0;
double          _OPERATION_ORI_SPRING   = 0.0;
double           _OPERATION_ORI_DAMPING = 0.0;

double          _OPERATION_POS_SPRING2 = 0.0;
double          _OPERATION_POS_DAMPING2 = 0.0;
double          _OPERATION_ORI_SPRING2 = 0.0;
double          _OPERATION_ORI_DAMPING2 = 0.0;

double          _OPERATION_POS_SPRING3 = 0.0;
double          _OPERATION_POS_DAMPING3 = 0.0;
double          _OPERATION_ORI_SPRING3 = 0.0;
double          _OPERATION_ORI_DAMPING3 = 0.0;

double          _OPERATION_X_OFFSET = 0.0;
double          _OPERATION_Y_OFFSET = 0.0;
double          _OPERATION_Z_OFFSET = 0.0;
double          _OPERATION_GRAVITY = 0.0;

double          _OPERATION_STEP_TIME = 0.0;
double          _OPERATION_STEP_TIME2 = 0.0;
double          _OPERATION_STEP_LENGTH = 0.0;
double          _OPERATION_STEP_LENGTH2 = 0.0;
double          _OPERATION_STEP_YAW = 0.0;
double          _OPERATION_STANCE_WIDTH = 0.0;
double          _OPERATION_STANCE_OFFSET = 0.0;
int             _OPERATION_STEP_NUMBER = 0.0;

int             _OPERATION_TYPE;
int             _OPERATION_STAGE;
int             _OPERATION_JOINT;
double          _OPERATION_MAG;
double          _OPERATION_FREQ;
double          _OPERATION_PERIOD;
int             _OPERATION_NUM;
double          _OPERATION_TIME;
bool            _OPERATION_ONOFF;
double          _OPERATION_FREQ_NOW;

Matrix3d        _OPERATION_ORI_PEL;
Matrix3d        _OPERATION_ORI_RF;
Matrix3d        _OPERATION_ORI_LF;
Vector3d        _OPERATION_POS_PEL2RF;
Vector3d        _OPERATION_POS_PEL2LF;
Vector3d        _OPERATION_POS_RF2CoM;
Vector3d        _OPERATION_POS_RF2LF;
Vector3d        _OPERATION_POS_LF2CoM;
Vector3d        _OPERATION_POS_LF2RF;
MatrixNd        _OPERATION_Q;


bool _ReadyToJoyStickWalk = false;


int main(int argc, char *argv[])
{

    // =================== Program initiated ===================
    // Termination signal ---------------------------------
    signal(SIGTERM, CatchSignals);   // "kill" from shell
    signal(SIGINT,  CatchSignals);    // Ctrl-c
    signal(SIGHUP,  CatchSignals);    // shell termination
    signal(SIGKILL, CatchSignals);
    signal(SIGSEGV, CatchSignals);

    // Block memory swapping ------------------------------
    mlockall(MCL_CURRENT|MCL_FUTURE);

    // Setting the AL Name <-- Must be a unique name!!
    sprintf(__AL_NAME, "LIGHTWalking");


    CheckArguments(argc, argv);
    if(PODO_NO == -1){
        FILE_LOG(logERROR) << "Please check the AL Number";
        FILE_LOG(logERROR) << "Terminate this AL..";
        return 0;
    }

    // Initialize RBCore -----------------------------------
    if(RBInitialize() == false)
        __IS_WORKING = false;

    jCon->SetMotionOwner(0);

    jCon->RefreshToCurrentReference();
    jCon->SetAllMotionOwner();

    // Reference Enable
    for(int i=0;i<MAX_MC;i++) {
        sharedREF->ValveCtrlMode_Command[i] = ValveControlMode_PosOrFor;
    }
    FILE_LOG(logSUCCESS) << "Setting Complete!!";

    // ========================================================
    // Free Note

//    Vector3d InitStance(0.0,0.20,0.0);
//    LIGHT_WholeMotions.StepDataBuffer_Initialize_Walking(InitStance, 0.50, 0.30, 0.20, 0.0, 0.0, 0.0, 0.30, 0.0, 20.0*D2R);

//    for(int i=0;i<30;i++) {
//        int StanceLeg;
//        Vector3d StancePosition;
//        double StanceZAngle;
//        bool StepOn;
//        Vector3d StepPotision;
//        double StepZAngle;

//        LIGHT_WholeMotions.GetFutureStance((double)i*0.1,
//                                              StanceLeg, StancePosition, StanceZAngle, StepOn, StepPotision, StepZAngle);
//        cout << "t : "<< (double)i*0.1 << endl;
//        cout << "StanceLeg : "<< StanceLeg << endl;
//        cout << "StancePosition : "<< StancePosition.transpose() << endl;
//        cout << "StanceZAngle : "<< StanceZAngle*R2D << endl;
//        cout << "StepOn : "<< StepOn << endl;
//        cout << "StepPotision : "<< StepPotision.transpose() << endl;
//        cout << "StepZAngle : "<< StepZAngle*R2D << endl;

//        MatrixNd _A;
//        VectorNd _B;
//        Get_SupportConvexHull(StanceLeg, StancePosition, StanceZAngle,
//                              StepOn, StepPotision, StepZAngle,
//                              _A, _B);

////        Vector3d FutureVel;
////        LIGHT_WholeMotions.GetFutureVelocityRef((double)i*0.1,FutureVel);
////        cout << "FutureVel : "<< FutureVel.transpose() << endl;3
//        cout << "==================================="<< endl;
//    }

//    Vector3d InitStance(0.0,0.20,0.0);
//    LIGHT_WholeMotions.Xref_CP = InitStance/2.0;
//    LIGHT_WholeMotions.Xref_CoM = InitStance/2.0;
//    LIGHT_WholeMotions.StepDataBuffer_Initialize_Walking(InitStance,
//                                                         0.60, 0.00,
//                                                         0.20, 0.00, 0.00, 0.00,
//                                                         0.30, 0.00, 0.0*D2R);

//    Vector3d* ZMPref_horizon = new Vector3d[12]; // Input
//    Vector3d* CPref_horizon = new Vector3d[12+1]; // State
//    Vector3d* CoMref_horizon = new Vector3d[12+1]; // Resultant state
//    LIGHT_WholeMotions.Generate_CPref_withMPC(12,0.15,CPref_horizon,ZMPref_horizon,CoMref_horizon);
////    LIGHT_WholeMotions.Calc_FuturePressureReference_WithWalkPattern(12,0.15,CPref_horizon,ZMPref_horizon,CoMref_horizon);

//    for(int i=0;i<12;i++) {
//        LIGHT.SavingVector[0+i] = CPref_horizon[i];
//        LIGHT.SavingVector[15+i] = ZMPref_horizon[i];
//        LIGHT.SavingVector[27+i] = CoMref_horizon[i];
//    }

//    delete []CPref_horizon;
//    delete []ZMPref_horizon;
//    delete []CoMref_horizon;

    cout << "LIGHT Total Weight : "<< LIGHT_Info.m_robot << endl;

    // =========================================================

//    system_clock::time_point StartTime = system_clock::now();

//    FILE* pFile = fopen ( "PumpOperationReference_FullScenarioTask.txt" , "r" );
//    if (pFile==NULL) {fputs ("File error",stderr); exit (1);}

//    // obtain file size:
//    int ch;
//    unsigned int number_of_lines = 0;
//    while (EOF != (ch=getc(pFile)))
//        if ('\n' == ch) ++number_of_lines;
//    fseek(pFile, 0, SEEK_SET);

//    LIGHT_WholeMotions.Ps_Future_FullTaskScenario.resize(number_of_lines);
//    LIGHT_WholeMotions.Qp_Future_FullTaskScenario.resize(number_of_lines);

//    // copy the file into the buffer:
//    for(int i=0;i<number_of_lines;i++) {
//        float temp1,temp2,temp3;
//        fscanf(pFile, "%f %f %f \n", &temp1, &temp2, &temp3);
//        LIGHT_WholeMotions.Ps_Future_FullTaskScenario(i) = temp2;
//        LIGHT_WholeMotions.Qp_Future_FullTaskScenario(i) = temp3;
//    }

//    cout << LIGHT_WholeMotions.Ps_Future_FullTaskScenario.segment(0,100).transpose() << endl;

//    // terminate
//    fclose (pFile);

//    system_clock::time_point EndTime1 = system_clock::now();
//    microseconds t_StateEstim = duration_cast<std::chrono::microseconds>(EndTime1 - StartTime);
//    FILE_LOG(logDEBUG1) << "t_StateEstim Time : "<< t_StateEstim.count() <<" usecond(s).";

    // =========================================================

    _OPERATION_ONOFF = false;
    _OPERATION_ORI_PEL = I3;
    _OPERATION_ORI_RF = I3;
    _OPERATION_ORI_LF = I3;
    _OPERATION_POS_PEL2RF = Vector3d::Zero();
    _OPERATION_POS_PEL2LF = Vector3d::Zero();
    _OPERATION_POS_RF2CoM = Vector3d::Zero();
    _OPERATION_POS_RF2LF = Vector3d::Zero();
    _OPERATION_POS_LF2CoM = Vector3d::Zero();
    _OPERATION_POS_LF2RF = Vector3d::Zero();

    _OPERATION_Q = MatrixNd::Zero(LIGHT_ACT_DOF,1);

    while(__IS_WORKING){
        usleep(100*1000);

        if(__AL_TERMINATE == false) {

            // JoyStick Operation Handling
            if(userData->G2M.JoyStick_OnOff == 1 || userData->G2M.JoyStick_OnOff == 2) {
                double f_cut = 10.0;
                LIGHT_WholeMotions.JoyStickCommand_StepX = LPF_1st(LIGHT_WholeMotions.JoyStickCommand_StepX,
                                                                   userData->G2M.JoyStick_Command_X, f_cut);
                LIGHT_WholeMotions.JoyStickCommand_StepY = LPF_1st(LIGHT_WholeMotions.JoyStickCommand_StepY,
                                                                   userData->G2M.JoyStick_Command_Y, f_cut);
                LIGHT_WholeMotions.JoyStickCommand_StepYaw = LPF_1st(LIGHT_WholeMotions.JoyStickCommand_StepYaw,
                                                                   userData->G2M.JoyStick_Command_Yaw, f_cut);
                if(userData->G2M.JoyStick_Command_StartWalk) LIGHT_WholeMotions.JoyStickCommand_StartWalk = true;
                if(userData->G2M.JoyStick_Command_StopWalk) LIGHT_WholeMotions.JoyStickCommand_StopWalk = true;
                if(userData->G2M.JoyStick_OnOff == 2) {
                    userData->G2M.JoyStick_OnOff = 0;
                }
            } else {
                LIGHT_WholeMotions.JoyStickCommand_StepX = 0.0;
                LIGHT_WholeMotions.JoyStickCommand_StepY = 0.0;
                LIGHT_WholeMotions.JoyStickCommand_StepYaw = 0.0;
            }

            // AL Command Handling
            switch(sharedCMD->COMMAND[PODO_NO].USER_COMMAND){
            case LIGHT_ALL_STOP:
            {
                FILE_LOG(logERROR) << "============ Emergency ==============";
                FILE_LOG(logERROR) << "============ Emergency ==============";
                FILE_LOG(logERROR) << "============ Emergency ==============";

                FILE_LOG(logERROR) << " All motion is stopped.";
                FILE_LOG(logERROR) << " All motion is stopped.";
                FILE_LOG(logERROR) << " All motion is stopped.";

                FILE_LOG(logERROR) << "============ Emergency ==============";
                FILE_LOG(logERROR) << "============ Emergency ==============";
                FILE_LOG(logERROR) << "============ Emergency ==============";

                _OPERATION_MODE = _OPERATION_NO;
                sharedCMD->COMMAND[PODO_NO].USER_COMMAND = LIGHT_NO_ACT;
                break;
            }

            case LIGHT_DATA_SAVE:
            {
                if(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0]) {
                    save_PutDataFlag = true;
                    FILE_LOG(logWARNING) << "SAVING AL DATA START!!";
                } else {
                    if(save_PutDataFlag) {
                        if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == NULL) {
                            save_File();
                        } else {
                            save_File(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR);
                        }
                        FILE_LOG(logERROR) << "SAVING AL DATA END!!";
                    }

                    if(save_PutDataFlag_SupPres) {
                        save_File_SupPres();
                    }
                }
                sharedCMD->COMMAND[PODO_NO].USER_COMMAND = LIGHT_NO_ACT;
                break;
            }

            case LIGHT_DATA_SAVE_SUPPRES:
            {
                save_File_SupPres();
                sharedCMD->COMMAND[PODO_NO].USER_COMMAND = LIGHT_NO_ACT;
                break;
            }

//            case LIGHT_DATA_SAVE_SYS_ID:
//            {
//                if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0]) {
//                    save_Flag_SYS_ID = true;
//                    save_DataType_SYS_ID = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0];
//                    FILE_LOG(logWARNING) << "SAVING SYS ID DATA START!!";
//                    FILE_LOG(logWARNING) << "DATA TYPE : " << save_DataType_SYS_ID;
//                } else {
//                    save_Flag_SYS_ID = false;
//                    save_File_SYS_ID(save_DataType_SYS_ID);
//                    FILE_LOG(logERROR) << "SAVING SYS ID DATA END!!";
//                }
//                sharedCMD->COMMAND[PODO_NO].USER_COMMAND = LIGHT_NO_ACT;
//                break;
//            }

            case LIGHT_PARAMETER_SETTING:
            {
                switch(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0])
                {
                case CHECK_PARAMETERS:
                {
                    Show_TuningParameters();
                    break;
                }
                case SUPPORTCONTROL_DSP:
                {
                    double GravityCompensation = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[8];
                    _OPERATION_POS_SPRING = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
                    _OPERATION_ORI_SPRING = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
                    _OPERATION_POS_SPRING2 = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[2];
                    _OPERATION_ORI_SPRING2 = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[3];
                    _OPERATION_POS_DAMPING = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[4];
                    _OPERATION_ORI_DAMPING = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[5];
                    _OPERATION_POS_DAMPING2 = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[6];
                    _OPERATION_ORI_DAMPING2 = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[7];

                    FILE_LOG(logINFO) << " == CoM Damping Control(DSP) Gain Change ==";
                    FILE_LOG(logINFO) << "Gravity Compensation Factor : " << GravityCompensation;
                    FILE_LOG(logINFO) << "Body Position wn : " << _OPERATION_POS_SPRING;
                    FILE_LOG(logINFO) << "Body Position zeta : " << _OPERATION_POS_DAMPING;
                    FILE_LOG(logINFO) << "Body Orientation wn : " << _OPERATION_ORI_SPRING;
                    FILE_LOG(logINFO) << "Body Orientation zeta : " << _OPERATION_ORI_DAMPING;
                    FILE_LOG(logINFO) << "Foot Position wn : " << _OPERATION_POS_SPRING2;
                    FILE_LOG(logINFO) << "Foot Position zeta : " << _OPERATION_POS_DAMPING2;
                    FILE_LOG(logINFO) << "Foot Orientation wn : " << _OPERATION_ORI_SPRING2;
                    FILE_LOG(logINFO) << "Foot Orientation zeta : " << _OPERATION_ORI_DAMPING2;
                    FILE_LOG(logINFO) << " ==========================================";

                    LIGHT_WholeMotions.SetSupportControlGain_DSP(GravityCompensation,
                                                                 Vector3d::Ones()*_OPERATION_POS_SPRING,
                                                                 Vector3d::Ones()*_OPERATION_POS_DAMPING,
                                                                 Vector3d::Ones()*_OPERATION_ORI_SPRING,
                                                                 Vector3d::Ones()*_OPERATION_ORI_DAMPING,
                                                                 Vector3d::Ones()*_OPERATION_POS_SPRING2,
                                                                 Vector3d::Ones()*_OPERATION_POS_DAMPING2,
                                                                 Vector3d::Ones()*_OPERATION_ORI_SPRING2,
                                                                 Vector3d::Ones()*_OPERATION_ORI_DAMPING2);
                    break;
                }
                case SUPPORTCONTROL_RSSP:
                {
                    double GravityCompensation = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[8];
                    _OPERATION_POS_SPRING = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
                    _OPERATION_ORI_SPRING = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
                    _OPERATION_POS_SPRING2 = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[2];
                    _OPERATION_ORI_SPRING2 = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[3];
                    _OPERATION_POS_DAMPING = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[4];
                    _OPERATION_ORI_DAMPING = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[5];
                    _OPERATION_POS_DAMPING2 = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[6];
                    _OPERATION_ORI_DAMPING2 = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[7];

                    FILE_LOG(logINFO) << " == CoM Damping Control(SSP) Gain Change ==";
                    FILE_LOG(logINFO) << "Gravity Compensation Factor : " << GravityCompensation;
                    FILE_LOG(logINFO) << "Body Position wn : " << _OPERATION_POS_SPRING;
                    FILE_LOG(logINFO) << "Body Position zeta : " << _OPERATION_POS_DAMPING;
                    FILE_LOG(logINFO) << "Body Orientation wn : " << _OPERATION_ORI_SPRING;
                    FILE_LOG(logINFO) << "Body Orientation zeta : " << _OPERATION_ORI_DAMPING;
                    FILE_LOG(logINFO) << "Foot Position wn : " << _OPERATION_POS_SPRING2;
                    FILE_LOG(logINFO) << "Foot Position zeta : " << _OPERATION_POS_DAMPING2;
                    FILE_LOG(logINFO) << "Foot Orientation wn : " << _OPERATION_ORI_SPRING2;
                    FILE_LOG(logINFO) << "Foot Orientation zeta : " << _OPERATION_ORI_DAMPING2;
                    FILE_LOG(logINFO) << " ==========================================";

                    LIGHT_WholeMotions.SetSupportControlGain_RSSP(GravityCompensation,
                                                                 Vector3d::Ones()*_OPERATION_POS_SPRING,
                                                                 Vector3d::Ones()*_OPERATION_POS_DAMPING,
                                                                 Vector3d::Ones()*_OPERATION_ORI_SPRING,
                                                                 Vector3d::Ones()*_OPERATION_ORI_DAMPING,
                                                                 Vector3d::Ones()*_OPERATION_POS_SPRING2,
                                                                 Vector3d::Ones()*_OPERATION_POS_DAMPING2,
                                                                 Vector3d::Ones()*_OPERATION_ORI_SPRING2,
                                                                 Vector3d::Ones()*_OPERATION_ORI_DAMPING2);
                    break;
                }
                case SUPPORTCONTROL_LSSP:
                {
                    double GravityCompensation = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[8];
                    _OPERATION_POS_SPRING = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
                    _OPERATION_ORI_SPRING = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
                    _OPERATION_POS_SPRING2 = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[2];
                    _OPERATION_ORI_SPRING2 = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[3];
                    _OPERATION_POS_DAMPING = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[4];
                    _OPERATION_ORI_DAMPING = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[5];
                    _OPERATION_POS_DAMPING2 = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[6];
                    _OPERATION_ORI_DAMPING2 = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[7];

                    FILE_LOG(logINFO) << " == CoM Damping Control(SSP) Gain Change ==";
                    FILE_LOG(logINFO) << "Gravity Compensation Factor : " << GravityCompensation;
                    FILE_LOG(logINFO) << "Body Position wn : " << _OPERATION_POS_SPRING;
                    FILE_LOG(logINFO) << "Body Position zeta : " << _OPERATION_POS_DAMPING;
                    FILE_LOG(logINFO) << "Body Orientation wn : " << _OPERATION_ORI_SPRING;
                    FILE_LOG(logINFO) << "Body Orientation zeta : " << _OPERATION_ORI_DAMPING;
                    FILE_LOG(logINFO) << "Foot Position wn : " << _OPERATION_POS_SPRING2;
                    FILE_LOG(logINFO) << "Foot Position zeta : " << _OPERATION_POS_DAMPING2;
                    FILE_LOG(logINFO) << "Foot Orientation wn : " << _OPERATION_ORI_SPRING2;
                    FILE_LOG(logINFO) << "Foot Orientation zeta : " << _OPERATION_ORI_DAMPING2;
                    FILE_LOG(logINFO) << " ==========================================";

                    LIGHT_WholeMotions.SetSupportControlGain_LSSP(GravityCompensation,
                                                                 Vector3d::Ones()*_OPERATION_POS_SPRING,
                                                                 Vector3d::Ones()*_OPERATION_POS_DAMPING,
                                                                 Vector3d::Ones()*_OPERATION_ORI_SPRING,
                                                                 Vector3d::Ones()*_OPERATION_ORI_DAMPING,
                                                                 Vector3d::Ones()*_OPERATION_POS_SPRING2,
                                                                 Vector3d::Ones()*_OPERATION_POS_DAMPING2,
                                                                 Vector3d::Ones()*_OPERATION_ORI_SPRING2,
                                                                 Vector3d::Ones()*_OPERATION_ORI_DAMPING2);
                    break;
                }
                case SUPPORTCONTROL_FLOAT:
                {
                    _OPERATION_POS_SPRING = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
                    _OPERATION_ORI_SPRING = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
                    _OPERATION_POS_DAMPING = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[2];
                    _OPERATION_ORI_DAMPING = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[3];

                    FILE_LOG(logINFO) << " ====== Foot Impedance Gain Change ======";
                    FILE_LOG(logINFO) << " Position Spring : " << _OPERATION_POS_SPRING << "[N/m]";
                    FILE_LOG(logINFO) << " Position Damper : " << _OPERATION_POS_DAMPING << "[N/(m/s)]";
                    FILE_LOG(logINFO) << " Orientation Spring : " << _OPERATION_ORI_SPRING << "[Nm/rad]";
                    FILE_LOG(logINFO) << " Orientation Damper : " << _OPERATION_ORI_DAMPING << "[Nm/(rad/s)]";
                    FILE_LOG(logINFO) << " ========================================";

                    LIGHT_WholeMotions.SetSupportControlGain_Float(Vector3d::Ones()*_OPERATION_POS_SPRING,
                                                                   Vector3d::Ones()*_OPERATION_POS_DAMPING,
                                                                   Vector3d::Ones()*_OPERATION_ORI_SPRING,
                                                                   Vector3d::Ones()*_OPERATION_ORI_DAMPING);
                    break;
                }
                case JOINGIMPEDANCE_RF:
                {
                    VectorNd _StiffnDamp(24);
                    for(int i=0;i<24;i++) {
                        _StiffnDamp(i) = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[i];
                    }
                    FILE_LOG(logINFO) << "Joint Impedance Gain (RF) Change!";
                    FILE_LOG(logINFO) << "StiffnDamp : " << _StiffnDamp.transpose();
                    LIGHT.SetFootStiffnDamp_RF(_StiffnDamp);
                    break;
                }
                case JOINGIMPEDANCE_LF:
                {
                    VectorNd _StiffnDamp(24);
                    for(int i=0;i<24;i++) {
                        _StiffnDamp(i) = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[i];
                    }
                    FILE_LOG(logINFO) << "Joint Impedance Gain (LF) Change!";
                    FILE_LOG(logINFO) << "StiffnDamp : " << _StiffnDamp.transpose();
                    LIGHT.SetFootStiffnDamp_LF(_StiffnDamp);
                    break;
                }
                case CoMLEADCOMPENSATE:
                {
                    _OPERATION_POS_SPRING = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
                    _OPERATION_POS_DAMPING = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
                    _OPERATION_POS_SPRING2 = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[2];
                    _OPERATION_POS_DAMPING2 = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[3];

                    FILE_LOG(logINFO) << " ====== CoM Lead Compensate Gain Change ======";
                    FILE_LOG(logINFO) << " Wn_X : " << _OPERATION_POS_SPRING;
                    FILE_LOG(logINFO) << " Wn_Y : " << _OPERATION_POS_DAMPING;
                    FILE_LOG(logINFO) << " Zeta_X : " << _OPERATION_POS_SPRING2;
                    FILE_LOG(logINFO) << " Zeta_Y : " << _OPERATION_POS_DAMPING2;
                    FILE_LOG(logINFO) << " =============================================";

                    LIGHT_WholeMotions.SetLeadCompensateGain(_OPERATION_POS_SPRING,_OPERATION_POS_SPRING2,
                                                             _OPERATION_POS_DAMPING,_OPERATION_POS_DAMPING2);
                    break;
                }
                case NO_PARAMTER:
                {
                    break;
                }
                default:
                    break;
                }

                save_File_TuningParameters();
                sharedCMD->COMMAND[PODO_NO].USER_COMMAND = LIGHT_NO_ACT;
                break;
            }

            case LIGHT_SUPPLYPRESSURE_SETTING:
            {
                switch(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0])
                {
                case _OPERATION_LIGHT_FULL_SCINARIO:
                    if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0]) {
                        LIGHT_WholeMotions.Load_FuturePumpReference_FullTaskScenario();
                        FILE_LOG(logSUCCESS) << "Pump Data (Full Task Scenario) Is Activated!!";
                    } else {
                        LIGHT_WholeMotions.IsLoaded_FuturePumpReference_FullTaskScenario = false;
                        FILE_LOG(logERROR) << "Pump Data (Full Task Scenario) Is Deactivated..";
                    } break;
                default:

                    break;
                }


                sharedCMD->COMMAND[PODO_NO].USER_COMMAND = LIGHT_NO_ACT;
                break;
            }

            //  Joint space control ///////////////////////////////////////////////////////////////////

            case LIGHT_GOTO_HOMEPOSE:
            {
                FILE_LOG(logSUCCESS) << "Command LIGHT_GOTO_HOMEPOSE received..";

                // Desired Joint Angles (unit : degree)
                _OPERATION_Q.resize(LIGHT_ACT_DOF,1);
                _OPERATION_Q << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   // right leg
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   // left leg
                        0.0;                           // waist
                _OPERATION_Q = _OPERATION_Q*PI/180.0;
                _OPERATION_TIME = 3.0;

                _OPERATION_MODE_NEW = _OPERATION_GOTO_HOMEPOSE;
                sharedCMD->COMMAND[PODO_NO].USER_COMMAND = LIGHT_NO_ACT;
                break;
            }

            // Task space control - Base is floating  /////////////////////////////////////////////////

            case LIGHT_WORKSPACE_MOVE:
            {
                FILE_LOG(logSUCCESS) << "Command LIGHT_WORKSPACE_MOVE received..";

                double _RIGHT_X = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
                double _RIGHT_Y = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
                double _RIGHT_Z = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[2];
                double _LEFT_X = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[3];
                double _LEFT_Y = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[4];
                double _LEFT_Z = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[5];
                _OPERATION_TIME = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[6];

                // Desired Joint Angles (unit : degree)
                // [Initial Position]
                // _OPERATION_POS_R : 0.0439, -0.15585, -0.4573
                // _OPERATION_ORI_R : identity(3)
                // _OPERATION_POS_R : 0.0439, 0.15585, -0.4573
                // _OPERATION_ORI_R : identity(3)

                _OPERATION_ORI_RF = I3;
                _OPERATION_POS_PEL2RF << _RIGHT_X, _RIGHT_Y, _RIGHT_Z;
                _OPERATION_ORI_LF = I3;
                _OPERATION_POS_PEL2LF << _LEFT_X, _LEFT_Y, _LEFT_Z;

                FILE_LOG(logSUCCESS) << " ==== Work Space Moving Test ====";
                FILE_LOG(logSUCCESS) << " Moving Time : " << _OPERATION_TIME;
                FILE_LOG(logSUCCESS) << " Right Pos : " << _OPERATION_POS_PEL2RF.transpose();
                FILE_LOG(logSUCCESS) << " Left Pos : " << _OPERATION_POS_PEL2LF.transpose();
                FILE_LOG(logSUCCESS) << " ================================";

                _OPERATION_MODE_NEW = _OPERATION_WORKSPACE_MOVING_TEST;
                sharedCMD->COMMAND[PODO_NO].USER_COMMAND = LIGHT_NO_ACT;
                break;
            }

            case LIGHT_AIRWALKING:
            {
                _OPERATION_MAG = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
                _OPERATION_PERIOD = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
                _OPERATION_NUM = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0];

                FILE_LOG(logSUCCESS) << " ======= Air Walking Motion =======";
                FILE_LOG(logSUCCESS) << " Step Length : " << _OPERATION_MAG << "[m]";
                FILE_LOG(logSUCCESS) << " Step Time : " << _OPERATION_PERIOD << "[sec]";
                FILE_LOG(logSUCCESS) << " Number : " << _OPERATION_NUM << "[steps]";
                FILE_LOG(logSUCCESS) << " ==================================";

                _OPERATION_MODE_NEW = _OPERATION_AIRWALKING;
                sharedCMD->COMMAND[PODO_NO].USER_COMMAND = LIGHT_NO_ACT;
                break;
            }

            case LIGHT_TESTJUMPING:
            {
//                if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 0) {
//                    _OPERATION_STAGE = 0;
//                    _OPERATION_X_OFFSET = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[8];
//                    _OPERATION_Z_OFFSET = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[9];
//                    FILE_LOG(logINFO) << "X Offset : " << _OPERATION_X_OFFSET;
//                    FILE_LOG(logINFO) << "Z Offset : " << _OPERATION_Z_OFFSET;
//                }else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 1) {
//                    _OPERATION_POS_SPRING2 = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
//                    _OPERATION_POS_DAMPING2 = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
//                    _OPERATION_ORI_SPRING2 = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[2];
//                    _OPERATION_ORI_DAMPING2 = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[3];
//                    _OPERATION_POS_SPRING3 = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[4];
//                    _OPERATION_POS_DAMPING3 = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[5];
//                    _OPERATION_ORI_SPRING3 = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[6];
//                    _OPERATION_ORI_DAMPING3 = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[7];
//                    _OPERATION_X_OFFSET = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[8];
//                    _OPERATION_Z_OFFSET = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[9];
//                    _OPERATION_GRAVITY = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[10];

//                    _OPERATION_STAGE = 1;
//                    //                FILE_LOG(logWARNING) << "Test Jumping Go!";
//                }else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 2) {
//                    _OPERATION_STAGE = 2;
//                    //                FILE_LOG(logERROR) << "Test Jumping Finish!";
//                }

//                _OPERATION_MODE_NEW = _OPERATION_TESTJUMPING;
//                sharedCMD->COMMAND[PODO_NO].USER_COMMAND = LIGHT_NO_ACT;
//                break;
            }

            // Task space control - Contact to ground  /////////////////////////////////////////////////

            case LIGHT_COM_MOVING_DSP:
            {
                if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == -1) { // w.r.t RF
                    double _CoM_X = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
                    double _CoM_Y = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
                    Pelvis_BaseHeight = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[2];
                    _OPERATION_TIME = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[3];
                    double _Pel_Roll = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[4];

                    _OPERATION_ORI_PEL = RotY(_Pel_Roll/180.0*PI);
                    _OPERATION_ORI_RF = I3;
                    _OPERATION_ORI_LF = I3;
                    _OPERATION_POS_RF2CoM << _CoM_X, _CoM_Y, Pelvis_BaseHeight;
                    _OPERATION_POS_RF2CoM(1) = _OPERATION_POS_RF2CoM(1) + LIGHT.Xref_RF2LF(1)/2.0;
                    wn_LIPM = sqrt(g_const/(Pelvis_BaseHeight-HeightDiff_CoM2Pel));

                    FILE_LOG(logSUCCESS) << " ======= CoM Moving Test ========";
                    FILE_LOG(logSUCCESS) << " Reference Frame : Right Foot ";
                    FILE_LOG(logSUCCESS) << " Moving Time : " << _OPERATION_TIME;
                    FILE_LOG(logSUCCESS) << " Pelvis Orientation : " << endl <<_OPERATION_ORI_PEL;
                    FILE_LOG(logSUCCESS) << " CoM Pos : " << _CoM_X << ", " << _CoM_Y;
                    FILE_LOG(logSUCCESS) << " Base Height : " << Pelvis_BaseHeight;
                    FILE_LOG(logSUCCESS) << " ================================";

                    _OPERATION_MODE_NEW = _OPERATION_CoM_MOVING_RDSP;
                } else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 1) {  // w.r.t LF
                    double _CoM_X = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
                    double _CoM_Y = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
                    Pelvis_BaseHeight = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[2];
                    _OPERATION_TIME = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[3];
                    double _Pel_Pitch = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[4];

                    _OPERATION_ORI_PEL = RotY(_Pel_Pitch/180.0*PI);
                    _OPERATION_ORI_RF = I3;
                    _OPERATION_ORI_LF = I3;
                    _OPERATION_POS_LF2CoM << _CoM_X, _CoM_Y, Pelvis_BaseHeight;
                    _OPERATION_POS_LF2CoM(1) = _OPERATION_POS_LF2CoM(1) + LIGHT.Xref_LF2RF(1)/2.0;
                    wn_LIPM = sqrt(g_const/(Pelvis_BaseHeight-HeightDiff_CoM2Pel));

                    FILE_LOG(logSUCCESS) << " ======= CoM Moving Test ========";
                    FILE_LOG(logSUCCESS) << " Reference Frame : Left Foot ";
                    FILE_LOG(logSUCCESS) << " Moving Time : " << _OPERATION_TIME;
                    FILE_LOG(logSUCCESS) << " Pelvis Orientation : " << endl <<_OPERATION_ORI_PEL;
                    FILE_LOG(logSUCCESS) << " CoM Pos : " << _CoM_X << ", " << _CoM_Y;
                    FILE_LOG(logSUCCESS) << " Base Height : " << Pelvis_BaseHeight;
                    FILE_LOG(logSUCCESS) << " ================================";

                    _OPERATION_MODE_NEW = _OPERATION_CoM_MOVING_LDSP;
                } else {
                    FILE_LOG(logSUCCESS) << " Reference Frame is not defined!";
                }

                sharedCMD->COMMAND[PODO_NO].USER_COMMAND = LIGHT_NO_ACT;
                break;
            }

            case LIGHT_COM_MOVING_SSP:
            {
                if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == -1) { // w.r.t RF
                    if(LIGHT.IsCurRefState_RSSP()) {
                        double _LF_X = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
                        double _LF_Y = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
                        double _LF_Z = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[2];
                        _OPERATION_TIME = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[3];

                        _OPERATION_ORI_PEL = I3;
                        _OPERATION_ORI_RF = I3;
                        _OPERATION_ORI_LF = I3;
                        _OPERATION_POS_RF2CoM << LIGHT.Xref_RF2CoM;
                        _OPERATION_POS_RF2LF << _LF_X, _LF_Y, _LF_Z;

                        FILE_LOG(logSUCCESS) << " ======= CoM Moving Test ========";
                        FILE_LOG(logSUCCESS) << " Reference Frame : Right Foot ";
                        FILE_LOG(logSUCCESS) << " Moving Time : " << _OPERATION_TIME;
                        FILE_LOG(logSUCCESS) << " Pelvis Orientation : " << endl <<_OPERATION_ORI_PEL;
                        FILE_LOG(logSUCCESS) << " LF Pos : " << _LF_X << ", " << _LF_Y << ", " << _LF_Z;
                        FILE_LOG(logSUCCESS) << " ================================";
                        _OPERATION_MODE_NEW = _OPERATION_CoM_MOVING_RSSP;
                    } else {
                        FILE_LOG(logERROR) << " [Error] Current support phase is not RSSP.";
                    }
                } else if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 1) {  // w.r.t LF
                    if(LIGHT.IsCurRefState_LSSP()) {
                        double _RF_X = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
                        double _RF_Y = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
                        double _RF_Z = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[2];
                        _OPERATION_TIME = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[3];

                        _OPERATION_ORI_PEL = I3;
                        _OPERATION_ORI_RF = I3;
                        _OPERATION_ORI_LF = I3;
                        _OPERATION_POS_LF2CoM << LIGHT.Xref_LF2CoM;
                        _OPERATION_POS_LF2RF << _RF_X, _RF_Y, _RF_Z;

                        FILE_LOG(logSUCCESS) << " ======= CoM Moving Test ========";
                        FILE_LOG(logSUCCESS) << " Reference Frame : Left Foot ";
                        FILE_LOG(logSUCCESS) << " Moving Time : " << _OPERATION_TIME;
                        FILE_LOG(logSUCCESS) << " Pelvis Orientation : " << endl <<_OPERATION_ORI_PEL;
                        FILE_LOG(logSUCCESS) << " RF Pos : " << _RF_X << ", " << _RF_Y << ", " << _RF_Z;
                        FILE_LOG(logSUCCESS) << " ================================";
                        _OPERATION_MODE_NEW = _OPERATION_CoM_MOVING_LSSP;
                    } else {
                        FILE_LOG(logERROR) << " [Error] Current support phase is not LSSP.";
                    }
                } else {
                    FILE_LOG(logSUCCESS) << " Reference Frame is not defined!";
                }
                sharedCMD->COMMAND[PODO_NO].USER_COMMAND = LIGHT_NO_ACT;
                break;
            }

            case LIGHT_SUPPORT_TRANSITION:
            {
                _OPERATION_TYPE = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0];
                _OPERATION_MODE_NEW = _OPERATION_SUPPORT_TRANSITION;

                sharedCMD->COMMAND[PODO_NO].USER_COMMAND = LIGHT_NO_ACT;
                break;
            }

            case LIGHT_SYSID_COM:
            {
                FILE_LOG(logINFO) << "Command LIGHT_SYSID_COM received..";
                _OPERATION_MAG      = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0]; // CoM Amplitude
                _OPERATION_PERIOD   = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1]; // Maximum Frequency
                _OPERATION_FREQ     = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[2]; // Frequency ratio
                _OPERATION_TYPE     = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[1];

                _OPERATION_MODE_NEW = _OPERATION_SYSID_COM;
                sharedCMD->COMMAND[PODO_NO].USER_COMMAND = LIGHT_NO_ACT;
                break;
            }

            case LIGHT_SQUAT:
            {
                if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 0) { // DSP Squat Mode
                    if(LIGHT.IsCurRefState_DSP()) {
                        FILE_LOG(logSUCCESS) << "Command LIGHT_SQUAT(DSP) received..";
                        _OPERATION_STEP_LENGTH = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
                        _OPERATION_STEP_TIME = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
                        _OPERATION_STEP_NUMBER = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0];
                        FILE_LOG(logSUCCESS) << " ========== DSP Squat Motion ==========";
                        FILE_LOG(logSUCCESS) << " Height Change : " << _OPERATION_STEP_LENGTH << "[m]";
                        FILE_LOG(logSUCCESS) << " Period : " << _OPERATION_STEP_TIME << "[sec]";
                        FILE_LOG(logSUCCESS) << " Number : " << _OPERATION_STEP_NUMBER << "[steps]";
                        FILE_LOG(logSUCCESS) << " ==================================";
                        _OPERATION_MODE_NEW = _OPERATION_SQUAT_DSP;
                    } else {
                        FILE_LOG(logERROR) << " [Error] Current support phase is not DSP.";
                    }
                } else if (sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 1) { // SSP Squat Mode
                    if(LIGHT.IsCurRefState_RSSP()||LIGHT.IsCurRefState_LSSP()) {
                        FILE_LOG(logSUCCESS) << "Command LIGHT_SQUAT(SSP) received..";
                        _OPERATION_STEP_LENGTH = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
                        _OPERATION_STEP_TIME = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
                        _OPERATION_STEP_NUMBER = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0];
                        FILE_LOG(logSUCCESS) << " ========== SSP Squat Motion ==========";
                        FILE_LOG(logSUCCESS) << " Height Change : " << _OPERATION_STEP_LENGTH << "[m]";
                        FILE_LOG(logSUCCESS) << " Period : " << _OPERATION_STEP_TIME << "[sec]";
                        FILE_LOG(logSUCCESS) << " Number : " << _OPERATION_STEP_NUMBER << "[steps]";
                        FILE_LOG(logSUCCESS) << " ======================================";
                        _OPERATION_MODE_NEW = _OPERATION_SQUAT_SSP;
                    } else {
                        FILE_LOG(logERROR) << " [Error] Current support phase is not SSP.";
                    }
                }
                sharedCMD->COMMAND[PODO_NO].USER_COMMAND = LIGHT_NO_ACT;
                break;
            }

            // Task space control - Related to Walking  /////////////////////////////////////////////////

            case LIGHT_COM_MOVING_RDSP_SMOOTH:
            {
//                FILE_LOG(logSUCCESS) << "Command LIGHT_COM_MOVING_RDSP_SMOOTH received..";
//                _OPERATION_MODE_NEW = _OPERATION_COM_MOVING_RDSP_SMOOTH;
//                sharedCMD->COMMAND[PODO_NO].USER_COMMAND = LIGHT_NO_ACT;
//                break;
            }
            case LIGHT_COM_MOVING_LDSP_SMOOTH:
            {
//                FILE_LOG(logSUCCESS) << "Command LIGHT_COM_MOVING_LDSP_SMOOTH received..";
//                _OPERATION_MODE_NEW = _OPERATION_COM_MOVING_LDSP_SMOOTH;
//                sharedCMD->COMMAND[PODO_NO].USER_COMMAND = LIGHT_NO_ACT;
//                break;
            }
            case LIGHT_RFSWINGUP_DYNAMIC:
            {
                if(LIGHT.IsCurRefState_DSP()) {
                    FILE_LOG(logSUCCESS) << "Command LIGHT_RFSWINGUP_DYNAMIC received..";
                    _OPERATION_MODE_NEW = _OPERATION_RFSWINGUP_DYNAMIC;
                } else {
                    FILE_LOG(logERROR) << " [Error] Current support phase is already SSP.";
                }
                sharedCMD->COMMAND[PODO_NO].USER_COMMAND = LIGHT_NO_ACT;
                break;
            }
            case LIGHT_RFSWINGDOWN_DYNAMIC:
            {
                if(LIGHT.IsCurRefState_LSSP()) {
                    FILE_LOG(logSUCCESS) << "Command LIGHT_RFSWINGDOWN_DYNAMIC received..";
                    _OPERATION_MODE_NEW = _OPERATION_RFSWINGDOWN_DYNAMIC;
                } else {
                    FILE_LOG(logERROR) << " [Error] Current State is not LSSP.";
                }
                sharedCMD->COMMAND[PODO_NO].USER_COMMAND = LIGHT_NO_ACT;
                break;
            }
            case LIGHT_LFSWINGUP_DYNAMIC:
            {
                if(LIGHT.IsCurRefState_DSP()) {
                    FILE_LOG(logSUCCESS) << "Command LIGHT_LFSWINGUP_DYNAMIC received..";
                    _OPERATION_MODE_NEW = _OPERATION_LFSWINGUP_DYNAMIC;
                } else {
                    FILE_LOG(logERROR) << " [Error] Current support phase is already SSP.";
                }
                sharedCMD->COMMAND[PODO_NO].USER_COMMAND = LIGHT_NO_ACT;
                break;
            }
            case LIGHT_LFSWINGDOWN_DYNAMIC:
            {
                if(LIGHT.IsCurRefState_RSSP()) {
                    FILE_LOG(logSUCCESS) << "Command LIGHT_LFSWINGDOWN_DYNAMIC received..";
                    _OPERATION_MODE_NEW = _OPERATION_LFSWINGDOWN_DYNAMIC;
                } else {
                    FILE_LOG(logERROR) << " [Error] Current State is not RSSP.";
                }
                sharedCMD->COMMAND[PODO_NO].USER_COMMAND = LIGHT_NO_ACT;
                break;
            }
            case LIGHT_WALK:
            {
                FILE_LOG(logSUCCESS) << "Command LIGHT_WALK received..";

                _OPERATION_STEP_TIME = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
                _OPERATION_STEP_TIME2 = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
                _OPERATION_STANCE_WIDTH = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[2];
                _OPERATION_STEP_NUMBER = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0];
                _OPERATION_X_OFFSET = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[3];
                _OPERATION_Y_OFFSET = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[4];
                _OPERATION_Z_OFFSET = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[5];

                _OPERATION_STEP_LENGTH = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[6];
                _OPERATION_STEP_LENGTH2 = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[7];
                _OPERATION_STEP_YAW = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[8];
                _OPERATION_STANCE_OFFSET = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[9];

                FILE_LOG(logSUCCESS) << " ========== LIGHT Walking ==========";
                FILE_LOG(logSUCCESS) << " Step Time : " << _OPERATION_STEP_TIME << "[sec]";
                FILE_LOG(logSUCCESS) << " DSP Time : " << _OPERATION_STEP_TIME2 << "[sec]";
                FILE_LOG(logSUCCESS) << " Sway Length : " << _OPERATION_STANCE_WIDTH << "[m]";
                FILE_LOG(logSUCCESS) << " Step Number : " << _OPERATION_STEP_NUMBER << "[steps]";
                FILE_LOG(logSUCCESS) << " X Offset : " << _OPERATION_X_OFFSET << "[m]";
                FILE_LOG(logSUCCESS) << " Y Offset : " << _OPERATION_Y_OFFSET << "[m]";
                FILE_LOG(logSUCCESS) << " Yaw Offset : " << _OPERATION_Z_OFFSET << "[deg]";
                FILE_LOG(logSUCCESS) << " X-direction Step : " << _OPERATION_STEP_LENGTH << "[m]";
                FILE_LOG(logSUCCESS) << " Y-direction Step : " << _OPERATION_STEP_LENGTH2 << "[m]";
                FILE_LOG(logSUCCESS) << " Yaw Rotation : " << _OPERATION_STEP_YAW << "[deg]";
                FILE_LOG(logSUCCESS) << " Stance Offset : " << _OPERATION_STANCE_OFFSET << "[deg]";
                FILE_LOG(logSUCCESS) << " ==================================";

                _OPERATION_MODE_NEW = _OPERATION_WALK;
                sharedCMD->COMMAND[PODO_NO].USER_COMMAND = LIGHT_NO_ACT;
                break;
            }
            case LIGHT_WALK_withJOYSTICK:
            {
                FILE_LOG(logSUCCESS) << "Command LIGHT_WALK_withJOYSTICK received..";

                LIGHT_WholeMotions.JoyStickCommand_StartWalk = false;
                LIGHT_WholeMotions.JoyStickCommand_StopWalk = false;
                _OPERATION_STEP_TIME = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
                _OPERATION_STEP_TIME2 = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
                _OPERATION_STANCE_WIDTH = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[2];
                _OPERATION_STEP_NUMBER = sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0];
                _OPERATION_X_OFFSET = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[3];
                _OPERATION_Y_OFFSET = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[4];
                _OPERATION_Z_OFFSET = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[5];
                _OPERATION_STANCE_OFFSET = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[6];

                FILE_LOG(logSUCCESS) << " ========== LIGHT Walking ==========";
                FILE_LOG(logSUCCESS) << " Step Time : " << _OPERATION_STEP_TIME << "[sec]";
                FILE_LOG(logSUCCESS) << " DSP Time : " << _OPERATION_STEP_TIME2 << "[sec]";
                FILE_LOG(logSUCCESS) << " Sway Length : " << _OPERATION_STANCE_WIDTH << "[m]";
                FILE_LOG(logSUCCESS) << " X Offset : " << _OPERATION_X_OFFSET << "[m]";
                FILE_LOG(logSUCCESS) << " Y Offset : " << _OPERATION_Y_OFFSET << "[m]";
                FILE_LOG(logSUCCESS) << " Yaw Offset : " << _OPERATION_Z_OFFSET << "[deg]";
                FILE_LOG(logSUCCESS) << " Stance Offset : " << _OPERATION_STANCE_OFFSET << "[deg]";
                FILE_LOG(logSUCCESS) << " ==================================";

                _OPERATION_MODE_NEW = _OPERATION_WALK_withJOYSTICK;
                sharedCMD->COMMAND[PODO_NO].USER_COMMAND = LIGHT_NO_ACT;
                break;
            }

            case LIGHT_FULLTASK:
            {
                FILE_LOG(logSUCCESS) << "Command LIGHT_FULLTASK received..";
                _OPERATION_MODE_NEW = _OPERATION_LIGHT_FULL_SCINARIO;
                sharedCMD->COMMAND[PODO_NO].USER_COMMAND = LIGHT_NO_ACT;
                break;
            }
            default:
                break;
            }
        }

        if((_OPERATION_MODE == _OPERATION_NO)&&(_OPERATION_MODE_NEW != _OPERATION_NO)) {
            _OPERATION_MODE = _OPERATION_MODE_NEW;
            _OPERATION_MODE_NEW = _OPERATION_NO;
        } else if((_OPERATION_MODE != _OPERATION_NO)&&(_OPERATION_MODE_NEW != _OPERATION_NO)&&(_OPERATION_MODE != _OPERATION_MODE_NEW)) {
            _OPERATION_MODE_CHANGED = true;
            while(_OPERATION_MODE != _OPERATION_NO) {
                usleep(10*1000);
            }
            _OPERATION_MODE = _OPERATION_MODE_NEW;
            _OPERATION_MODE_NEW = _OPERATION_NO;
            _OPERATION_MODE_CHANGED = false;

        }
    }

    FILE_LOG(logERROR) << "Process \"" << __AL_NAME << "\" is terminated" << endl;
    return 0;
}

//==============================//
// Task Thread
//==============================//
unsigned int _ThreadCnt = 0;
int CNT_display = 0;
bool Flag_DispDebugNumber = false;
void RBTaskThread(void *)
{
    while(__IS_WORKING)
    {        
        system_clock::time_point StartTime = system_clock::now();

        // -----------------------------------
        // Get joint position(sensor data) and LIGHT Qnow Update (real-time)
        // -----------------------------------
        LIGHT.UpdateSensorData();
        LIGHT.UpdateStates();

        system_clock::time_point EndTime1 = system_clock::now();
        microseconds t_StateEstim = duration_cast<std::chrono::microseconds>(EndTime1 - StartTime);
        if(Flag_DispDebugNumber) { 
//            PrintHere(1); 
            FILE_LOG(logDEBUG1) << "StateEstim Time : "<< t_StateEstim.count() <<" usecond(s).";
        }

        // -----------------------------------
        // Current State Check
        // If the state of robot is not good, operation will be stopped.
        // -----------------------------------
        LIGHT.CheckStates();

        system_clock::time_point EndTime2 = system_clock::now();
        microseconds t_StateCheck = duration_cast<std::chrono::microseconds>(EndTime2 - EndTime1);
        if(Flag_DispDebugNumber) { 
//            PrintHere(2); 
            FILE_LOG(logDEBUG1) << "StateCheck Time : "<< t_StateCheck.count() <<" usecond(s).";
        }

        // -----------------------------------
        // Error Check
        // -----------------------------------
        if (LIGHT.Flag_UnnormalStates) {
            _OPERATION_MODE = _OPERATION_NO;
            _OPERATION_MODE_NEW = _OPERATION_NO;
            INFO_InvKin.Reset();

            for(int i=0;i<12;i++) {
                sharedREF->PosOrFor_Selection[i] = JointControlMode_Position;
            }

            LIGHT.dQref = MatrixNd::Zero(LIGHT_DOF,1);
            LIGHT.ddQref = MatrixNd::Zero(LIGHT_DOF,1);
            LIGHT.Tref = MatrixNd::Zero(LIGHT_ACT_DOF,1);
            for(int idx=0; idx<LIGHT_ACT_DOF; idx++){
                jCon->SetJointRefTorque(idx, 0.0);
            }

            if(save_PutDataFlag){
                save_ActivateFlag = true;
            }

            LIGHT.Flag_StateCheck = false;
            LIGHT.Flag_UnnormalStates = false;

            FILE_LOG(logERROR) << " ==== Unnormal States Are Detected!! ==== " << endl;
            __AL_TERMINATE = true;
            LIGHT_WholeMotions.TimeReset();
        }

        if(__AL_TERMINATE) {
            _OPERATION_Q.resize(LIGHT_ACT_DOF,1);
            _OPERATION_Q << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   // right leg
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,   // left leg
                    0.0;                           // waist
            _OPERATION_TIME = 5.0;

            if(LIGHT_WholeMotions.JointSpaceMove_ALL(_OPERATION_TIME, _OPERATION_Q)) {
                __IS_WORKING = false;
            }
        }

        system_clock::time_point EndTime3 = system_clock::now();
        microseconds t_ErrorCheck = duration_cast<std::chrono::microseconds>(EndTime3 - EndTime2);
        if(Flag_DispDebugNumber) { 
//            PrintHere(3); 
            FILE_LOG(logDEBUG1) << "ErrorCheck Time : "<< t_ErrorCheck.count() <<" usecond(s).";
        }

        // -----------------------------------
        // Operation
        // - Pattern Generation
        // >> desired position (Xdes,dXdes,ddXdes)
        // >> desired orientation (Rdes,Wdes,dWdes) setting
        // -----------------------------------
        bool _FINISH = false;

        switch(_OPERATION_MODE)
        {
        case _OPERATION_NO:
            break;

        // Joint Space Operation ----------------------------------------------------------------//
        // ( Desired Task - Total 19)
        //  - Pelvis Zero (6)
        //  - Right Leg Joint Angle (6)
        //  - Left Leg Joint Angle (6)
        //  - Waist Joint Angle (1)
        case _OPERATION_GOTO_HOMEPOSE:
        {
            _FINISH = LIGHT_WholeMotions.JointSpaceMove_ALL(_OPERATION_TIME, _OPERATION_Q);
            break;
        }

        // Pelvis Base Work(Task) Space Operation -----------------------------------------------//
        // ( Desired Task - Total 19)
        //  - Pelvis Zero (6)
        //  - Global Frame >> Right Foot Orientation/Position (6)
        //  - Global Frame >> Left Foot Orientation/Position (6)
        //  - Waist Joint Angle (1)
        case _OPERATION_WORKSPACE_MOVING_TEST:
        {
            _FINISH = LIGHT_WholeMotions.WorkSpaceMove_ALL(_OPERATION_TIME,
                                                           _OPERATION_ORI_RF,_OPERATION_POS_PEL2RF,
                                                           _OPERATION_ORI_LF,_OPERATION_POS_PEL2LF);
            break;
        }
        case _OPERATION_AIRWALKING:
        {
            _FINISH = LIGHT_WholeMotions.AirWalking(_OPERATION_PERIOD, _OPERATION_MAG, _OPERATION_NUM);
            break;
        }
        case _OPERATION_TESTJUMPING:
        {
            //            _FINISH = LIGHT_WholeMotions.TestJumping(_OPERATION_STAGE, _OPERATION_X_OFFSET,_OPERATION_Z_OFFSET, _OPERATION_GRAVITY,
            //                                                     _OPERATION_POS_SPRING2,_OPERATION_POS_DAMPING2,_OPERATION_ORI_SPRING2,_OPERATION_ORI_DAMPING2,
            //                                                     _OPERATION_POS_SPRING3,_OPERATION_POS_DAMPING3,_OPERATION_ORI_SPRING3,_OPERATION_ORI_DAMPING3);
            break;
        }

        // Stance Foot Base Work(Task) Space Operation ------------------------------------------//
        // ( Desired Task - Total 13 (Contact constraint : 6))
        //  - Stance Foot Frame >> Pelvis Orientation (3)
        //  - Stance Foot Frame >> CoM Position (3)
        //  - Stance Foot Frame >> Swing Foot Orientation/Position (6)
        //  - Waist Joint Angle (1)
        case _OPERATION_CoM_MOVING_RDSP:
        {
            _FINISH = LIGHT_WholeMotions.CoM_Move_RDSP(_OPERATION_TIME,
                                                       _OPERATION_ORI_PEL,_OPERATION_ORI_RF,_OPERATION_ORI_LF,
                                                       _OPERATION_POS_RF2CoM,zv,zv);
            break;
        }
        case _OPERATION_CoM_MOVING_LDSP:
        {
            _FINISH = LIGHT_WholeMotions.CoM_Move_LDSP(_OPERATION_TIME,
                                                       _OPERATION_ORI_PEL,_OPERATION_ORI_RF,_OPERATION_ORI_LF,
                                                       _OPERATION_POS_LF2CoM,zv,zv);
            break;
        }
        case _OPERATION_CoM_MOVING_RSSP:
        {
            _FINISH = LIGHT_WholeMotions.CoM_Move_RSSP(_OPERATION_TIME,
                                                       _OPERATION_ORI_PEL,_OPERATION_ORI_RF,_OPERATION_ORI_LF,
                                                       _OPERATION_POS_RF2CoM,zv,zv,
                                                       _OPERATION_POS_RF2LF,zv,zv);
            break;
        }
        case _OPERATION_CoM_MOVING_LSSP:
        {
            _FINISH = LIGHT_WholeMotions.CoM_Move_LSSP(_OPERATION_TIME,
                                                       _OPERATION_ORI_PEL,_OPERATION_ORI_RF,_OPERATION_ORI_LF,
                                                       _OPERATION_POS_LF2CoM,zv,zv,
                                                       _OPERATION_POS_LF2RF,zv,zv);
            break;
        }
        case _OPERATION_SUPPORT_TRANSITION:
        {
            _FINISH = LIGHT_WholeMotions.SupportPhase_Transition(_OPERATION_TYPE);
            break;
        }
        case _OPERATION_SYSID_COM:
        {
            _FINISH = LIGHT_WholeMotions.SYSID_CoMRef2CoM(_OPERATION_TYPE,_OPERATION_MAG,_OPERATION_PERIOD,_OPERATION_FREQ);
            break;
        }
        case _OPERATION_SQUAT_DSP:
        {
            _FINISH = LIGHT_WholeMotions.Squat_DSP(_OPERATION_STEP_LENGTH, _OPERATION_STEP_TIME, _OPERATION_STEP_NUMBER);
            break;
        }
        case _OPERATION_SQUAT_SSP:
        {
            _FINISH = LIGHT_WholeMotions.Squat_SSP(_OPERATION_STEP_LENGTH, _OPERATION_STEP_TIME, _OPERATION_STEP_NUMBER);
            break;
        }
        case _OPERATION_COM_MOVING_RDSP_SMOOTH:
        {
            double _TIME = 2.0;
            double _Yoffset_RF = 0.01;
            _FINISH = LIGHT_WholeMotions.CoM_Move_RDSP_smooth(_TIME, _Yoffset_RF, false);
            break;
        }
        case _OPERATION_COM_MOVING_LDSP_SMOOTH:
        {
            double _TIME = 2.0;
            double _Yoffset_LF = -0.01;
            _FINISH = LIGHT_WholeMotions.CoM_Move_LDSP_smooth(_TIME, _Yoffset_LF, false);
            break;
        }
        case _OPERATION_RFSWINGUP_DYNAMIC:
        {
            Vector3d ZMP_offset;
            ZMP_offset << 0.0, -0.025, 0.0;
            _FINISH = LIGHT_WholeMotions.RFSwingUp_Dynamic(2.0, ZMP_offset, -0.0*D2R);
            break;
        }
        case _OPERATION_RFSWINGDOWN_DYNAMIC:
        {
            _OPERATION_POS_LF2RF << 0.00, -0.210, 0.0;
            _FINISH = LIGHT_WholeMotions.RFSwingDown_Dynamic(2.0, _OPERATION_POS_LF2RF, -0.0*D2R);
            break;
        }
        case _OPERATION_LFSWINGUP_DYNAMIC:
        {
            Vector3d ZMP_offset;
            ZMP_offset << -0.05, 0.025, 0.0;
            _FINISH = LIGHT_WholeMotions.LFSwingUp_Dynamic(2.0, ZMP_offset, 0.0*D2R);
            break;
        }
        case _OPERATION_LFSWINGDOWN_DYNAMIC:
        {
            _OPERATION_POS_RF2LF << 0.00, 0.210, 0.0;
            _FINISH = LIGHT_WholeMotions.LFSwingDown_Dynamic(2.0, _OPERATION_POS_RF2LF, 0.0*D2R);
            break;
        }
        case _OPERATION_WALK:
        {
            double STEP_TIME = _OPERATION_STEP_TIME;
            double DSP_TIME = _OPERATION_STEP_TIME2;
            double X_Offset = _OPERATION_X_OFFSET;
            double Y_Offset = _OPERATION_Y_OFFSET;
            double Yaw_Offset = _OPERATION_Z_OFFSET*D2R;
            double X_STEP = _OPERATION_STEP_LENGTH;
            double Y_STEP = _OPERATION_STEP_LENGTH2;
            double Yaw_STEP = _OPERATION_STEP_YAW*D2R;
            double StanceOffset = _OPERATION_STANCE_OFFSET;
            int CurStepNumber;
            _FINISH = LIGHT_WholeMotions.Walk(STEP_TIME, DSP_TIME, _OPERATION_STANCE_WIDTH, _OPERATION_STEP_NUMBER,
                                              X_Offset, Y_Offset, Yaw_Offset,
                                              X_STEP, Y_STEP, Yaw_STEP, StanceOffset,
                                              false, CurStepNumber);
            break;
        }
        case _OPERATION_WALK_withJOYSTICK:
        {
            if(LIGHT_WholeMotions.JoyStickCommand_StartWalk) {
                double STEP_TIME = _OPERATION_STEP_TIME;
                double DSP_TIME = _OPERATION_STEP_TIME2;
                double X_Offset = _OPERATION_X_OFFSET;
                double Y_Offset = _OPERATION_Y_OFFSET;
                double Yaw_Offset = _OPERATION_Z_OFFSET*D2R;
                double X_STEP = LIGHT_WholeMotions.JoyStickCommand_StepX*0.23;
                double Y_STEP = LIGHT_WholeMotions.JoyStickCommand_StepY*0.10;
                double Yaw_STEP = LIGHT_WholeMotions.JoyStickCommand_StepYaw*12.0*D2R;
                double StanceOffset = _OPERATION_STANCE_OFFSET;
                int CurStepNumber;
                _FINISH = LIGHT_WholeMotions.Walk(STEP_TIME, DSP_TIME, _OPERATION_STANCE_WIDTH, 99999,
                                                  X_Offset, Y_Offset, Yaw_Offset,
                                                  X_STEP, Y_STEP, Yaw_STEP, StanceOffset,
                                                  LIGHT_WholeMotions.JoyStickCommand_StopWalk, CurStepNumber);
            } else {
                static int Cnt_ReadyWalk = 0;
                if(Cnt_ReadyWalk%(int)(SYS_FREQ*3)==0) {
                    FILE_LOG(logWARNING) << "Waiting for JoyStick Command...";
                    FILE_LOG(logWARNING) << "- Start Button : Start Walking";
                    FILE_LOG(logWARNING) << "- Back Button : Cancel";
                    Cnt_ReadyWalk = 0;
                } Cnt_ReadyWalk++;
                if(LIGHT_WholeMotions.JoyStickCommand_StopWalk) {
                    LIGHT_WholeMotions.JoyStickCommand_StartWalk = false;
                    LIGHT_WholeMotions.JoyStickCommand_StopWalk = false;
                    Cnt_ReadyWalk = 0;
                    _FINISH = true;
                }
            }
            break;
        }
        case _OPERATION_LIGHT_FULL_SCINARIO:
        {
            _FINISH = LIGHT_WholeMotions.FullTaskScenario();
            break;
        }
        default:
            break;
        }

        if (_FINISH == true){
            _OPERATION_MODE = _OPERATION_NO;
            FILE_LOG(logERROR) << " Operation is finished!! ";
        }

        // -----------------------------------
        // Sub-Controller (Pattern Modification)
        // >> Desired position/orientation change
        // -----------------------------------

//        switch(LIGHT_WholeMotions.CheckSwingFootCompMode()){
//        case LIGHT_WholeMotions.SWINGFOOT_COMP_FLOATING:
//        {
////            LIGHT_WholeMotions.SwingFootComp_Float();
//            break;
//        }
//        case LIGHT_WholeMotions.SWINGFOOT_COMP_RSP:
//        {
////            LIGHT_WholeMotions.SwingFootComp_RF();
//            break;
//        }
//        case LIGHT_WholeMotions.SWINGFOOT_COMP_LSP:
//        {
//            break;
//        }
//        default:
//            break;
//        }

        system_clock::time_point EndTime4 = system_clock::now();
        microseconds t_MotionGene = duration_cast<std::chrono::microseconds>(EndTime4 - EndTime3);
        if(Flag_DispDebugNumber) { 
//            PrintHere(4); 
            FILE_LOG(logDEBUG1) << "MotionGene Time : "<< t_MotionGene.count() <<" usecond(s).";
        }

        // -----------------------------------
        // Inverse Kinematics
        // -----------------------------------
        if(!INFO_InvKin.CheckWeightZero()) {
            // Derive q, dq
            LIGHT.InverseKinematics();
        }

        system_clock::time_point EndTime5 = system_clock::now();
        microseconds t_InvKinemat = duration_cast<std::chrono::microseconds>(EndTime5 - EndTime4);
        if(Flag_DispDebugNumber) { 
//            PrintHere(5);
            FILE_LOG(logDEBUG1) << "InvKinemat Time : "<< t_InvKinemat.count() <<" usecond(s).";
        }

        // -----------------------------------
        // Inverse Dynamics (CoM Damping Control)
        // -----------------------------------
        switch(LIGHT.CheckCurrentRefState()){
        case LIGHT.REFSTATE_RDSP:
        {
            LIGHT_WholeMotions.SupportControl_DSP_byWBD(LIGHT.Cur_RefFrame);
            break;
        }
        case LIGHT.REFSTATE_LDSP:
        {
            LIGHT_WholeMotions.SupportControl_DSP_byWBD(LIGHT.Cur_RefFrame);
            break;
        }
        case LIGHT.REFSTATE_RSSP:
        {
            LIGHT_WholeMotions.SupportControl_SSP_RF_byWBD();
            break;
        }
        case LIGHT.REFSTATE_LSSP:
        {
            LIGHT_WholeMotions.SupportControl_SSP_LF_byWBD();
            break;
        }
        case LIGHT.REFSTATE_FLOAT:
        {
            LIGHT_WholeMotions.SupportControl_Float_byWBD();
            break;
        }
        case LIGHT.REFSTATE_NOACT:
        {
            LIGHT_WholeMotions.SupportControl_Noact();
            break;
        }
        default:
            break;
        }

        system_clock::time_point EndTime6 = system_clock::now();
        microseconds t_InvDynamic = duration_cast<std::chrono::microseconds>(EndTime6 - EndTime5);
        if(Flag_DispDebugNumber) { 
//            PrintHere(6); 
            FILE_LOG(logDEBUG1) << "InvDynamic Time : "<< t_InvDynamic.count() <<" usecond(s).";
        }

        // -----------------------------------
        // Error Check
        // -----------------------------------
        if (LIGHT.Flag_Error) {
            _OPERATION_MODE = _OPERATION_NO;
            _OPERATION_MODE_NEW = _OPERATION_NO;
            LIGHT.dQref = MatrixNd::Zero(LIGHT_DOF,1);
            LIGHT.ddQref = MatrixNd::Zero(LIGHT_DOF,1);
            LIGHT.Tref = MatrixNd::Zero(LIGHT_ACT_DOF,1);
            INFO_InvKin.Reset();
            INFO_InvKin_SUB.Reset();

            if(save_PutDataFlag){
                save_ActivateFlag = true;
            }
            FILE_LOG(logERROR) << " ==== Error!! Operation is stopped!! ==== " << endl;
            LIGHT.Flag_Error = false;
        }

        // -----------------------------------
        // Update and Set(Command) Reference (real-time)
        // -----------------------------------
        LIGHT.UpdateReferenceStates(RefUpdateEnable);
        LIGHT.SetReference_Joint(RefUpdateEnable);
        LIGHT.SetReference_Actuator(RefUpdateEnable);
        LIGHT.GenerateSupplyPressureReference();

        LIGHT.UpdateFootStiffnDamp(RefUpdateEnable);
        if(_ThreadCnt%((int)(SYS_FREQ/50.0))==0) { // (UpdateFrequency : 50Hz)
            LIGHT.SetActuatorStiffnDamp(RefUpdateEnable);
        }


        system_clock::time_point EndTime7 = system_clock::now();
        microseconds t_RefeUpdate = duration_cast<std::chrono::microseconds>(EndTime7 - EndTime6);                
        if(Flag_DispDebugNumber) { 
//            PrintHere(7); 
            FILE_LOG(logDEBUG1) << "RefeUpdate Time : "<< t_RefeUpdate.count() <<" usecond(s).";
        }

        //////======================================================================================

        if (CNT_display >= 1000) {
            CNT_display = 0;

        //            FILE_LOG(logINFO) << "Znow_SSP_RF : " << LIGHT.Znow_SSP_RF.transpose();
//            FILE_LOG(logINFO) << "Znow_SSP_LF : " << LIGHT.Znow_SSP_LF.transpose();
//            FILE_LOG(logINFO) << "Znow_DSP_RF : " << LIGHT.Znow_DSP_RF.transpose();
//            FILE_LOG(logINFO) << "Znow_DSP_LF : " << LIGHT.Znow_DSP_LF.transpose();
            //            FILE_LOG(logINFO) << "Xnow_RF2LF : " << LIGHT.Xnow_RF2LF.transpose();
            //            FILE_LOG(logINFO) << "Xref_RF2LF : " << LIGHT.Xref_RF2LF.transpose();
//            FILE_LOG(logINFO) << "Wnow_RF : " << LIGHT.Wnow_RF.transpose();
//            FILE_LOG(logINFO) << "dXnow_RF : " << LIGHT.dXnow_RF.transpose();
//            FILE_LOG(logINFO) << "Rnow_RF : " << LIGHT.Rnow_RF;

//            FILE_LOG(logINFO) << "Xnow_RF2CoM : " << LIGHT.Xnow_RF2CoM.transpose();
//            FILE_LOG(logINFO) << "Xnow_LF2CoM : " << LIGHT.Xnow_LF2CoM.transpose();
//            FILE_LOG(logINFO) << "Xnow_CoM : " << LIGHT.Xnow_CoM.transpose();
//            FILE_LOG(logINFO) << "Xnow_RF2CoM : " << LIGHT.Xnow_RF2CoM.transpose();
//            FILE_LOG(logINFO) << "Xdes_RF2CoM : " << LIGHT.Xdes_RF2CoM.transpose();
//            FILE_LOG(logINFO) << "Xdes_RF2Pel : " << -LIGHT.Xnow_Pel2RF.transpose();

//            FILE_LOG(logINFO) << "Pelvis Height : " << LIGHT.Xnow_RF2CoM(2);
//            FILE_LOG(logINFO) << "Com Height : " << LIGHT.Xnow_CoM(2);
//            FILE_LOG(logINFO) << "RF Height : " << -LIGHT.Xnow_Pel2RF(2);
//            FILE_LOG(logINFO) << "LF Height : " << -LIGHT.Xnow_Pel2LF(2);

//            FILE_LOG(logINFO) << "Joint Stiffness at RKN : " << sharedREF->JointStiffness[RKN];
//            FILE_LOG(logINFO) << "Joint Stiffness at LKN : " << sharedREF->JointStiffness[LKN];

            //            if(LIGHT.ContactState == LIGHT.CONTACT_BOTH) {
            //                FILE_LOG(logINFO) << "Znow_DS w.r.t RF : " << LIGHT.Znow_DS_wrtRF.transpose();
            //            } else if (LIGHT.ContactState == LIGHT.CONTACT_RF) {
            //                FILE_LOG(logINFO) << "Znow_RF : " << LIGHT.Znow_RF.transpose();
            //            } else if (LIGHT.ContactState == LIGHT.CONTACT_LF) {
            //                FILE_LOG(logINFO) << "Znow_LF : " << LIGHT.Znow_LF.transpose();
            //            }

//        FILE_LOG(logINFO) << "Fextref_RF : " << LIGHT.Fextref_RF.transpose();
//        FILE_LOG(logINFO) << "Fextref_LF : " << LIGHT.Fextref_LF.transpose();Xnow_RF2LF

//            FILE_LOG(logINFO) << "Xnow_LF2RF : " << LIGHT.Xnow_LF2RF.transpose();
//            FILE_LOG(logINFO) << "Xnow_RF2LF : " << LIGHT.Xnow_RF2LF.transpose();

//            FILE_LOG(logINFO) << "Xnow_Pel : " << LIGHT.Xnow_Pel.transpose();
//            FILE_LOG(logINFO) << "Xnow_Pel2RF : " << LIGHT.Xnow_Pel2RF.transpose();

//            FILE_LOG(logINFO) << "Xref_Pel2RF : " << LIGHT.Xref_Pel2RF.transpose();
//            FILE_LOG(logINFO) << "Xref_Pel2LF : " << LIGHT.Xref_Pel2LF.transpose();

//            FILE_LOG(logINFO) << "Xref_RF : " << LIGHT.Xref_RF.transpose();
//            FILE_LOG(logINFO) << "Xref_LF : " << LIGHT.Xref_LF.transpose();

//            double psi_Rnow_RF = atan2(LIGHT.Rnow_RF(1,0), LIGHT.Rnow_RF(0,0))*R2D;   // z-axis (Yaw)
//            double theta_Rnow_RF = -asin(LIGHT.Rnow_RF(2,0))*R2D;                     // y'-axis (Pitch)
//            double phi_Rnow_RF = atan2(LIGHT.Rnow_RF(2,1), LIGHT.Rnow_RF(2,2))*R2D;   // x''-axis (Roll)
//            FILE_LOG(logINFO) << "RF Yaw Orienation : " << psi_Rnow_RF;
//            FILE_LOG(logINFO) << "RF Orienation : " << psi_Rnow_RF << ", " << theta_Rnow_RF << ", " << phi_Rnow_RF;

//            double psi_Rnow_LF = atan2(LIGHT.Rnow_LF(1,0), LIGHT.Rnow_LF(0,0))*R2D;   // z-axis (Yaw)
//            double theta_Rnow_LF = -asin(LIGHT.Rnow_LF(2,0))*R2D;                     // y'-axis (Pitch)
//            double phi_Rnow_LF = atan2(LIGHT.Rnow_LF(2,1), LIGHT.Rnow_LF(2,2))*R2D;   // x''-axis (Roll)
//            FILE_LOG(logINFO) << "LF Yaw Orienation : " << psi_Rnow_LF;
//            FILE_LOG(logINFO) << "LF Orienation : " << psi_Rnow_LF << ", " << theta_Rnow_LF << ", " << phi_Rnow_LF;

//            double psi_Rnow_Pel = atan2(LIGHT.Rnow_Pel(1,0), LIGHT.Rnow_Pel(0,0))*R2D;   // z-axis (Yaw)
//            double theta_Rnow_Pel = -asin(LIGHT.Rnow_Pel(2,0))*R2D;                     // y'-axis (Pitch)
//            double phi_Rnow_Pel = atan2(LIGHT.Rnow_Pel(2,1), LIGHT.Rnow_Pel(2,2))*R2D;   // x''-axis (Roll)
//            FILE_LOG(logINFO) << "Pel Yaw Orienation : " << psi_Rnow_Pel;
//            FILE_LOG(logINFO) << "Pel Orienation : " << psi_Rnow_Pel << ", " << theta_Rnow_Pel << ", " << phi_Rnow_Pel;

//            double psi_Rpel = atan2(LIGHT.Rdes_Pel(1,0), LIGHT.Rdes_Pel(0,0))*R2D;   // z-axis (Yaw)
//            double theta_Rpel = -asin(LIGHT.Rdes_Pel(2,0))*R2D;                     // y'-axis (Pitch)
//            double phi_Rpel = atan2(LIGHT.Rdes_Pel(2,1), LIGHT.Rdes_Pel(2,2))*R2D;   // x''-axis (Roll)
//            FILE_LOG(logINFO) << "Pel Orienation : " << psi_Rpel << ", " << theta_Rpel << ", " << phi_Rpel;

//                        FILE_LOG(logINFO) << "Wnow_Pel : " << LIGHT.Wnow_Pel.transpose();
            //            FILE_LOG(logINFO) << "Rnow_Pel : " << LIGHT.Rnow_Pel;

//            FILE_LOG(logINFO) << "dQ : " << LIGHT.dQref.transpose();
//            FILE_LOG(logINFO) << "ddQ : " << LIGHT.ddQref.transpose();

//            FILE_LOG(logINFO) << "F_Rknee : " << sharedSEN->ENCODER[RKN][0].CurrentRefActForce;
//            FILE_LOG(logINFO) << "F_Rank1 : " << sharedSEN->ENCODER[RAP][0].CurrentRefActForce;
//            FILE_LOG(logINFO) << "F_Rank2 : " << sharedSEN->ENCODER[RAR][0].CurrentRefActForce;
//            FILE_LOG(logINFO) << "F_Lknee : " << sharedSEN->ENCODER[LKN][0].CurrentRefActForce;
//            FILE_LOG(logINFO) << "F_Lank1 : " << sharedSEN->ENCODER[LAP][0].CurrentRefActForce;
//            FILE_LOG(logINFO) << "F_Lank2 : " << sharedSEN->ENCODER[LAR][0].CurrentRefActForce;


        } CNT_display++;

        _ThreadCnt++;
        if (_ThreadCnt == 1000000) {
            _ThreadCnt = 0;
        }

        if(save_PutDataFlag == true){
            save_PutData(save_Index);
            save_Index++;
            if(save_ActivateFlag == true || save_Index == (SAVENUM-1)) {
                if(save_Index == (SAVENUM-1)) FILE_LOG(logERROR) << "Save Buffer is Full!!";
                save_ActivateFlag = false;
                save_PutDataFlag = false;
                sharedCMD->COMMAND[PODO_NO].USER_COMMAND = LIGHT_DATA_SAVE;
                sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0] = false;
                FILE_LOG(logERROR) << "SAVING AL DATA END!!";
            }
        } else {
            if(save_ActivateFlag == true) {
                save_ActivateFlag = false;
                FILE_LOG(logERROR) << "There is no saved data...";
            }
        }

        if(save_PutDataFlag_SupPres == true){
            save_PutData_SupPres(save_Index_SupPres);
            save_Index_SupPres++;
            if(save_ActivateFlag_SupPres == true || save_Index_SupPres == (SAVENUM_SupPres-1)) {
                if(save_Index_SupPres == (SAVENUM_SupPres-1)) FILE_LOG(logERROR) << "Save Buffer (Supply Pressure) is Full!!";
                save_ActivateFlag_SupPres = false;
                save_PutDataFlag_SupPres = false;
                sharedCMD->COMMAND[PODO_NO].USER_COMMAND = LIGHT_DATA_SAVE_SUPPRES;
                FILE_LOG(logERROR) << "SAVING (Supply Pressure) DATA END. Saved Index : " << save_Index_SupPres;
            }
        } else {
            if(save_ActivateFlag_SupPres == true) {
                save_ActivateFlag_SupPres = false;
                FILE_LOG(logERROR) << "There is no saved data...";
            }
        }

//        if(save_Flag_SYS_ID == true){
//            save_PutData_SYS_ID(save_Index_SYS_ID,save_DataType_SYS_ID,_OPERATION_MAG,_OPERATION_FREQ,_OPERATION_PERIOD);
//            save_Index_SYS_ID++;
//            if(save_Index_SYS_ID == (SAVENUM_SYS_ID-1))
//            {
//                FILE_LOG(logERROR) << "Save Buffer is Full!!";
//                save_Flag_SYS_ID = false;
//                save_File_SYS_ID(save_DataType_SYS_ID);
//                save_Index_SYS_ID=0;
//                FILE_LOG(logERROR) << "SAVING SYS ID DATA END!!";
//            }
//        }

        system_clock::time_point EndTime8 = system_clock::now();
        microseconds t_MsgDisplay = duration_cast<std::chrono::microseconds>(EndTime8 - EndTime7);
        microseconds t_TotalCycle = duration_cast<std::chrono::microseconds>(EndTime8 - StartTime);
        static int OverTimeCnt_4000ms = 0;
        static int OverTimeCnt_3000ms = 0;
        static int OverTimeCnt_2000ms = 0;
        if(t_TotalCycle.count() > 4000) {
            OverTimeCnt_4000ms++;
        } else if(t_TotalCycle.count() > 3000) {
            OverTimeCnt_3000ms++;
        } else if(t_TotalCycle.count() > 2000) {
            OverTimeCnt_2000ms++;
        }
        if(Flag_DispDebugNumber) { 
//            PrintHere(8); 
            FILE_LOG(logDEBUG1) << "MsgDisplay Time : "<< t_MsgDisplay.count() <<" usecond(s).";
            FILE_LOG(logDEBUG1) << "TotalCycle Time : "<< t_TotalCycle.count() <<" usecond(s).";
            FILE_LOG(logDEBUG1) << "OverTimeCnt (4000ms) : "<< OverTimeCnt_4000ms;
            FILE_LOG(logDEBUG1) << "OverTimeCnt (3000ms) : "<< OverTimeCnt_3000ms;
            FILE_LOG(logDEBUG1) << "OverTimeCnt (2000ms) : "<< OverTimeCnt_2000ms;
            FILE_LOG(logDEBUG1) << "=================================";
        }

        //////======================================================================================

        rt_task_suspend(&rtTaskCon);
    }
}
//==============================//


//==============================//
// Flag Thread
//==============================//
int CntFlagThread = 0;
void RBFlagThread(void *)
{
    rt_task_set_periodic(NULL, TM_NOW, 40*1000);

    while(__IS_WORKING)
    {
        rt_task_wait_period(NULL);

//        if(sharedCMD->SYNC_SIGNAL[PODO_NO] == true) { // Daemon : 500Hz, LIGHTWalkingAL : 166.6Hz
//            jCon->JointUpdate();
//            if(CntFlagThread%3 == 0) {
//                rt_task_resume(&rtTaskCon);
//                CntFlagThread = 0;
//            } else {
////                sharedCMD->SYNC_SIGNAL[PODO_NO] = false;
//            }
//            CntFlagThread++;
//        }

        if(sharedCMD->SYNC_SIGNAL[PODO_NO] == true) { // Daemon : 500Hz, LIGHTWalkingAL : 500Hz
            jCon->JointUpdate();
            rt_task_resume(&rtTaskCon);
        }
    }

}


//==============================//
