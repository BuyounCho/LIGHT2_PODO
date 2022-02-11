#include "BasicFiles/BasicSetting.h"
#include "BasicFiles/BasicJoint.h"
#include "PumpController_BasicFunctions.h"
#include "PumpControl_ControlFunctions.h"

#include <unistd.h>
#include <cmath>
#include <chrono>
#include <iostream>
using namespace std::chrono;
using namespace std;

#define PODO_AL_NAME       "PumpControl"

// Variables ===========================================
// Shared memory
pRBCORE_SHM_COMMAND     sharedCMD;
pRBCORE_SHM_REFERENCE   sharedREF;
pRBCORE_SHM_SENSOR      sharedSEN;
JointControlClass       *joint;
JointControlClass       *jCon;


// Program variable
int isTerminated;
int     __IS_WORKING = false;
int     __IS_GAZEBO = false;

int     PODO_NO = -1;

bool Flag_ActiveControl = false;
bool Flag_InstantInput = false;

char _BNO = 0;

double P_sine_mag = 0.0;
double P_sine_per = 0.0;
double P_sine_num = 0.0;

double Ps_fin = 1.0;
double Ps_des[MAX_PREVIEW+1] = {Ps_min,0.0,};
double Ps_now = Ps_min+10.0;
char   PsRef_type = 0;
char   PsRef_type_transition = 0; // -1 : sharedREF >> thisAL, 1 : sharedREF << thisAL

double t = 0.0;
double t_change = 10.0;

double InputVoltage = 75.0;
double InitSpeed = 0.0;
double InitTime = 10.0;

double wp_ref_max = PUMPSPEED_MAX/100.0*InputVoltage*0.95; // [rpm] 0.90 = 90% limit
double wp_ref_min = PUMPSPEED_MIN; // [rpm]

double wp_now = 0.0;
double wdp_now = 0.0;
double wp_ref = 0.0;

extern double Qflow_ref;
extern double Qflow_ref_JointVel;
extern double Qflow_ref_Leakage;
extern double Qflow_ref_Accumulator;
extern double Qflow_ref_Feedback;

double IDX = 0.0;

int _OPERATION_MODE = 0;
enum _OPERATION_MODE_TYPE
{
    _OPERATION_NO = 200,
    _OPERATION_ACTIVE_CONTROL_INIT,
    _OPERATION_PRESSURE_CHANGE_LINEAR,
};

// Save Function ------
#define SAVEKIND    15
#define SAVENUM     100000

bool            save_Flag = false;
unsigned int    save_Index = 0;
float           save_Buf[SAVEKIND][SAVENUM];
void            save_PutData(unsigned int cur_Index);
void            save_File(char* filecomment = "");
void            save_File_forID(int _PWM, int direction);

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
    sprintf(__AL_NAME, PODO_AL_NAME);

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

    // Pump Duty Reference Disable
    sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_INT[0] = 0;   // Pump Reference Disable
    sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_PUMP_ACTIVE_VELOCITY_CONTROL_ONOFF;

    FILE_LOG(logSUCCESS) << "Setting Complete!!";

    // =========================================================

    while(__IS_WORKING){
        usleep(100*1000);

        switch(sharedCMD->COMMAND[PODO_NO].USER_COMMAND){  

        case PUMP_CONTROL_SAVE_DATA:
        {
            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_INT[0]) {
                save_Flag = true;
                FILE_LOG(logWARNING) << "SAVING AL DATA START!!";
            } else {
                if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == NULL) {
                    save_File();
                } else {
                    save_File(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR);
                }
                save_Flag = false;
                FILE_LOG(logERROR) << "SAVING AL DATA END!!";
            }
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = PUMP_CONTROL_NO_ACT;
            break;
        }

        case PUMP_CONTROL_PRESREF_SELECTION:
        {
            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == -1) { // Ps_ref : From This AL
                if(PsRef_type == 0) {
                    FILE_LOG(logERROR) << "Pres. Ref. Mode Error! (already this AL)";
                } else if(PsRef_type == 1) {
                    PsRef_type_transition = -1;
                    FILE_LOG(logSUCCESS) << "Pres. Ref. will be obtained from thisAL.";
                }
            } else if (sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 1){ // Ps_ref : From Shared Memory
                if(PsRef_type == 0) {
                    PsRef_type_transition = 1;
                    FILE_LOG(logSUCCESS) << "Pres. Ref. will be obtained from SharedREF.";
                } else if(PsRef_type == 1) {
                    FILE_LOG(logERROR) << "Pres. Ref. Mode Error! (already SharedREF)";
                }
            }
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = PUMP_CONTROL_NO_ACT;
            break;
        }

        case PUMP_ACTIVE_CONTROL_ONOFF:
        {
            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 1) { // Active Control On!
                InitSpeed = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
                InitTime = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];

                InputVoltage = sharedSEN->PUMP[0].CurrentSettingVoltage;

                wp_ref_max = PUMPSPEED_MAX/100.0*InputVoltage*0.95; // 0.90 = 90% limit
                wp_ref_min = PUMPSPEED_MIN;

                for(int i=0;i<=MAX_PREVIEW;i++) {
                    Ps_des[i] = Ps_min;
                }
                if(Flag_ActiveControl) {
                    FILE_LOG(logERROR) << "Active Duty Control is already Activated!!";
                } else {
                    FILE_LOG(logWARNING) << "Active Duty Control Initialize...";
                    Flag_InstantInput = true;
                    _OPERATION_MODE = _OPERATION_ACTIVE_CONTROL_INIT;
                }
            } else if (sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 0) { // Active Control OFF!
                if(Flag_ActiveControl) {
                    FILE_LOG(logWARNING) << "Active Duty Control is terminated...";
                    Flag_ActiveControl = false;
                    _OPERATION_MODE = _OPERATION_NO;
                } else {
                    FILE_LOG(logERROR) << "Active Duty Control is not Activated!!";
                }
                sharedREF->PumpVelocityReference[0] = 0.0;
                Flag_InstantInput = false;
                FILE_LOG(logERROR) << "Pump Control Is Stopped!!";
            }
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = PUMP_CONTROL_NO_ACT;
            break;
        }

        case PUMP_ACTIVE_CONTROL_MPC_ONOFF:
        {
            if(sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 1) { // Active Control On!
                if(Flag_ActiveControl) {
                    FILE_LOG(logSUCCESS) << "Linear MPC On!";
                    sharedREF->Flag_PumpControlMPC = true;
                } else {
                    FILE_LOG(logERROR) << "Active Duty Control is not Activated!!";
                }
            } else if (sharedCMD->COMMAND[PODO_NO].USER_PARA_CHAR[0] == 0) { // Active Control OFF!
                if(Flag_ActiveControl) {
                    FILE_LOG(logWARNING) << "Linear MPC Off!";
                    sharedREF->Flag_PumpControlMPC = false;
                } else {
                    FILE_LOG(logERROR) << "Active Duty Control is not Activated!!";
                }
            }
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = PUMP_CONTROL_NO_ACT;
            break;
        }
        case PUMP_ACTIVE_CONTROL_LINEAR:
        {
            if(Flag_ActiveControl) {
                Ps_fin = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[0];
                t_change = sharedCMD->COMMAND[PODO_NO].USER_PARA_DOUBLE[1];
                _OPERATION_MODE = _OPERATION_PRESSURE_CHANGE_LINEAR;
            } else {
                FILE_LOG(logERROR) << "Active Duty Control is not Activated!!";
            }
            sharedCMD->COMMAND[PODO_NO].USER_COMMAND = PUMP_CONTROL_NO_ACT;
            break;
        }

        default:
            break;
        }
    }

    FILE_LOG(logERROR) << "Process \"" << __AL_NAME << "\" is terminated" << endl;
    return 0;
}


int CNT_Display = 0;
//==============================//
// Task Thread
//==============================//
void RBTaskThread(void *)
{
    while(__IS_WORKING)
    {
        system_clock::time_point CurrentTime = system_clock::now();

//        static system_clock::time_point LastTime = CurrentTime;
//        microseconds t_interval = duration_cast<std::chrono::microseconds>(CurrentTime - LastTime);
//        LastTime = CurrentTime;
//        FILE_LOG(logDEBUG1) << "Time Interval : "<< t_interval.count() <<" usecond(s).";

//        // MPC Performance Checking ======================
//        static double t_Sim = 0.0;
//        double dT_Sim = 0.004;
//        static bool Flag_Sim = true;

//        int N_window = 30;
//        double dT_window = 0.02;

//        static VectorNd Xnow = VectorNd::Zero(2);

//        if (t_Sim < 5.0) {
//            if(t_Sim == 0.0) {
//                Xnow(0) = 60.0;
//                Xnow(1) = 50.0;
//            }
//            cout << "Ps (t = " << t_Sim << ") : " << Xnow(0) << endl;

//            double Q_act;
////            PumpControl_FutureRefGenerator_SingleJoint(t_Sim,N_window,dT_window,Ps_des,Q_act);

//            for(int i=0;i<N_window;i++) {
//                for(int j=0;j<MAX_MC;j++){
//                    sharedREF->LoadPressureReference_Future[i][j] = Ps_des;
//                }
//                sharedREF->ActFlowrateReference_Future[i][0] = 0.0;
//            }

//            PumpPressureController_LinearMPC_Formulation(Xnow, N_window, dT_window);

//            VectorNd U_sol = VectorNd::Zero(N_window);
//            PumpPressureController_LinearMPC_Solve(U_sol);

//            double Ps_next;
//            Xnow(1) += U_sol(0)*SYS_DT;
//            double wp_now = Xnow(1);
//            PumpPressure_Dynamics(Xnow(0), wp_now, 0.0, SYS_DT, Ps_next);
//            Xnow(0) = Ps_next;

//            t_Sim += dT_Sim;
//            save_Flag = true;
//        } else {
//            if(Flag_Sim) {
//                save_Flag = false;
//                save_File("LinearMPCTest");
//                Flag_Sim = false;
//            }
//        }


        // Read Sensor Data ===============================================================================
        bool MPCTestByVirtualModel = false;
        if(MPCTestByVirtualModel) {
            // Virtual model --------------------------------------------
            double Qout_temp = 0.0;
            for(int i=0;i<=MAX_MC;i++) {
                Qout_temp += sharedREF->ActFlowrateReference_Future[0][i];
            }

            double Ps_next, wp_next;
            double wdp = wp_ref/60.0*2.0*PI; // rpm >> rad/s
            PumpPressure_Dynamics(Ps_now, wp_now, wdp, Qout_temp, SYS_DT, Ps_next, wp_next);
            Ps_now = Ps_next;
            wp_now = wp_next*60.0/(2.0*PI); // rad/s >> rpm
            cout << "Ps_des : " << sharedREF->PumpPressureReference[0] << endl;
            cout << "Ps_now : " << Ps_now << endl;
            //  Virtual model --------------------------------------------
        } else {
            Ps_now = sharedSEN->PUMP[0].CurrentPressure;
            wp_now = sharedSEN->PUMP[0].CurrentVelocity; // [rpm]
        }

        // Instant Pump Reference Generation  =============================================================
        switch(_OPERATION_MODE)
        {
        case _OPERATION_NO:
            break;
        case _OPERATION_ACTIVE_CONTROL_INIT:
        {
            static double TempSpeed = 0.0;
            double Ps_start = 0.0;
            if(MPCTestByVirtualModel) {
                Ps_start = 0.0;
            } else {
                Ps_start = Ps_min+3.0;
            }

            if(Ps_now < Ps_start) {
                wp_ref = TempSpeed;
                if(TempSpeed<InitSpeed){
                    TempSpeed = TempSpeed + (InitSpeed/InitTime)*SYS_DT;
                } else {
                    TempSpeed = InitSpeed;
                }
                Ps_des[0] = Ps_min;
            } else {
                TempSpeed = 0.0;
                Ps_fin = Ps_min;
                t_change = 1.0;
                Flag_ActiveControl = true;
                _OPERATION_MODE = _OPERATION_PRESSURE_CHANGE_LINEAR;
            }
            break;
        }
        case _OPERATION_PRESSURE_CHANGE_LINEAR:
        {
            double dT_window = sharedREF->dT_PrevPump;
            double P_next, dP_next;
            for(int i=0;i<=MAX_PREVIEW;i++) {
                double t_view = SYS_DT + dT_window*(double)i;
                if(t+t_view <=t_change) {
                    linear_trajectory_pressure(t_change-t, t_view, Ps_des[0], Ps_fin, P_next, dP_next);
                    Ps_des[i] = P_next;
                } else {
                    Ps_des[i] = Ps_fin;
                }
            }

            t += SYS_DT;
            if (t >= t_change) {
                FILE_LOG(logSUCCESS) << "Pressure Change is Finished!!";
                t = 0.0;
                _OPERATION_MODE = _OPERATION_NO;
            }
            break;
        }
        default:
            break;
        }

        // Future Pump Reference Generation  =============================================================
        VectorNd Ps_des_window = VectorNd::Zero(sharedREF->N_PrevPump+1);
        VectorNd Qact_des_window = VectorNd::Zero(sharedREF->N_PrevPump+1);
        for(int i=0;i<=sharedREF->N_PrevPump;i++) {
            double Ps_ref = 0.0; // [bar]
            double Ps_AL = Ps_des[i];
            double Ps_SM = sharedREF->RequiredPressureReference_Future[i];

            static double alpha_trans = 0.0;
            if(PsRef_type_transition == 1) {
                double alpha_trans_prev = alpha_trans + (double)i*sharedREF->dT_PrevPump/5.0;
                if(alpha_trans_prev>1.0) alpha_trans_prev = 1.0;
                Ps_ref = alpha_trans_prev*Ps_SM + (1.0-alpha_trans_prev)*Ps_AL;
                alpha_trans = alpha_trans + SYS_DT/5.0/(double)(sharedREF->N_PrevPump+1);
                if(alpha_trans > 1.0) {
                    alpha_trans = 1.0;
                    PsRef_type_transition = 0;
                    PsRef_type = 1;
                    FILE_LOG(logSUCCESS) << "Pres. Ref. Transition Done! : ThisAL >> SharedREF";
                }
            } else if (PsRef_type_transition == -1) {
                double alpha_trans_prev = alpha_trans - (double)i*sharedREF->dT_PrevPump/5.0;
                if(alpha_trans_prev<0.0) alpha_trans_prev = 0.0;
                Ps_ref = alpha_trans_prev*Ps_SM + (1.0-alpha_trans_prev)*Ps_AL;
                alpha_trans = alpha_trans - SYS_DT/5.0/(double)(sharedREF->N_PrevPump+1);
                if(alpha_trans < 0.0) {
                    alpha_trans = 0.0;
                    PsRef_type_transition = 0;
                    PsRef_type = 0;
                    FILE_LOG(logSUCCESS) << "Pres. Ref. Transition Done! : SharedREF >> ThisAL";
                }
            } else if (PsRef_type_transition == 0) {
                if(PsRef_type == 0) {
                    Ps_ref = Ps_AL;
                } else if (PsRef_type == 1) {
                    Ps_ref = Ps_SM;
                }
            }
            Ps_ref = Ps_ref + Ps_margin;

            if(Ps_ref < Ps_min) {
                Ps_ref = Ps_min;
            } else if (Ps_ref > Ps_max) {
                Ps_ref = Ps_max;
            }
            Ps_des_window(i) = Ps_ref;
            Qact_des_window(i) = sharedREF->RequiredFlowrateReference_Future[i];
        }

        sharedREF->PumpPressureReference[0] = Ps_des_window(0);

        // Pump Speed Controller Selection ==============================================================

        if(Flag_ActiveControl) {
            if(sharedREF->Flag_PumpControlMPC) {
                double wdp_last = wp_ref/60.0*(2.0*PI); // rpm >> rad/s
                PumpPressureController_LinearMPC_NoDelay_Formulation(Ps_now, wdp_last,
                                                                     Ps_des_window, Qact_des_window);
                VectorNd U_sol = VectorNd::Zero(sharedREF->N_PrevPump);
                PumpPressureController_LinearMPC_Solve(U_sol);
                wp_ref = U_sol(0)*60.0/(2.0*PI); // rad/s >> rpm
            } else {
                double u_sol = 0.0;
                double Ps_des_in = Ps_des_window(0);
                double dPs_des_in = (Ps_des_window(1)-Ps_des_window(0))/sharedREF->dT_PrevPump;
                PumpFlowControl_Active_LIGHT2(Ps_now,Ps_des_in, dPs_des_in, u_sol);
//                PumpFlowControl_Active_QuadTest(Ps_now, PsRef_type, u_sol);
                wp_ref = u_sol;
            }
        }

        double fcut_wp = 20.0;
        double alpha_wp = 1.0/(1.0+2.0*PI*fcut_wp*SYS_DT);
        static double wp_ref_fil = 0.0;
        wp_ref_fil = alpha_wp*wp_ref_fil + (1.0-alpha_wp)*wp_ref;

        double wp_ref_sat;
        if(wp_ref_fil < wp_ref_min) {
            wp_ref_sat = wp_ref_min;
        } else if (wp_ref_fil > wp_ref_max) {
            wp_ref_sat = wp_ref_max;
        } else {
            wp_ref_sat = wp_ref_fil;
        }
        sharedREF->PumpVelocityReference[0] = wp_ref_sat;

        // Debugging ===================================================================================

        system_clock::time_point CurrentTime2 = system_clock::now();
        microseconds t_interval2 = duration_cast<std::chrono::microseconds>(CurrentTime2 - CurrentTime);
//        FILE_LOG(logDEBUG1) << "Time Interval : "<< t_interval2.count() <<" usecond(s).";

//        static int OverTimeCnt_4000ms = 0;
//        static int OverTimeCnt_3000ms = 0;
//        static int OverTimeCnt_2000ms = 0;
//        if(t_interval2.count() > 4000) {
//            OverTimeCnt_4000ms++;
//            FILE_LOG(logDEBUG1) << "OverTimeCnt (4000ms) : "<< OverTimeCnt_4000ms;
//        } else if(t_interval2.count() > 3000) {
//            OverTimeCnt_3000ms++;
//            FILE_LOG(logDEBUG1) << "OverTimeCnt (3000ms) : "<< OverTimeCnt_3000ms;
//        } else if(t_interval2.count() > 2000) {
//            OverTimeCnt_2000ms++;
//        }


        if (CNT_Display>=100) {
//            cout << "Ps_des : "<< sharedREF->PumpPressureReference[0] << endl;
//            FILE_LOG(logDEBUG) << "Qflow_ref_JointVel : " << Qflow_ref_JointVel;
//            FILE_LOG(logDEBUG) << "Qflow_ref_Leakage : " << Qflow_ref_Leakage;
//            FILE_LOG(logDEBUG) << "Qflow_ref_Accumulator : " << Qflow_ref_Accumulator;
//            FILE_LOG(logDEBUG) << "Qflow_ref_Feedback : " << Qflow_ref_Feedback;
            CNT_Display = 0;
        } CNT_Display++;

        if(save_Flag == true){
            if(CNT_Display % 1 == 0) {
                save_PutData(save_Index);
                save_Index++;
                if(save_Index == (SAVENUM-1)) {
                    save_Index=0;
                }
            }
        }

        jCon->MoveAllJoint();
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

//        if(sharedCMD->SYNC_SIGNAL[PODO_NO] == true) { // Daemon : 250Hz, PumpControl AL : 125Hz
//            jCon->JointUpdate();
//            if(CntFlagThread%2 == 1) {
//                rt_task_resume(&rtTaskCon);
//                CntFlagThread = 1;
//            } else {
////                rt_task_resume(&rtTaskCon);
//            }
//            CntFlagThread++;
//        }

        if(sharedCMD->SYNC_SIGNAL[PODO_NO] == true){
            jCon->JointUpdate();
            rt_task_resume(&rtTaskCon);
        }
    }

}

void save_PutData(unsigned int cur_Index)
{
    // System Period
    save_Buf[0][cur_Index] = SYS_DT;

    save_Buf[1][cur_Index] = Ps_now; // Ps_now [bar]
    save_Buf[2][cur_Index] = wp_now; // W_now [rpm]
    save_Buf[3][cur_Index] = sharedSEN->PUMP[0].CurrentTemperature; // Wire Temperature [deg]

    save_Buf[4][cur_Index] = Qflow_ref;
    save_Buf[5][cur_Index] = Qflow_ref_JointVel;
    save_Buf[6][cur_Index] = Qflow_ref_Leakage;
    save_Buf[7][cur_Index] = Qflow_ref_Accumulator;
    save_Buf[8][cur_Index] = Qflow_ref_Feedback;

    save_Buf[9][cur_Index] = sharedREF->PumpPressureReference[0]; // [bar]
    save_Buf[10][cur_Index] = sharedREF->PumpVelocityReference[0]; // [rpm]

}

void save_File(char *filecomment)
{
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "PumpControl_AL_Data/%Y%m%d_%X");
    string filedate_s = ss.str();
    filedate_s.erase(filedate_s.end()-6);
    filedate_s.erase(filedate_s.end()-3);
    const char* filedate = filedate_s.c_str();
    const char* filetype = ".txt";
    char filename[100];

    strcpy(filename, filedate);
    if(!filecomment[0] == NULL) {
        strcat(filename, "_");
        strcat(filename, filecomment);
    }
    strcat(filename, filetype);

    FILE* fp;
    unsigned int i, j;
    fp = fopen(filename, "w");

    for(i=0 ; i<save_Index ; i++)
    {
        for(j=0 ; j<SAVEKIND ; j++){
            fprintf(fp, "%f\t", save_Buf[j][i]);
        }
        fprintf(fp, "\n");
    }
    fclose(fp);

    save_Index=0;
    save_Flag=0;

    std::cout << "Saved Filename : " << filename << std::endl;
}


//////=====================================================================================

