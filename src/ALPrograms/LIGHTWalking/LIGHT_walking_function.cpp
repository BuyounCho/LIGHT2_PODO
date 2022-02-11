#include "LIGHT_motion.h"
#include "LIGHT_savedata.h"

extern pRBCORE_SHM_COMMAND     sharedCMD;
extern pRBCORE_SHM_REFERENCE   sharedREF;
extern pRBCORE_SHM_SENSOR      sharedSEN;

extern LIGHT_InvKinematics_INFO INFO_InvKin;
extern LIGHT_InvKinematics_SUB_INFO INFO_InvKin_SUB;

extern LIGHTWholeBody LIGHT;
extern LIGHTWholeMotions LIGHT_WholeMotions;
extern INFO_LIGHT LIGHT_Info;

extern Vector3d OrientationError(Matrix3d Rdes, Matrix3d R);

extern void Resize_TempMatrix(int m, int n, MatrixNd& A, VectorNd& B);
extern void Matrix4QP_Addition(MatrixNd& A_cost, VectorNd& B_cost, MatrixNd A_temp, VectorNd B_temp);

extern bool _OPERATION_MODE_CHANGED;

extern Vector3d _Xini_temp;
extern Matrix3d _Rini_temp, _Rini_temp2, _Rini_temp3;

// Cost Function Weight
double W_SystemDynamics = 100000.0;
double W_CPTracking = 100.0;
double W_ZMPTracking = 1.0;
double W_Delta_Pos = 1e-6;
double W_Delta_Vel = 1e-6;
double W_Delta_PosRef = 1e-6;
double W_Delta_VelRef = 1e-6;
double W_Delta_Input = 1.0;

// MPC Model
int _N_horizon = 12;
double _dT_horizon = 0.12;
double _StepTime_Nom = 0.60;

double _T_wait = 3.0;

bool IsWalking = false;


void SetMotionBase_RF(bool IsDSP)
{
    if(IsDSP) {
        LIGHT.CurRefStateIs_RDSP();
    } else {
        LIGHT.CurRefStateIs_RSSP();
    }
    // Set Cost Weight for Kinematics Problem
    INFO_InvKin.WeightSet_RFBaseMotion();

    LIGHT.Xdes_RF = zv;
    LIGHT.dXdes_RF = zv;
    LIGHT.Xdes_RF2LF = LIGHT.Xref_RF2LF;
    LIGHT.dXdes_RF2LF = LIGHT.dXref_RF2LF;
    LIGHT.ddXdes_RF2LF = LIGHT.ddXref_RF2LF;
    LIGHT.Xdes_RF2CoM = LIGHT.Xref_RF2CoM;
    LIGHT.dXdes_RF2CoM = LIGHT.dXref_RF2CoM;
    LIGHT.ddXdes_RF2CoM = LIGHT.ddXref_RF2CoM;
}

void SetMotionBase_LF(bool IsDSP)
{
    if(IsDSP) {
        LIGHT.CurRefStateIs_LDSP();
    } else {
        LIGHT.CurRefStateIs_LSSP();
    }
    // Set Cost Weight for Kinematics Problem
    INFO_InvKin.WeightSet_LFBaseMotion();

    LIGHT.Xdes_LF = zv;
    LIGHT.dXdes_LF = zv;
    LIGHT.Xdes_LF2RF = LIGHT.Xref_LF2RF;
    LIGHT.dXdes_LF2RF = LIGHT.dXref_LF2RF;
    LIGHT.ddXdes_LF2RF = LIGHT.ddXref_LF2RF;
    LIGHT.Xdes_LF2CoM = LIGHT.Xref_LF2CoM;
    LIGHT.dXdes_LF2CoM = LIGHT.dXref_LF2CoM;
    LIGHT.ddXdes_LF2CoM = LIGHT.ddXref_LF2CoM;
}


// ======================== Air Walking Motion ============================= //

enum AirWalking_STAGE {
    AirWalking_START = 0,
    AirWalking_RF_SWING,
    AirWalking_LF_SWING,
    AirWalking_FINISH,
};
unsigned int AirWalking_CurrentStage = AirWalking_STAGE::AirWalking_START;

bool LIGHTWholeMotions::AirWalking(double _TIME, double _LENGTH, int _NUM)
{
    // _TIME : Step Time
    // _LENGTH : Step Length (stride)
    // _NUM : the number of steps

    static Matrix3d _Rdes_RF;
    static Vector3d _Xdes_RF;
    static Matrix3d _Rdes_LF;
    static Vector3d _Xdes_LF;
    static int   NUM_now;

    double _Z_Stance = -0.65;
    double _Z_SwingUp = 0.10;
    double _WIDTH = 0.250;

    switch(AirWalking_CurrentStage) {
    case AirWalking_START:
    {
        double t_ready = 3.0;
        if (TimeIsZero()) {
            _Rdes_RF = Matrix3d::Identity();
            _Xdes_RF << -_LENGTH/2.0, -_WIDTH/2.0, _Z_Stance;
            _Rdes_LF = Matrix3d::Identity();
            _Xdes_LF << _LENGTH/2.0, _WIDTH/2.0, _Z_Stance;

            FILE_LOG(logWARNING) << " Air Walking Motion Ready... ";
        }

        bool FINISH = WorkSpaceMove_ALL(t_ready,
                                        _Rdes_RF ,_Xdes_RF,
                                        _Rdes_LF, _Xdes_LF);
        if(FINISH) {
            AirWalking_CurrentStage = AirWalking_RF_SWING;
            NUM_now = 0;
        }  return false; // not finished!
    }
    case AirWalking_RF_SWING:
    {
        if (TimeIsZero()) {
            LIGHT.ContactOff_RF();
            LIGHT.ContactOff_LF();
            TimeLimitSet(_TIME);
            FILE_LOG(logSUCCESS) << " Air Walking Motion Go!! ";
        }

        // LIPM Dynamics
        Vector3d _Xdes_LF = Vector3d(0.0, _WIDTH/2.0, _Z_Stance);
        Vector3d _dXdes_LF = Vector3d(0.0, 0.0, 0.0);
        Vector3d _ddXdes_LF = Vector3d(0.0, 0.0, 0.0);
        _Xdes_LF(0) = _LENGTH/2.0*cosh(wn_LIPM*_t)-_LENGTH/(2.0*tanh(wn_LIPM*_TIME/2.0))*sinh(wn_LIPM*_t);
        _dXdes_LF(0) = wn_LIPM*(_LENGTH/2.0*sinh(wn_LIPM*_t)-_LENGTH/(2.0*tanh(wn_LIPM*_TIME/2.0))*cosh(wn_LIPM*_t));
        _ddXdes_LF(0) = wn_LIPM*wn_LIPM*(_LENGTH/2.0*cosh(wn_LIPM*_t)-_LENGTH/(2.0*tanh(wn_LIPM*_TIME/2.0))*sinh(wn_LIPM*_t));

        Submotion_Pelvis2LF_Pos(SYS_DT, _Xdes_LF, _dXdes_LF, _ddXdes_LF);

        Vector3d _Xdes_LF2RF = Vector3d(_LENGTH, -_WIDTH, 0.00);
        Submotion_LF2RFSwing_forWalking(_TIME, _Xdes_LF2RF,_Z_SwingUp);
        LIGHT.Xdes_Pel2RF = LIGHT.Xdes_Pel2LF + LIGHT.Xdes_LF2RF;
        LIGHT.dXdes_Pel2RF = LIGHT.dXdes_Pel2LF + LIGHT.dXdes_LF2RF;
        LIGHT.ddXdes_Pel2RF = LIGHT.ddXdes_Pel2LF + LIGHT.ddXdes_LF2RF;

        TimeUpdate();
        if (TimeCheck()){
            TimeReset();
            NUM_now++;
            if(NUM_now == _NUM) {
                AirWalking_CurrentStage = AirWalking_FINISH;
                LIGHT.dXdes_RF = zv;
                LIGHT.ddXdes_RF = zv;
                LIGHT.dXdes_LF = zv;
                LIGHT.ddXdes_LF = zv;
            } else {
                AirWalking_CurrentStage = AirWalking_LF_SWING;
            }
        } return false;
    }
    case AirWalking_LF_SWING:
    {
        if (TimeIsZero()) {
            LIGHT.ContactOff_RF();
            LIGHT.ContactOff_LF();
            TimeLimitSet(_TIME);
            FILE_LOG(logSUCCESS) << " Air Walking Motion Go!! ";
        }

        // LIPM Dynamics
        Vector3d _Xdes_RF = Vector3d(0.0, -_WIDTH/2.0, _Z_Stance);
        Vector3d _dXdes_RF = Vector3d(0.0, 0.0, 0.0);
        Vector3d _ddXdes_RF = Vector3d(0.0, 0.0, 0.0);
        _Xdes_RF(0) = _LENGTH/2.0*cosh(wn_LIPM*_t)-_LENGTH/(2.0*tanh(wn_LIPM*_TIME/2.0))*sinh(wn_LIPM*_t);
        _dXdes_RF(0) = wn_LIPM*(_LENGTH/2.0*sinh(wn_LIPM*_t)-_LENGTH/(2.0*tanh(wn_LIPM*_TIME/2.0))*cosh(wn_LIPM*_t));
        _ddXdes_RF(0) = wn_LIPM*wn_LIPM*(_LENGTH/2.0*cosh(wn_LIPM*_t)-_LENGTH/(2.0*tanh(wn_LIPM*_TIME/2.0))*sinh(wn_LIPM*_t));

        Submotion_Pelvis2RF_Pos(SYS_DT, _Xdes_RF, _dXdes_RF, _ddXdes_RF);

        Vector3d _Xdes_RF2LF = Vector3d(_LENGTH, _WIDTH, 0.00);
        Submotion_RF2LFSwing_forWalking(_TIME,_Xdes_RF2LF,_Z_SwingUp);
        LIGHT.Xdes_Pel2LF = LIGHT.Xdes_Pel2RF + LIGHT.Xdes_RF2LF;
        LIGHT.dXdes_Pel2LF = LIGHT.dXdes_Pel2RF + LIGHT.dXdes_RF2LF;
        LIGHT.ddXdes_Pel2LF = LIGHT.ddXdes_Pel2RF + LIGHT.ddXdes_RF2LF;

        TimeUpdate();
        if (TimeCheck()){
            TimeReset();
            NUM_now++;
            if(NUM_now == _NUM) {
                AirWalking_CurrentStage = AirWalking_FINISH;
                //                LIGHT.dXdes_RF = zv;
                //                LIGHT.ddXdes_RF = zv;
                //                LIGHT.dXdes_LF = zv;
                //                LIGHT.ddXdes_LF = zv;
            } else {
                AirWalking_CurrentStage = AirWalking_RF_SWING;
            }
        } return false;
    }
    case AirWalking_FINISH:
    {
        double t_finish = 1.5;
        if (TimeIsZero()) {
            _Rdes_RF = Matrix3d::Identity();
            _Xdes_RF << 0.04, -_WIDTH/2.0, _Z_Stance;
            _Rdes_LF = Matrix3d::Identity();
            _Xdes_LF << 0.04, _WIDTH/2.0, _Z_Stance;
            FILE_LOG(logSUCCESS) << " Air Walking Motion Finished!! ";
        }

        bool FINISH = WorkSpaceMove_ALL(t_finish,
                                        _Rdes_RF ,_Xdes_RF,
                                        _Rdes_LF, _Xdes_LF);
        if(FINISH) {
            TimeReset();
            AirWalking_CurrentStage = AirWalking_STAGE::AirWalking_START;
            return true;
        } else { return false; }
    }
    }
}

////////////////////////////////////////////////////////////////////////////////////////

// ======================== Static Walking Motion ============================= //

enum StaticWalking_STAGE {
    START = 0,
    CoMMove2LF,
    RFMove,
    CoMMove2RF,
    LFMove,
    TERMINATE,
    FINISH
};
int StaticWalking_CurrentStage = StaticWalking_STAGE::START;

int StaticWalking_StepNumNow = 0;
bool StaticWalking_Flag_LastStep = false;

double StaticWalking_RF_iniX = 0.0;
double StaticWalking_RF_iniY = 0.0;
double StaticWalking_RF_iniZ = 0.0;
double StaticWalking_LF_iniX = 0.0;
double StaticWalking_LF_iniY = 0.0;
double StaticWalking_LF_iniZ = 0.0;
double StaticWalking_upZ = 0.08;
double StaticWalking_additionalZ = -0.0;
double StaticWalking_Width = 0.180;

bool LIGHTWholeMotions::StaticWalking(double _STEP_LENGTH, double _STEP_TIME, int _STEP_NUM)
{
    switch(StaticWalking_CurrentStage) {
    case StaticWalking_STAGE::START:
    {
        static double t_ready;
        if (TimeIsZero()) {
            t_ready = 2.0;
            TimeLimitSet(t_ready);

            // Set Cost Weight for Kinematics Problem
            INFO_InvKin.WeightSet_RFBaseMotion();

            FILE_LOG(logWARNING) << " Static Walking Motion Ready... ";
        }
        // =============== Global >> Pelvis Orientation Trajectory ===============
        Submotion_Global2Pel_Ori(TimeNow(), TimeLimit(),I3);
        // =============== Global >> RF Orientation Trajectory ===============
        Submotion_Global2RF_Ori(TimeNow(), TimeLimit(),I3);
        // =============== Global >> LF Orientation Trajectory ===============
        Submotion_Global2LF_Ori(TimeNow(), TimeLimit(),I3);
        // =============== RightFoot >> CoM Position Trajectory ===============
        Vector3d Xfin_RF2CoM;
        Xfin_RF2CoM << 0.0, StaticWalking_Width/2.0, Pelvis_BaseHeight;
        Submotion_RF2CoM_Pos(TimeLeft(), Xfin_RF2CoM, zv, zv);
        // =============== RightFoot >> LeftFoot Trajectory ===============
        Submotion_RF2LF_Pos(TimeLeft(), LIGHT.Xref_RF2LF, zv, zv);

        TimeUpdate();
        if(TimeCheck()) {
            StaticWalking_CurrentStage = StaticWalking_STAGE::CoMMove2LF;
            TimeReset();
        }
        return false;
    }
    case StaticWalking_STAGE::CoMMove2LF:
    {
        static double t_CoMMove2LF;
        if (TimeIsZero()) {
            t_CoMMove2LF = 2.0;
            TimeLimitSet(t_CoMMove2LF);

            // Set Cost Weight for Kinematics Problem
            INFO_InvKin.WeightSet_LFBaseMotion();

            FILE_LOG(logWARNING) << " CoM Move to Left Foot! ";
        }
        // =============== LeftFoot >> CoM Position Trajectory ===============
        Vector3d Xfin_LF2CoM;
        Xfin_LF2CoM << 0.0, 0.0, Pelvis_BaseHeight;
        Submotion_LF2CoM_Pos(TimeLeft(), Xfin_LF2CoM, zv, zv);
        // =============== LeftFoot >> RightFoot Trajectory ===============
        Submotion_LF2RF_Pos(TimeLeft(), LIGHT.Xref_LF2RF, zv, zv);

        TimeUpdate();
        if (TimeCheck()) {
            TimeReset();
            StaticWalking_CurrentStage = StaticWalking_STAGE::RFMove;
        }
        return false;
    }
    case StaticWalking_STAGE::RFMove:
    {
        static double t_RFMove;
        if (TimeIsZero()) {
            t_RFMove = _STEP_TIME;
            TimeLimitSet(t_RFMove);
            StaticWalking_RF_iniX = LIGHT.Xref_LF2RF(0);
            StaticWalking_RF_iniY = LIGHT.Xref_LF2RF(1);
            StaticWalking_RF_iniZ = LIGHT.Xref_LF2RF(2);
            FILE_LOG(logWARNING) << " Right Foot Moves! ";
        }
        // =============== LeftFoot >> CoM Position Trajectory ===============
        Submotion_LF2CoM_Pos(TimeLeft(), LIGHT.Xref_LF2CoM, zv, zv);
        // =============== LeftFoot >> RightFoot Position Trajectory ===============
        double T = t_RFMove;
        double Des_Pos_X,Des_Vel_X,Des_Acc_X;
        double Des_Pos_Y,Des_Vel_Y,Des_Acc_Y;
        double Des_Pos_Z,Des_Vel_Z,Des_Acc_Z;
        if(StaticWalking_Flag_LastStep == false) {
            Des_Pos_X = StaticWalking_RF_iniX + (_STEP_LENGTH/2.0-StaticWalking_LF_iniX)*(1.0-cos(PI*TimeNow()/T))/2.0;
            Des_Vel_X = (_STEP_LENGTH/2.0-StaticWalking_LF_iniX)*(PI/T)*sin(PI*TimeNow()/T)/2.0;
            Des_Acc_X = (_STEP_LENGTH/2.0-StaticWalking_LF_iniX)*(PI/T)*(PI/T)*cos(PI*TimeNow()/T)/2.0;
        } else {
            Des_Pos_X = StaticWalking_RF_iniX*(1.0+cos(PI*TimeNow()/T))/2.0;
            Des_Vel_X = -StaticWalking_RF_iniX*(PI/T)*sin(PI*TimeNow()/T)/2.0;
            Des_Acc_X = -StaticWalking_RF_iniX*(PI/T)*(PI/T)*cos(PI*TimeNow()/T)/2.0;
        }
        Des_Pos_Y = -StaticWalking_Width;
        Des_Vel_Y = 0.0;
        Des_Acc_Y = 0.0;
        Des_Pos_Z = StaticWalking_LF_iniZ + StaticWalking_upZ*(1.0-cos(2.0*PI*TimeNow()/T))/2.0;
        Des_Vel_Z = StaticWalking_upZ*(2.0*PI/T)*sin(2.0*PI*TimeNow()/T)/2.0;
        Des_Acc_Z = StaticWalking_upZ*(2.0*PI/T)*(2.0*PI/T)*cos(2.0*PI*TimeNow()/T)/2.0;

        Vector3d Xdes_LF2RF, dXdes_LF2RF, ddXdes_LF2RF;
        Xdes_LF2RF << Des_Pos_X, Des_Pos_Y, Des_Pos_Z;
        dXdes_LF2RF << Des_Vel_X, Des_Vel_Y, Des_Vel_Z;
        ddXdes_LF2RF << Des_Acc_X, Des_Acc_Y, Des_Acc_Z;
        Submotion_LF2RF_Pos(SYS_DT, Xdes_LF2RF, dXdes_LF2RF, ddXdes_LF2RF);

        Vector3d LF2RF_add = Vector3d(0.0,0.0,TimeNow()/T*StaticWalking_additionalZ);

        TimeUpdate();
        if (TimeCheck()){
            TimeReset();
            if(StaticWalking_Flag_LastStep) {
                StaticWalking_CurrentStage = StaticWalking_STAGE::TERMINATE;
            } else {
                StaticWalking_CurrentStage = StaticWalking_STAGE::CoMMove2RF;
                StaticWalking_StepNumNow++;
                if(StaticWalking_StepNumNow == (_STEP_NUM)) { StaticWalking_Flag_LastStep = true; }
            }
        }
        return false;
    }
    case StaticWalking_STAGE::CoMMove2RF:
    {
        static double t_CoMMove2RF;
        if (TimeIsZero()) {
            t_CoMMove2RF = 2.0;
            TimeLimitSet(t_CoMMove2RF);

            // Set Cost Weight for Kinematics Problem
            INFO_InvKin.WeightSet_RFBaseMotion();

            FILE_LOG(logWARNING) << " CoM Move to Right Foot! ";
        }

        // =============== RightFoot >> CoM Position Trajectory ===============
        Vector3d Xfin_RF2CoM;
        Xfin_RF2CoM << 0.0, 0.00, Pelvis_BaseHeight;
        Submotion_RF2CoM_Pos(TimeLeft(), Xfin_RF2CoM, zv, zv);
        // =============== RightFoot >> LeftFoot Trajectory ===============
        Submotion_RF2LF_Pos(TimeLeft(), LIGHT.Xref_RF2LF, zv, zv);

        TimeUpdate();
        if(TimeCheck()) {
            TimeReset();
            StaticWalking_CurrentStage = StaticWalking_STAGE::LFMove;
        }
        return false;
    }
    case StaticWalking_STAGE::LFMove:
    {
        static double t_LFMove;
        if (TimeIsZero()) {
            t_LFMove = _STEP_TIME;
            TimeLimitSet(t_LFMove);
            StaticWalking_LF_iniX = LIGHT.Xref_RF2LF(0);
            StaticWalking_LF_iniY = LIGHT.Xref_RF2LF(1);
            StaticWalking_LF_iniZ = LIGHT.Xref_RF2LF(2);
            FILE_LOG(logWARNING) << " Right Foot Moves! ";
        }

        // =============== RightFoot >> CoM Position Trajectory ===============
        Submotion_RF2CoM_Pos(TimeLeft(), LIGHT.Xref_RF2CoM, zv, zv);
        // =============== RightFoot >> LeftFoot Position Trajectory ===============
        double T = t_LFMove;
        double Des_Pos_X,Des_Vel_X,Des_Acc_X;
        double Des_Pos_Y,Des_Vel_Y,Des_Acc_Y;
        double Des_Pos_Z,Des_Vel_Z,Des_Acc_Z;
        if(StaticWalking_Flag_LastStep == false) {
            Des_Pos_X = StaticWalking_LF_iniX + (_STEP_LENGTH/2.0-StaticWalking_LF_iniX)*(1.0-cos(PI*TimeNow()/T))/2.0;
            Des_Vel_X = (_STEP_LENGTH/2.0-StaticWalking_LF_iniX)*(PI/T)*sin(PI*TimeNow()/T)/2.0;
            Des_Acc_X = (_STEP_LENGTH/2.0-StaticWalking_LF_iniX)*(PI/T)*(PI/T)*cos(PI*TimeNow()/T)/2.0;
        } else {
            Des_Pos_X = StaticWalking_LF_iniX*(1.0+cos(PI*TimeNow()/T))/2.0;
            Des_Vel_X = -StaticWalking_LF_iniX*(PI/T)*sin(PI*TimeNow()/T)/2.0;
            Des_Acc_X = -StaticWalking_LF_iniX*(PI/T)*(PI/T)*cos(PI*TimeNow()/T)/2.0;
        }
        Des_Pos_Y = StaticWalking_Width;
        Des_Vel_Y = 0.0;
        Des_Acc_Y = 0.0;
        Des_Pos_Z = StaticWalking_LF_iniZ + StaticWalking_upZ*(1.0-cos(2.0*PI*TimeNow()/T))/2.0;
        Des_Vel_Z = StaticWalking_upZ*(2.0*PI/T)*sin(2.0*PI*TimeNow()/T)/2.0;
        Des_Acc_Z = StaticWalking_upZ*(2.0*PI/T)*(2.0*PI/T)*cos(2.0*PI*TimeNow()/T)/2.0;

        Vector3d Xdes_RF2LF, dXdes_RF2LF, ddXdes_RF2LF;
        Xdes_RF2LF << Des_Pos_X, Des_Pos_Y, Des_Pos_Z;
        dXdes_RF2LF << Des_Vel_X, Des_Vel_Y, Des_Vel_Z;
        ddXdes_RF2LF << Des_Acc_X, Des_Acc_Y, Des_Acc_Z;
        Submotion_RF2LF_Pos(SYS_DT, Xdes_RF2LF, dXdes_RF2LF, ddXdes_RF2LF);

        Vector3d RF2LF_add = Vector3d(0.0,0.0,TimeNow()/T*StaticWalking_additionalZ);

        TimeUpdate();
        if (TimeCheck()){
            TimeReset();
            if(StaticWalking_Flag_LastStep) {
                StaticWalking_CurrentStage = StaticWalking_STAGE::TERMINATE;
            } else {
                StaticWalking_CurrentStage = StaticWalking_STAGE::CoMMove2LF;
                StaticWalking_StepNumNow++;
                if(StaticWalking_StepNumNow == (_STEP_NUM)) { StaticWalking_Flag_LastStep = true; }
            }
        }
        return false;
    }
    case StaticWalking_STAGE::TERMINATE:
    {
        static double t_finish;

        if((_STEP_NUM % 2) == 1) {
            if (TimeIsZero()) {
                t_finish = 2.0;
                TimeLimitSet(t_finish);

                // Set Cost Weight for Kinematics Problem
                INFO_InvKin.WeightSet_RFBaseMotion();

                FILE_LOG(logWARNING) << " Return to Walk Ready! ";
            }

            // =============== RightFoot >> CoM Position Trajectory ===============
            Vector3d Xfin_RF2CoM;
            Xfin_RF2CoM << 0.0, StaticWalking_Width/2.0, Pelvis_BaseHeight;
            Submotion_RF2CoM_Pos(TimeLeft(), Xfin_RF2CoM, zv, zv);
            // =============== RightFoot >> LeftFoot Trajectory ===============
            Submotion_RF2LF_Pos(TimeLeft(), LIGHT.Xref_RF2LF, zv, zv);

        } else {
            if (TimeIsZero()) {
                t_finish = 3.0;
                TimeLimitSet(t_finish);

                // Set Cost Weight for Kinematics Problem
                INFO_InvKin.WeightSet_LFBaseMotion();

                FILE_LOG(logWARNING) << " Return to Walk Ready! ";
            }

            // =============== LeftFoot >> CoM Position Trajectory ===============
            Vector3d Xfin_LF2CoM;
            Xfin_LF2CoM << 0.0, -StaticWalking_Width/2.0, Pelvis_BaseHeight;
            Submotion_LF2CoM_Pos(TimeLeft(), Xfin_LF2CoM, zv, zv);
            // =============== LeftFoot >> RightFoot Trajectory ===============
            Submotion_LF2RF_Pos(TimeLeft(), LIGHT.Xref_LF2RF, zv, zv);
        }

        TimeUpdate();
        if(TimeCheck()) {
            StaticWalking_CurrentStage = StaticWalking_STAGE::FINISH;
            StaticWalking_StepNumNow = 0;
            StaticWalking_Flag_LastStep = false;
        }
        return false;
    }
    case StaticWalking_STAGE::FINISH:
        if (TimeIsZero()) {
            FILE_LOG(logSUCCESS) << " Static Walking Finish! ";
        }
        TimeUpdate();
        return false;
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Dynamic Walking Motion ///////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

// Step Data Buffer Handling ==========================================================================

double _T_DSP           = 0.01;
double _T_SWING         = 0.01;

void LIGHTWholeMotions::StepInterferenceCheck(int i)
{
    double YawAngle_Saturation = LIGHT_Info.YawStepInside_Saturation;
    double Distance_Saturation = LIGHT_Info.YStepInside_Saturation;
    if(SDB[i].NextSupportPhase == SUPPORTCONTROL_RDSP || SDB[i].NextSupportPhase == SUPPORTCONTROL_RSSP) {
        if(SDB[i].Yaw_STEP > YawAngle_Saturation) {
            SDB[i].Yaw_STEP =  YawAngle_Saturation;
        }
        Vector3d v1 = RotZ(SDB[i].Yaw_STEP)*Vector3d(LIGHT_Info.footsize_toe,LIGHT_Info.footsize_inside,0.0);
        Vector3d v2 = RotZ(SDB[i].Yaw_STEP)*Vector3d(-LIGHT_Info.footsize_heel,LIGHT_Info.footsize_inside,0.0);
        if(SDB[i].X_STEP(1) > -(Distance_Saturation-LIGHT_Info.footsize_inside)-max(v1(1),v2(1))) {
            SDB[i].X_STEP(1) = -(Distance_Saturation-LIGHT_Info.footsize_inside)-max(v1(1),v2(1));
        }
    } else {
        if(SDB[i].Yaw_STEP < -YawAngle_Saturation) {
            SDB[i].Yaw_STEP = -YawAngle_Saturation;
        }
        Vector3d v1 = RotZ(SDB[i].Yaw_STEP)*Vector3d(LIGHT_Info.footsize_toe,LIGHT_Info.footsize_inside,0.0);
        Vector3d v2 = RotZ(SDB[i].Yaw_STEP)*Vector3d(-LIGHT_Info.footsize_heel,LIGHT_Info.footsize_inside,0.0);
        if(SDB[i].X_STEP(1) < (Distance_Saturation-LIGHT_Info.footsize_inside)+max(v1(1),v2(1))) {
            SDB[i].X_STEP(1) = (Distance_Saturation-LIGHT_Info.footsize_inside)+max(v1(1),v2(1));
        }
    }
}

void LIGHTWholeMotions::StepAdjustment()
{

    if( (SDB[0].t_STEP-TimeNow()) >= SDB[0].t_DSP) {
        double Kp_adj = 10.0;
        Vector3d X_adj = zv;
        if((CurStepSupportPhase == SUPPORTCONTROL_RDSP)||(CurStepSupportPhase == SUPPORTCONTROL_RSSP)) {
            X_adj = Kp_adj*(LIGHT.dXnow_RF2CoM-LIGHT.dXref_RF2CoM)/wn_LIPM;
        } else if((CurStepSupportPhase == SUPPORTCONTROL_LDSP)||(CurStepSupportPhase == SUPPORTCONTROL_LSSP)) {
            X_adj = Kp_adj*(LIGHT.dXnow_LF2CoM-LIGHT.dXref_LF2CoM)/wn_LIPM;
        }
        X_adj(2) = 0.0;
        SDB[0].X_STEP_adj = X_adj;
        FILE_LOG(logDEBUG1) << "X_adj : " << X_adj.transpose();

        SDB[0].ZMP = SDB[0].X_STEP + SDB[0].X_STEP_adj
                + RotZ(SDB[0].Yaw_STEP)*SDB[0].X_OFFSET;
        SDB[1].ZMP = SDB[0].X_STEP + SDB[0].X_STEP_adj
                + RotZ(SDB[0].Yaw_STEP)*SDB[1].X_STEP
                + RotZ(SDB[0].Yaw_STEP+SDB[1].Yaw_STEP)*SDB[1].X_OFFSET;
        SDB[2].ZMP = SDB[0].X_STEP + SDB[0].X_STEP_adj
                + RotZ(SDB[0].Yaw_STEP)*SDB[1].X_STEP
                + RotZ(SDB[0].Yaw_STEP+SDB[1].Yaw_STEP)*SDB[2].X_STEP
                + RotZ(SDB[0].Yaw_STEP+SDB[1].Yaw_STEP+SDB[2].Yaw_STEP)*SDB[2].X_OFFSET;
        SDB[3].ZMP = SDB[0].X_STEP + SDB[0].X_STEP_adj
                + RotZ(SDB[0].Yaw_STEP)*SDB[1].X_STEP
                + RotZ(SDB[0].Yaw_STEP+SDB[1].Yaw_STEP)*SDB[2].X_STEP
                + RotZ(SDB[0].Yaw_STEP+SDB[1].Yaw_STEP+SDB[2].Yaw_STEP)*SDB[3].X_STEP
                + RotZ(SDB[0].Yaw_STEP+SDB[1].Yaw_STEP+SDB[2].Yaw_STEP+SDB[3].Yaw_STEP)*SDB[3].X_OFFSET;
    } else {
        SDB[0].X_STEP_adj = zv;
    }
}

void LIGHTWholeMotions::StepDataBuffer_Clear(int _N) {
    // _N=0 : All step info clear
    // _N=1 : second~last step info clear
    // _N=2 : third~last ...
    for(int i=_N;i<PREVIEW_STEPS;i++) {
        SDB[i].Occupied = false;
        SDB[i].t_STEP = 0.0;
        SDB[i].X_STEP = zv;
        SDB[i].X_STEP_adj = zv;
        SDB[i].X_OFFSET = zv;
        SDB[i].Yaw_STEP = 0.0;
        SDB[i].NextSupportPhase = SUPPORTCONTROL_NOACT;
        SDB[i].FinalStep = false;
    }
}

void LIGHTWholeMotions::StepDataBuffer_Initialize_LDSP2RDSP(double _StepTime, double _X_Offset, double _Y_Offset)
{
    StepDataBuffer_Clear();

    t_CurSTEP = 0.0;
    CurStepSupportPhase = SUPPORTCONTROL_LDSP;

    SDB[0].Occupied = true;
    SDB[0].NextSupportPhase = SUPPORTCONTROL_RDSP;
    SDB[0].t_STEP = _StepTime;
    SDB[0].X_STEP = LIGHT.Xref_LF2RF;
    SDB[0].V_DES = zv;
    SDB[0].X_OFFSET << _X_Offset, _Y_Offset, 0.0;
    SDB[0].Yaw_STEP = 0.0;
    SDB[0].DSP_Transition = true;
    SDB[0].t_DSP = SDB[0].t_STEP;
    SDB[0].FinalStep = true;

    SDB[0].ZMP = SDB[0].X_STEP + RotZ(SDB[0].Yaw_STEP)*SDB[0].X_OFFSET;
}

void LIGHTWholeMotions::StepDataBuffer_Initialize_RDSP2LDSP(double _StepTime, double _X_Offset, double _Y_Offset)
{
    StepDataBuffer_Clear();

    t_CurSTEP = 0.0;
    CurStepSupportPhase = SUPPORTCONTROL_RDSP;

    SDB[0].Occupied = true;
    SDB[0].NextSupportPhase = SUPPORTCONTROL_LDSP;
    SDB[0].t_STEP = _StepTime;
    SDB[0].X_STEP = LIGHT.Xref_RF2LF;
    SDB[0].V_DES = zv;
    SDB[0].X_OFFSET << _X_Offset, _Y_Offset, 0.0;
    SDB[0].Yaw_STEP = 0.0;
    SDB[0].DSP_Transition = true;
    SDB[0].t_DSP = SDB[0].t_STEP;
    SDB[0].FinalStep = true;

    SDB[0].ZMP = SDB[0].X_STEP + RotZ(SDB[0].Yaw_STEP)*SDB[0].X_OFFSET;
}

void LIGHTWholeMotions::StepDataBuffer_Initialize_RFSwingUp(double _StepTime, double _X_Offset, double _Y_Offset)
{
    StepDataBuffer_Clear();

    t_CurSTEP = 0.0;
    CurStepSupportPhase = SUPPORTCONTROL_RDSP;

    SDB[0].Occupied = true;
    SDB[0].NextSupportPhase = SUPPORTCONTROL_LSSP;
    SDB[0].t_STEP = _StepTime;
    SDB[0].X_STEP = LIGHT.Xref_RF2LF;
    SDB[0].V_DES = zv;
    SDB[0].X_OFFSET << _X_Offset, _Y_Offset, 0.0;
    SDB[0].Yaw_STEP = 0.0;
    SDB[0].DSP_Transition = true;
    SDB[0].t_DSP = SDB[0].t_STEP;
    SDB[0].FinalStep = true;

    SDB[0].ZMP = SDB[0].X_STEP + RotZ(SDB[0].Yaw_STEP)*SDB[0].X_OFFSET;
}

void LIGHTWholeMotions::StepDataBuffer_Initialize_RFSwingDown(double _StepTime, double _DSPTime, double _X_Step, double _Y_Step, double _Yaw_Step)
{
    StepDataBuffer_Clear();

    t_CurSTEP = 0.0;
    CurStepSupportPhase = SUPPORTCONTROL_LSSP;

    SDB[0].Occupied = true;
    SDB[0].NextSupportPhase = SUPPORTCONTROL_RDSP;
    SDB[0].t_STEP = _StepTime;
    SDB[0].X_STEP << _X_Step, _Y_Step, 0.0;
    SDB[0].V_DES = zv;
    SDB[0].X_OFFSET = -0.5*SDB[0].X_STEP;
    SDB[0].Yaw_STEP = 0.0;
    if(_DSPTime == 0.0) {
        SDB[0].DSP_Transition = false;
        SDB[0].t_DSP = 0.0;
    } else {
        SDB[0].DSP_Transition = true;
        SDB[0].t_DSP = _DSPTime;
    }
    SDB[0].FinalStep = true;

    SDB[0].ZMP = SDB[0].X_STEP + RotZ(SDB[0].Yaw_STEP)*SDB[0].X_OFFSET;
}

void LIGHTWholeMotions::StepDataBuffer_Initialize_LFSwingUp(double _StepTime, double _X_Offset, double _Y_Offset)
{
    StepDataBuffer_Clear();

    t_CurSTEP = 0.0;
    CurStepSupportPhase = SUPPORTCONTROL_LDSP;

    SDB[0].Occupied = true;
    SDB[0].NextSupportPhase = SUPPORTCONTROL_RDSP;
    SDB[0].t_STEP = _StepTime;
    SDB[0].X_STEP = LIGHT.Xref_LF2RF;
    SDB[0].V_DES = zv;
    SDB[0].X_OFFSET << _X_Offset, _Y_Offset, 0.0;
    SDB[0].Yaw_STEP = 0.0;
    SDB[0].DSP_Transition = true;
    SDB[0].t_DSP = SDB[0].t_STEP;
    SDB[0].FinalStep = true;

    SDB[0].ZMP = SDB[0].X_STEP + RotZ(SDB[0].Yaw_STEP)*SDB[0].X_OFFSET;
}

void LIGHTWholeMotions::StepDataBuffer_Initialize_LFSwingDown(double _StepTime, double _DSPTime, double _X_Step, double _Y_Step, double _Yaw_Step)
{
    StepDataBuffer_Clear();

    t_CurSTEP = 0.0;
    CurStepSupportPhase = SUPPORTCONTROL_RSSP;

    SDB[0].Occupied = true;
    SDB[0].NextSupportPhase = SUPPORTCONTROL_LDSP;
    SDB[0].t_STEP = _StepTime;
    SDB[0].X_STEP << _X_Step, _Y_Step, 0.0;
    SDB[0].V_DES = zv;
    SDB[0].X_OFFSET = -0.5*SDB[0].X_STEP;
    SDB[0].Yaw_STEP = 0.0;
    if(_DSPTime == 0.0) {
        SDB[0].DSP_Transition = false;
        SDB[0].t_DSP = 0.0;
    } else {
        SDB[0].DSP_Transition = true;
        SDB[0].t_DSP = _DSPTime;
    }
    SDB[0].FinalStep = true;

    SDB[0].ZMP = SDB[0].X_STEP + RotZ(SDB[0].Yaw_STEP)*SDB[0].X_OFFSET;
}

void LIGHTWholeMotions::StepDataBuffer_Initialize_Walking(Vector3d _Initial_Stance,
                                                          double _StepTime, double _DSPTime,
                                                          double _StanceWidth, double _X_Offset, double _Y_Offset, double _Yaw_Offset,
                                                          double _X_Step, double _Y_Step, double _Yaw_Step)
{
    StepDataBuffer_Clear();

    t_CurSTEP = 0.0;
    CurStepSupportPhase = SUPPORTCONTROL_RDSP;

    SDB[0].Occupied = true;
    SDB[0].NextSupportPhase = SUPPORTCONTROL_LSSP;
    SDB[0].t_STEP = 1.0*_StepTime;
    SDB[0].X_STEP = _Initial_Stance;
    SDB[0].V_DES << 0.0, 0.0, 0.0;
    SDB[0].X_OFFSET << _X_Offset+_X_Step/0.30*0.05, -_Y_Offset, 0.0;
    SDB[0].Yaw_STEP = 0.0;
    SDB[0].DSP_Transition = true;
    SDB[0].t_DSP = SDB[0].t_STEP;
    SDB[0].FinalStep = false;

    SDB[1].Occupied = true;
    SDB[1].NextSupportPhase = SUPPORTCONTROL_RSSP;
    SDB[1].t_STEP = _StepTime;
    if(_Y_Step > 0.0) {
        SDB[1].X_STEP << _X_Step, -_StanceWidth, 0.0;
        SDB[1].V_DES << _X_Step/_StepTime, 0.0, 0.0;
    } else {
        SDB[1].X_STEP << _X_Step, -_StanceWidth+_Y_Step, 0.0;
        SDB[1].V_DES << _X_Step/_StepTime, _Y_Step/_StepTime, 0.0;
    }
    SDB[1].Yaw_STEP =  _Yaw_Step;
    StepInterferenceCheck(1);
    SDB[1].X_OFFSET << _X_Offset+_X_Step/0.30*0.05, _Y_Offset, 0.0;
    if(_DSPTime == 0.0) {
        SDB[1].DSP_Transition = false;
        SDB[1].t_DSP = 0.0;
    } else {
        SDB[1].DSP_Transition = true;
        SDB[1].t_DSP = _DSPTime;
    }
    SDB[1].FinalStep = false;

    SDB[2].Occupied = true;
    SDB[2].NextSupportPhase = SUPPORTCONTROL_LSSP;
    SDB[2].t_STEP = _StepTime;
    if(_Y_Step > 0.0) {
        SDB[2].X_STEP << _X_Step, _StanceWidth+_Y_Step, 0.0;
        SDB[2].V_DES << _X_Step/_StepTime, _Y_Step/_StepTime, 0.0;
    } else {
        SDB[2].X_STEP << _X_Step, _StanceWidth, 0.0;
        SDB[2].V_DES << _X_Step/_StepTime, 0.0, 0.0;
    }
    SDB[2].Yaw_STEP =  _Yaw_Step;
    StepInterferenceCheck(2);
    SDB[2].X_OFFSET << _X_Offset, -_Y_Offset, 0.0;
    if(_DSPTime == 0.0) {
        SDB[2].DSP_Transition = false;
        SDB[2].t_DSP = 0.0;
    } else {
        SDB[2].DSP_Transition = true;
        SDB[2].t_DSP = _DSPTime;
    }
    SDB[2].FinalStep = false;

    SDB[3].Occupied = true;
    SDB[3].NextSupportPhase = SUPPORTCONTROL_RDSP;
    SDB[3].t_STEP = _StepTime;
    SDB[3].X_STEP << 0.0,-_StanceWidth,0.0;
    SDB[3].V_DES = zv;
    SDB[3].Yaw_STEP = 0.0;
    StepInterferenceCheck(3);
    SDB[3].X_OFFSET << _X_Offset,0.0, 0.0;
    SDB[3].X_OFFSET += -0.5*SDB[3].X_STEP;
    if(_DSPTime == 0.0) {
        SDB[3].DSP_Transition = false;
        SDB[3].t_DSP = 0.0;
    } else {
        SDB[3].DSP_Transition = true;
        SDB[3].t_DSP = _DSPTime;
    }
    SDB[3].FinalStep = true;

    SDB[0].ZMP = SDB[0].X_STEP + RotZ(SDB[0].Yaw_STEP)*SDB[0].X_OFFSET;
    SDB[1].ZMP = SDB[0].X_STEP
            + RotZ(SDB[0].Yaw_STEP)*SDB[1].X_STEP
            + RotZ(SDB[0].Yaw_STEP+SDB[1].Yaw_STEP)*SDB[1].X_OFFSET;
    SDB[2].ZMP = SDB[0].X_STEP
            + RotZ(SDB[0].Yaw_STEP)*SDB[1].X_STEP
            + RotZ(SDB[0].Yaw_STEP+SDB[1].Yaw_STEP)*SDB[2].X_STEP
            + RotZ(SDB[0].Yaw_STEP+SDB[1].Yaw_STEP+SDB[2].Yaw_STEP)*SDB[2].X_OFFSET;
    SDB[3].ZMP = SDB[0].X_STEP
            + RotZ(SDB[0].Yaw_STEP)*SDB[1].X_STEP
            + RotZ(SDB[0].Yaw_STEP+SDB[1].Yaw_STEP)*SDB[2].X_STEP
            + RotZ(SDB[0].Yaw_STEP+SDB[1].Yaw_STEP+SDB[2].Yaw_STEP)*SDB[3].X_STEP
            + RotZ(SDB[0].Yaw_STEP+SDB[1].Yaw_STEP+SDB[2].Yaw_STEP+SDB[3].Yaw_STEP)*SDB[3].X_OFFSET;
}

void LIGHTWholeMotions::StepDataBuffer_AddStep(double _StepTime, double _DSPTime, double _StanceWidth, double _X_Offset, double _Y_Offset, double _Yaw_Offset,
                                               double _X_Step, double _Y_Step, double _Yaw_Step)
{
    if(SDB[0].NextSupportPhase == SUPPORTCONTROL_RSSP) {

        // Modify intermid step
        SDB[1].Occupied = true;
        SDB[1].NextSupportPhase = SUPPORTCONTROL_LSSP;
        SDB[1].t_STEP = _StepTime;
        if(_Y_Step > 0.0) {
            SDB[1].X_STEP << _X_Step, _StanceWidth+_Y_Step, 0.0;
            SDB[1].V_DES << _X_Step/_StepTime, _Y_Step/_StepTime, 0.0;
        } else {
            SDB[1].X_STEP << _X_Step, _StanceWidth, 0.0;
            SDB[1].V_DES << _X_Step/_StepTime, 0.0, 0.0;
        }
        SDB[1].Yaw_STEP =  _Yaw_Step;
        StepInterferenceCheck(1);
        SDB[1].X_OFFSET << _X_Offset+_X_Step/0.30*0.05, -_Y_Offset, 0.0;
        if(_DSPTime == 0.0) {
            SDB[1].DSP_Transition = false;
            SDB[1].t_DSP = 0.0;
        } else {
            SDB[1].DSP_Transition = true;
            SDB[1].t_DSP = _DSPTime;
        }
        SDB[1].FinalStep = false;

        SDB[2].Occupied = true;
        SDB[2].NextSupportPhase = SUPPORTCONTROL_RSSP;
        SDB[2].t_STEP = _StepTime;
        if(_Y_Step > 0.0) {
            SDB[2].X_STEP << _X_Step, -_StanceWidth, 0.0;
            SDB[2].V_DES << _X_Step/_StepTime, 0.0, 0.0;
        } else {
            SDB[2].X_STEP << _X_Step, -_StanceWidth+_Y_Step, 0.0;
            SDB[2].V_DES << _X_Step/_StepTime, _Y_Step/_StepTime, 0.0;
        }
        SDB[2].Yaw_STEP =  _Yaw_Step;
        StepInterferenceCheck(2);
        SDB[2].X_OFFSET << _X_Offset, _Y_Offset, 0.0;
        if(_DSPTime == 0.0) {
            SDB[2].DSP_Transition = false;
            SDB[2].t_DSP = 0.0;
        } else {
            SDB[2].DSP_Transition = true;
            SDB[2].t_DSP = _DSPTime;
        }
        SDB[2].FinalStep = false;

        // Add a Last Step
        SDB[3].Occupied = true;
        SDB[3].NextSupportPhase = SUPPORTCONTROL_LDSP;
        SDB[3].t_STEP = _StepTime;
        SDB[3].X_STEP << 0.0,_StanceWidth,0.0;
        SDB[3].V_DES = zv;
        SDB[3].X_OFFSET << _X_Offset,0.0, 0.0;
        SDB[3].Yaw_STEP = 0.0;
        StepInterferenceCheck(3);
        SDB[3].X_OFFSET += -0.5*SDB[3].X_STEP;
        if(_DSPTime == 0.0) {
            SDB[3].DSP_Transition = false;
            SDB[3].t_DSP = 0.0;
        } else {
            SDB[3].DSP_Transition = true;
            SDB[3].t_DSP = _DSPTime;
        }
        SDB[3].FinalStep = true;

        SDB[0].ZMP = SDB[0].X_STEP + RotZ(SDB[0].Yaw_STEP)*SDB[0].X_OFFSET;
        SDB[1].ZMP = SDB[0].X_STEP
                + RotZ(SDB[0].Yaw_STEP)*SDB[1].X_STEP
                + RotZ(SDB[0].Yaw_STEP+SDB[1].Yaw_STEP)*SDB[1].X_OFFSET;
        SDB[2].ZMP = SDB[0].X_STEP
                + RotZ(SDB[0].Yaw_STEP)*SDB[1].X_STEP
                + RotZ(SDB[0].Yaw_STEP+SDB[1].Yaw_STEP)*SDB[2].X_STEP
                + RotZ(SDB[0].Yaw_STEP+SDB[1].Yaw_STEP+SDB[2].Yaw_STEP)*SDB[2].X_OFFSET;
        SDB[3].ZMP = SDB[0].X_STEP
                + RotZ(SDB[0].Yaw_STEP)*SDB[1].X_STEP
                + RotZ(SDB[0].Yaw_STEP+SDB[1].Yaw_STEP)*SDB[2].X_STEP
                + RotZ(SDB[0].Yaw_STEP+SDB[1].Yaw_STEP+SDB[2].Yaw_STEP)*SDB[3].X_STEP
                + RotZ(SDB[0].Yaw_STEP+SDB[1].Yaw_STEP+SDB[2].Yaw_STEP+SDB[3].Yaw_STEP)*SDB[3].X_OFFSET;


    } else if (SDB[0].NextSupportPhase == SUPPORTCONTROL_LSSP) {

        // Modify intermid step
        SDB[1].Occupied = true;
        SDB[1].NextSupportPhase = SUPPORTCONTROL_RSSP;
        SDB[1].t_STEP = _StepTime;
        if(_Y_Step > 0.0) {
            SDB[1].X_STEP << _X_Step, -_StanceWidth, 0.0;
            SDB[1].V_DES << _X_Step/_StepTime, 0.0, 0.0;
        } else {
            SDB[1].X_STEP << _X_Step, -_StanceWidth+_Y_Step, 0.0;
            SDB[1].V_DES << _X_Step/_StepTime, _Y_Step/_StepTime, 0.0;
        }
        SDB[1].Yaw_STEP =  _Yaw_Step;
        StepInterferenceCheck(1);
        SDB[1].X_OFFSET << _X_Offset+_X_Step/0.30*0.05, _Y_Offset, 0.0;
        if(_DSPTime == 0.0) {
            SDB[1].DSP_Transition = false;
            SDB[1].t_DSP = 0.0;
        } else {
            SDB[1].DSP_Transition = true;
            SDB[1].t_DSP = _DSPTime;
        }
        SDB[1].FinalStep = false;

        SDB[2].Occupied = true;
        SDB[2].NextSupportPhase = SUPPORTCONTROL_LSSP;
        SDB[2].t_STEP = _StepTime;
        if(_Y_Step > 0.0) {
            SDB[2].X_STEP << _X_Step, _StanceWidth+_Y_Step, 0.0;
            SDB[2].V_DES << _X_Step/_StepTime, _Y_Step/_StepTime, 0.0;
        } else {
            SDB[2].X_STEP << _X_Step, _StanceWidth, 0.0;
            SDB[2].V_DES << _X_Step/_StepTime, 0.0, 0.0;
        }
        SDB[2].Yaw_STEP =  _Yaw_Step;
        StepInterferenceCheck(2);
        SDB[2].X_OFFSET << _X_Offset, -_Y_Offset, 0.0;
        if(_DSPTime == 0.0) {
            SDB[2].DSP_Transition = false;
            SDB[2].t_DSP = 0.0;
        } else {
            SDB[2].DSP_Transition = true;
            SDB[2].t_DSP = _DSPTime;
        }
        SDB[2].FinalStep = false;

        // Add a Last Step
        SDB[3].Occupied = true;
        SDB[3].NextSupportPhase = SUPPORTCONTROL_RDSP;
        SDB[3].t_STEP = _StepTime;
        SDB[3].X_STEP << 0.0,-_StanceWidth,0.0;
        SDB[3].V_DES = zv;
        SDB[3].Yaw_STEP = 0.0;
        StepInterferenceCheck(3);
        SDB[3].X_OFFSET << _X_Offset,0.0, 0.0;
        SDB[3].X_OFFSET += -0.5*SDB[3].X_STEP;
        if(_DSPTime == 0.0) {
            SDB[3].DSP_Transition = false;
            SDB[3].t_DSP = 0.0;
        } else {
            SDB[3].DSP_Transition = true;
            SDB[3].t_DSP = _DSPTime;
        }
        SDB[3].FinalStep = true;

        SDB[0].ZMP = SDB[0].X_STEP + RotZ(SDB[0].Yaw_STEP)*SDB[0].X_OFFSET;
        SDB[1].ZMP = SDB[0].X_STEP
                + RotZ(SDB[0].Yaw_STEP)*SDB[1].X_STEP
                + RotZ(SDB[0].Yaw_STEP+SDB[1].Yaw_STEP)*SDB[1].X_OFFSET;
        SDB[2].ZMP = SDB[0].X_STEP
                + RotZ(SDB[0].Yaw_STEP)*SDB[1].X_STEP
                + RotZ(SDB[0].Yaw_STEP+SDB[1].Yaw_STEP)*SDB[2].X_STEP
                + RotZ(SDB[0].Yaw_STEP+SDB[1].Yaw_STEP+SDB[2].Yaw_STEP)*SDB[2].X_OFFSET;
        SDB[3].ZMP = SDB[0].X_STEP
                + RotZ(SDB[0].Yaw_STEP)*SDB[1].X_STEP
                + RotZ(SDB[0].Yaw_STEP+SDB[1].Yaw_STEP)*SDB[2].X_STEP
                + RotZ(SDB[0].Yaw_STEP+SDB[1].Yaw_STEP+SDB[2].Yaw_STEP)*SDB[3].X_STEP
                + RotZ(SDB[0].Yaw_STEP+SDB[1].Yaw_STEP+SDB[2].Yaw_STEP+SDB[3].Yaw_STEP)*SDB[3].X_OFFSET;

    } else {
        FILE_LOG(logERROR) << "[Error] Support Phase Setting Error!";
    }
}

void LIGHTWholeMotions::StepDataBuffer_PushOutStep()
{
    int n_finalstep = GetFinalStep();
    t_CurSTEP = 0.0;
    CurStepSupportPhase = SDB[0].NextSupportPhase;
    CurFlagDSP = false;

    if(n_finalstep == 0) {
        ZMP_forStepOver = SDB[0].X_OFFSET;
    } else {
        for(int i=0;i<n_finalstep;i++) {
            SDB[i] = SDB[i+1];
        }
        if(n_finalstep == 1) {
            SDB[0].ZMP = SDB[0].X_STEP + RotZ(SDB[0].Yaw_STEP)*SDB[0].X_OFFSET;
        } else if(n_finalstep == 2) {
            SDB[0].ZMP = SDB[0].X_STEP + RotZ(SDB[0].Yaw_STEP)*SDB[0].X_OFFSET;
            SDB[1].ZMP = SDB[0].X_STEP
                    + RotZ(SDB[0].Yaw_STEP)*SDB[1].X_STEP
                    + RotZ(SDB[0].Yaw_STEP+SDB[1].Yaw_STEP)*SDB[1].X_OFFSET;
        } else if(n_finalstep == 3) {
            SDB[0].ZMP = SDB[0].X_STEP + RotZ(SDB[0].Yaw_STEP)*SDB[0].X_OFFSET;
            SDB[1].ZMP = SDB[0].X_STEP
                    + RotZ(SDB[0].Yaw_STEP)*SDB[1].X_STEP
                    + RotZ(SDB[0].Yaw_STEP+SDB[1].Yaw_STEP)*SDB[1].X_OFFSET;
            SDB[2].ZMP = SDB[0].X_STEP
                    + RotZ(SDB[0].Yaw_STEP)*SDB[1].X_STEP
                    + RotZ(SDB[0].Yaw_STEP+SDB[1].Yaw_STEP)*SDB[2].X_STEP
                    + RotZ(SDB[0].Yaw_STEP+SDB[1].Yaw_STEP+SDB[2].Yaw_STEP)*SDB[2].X_OFFSET;
        }
    }
    StepDataBuffer_Clear(n_finalstep);
}


bool LIGHTWholeMotions::StepDataBuffer_Update()
{
    t_CurSTEP = t_CurSTEP + SYS_DT;

    switch(CurStepSupportPhase) {
    case SUPPORTCONTROL_RDSP:
    {
        if(SDB[0].NextSupportPhase == SUPPORTCONTROL_RSSP) {
            return StepDataBuffer_Update_RDSP2RSSP();
        } else if(SDB[0].NextSupportPhase == SUPPORTCONTROL_LDSP) {
            return StepDataBuffer_Update_RDSP2LDSP();
        } else if(SDB[0].NextSupportPhase == SUPPORTCONTROL_LSSP) {
            return StepDataBuffer_Update_RDSP2LSSP();
        }
        break;
    }
    case SUPPORTCONTROL_LDSP:
    {
        if(SDB[0].NextSupportPhase == SUPPORTCONTROL_LSSP) {
            return StepDataBuffer_Update_LDSP2LSSP();
        } else if(SDB[0].NextSupportPhase == SUPPORTCONTROL_RDSP) {
            return StepDataBuffer_Update_LDSP2RDSP();
        } else if(SDB[0].NextSupportPhase == SUPPORTCONTROL_RSSP) {
            return StepDataBuffer_Update_LDSP2RSSP();
        }
        break;
    }
    case SUPPORTCONTROL_RSSP:
    {
        if(SDB[0].NextSupportPhase == SUPPORTCONTROL_RDSP) {
            return StepDataBuffer_Update_RSSP2RDSP();
        } else if(SDB[0].NextSupportPhase == SUPPORTCONTROL_LDSP) {
            return StepDataBuffer_Update_RSSP2LDSP();
        } else if(SDB[0].NextSupportPhase == SUPPORTCONTROL_LSSP) {
            return StepDataBuffer_Update_RSSP2LSSP();
        }
        break;
    }
    case SUPPORTCONTROL_LSSP:
    {
        if(SDB[0].NextSupportPhase == SUPPORTCONTROL_LDSP) {
            return StepDataBuffer_Update_LSSP2LDSP();
        } else if(SDB[0].NextSupportPhase == SUPPORTCONTROL_RDSP) {
            return StepDataBuffer_Update_LSSP2RDSP();
        } else if(SDB[0].NextSupportPhase == SUPPORTCONTROL_RSSP) {
            return StepDataBuffer_Update_LSSP2RSSP();
        }
        break;
    }
    default:
        break;
    }

    return false;
}

bool LIGHTWholeMotions::StepDataBuffer_Update_RDSP2RSSP()
{
    FILE_LOG(logERROR) << "[StepDataBuffer Error] RDSP to RSSP is not defined!";
    return false;
}

bool LIGHTWholeMotions::StepDataBuffer_Update_RDSP2LDSP()
{
    if(!SDB[0].DSP_Transition) {
        FILE_LOG(logERROR) << "[StepDataBuffer Error] DSP_Transition Flag is not set!";
        return false;
    }

    int n_finalstep = GetFinalStep();
    if(n_finalstep == 0) {
        if(t_CurSTEP >= SDB[0].t_DSP) {
            StepDataBuffer_PushOutStep();
            return true;
        }
    } else if (n_finalstep > 0) {
        FILE_LOG(logERROR) << "[StepDataBuffer Error] This step must be final step!";
    }
    return false;
}

bool LIGHTWholeMotions::StepDataBuffer_Update_RDSP2LSSP()
{
    if(!SDB[0].DSP_Transition) {
        FILE_LOG(logERROR) << "[StepDataBuffer Error] DSP_Transition Flag is not set!";
        return false;
    }

    int n_finalstep = GetFinalStep();
    if(n_finalstep == 0) {
        if(t_CurSTEP >= SDB[0].t_DSP) {
            StepDataBuffer_PushOutStep();
            return true;
        }
    } else if (n_finalstep > 0) {
        if(t_CurSTEP >= SDB[0].t_DSP) {
            StepDataBuffer_PushOutStep();
            return true;
        }
    }
    return false;
}


bool LIGHTWholeMotions::StepDataBuffer_Update_LDSP2LSSP()
{
    FILE_LOG(logERROR) << "[StepDataBuffer Error] LDSP to LSSP is not defined!";
    return false;
}

bool LIGHTWholeMotions::StepDataBuffer_Update_LDSP2RDSP()
{
    if(!SDB[0].DSP_Transition) {
        FILE_LOG(logERROR) << "[StepDataBuffer Error] DSP_Transition Flag is not set!";
        return false;
    }

    int n_finalstep = GetFinalStep();
    if(n_finalstep == 0) {
        if(t_CurSTEP >= SDB[0].t_DSP) {
            StepDataBuffer_PushOutStep();
            return true;
        }
    } else if (n_finalstep > 0) {
        FILE_LOG(logERROR) << "[StepDataBuffer Error] This step must be final step!";
    }
    return false;
}

bool LIGHTWholeMotions::StepDataBuffer_Update_LDSP2RSSP()
{
    if(!SDB[0].DSP_Transition) {
        FILE_LOG(logERROR) << "[StepDataBuffer Error] DSP_Transition Flag is not set!";
        return false;
    }

    int n_finalstep = GetFinalStep();
    if(n_finalstep == 0) {
        if(t_CurSTEP >= SDB[0].t_DSP) {
            StepDataBuffer_PushOutStep();
            return true;
        }
    } else if (n_finalstep > 0) {
        if(t_CurSTEP >= SDB[0].t_DSP) {
            StepDataBuffer_PushOutStep();
            return true;
        }
    }
    return false;
}

bool LIGHTWholeMotions::StepDataBuffer_Update_RSSP2RDSP()
{
    FILE_LOG(logERROR) << "[StepDataBuffer Error] RSSP to RDSP is not defined!";
    return false;
}

bool LIGHTWholeMotions::StepDataBuffer_Update_RSSP2LDSP()
{
    int n_finalstep = GetFinalStep();
    if(n_finalstep == 0) {
        if(SDB[0].DSP_Transition == false) {
            if(t_CurSTEP >= SDB[0].t_STEP) {
                StepDataBuffer_PushOutStep();
                return true;
            }
        } else {
            double t_SWING = SDB[0].t_STEP - SDB[0].t_DSP;
            if(t_CurSTEP >= t_SWING && CurFlagDSP==false) {
                CurFlagDSP = true;
                return true;
            } else if(t_CurSTEP >= SDB[0].t_STEP) {
                StepDataBuffer_PushOutStep();
                return true;
            }
        }
    } else if (n_finalstep > 0) {
        FILE_LOG(logERROR) << "[StepDataBuffer Error] This step must be final step!";
    }
    return false;
}

bool LIGHTWholeMotions::StepDataBuffer_Update_RSSP2LSSP()
{
    if(SDB[0].DSP_Transition == false) {
        if(t_CurSTEP >= SDB[0].t_STEP) {
            StepDataBuffer_PushOutStep();
            return true;
        }
    } else {
        double t_SWING = SDB[0].t_STEP - SDB[0].t_DSP;
        if(t_CurSTEP >= t_SWING && CurFlagDSP==false) {
            CurFlagDSP = true;
            return true;
        } else if(t_CurSTEP >= SDB[0].t_STEP) {
            StepDataBuffer_PushOutStep();
            return true;
        }
    }
    return false;
}

bool LIGHTWholeMotions::StepDataBuffer_Update_LSSP2LDSP()
{
    FILE_LOG(logERROR) << "[StepDataBuffer Error] LSSP to LDSP is not defined!";
    return false;
}

bool LIGHTWholeMotions::StepDataBuffer_Update_LSSP2RDSP()
{
    int n_finalstep = GetFinalStep();
    if(n_finalstep == 0) {
        if(SDB[0].DSP_Transition == false) {
            if(t_CurSTEP >= SDB[0].t_STEP) {
                StepDataBuffer_PushOutStep();
                return true;
            }
        } else {
            double t_SWING = SDB[0].t_STEP - SDB[0].t_DSP;
            if(t_CurSTEP >= t_SWING && CurFlagDSP==false) {
                CurFlagDSP = true;
                return true;
            } else if(t_CurSTEP >= SDB[0].t_STEP) {
                StepDataBuffer_PushOutStep();
                return true;
            }
        }
    } else if (n_finalstep > 0) {
        FILE_LOG(logERROR) << "[StepDataBuffer Error] This step must be final step!";
    }
    return false;
}

bool LIGHTWholeMotions::StepDataBuffer_Update_LSSP2RSSP()
{
    if(SDB[0].DSP_Transition == false) {
        if(t_CurSTEP >= SDB[0].t_STEP) {
            StepDataBuffer_PushOutStep();
            return true;
        }
    } else {
        double t_SWING = SDB[0].t_STEP - SDB[0].t_DSP;
        if(t_CurSTEP >= t_SWING && CurFlagDSP==false) {
            CurFlagDSP = true;
            return true;
        } else if(t_CurSTEP >= SDB[0].t_STEP) {
            StepDataBuffer_PushOutStep();
            return true;
        }
    }
    return false;
}

void LIGHTWholeMotions::GetFutureStance(double _t,
                                        int& StanceLeg, Vector3d& StancePosition, double& StanceZAngle,
                                        bool& StepOn, Vector3d& StepPosition, double& StepZAngle)
{
    // This function is to get support polygon.
    int n = GetFinalStep();
    double t_Prev = _t + t_CurSTEP;

    if(n==-1) {
        if(CurStepSupportPhase == SUPPORTCONTROL_RDSP) {
            StanceLeg = RIGHTLEG;
            StancePosition = zv;
            StanceZAngle = 0.0;
            StepOn = true;
            StepPosition = LIGHT.Xref_RF2LF;
            StepZAngle = 0.0;
        } else if(CurStepSupportPhase == SUPPORTCONTROL_LDSP) {
            StanceLeg = LEFTLEG;
            StancePosition = zv;
            StanceZAngle = 0.0;
            StepOn = true;
            StepPosition = LIGHT.Xref_LF2RF;
            StepZAngle = 0.0;
        } else if(CurStepSupportPhase == SUPPORTCONTROL_RSSP) {
            StanceLeg = RIGHTLEG;
            StancePosition = zv;
            StanceZAngle = 0.0;
            StepOn = false;
            StepPosition = zv;
            StepZAngle = 0.0;
        } else if(CurStepSupportPhase == SUPPORTCONTROL_LSSP) {
            StanceLeg = LEFTLEG;
            StancePosition = zv;
            StanceZAngle = 0.0;
            StepOn = false;
            StepPosition = zv;
            StepZAngle = 0.0;
        }
        return;
    } else if (t_Prev <= SDB[0].t_STEP) {
        if(CurStepSupportPhase == SUPPORTCONTROL_RDSP) {
            StanceLeg = RIGHTLEG;
            StancePosition = zv;
            StanceZAngle = 0.0;
            StepOn = true;
            StepPosition = SDB[0].X_STEP + SDB[0].X_STEP_adj;
            StepZAngle = SDB[0].Yaw_STEP;
        } else if(CurStepSupportPhase == SUPPORTCONTROL_LDSP) {
            StanceLeg = LEFTLEG;
            StancePosition = zv;
            StanceZAngle = 0.0;
            StepOn = true;
            StepPosition = SDB[0].X_STEP + SDB[0].X_STEP_adj;
            StepZAngle = SDB[0].Yaw_STEP;
        } else if(CurStepSupportPhase == SUPPORTCONTROL_RSSP) {
            StanceLeg = RIGHTLEG;
            StancePosition = zv;
            StanceZAngle = 0.0;
            if(t_Prev >= (SDB[0].t_STEP-SDB[0].t_DSP)) {
                StepOn = true;
                StepPosition = SDB[0].X_STEP + SDB[0].X_STEP_adj;
                StepZAngle = SDB[0].Yaw_STEP;
            } else {
                StepOn = false;
                StepPosition = zv;
                StepZAngle = 0.0;
            }
        } else if(CurStepSupportPhase == SUPPORTCONTROL_LSSP) {
            StanceLeg = LEFTLEG;
            StancePosition = zv;
            StanceZAngle = 0.0;
            if(t_Prev >= (SDB[0].t_STEP-SDB[0].t_DSP)) {
                StepOn = true;
                StepPosition = SDB[0].X_STEP + SDB[0].X_STEP_adj;
                StepZAngle = SDB[0].Yaw_STEP;
            } else {
                StepOn = false;
                StepPosition = zv;
                StepZAngle = 0.0;
            }
        }
        return;
    } else {
        t_Prev = t_Prev - SDB[0].t_STEP;
        Vector3d _X_STANCE = SDB[0].X_STEP;
        double _Yaw_STANCE = SDB[0].Yaw_STEP;

        for(int i=1;i<=n;i++) {
            if(t_Prev <= SDB[i].t_STEP) {
                if(SDB[i-1].NextSupportPhase == SUPPORTCONTROL_RDSP) {
                    StanceLeg = RIGHTLEG;
                    StancePosition = _X_STANCE;
                    StanceZAngle = _Yaw_STANCE;
                    StepOn = true;
                    StepPosition = SDB[i].X_STEP + SDB[i].X_STEP_adj;
                    StepZAngle = SDB[i].Yaw_STEP;
                } else if(SDB[i-1].NextSupportPhase == SUPPORTCONTROL_LDSP) {
                    StanceLeg = LEFTLEG;
                    StancePosition = _X_STANCE;
                    StanceZAngle = _Yaw_STANCE;
                    StepOn = true;
                    StepPosition = SDB[i].X_STEP + SDB[i].X_STEP_adj;
                    StepZAngle = SDB[i].Yaw_STEP;
                } else if(SDB[i-1].NextSupportPhase == SUPPORTCONTROL_RSSP) {
                    StanceLeg = RIGHTLEG;
                    StancePosition = _X_STANCE;
                    StanceZAngle = _Yaw_STANCE;
                    if(t_Prev >= (SDB[i].t_STEP-SDB[i].t_DSP)) {
                        StepOn = true;
                        StepPosition = SDB[i].X_STEP + SDB[i].X_STEP_adj;
                        StepZAngle = SDB[i].Yaw_STEP;
                    } else {
                        StepOn = false;
                        StepPosition = zv;
                        StepZAngle = 0.0;
                    }
                } else if(SDB[i-1].NextSupportPhase == SUPPORTCONTROL_LSSP) {
                    StanceLeg = LEFTLEG;
                    StancePosition = _X_STANCE;
                    StanceZAngle = _Yaw_STANCE;
                    if(t_Prev >= (SDB[i].t_STEP-SDB[i].t_DSP)) {
                        StepOn = true;
                        StepPosition = SDB[i].X_STEP + SDB[i].X_STEP_adj;
                        StepZAngle = SDB[i].Yaw_STEP;
                    } else {
                        StepOn = false;
                        StepPosition = zv;
                        StepZAngle = 0.0;
                    }
                }
                return;
            } else {
                t_Prev = t_Prev - SDB[i].t_STEP;
                _X_STANCE = _X_STANCE + RotZ(_Yaw_STANCE)*(SDB[i].X_STEP + SDB[i].X_STEP_adj);
                _Yaw_STANCE = _Yaw_STANCE + SDB[i].Yaw_STEP;
            }
        }

        if(SDB[n].NextSupportPhase == SUPPORTCONTROL_RDSP) {
            StanceLeg = RIGHTLEG;
            StancePosition = _X_STANCE;
            StanceZAngle = _Yaw_STANCE;
            StepOn = true;
            StepPosition = -(SDB[n].X_STEP + SDB[n].X_STEP_adj);
            StepZAngle = -SDB[n].Yaw_STEP;
        } else if(SDB[n].NextSupportPhase == SUPPORTCONTROL_LDSP) {
            StanceLeg = LEFTLEG;
            StancePosition = _X_STANCE;
            StanceZAngle = _Yaw_STANCE;
            StepOn = true;
            StepPosition = -(SDB[n].X_STEP + SDB[n].X_STEP_adj);
            StepZAngle = -SDB[n].Yaw_STEP;
        } else if(SDB[n].NextSupportPhase == SUPPORTCONTROL_RSSP) {
            StanceLeg = RIGHTLEG;
            StancePosition = _X_STANCE;
            StanceZAngle = _Yaw_STANCE;
            if(t_Prev >= (SDB[n].t_STEP-SDB[n].t_DSP)) {
                StepOn = true;
                StepPosition = -(SDB[n].X_STEP + SDB[n].X_STEP_adj);
                StepZAngle = -SDB[n].Yaw_STEP;
            } else {
                StepOn = false;
                StepPosition = zv;
                StepZAngle = 0.0;
            }
        } else if(SDB[n].NextSupportPhase == SUPPORTCONTROL_LSSP) {
            StanceLeg = LEFTLEG;
            StancePosition = _X_STANCE;
            StanceZAngle = _Yaw_STANCE;
            if(t_Prev >= (SDB[n].t_STEP-SDB[n].t_DSP)) {
                StepOn = true;
                StepPosition = -(SDB[n].X_STEP + SDB[n].X_STEP_adj);
                StepZAngle = -SDB[n].Yaw_STEP;
            } else {
                StepOn = false;
                StepPosition = zv;
                StepZAngle = 0.0;
            }
        }
        return;
    }
}

Vector3d LIGHTWholeMotions::GetFutureVelocityRef(double _t)
{
    Vector3d _V;

    int n = GetFinalStep();
    double t_Prev = _t + t_CurSTEP;

    if(n==-1) {
        _V = zv;
        return _V;
    } else {
        for(int i=0;i<=n;i++) {
            if(t_Prev <= SDB[i].t_STEP) {
                _V = SDB[i].V_DES;
                return _V;
            } else {
                t_Prev = t_Prev - SDB[i].t_STEP;
            }
        }
        _V = zv;
        return _V;
    }
}


Vector3d LIGHTWholeMotions::GetFutureStepPosition(double _t)
{
    Vector3d _X = zv;

    int n = GetFinalStep();
    double t_Prev = _t + t_CurSTEP;

    if(n==-1) {
        return _X;
    } else {
        for(int i=0;i<=n;i++) {
            if(t_Prev <= SDB[i].t_STEP) {
                return SDB[i].ZMP;
            } else {
                t_Prev = t_Prev - SDB[i].t_STEP;
            }
        }
        return SDB[n].ZMP;
    }
}

// Future Reference Generator ==========================================================================

// Input : Current Capture point
//         Future Step Info(Step point, Step Time, Stance Type, etc...)
//         Number of Preview Samples
//         dT for Preview
// Output : Future Capture Reference for horizon time
void LIGHTWholeMotions::Generate_CPandZMP_BasicPattern(int _N_Prev, double _dT_Prev, Vector3d* _CPref_horizon, Vector3d* _ZMPref_horizon)
{
    int n_finalstep = GetFinalStep();
    if(n_finalstep == -1) { // Step Data Buffer is void
        _CPref_horizon[0] = ZMP_forStepOver;
        for(int i=0;i<_N_Prev;i++) {
            _ZMPref_horizon[i] = ZMP_forStepOver;
            _CPref_horizon[i+1] = ZMP_forStepOver;
            Xref_CP = ZMP_forStepOver;
        }
    } else {
        // Generate End of Step (EoS) Capture Point and Start of Step (SoS) Capture Point ====================
        // CP way points is derived from future steps (Backward)
        Vector3d CP_EoS[PREVIEW_STEPS]; // Capture Point Position @ End of Step
        Vector3d CP_SoS[PREVIEW_STEPS]; // Capture Point Position @ Start of Step
        if(n_finalstep == 0) {
            CP_EoS[0] = SDB[0].ZMP;
            CP_SoS[0] = Xref_CP;
        } else {
            CP_EoS[n_finalstep] = SDB[n_finalstep].ZMP;
            CP_SoS[n_finalstep] = SDB[n_finalstep-1].ZMP + exp(-wn_LIPM*SDB[n_finalstep].t_STEP)*I3*(CP_EoS[n_finalstep]-SDB[n_finalstep-1].ZMP);
            for(int i=n_finalstep-1;i>=1;i--) {
                CP_EoS[i] = CP_SoS[i+1];
                CP_SoS[i] = SDB[i-1].ZMP + exp(-wn_LIPM*SDB[i].t_STEP)*I3*(CP_EoS[i]-SDB[i-1].ZMP);
            }
            CP_EoS[0] = CP_SoS[1];
            CP_SoS[0] = Xref_CP; // Current Reference Capture Point
        }

        for(int i=0;i<PREVIEW_STEPS;i++){
            CP_EoS[i](2) = Pelvis_BaseHeight;
            CP_SoS[i](2) = Pelvis_BaseHeight;
        }

        // CP trajectory generation based on CP way points ===================================================
        int idx_CheckStep = 0;
        bool Flag_StepOver = false;

        _CPref_horizon[0] = Xref_CP;

        double _t_Prev = t_CurSTEP;
        for(int i=0;i<_N_Prev;i++) {
            _t_Prev += _dT_Prev;
            if(!Flag_StepOver) {
                if(idx_CheckStep == 0) { // Check First Step
                    double _T = SDB[0].t_STEP; // Remaining Step Time of Current Step
                    if(_t_Prev <= _T) {
                        _ZMPref_horizon[i] = (exp(wn_LIPM*(_T-t_CurSTEP))*I3*CP_SoS[0]-CP_EoS[0])/(exp(wn_LIPM*(_T-t_CurSTEP))-1.0);
                        _CPref_horizon[i+1] = _ZMPref_horizon[i] + exp(wn_LIPM*(_t_Prev-t_CurSTEP))*I3*(CP_SoS[0]-_ZMPref_horizon[i]);
                    } else if ((_t_Prev > _T) && (n_finalstep != 0)) {
                        _t_Prev = _t_Prev - _T;
                        _ZMPref_horizon[i] = SDB[0].ZMP;
                        _CPref_horizon[i+1] = _ZMPref_horizon[i] + exp(wn_LIPM*_t_Prev)*I3*(CP_SoS[1]-_ZMPref_horizon[i]);
                        idx_CheckStep = 1;
                    } else if ((_t_Prev > _T)&&(n_finalstep == 0)) {
                        Flag_StepOver = true;
                        _ZMPref_horizon[i] = CP_EoS[n_finalstep];
                        _CPref_horizon[i+1] = CP_EoS[n_finalstep];
                    }
                } else if(_t_Prev <= SDB[idx_CheckStep].t_STEP) {
                    _ZMPref_horizon[i] = _ZMPref_horizon[i-1];
                    _CPref_horizon[i+1] = _ZMPref_horizon[i] + exp(wn_LIPM*_t_Prev)*I3*(CP_SoS[idx_CheckStep]-_ZMPref_horizon[i]);
                } else if((_t_Prev > SDB[idx_CheckStep].t_STEP)&&(idx_CheckStep != n_finalstep)) {
                    _t_Prev = _t_Prev - SDB[idx_CheckStep].t_STEP;
                    _ZMPref_horizon[i] = SDB[idx_CheckStep].ZMP;
                    _CPref_horizon[i+1] = _ZMPref_horizon[i] + exp(wn_LIPM*_t_Prev)*I3*(CP_SoS[idx_CheckStep+1]-_ZMPref_horizon[i]);
                    idx_CheckStep++;
                } else if ((_t_Prev > SDB[idx_CheckStep].t_STEP)&&(idx_CheckStep == n_finalstep)) {
                    Flag_StepOver = true;
                    _ZMPref_horizon[i] = CP_EoS[n_finalstep];
                    _CPref_horizon[i+1] = CP_EoS[n_finalstep];
                } else {
                    FILE_LOG(logINFO) << "??";
                }
            } else {
                _ZMPref_horizon[i] = CP_EoS[n_finalstep];
                _CPref_horizon[i+1] = CP_EoS[n_finalstep];
            }
            
            _ZMPref_horizon[i](2) = Pelvis_BaseHeight;
            _CPref_horizon[i+1](2) = Pelvis_BaseHeight;
            
            //            FILE_LOG(logINFO) << _t_Prev;
            //            FILE_LOG(logINFO) << "CP : " << _CPref_horizon[i].transpose();
            //            FILE_LOG(logINFO) << "ZMP : " << _ZMPref_horizon[i].transpose();
        }

        // Next Step CP and ZMP reference ===================================================================
        double _t_Next = t_CurSTEP + SYS_DT;
        if(_t_Next<=SDB[0].t_STEP) {
            double _T = SDB[0].t_STEP;
            Vector3d _ZMPnext = (exp(wn_LIPM*(_T-_t_Next))*I3*CP_SoS[0]-CP_EoS[0])/(exp(wn_LIPM*(_T-_t_Next))-1.0);
            Xref_CP = _ZMPnext + exp(wn_LIPM*SYS_DT)*I3*(CP_SoS[0]-_ZMPnext);
        } else {
            if(n_finalstep > 0) {
                _t_Prev = SYS_DT - (SDB[0].t_STEP-t_CurSTEP);
                Vector3d _ZMPnext = SDB[0].X_STEP + SDB[0].X_OFFSET;
                Xref_CP = _ZMPnext + exp(wn_LIPM*_t_Prev)*I3*(CP_SoS[1]-_ZMPnext);
            } else if (n_finalstep == 0) {
                Xref_CP = CP_EoS[0];
            }
        }
    }

    //    for(int i=0;i<_N_Prev;i++) {
    //        LIGHT.SavingVector[15+i] = _CPref_horizon[i];
    //    }
}

bool LIGHTWholeMotions::Generate_CPref_withMPC(int _N_Prev, double _dT_Prev,
                                               Vector3d* _CPref_horizon, Vector3d* _ZMPref_horizon, Vector3d* _CoMref_horizon)
{
    // Initial States
    VectorNd _xnow = Xref_CP.segment(0,2);

    int N = _N_Prev;
    double dT = _dT_Prev;
    VectorNd CP_MPC = VectorNd::Zero(2*N,1); // Capture Point Future Reference
    VectorNd ZMP_MPC = VectorNd::Zero(2*N,1); // ZMP Future Reference
    VectorNd CoM_MPC = VectorNd::Zero(2*N,1); // CoM Future Reference

    ////////////////////////////////////////////////////////////////////////////////////
    //// QP Formulation ////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////
    {
        LIGHT_QP QP_CPref(SolverIsQPSwift);
        MatrixNd A_temp;
        VectorNd B_temp;

        // state variable : [Xx_MPC(2*N); Ux_MPC(2*N)]
        const int size_CP = 2*N;
        const int size_ZMP = 2*N;
        const int idx_CP = 0;
        const int idx_ZMP = size_CP;
        int n_var   = size_CP+size_ZMP;

        //////// Cost Functions /////////////////////////////////////////////////////////////////////////
        /// 1)
        /////////////////////////////////////////////////////////////////////////////////////////////////

        int n_cost = 0;
        int idx_cost = 0;
        int temp_size_cost = 0;
        MatrixNd A_cost;
        VectorNd B_cost;
        double W_temp = 0.0;

        Generate_CPandZMP_BasicPattern(N, dT, _CPref_horizon, _ZMPref_horizon);

        // 1-1. Capture Point Tracking (X-direction)
        W_temp = 1.0;
        temp_size_cost = N;
        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
        for(int i=0;i<N;i++) {
            A_temp(i,idx_CP+2*i) = 1.0;
            B_temp(i) = _CPref_horizon[i+1](0);
        }
        Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
        idx_cost += temp_size_cost;

        // 1-2. Capture Point Tracking (Y-direction)
        W_temp = 1.0;
        temp_size_cost = N;
        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
        for(int i=0;i<N;i++) {
            A_temp(i,idx_CP+2*i+1) = 1.0;
            B_temp(i) = _CPref_horizon[i+1](1);
        }
        Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
        idx_cost += temp_size_cost;

        // 2-1. Capture Point0 Change Minimization (X-direction)
        W_temp = 0.40;
        temp_size_cost = (N-1);
        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
        for(int i=0;i<(N-1);i++) {
            MatrixNd _A(1,4);
            _A.row(0) << -1.0,0.0,1.0,0.0;
            A_temp.block(i,idx_CP+2*i,1,4) = _A;
            B_temp(i) = GetFutureVelocityRef((double)(i+1)*dT)(0)*dT;
        }
        Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
        idx_cost += temp_size_cost;

        // 2-2. Capture Point0 Change Minimization (Y-direction)
        W_temp = 1.0;
        temp_size_cost = (N-1);
        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
        for(int i=0;i<(N-1);i++) {
            MatrixNd _A(1,4);
            _A.row(0) << 0.0,-1.0,0.0,1.0;
            A_temp.block(i,idx_CP+2*i,1,4) = _A;
            B_temp(i) = GetFutureVelocityRef((double)(i+1)*dT)(1)*dT;
        }
        Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
        idx_cost += temp_size_cost;

        // 3-1. CoM Position is placed around the step position (X-direction)
        W_temp = 0.05;
        temp_size_cost = N;
        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
        {
            double A = (1.0-wn_LIPM*dT+0.5*wn_LIPM*dT*wn_LIPM*dT);
            MatrixNd A_CoM = MatrixNd::Zero(N,1);
            MatrixNd B_CP = MatrixNd::Zero(N,N+1);
            MatrixNd B_ZMP = MatrixNd::Zero(N,N);
            A_CoM(0,0) = A;
            for(int i=0;i<N;i++) {
                B_CP(i,i) = wn_LIPM*dT;
                B_ZMP(i,i) = -0.5*wn_LIPM*dT*wn_LIPM*dT;
            }
            for(int i=1;i<N;i++) {
                A_CoM(i,0) = A*A_CoM(i-1,0);
                for(int j=0;j<i;j++) {
                    B_CP(i,j) = A*B_CP(i-1,j);
                    B_ZMP(i,j) = A*B_ZMP(i-1,j);
                }
            }

            for(int i=0;i<N;i++) {
                A_temp.block(0,idx_CP+2*i,N,1) = B_CP.col(i+1);
                A_temp.block(0,idx_ZMP+2*i,N,1) = B_ZMP.col(i);
                B_temp(i) = GetFutureStepPosition((double)(i+1)*dT)(0)-A_CoM(i,0)*Xref_CoM(0)-B_CP(i,0)*Xref_CP(0);
            }
        }
        Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
        idx_cost += temp_size_cost;

        n_cost = idx_cost;

        //        Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(A_cost.transpose()*A_cost);
        //        std::cout << "The rank of A is " << lu_decomp.rank() << std::endl;

        //////// Equality Constraints (Ax=B) /////////////////////////////////////////////////////////////////
        /// Optimization Variables : x = [Xx_MPC(4*N);
        ///                               Ux_MPC(N)];
        ///
        /// 1) Model Predictive Control :
        /// X_MPC = [x_k+1; x_k+2; ...; x_k+N]
        /// A_MPC = [A^1; A^2; ...; A^N]
        /// B_MPC = [B 0 .. 0; A*B B 0 .. 0; A*A*B A*B B 0 .. 0; ... ; A^(N-1)*B .. A*B B]
        /// U_MPC = [x_vel_ref(k+1);x_acc_ref(k+1);...;x_vel_ref(k+N);x_acc_ref(k+N)]
        ///    >> X_MPC = A_MPC*_xnow + B_MPC*U_MPC;
        ///    >> X_MPC - B_MPC*U_MPC = -A_MPC*_xnow;
        ///    >> [I -B_MPC]*[X_MPC;U_MPC] = -A_MPC*_xnow;
        /////////////////////////////////////////////////////////////////////////////////////////////////

        int n_equ =0;
        int idx_equ = 0;
        int temp_size_equ = 0;
        MatrixNd A_equ;
        VectorNd B_equ;

        MatrixNd Ad = MatrixNd::Zero(2,2);
        MatrixNd Bd = MatrixNd::Zero(2,2);
        MatrixNd A_MPC = MatrixNd::Zero(2*N,2);
        MatrixNd B_MPC = MatrixNd::Zero(2*N,2*N);

        Ad <<  1.0+wn_LIPM*dT, 0.0,
                0.0, 1.0+wn_LIPM*dT;
        Bd <<  -wn_LIPM*dT,0.0,
                0.0, -wn_LIPM*dT;

        A_MPC.block(0,0,2,2) = Ad;
        for(int i=0;i<N;i++) {
            B_MPC.block(2*i,2*i,2,2) = Bd;
        }
        for(int i=1;i<N;i++) {
            A_MPC.block(2*i,0,2,2) = Ad*A_MPC.block(2*(i-1),0,2,2);
            for(int j=0;j<i;j++) {
                B_MPC.block(2*i,2*j,2,2) = Ad*B_MPC.block(2*(i-1),2*j,2,2);
            }
        }

        temp_size_equ = 2*N;
        Resize_TempMatrix(temp_size_equ,n_var,A_temp,B_temp);
        A_temp.block(0,idx_CP,2*N,2*N) = MatrixNd::Identity(2*N,2*N);
        A_temp.block(0,idx_ZMP,2*N,2*N) = -B_MPC;
        B_temp.block(0,0,2*N,1) = A_MPC*_xnow;
        Matrix4QP_Addition(A_equ,B_equ,A_temp,B_temp);
        idx_equ += temp_size_equ;

        n_equ = idx_equ;

        //////// Inequality Constraints (Aq<=B) /////////////////////////////////////////////////////////
        /// 1) ZMP Constraints
        /// 2) Fz > 0
        /////////////////////////////////////////////////////////////////////////////////////////////////

        int n_inequ = 0;
        int idx_inequ = 0;
        int temp_size_inequ = 0;
        MatrixNd A_inequ;
        VectorNd B_inequ;

        // 1. ZMP limitation
        for(int i=0;i<_N_Prev;i++) {
            int StanceLeg;
            Vector3d StancePosition;
            double StanceZAngle;
            bool StepOn;
            Vector3d StepPotision;
            double StepZAngle;
            GetFutureStance((double)i*dT,
                            StanceLeg, StancePosition, StanceZAngle, StepOn, StepPotision, StepZAngle);

            MatrixNd _A;
            VectorNd _B;
            Get_SupportConvexHull(StanceLeg, StancePosition, StanceZAngle,
                                  StepOn, StepPotision, StepZAngle,
                                  _A, _B);
            temp_size_inequ = _A.rows();
            Resize_TempMatrix(temp_size_inequ,n_var,A_temp,B_temp);
            A_temp.block(0,idx_ZMP+2*i,temp_size_inequ,2) = _A.block(0,0,temp_size_inequ,2);
            B_temp.block(0,0,temp_size_inequ,1) = _B;

            Matrix4QP_Addition(A_inequ,B_inequ,A_temp,B_temp);
            idx_inequ += temp_size_inequ;
        }

        n_inequ = idx_inequ;

        ///////// Matrix and Vector Setting  ////////////////////////////////////////////////////////////////

        QP_CPref.setNums(n_var,n_cost,n_equ,n_inequ);
        QP_CPref.make_COST(A_cost, B_cost);
        QP_CPref.make_EQ(A_equ,B_equ);
        QP_CPref.make_IEQ(A_inequ,B_inequ);

        ///////// Solve the problem ////////////////////////////////////////////////////////////////

        switch(QP_CPref.WhichSolver()) {
        case SolverIsQuadProg:
        {
            // 1. Solve the problem.
            VectorNd QP_sol = QP_CPref.solve_QP();
            CP_MPC = QP_sol.segment(0,2*N);
            ZMP_MPC = QP_sol.segment(2*N,2*N);
            break;
        }
        case SolverIsQPSwift:
        {
            qp_int n_var = QP_CPref.NUMCOLS;
            qp_int n_equ = QP_CPref.NUMEQ;
            qp_int n_inequ = QP_CPref.NUMINEQ;

            qp_int   P_nnz,A_nnz,G_nnz;
            qp_real  *P_x = NULL;
            qp_int   *P_i = NULL,*P_p = NULL;
            qp_real  *q_x = NULL;
            qp_real  *A_x = NULL;
            qp_int   *A_i = NULL, *A_p = NULL;
            qp_real  *b_x = NULL;
            qp_real  *G_x = NULL;
            qp_int   *G_i = NULL, *G_p = NULL;
            qp_real  *h_x = NULL;

            // 1-0. Convert cost function to sparse form .
            MatrixNd P = QP_CPref.P_cost;
            VectorNd q = QP_CPref.Q_cost;
            P_nnz = QP_CPref.GetNumberOfNonZero_QPswift(P);
            P_x = new qp_real[P_nnz];
            P_i = new qp_int[P_nnz];
            P_p = new qp_int[n_var+1];
            QP_CPref.ConvertMatrixA_Full2CCS_QPswift(P,P_x,P_i,P_p);
            q_x = new qp_real[n_var];
            QP_CPref.ConvertVector2Array_QPswift(q,n_var,q_x);

            // 1-1. Convert equality constraint to sparse form .
            if(n_equ > 0) {
                MatrixNd A = QP_CPref.A_equ;
                MatrixNd b = QP_CPref.B_equ;
                A_nnz = QP_CPref.GetNumberOfNonZero_QPswift(A);
                A_x = new qp_real[A_nnz];
                A_i = new qp_int[A_nnz];
                A_p = new qp_int[n_var+1];
                QP_CPref.ConvertMatrixA_Full2CCS_QPswift(A,A_x,A_i,A_p);
                b_x = new qp_real[n_equ];
                QP_CPref.ConvertVector2Array_QPswift(b,n_equ,b_x);
            }

            // 1-2. Convert inequality constraint to sparse form .
            if(n_inequ > 0) {
                MatrixNd G = QP_CPref.A_inequ;
                MatrixNd h = QP_CPref.B_inequ;
                G_nnz = QP_CPref.GetNumberOfNonZero_QPswift(G);
                G_x = new qp_real[G_nnz];
                G_i = new qp_int[G_nnz];
                G_p = new qp_int[n_var+1];
                QP_CPref.ConvertMatrixA_Full2CCS_QPswift(G,G_x,G_i,G_p);
                h_x = new qp_real[n_inequ];
                QP_CPref.ConvertVector2Array_QPswift(h,n_inequ,h_x);
            }

            // 2. Set Parameters and solve.
            QPswift      *myQP;
            if(n_inequ != 0) {
                myQP = QP_SETUP(n_var,n_inequ,n_equ,
                                P_p,P_i,P_x,
                                A_p,A_i,A_x,
                                G_p,G_i,G_x,
                                q_x,h_x,b_x,
                                0.0,nullptr);
                QP_SOLVE(myQP);
            } else {
                myQP = QP_SETUP_NoInequ(n_var,n_equ,
                                        P_p,P_i,P_x,
                                        A_p,A_i,A_x,
                                        q_x,b_x,
                                        0.0,nullptr);
                QP_SOLVE_NoInequ(myQP);
                //                PRINT("Setup Time     : %f ms\n", (myQP->stats->tsetup)*1000.0);
                //                PRINT("Solve Time     : %f ms\n", (myQP->stats->tsolve)*1000.0);
            }
            VectorNd QP_sol = VectorNd::Zero(n_var);
            QP_CPref.ConvertArray2Vector_QPswift(myQP->x, n_var, QP_sol);
            CP_MPC = QP_sol.segment(0,2*N);
            ZMP_MPC = QP_sol.segment(2*N,2*N);

            // 3. destruction allocated memory
            QP_CLEANUP(myQP);
            delete []P_x;  delete []P_i;  delete []P_p;  delete []q_x;
            delete []A_x;  delete []A_i;  delete []A_p;  delete []b_x;
            delete []G_x;  delete []G_i;  delete []G_p;  delete []h_x;
            break;
        }
        case SolverIsAnalytic:
        {
            // 1. Solve the problem.
            VectorNd QP_sol = -pseudoInverse(QP_CPref.P_cost)*QP_CPref.Q_cost;
            CP_MPC = QP_sol.segment(0,2*N);
            ZMP_MPC = QP_sol.segment(2*N,2*N);
            break;
        }
        default:
            FILE_LOG(logERROR) << "[CP Pattern Generation Error] QP Solver is not set!";
            return false;
        }
    }

    // CP and ZMP reference which is revised by the optimization problem
    _CPref_horizon[0].segment(0,2) = Xref_CP.segment(0,2);
    for(int i=0;i<N;i++) {
        _CPref_horizon[i+1].segment(0,2) = CP_MPC.segment(2*i,2);
        _ZMPref_horizon[i].segment(0,2) = ZMP_MPC.segment(2*i,2);
    }

    // Resultant CoM Trajectory ========
    MatrixNd Ad = MatrixNd::Zero(2,2);
    MatrixNd Bd = MatrixNd::Zero(2,4);
    MatrixNd A_CoM = MatrixNd::Zero(2*N,2);
    MatrixNd B_CoM = MatrixNd::Zero(2*N,4*N);
    Ad <<  1.0-wn_LIPM*dT+0.5*wn_LIPM*dT*wn_LIPM*dT, 0.0,
            0.0, 1.0-wn_LIPM*dT+0.5*wn_LIPM*dT*wn_LIPM*dT;
    Bd <<  wn_LIPM*dT, -0.5*wn_LIPM*dT*wn_LIPM*dT, 0.0, 0.0,
            0.0, 0.0, wn_LIPM*dT, -0.5*wn_LIPM*dT*wn_LIPM*dT;

    A_CoM.block(0,0,2,2) = Ad;
    for(int i=0;i<N;i++) {
        B_CoM.block(2*i,4*i,2,4) = Bd;
    }
    for(int i=1;i<N;i++) {
        A_CoM.block(2*i,0,2,2) = Ad*A_CoM.block(2*(i-1),0,2,2);
        for(int j=0;j<i;j++) {
            B_CoM.block(2*i,4*j,2,4) = Ad*B_CoM.block(2*(i-1),4*j,2,4);
        }
    }

    VectorNd U_CoM = VectorNd::Zero(4*N);
    for(int i=0;i<N;i++) {
        U_CoM(4*i+0) = _CPref_horizon[i](0);
        U_CoM(4*i+1) = _ZMPref_horizon[i](0);
        U_CoM(4*i+2) = _CPref_horizon[i](1);
        U_CoM(4*i+3) = _ZMPref_horizon[i](1);
    }
    CoM_MPC = A_CoM*Xref_CoM.segment(0,2) + B_CoM*U_CoM;

    _CoMref_horizon[0].segment(0,2) = Xref_CoM.segment(0,2);
    for(int i=0;i<N;i++) {
        _CoMref_horizon[i+1].segment(0,2) = CoM_MPC.segment(2*i,2);
    }

    // Update states of next control period (SYS_DT) ========

    // CoM
    Xref_CoM.segment(0,2) = Xref_CoM.segment(0,2) + wn_LIPM*SYS_DT*(Xref_CP.segment(0,2)-Xref_CoM.segment(0,2))
            + 0.5*wn_LIPM*SYS_DT*wn_LIPM*SYS_DT*(Xref_CoM.segment(0,2)-_ZMPref_horizon[0].segment(0,2));

    // CP, dCP
    VectorNd _dxnext = VectorNd::Zero(2);
    VectorNd _xnext = VectorNd::Zero(2);
    VectorNd _unow = _ZMPref_horizon[0].segment(0,2);
    MatrixNd Ac = MatrixNd::Zero(2,2);
    MatrixNd Bc = MatrixNd::Zero(2,2);
    Ac <<  wn_LIPM, 0.0,
            0.0, wn_LIPM;
    Bc <<  -wn_LIPM,0.0,
            0.0, -wn_LIPM;
    _dxnext = Ac*_xnow + Bc*_unow;
    _xnext = _xnow + _dxnext*SYS_DT;

    dXref_CP.segment(0,2) = _dxnext;
    Xref_CP.segment(0,2) = _xnext;

    return true;
}


bool LIGHTWholeMotions::GenerateWalkingPattern_Formulation(Vector3d Xref_CoM, Vector3d dXref_CoM,
                                                           Vector3d u_CoM, Vector3d u_dCoM, Vector3d u_ddCoM)
{
    // _xnow = [x_pos, x_vel, xdes_pos, xdes_vel] (Current CoM)
    // X_MPC = [x_k+1; x_k+2; ...; x_k+N]
    // A_MPC = [A^1; A^2; ...; A^N]
    // B_MPC = [B 0 .. 0; A*B B 0 .. 0; A*A*B A*B B 0 .. 0; ... ; A^(N-1)*B .. A*B B]
    // U_MPC = [x_vel_ref(k+1);x_acc_ref(k+1);...;x_vel_ref(k+N);x_acc_ref(k+N)]
    //    >> X_MPC = A_MPC*_xnow + B_MPC*U_MPC;

    Vector3d* ZMPref_horizon = new Vector3d[N_horizon];
    Vector3d* CPref_horizon = new Vector3d[N_horizon+1];
    Vector3d* CoMref_horizon = new Vector3d[N_horizon+1];

    Generate_CPref_withMPC(N_horizon, dT_horizon, CPref_horizon, ZMPref_horizon, CoMref_horizon);

    // Initial States
    VectorNd _xnow = VectorNd::Zero(4);
    _xnow << Xref_CoM(0), dXref_CoM(0), u_CoM(0), u_dCoM(0);
    VectorNd _ynow = VectorNd::Zero(4);
    _ynow << Xref_CoM(1), dXref_CoM(1), u_CoM(1), u_dCoM(1);

    double zeta_X = zeta_X_pattern;
    double wn_X = wn_X_pattern;
    double zeta_Y = zeta_Y_pattern;
    double wn_Y = wn_X_pattern;
    int N = N_horizon;
    double dT = dT_horizon;

    ////////////////////////////////////////////////////////////////////////////////////
    //// QP Formulation for x-direction ////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////
    {
        MatrixNd A_temp;
        VectorNd B_temp;

        // state variable : [Xx_MPC(4*N); Ux_MPC(N)]
        int n_var   = 4*N+N;

        //////// Cost Functions /////////////////////////////////////////////////////////////////////////
        /// 1)
        /////////////////////////////////////////////////////////////////////////////////////////////////

        int n_cost = 0;
        int idx_cost = 0;
        int temp_size_cost = 0;
        MatrixNd A_cost;
        VectorNd B_cost;
        double W_temp = 0.0;

        // 1. Capture Point Reference Tracking
        W_temp = W_CPTracking;
        temp_size_cost = N;
        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
        for(int i=0;i<N;i++) {
            MatrixNd Ax_CP(1,4);
            Ax_CP << 1.0, 1.0/wn_LIPM, 0.0, 0.0;
            A_temp.block(i,4*i,1,4) = Ax_CP;
            B_temp(i,0) = CPref_horizon[i](0); // CP_x ref
        }
        Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
        idx_cost += temp_size_cost;

        //        // 2. ZMP Reference Tracking
        //        W_temp = W_ZMPTracking;
        //        temp_size_cost = N;
        //        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
        //        for(int i=0;i<N;i++) {
        //            MatrixNd Ax_ZMP(1,4);
        //            Ax_ZMP << 1.0+(wn_X*wn_X)/(wn_LIPM*wn_LIPM), (2.0*zeta_X*wn_X)/(wn_LIPM*wn_LIPM), -(wn_X*wn_X)/(wn_LIPM*wn_LIPM), -(2.0*zeta_X*wn_X)/(wn_LIPM*wn_LIPM);
        //            A_temp.block(i,4*i,1,4) = Ax_ZMP;
        //            B_temp(i,0) = ZMPref_horizon[i](0); // ZMP_x ref
        //        }
        //        Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
        //        idx_cost += temp_size_cost;

        //        // 3-0. Initial State Change Minimization
        //        temp_size_cost = 4;
        //        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
        //        A_temp(0,0) = W_Delta_Pos;
        //        A_temp(1,1) = W_Delta_Vel;
        //        A_temp(2,2) = W_Delta_PosRef;
        //        A_temp(3,3) = W_Delta_VelRef;
        //        B_temp(0,0) = W_Delta_Pos*_xnow(0);
        //        B_temp(1,0) = W_Delta_Vel*_xnow(1);
        //        B_temp(2,0) = W_Delta_PosRef*_xnow(2);
        //        B_temp(3,0) = W_Delta_VelRef*_xnow(3);
        //        Matrix4QP_Addition(A_cost,B_cost,A_temp,B_temp);
        //        idx_cost += temp_size_cost;

        //        // 3-1. Position State Change Minimization
        //        W_temp = W_Delta_Pos;
        //        temp_size_cost = (N-1);
        //        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
        //        for(int i=0;i<(N-1);i++) {
        //            MatrixNd _A(1,8);
        //            _A << 1.0,0.0,0.0,0.0,-1.0,0.0,0.0,0.0;
        //            A_temp.block(i,4*i,1,8) = _A;
        //        }
        //        B_temp.block(0,0,(N-1),1) = VectorNd::Zero(N-1);
        //        Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
        //        idx_cost += temp_size_cost;

        //        // 3-2. Velocity State Change Minimization
        //        W_temp = W_Delta_Vel;
        //        temp_size_cost = (N-1);
        //        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
        //        for(int i=0;i<(N-1);i++) {
        //            MatrixNd _A(1,8);
        //            _A << 0.0,1.0,0.0,0.0,0.0,-1.0,0.0,0.0;
        //            A_temp.block(i,4*i,1,8) = _A;
        //        }
        //        B_temp.block(0,0,(N-1),1) = VectorNd::Zero(N-1);
        //        Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
        //        idx_cost += temp_size_cost;

        //        // 3-3. Desired Position State Change Minimization
        //        W_temp = W_Delta_PosRef;
        //        temp_size_cost = (N-1);
        //        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
        //        for(int i=0;i<(N-1);i++) {
        //            MatrixNd _A(1,8);
        //            _A << 0.0,0.0,1.0,0.0,0.0,0.0,-1.0,0.0;
        //            A_temp.block(i,4*i,1,8) = _A;
        //        }
        //        B_temp.block(0,0,(N-1),1) = VectorNd::Zero(N-1);
        //        Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
        //        idx_cost += temp_size_cost;

        //        // 3-4. Desired Velocity State Change Minimization
        //        W_temp = W_Delta_VelRef;
        //        temp_size_cost = (N-1);
        //        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
        //        for(int i=0;i<(N-1);i++) {
        //            MatrixNd _A(1,8);
        //            _A << 0.0,0.0,0.0,1.0,0.0,0.0,0.0,-1.0;
        //            A_temp.block(i,4*i,1,8) = _A;
        //        }
        //        B_temp.block(0,0,(N-1),1) = VectorNd::Zero(N-1);
        //        Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
        //        idx_cost += temp_size_cost;

        //        // 4-0. Initial Input Change Minimization
        //        W_temp = W_Delta_Input*(SYS_DT/dT);
        //        temp_size_cost = 1;
        //        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
        //        A_temp(0,4*N) = 1.0;
        //        B_temp(0,0) = u_ddCoM(0);
        //        Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
        //        idx_cost += temp_size_cost;

        // 4-1. Input Change Minimization
        W_temp = W_Delta_Input;
        temp_size_cost = (N-1);
        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
        for(int i=0;i<(N-1);i++) {
            MatrixNd _A(1,2);
            _A << 1.0,-1.0;
            A_temp.block(i,4*N+i,1,2) = _A;
        }
        B_temp.block(0,0,(N-1),1) = VectorNd::Zero(N-1);
        Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
        idx_cost += temp_size_cost;

        n_cost = idx_cost;

        //        Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(A_cost.transpose()*A_cost);
        //        std::cout << "The rank of A is " << lu_decomp.rank() << std::endl;

        //////// Equality Constraints (Ax=B) /////////////////////////////////////////////////////////////////
        /// Optimization Variables : x = [Xx_MPC(4*N);
        ///                               Ux_MPC(N)];
        ///
        /// 1) Model Predictive Control :
        /// X_MPC = [x_k+1; x_k+2; ...; x_k+N]
        /// A_MPC = [A^1; A^2; ...; A^N]
        /// B_MPC = [B 0 .. 0; A*B B 0 .. 0; A*A*B A*B B 0 .. 0; ... ; A^(N-1)*B .. A*B B]
        /// U_MPC = [x_vel_ref(k+1);x_acc_ref(k+1);...;x_vel_ref(k+N);x_acc_ref(k+N)]
        ///    >> X_MPC = A_MPC*_xnow + B_MPC*U_MPC;
        ///    >> X_MPC - B_MPC*U_MPC = -A_MPC*_xnow;
        ///    >> [I -B_MPC]*[X_MPC;U_MPC] = -A_MPC*_xnow;
        /////////////////////////////////////////////////////////////////////////////////////////////////

        int n_equ =0;
        int idx_equ = 0;
        int temp_size_equ = 0;
        MatrixNd A_equ;
        VectorNd B_equ;

        MatrixNd Adx = MatrixNd::Zero(4,4);
        MatrixNd Bdx = MatrixNd::Zero(4,1);
        MatrixNd Ax_MPC = MatrixNd::Zero(4*N,4);
        MatrixNd Bx_MPC = MatrixNd::Zero(4*N,N);
        Adx <<  1.0-(wn_X*wn_X)*dT*dT/2.0,  dT-(2.0*wn_X*zeta_X)*dT*dT/2.0,  (wn_X*wn_X)*dT*dT/2.0,  (2.0*wn_X*zeta_X)*dT*dT/2.0,
                -(wn_X*wn_X)*dT,        1.0-(2.0*wn_X*zeta_X)*dT,         (wn_X*wn_X)*dT,         (2.0*wn_X*zeta_X)*dT,
                0.0,                             0.0,                    1.0,                           dT,
                0.0,                             0.0,                    0.0,                          1.0;
        Bdx <<                  dT*dT/2.0,                              dT,              dT*dT/2.0,                           dT;
        Ax_MPC.block(0,0,4,4) = Adx;
        for(int i=0;i<N;i++) {
            Bx_MPC.block(4*i,i,4,1) = Bdx;
        }
        for(int i=1;i<N;i++) {
            Ax_MPC.block(4*i,0,4,4) = Adx*Ax_MPC.block(4*(i-1),0,4,4);
            for(int j=0;j<i;j++) {
                Bx_MPC.block(4*i,j,4,1) = Adx*Bx_MPC.block(4*(i-1),j,4,1);
            }
        }
        temp_size_equ = 4*N;
        Resize_TempMatrix(temp_size_equ,n_var,A_temp,B_temp);
        A_temp.block(0,0*N,4*N,4*N) = MatrixNd::Identity(4*N,4*N);
        A_temp.block(0,4*N,4*N,1*N) = -Bx_MPC;
        B_temp.block(0,0,4*N,1) = Ax_MPC*_xnow;
        Matrix4QP_Addition(A_equ,B_equ,A_temp,B_temp);
        idx_equ += temp_size_equ;

        n_equ = idx_equ;

        //////// Inequality Constraints (Aq<=B) /////////////////////////////////////////////////////////
        /// 1) ZMP Constraints
        /// 2) Fz > 0
        /////////////////////////////////////////////////////////////////////////////////////////////////

        int n_inequ = 0;
        int idx_inequ = 0;
        int temp_size_inequ = 0;
        MatrixNd A_inequ;
        VectorNd B_inequ;

        n_inequ = idx_inequ;

        ///////// Matrix and Vector Setting  ////////////////////////////////////////////////////////////////

        QP_xCoMRef4Walking.setNums(n_var,n_cost,n_equ,n_inequ);
        QP_xCoMRef4Walking.make_COST(A_cost, B_cost);
        QP_xCoMRef4Walking.make_EQ(A_equ,B_equ);
        QP_xCoMRef4Walking.make_IEQ(A_inequ,B_inequ);
    }


    ////////////////////////////////////////////////////////////////////////////////////
    //// QP Formulation for y-direction ////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////
    {
        MatrixNd A_temp;
        VectorNd B_temp;

        // state variable : [Xx_MPC(4*N); Ux_MPC(N)]
        int n_var   = 4*N+N;

        //////// Cost Functions /////////////////////////////////////////////////////////////////////////
        /// 1)
        /////////////////////////////////////////////////////////////////////////////////////////////////

        int n_cost = 0;
        int idx_cost = 0;
        int temp_size_cost = 0;
        MatrixNd A_cost;
        VectorNd B_cost;
        double W_temp = 0.0;

        // 1. Capture Point Reference Tracking
        W_temp = W_CPTracking;
        temp_size_cost = N;
        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
        for(int i=0;i<N;i++) {
            MatrixNd Ay_CP(1,4);
            Ay_CP << 1.0, 1.0/wn_LIPM, 0.0, 0.0;
            A_temp.block(i,4*i,1,4) = Ay_CP;
            B_temp(i,0) = CPref_horizon[i](1); // CP_y ref
        }
        Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
        idx_cost += temp_size_cost;

        //        // 2. ZMP Reference Tracking
        //        W_temp = W_ZMPTracking;
        //        temp_size_cost = N;
        //        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
        //        for(int i=0;i<N;i++) {
        //            MatrixNd Ay_ZMP(1,4);
        //            Ay_ZMP << 1.0+(wn_Y*wn_Y)/(wn_LIPM*wn_LIPM), (2.0*zeta_Y*wn_Y)/(wn_LIPM*wn_LIPM), -(wn_Y*wn_Y)/(wn_LIPM*wn_LIPM), -(2.0*zeta_Y*wn_Y)/(wn_LIPM*wn_LIPM);
        //            A_temp.block(i,4*i,1,4) = Ay_ZMP;
        //            B_temp(i,0) = ZMPref_horizon[i](1); // ZMP_y ref
        //        }
        //        Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
        //        idx_cost += temp_size_cost;

        //        // 3-0. Initial State Change Minimization
        //        temp_size_cost = 4;
        //        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
        //        A_temp(0,0) = W_Delta_Pos;
        //        A_temp(1,1) = W_Delta_Vel;
        //        A_temp(2,2) = W_Delta_PosRef;
        //        A_temp(3,3) = W_Delta_VelRef;
        //        B_temp(0,0) = W_Delta_Pos*_xnow(0);
        //        B_temp(1,0) = W_Delta_Vel*_xnow(1);
        //        B_temp(2,0) = W_Delta_PosRef*_xnow(2);
        //        B_temp(3,0) = W_Delta_VelRef*_xnow(3);
        //        Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
        //        idx_cost += temp_size_cost;

        //        // 3-1. Position State Change Minimization
        //        W_temp = W_Delta_Pos;
        //        temp_size_cost = (N-1);
        //        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
        //        for(int i=0;i<(N-1);i++) {
        //            MatrixNd _A(1,8);
        //            _A << 1.0,0.0,0.0,0.0,-1.0,0.0,0.0,0.0;
        //            A_temp.block(i,4*i,1,8) = _A;
        //        }
        //        B_temp.block(0,0,(N-1),1) = VectorNd::Zero(N-1);
        //        Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
        //        idx_cost += temp_size_cost;

        //        // 3-2. Velocity State Change Minimization
        //        W_temp = W_Delta_Vel;
        //        temp_size_cost = (N-1);
        //        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
        //        for(int i=0;i<(N-1);i++) {
        //            MatrixNd _A(1,8);
        //            _A << 0.0,1.0,0.0,0.0,0.0,-1.0,0.0,0.0;
        //            A_temp.block(i,4*i,1,8) = _A;
        //        }
        //        B_temp.block(0,0,(N-1),1) = VectorNd::Zero(N-1);
        //        Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
        //        idx_cost += temp_size_cost;

        //        // 3-3. Desired Position State Change Minimization
        //        W_temp = W_Delta_PosRef;
        //        temp_size_cost = (N-1);
        //        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
        //        for(int i=0;i<(N-1);i++) {
        //            MatrixNd _A(1,8);
        //            _A << 0.0,0.0,1.0,0.0,0.0,0.0,-1.0,0.0;
        //            A_temp.block(i,4*i,1,8) = _A;
        //        }
        //        B_temp.block(0,0,(N-1),1) = VectorNd::Zero(N-1);
        //        Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
        //        idx_cost += temp_size_cost;

        //        // 3-4. Desired Velocity Input Change Minimization
        //        W_temp = W_Delta_VelRef;
        //        temp_size_cost = (N-1);
        //        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
        //        for(int i=0;i<(N-1);i++) {
        //            MatrixNd _A(1,8);
        //            _A << 0.0,0.0,0.0,1.0,0.0,0.0,0.0,-1.0;
        //            A_temp.block(i,4*i,1,8) = _A;
        //        }
        //        B_temp.block(0,0,(N-1),1) = VectorNd::Zero(N-1);
        //        Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
        //        idx_cost += temp_size_cost;

        //        // 4-0. Initial Input Change Minimization
        //        W_temp = W_Delta_Input*(SYS_DT/dT);
        //        temp_size_cost = 1;
        //        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
        //        A_temp(0,4*N) = 1.0;
        //        B_temp(0,0) = u_ddCoM(1);
        //        Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
        //        idx_cost += temp_size_cost;

        // 4-1. Input Change Minimization
        W_temp = W_Delta_Input;
        temp_size_cost = N-1;
        Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
        for(int i=0;i<(N-1);i++) {
            MatrixNd _A(1,2);
            _A << 1.0,-1.0;
            A_temp.block(i,4*N+i,1,2) = _A;
        }
        B_temp.block(0,0,(N-1),1) = VectorNd::Zero(N-1);
        Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
        idx_cost += temp_size_cost;

        n_cost = idx_cost;

        //        Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(A_cost.transpose()*A_cost);
        //        std::cout << "The rank of A is " << lu_decomp.rank() << std::endl;

        //////// Equality Constraints (Ax=B) /////////////////////////////////////////////////////////////////
        /// Optimization Variables : x = [Xx_MPC(4*N);
        ///                               Ux_MPC(N)];
        ///
        /// 1) Model Predictive Control :
        /// X_MPC = [x_k+1; x_k+2; ...; x_k+N]
        /// A_MPC = [A^1; A^2; ...; A^N]
        /// B_MPC = [B 0 .. 0; A*B B 0 .. 0; A*A*B A*B B 0 .. 0; ... ; A^(N-1)*B .. A*B B]
        /// U_MPC = [x_vel_ref(k+1);x_acc_ref(k+1);...;x_vel_ref(k+N);x_acc_ref(k+N)]
        ///    >> X_MPC = A_MPC*_xnow + B_MPC*U_MPC;
        ///    >> X_MPC - B_MPC*U_MPC = -A_MPC*_xnow;
        ///    >> [I -B_MPC]*[X_MPC;U_MPC] = -A_MPC*_xnow;
        /////////////////////////////////////////////////////////////////////////////////////////////////

        int n_equ =0;
        int idx_equ = 0;
        int temp_size_equ = 0;
        MatrixNd A_equ;
        VectorNd B_equ;

        MatrixNd Ady = MatrixNd::Zero(4,4);
        MatrixNd Bdy = MatrixNd::Zero(4,1);
        MatrixNd Ay_MPC = MatrixNd::Zero(4*N,4);
        MatrixNd By_MPC = MatrixNd::Zero(4*N,N);

        Ady <<  1.0-(wn_Y*wn_Y)*dT*dT/2.0,  dT-(2.0*wn_Y*zeta_Y)*dT*dT/2.0,  (wn_Y*wn_Y)*dT*dT/2.0,  (2.0*wn_Y*zeta_Y)*dT*dT/2.0,
                -(wn_Y*wn_Y)*dT,        1.0-(2.0*wn_Y*zeta_Y)*dT,         (wn_Y*wn_Y)*dT,         (2.0*wn_Y*zeta_Y)*dT,
                0.0,                             0.0,                    1.0,                           dT,
                0.0,                             0.0,                    0.0,                          1.0;
        Bdy <<                  dT*dT/2.0,                              dT,              dT*dT/2.0,                           dT;
        Ay_MPC.block(0,0,4,4) = Ady;
        for(int i=0;i<N;i++) {
            By_MPC.block(4*i,i,4,1) = Bdy;
        }
        for(int i=1;i<N;i++) {
            Ay_MPC.block(4*i,0,4,4) = Ady*Ay_MPC.block(4*(i-1),0,4,4);
            for(int j=0;j<i;j++) {
                By_MPC.block(4*i,j,4,1) = Ady*By_MPC.block(4*(i-1),j,4,1);
            }
        }
        temp_size_equ = 4*N;
        Resize_TempMatrix(temp_size_equ,n_var,A_temp,B_temp);
        A_temp.block(0,0*N,4*N,4*N) = MatrixNd::Identity(4*N,4*N);
        A_temp.block(0,4*N,4*N,1*N) = -By_MPC;
        B_temp.block(0,0,4*N,1) = Ay_MPC*_ynow;
        Matrix4QP_Addition(A_equ,B_equ,A_temp,B_temp);
        idx_equ += temp_size_equ;

        n_equ = idx_equ;

        //////// Inequality Constraints (Aq<=B) /////////////////////////////////////////////////////////
        /// 1) ZMP Constraints
        /// 2) Fz > 0
        /////////////////////////////////////////////////////////////////////////////////////////////////

        int n_inequ = 0;
        int idx_inequ = 0;
        int temp_size_inequ = 0;
        MatrixNd A_inequ;
        VectorNd B_inequ;

        n_inequ = idx_inequ;

        ///////// Matrix and Vector Setting  ////////////////////////////////////////////////////////////////

        QP_yCoMRef4Walking.setNums(n_var,n_cost,n_equ,n_inequ);
        QP_yCoMRef4Walking.make_COST(A_cost, B_cost);
        QP_yCoMRef4Walking.make_EQ(A_equ,B_equ);
        QP_yCoMRef4Walking.make_IEQ(A_inequ,B_inequ);
    }

//    Calc_FuturePressureReference_WithWalkPattern(N_horizon, dT_horizon,CPref_horizon,ZMPref_horizon,CoMref_horizon);
    delete []CPref_horizon;
    delete []ZMPref_horizon;
    delete []CoMref_horizon;
}

bool LIGHTWholeMotions:: GenerateWalkingPattern_Update(Vector3d Xnow_CoM, Vector3d dXnow_CoM,
                                                       Vector3d u_CoM, Vector3d u_dCoM, Vector3d u_ddCoM,
                                                       Vector3d* _CPref, Vector3d* _ZMPref)
{

    // Initial States
    VectorNd _xnow = VectorNd::Zero(4);
    _xnow << Xnow_CoM(0), dXnow_CoM(0), u_CoM(0), u_dCoM(0);
    VectorNd _ynow = VectorNd::Zero(4);
    _ynow << Xnow_CoM(1), dXnow_CoM(1), u_CoM(1), u_dCoM(1);

    double zeta_X = zeta_X_pattern;
    double wn_X = wn_X_pattern;
    double zeta_Y = zeta_Y_pattern;
    double wn_Y = wn_Y_pattern;
    int N = N_horizon;
    double dT = dT_horizon;

    ////////////////////////////////////////////////////////////////////////////////////
    //// QP Formulation for x-direction ////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////
    {

        //////// Cost Functions /////////////////////////////////////////////////////////////////////////

        int n_cost = 0;
        int idx_cost = 0;
        VectorNd B_cost = VectorNd::Zero(QP_xCoMRef4Walking.NUMCOST);

        // 1. Capture Point Reference Tracking
        for(int i=0;i<N;i++) {
            B_cost(idx_cost+i,0) = W_CPTracking*_CPref[i](0); // CP_x ref
        }
        idx_cost += N;

        //        // 2. ZMP Reference Tracking
        //        for(int i=0;i<N;i++) {
        //            B_cost(idx_cost+i,0) = W_ZMPTracking*_ZMPref[i](0); // ZMP_y ref
        //        }
        //        idx_cost += N;

        //        // 3-0. Initial State Change Minimization
        //        B_cost(idx_cost+0,0) = W_Delta_Pos*_xnow(0);
        //        B_cost(idx_cost+1,0) = W_Delta_Vel*_xnow(1);
        //        B_cost(idx_cost+2,0) = W_Delta_PosRef*_xnow(2);
        //        B_cost(idx_cost+3,0) = W_Delta_VelRef*_xnow(3);
        //        idx_cost += 4;

        //        // 3-1. Position State Change Minimization
        //        B_cost.block(idx_cost,0,(N-1),1) = VectorNd::Zero(N-1);
        //        idx_cost += (N-1);

        //        // 3-2. Velocity State Change Minimization
        //        B_cost.block(idx_cost,0,(N-1),1) = VectorNd::Zero(N-1);
        //        idx_cost += (N-1);

        //        // 3-3. Desired Position State Change Minimization
        //        B_cost.block(idx_cost,0,(N-1),1) = VectorNd::Zero(N-1);
        //        idx_cost += (N-1);

        //        // 3-4. Desired Velocity State Change Minimization
        //        B_cost.block(idx_cost,0,(N-1),1) = VectorNd::Zero(N-1);
        //        idx_cost += (N-1);

        //        // 4-0. Initial Input Change Minimization
        //        B_cost(idx_cost,0) = W_Delta_Input*(SYS_DT/dT)*u_ddCoM(0);
        //        idx_cost += 1;

        // 4-1. Input Change Minimization
        B_cost.block(idx_cost,0,(N-1),1) = VectorNd::Zero(N-1);
        idx_cost += (N-1);

        n_cost = idx_cost;

        //////// Equality Constraints (Ax=B) /////////////////////////////////////////////////////////////////

        int n_equ =0;
        int idx_equ = 0;
        VectorNd B_equ = VectorNd::Zero(QP_xCoMRef4Walking.NUMEQ);

        MatrixNd Adx = MatrixNd::Zero(4,4);
        MatrixNd Ax_MPC = MatrixNd::Zero(4*N,4);
        Adx <<  1.0-(wn_X*wn_X)*dT*dT/2.0,  dT-(2.0*wn_X*zeta_X)*dT*dT/2.0,  (wn_X*wn_X)*dT*dT/2.0,  (2.0*wn_X*zeta_X)*dT*dT/2.0,
                -(wn_X*wn_X)*dT,        1.0-(2.0*wn_X*zeta_X)*dT,         (wn_X*wn_X)*dT,         (2.0*wn_X*zeta_X)*dT,
                0.0,                             0.0,                    1.0,                           dT,
                0.0,                             0.0,                    0.0,                          1.0;
        Ax_MPC.block(0,0,4,4) = Adx;
        for(int i=1;i<N;i++) {
            Ax_MPC.block(4*i,0,4,4) = Adx*Ax_MPC.block(4*(i-1),0,4,4);
        }
        B_equ.block(0,0,4*N,1) = Ax_MPC*_xnow;
        idx_equ += 4*N;

        n_equ = idx_equ;

        //////// Inequality Constraints (Aq<=B) /////////////////////////////////////////////////////////
        /// 1) ZMP Constraints
        /// 2) Fz > 0
        /////////////////////////////////////////////////////////////////////////////////////////////////

        int n_inequ = 0;
        int idx_inequ = 0;
        VectorNd B_inequ = VectorNd::Zero(QP_xCoMRef4Walking.NUMINEQ);

        n_inequ = idx_inequ;

        ///////// Matrix and Vector Setting  ////////////////////////////////////////////////////////////////
        QP_xCoMRef4Walking.update_Bcost(B_cost);
        QP_xCoMRef4Walking.update_Bequ(B_equ);
        QP_xCoMRef4Walking.update_Binequ(B_inequ);
    }

    ////////////////////////////////////////////////////////////////////////////////////
    //// QP Formulation for y-direction ////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////
    {
        //////// Cost Functions /////////////////////////////////////////////////////////////////////////

        int n_cost = 0;
        int idx_cost = 0;
        VectorNd B_cost = VectorNd::Zero(QP_yCoMRef4Walking.NUMCOST);

        // 1. Capture Point Reference Tracking ()
        for(int i=0;i<N;i++) {
            B_cost(idx_cost+i,0) = W_CPTracking*_CPref[i+1](1); // CP_x ref
        }
        idx_cost += N;

        //        // 2. ZMP Reference Tracking
        //        for(int i=0;i<N;i++) {
        //            B_cost(idx_cost+i,0) = W_ZMPTracking*_ZMPref[i](1); // ZMP_y ref
        //        }
        //        idx_cost += N;

        //        // 3-0. Initial State Change Minimization
        //        B_cost(idx_cost+0,0) = W_Delta_Pos*_ynow(0);
        //        B_cost(idx_cost+1,0) = W_Delta_Vel*_ynow(1);
        //        B_cost(idx_cost+2,0) = W_Delta_PosRef*_ynow(2);
        //        B_cost(idx_cost+3,0) = W_Delta_VelRef*_ynow(3);
        //        idx_cost += 4;

        //        // 3-1. Position State Change Minimization
        //        B_cost.block(idx_cost,0,(N-1),1) = VectorNd::Zero(N-1);
        //        idx_cost += (N-1);

        //        // 3-2. Velocity State Change Minimization
        //        B_cost.block(idx_cost,0,(N-1),1) = VectorNd::Zero(N-1);
        //        idx_cost += (N-1);

        //        // 3-3. Desired Position State Change Minimization
        //        B_cost.block(idx_cost,0,(N-1),1) = VectorNd::Zero(N-1);
        //        idx_cost += (N-1);

        //        // 3-4. Desired Velocity State Change Minimization
        //        B_cost.block(idx_cost,0,(N-1),1) = VectorNd::Zero(N-1);
        //        idx_cost += (N-1);

        //        // 4-0. Initial Input Change Minimization
        //        B_cost(idx_cost,0) = W_Delta_Input*(SYS_DT/dT)*u_ddCoM(1);
        //        idx_cost += 1;

        // 4-1. Input Change Minimization
        B_cost.block(idx_cost,0,(N-1),1) = VectorNd::Zero(N-1);
        idx_cost += (N-1);

        n_cost = idx_cost;

        //////// Equality Constraints (Ax=B) /////////////////////////////////////////////////////////////////

        int n_equ =0;
        int idx_equ = 0;
        VectorNd B_equ = VectorNd::Zero(QP_yCoMRef4Walking.NUMEQ);

        MatrixNd Ady = MatrixNd::Zero(4,4);
        MatrixNd Ay_MPC = MatrixNd::Zero(4*N,4);
        Ady <<  1.0-(wn_Y*wn_Y)*dT*dT/2.0,  dT-(2.0*wn_Y*zeta_Y)*dT*dT/2.0,  (wn_Y*wn_Y)*dT*dT/2.0,  (2.0*wn_Y*zeta_Y)*dT*dT/2.0,
                -(wn_Y*wn_Y)*dT,        1.0-(2.0*wn_Y*zeta_Y)*dT,         (wn_Y*wn_Y)*dT,         (2.0*wn_Y*zeta_Y)*dT,
                0.0,                             0.0,                    1.0,                           dT,
                0.0,                             0.0,                    0.0,                          1.0;
        Ay_MPC.block(0,0,4,4) = Ady;
        for(int i=1;i<N;i++) {
            Ay_MPC.block(4*i,0,4,4) = Ady*Ay_MPC.block(4*(i-1),0,4,4);
        }
        B_equ.block(0,0,4*N,1) = Ay_MPC*_ynow;
        idx_equ += 4*N;

        n_equ = idx_equ;

        //////// Inequality Constraints (Aq<=B) /////////////////////////////////////////////////////////
        /// 1) ZMP Constraints
        /// 2) Fz > 0
        /////////////////////////////////////////////////////////////////////////////////////////////////

        int n_inequ = 0;
        int idx_inequ = 0;
        VectorNd B_inequ = VectorNd::Zero(QP_yCoMRef4Walking.NUMINEQ);

        n_inequ = idx_inequ;

        ///////// Matrix and Vector Setting  ////////////////////////////////////////////////////////////////
        QP_yCoMRef4Walking.update_Bcost(B_cost);
        QP_yCoMRef4Walking.update_Bequ(B_equ);
        QP_yCoMRef4Walking.update_Binequ(B_inequ);
    }
}

bool LIGHTWholeMotions::GenerateWalkingPattern_Solve(Vector3d Xpre_CoM, Vector3d dXpre_CoM,
                                                     Vector3d &u_CoM, Vector3d &u_dCoM, Vector3d &u_ddCoM,
                                                     Vector3d &Xnext_CoM, Vector3d &dXnext_CoM, Vector3d &ddXnext_CoM)
{
    Vector3d* ZMPref_horizon = new Vector3d[N_horizon];
    Vector3d* CPref_horizon = new Vector3d[N_horizon+1];
    Vector3d* CoMref_horizon = new Vector3d[N_horizon+1];

    Generate_CPref_withMPC(N_horizon, dT_horizon, CPref_horizon, ZMPref_horizon, CoMref_horizon);

    GenerateWalkingPattern_Update(Xpre_CoM, dXpre_CoM,
                                  u_CoM, u_dCoM, u_ddCoM,
                                  CPref_horizon, ZMPref_horizon);

    ///////// Solve the problem for X-direction ////////////////////////////////////////////////////////////////
    int N = N_horizon;
    VectorNd Xx_MPC = VectorNd::Zero(4*N,1);
    VectorNd Ux_MPC = VectorNd::Zero(N,1);

    switch(QP_xCoMRef4Walking.WhichSolver()) {
    case SolverIsQuadProg:
    {
        // 1. Solve the problem.
        VectorNd QP_sol = QP_xCoMRef4Walking.solve_QP();
        Xx_MPC = QP_sol.segment(0,4*N);
        Ux_MPC = QP_sol.segment(4*N,N);
        break;
    }
    case SolverIsQPSwift:
    {
        qp_int n_var = QP_xCoMRef4Walking.NUMCOLS;
        qp_int n_equ = QP_xCoMRef4Walking.NUMEQ;
        qp_int n_inequ = QP_xCoMRef4Walking.NUMINEQ;

        qp_int   P_nnz,A_nnz,G_nnz;
        qp_real  *P_x = NULL;
        qp_int   *P_i = NULL,*P_p = NULL;
        qp_real  *q_x = NULL;
        qp_real  *A_x = NULL;
        qp_int   *A_i = NULL, *A_p = NULL;
        qp_real  *b_x = NULL;
        qp_real  *G_x = NULL;
        qp_int   *G_i = NULL, *G_p = NULL;
        qp_real  *h_x = NULL;

        // 1-0. Convert cost function to sparse form .
        MatrixNd P = QP_xCoMRef4Walking.P_cost;
        VectorNd q = QP_xCoMRef4Walking.Q_cost;
        P_nnz = QP_xCoMRef4Walking.GetNumberOfNonZero_QPswift(P);
        P_x = new qp_real[P_nnz];
        P_i = new qp_int[P_nnz];
        P_p = new qp_int[n_var+1];
        QP_xCoMRef4Walking.ConvertMatrixA_Full2CCS_QPswift(P,P_x,P_i,P_p);
        q_x = new qp_real[n_var];
        QP_xCoMRef4Walking.ConvertVector2Array_QPswift(q,n_var,q_x);

        // 1-1. Convert equality constraint to sparse form .
        if(n_equ > 0) {
            MatrixNd A = QP_xCoMRef4Walking.A_equ;
            MatrixNd b = QP_xCoMRef4Walking.B_equ;
            A_nnz = QP_xCoMRef4Walking.GetNumberOfNonZero_QPswift(A);
            A_x = new qp_real[A_nnz];
            A_i = new qp_int[A_nnz];
            A_p = new qp_int[n_var+1];
            QP_xCoMRef4Walking.ConvertMatrixA_Full2CCS_QPswift(A,A_x,A_i,A_p);
            b_x = new qp_real[n_equ];
            QP_xCoMRef4Walking.ConvertVector2Array_QPswift(b,n_equ,b_x);
        }

        // 1-2. Convert inequality constraint to sparse form .
        if(n_inequ > 0) {
            MatrixNd G = QP_xCoMRef4Walking.A_inequ;
            MatrixNd h = QP_xCoMRef4Walking.B_inequ;
            G_nnz = QP_xCoMRef4Walking.GetNumberOfNonZero_QPswift(G);
            G_x = new qp_real[G_nnz];
            G_i = new qp_int[G_nnz];
            G_p = new qp_int[n_var+1];
            QP_xCoMRef4Walking.ConvertMatrixA_Full2CCS_QPswift(G,G_x,G_i,G_p);
            h_x = new qp_real[n_inequ];
            QP_xCoMRef4Walking.ConvertVector2Array_QPswift(h,n_inequ,h_x);
        }

        // 2. Set Parameters and solve.
        QPswift      *myQP;
        if(n_inequ != 0) {
            myQP = QP_SETUP(n_var,n_inequ,n_equ,
                            P_p,P_i,P_x,
                            A_p,A_i,A_x,
                            G_p,G_i,G_x,
                            q_x,h_x,b_x,
                            0.0,nullptr);
            QP_SOLVE(myQP);
        } else {
            myQP = QP_SETUP_NoInequ(n_var,n_equ,
                                    P_p,P_i,P_x,
                                    A_p,A_i,A_x,
                                    q_x,b_x,
                                    0.0,nullptr);
            QP_SOLVE_NoInequ(myQP);
            //            PRINT("Setup Time     : %f ms\n", (myQP->stats->tsetup)*1000.0);
            //            PRINT("Solve Time     : %f ms\n", (myQP->stats->tsolve)*1000.0);
        }
        VectorNd QP_sol = VectorNd::Zero(n_var);
        QP_xCoMRef4Walking.ConvertArray2Vector_QPswift(myQP->x, n_var, QP_sol);
        Xx_MPC = QP_sol.segment(0,4*N);
        Ux_MPC = QP_sol.segment(4*N,N);

        // 3. destruction allocated memory
        QP_CLEANUP(myQP);
        delete []P_x;  delete []P_i;  delete []P_p;  delete []q_x;
        delete []A_x;  delete []A_i;  delete []A_p;  delete []b_x;
        delete []G_x;  delete []G_i;  delete []G_p;  delete []h_x;
        break;
    }
    default:
        FILE_LOG(logERROR) << "[Pattern Generation X Error] QP Solver is not set!";
        return false;
    }

    ///////// Solve the problem for Y-direction ////////////////////////////////////////////////////////////////
    VectorNd Xy_MPC = VectorNd::Zero(4*N,1);
    VectorNd Uy_MPC = VectorNd::Zero(N,1);

    switch(QP_yCoMRef4Walking.WhichSolver()) {
    case SolverIsQuadProg:
    {
        VectorNd QP_sol = QP_yCoMRef4Walking.solve_QP();
        Xy_MPC = QP_sol.segment(0,4*N);
        Uy_MPC = QP_sol.segment(4*N,N);
        break;
    }
    case SolverIsQPSwift:
    {
        qp_int n_var = QP_yCoMRef4Walking.NUMCOLS;
        qp_int n_equ = QP_yCoMRef4Walking.NUMEQ;
        qp_int n_inequ = QP_yCoMRef4Walking.NUMINEQ;

        qp_int   P_nnz,A_nnz,G_nnz;
        qp_real  *P_x = NULL;
        qp_int   *P_i = NULL,*P_p = NULL;
        qp_real  *q_x = NULL;
        qp_real  *A_x = NULL;
        qp_int   *A_i = NULL, *A_p = NULL;
        qp_real  *b_x = NULL;
        qp_real  *G_x = NULL;
        qp_int   *G_i = NULL, *G_p = NULL;
        qp_real  *h_x = NULL;

        // 1-0. Convert cost function to sparse form .
        MatrixNd P = QP_yCoMRef4Walking.P_cost;
        VectorNd q = QP_yCoMRef4Walking.Q_cost;
        P_nnz = QP_yCoMRef4Walking.GetNumberOfNonZero_QPswift(P);
        P_x = new qp_real[P_nnz];
        P_i = new qp_int[P_nnz];
        P_p = new qp_int[n_var+1];
        QP_yCoMRef4Walking.ConvertMatrixA_Full2CCS_QPswift(P,P_x,P_i,P_p);
        q_x = new qp_real[n_var];
        QP_yCoMRef4Walking.ConvertVector2Array_QPswift(q,n_var,q_x);

        // 1-1. Convert equality constraint to sparse form .
        if(n_equ > 0) {
            MatrixNd A = QP_yCoMRef4Walking.A_equ;
            MatrixNd b = QP_yCoMRef4Walking.B_equ;
            A_nnz = QP_yCoMRef4Walking.GetNumberOfNonZero_QPswift(A);
            A_x = new qp_real[A_nnz];
            A_i = new qp_int[A_nnz];
            A_p = new qp_int[n_var+1];
            QP_yCoMRef4Walking.ConvertMatrixA_Full2CCS_QPswift(A,A_x,A_i,A_p);
            b_x = new qp_real[n_equ];
            QP_yCoMRef4Walking.ConvertVector2Array_QPswift(b,n_equ,b_x);
        }

        // 1-2. Convert inequality constraint to sparse form .
        if(n_inequ > 0) {
            MatrixNd G = QP_yCoMRef4Walking.A_inequ;
            MatrixNd h = QP_yCoMRef4Walking.B_inequ;
            G_nnz = QP_yCoMRef4Walking.GetNumberOfNonZero_QPswift(G);
            G_x = new qp_real[G_nnz];
            G_i = new qp_int[G_nnz];
            G_p = new qp_int[n_var+1];
            QP_yCoMRef4Walking.ConvertMatrixA_Full2CCS_QPswift(G,G_x,G_i,G_p);
            h_x = new qp_real[n_inequ];
            QP_yCoMRef4Walking.ConvertVector2Array_QPswift(h,n_inequ,h_x);
        }

        // 2. Set Parameters and solve.
        QPswift      *myQP;
        if(n_inequ != 0) {
            myQP = QP_SETUP(n_var,n_inequ,n_equ,
                            P_p,P_i,P_x,
                            A_p,A_i,A_x,
                            G_p,G_i,G_x,
                            q_x,h_x,b_x,
                            0.0,nullptr);
            QP_SOLVE(myQP);
        } else {
            myQP = QP_SETUP_NoInequ(n_var,n_equ,
                                    P_p,P_i,P_x,
                                    A_p,A_i,A_x,
                                    q_x,b_x,
                                    0.0,nullptr);
            QP_SOLVE_NoInequ(myQP);
        }

        VectorNd QP_sol = VectorNd::Zero(n_var);
        QP_yCoMRef4Walking.ConvertArray2Vector_QPswift(myQP->x, n_var, QP_sol);
        Xy_MPC = QP_sol.segment(0,4*N);
        Uy_MPC = QP_sol.segment(4*N,N);

        MatrixNd A_temp = MatrixNd::Zero(N,5*N);
        VectorNd B_temp = VectorNd::Zero(N);
        for(int i=0;i<N;i++) {
            MatrixNd Ay_CP(1,4);
            Ay_CP << 1.0, 1.0/wn_LIPM, 0.0, 0.0;
            A_temp.block(i,4*i,1,4) = Ay_CP;
            B_temp(i) = CPref_horizon[i](1); // ZMP_y ref
            LIGHT.SavingVector[15+i] = CPref_horizon[i];
        }

        VectorNd temp_vec = A_temp*QP_sol;
        for(int i=0;i<N;i++) {
            LIGHT.SavingVector[15+i](1) = temp_vec(i);
        }

        // 3. destruction allocated memory
        QP_CLEANUP(myQP);
        delete []P_x;  delete []P_i;  delete []P_p;  delete []q_x;
        delete []A_x;  delete []A_i;  delete []A_p;  delete []b_x;
        delete []G_x;  delete []G_i;  delete []G_p;  delete []h_x;
        break;
    }
    default:
        FILE_LOG(logERROR) << "[Pattern Generation Y Error] QP Solver is not set!";
        return false;
    }

    //////// Extract Pattern from MPC Solution ////////////////////////////////////////////////////////////////////////

    u_ddCoM(0) = Ux_MPC(0);
    u_ddCoM(1) = Uy_MPC(0);
    u_ddCoM(2) = 0.0;

    double zeta_X = zeta_X_pattern;
    double wn_X = wn_X_pattern;
    double zeta_Y = zeta_Y_pattern;
    double wn_Y = wn_X_pattern;

    // Predict Next CoM and dCoM
    MatrixNd Adx(4,4),Ady(4,4);
    VectorNd Bdx(4),Bdy(4);
    double dT = SYS_DT;
    Adx <<  1.0-(wn_X*wn_X)*dT*dT/2.0,  dT-(2.0*wn_X*zeta_X)*dT*dT/2.0,  (wn_X*wn_X)*dT*dT/2.0,  (2.0*wn_X*zeta_X)*dT*dT/2.0,
            -(wn_X*wn_X)*dT,        1.0-(2.0*wn_X*zeta_X)*dT,         (wn_X*wn_X)*dT,         (2.0*wn_X*zeta_X)*dT,
            0.0,                             0.0,                    1.0,                           dT,
            0.0,                             0.0,                    0.0,                          1.0;
    Bdx <<                  dT*dT/2.0,                              dT,              dT*dT/2.0,                           dT;
    Ady <<  1.0-(wn_Y*wn_Y)*dT*dT/2.0,  dT-(2.0*wn_Y*zeta_Y)*dT*dT/2.0,  (wn_Y*wn_Y)*dT*dT/2.0,  (2.0*wn_Y*zeta_Y)*dT*dT/2.0,
            -(wn_Y*wn_Y)*dT,        1.0-(2.0*wn_Y*zeta_Y)*dT,         (wn_Y*wn_Y)*dT,         (2.0*wn_Y*zeta_Y)*dT,
            0.0,                             0.0,                    1.0,                           dT,
            0.0,                             0.0,                    0.0,                          1.0;
    Bdy <<                  dT*dT/2.0,                              dT,              dT*dT/2.0,                           dT;

    VectorNd _xnow = VectorNd::Zero(4);
    _xnow << Xpre_CoM(0), dXpre_CoM(0), u_CoM(0), u_dCoM(0);
    VectorNd _ynow = VectorNd::Zero(4);
    _ynow << Xpre_CoM(1), dXpre_CoM(1), u_CoM(1), u_dCoM(1);
    VectorNd _xnext = Adx*_xnow + Bdx*Ux_MPC(0);
    VectorNd _ynext = Ady*_ynow + Bdy*Uy_MPC(0);

    u_dCoM(0) = _xnext(3);
    u_dCoM(1) = _ynext(3);
    u_dCoM(2) = 0.0;

    u_CoM(0) = _xnext(2);
    u_CoM(1) = _ynext(2);
    u_CoM(2) = Pelvis_BaseHeight;

    Xnext_CoM << _xnext(0), _ynext(0), Pelvis_BaseHeight;
    dXnext_CoM << _xnext(1), _ynext(1), 0.0;
    ddXnext_CoM(0) = 2.0*zeta_X*wn_X*(u_dCoM(0)-dXnext_CoM(0)) + wn_X*wn_X*(u_CoM(0)-Xnext_CoM(0));
    ddXnext_CoM(1) = 2.0*zeta_Y*wn_Y*(u_dCoM(1)-dXnext_CoM(1)) + wn_Y*wn_Y*(u_CoM(1)-Xnext_CoM(1));
    ddXnext_CoM(2) = 0.0;

    LIGHT.SavingVector[0] = Xnext_CoM;                               // Predicted CoM Position
    LIGHT.SavingVector[1] = Xpre_CoM;                                // Current CoM Position
    LIGHT.SavingVector[2] = u_CoM;                                   // Commanded CoM Position

    LIGHT.SavingVector[3] = Xref_CP;                                // Reference CP Position
    LIGHT.SavingVector[4] = Xpre_CoM + dXpre_CoM/wn_LIPM;            // Current CP position
    LIGHT.SavingVector[5] = Xnext_CoM + dXnext_CoM/wn_LIPM;          // Predicted CP position

//    Calc_FuturePressureReference_WithWalkPattern(N_horizon, dT_horizon,CPref_horizon,ZMPref_horizon,CoMref_horizon);
    delete []CPref_horizon;
    delete []ZMPref_horizon;
    delete []CoMref_horizon;
}


///////////////////////////////////////////////////////////////////////////////////////////////////

enum CoM_MOVE_RDSP_SMOOTH_STAGESET{
    CoM_MOVE_RDSP_SMOOTH_PARAMETER_SETTING = 0,
    CoM_MOVE_RDSP_SMOOTH_INITIALIZE,
    CoM_MOVE_RDSP_SMOOTH_TERMINATE,
};
int CoM_Move_RDSP_smooth_Stage = CoM_MOVE_RDSP_SMOOTH_PARAMETER_SETTING;

bool LIGHTWholeMotions::CoM_Move_RDSP_smooth(double _TIME, double _Yoffset, bool _CtrlOn)
{
    Vector3d            Xpre_CoM, dXpre_CoM;
    static Vector3d     Xnext_CoM, dXnext_CoM, ddXnext_CoM;
    static Vector3d     u_CoM, u_dCoM, u_ddCoM;

    switch(CoM_Move_RDSP_smooth_Stage) {
    case CoM_MOVE_RDSP_SMOOTH_PARAMETER_SETTING: {
        if (TimeIsZero()) {

            StepDataBuffer_Initialize_LDSP2RDSP(_TIME, 0.0, _Yoffset);
            Xref_CP = LIGHT.Xref_LF2CoM + LIGHT.dXref_LF2CoM/wn_LIPM;
            dXref_CP = zv;

            QP_xCoMRef4Walking.SelectSolver(SolverIsQPSwift);
            QP_yCoMRef4Walking.SelectSolver(SolverIsQPSwift);

            wn_X_pattern = CoM_wn_X;
            zeta_X_pattern = CoM_zeta_X;
            wn_Y_pattern = CoM_wn_Y;
            zeta_Y_pattern = CoM_zeta_Y;
            N_horizon = _N_horizon;
            dT_horizon = _dT_horizon;

            GenerateWalkingPattern_Formulation(LIGHT.Xref_LF2CoM,LIGHT.dXref_LF2CoM,
                                               zv,zv,zv);

            TimeLimitSet(_T_wait); // Wait for 5 seconds before starting
            FILE_LOG(logWARNING) << "[CoMMove2RF] Parameter Setting...";
        }

        TimeUpdate();
        if(TimeCheck()) {
            TimeReset();
            CoM_Move_RDSP_smooth_Stage = CoM_MOVE_RDSP_SMOOTH_INITIALIZE;
        }
        return false;
    }
    case CoM_MOVE_RDSP_SMOOTH_INITIALIZE:
    {
        if (TimeIsZero()) {
            SetMotionBase_LF(true);
            _Xini_temp = LIGHT.Xref_LF2RF;
            Xnext_CoM = LIGHT.Xref_LF2CoM;
            dXnext_CoM = LIGHT.dXref_LF2CoM;
            u_CoM = LIGHT.Xref_LF2CoM;
            u_dCoM = LIGHT.dXref_LF2CoM;
            u_ddCoM = LIGHT.ddXref_LF2CoM;

            FILE_LOG(logWARNING) << "[CoMMove2RF] Moving Start!";
        }

        if(_CtrlOn) {
            Xpre_CoM = LIGHT.Xnow_LF2CoM;
            dXpre_CoM = LIGHT.dXnow_LF2CoM;
        } else {
            Xpre_CoM = Xnext_CoM;
            dXpre_CoM = dXnext_CoM;
        }
        GenerateWalkingPattern_Solve(Xpre_CoM,dXpre_CoM,
                                     u_CoM, u_dCoM, u_ddCoM,
                                     Xnext_CoM,dXnext_CoM,ddXnext_CoM);

        // =============== RF >> CoM Position Trajectory ===============
        Submotion_LF2CoM_Pos(SYS_DT, u_CoM, u_dCoM, u_ddCoM);
        // =============== RF >> LF Position Trajectory ===============
        Submotion_LF2RF_Pos(SYS_DT, _Xini_temp, zv, zv);

        TimeUpdate();
        if(StepDataBuffer_Update()) {
            TimeReset();
            CoM_Move_RDSP_smooth_Stage = CoM_MOVE_RDSP_SMOOTH_TERMINATE;
        }
        return false;
    }
    case CoM_MOVE_RDSP_SMOOTH_TERMINATE:
    {
        if (TimeIsZero()) {
            SetMotionBase_RF(true);
            Xref_CP = Xref_CP + LIGHT.Xref_RF2LF;

            _Xini_temp = LIGHT.Xref_RF2LF;
            Xnext_CoM = Xnext_CoM + LIGHT.Xref_RF2LF;
            u_CoM = u_CoM + LIGHT.Xref_RF2LF;
            u_dCoM = u_dCoM + LIGHT.dXref_RF2LF;
            u_ddCoM = u_ddCoM + LIGHT.ddXref_RF2LF;

            TimeLimitSet(5.0);
        }

        if(_CtrlOn) {
            Xpre_CoM = LIGHT.Xnow_RF2CoM;
            dXpre_CoM = LIGHT.dXnow_RF2CoM;
        } else {
            Xpre_CoM = Xnext_CoM;
            dXpre_CoM = dXnext_CoM;
        }
        GenerateWalkingPattern_Solve(Xpre_CoM,dXpre_CoM,
                                     u_CoM, u_dCoM, u_ddCoM,
                                     Xnext_CoM,dXnext_CoM,ddXnext_CoM);

        // =============== RF >> CoM Position Trajectory ===============
        Submotion_RF2CoM_Pos(SYS_DT, u_CoM, u_dCoM, u_ddCoM);
        // =============== RF >> LF Position Trajectory ===============
        Submotion_RF2LF_Pos(SYS_DT, _Xini_temp, zv, zv);

        TimeUpdate();
        StepDataBuffer_Update();
        if(TimeCheck()||_OPERATION_MODE_CHANGED)
        {
            TimeReset();
            CoM_Move_RDSP_smooth_Stage = CoM_MOVE_RDSP_SMOOTH_PARAMETER_SETTING;
            Submotion_RF2CoM_Pos(SYS_DT, LIGHT.Xref_RF2CoM, zv, zv);
            Submotion_LF2CoM_Pos(SYS_DT, LIGHT.Xref_LF2CoM, zv, zv);
            Submotion_RF2LF_Pos(SYS_DT, LIGHT.Xref_RF2LF, zv, zv);
            Submotion_LF2RF_Pos(SYS_DT, LIGHT.Xref_LF2RF, zv, zv);
            return true;
        }
        return false;
    }
    default:
        break;
    }

    if(_OPERATION_MODE_CHANGED) {
        TimeReset();
        CoM_Move_RDSP_smooth_Stage = CoM_MOVE_RDSP_SMOOTH_PARAMETER_SETTING;
        Submotion_RF2CoM_Pos(SYS_DT, LIGHT.Xref_RF2CoM, zv, zv);
        Submotion_LF2CoM_Pos(SYS_DT, LIGHT.Xref_LF2CoM, zv, zv);
        Submotion_RF2LF_Pos(SYS_DT, LIGHT.Xref_RF2LF, zv, zv);
        Submotion_LF2RF_Pos(SYS_DT, LIGHT.Xref_LF2RF, zv, zv);
        LIGHT.Flag_Error = true;
        FILE_LOG(logERROR) << "[Error] Operation is Changed during Walking!! ";
        return true;
    }
    return false;
}

enum CoM_MOVE_LDSP_SMOOTH_STAGESET{
    CoM_MOVE_LDSP_SMOOTH_PARAMETER_SETTING = 0,
    CoM_MOVE_LDSP_SMOOTH_INITIALIZE,
    CoM_MOVE_LDSP_SMOOTH_TERMINATE,
};
int CoM_Move_LDSP_smooth_Stage = CoM_MOVE_LDSP_SMOOTH_PARAMETER_SETTING;

bool LIGHTWholeMotions::CoM_Move_LDSP_smooth(double _TIME, double _Yoffset, bool _CtrlOn)
{
    static Vector3d     Xpre_CoM, dXpre_CoM;
    static Vector3d     u_CoM, u_dCoM, u_ddCoM;
    static Vector3d     Xnext_CoM, dXnext_CoM, ddXnext_CoM;

    switch(CoM_Move_LDSP_smooth_Stage) {
    case CoM_MOVE_LDSP_SMOOTH_PARAMETER_SETTING:
    {
        if (TimeIsZero()) {

            StepDataBuffer_Initialize_RDSP2LDSP(_TIME, 0.0, _Yoffset);
            Xref_CP = LIGHT.Xref_RF2CoM + LIGHT.dXref_RF2CoM/wn_LIPM;
            dXref_CP = zv;

            QP_xCoMRef4Walking.SelectSolver(SolverIsQPSwift);
            QP_yCoMRef4Walking.SelectSolver(SolverIsQPSwift);

            wn_X_pattern = CoM_wn_X;
            zeta_X_pattern = CoM_zeta_X;
            wn_Y_pattern = CoM_wn_Y;
            zeta_Y_pattern = CoM_zeta_Y;
            N_horizon = _N_horizon;
            dT_horizon = _dT_horizon;

            GenerateWalkingPattern_Formulation(LIGHT.Xref_RF2CoM,LIGHT.dXref_RF2CoM,
                                               zv,zv,zv);

            TimeLimitSet(_T_wait); // Wait for 5 seconds before starting
            FILE_LOG(logWARNING) << "[CoMMove2LF] Parameter Setting...";
        }

        TimeUpdate();
        if(TimeCheck()) {
            TimeReset();
            CoM_Move_LDSP_smooth_Stage = CoM_MOVE_LDSP_SMOOTH_INITIALIZE;
        }
        return false;
    }
    case CoM_MOVE_LDSP_SMOOTH_INITIALIZE:
    {
        if (TimeIsZero()) {
            SetMotionBase_RF(true);
            _Xini_temp = LIGHT.Xref_RF2LF;
            Xnext_CoM = LIGHT.Xref_RF2CoM;
            dXnext_CoM = LIGHT.dXref_RF2CoM;
            u_CoM = LIGHT.Xref_RF2CoM;
            u_dCoM = LIGHT.dXref_RF2CoM;
            u_ddCoM = LIGHT.ddXref_RF2CoM;

            FILE_LOG(logWARNING) << "[CoMMove2LF] Moving Start!";
        }

        if(_CtrlOn) {
            Xpre_CoM = LIGHT.Xnow_RF2CoM;
            dXpre_CoM = LIGHT.dXnow_RF2CoM;
        } else {
            Xpre_CoM = Xnext_CoM;
            dXpre_CoM = dXnext_CoM;
        }
        GenerateWalkingPattern_Solve(Xpre_CoM,dXpre_CoM,
                                     u_CoM, u_dCoM, u_ddCoM,
                                     Xnext_CoM,dXnext_CoM,ddXnext_CoM);


        // =============== Global >> Pelvis Orientation and Position Trajectory ===============
        Submotion_Global2Pel_Ori(TimeNow(), TimeLimit(),I3);
        // =============== Global >> RF Orientation Trajectory ===============
        Submotion_Pelvis2RF_Ori(TimeNow(), TimeLimit(),I3);
        // =============== Global >> LF Orientation Trajectory ===============
        Submotion_Pelvis2LF_Ori(TimeNow(), TimeLimit(),I3);
        // =============== RF >> CoM Position Trajectory ===============
        Submotion_RF2CoM_Pos(SYS_DT, u_CoM, u_dCoM, u_ddCoM);
        // =============== RF >> LF Position Trajectory ===============
        Submotion_RF2LF_Pos(SYS_DT, _Xini_temp, zv, zv);

        TimeUpdate();
        if(StepDataBuffer_Update()) {
            TimeReset();
            CoM_Move_LDSP_smooth_Stage = CoM_MOVE_LDSP_SMOOTH_TERMINATE;
        }
        return false;
    }
    case CoM_MOVE_LDSP_SMOOTH_TERMINATE:
    {
        if (TimeIsZero()) {
            SetMotionBase_LF(true);
            Xref_CP = Xref_CP + LIGHT.Xref_LF2RF;

            _Xini_temp = LIGHT.Xref_LF2RF;
            Xnext_CoM = Xnext_CoM + LIGHT.Xref_LF2RF;
            u_CoM = u_CoM + LIGHT.Xref_LF2RF;
            u_dCoM = u_dCoM + LIGHT.dXref_LF2RF;
            u_ddCoM = u_ddCoM + LIGHT.ddXref_LF2RF;

            TimeLimitSet(5.0);
        }

        if(_CtrlOn) {
            Xpre_CoM = LIGHT.Xnow_LF2CoM;
            dXpre_CoM = LIGHT.dXnow_LF2CoM;
        } else {
            Xpre_CoM = Xnext_CoM;
            dXpre_CoM = dXnext_CoM;
        }
        GenerateWalkingPattern_Solve(Xpre_CoM,dXpre_CoM,
                                     u_CoM, u_dCoM, u_ddCoM,
                                     Xnext_CoM,dXnext_CoM,ddXnext_CoM);


        // =============== RF >> CoM Position Trajectory ===============
        Submotion_LF2CoM_Pos(SYS_DT, u_CoM, u_dCoM, u_ddCoM);
        // =============== RF >> LF Position Trajectory ===============
        Submotion_LF2RF_Pos(SYS_DT, _Xini_temp, zv, zv);

        TimeUpdate();
        StepDataBuffer_Update();
        if(TimeCheck()||_OPERATION_MODE_CHANGED)
        {
            TimeReset();
            CoM_Move_LDSP_smooth_Stage = CoM_MOVE_LDSP_SMOOTH_PARAMETER_SETTING;
            Submotion_RF2CoM_Pos(SYS_DT, LIGHT.Xref_RF2CoM, zv, zv);
            Submotion_LF2CoM_Pos(SYS_DT, LIGHT.Xref_LF2CoM, zv, zv);
            Submotion_RF2LF_Pos(SYS_DT, LIGHT.Xref_RF2LF, zv, zv);
            Submotion_LF2RF_Pos(SYS_DT, LIGHT.Xref_LF2RF, zv, zv);
            return true;
        }
        return false;
    }
    default:
        break;
    }

    if(_OPERATION_MODE_CHANGED) {
        TimeReset();
        CoM_Move_LDSP_smooth_Stage = CoM_MOVE_LDSP_SMOOTH_PARAMETER_SETTING;
        Submotion_RF2CoM_Pos(SYS_DT, LIGHT.Xref_RF2CoM, zv, zv);
        Submotion_LF2CoM_Pos(SYS_DT, LIGHT.Xref_LF2CoM, zv, zv);
        Submotion_RF2LF_Pos(SYS_DT, LIGHT.Xref_RF2LF, zv, zv);
        Submotion_LF2RF_Pos(SYS_DT, LIGHT.Xref_LF2RF, zv, zv);
        LIGHT.Flag_Error = true;
        FILE_LOG(logERROR) << "[Error] Operation is Changed during Walking!! ";
        return true;
    }
    return false;

}

enum RFSWINGUP_STAGESET{
    RFSWINGUP_PARAMETER_SETTING = 0,
    RFSWINGUP_INITIALIZE,
    RFSWINGUP_SWING,
    RFSWINGUP_TERMINATE,
};
int RFSwingUp_Stage = RFSWINGUP_PARAMETER_SETTING;
bool LIGHTWholeMotions::RFSwingUp_Dynamic(double _TIME, Vector3d _ZMP_OFFSET, double _Yaw_LF2RF)
{
    static Vector3d     Xpre_CoM, dXpre_CoM;
    static Vector3d     u_CoM, u_dCoM, u_ddCoM;
    static Vector3d     Xnext_CoM, dXnext_CoM, ddXnext_CoM;

    switch(RFSwingUp_Stage) {
    case RFSWINGUP_PARAMETER_SETTING: // Parameter Initializing and wait a moment before starting
    {
        if (TimeIsZero()) {

            _T_SWING = 1.00;

            StepDataBuffer_Initialize_RFSwingUp(_TIME, _ZMP_OFFSET(0), _ZMP_OFFSET(1));
            Xref_CP = LIGHT.Xref_RF2CoM + LIGHT.dXref_RF2CoM/wn_LIPM;
            dXref_CP = zv;

            QP_xCoMRef4Walking.SelectSolver(SolverIsQPSwift);
            QP_yCoMRef4Walking.SelectSolver(SolverIsQPSwift);

            wn_X_pattern = CoM_wn_X;
            zeta_X_pattern = CoM_zeta_X;
            wn_Y_pattern = CoM_wn_Y;
            zeta_Y_pattern = CoM_zeta_Y;
            N_horizon = _N_horizon;
            dT_horizon = _dT_horizon;

            GenerateWalkingPattern_Formulation(LIGHT.Xref_RF2CoM,LIGHT.dXref_RF2CoM,
                                               zv,zv,zv);

            TimeLimitSet(_T_wait); // Wait for seconds before starting
            FILE_LOG(logWARNING) << "[RFSwingUp] Parameter Setting...";
        }

        TimeUpdate();
        if(TimeCheck()) {
            TimeReset();
            RFSwingUp_Stage = RFSWINGUP_INITIALIZE;
        }
        return false;
    }
    case RFSWINGUP_INITIALIZE: // CoM Move to Left Foot
    {
        if (TimeIsZero()) {
            SetMotionBase_RF(true);
            _Xini_temp = LIGHT.Xref_RF2LF;
            _Rini_temp = LIGHT.Rref_RF.transpose()*LIGHT.Rref_Pel;
            _Rini_temp2 = LIGHT.Rref_RF.transpose()*LIGHT.Rref_LF;
            Xnext_CoM = LIGHT.Xref_RF2CoM;
            dXnext_CoM = LIGHT.dXref_RF2CoM;
            u_CoM = LIGHT.Xref_RF2CoM;
            u_dCoM = LIGHT.dXref_RF2CoM;
            u_ddCoM = LIGHT.ddXref_RF2CoM;

            TimeLimitSet(SDB[0].t_STEP);
            FILE_LOG(logWARNING) << "[RFSwingUp] CoM Move to LF!";
        }

        //        Xpre_CoM = LIGHT.Xnow_RF2CoM;
        //        dXpre_CoM = LIGHT.dXnow_RF2CoM;
        Xpre_CoM = Xnext_CoM;
        dXpre_CoM = dXnext_CoM;
        GenerateWalkingPattern_Solve(Xpre_CoM,dXpre_CoM,
                                     u_CoM, u_dCoM, u_ddCoM,
                                     Xnext_CoM,dXnext_CoM,ddXnext_CoM);

        // =============== Global >> Pelvis Orientation and Position Trajectory ===============
        Submotion_Global2Pel_Ori(0.0, 0.0, _Rini_temp);
        // =============== Global >> RF Orientation Trajectory ===============
        Submotion_Global2RF_Ori(0.0, 0.0, I3);
        // =============== Global >> LF Orientation Trajectory ===============
        Submotion_Global2LF_Ori(0.0, 0.0, _Rini_temp2);
        // =============== RF >> CoM Position Trajectory ===============
        Submotion_RF2CoM_Pos(SYS_DT, u_CoM, u_dCoM, u_ddCoM);
        // =============== RF >> LF Position Trajectory ===============
        Submotion_RF2LF_Pos(SYS_DT, _Xini_temp, zv, zv);

        TimeUpdate();
        if(StepDataBuffer_Update()) {
            TimeReset();
            RFSwingUp_Stage = RFSWINGUP_SWING;
        }
        return false;
    }
    case RFSWINGUP_SWING: // If the ZMP arrives to Left Foot, Right Foot Swing Up
    {
        if (TimeIsZero()) {
            SetMotionBase_LF(false); // RDSP > LSSP
            Xref_CP = Xref_CP + LIGHT.Xref_LF2RF;

            Xnext_CoM = Xnext_CoM + LIGHT.Xref_LF2RF;
            u_CoM = u_CoM + LIGHT.Xref_LF2RF;
            u_dCoM = u_dCoM + LIGHT.dXref_LF2RF;
            u_ddCoM = u_ddCoM + LIGHT.ddXref_LF2RF;

            TimeLimitSet(_T_SWING);
            FILE_LOG(logWARNING) << "[RFSwingUp] Right Foot SwingUp!";
        }

        //        Xpre_CoM = LIGHT.Xnow_LF2CoM;
        //        dXpre_CoM = LIGHT.dXnow_LF2CoM;
        Xpre_CoM = Xnext_CoM;
        dXpre_CoM = dXnext_CoM;
        GenerateWalkingPattern_Solve(Xpre_CoM,dXpre_CoM,
                                     u_CoM, u_dCoM, u_ddCoM,
                                     Xnext_CoM,dXnext_CoM,ddXnext_CoM);

        // =============== Global >> Pelvis Orientation Trajectory ===============
        Submotion_Global2Pel_Ori(TimeNow(), TimeLimit(),RotZ(_Yaw_LF2RF/2.0));
        // =============== Global >> RF Orientation Trajectory ===============
        Submotion_Global2RF_Ori(TimeNow(), TimeLimit(),RotZ(_Yaw_LF2RF));
        // =============== Global >> LF Orientation Trajectory ===============
        Submotion_Global2LF_Ori(0.0, 0.0, I3);
        // =============== LF >> CoM Position Trajectory ===============
        Submotion_LF2CoM_Pos(SYS_DT, u_CoM, u_dCoM, u_ddCoM);
        // =============== LF >> RF Position Trajectory ===============
        Vector3d Xdes_LF2RF = Vector3d(0.0,-0.20,0.10);
        Submotion_LF2RFSwingUp(_T_SWING, Xdes_LF2RF);

        TimeUpdate();
        StepDataBuffer_Update();
        if(TimeCheck()) {
            TimeReset();
            Submotion_LF2CoM_Pos(SYS_DT, LIGHT.Xdes_LF2CoM, zv, zv);
            RFSwingUp_Stage = RFSWINGUP_TERMINATE;
        }
        return false;
    }
    case RFSWINGUP_TERMINATE: // After right foot swings up, keep balance
    {
        if(TimeIsZero()) {
            TimeLimitSet(3.0);
        }

        //        Xpre_CoM = LIGHT.Xnow_LF2CoM;
        //        dXpre_CoM = LIGHT.dXnow_LF2CoM;
        Xpre_CoM = Xnext_CoM;
        dXpre_CoM = dXnext_CoM;
        GenerateWalkingPattern_Solve(Xpre_CoM,dXpre_CoM,
                                     u_CoM, u_dCoM, u_ddCoM,
                                     Xnext_CoM,dXnext_CoM,ddXnext_CoM);

        // =============== LF >> CoM Position Trajectory ===============
        Submotion_LF2CoM_Pos(SYS_DT, u_CoM, u_dCoM, u_ddCoM);

        TimeUpdate();
        if(_OPERATION_MODE_CHANGED||TimeCheck()) {
            TimeReset();
            RFSwingUp_Stage = RFSWINGUP_PARAMETER_SETTING;
            Submotion_LF2CoM_Pos(SYS_DT, LIGHT.Xdes_LF2CoM, zv, zv);
            FILE_LOG(logSUCCESS) << " RightFoot Swing Up Motion is done!! ";
            return true;
        }
        return false;
    }
    default:
        break;
    }

    if(_OPERATION_MODE_CHANGED) {
        TimeReset();
        RFSwingUp_Stage = RFSWINGUP_PARAMETER_SETTING;
        Submotion_RF2CoM_Pos(SYS_DT, LIGHT.Xref_RF2CoM, zv, zv);
        Submotion_RF2LF_Pos(SYS_DT, LIGHT.Xref_RF2LF, zv, zv);
        Submotion_LF2CoM_Pos(SYS_DT, LIGHT.Xref_LF2CoM, zv, zv);
        Submotion_LF2RF_Pos(SYS_DT, LIGHT.Xref_LF2RF, zv, zv);
        LIGHT.Flag_Error = true;
        FILE_LOG(logERROR) << "[Error] Operation is Changed during RF SwingUp!! ";
        return true;
    }
    return false;
}

enum RFSWINGDOWN_STAGESET{
    RFSWINGDOWN_PARAMETER_SETTING = 0,
    RFSWINGDOWN_SWING,
    RFSWINGDOWN_STEP,
    RFSWINGDOWN_TERMINATE,
};
int RFSwingDown_Stage = RFSWINGDOWN_PARAMETER_SETTING;
bool LIGHTWholeMotions::RFSwingDown_Dynamic(double _TIME, Vector3d _Xdes_LF2RF, double _Yaw_LF2RF)
{
    static Vector3d     Xpre_CoM, dXpre_CoM;
    static Vector3d     u_CoM, u_dCoM, u_ddCoM;
    static Vector3d     Xnext_CoM, dXnext_CoM, ddXnext_CoM;

    switch(RFSwingDown_Stage) {
    case RFSWINGDOWN_PARAMETER_SETTING: // Parameter Initializing and wait a moment before starting
    {
        if (TimeIsZero()) {

            _T_DSP = 0.20;
            _T_SWING = _TIME - _T_DSP;

            StepDataBuffer_Initialize_RFSwingDown(_TIME, _T_DSP, _Xdes_LF2RF(0), _Xdes_LF2RF(1), _Yaw_LF2RF);
            Xref_CP = LIGHT.Xref_LF2CoM + LIGHT.dXref_LF2CoM/wn_LIPM;
            dXref_CP = zv;

            QP_xCoMRef4Walking.SelectSolver(SolverIsQPSwift);
            QP_yCoMRef4Walking.SelectSolver(SolverIsQPSwift);

            wn_X_pattern = CoM_wn_X;
            zeta_X_pattern = CoM_zeta_X;
            wn_Y_pattern = CoM_wn_Y;
            zeta_Y_pattern = CoM_zeta_Y;
            N_horizon = _N_horizon;
            dT_horizon = _dT_horizon;

            GenerateWalkingPattern_Formulation(LIGHT.Xref_LF2CoM,LIGHT.dXref_LF2CoM,
                                               zv,zv,zv);

            TimeLimitSet(_T_wait); // Wait for 5 seconds before starting
            FILE_LOG(logWARNING) << "[RFSwingDown] Parameter Setting...";
        }

        TimeUpdate();
        if(TimeCheck()) {
            TimeReset();
            RFSwingDown_Stage = RFSWINGDOWN_SWING;
        }
        return false;
    }
    case RFSWINGDOWN_SWING: // SwingFoot get ready to land
    {
        if (TimeIsZero()) {
            SetMotionBase_LF(false);
            Xnext_CoM = LIGHT.Xref_LF2CoM;
            dXnext_CoM = LIGHT.dXref_LF2CoM;
            u_CoM = LIGHT.Xref_LF2CoM;
            u_dCoM = LIGHT.dXref_LF2CoM;
            u_ddCoM = LIGHT.ddXref_LF2CoM;

            TimeLimitSet(SDB[0].t_STEP-SDB[0].t_DSP);
            FILE_LOG(logWARNING) << "[RFSwingDown] Right Foot Step Down!";
        }

        //        Xpre_CoM = LIGHT.Xnow_LF2CoM;
        //        dXpre_CoM = LIGHT.dXnow_LF2CoM;
        Xpre_CoM = Xnext_CoM;
        dXpre_CoM = dXnext_CoM;
        GenerateWalkingPattern_Solve(Xpre_CoM,dXpre_CoM,
                                     u_CoM, u_dCoM, u_ddCoM,
                                     Xnext_CoM,dXnext_CoM,ddXnext_CoM);

        // =============== Global >> Pelvis Orientation Trajectory ===============
        Submotion_Global2Pel_Ori(TimeNow(), TimeLimit(),RotZ(_Yaw_LF2RF/2.0));
        // =============== Global >> RF Orientation Trajectory ===============
        Submotion_Global2RF_Ori(TimeNow(), TimeLimit(),RotZ(_Yaw_LF2RF));
        // =============== Global >> LF Orientation Trajectory ===============
        Submotion_Global2LF_Ori(0.0, 0.0, I3);
        // =============== LF >> CoM Position Trajectory ===============
        Submotion_LF2CoM_Pos(SYS_DT, u_CoM, u_dCoM, u_ddCoM);
        // =============== LF >> RF Position Trajectory ===============
        Submotion_LF2RFSwingDown(_T_SWING, SDB[0].X_STEP);

        TimeUpdate();
        if(StepDataBuffer_Update()) {
            TimeReset();
            if(CurFlagDSP) {
                RFSwingDown_Stage = RFSWINGDOWN_STEP;
            } else {
                RFSwingDown_Stage = RFSWINGDOWN_TERMINATE;
            }
        }
        break;
    }
    case RFSWINGDOWN_STEP: // Swing Foot Contact, keep balance
    {
        if(TimeIsZero()) {
            SetMotionBase_LF(true);
            FILE_LOG(logWARNING) << "[RFSwingDown] Right Foot touch the Ground!";
        }

        //        Xpre_CoM = LIGHT.Xnow_LF2CoM;
        //        dXpre_CoM = LIGHT.dXnow_LF2CoM;
        Xpre_CoM = Xnext_CoM;
        dXpre_CoM = dXnext_CoM;
        GenerateWalkingPattern_Solve(Xpre_CoM,dXpre_CoM,
                                     u_CoM, u_dCoM, u_ddCoM,
                                     Xnext_CoM,dXnext_CoM,ddXnext_CoM);

        // =============== LF >> CoM Position Trajectory ===============
        Submotion_LF2CoM_Pos(SYS_DT, u_CoM, u_dCoM, u_ddCoM);
        // =============== LF >> RF Position Trajectory ===============
        Submotion_LF2RF_Pos(SYS_DT, SDB[0].X_STEP, zv, zv);

        TimeUpdate();
        if(StepDataBuffer_Update()) {
            TimeReset();
            RFSwingDown_Stage = RFSWINGDOWN_TERMINATE;
        }
        break;
    }
    case RFSWINGDOWN_TERMINATE:
    {
        if(TimeIsZero()) {
            SetMotionBase_RF(true);

            Xref_CP = Xref_CP + LIGHT.Xref_RF2LF;
            _Xini_temp = LIGHT.Xref_RF2LF;
            Xnext_CoM = Xnext_CoM + LIGHT.Xref_RF2LF;
            u_CoM = u_CoM + LIGHT.Xref_RF2LF;
            u_dCoM = u_dCoM + LIGHT.dXref_RF2LF;
            u_ddCoM = u_ddCoM + LIGHT.ddXref_RF2LF;

            TimeLimitSet(3.0);
            FILE_LOG(logWARNING) << "[RFSwingDown] Terminate step down!";
        }

        //        Xpre_CoM = LIGHT.Xnow_RF2CoM;
        //        dXpre_CoM = LIGHT.dXnow_RF2CoM;
        Xpre_CoM = Xnext_CoM;
        dXpre_CoM = dXnext_CoM;
        GenerateWalkingPattern_Solve(Xpre_CoM,dXpre_CoM,
                                     u_CoM, u_dCoM, u_ddCoM,
                                     Xnext_CoM,dXnext_CoM,ddXnext_CoM);

        // =============== LF >> CoM Position Trajectory ===============
        Submotion_RF2CoM_Pos(SYS_DT, u_CoM, u_dCoM, u_ddCoM);
        // =============== LF >> RF Position Trajectory ===============
        Submotion_RF2LF_Pos(SYS_DT, _Xini_temp, zv, zv);

        TimeUpdate();
        if(_OPERATION_MODE_CHANGED||TimeCheck()) {
            TimeReset();
            RFSwingDown_Stage = RFSWINGDOWN_PARAMETER_SETTING;
            Submotion_RF2CoM_Pos(SYS_DT, LIGHT.Xref_RF2CoM, zv, zv);
            Submotion_RF2LF_Pos(SYS_DT, LIGHT.Xref_RF2LF, zv, zv);
            Submotion_LF2CoM_Pos(SYS_DT, LIGHT.Xref_LF2CoM, zv, zv);
            Submotion_LF2RF_Pos(SYS_DT, LIGHT.Xref_LF2RF, zv, zv);
            FILE_LOG(logSUCCESS) << "[RFSwingDown] Step Down is done!!";
            return true;
        }
        break;
    }
    default:
        break;
    }

    if(_OPERATION_MODE_CHANGED) {
        TimeReset();
        RFSwingDown_Stage = RFSWINGDOWN_PARAMETER_SETTING;
        Submotion_RF2CoM_Pos(SYS_DT, LIGHT.Xref_RF2CoM, zv, zv);
        Submotion_RF2LF_Pos(SYS_DT, LIGHT.Xref_RF2LF, zv, zv);
        Submotion_LF2CoM_Pos(SYS_DT, LIGHT.Xref_LF2CoM, zv, zv);
        Submotion_LF2RF_Pos(SYS_DT, LIGHT.Xref_LF2RF, zv, zv);
        LIGHT.Flag_Error = true;
        FILE_LOG(logERROR) << "[Error] Operation is Changed while RF Step Down!! ";
        return true;
    }
    return false;
}

enum LFSWINGUP_STAGESET{
    LFSWINGUP_PARAMETER_SETTING = 0,
    LFSWINGUP_INITIALIZE,
    LFSWINGUP_SWING,
    LFSWINGUP_TERMINATE,
};
int LFSwingUp_Stage = LFSWINGUP_PARAMETER_SETTING;
bool LIGHTWholeMotions::LFSwingUp_Dynamic(double _TIME, Vector3d _ZMP_OFFSET, double _Yaw_RF2LF)
{   
    static Vector3d     Xpre_CoM, dXpre_CoM;
    static Vector3d     u_CoM, u_dCoM, u_ddCoM;
    static Vector3d     Xnext_CoM, dXnext_CoM, ddXnext_CoM;

    switch(LFSwingUp_Stage) {
    case LFSWINGUP_PARAMETER_SETTING: // Parameter Initializing and wait a moment before starting
    {
        if (TimeIsZero()) {

            _T_DSP = 0.00;
            _T_SWING = 1.0;

            StepDataBuffer_Initialize_LFSwingUp(_TIME, _ZMP_OFFSET(0), _ZMP_OFFSET(1));
            Xref_CP = LIGHT.Xref_LF2CoM + LIGHT.dXref_LF2CoM/wn_LIPM;
            dXref_CP = zv;

            QP_xCoMRef4Walking.SelectSolver(SolverIsQPSwift);
            QP_yCoMRef4Walking.SelectSolver(SolverIsQPSwift);

            wn_X_pattern = CoM_wn_X;
            zeta_X_pattern = CoM_zeta_X;
            wn_Y_pattern = CoM_wn_Y;
            zeta_Y_pattern = CoM_zeta_Y;
            N_horizon = _N_horizon;
            dT_horizon = _dT_horizon;

            GenerateWalkingPattern_Formulation(LIGHT.Xref_LF2CoM,LIGHT.dXref_LF2CoM,
                                               zv,zv,zv);

            TimeLimitSet(_T_wait); // Wait for 5 seconds before starting
            FILE_LOG(logWARNING) << "[LFSwingUp] Parameter Setting...";
        }

        TimeUpdate();
        if(TimeCheck()) {
            TimeReset();
            LFSwingUp_Stage = LFSWINGUP_INITIALIZE;
        }
        return false;
    }
    case LFSWINGUP_INITIALIZE: // CoM Move to Right Foot
    {
        if (TimeIsZero()) {
            SetMotionBase_LF(true);
            _Xini_temp = LIGHT.Xref_LF2RF;
            _Rini_temp = LIGHT.Rref_LF.transpose()*LIGHT.Rref_Pel;
            _Rini_temp2 = LIGHT.Rref_LF.transpose()*LIGHT.Rref_RF;
            Xnext_CoM = LIGHT.Xref_LF2CoM;
            dXnext_CoM = LIGHT.dXref_LF2CoM;
            u_CoM = LIGHT.Xref_LF2CoM;
            u_dCoM = LIGHT.dXref_LF2CoM;
            u_ddCoM = LIGHT.ddXref_LF2CoM;

            TimeLimitSet(SDB[0].t_STEP);
            FILE_LOG(logWARNING) << "[LFSwingUp] CoM Move to RF!";
        }

        //        Xpre_CoM = LIGHT.Xnow_LF2CoM;
        //        dXpre_CoM = LIGHT.dXnow_LF2CoM;
        Xpre_CoM = Xnext_CoM;
        dXpre_CoM = dXnext_CoM;
        GenerateWalkingPattern_Solve(Xpre_CoM,dXpre_CoM,
                                     u_CoM, u_dCoM, u_ddCoM,
                                     Xnext_CoM,dXnext_CoM,ddXnext_CoM);

        // =============== Global >> Pelvis Orientation and Position Trajectory ===============
        Submotion_Global2Pel_Ori(0.0, 0.0,_Rini_temp);
        // =============== Global >> RF Orientation Trajectory ===============
        Submotion_Global2RF_Ori(0.0, 0.0, _Rini_temp2);
        // =============== Global >> LF Orientation Trajectory ===============
        Submotion_Global2LF_Ori(0.0, 0.0, I3);
        // =============== RF >> CoM Position Trajectory ===============
        Submotion_LF2CoM_Pos(SYS_DT, u_CoM, u_dCoM, u_ddCoM);
        // =============== RF >> LF Position Trajectory ===============
        Submotion_LF2RF_Pos(SYS_DT, _Xini_temp, zv, zv);

        TimeUpdate();
        if(StepDataBuffer_Update()) {
            TimeReset();
            LFSwingUp_Stage = LFSWINGUP_SWING;
        }
        return false;
    }
    case LFSWINGUP_SWING: // Left foot swings up
    {
        if (TimeIsZero()) {
            SetMotionBase_RF(false); // RDSP > RSSP
            Xref_CP = Xref_CP + LIGHT.Xref_RF2LF;

            Xnext_CoM = Xnext_CoM + LIGHT.Xref_RF2LF;
            u_CoM = u_CoM + LIGHT.Xref_RF2LF;
            u_dCoM = u_dCoM + LIGHT.dXref_RF2LF;
            u_ddCoM = u_ddCoM + LIGHT.ddXref_RF2LF;

            TimeLimitSet(_T_SWING);
            FILE_LOG(logWARNING) << "[LFSwingUp] Left Foot SwingUp!";
        }

        //        Xpre_CoM = LIGHT.Xnow_RF2CoM;
        //        dXpre_CoM = LIGHT.dXnow_RF2CoM;
        Xpre_CoM = Xnext_CoM;
        dXpre_CoM = dXnext_CoM;
        GenerateWalkingPattern_Solve(Xpre_CoM,dXpre_CoM,
                                     u_CoM, u_dCoM, u_ddCoM,
                                     Xnext_CoM,dXnext_CoM,ddXnext_CoM);

        // =============== Global >> Pelvis Orientation Trajectory ===============
        Submotion_Global2Pel_Ori(TimeNow(), TimeLimit(), RotZ(_Yaw_RF2LF/2.0));
        // =============== Global >> RF Orientation Trajectory ===============
        Submotion_Global2RF_Ori(0.0, 0.0, I3);
        // =============== Global >> RF Orientation Trajectory ===============
        Submotion_Global2LF_Ori(TimeNow(), TimeLimit(),RotZ(_Yaw_RF2LF));
        // =============== RF >> CoM Position Trajectory ===============
        Submotion_RF2CoM_Pos(SYS_DT, u_CoM, u_dCoM, u_ddCoM);
        // =============== LF >> RF Position Trajectory ===============
        Vector3d Xdes_RF2LF = Vector3d(0.0,0.20,0.10);
        Submotion_RF2LFSwingUp(_T_SWING, Xdes_RF2LF);

        TimeUpdate();
        StepDataBuffer_Update();
        if(TimeCheck()) {
            TimeReset();
            Submotion_RF2CoM_Pos(SYS_DT, LIGHT.Xdes_RF2CoM, zv, zv);
            LFSwingUp_Stage = LFSWINGUP_TERMINATE;
        }
        return false;
    }
    case LFSWINGUP_TERMINATE: // After left foot swings up, keep balance
    {
        if(TimeIsZero()) {
            TimeLimitSet(3.0);
        }

        Xpre_CoM = Xnext_CoM;
        dXpre_CoM = dXnext_CoM;
        GenerateWalkingPattern_Solve(Xpre_CoM,dXpre_CoM,
                                     u_CoM, u_dCoM, u_ddCoM,
                                     Xnext_CoM,dXnext_CoM,ddXnext_CoM);

        // =============== RF >> CoM Position Trajectory ===============
        Submotion_RF2CoM_Pos(SYS_DT, u_CoM, u_dCoM, u_ddCoM);

        TimeUpdate();
        if(_OPERATION_MODE_CHANGED||TimeCheck()) {
            TimeReset();
            LFSwingUp_Stage = LFSWINGUP_PARAMETER_SETTING;
            Submotion_RF2CoM_Pos(SYS_DT, LIGHT.Xdes_RF2CoM, zv, zv);
            FILE_LOG(logSUCCESS) << " LeftFoot Swing Up Motion is done!! ";
            return true;
        }
        return false;
    }
    default:
        break;
    }

    return false;
}

enum LFSWINGDOWN_STAGESET{
    LFSWINGDOWN_PARAMETER_SETTING = 0,
    LFSWINGDOWN_SWING,
    LFSWINGDOWN_STEP,
    LFSWINGDOWN_TERMINATE,
};
int LFSwingDown_Stage = LFSWINGDOWN_PARAMETER_SETTING;
bool LIGHTWholeMotions::LFSwingDown_Dynamic(double _TIME, Vector3d _Xdes_RF2LF, double _Yaw_RF2LF)
{
    static Vector3d     Xpre_CoM, dXpre_CoM;
    static Vector3d     u_CoM, u_dCoM, u_ddCoM;
    static Vector3d     Xnext_CoM, dXnext_CoM, ddXnext_CoM;

    switch(LFSwingDown_Stage) {
    case LFSWINGDOWN_PARAMETER_SETTING: // Parameter Initializing and wait a moment before starting
    {
        if (TimeIsZero()) {

            _T_DSP = 0.20;
            _T_SWING = _TIME - _T_DSP;

            StepDataBuffer_Initialize_LFSwingDown(_TIME, _T_DSP, _Xdes_RF2LF(0), _Xdes_RF2LF(1), _Yaw_RF2LF);
            Xref_CP = LIGHT.Xref_RF2CoM + LIGHT.dXref_RF2CoM/wn_LIPM;
            dXref_CP = zv;

            QP_xCoMRef4Walking.SelectSolver(SolverIsQPSwift);
            QP_yCoMRef4Walking.SelectSolver(SolverIsQPSwift);

            wn_X_pattern = CoM_wn_X;
            zeta_X_pattern = CoM_zeta_X;
            wn_Y_pattern = CoM_wn_Y;
            zeta_Y_pattern = CoM_zeta_Y;
            N_horizon = _N_horizon;
            dT_horizon = _dT_horizon;

            GenerateWalkingPattern_Formulation(LIGHT.Xref_RF2CoM,LIGHT.dXref_RF2CoM,
                                               zv,zv,zv);

            TimeLimitSet(_T_wait); // Wait for 5 seconds before starting
            FILE_LOG(logWARNING) << "[LFSwingDown] Parameter Setting...";
        }

        TimeUpdate();
        if(TimeCheck()) {
            TimeReset();
            LFSwingDown_Stage = LFSWINGDOWN_SWING;
        }
        return false;
    }
    case LFSWINGDOWN_SWING: // SwingFoot get ready to land
    {
        if (TimeIsZero()) {
            SetMotionBase_RF(false);
            Xnext_CoM = LIGHT.Xref_RF2CoM;
            dXnext_CoM = LIGHT.dXref_RF2CoM;
            u_CoM = LIGHT.Xref_RF2CoM;
            u_dCoM = LIGHT.dXref_RF2CoM;
            u_ddCoM = LIGHT.ddXref_RF2CoM;

            TimeLimitSet(SDB[0].t_STEP-SDB[0].t_DSP);
            FILE_LOG(logWARNING) << "[LFSwingDown] Left Foot Step Down!";
        }

        //        Xpre_CoM = LIGHT.Xnow_RF2CoM;
        //        dXpre_CoM = LIGHT.dXnow_RF2CoM;
        Xpre_CoM = Xnext_CoM;
        dXpre_CoM = dXnext_CoM;
        GenerateWalkingPattern_Solve(Xpre_CoM,dXpre_CoM,
                                     u_CoM, u_dCoM, u_ddCoM,
                                     Xnext_CoM,dXnext_CoM,ddXnext_CoM);

        // =============== Global >> Pelvis Orientation and Position Trajectory ===============
        Submotion_Global2Pel_Ori(TimeNow(), TimeLimit(),RotZ(_Yaw_RF2LF/2.0));
        // =============== Global >> RF Orientation Trajectory ===============
        Submotion_Global2RF_Ori(0.0, 0.0, I3);
        // =============== Global >> LF Orientation Trajectory ===============
        Submotion_Global2LF_Ori(TimeNow(), TimeLimit(),RotZ(_Yaw_RF2LF));
        // =============== LF >> CoM Position Trajectory ===============
        Submotion_RF2CoM_Pos(SYS_DT, u_CoM, u_dCoM, u_ddCoM);
        // =============== LF >> RF Position Trajectory ===============
        Submotion_RF2LFSwingDown(_T_SWING, SDB[0].X_STEP);

        TimeUpdate();
        if(StepDataBuffer_Update()) {
            TimeReset();
            if(CurFlagDSP) {
                LFSwingDown_Stage = LFSWINGDOWN_STEP;
            } else {
                LFSwingDown_Stage = LFSWINGDOWN_TERMINATE;
            }
        }
        break;
    }
    case LFSWINGDOWN_STEP: // Swing Foot Contact, keep balance
    {
        if(TimeIsZero()) {
            SetMotionBase_RF(true);
            FILE_LOG(logWARNING) << "[LFSwingDown] Left Foot touch the Ground!";
        }

        //        Xpre_CoM = LIGHT.Xnow_LF2CoM;
        //        dXpre_CoM = LIGHT.dXnow_LF2CoM;
        Xpre_CoM = Xnext_CoM;
        dXpre_CoM = dXnext_CoM;
        GenerateWalkingPattern_Solve(Xpre_CoM,dXpre_CoM,
                                     u_CoM, u_dCoM, u_ddCoM,
                                     Xnext_CoM,dXnext_CoM,ddXnext_CoM);

        // =============== RF >> CoM Position Trajectory ===============
        Submotion_RF2CoM_Pos(SYS_DT, u_CoM, u_dCoM, u_ddCoM);
        // =============== RF >> LF Position Trajectory ===============
        Submotion_RF2LF_Pos(SYS_DT, SDB[0].X_STEP, zv, zv);

        TimeUpdate();
        if(StepDataBuffer_Update()) {
            TimeReset();
            LFSwingDown_Stage = LFSWINGDOWN_TERMINATE;
        }
        break;
    }
    case LFSWINGDOWN_TERMINATE:
    {
        if(TimeIsZero()) {
            SetMotionBase_LF(true);
            Xref_CP = Xref_CP + LIGHT.Xref_LF2RF;

            _Xini_temp = LIGHT.Xref_LF2RF;
            Xnext_CoM = Xnext_CoM + LIGHT.Xref_LF2RF;
            u_CoM = u_CoM + LIGHT.Xref_LF2RF;
            u_dCoM = u_dCoM + LIGHT.dXref_LF2RF;
            u_ddCoM = u_ddCoM + LIGHT.ddXref_LF2RF;

            TimeLimitSet(3.0);
            FILE_LOG(logWARNING) << "[LFSwingDown] Terminate step down!";
        }

        //        Xpre_CoM = LIGHT.Xnow_LF2CoM;
        //        dXpre_CoM = LIGHT.dXnow_LF2CoM;
        Xpre_CoM = Xnext_CoM;
        dXpre_CoM = dXnext_CoM;
        GenerateWalkingPattern_Solve(Xpre_CoM,dXpre_CoM,
                                     u_CoM, u_dCoM, u_ddCoM,
                                     Xnext_CoM,dXnext_CoM,ddXnext_CoM);

        // =============== RF >> CoM Position Trajectory ===============
        Submotion_LF2CoM_Pos(SYS_DT, u_CoM, u_dCoM, u_ddCoM);
        // =============== RF >> LF Position Trajectory ===============
        Submotion_LF2RF_Pos(SYS_DT, _Xini_temp, zv, zv);

        TimeUpdate();
        if(_OPERATION_MODE_CHANGED||TimeCheck()) {
            TimeReset();
            LFSwingDown_Stage = LFSWINGDOWN_PARAMETER_SETTING;
            Submotion_RF2CoM_Pos(SYS_DT, LIGHT.Xref_RF2CoM, zv, zv);
            Submotion_RF2LF_Pos(SYS_DT, LIGHT.Xref_RF2LF, zv, zv);
            Submotion_LF2CoM_Pos(SYS_DT, LIGHT.Xref_LF2CoM, zv, zv);
            Submotion_LF2RF_Pos(SYS_DT, LIGHT.Xref_LF2RF, zv, zv);
            FILE_LOG(logSUCCESS) << "[LFSwingDown] Left Foot Step Down is done!!";
            return true;
        }
        break;
    }
    default:
        break;
    }

    if(_OPERATION_MODE_CHANGED) {
        TimeReset();
        LFSwingDown_Stage = LFSWINGDOWN_PARAMETER_SETTING;
        Submotion_RF2CoM_Pos(SYS_DT, LIGHT.Xref_RF2CoM, zv, zv);
        Submotion_RF2LF_Pos(SYS_DT, LIGHT.Xref_RF2LF, zv, zv);
        Submotion_LF2CoM_Pos(SYS_DT, LIGHT.Xref_LF2CoM, zv, zv);
        Submotion_LF2RF_Pos(SYS_DT, LIGHT.Xref_LF2RF, zv, zv);
        LIGHT.Flag_Error = true;
        FILE_LOG(logERROR) << "[Error] Operation is Changed while LF Step Down!! ";
        return true;
    }
    return false;
}


enum Walk_STAGESET{
    Walk_PARAMETER_SETTING = 0,
    Walk_INITIALIZE,
    Walk_RFSWING,
    Walk_RFLANDING,
    Walk_LFSWING,
    Walk_LFLANDING,
    Walk_FINISH
};
int Walk_CurrentStage = Walk_PARAMETER_SETTING;

bool LIGHTWholeMotions::Walk(double _StepTime, double _DSPTime, double _StanceWidth, int _StepNumber,
                             double _X_Offset, double _Y_Offset, double _Yaw_Offset,
                             double _X_Step, double _Y_Step, double _Yaw_Step, double _StanceOffset,
                             bool _Flag_Terminate, int &CurStepNumber)
{
    static int          CurStepBufferNumber;

    Vector3d     Xpre_CoM, dXpre_CoM;
    static Vector3d     Xnext_CoM, dXnext_CoM, ddXnext_CoM;
    static Vector3d     u_CoM, u_dCoM, u_ddCoM;

    double Z_SwingUp     = 0.05;
    double StepTime      = _StepTime;
    double DSPTime       = _DSPTime;
    double StanceWidth    = _StanceWidth;
    double X_Offset      = _X_Offset;
    double Y_Offset      = _Y_Offset;
    double Yaw_Offset    = _Yaw_Offset;
    double X_Step        = _X_Step;
    double Y_Step        = _Y_Step;
    double Yaw_Step      = _Yaw_Step;
    double StanceOffset   = _StanceOffset;

    switch(Walk_CurrentStage) {
    case Walk_PARAMETER_SETTING: {
        if(_StepNumber<2) {
            FILE_LOG(logERROR) << "Step Number should be set over 2 step!";
            return true;
        } else {
            if (TimeIsZero()) {
                IsWalking = true;

                StepTime      = _StepTime;
                DSPTime       = _DSPTime;
                StanceWidth    = _StanceWidth;
                X_Offset      = _X_Offset;
                Y_Offset      = _Y_Offset;
                Yaw_Offset    = _Yaw_Offset;
                //                X_Step        = _X_Step;
                //                Y_Step        = _Y_Step;
                //                Yaw_Step      = _Yaw_Step;

                //                if(X_Step > 0.25) {
                //                    X_Step = 0.25;
                //                    FILE_LOG(logERROR) << "X-direction Step Saturation!!";
                //                    FILE_LOG(logERROR) << "X-direction Step Saturation!!";
                //                    FILE_LOG(logERROR) << "X-direction Step Saturation!!";
                //                } else if(X_Step < -0.25) {
                //                    X_Step = -0.25;
                //                    FILE_LOG(logERROR) << "X-direction Step Saturation!!";
                //                    FILE_LOG(logERROR) << "X-direction Step Saturation!!";
                //                    FILE_LOG(logERROR) << "X-direction Step Saturation!!";
                //                }

                //                if(Y_Step > 0.10) {
                //                    Y_Step = 0.10;
                //                    FILE_LOG(logERROR) << "Y-direction Step Saturation!!";
                //                    FILE_LOG(logERROR) << "Y-direction Step Saturation!!";
                //                    FILE_LOG(logERROR) << "Y-direction Step Saturation!!";
                //                } else if(Y_Step < -0.10) {
                //                    Y_Step = -0.10;
                //                    FILE_LOG(logERROR) << "Y-direction Step Saturation!!";
                //                    FILE_LOG(logERROR) << "Y-direction Step Saturation!!";
                //                    FILE_LOG(logERROR) << "Y-direction Step Saturation!!";
                //                }

                //                if(Yaw_Step > 17.0*D2R) {
                //                    Yaw_Step = 17.0*D2R;
                //                    FILE_LOG(logERROR) << "Yaw-Rotation Step Saturation!!";
                //                    FILE_LOG(logERROR) << "Yaw-Rotation Step Saturation!!";
                //                    FILE_LOG(logERROR) << "Yaw-Rotation Step Saturation!!";
                //                } else if(Yaw_Step < -17.0*D2R) {
                //                    Yaw_Step = -17.0*D2R;
                //                    FILE_LOG(logERROR) << "Yaw-Rotation Step Saturation!!";
                //                    FILE_LOG(logERROR) << "Yaw-Rotation Step Saturation!!";
                //                    FILE_LOG(logERROR) << "Yaw-Rotation Step Saturation!!";
                //                }

                StepDataBuffer_Initialize_Walking(LIGHT.Xref_RF2LF, StepTime, DSPTime, StanceWidth, X_Offset, Y_Offset, Yaw_Offset, X_Step, Y_Step, Yaw_Step);
                Xref_CP = LIGHT.Xref_RF2CoM + LIGHT.dXref_RF2CoM/wn_LIPM;
                dXref_CP = zv;

                CurStepBufferNumber = 2;

                QP_xCoMRef4Walking.SelectSolver(SolverIsQPSwift);
                QP_yCoMRef4Walking.SelectSolver(SolverIsQPSwift);

                wn_X_pattern = CoM_wn_X;
                zeta_X_pattern = CoM_zeta_X;
                wn_Y_pattern = CoM_wn_Y;
                zeta_Y_pattern = CoM_zeta_Y;
                N_horizon = _N_horizon;
                dT_horizon = _dT_horizon;

                GenerateWalkingPattern_Formulation(LIGHT.Xref_RF2CoM,LIGHT.dXref_RF2CoM,
                                                   zv,zv,zv);

                TimeLimitSet(2.5); // Wait for 1 seconds before starting
                FILE_LOG(logWARNING) << "[Walk] Parameter Setting...";
            }

            TimeUpdate();
            if(TimeCheck()) {
                TimeReset();
                Walk_CurrentStage = Walk_INITIALIZE;
            }
        }
        break;
    }
    case Walk_INITIALIZE:
    {
        if (TimeIsZero()) {
            SetMotionBase_RF(true);
            _Xini_temp = LIGHT.Xref_RF2LF;
            _Rini_temp = LIGHT.Rref_Pel;
            _Rini_temp2 = LIGHT.Rref_LF;
            Xnext_CoM = LIGHT.Xref_RF2CoM;
            dXnext_CoM = LIGHT.dXref_RF2CoM;
            u_CoM = LIGHT.Xref_RF2CoM;
            u_dCoM = LIGHT.dXref_RF2CoM;
            u_ddCoM = LIGHT.ddXref_RF2CoM;

            TimeLimitSet(SDB[0].t_STEP);
            FILE_LOG(logWARNING) << "[Walk] Walking Motion Is Initiated...";
        }

        Xpre_CoM = Xnext_CoM;
        dXpre_CoM = dXnext_CoM;
        GenerateWalkingPattern_Solve(Xpre_CoM,dXpre_CoM,
                                     u_CoM, u_dCoM, u_ddCoM,
                                     Xnext_CoM,dXnext_CoM,ddXnext_CoM);

        // =============== Global >> Pelvis Orientation and Position Trajectory ===============
        Submotion_Global2Pel_Ori(0.0, 0.0,_Rini_temp);
        // =============== Global >> RF Orientation Trajectory ===============
        Submotion_Global2RF_Ori(0.0, 0.0, I3);
        // =============== Global >> LF Orientation Trajectory ===============
        Submotion_Global2LF_Ori(0.0, 0.0, _Rini_temp2);
        // =============== RF >> CoM Position Trajectory ===============
        Submotion_RF2CoM_Pos(SYS_DT, u_CoM, u_dCoM, u_ddCoM);
        // =============== RF >> LF Position Trajectory ===============
        Submotion_RF2LF_Pos(SYS_DT, _Xini_temp, zv, zv);

        TimeUpdate();
        if(StepDataBuffer_Update()) {
            TimeReset();
            Walk_CurrentStage = Walk_RFSWING;
        }
        break;
    }
    case Walk_RFSWING:
    {
        if(TimeIsZero()) {
            SetMotionBase_LF(false);

            Matrix3d _R = ExtractRotZ(LIGHT.Rref_LF).transpose()*ExtractRotZ(LIGHT.Rref_RF);
            Xref_CP = _R*Xref_CP + LIGHT.Xref_LF2RF;
            Xnext_CoM = _R*Xnext_CoM + LIGHT.Xref_LF2RF;
            dXnext_CoM = _R*dXnext_CoM;
            u_CoM = _R*u_CoM + LIGHT.Xref_LF2RF;
            u_dCoM = _R*u_dCoM;
            u_ddCoM = _R*u_ddCoM;

            _T_SWING = (SDB[0].t_STEP-SDB[0].t_DSP);
            TimeLimitSet(_T_SWING);
            FILE_LOG(logWARNING) << "[Walk] Right Foot Swing! (Swing Time : " << _T_SWING << ")";
        }

        //        StepAdjustment();

        Xpre_CoM = Xnext_CoM;
        dXpre_CoM = dXnext_CoM;
        GenerateWalkingPattern_Solve(Xpre_CoM,dXpre_CoM,
                                     u_CoM, u_dCoM, u_ddCoM,
                                     Xnext_CoM,dXnext_CoM,ddXnext_CoM);

        // =============== Global >> Pelvis Orientation Trajectory ===============
        Submotion_Global2Pel_Ori(TimeNow(), TimeLimit(),RotZ((SDB[0].Yaw_STEP)/2.0));
        // =============== Global >> LF Orientation Trajectory ===============
        Submotion_Global2LF_Ori(0.0, 0.0, I3);
        // =============== LF >> CoM Position Trajectory ===============
        Submotion_LF2CoM_Pos(SYS_DT, u_CoM, u_dCoM, u_ddCoM);

        // =============== Global >> RF Orientation Trajectory ===============
        //        Submotion_Global2RF_Ori(TimeNow(), TimeLimit(),RotZ(SDB[0].Yaw_STEP));
        Submotion_Global2RFswing_forWalking(_T_SWING, SDB[0].Yaw_STEP);
        // =============== LF >> RF Position Trajectory ===============
        Vector3d Xstep_off = Vector3d(0.0, -StanceOffset, 0.0);
        Vector3d Xstep = SDB[0].X_STEP + SDB[0].X_STEP_adj + Xstep_off;
        Submotion_LF2RFSwing_forWalking(_T_SWING, Xstep, Z_SwingUp);

        TimeUpdate();
        if(StepDataBuffer_Update())
        {
            Submotion_LF2RF_Pos(SYS_DT, LIGHT.Xdes_LF2RF, zv, zv);
            TimeReset();
            if((_StepNumber > CurStepBufferNumber)&&(_Flag_Terminate == false)) {
                StepDataBuffer_AddStep(StepTime, DSPTime, StanceWidth, X_Offset, Y_Offset, Yaw_Offset, X_Step, Y_Step, Yaw_Step);
                CurStepBufferNumber++;
            }
            if(CurFlagDSP) {
                Walk_CurrentStage = Walk_RFLANDING;
            } else {
                if(GetFinalStep() == -1) {
                    Walk_CurrentStage = Walk_FINISH;
                } else {
                    Walk_CurrentStage = Walk_LFSWING;
                }
            }
        }
        break;
    }
    case Walk_RFLANDING:
    {
        if (TimeIsZero()) {
            SetMotionBase_LF(true);

            TimeLimitSet(SDB[0].t_DSP);
            FILE_LOG(logWARNING) << "[Walk] Right Foot Landing!";
        }

        Xpre_CoM = Xnext_CoM;
        dXpre_CoM = dXnext_CoM;
        GenerateWalkingPattern_Solve(Xpre_CoM,dXpre_CoM,
                                     u_CoM, u_dCoM, u_ddCoM,
                                     Xnext_CoM,dXnext_CoM,ddXnext_CoM);

        // =============== RF >> CoM Position Trajectory ===============
        Submotion_LF2CoM_Pos(SYS_DT, u_CoM, u_dCoM, u_ddCoM);
        // =============== RF >> LF Position Trajectory ===============
        Vector3d Xstep_off = Vector3d(0.0, -StanceOffset, 0.0);
        Vector3d Xstep = SDB[0].X_STEP + SDB[0].X_STEP_adj + Xstep_off;
        Submotion_LF2RF_Pos(SYS_DT, Xstep, zv, zv);

        TimeUpdate();
        if(StepDataBuffer_Update())
        {
            TimeReset();
            if(GetFinalStep() == -1) {
                Walk_CurrentStage = Walk_FINISH;
            } else {
                Walk_CurrentStage = Walk_LFSWING;
            }
        }
        break;
    }
    case Walk_LFSWING:
    {
        if(TimeIsZero()) {
            SetMotionBase_RF(false);

            Matrix3d _R = ExtractRotZ(LIGHT.Rref_RF).transpose()*ExtractRotZ(LIGHT.Rref_LF);
            Xref_CP = _R*Xref_CP + LIGHT.Xref_RF2LF;
            Xnext_CoM = _R*Xnext_CoM + LIGHT.Xref_RF2LF;
            dXnext_CoM = _R*dXnext_CoM;
            u_CoM = _R*u_CoM + LIGHT.Xref_RF2LF;
            u_dCoM = _R*u_dCoM;
            u_ddCoM = _R*u_ddCoM;

            _T_SWING = (SDB[0].t_STEP-SDB[0].t_DSP);
            TimeLimitSet(_T_SWING);
            FILE_LOG(logWARNING) << "[Walk] Left Foot Swing! (Swing Time : " << _T_SWING << ")";
        }

        //        StepAdjustment();

        Xpre_CoM = Xnext_CoM;
        dXpre_CoM = dXnext_CoM;
        GenerateWalkingPattern_Solve(Xpre_CoM,dXpre_CoM,
                                     u_CoM, u_dCoM, u_ddCoM,
                                     Xnext_CoM,dXnext_CoM,ddXnext_CoM);

        // =============== Global >> Pelvis Orientation Trajectory ===============
        Submotion_Global2Pel_Ori(TimeNow(), TimeLimit(),RotZ((SDB[0].Yaw_STEP)/2.0));
        // =============== Global >> RF Orientation Trajectory ===============
        Submotion_Global2RF_Ori(0.0, 0.0, I3);
        // =============== RF >> CoM Position Trajectory ===============
        Submotion_RF2CoM_Pos(SYS_DT, u_CoM, u_dCoM, u_ddCoM);

        // =============== Global >> LF Orientation Trajectory ===============
        //        Submotion_Global2LF_Ori(TimeNow(), TimeLimit(),RotZ(SDB[0].Yaw_STEP));
        Submotion_Global2LFswing_forWalking(_T_SWING, SDB[0].Yaw_STEP);
        // =============== RF >> LF Position Trajectory ===============
        Vector3d Xstep_off = Vector3d(0.0, StanceOffset, 0.0);
        Vector3d Xstep = SDB[0].X_STEP + SDB[0].X_STEP_adj + Xstep_off;
        Submotion_RF2LFSwing_forWalking(_T_SWING, Xstep, Z_SwingUp);

        TimeUpdate();
        if(StepDataBuffer_Update()) // true : change to DSP
        {
            Submotion_RF2LF_Pos(SYS_DT, LIGHT.Xdes_RF2LF, zv, zv);
            TimeReset();
            if((_StepNumber > CurStepBufferNumber)&&(_Flag_Terminate == false)) {
                StepDataBuffer_AddStep(StepTime, DSPTime, StanceWidth, X_Offset, Y_Offset, Yaw_Offset, X_Step, Y_Step, Yaw_Step);
                CurStepBufferNumber++;
            }
            if(CurFlagDSP) {
                Walk_CurrentStage = Walk_LFLANDING;
            } else {
                if(GetFinalStep() == -1) {
                    Walk_CurrentStage = Walk_FINISH;
                } else {
                    Walk_CurrentStage = Walk_RFSWING;
                }
            }
        }
        break;
    }
    case Walk_LFLANDING:
    {
        if (TimeIsZero()) {
            SetMotionBase_RF(true);

            TimeLimitSet(SDB[0].t_DSP);
            FILE_LOG(logWARNING) << "[Walk] Left Foot Landing!";
        }

        Xpre_CoM = Xnext_CoM;
        dXpre_CoM = dXnext_CoM;
        GenerateWalkingPattern_Solve(Xpre_CoM,dXpre_CoM,
                                     u_CoM, u_dCoM, u_ddCoM,
                                     Xnext_CoM,dXnext_CoM,ddXnext_CoM);

        // =============== RF >> CoM Position Trajectory ===============
        Submotion_RF2CoM_Pos(SYS_DT, u_CoM, u_dCoM, u_ddCoM);
        // =============== RF >> LF Position Trajectory ===============
        Vector3d Xstep_off = Vector3d(0.0, StanceOffset, 0.0);
        Vector3d Xstep = SDB[0].X_STEP + SDB[0].X_STEP_adj + Xstep_off;
        Submotion_RF2LF_Pos(SYS_DT, Xstep, zv, zv);

        TimeUpdate();
        if(StepDataBuffer_Update())
        {
            TimeReset();
            if(GetFinalStep() == -1) {
                Walk_CurrentStage = Walk_FINISH;
            } else {
                Walk_CurrentStage = Walk_RFSWING;
            }
        }
        break;
    }
    case Walk_FINISH:
    {
        if(TimeIsZero()) {
            if(CurStepSupportPhase == SUPPORTCONTROL_RDSP) {
                SetMotionBase_RF(true);
                _Xini_temp = LIGHT.Xref_RF2LF;
                Matrix3d _R = ExtractRotZ(LIGHT.Rref_RF).transpose()*ExtractRotZ(LIGHT.Rref_LF);
                Xref_CP = _R*Xref_CP + LIGHT.Xref_RF2LF;
                Xnext_CoM = _R*Xnext_CoM + LIGHT.Xref_RF2LF;
                dXnext_CoM = _R*dXnext_CoM;
                u_CoM = _R*u_CoM + LIGHT.Xref_RF2LF;
                u_dCoM = _R*u_dCoM;
                u_ddCoM = _R*u_ddCoM;
            } else if(CurStepSupportPhase == SUPPORTCONTROL_LDSP) {
                SetMotionBase_LF(true);
                _Xini_temp = LIGHT.Xref_LF2RF;
                Matrix3d _R = ExtractRotZ(LIGHT.Rref_LF).transpose()*ExtractRotZ(LIGHT.Rref_RF);
                Xref_CP = _R*Xref_CP + LIGHT.Xref_LF2RF;
                Xnext_CoM = _R*Xnext_CoM + LIGHT.Xref_LF2RF;
                dXnext_CoM = _R*dXnext_CoM;
                u_CoM = _R*u_CoM + LIGHT.Xref_LF2RF;
                u_dCoM = _R*u_dCoM;
                u_ddCoM = _R*u_ddCoM;
            }

            TimeLimitSet(3.0);
            FILE_LOG(logSUCCESS) << "[Walk] Walking with DSP is finished!";
        }

        if(CurStepSupportPhase == SUPPORTCONTROL_RDSP) {
            Xpre_CoM = Xnext_CoM;
            dXpre_CoM = dXnext_CoM;
            GenerateWalkingPattern_Solve(Xpre_CoM,dXpre_CoM,
                                         u_CoM, u_dCoM, u_ddCoM,
                                         Xnext_CoM,dXnext_CoM,ddXnext_CoM);

            // =============== RF >> CoM Position Trajectory ===============
            Submotion_RF2CoM_Pos(SYS_DT, u_CoM, u_dCoM, u_ddCoM);
            // =============== RF >> LF Position Trajectory ===============
            Submotion_RF2LF_Pos(SYS_DT, _Xini_temp, zv, zv);
        } else if(CurStepSupportPhase == SUPPORTCONTROL_LDSP) {
            Xpre_CoM = Xnext_CoM;
            dXpre_CoM = dXnext_CoM;
            GenerateWalkingPattern_Solve(Xpre_CoM,dXpre_CoM,
                                         u_CoM, u_dCoM, u_ddCoM,
                                         Xnext_CoM,dXnext_CoM,ddXnext_CoM);

            // =============== RF >> CoM Position Trajectory ===============
            Submotion_LF2CoM_Pos(SYS_DT, u_CoM, u_dCoM, u_ddCoM);
            // =============== RF >> LF Position Trajectory ===============
            Submotion_LF2RF_Pos(SYS_DT, _Xini_temp, zv, zv);
        }

        TimeUpdate();
        if(_OPERATION_MODE_CHANGED||TimeCheck()) {
            TimeReset();
            IsWalking = false;

            CurStepNumber = CurStepBufferNumber;
            Walk_CurrentStage = Walk_PARAMETER_SETTING;
            Submotion_RF2CoM_Pos(SYS_DT, LIGHT.Xref_RF2CoM, zv, zv);
            Submotion_RF2LF_Pos(SYS_DT, LIGHT.Xref_RF2LF, zv, zv);
            Submotion_LF2CoM_Pos(SYS_DT, LIGHT.Xref_LF2CoM, zv, zv);
            Submotion_LF2RF_Pos(SYS_DT, LIGHT.Xref_LF2RF, zv, zv);
            JoyStickCommand_StartWalk = false;
            JoyStickCommand_StopWalk = false;
            FILE_LOG(logSUCCESS) << " Walking Motion is done!! ";
            return true;
        }
        break;
    }
    default:
        break;
    }

    CurStepNumber = CurStepBufferNumber;

    if(_OPERATION_MODE_CHANGED) {
        TimeReset();
        IsWalking = false;

        Walk_CurrentStage = Walk_PARAMETER_SETTING;
        Submotion_RF2CoM_Pos(SYS_DT, LIGHT.Xref_RF2CoM, zv, zv);
        Submotion_RF2LF_Pos(SYS_DT, LIGHT.Xref_RF2LF, zv, zv);
        Submotion_LF2CoM_Pos(SYS_DT, LIGHT.Xref_LF2CoM, zv, zv);
        Submotion_LF2RF_Pos(SYS_DT, LIGHT.Xref_LF2RF, zv, zv);
        JoyStickCommand_StartWalk = false;
        JoyStickCommand_StopWalk = false;
        LIGHT.Flag_Error = true;
        FILE_LOG(logERROR) << "[Error] Operation is Changed during Walking!! ";
        return true;
    }
    return false;
}

// =====================================================================================================


enum FullTask_STAGESET{
    FullTask_PARAMETER_SETTING = 0,
    FullTask_DSP_SQUAT,
    FullTask_SSP_READY,
    FullTask_SSP_SQUAT,
    FullTask_SSP_LANDING,
    FullTask_WALKING_SLOW,
    FullTask_WALKING_FAST,
    FullTask_FINISH
};
int FullTask_CurrentStage = FullTask_PARAMETER_SETTING;
int save_Type_FullTaskScenario = 1;
bool LIGHTWholeMotions::FullTaskScenario() {

    static double STEP_TIME = 1.50;
    static double DSP_TIME = 0.40;
    static double StanceWidth = 0.190;
    static double X_Offset = 0.00;
    static double Y_Offset = 0.01;
    static double Yaw_Offset = 0.0*D2R;
    static double X_STEP = 0.35;
    static double Y_STEP = 0.00;
    static double Yaw_STEP = 0.0*D2R;
    static double StanceOffset = 0.01;
    static bool WalkingTerminate = false;

    // Setting for future pressure and flowrate reference
    static int idx_PumpRefGen = 0;
    if(IsLoaded_FuturePumpReference_FullTaskScenario) {
        for(int i=0;i<=sharedREF->N_PrevPump;i++) {
            int idx_Prev = idx_PumpRefGen + i*(double)(sharedREF->dT_PrevPump/SYS_DT);
            if(idx_Prev<Ps_Future_FullTaskScenario.size()){
                sharedREF->RequiredPressureReference_Future[i] = Ps_Future_FullTaskScenario(idx_Prev);
                sharedREF->RequiredFlowrateReference_Future[i] = Qp_Future_FullTaskScenario(idx_Prev);
            } else {
                sharedREF->RequiredPressureReference_Future[i] = Ps_Future_FullTaskScenario(Ps_Future_FullTaskScenario.size()-1);
                sharedREF->RequiredFlowrateReference_Future[i] = Qp_Future_FullTaskScenario(Qp_Future_FullTaskScenario.size()-1);
            }
        }
//        cout << "Ps : " << sharedREF->RequiredPressureReference_Future[0] << endl;
//        cout << "Qp : " << sharedREF->RequiredFlowrateReference_Future[0] << endl;
    } idx_PumpRefGen++;


    switch(FullTask_CurrentStage) {
    case FullTask_PARAMETER_SETTING:
    {
        FullTask_CurrentStage = FullTask_DSP_SQUAT;
        if(save_Type_FullTaskScenario == 1) save_PutDataFlag = true;
        else if(save_Type_FullTaskScenario == 2) save_PutDataFlag_SupPres = true;
        break;
    }
    case FullTask_DSP_SQUAT:
    {
        double Squat_Height = 0.20;
        double Squat_Period = 2.8;
        double Squat_Number = 4.0;
        if(LIGHT_WholeMotions.Squat_DSP(Squat_Height, Squat_Period, Squat_Number)){
            FullTask_CurrentStage = FullTask_SSP_READY;
        }
        break;
    }
    case FullTask_SSP_READY:
    {
        double t_SwingUp = 2.0;
        Vector3d ZMP_offset;
        ZMP_offset << 0.0, -0.02, 0.0;
        if(LIGHT_WholeMotions.RFSwingUp_Dynamic(t_SwingUp, ZMP_offset, 0.0)){
            FullTask_CurrentStage = FullTask_SSP_SQUAT;
        }
        break;
    }
    case FullTask_SSP_SQUAT:
    {
        double Squat_Height = 0.16;
        double Squat_Period = 3.5;
        double Squat_Number = 4.0;
        if(LIGHT_WholeMotions.Squat_SSP(Squat_Height, Squat_Period, Squat_Number)){
            FullTask_CurrentStage = FullTask_SSP_LANDING;
        }
        break;
    }
    case FullTask_SSP_LANDING:
    {
        double t_SwingDown = 2.0;
        Vector3d LangingPosition;
        LangingPosition << 0.0, -0.200, 0.0;
        if(LIGHT_WholeMotions.RFSwingDown_Dynamic(t_SwingDown, LangingPosition, 0.0)){
//            FullTask_CurrentStage = FullTask_WALKING_SLOW;
//            STEP_TIME = 1.40;
//            DSP_TIME = 0.35;
//            StanceWidth = 0.160;
//            X_Offset = 0.00;
//            Y_Offset = 0.00;
//            Yaw_Offset = 0.0*D2R;
//            X_STEP = 0.35;
//            Y_STEP = 0.00;
//            Yaw_STEP = 0.0*D2R;
//            StanceOffset = 0.00;
//            WalkingTerminate = false;

            FullTask_CurrentStage = FullTask_WALKING_FAST;
            STEP_TIME = 0.65;
            DSP_TIME = 0.03;
            StanceWidth = 0.190;
            X_Offset = 0.00;
            Y_Offset = 0.015;
            Yaw_Offset = 0.0*D2R;
            X_STEP = 0.00; // 0.20
            Y_STEP = 0.00;
            Yaw_STEP = 0.0*D2R;
            StanceOffset = 0.015;
            WalkingTerminate = false;
        }
        break;
    }
//    case FullTask_WALKING_SLOW:
//    {
//        int CurStepNumber;
//        bool FINISH = LIGHT_WholeMotions.Walk(STEP_TIME, DSP_TIME, StanceWidth, 99999,
//                                              X_Offset, Y_Offset, Yaw_Offset,
//                                              X_STEP, Y_STEP, Yaw_STEP, StanceOffset,
//                                              WalkingTerminate, CurStepNumber);

//        if(CurStepNumber>8 && CurStepNumber<16) {
//            X_STEP = FullTask_WALKING_SLOW;
//            X_STEP = -0.35;
//        } else if(CurStepNumber>=16) {
//            WalkingTerminate = true;
//        }

//        if(FINISH){
//            FullTask_CurrentStage = FullTask_WALKING_FAST;
//            STEP_TIME = 0.60;
//            DSP_TIME = 0.03;
//            StanceWidth = 0.190;
//            X_Offset = 0.00;
//            Y_Offset = 0.00;
//            Yaw_Offset = 0.0*D2R;
//            X_STEP = 0.30;
//            Y_STEP = 0.00;
//            Yaw_STEP = 0.0*D2R;
//            StanceOffset = 0.00;
//            WalkingTerminate = false;
//        }
//        break;
//    }
    case FullTask_WALKING_FAST:
    {
        int CurStepNumber;
        bool FINISH = LIGHT_WholeMotions.Walk(STEP_TIME, DSP_TIME, StanceWidth, 99999,
                                              X_Offset, Y_Offset, Yaw_Offset,
                                              X_STEP, Y_STEP, Yaw_STEP, StanceOffset,
                                              WalkingTerminate, CurStepNumber);

        if(CurStepNumber==5) {
            X_STEP = 0.00;
        } else if(CurStepNumber>5 && CurStepNumber<10) {
            X_STEP = -0.00; // -0.20
        } else if(CurStepNumber>=10) {
            WalkingTerminate = true;
        }

        if(FINISH){
            FullTask_CurrentStage = FullTask_FINISH;
            FILE_LOG(logWARNING) << " Full Task Scenario is terminated.. ";
        }
        break;
    }
    case FullTask_FINISH:
    {
        bool FINISH = LIGHT_WholeMotions.CoM_Move_RDSP(2.0,
                                                   I3,I3,I3,
                                                   LIGHT.Xref_RF2CoM,zv,zv);

        if(FINISH){
            FullTask_CurrentStage = FullTask_PARAMETER_SETTING;
            FILE_LOG(logSUCCESS) << " Full Task Scenario is done!! ";
            if(save_Type_FullTaskScenario == 1) save_ActivateFlag = true;
            else if(save_Type_FullTaskScenario == 2) save_ActivateFlag_SupPres = true;
            idx_PumpRefGen = 0;
            return true;
        }
        break;
    }
    default:
        break;
    }

    if(_OPERATION_MODE_CHANGED) {
        TimeReset();
        FullTask_CurrentStage = FullTask_PARAMETER_SETTING;
        Submotion_RF2CoM_Pos(SYS_DT, LIGHT.Xref_RF2CoM, zv, zv);
        Submotion_RF2LF_Pos(SYS_DT, LIGHT.Xref_RF2LF, zv, zv);
        Submotion_LF2CoM_Pos(SYS_DT, LIGHT.Xref_LF2CoM, zv, zv);
        Submotion_LF2RF_Pos(SYS_DT, LIGHT.Xref_LF2RF, zv, zv);
        LIGHT.Flag_Error = true;
        idx_PumpRefGen = 0;
        FILE_LOG(logERROR) << "[Error] Operation is Changed during Tasks!! ";
        return true;
    }
    return false;
}



