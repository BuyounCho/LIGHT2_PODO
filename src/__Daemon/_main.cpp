#include <iostream>
#include <sys/mman.h>
#include <unistd.h>
#include <signal.h>
#include <fcntl.h>

#include <QSettings>

#include "RBCAN.h"
#include "RBRawLAN.h"
#include "RBDataBase.h"
#include "../../share/Headers/LANData/GazeboLANData.h"

#include "RBProcessManager.h"

#include "HydraulicActuatorDataConverting.h"
#include "HydraulicActuatorController.h"
#include "RBFTSensor.h"
#include "RBIMUSensor.h"
#include "RBSmartPower.h"
#include "RBOpticFlowSensor.h"
#include "RBFOGSensor.h"
#include "RBELMO.h"

//#include "Eigen/Dense"
//#include "rbdl/rbdl.h"

using namespace std;

QString     settingFile;

// Basic --------
int     IS_WORKING = false;
int     IS_CHILD = false;
int     IS_CAN_OK = false;
int     IS_RS232_OK = false;

int     NO_RESPONSE_CNT = 0;

pRBCORE_SHM_COMMAND     sharedCMD;
pRBCORE_SHM_REFERENCE   sharedREF;
pRBCORE_SHM_SENSOR      sharedSEN;
pRBCAN                  canHandler;
pRBLAN                  lanHandler;
RBProcessManager        *pmHandler;
OpticalDisplacement     ODHandler;

// Daemon Options
int     __IS_GAZEBO = false;
int     __IS_FOG = false;
int     __IS_ROS = false;

// Initialize --------
int     RBCore_Initialize();
int     RBCore_SMInitialize();
int     RBCore_DBInitialize();
int     RBCore_CANInitialize();
int     RBCore_LANInitialize();
int     RBCore_ThreadInitialize();
int     RBCore_PMInitialize();
int     RBCore_Termination();
void    RBCore_RTThreadCon(void *);
void    *RBCore_NRTThreadCon(void *);
bool    DO_CANCHECK = false;
RT_TASK rtTaskCon;
ulong   nrtTaskCon;

// Database --------
DB_GENERAL      RBDataBase::_DB_GENERAL;
DB_MC           RBDataBase::_DB_MC[MAX_MC];
DB_PC           RBDataBase::_DB_PC[MAX_PC];
DB_FT           RBDataBase::_DB_FT[MAX_FT];
DB_IMU          RBDataBase::_DB_IMU[MAX_IMU];
DB_SP           RBDataBase::_DB_SP[MAX_SP];
DB_OF           RBDataBase::_DB_OF[MAX_OF];
DB_AL           RBDataBase::_DB_AL[MAX_AL];

int     _VERSION;
int     _NO_OF_AL;
int     _NO_OF_COMM_CH;
int     _NO_OF_MC;
int     _NO_OF_PC;
int     _NO_OF_FT;
int     _NO_OF_IMU;
int     _NO_OF_SP;
int     _NO_OF_OF;


// Devices --------
HydraulicActuatorController _DEV_MC[MAX_MC];
HydraulicPumpController     _DEV_PC[MAX_PC];
RBFTSensor                  _DEV_FT[MAX_FT];
RBIMUSensor                 _DEV_IMU[MAX_IMU];
RBSmartPower                _DEV_SP[MAX_SP];
RBOpticFlowSensor           _DEV_OF[MAX_OF];
RBFOGSensor                 _DEV_FOG;

int _CANOUT_ENABLED = false;
int _SENSOR_ENABLED = false;

// Save Function ------
#define SAVEKIND    30
#define SAVENUM     100000

bool            save_Flag = false;
unsigned int    save_Index = 0;
float           save_Buf[SAVEKIND][SAVENUM];
void            save_PutData(unsigned int cur_Index);
void            save_File();

void            SetWaitTime(int T);
void            WaitDisplay(void);
int             WaitTime = 0; // msec

int _ALCommandCnt[MAX_AL] = {0,};

long _ThreadCnt = 0;

bool StatusReadFlag[MAX_MC] = {0,};
bool ErrorClearStart = false;

// Thread Functions
void    THREAD_UpdateHCBInfo();
void    THREAD_ReadEncoder();
void    THREAD_ReadValvePos();
void    THREAD_ReadOtherInfo();

void    THREAD_ReadNewIMU();
void    THREAD_ReadFT();

void    THREAD_ReadPumpData();


// Command Functions
void    RBCMD_InitCheckDevice();
void    RBCMD_CAN_Channel_Arrange();

void    RBCMD_InitFETOnOff();
void    RBCMD_EncoderZero();
void    RBCMD_MotionRefOnOff();

void    RBCMD_SensorFTOnOff();
void    RBCMD_SensorFTNull();

void    RBCMD_SensorIMUONOFF();
void    RBCMD_SensorIMUNull();
void    RBCMD_SensorIMUOffsetSetting();

void    RBCMD_MotionSetPWM();
void    RBCMD_MotionErrorClear();
void    RBCMD_CANEnableDisable();

void    RBCMD_HCB_BoardTest(void);
void    RBCMD_HCB_BoardTest_ErrorClear(void);

void    RBCMD_HCB_RequestEnableDisable(void);
void    RBCMD_HCB_AskEverything(void);
void    RBCMD_HCB_SetEverything(void);
void    RBCMD_HCB_SetBNO(void);

void    RBCMD_HCB_TorqueForce_Nulling(void);
void    RBCMD_HCB_Pres_Nulling(void);

void    RBCMD_HCB_AllReference_Reset(void);
void    RBCMD_EncZero(void);
void    RBCMD_FindHome(void);
void    FindHomeOperation(int _BN);

extern void CylinderPos2Angle_Ankle(double theta, double phi, double x_R, double x_L,
                                    double& new_theta, double& new_phi);
extern void CylinderVel2AngularVel_Ankle(double theta, double phi, double dx_R, double dx_L,
                                         double& dtheta, double& dphi);
extern void CylinderForce2Torque_Ankle(double theta, double phi, double F_R, double F_L,
                                       double& T_pitch, double& T_roll);

extern void CylinderPos2Angle_Ankle_NewSW(double theta, double phi, double x_R, double x_L,
                                          double& new_theta, double& new_phi);
extern void CylinderVel2AngularVel_Ankle_NewSW(double theta, double phi, double dx_R, double dx_L,
                                               double& dtheta, double& dphi);
extern void CylinderForce2Torque_Ankle_NewSW(double theta, double phi, double F_R, double F_L,
                                             double& T_pitch, double& T_roll);

extern void Rotary2Joint_QuadKnee(double S, double dS, double F,
                            double &theta, double &dtheta, double &T);

#define     g_const      9.81
#define     COLS_InEKF   3
#define     ROWS_InEKF   3

class Vector3_InEKF {
public:
    double v[COLS_InEKF];
public:
    Vector3_InEKF(double a = 0.0, double b = 0.0, double c = 0.0) {
        v[0] = a;        v[1] = b;        v[2] = c;
    }
    ~Vector3_InEKF() {

    }
    void show() {
        std::cout << v[0] << ", " << v[1] << ", " << v[2] << std::endl << std::endl;
    }

    void operator=(const Vector3_InEKF &in)
    {
        for(int i=0;i<COLS_InEKF;i++) {
            v[i] = in.v[i];
        }
    }

    Vector3_InEKF operator+(const Vector3_InEKF &in) {
        Vector3_InEKF out;
        for(int i=0;i<COLS_InEKF;i++) {
            out.v[i] = v[i] + in.v[i];
        }
        return out;
    }
    Vector3_InEKF operator-(const Vector3_InEKF &in) {
        Vector3_InEKF out;
        for(int i=0;i<COLS_InEKF;i++) {
            out.v[i] = v[i] - in.v[i];
        }
        return out;
    }

    Vector3_InEKF operator*(const double &in) {
        Vector3_InEKF out;
        for (int i=0; i<COLS_InEKF; i++) {
            out.v[i] = in*v[i];
        }
        return out;
    }

    Vector3_InEKF Zero() {
        Vector3_InEKF out;
        for(int i=0;i<COLS_InEKF;i++) {
            out.v[i] = 0.0;
        }
        return out;
    }

    double norm() {
        return sqrt(this->v[0]*this->v[0]
                +this->v[1]*this->v[1]
                +this->v[2]*this->v[2]);
    }
};
Vector3_InEKF operator*(const double &a,const Vector3_InEKF &in) {
    Vector3_InEKF out;
    for (int i=0; i<COLS_InEKF; i++) {
        out.v[i] = a*in.v[i];
    }
    return out;
}
Vector3_InEKF Ones_Vec3() {
    Vector3_InEKF out;
    for (int i=0; i<COLS_InEKF; i++) {
        out.v[i] = 1.0;
    }
    return out;
}
Vector3_InEKF Zero_Vec3() {
    Vector3_InEKF out;
    for (int i=0; i<COLS_InEKF; i++) {
        out.v[i] = 0.0;
    }
    return out;
}

class Matrix3_InEKF {
public:
    double M[3][3];
public:
    Matrix3_InEKF(double a = 0.0, double b = 0.0, double c = 0.0,
                  double d = 0.0, double e = 0.0, double f = 0.0,
                  double g = 0.0, double h = 0.0, double i = 0.0)
    {
        M[0][0] = a; M[0][1] = b; M[0][2] = c;
        M[1][0] = d; M[1][1] = e; M[1][2] = f;
        M[2][0] = g; M[2][1] = h; M[2][2] = i;
    }

    ~Matrix3_InEKF() {

    }

    void show() {
        std::cout << M[0][0] << ", " << M[0][1] << ", " << M[0][2] << std::endl;
        std::cout << M[1][0] << ", " << M[1][1] << ", " << M[1][2] << std::endl;
        std::cout << M[2][0] << ", " << M[2][1] << ", " << M[2][2] << std::endl << std::endl;
    }

    void operator=(const Matrix3_InEKF &in)
    {
        for (int i=0; i<COLS_InEKF; i++) {
            for (int j=0; j<ROWS_InEKF; j++) {
                M[i][j] = in.M[i][j];
            }
        }
    }

    Matrix3_InEKF Transpose(){
        Matrix3_InEKF copy;
        for (int i=0; i<COLS_InEKF; i++) {
            for (int j=0; j<ROWS_InEKF; j++) {
                copy.M[i][j] = M[i][j];
            }
        }
        for (int i=0; i<COLS_InEKF; i++) {
            for (int j=0; j<ROWS_InEKF; j++) {
                M[j][i] = copy.M[i][j];
            }
        }
        return *this;
    }

    Matrix3_InEKF Inverse() {
        Matrix3_InEKF out;
        double det = M[0][0] * (M[1][1] * M[2][2] - M[2][1] * M[1][2]) -
                M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) +
                M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);

        if(abs(det)<1e-10) {
            if(det>=0.0) det = 1e-10;
            else det = -1e-10;
        }

        out.M[0][0] =  (M[1][1] * M[2][2] - M[2][1] * M[1][2]) / det;
        out.M[0][1] =  (M[0][2] * M[2][1] - M[0][1] * M[2][2]) / det;
        out.M[0][2] =  (M[0][1] * M[1][2] - M[0][2] * M[1][1]) / det;
        out.M[1][0] =  (M[1][2] * M[2][0] - M[1][0] * M[2][2]) / det;
        out.M[1][1] =  (M[0][0] * M[2][2] - M[0][2] * M[2][0]) / det;
        out.M[1][2] =  (M[1][0] * M[0][2] - M[0][0] * M[1][2]) / det;
        out.M[2][0] =  (M[1][0] * M[2][1] - M[2][0] * M[1][1]) / det;
        out.M[2][1] =  (M[2][0] * M[0][1] - M[0][0] * M[2][1]) / det;
        out.M[2][2] =  (M[0][0] * M[1][1] - M[1][0] * M[0][1]) / det;
        return out;
    }

    Matrix3_InEKF operator+(const Matrix3_InEKF &in) {
        Matrix3_InEKF out;
        for (int i=0; i<COLS_InEKF; i++) {
            for (int j=0; j<ROWS_InEKF; j++) {
                out.M[i][j] = M[i][j] + in.M[i][j];
            }
        }
        return out;
    }

    Matrix3_InEKF operator-(const Matrix3_InEKF &in) {
        Matrix3_InEKF out;
        for (int i=0; i<COLS_InEKF; i++) {
            for (int j=0; j<ROWS_InEKF; j++) {
                out.M[i][j] = M[i][j] - in.M[i][j];
            }
        }
        return out;
    }

    Matrix3_InEKF operator*(const Matrix3_InEKF &in) {
        Matrix3_InEKF out;
        for (int i=0; i<COLS_InEKF; i++) {
            for (int j=0; j<ROWS_InEKF; j++) {
                for (int k=0; k<ROWS_InEKF; k++) {
                    out.M[i][j] += M[i][k] * in.M[k][j];
                }
            }
        }
        return out;
    }

    Vector3_InEKF operator*(const Vector3_InEKF &in) {
        Vector3_InEKF out;
        for (int i=0; i<COLS_InEKF; i++) {
            for (int j=0; j<ROWS_InEKF; j++) {
                out.v[i] += M[i][j] * in.v[j];
            }
        }
        return out;
    }

    Matrix3_InEKF operator*(const double &in) {
        Matrix3_InEKF out;
        for (int i=0; i<COLS_InEKF; i++) {
            for (int j=0; j<ROWS_InEKF; j++) {
                out.M[i][j] = in*M[i][j];
            }
        }
        return out;
    }

};
Matrix3_InEKF Zero_Mat3() {
    Matrix3_InEKF out;
    for (int i=0; i<COLS_InEKF; i++) {
        for (int j=0; j<ROWS_InEKF; j++) {
            out.M[i][j] = 0.0;
        }
    }
    return out;
}
Matrix3_InEKF Identity_Mat3() {
    Matrix3_InEKF out;
    for (int i=0; i<COLS_InEKF; i++) {
        for (int j=0; j<ROWS_InEKF; j++) {
            if(i==j) out.M[i][j] = 1.0;
            else out.M[i][j] = 0.0;
        }
    }
    return out;
}
Matrix3_InEKF operator*(const double &a, const Matrix3_InEKF &in) {
    Matrix3_InEKF out;
    for (int i=0; i<COLS_InEKF; i++) {
        for (int j=0; j<ROWS_InEKF; j++) {
            out.M[i][j] = a*in.M[i][j];
        }
    }
    return out;
}
Matrix3_InEKF hat_oper(const Vector3_InEKF in) {
    double a = in.v[0];
    double b = in.v[1];
    double c = in.v[2];
    Matrix3_InEKF out(0.0,   -c,    b,
                      c,  0.0,   -a,
                      -b,    a,  0.0);
    return out;
}
Matrix3_InEKF expM(const Matrix3_InEKF M) {
    // exp(M) := I + M + M^2/2! + M^3/3! + M^4/4! + .... + M^N/N!
    Matrix3_InEKF out = Identity_Mat3();
    Matrix3_InEKF tempM = M;

    int N = 20; // approximation
    double k = 2.0;
    for(int i=1;i<=N;i++) {
        out = out + tempM;
        tempM = tempM*M*(1.0/k);
        k = k + 1.0;
    }
    return out;
}

Matrix3_InEKF exp_so3(Vector3_InEKF v) {
    // M = hat(v)
    // exp(M) = I + M + M^2/2! + M^3/3! + M^4/4! + .... + M^N/N!
    //        = I + sin(theta)*hat(v) + (1-cos(theta))*hat(v)*hat(v)
    double theta = v.norm();
    if (fabs(theta) > 1e-6) {
        Matrix3_InEKF K = hat_oper(v*(1.0/v.norm()));
        return (Identity_Mat3() + sin(theta)*K + (1-cos(theta))*K*K);
    } else {
        return Identity_Mat3();
    }
}

class InEKF_Parameter {
private:
    bool Flag_initialized;
    double t;

    Vector3_InEKF stdev_gyro; //standard deviation for the gyroscope
    Vector3_InEKF stdev_acc; //standard deviation for the accelerometer
    Vector3_InEKF rotation_stdev_prior; //standard deviation for the initial position estimate
    Vector3_InEKF g_vec; // Gravity Vector
    //    Vector3_InEKF g_vec(0.0,-9.81,0.0); // Gravity Vector
    //    Vector3_InEKF g_vec(0.0,9.81,0.0); // Gravity Vector

    Vector3_InEKF w_init; // initial angular velocity
    Vector3_InEKF w_prev; // previous angular velocity

    Matrix3_InEKF Qg; // Covariance Matrix for gyroscope
    Matrix3_InEKF Qa; // Covariance Matrix for accelerometer
    Matrix3_InEKF P_prior; // covariance matrix for the initial position

    Matrix3_InEKF R_Offset;

public:
    Matrix3_InEKF R; // Rotation Matrix
    Matrix3_InEKF P; // Covariance Matrix

public:
    InEKF_Parameter() {
        Flag_initialized = false;
        t = 0.0;

        stdev_gyro.v[0] = 0.001;
        stdev_gyro.v[1] = 0.001;
        stdev_gyro.v[2] = 0.001;

        stdev_acc.v[0] = 0.005;
        stdev_acc.v[1] = 0.005;
        stdev_acc.v[2] = 0.005;

        rotation_stdev_prior.v[0] = 0.01;
        rotation_stdev_prior.v[1] = 0.01;
        rotation_stdev_prior.v[2] = 0.01;

        g_vec.v[0] = 0.0;
        g_vec.v[1] = 0.0;
        g_vec.v[2] = g_const;

        R = Identity_Mat3();
        P = Identity_Mat3();

        Qg = Identity_Mat3();
        Qg.M[0][0] = stdev_gyro.v[0]*stdev_gyro.v[0];
        Qg.M[1][1] = stdev_gyro.v[1]*stdev_gyro.v[1];
        Qg.M[2][2] = stdev_gyro.v[2]*stdev_gyro.v[2];

        Qa = Identity_Mat3();
        Qa.M[0][0] = stdev_acc.v[0]*stdev_acc.v[0];
        Qa.M[1][1] = stdev_acc.v[1]*stdev_acc.v[1];
        Qa.M[2][2] = stdev_acc.v[2]*stdev_acc.v[2];

        P_prior = Identity_Mat3();
        P_prior.M[0][0] = rotation_stdev_prior.v[0]*rotation_stdev_prior.v[0];
        P_prior.M[1][1] = rotation_stdev_prior.v[1]*rotation_stdev_prior.v[1];
        P_prior.M[2][2] = rotation_stdev_prior.v[2]*rotation_stdev_prior.v[2];
    }

    void Zero() {
        t = 0.0;
    }

    bool Initialize(Vector3_InEKF _a) {
        double norm = _a.norm();
        if(norm > 1e-6) {
            Vector3_InEKF a_n = _a*(1.0/norm);

            // axis-angle
            // unit vector : u = [ux;uy;uz] = [cos(phi) sin(phi) 0]; (-PI/2.0<phi<PI/2.0)
            // angle : theta;
            // R = [ cos(theta) + ux*ux*(1-cos(theta));    ux*uy*(1-cos(theta))-uz*sin(theta);   ux*uz*(1-cos(theta))+uy*sin(theta);
            //       ux*uy*(1-cos(theta))+uz*sin(theta);   cos(theta) + uy*uy*(1-cos(theta));    uy*uz*(1-cos(theta))-ux*sin(theta);
            //       ux*uz*(1-cos(theta))-uy*sin(theta);   uy*uz*(1-cos(theta))+ux*sin(theta);   cos(theta) + uz*uz*(1-cos(theta))];
            //     [ cos(theta) + ux*ux*(1-cos(theta));    ux*uy*(1-cos(theta));                 uy*sin(theta);
            //       ux*uy*(1-cos(theta));                 cos(theta) + uy*uy*(1-cos(theta));    -ux*sin(theta);
            //       -uy*sin(theta);                       ux*sin(theta);                        cos(theta)];

            double theta = acos(a_n.v[2]);

            if(fabs(theta) < 1e-2) {
                Matrix3_InEKF _R_init = Identity_Mat3();
                R = _R_init;
                P = P_prior;
            } else {
                double ux = a_n.v[1]/sin(theta);
                double uy = -a_n.v[0]/sin(theta);

                Matrix3_InEKF _R_init(cos(theta) + ux*ux*(1-cos(theta)), ux*uy*(1-cos(theta))              , uy*sin(theta),
                                      ux*uy*(1-cos(theta))             , cos(theta) + uy*uy*(1-cos(theta)) ,-ux*sin(theta),
                                      -uy*sin(theta)                   , ux*sin(theta)                     , cos(theta));

                //                _R_init.show();
                R = _R_init;
                P = P_prior;
            }
            FILE_LOG(logSUCCESS) << "IMU Initializing is Done!!";
            return Flag_initialized = true;
        } else {
            return Flag_initialized = false;
        }
    }

    void Update(Vector3_InEKF _Uw, Vector3_InEKF _Ua, bool Correction_OnOff = true) {
        // calculate the state from the input information

        // Initializing
        if(t<(double)RT_TIMER_PERIOD_MS/1000.0/2.0) {
            Initialize(_Ua);
        }

        // prediction step - update all the state using the gyroscope data
        double dT = (double)RT_TIMER_PERIOD_MS/1000.0;
        double K_YawZero = 2.0*3.141592/(30.0); // 30sec convergence for prevent drift
        Vector3_InEKF psi_vec(0.0,0.0,atan2(R.M[1][0], R.M[0][0]));
        Vector3_InEKF w = _Uw - K_YawZero*(psi_vec);
        Matrix3_InEKF R_pred = R*exp_so3(w*dT);

        // Define the Adjoint matrix
        Matrix3_InEKF Adj = R_pred;

        // define Linearized dynamics matrix
        Matrix3_InEKF A = Zero_Mat3();

        // propagate the covariance matrix with the riccati equation
        Matrix3_InEKF F = Identity_Mat3()+A*dT;
        Matrix3_InEKF Cov_w = Qg;
        Matrix3_InEKF Q = F*Adj*Cov_w*Adj.Transpose()*F*dT; // approximation
        Matrix3_InEKF P_pred = F*P*F.Transpose() + Q;

        // correction step
        if(Correction_OnOff) {
            // kalman filter gain
            Matrix3_InEKF H = hat_oper(g_vec);
            Matrix3_InEKF S = H*P_pred*H.Transpose() + Qa;
            Matrix3_InEKF K = P_pred*H.Transpose()*S.Inverse();

            Vector3_InEKF g_error = R_pred*_Ua*(g_vec.norm()/_Ua.norm())-g_vec;
            Matrix3_InEKF R_cor = exp_so3(K*g_error);

            R = R_cor*R_pred;
            P = (Identity_Mat3() - K*H)*P_pred;
            //            R.show();

            //            double norm = _Ua.norm();
            //            double theta = 0.0;
            //            Vector3_InEKF u;
            //            if(norm > 1e-6) {
            //                Vector3_InEKF Ua_n = _Ua*(1.0/norm);
            //                theta = acos(Ua_n.v[2]);
            //                if(fabs(theta) < 1e-2) {
            //                    u.v[0] = 1.0;
            //                    u.v[1] = 0.0;
            //                    u.v[2] = 0.0;
            //                } else {
            //                    u.v[0] = Ua_n.v[1]/sin(theta);
            //                    u.v[1] = -Ua_n.v[0]/sin(theta);
            //                    u.v[2] = 0.0;
            //                }
            //            } else {
            //                theta = 0.0;
            //                u.v[0] = 1.0;
            //                u.v[1] = 0.0;
            //                u.v[2] = 0.0;
            //            }
            //            Matrix3_InEKF R_cor = exp_so3(K*u*theta);
            //            R = R_cor*R_pred;
            //            P = (Identity_Mat3() - K*H)*P_pred;
            //            R_cor.show();

        } else {
            R = R_pred;
            P = P_pred;
        }

        t += dT;
        if(t>100000.0) {
            t = 0.0;
        }
    }
};

InEKF_Parameter  InEKF_IMU;

#define  PI     3.141592
#define  D2R    3.141592/180.0
#define  R2D    180.0/3.141592

void CatchSignals(int _signal)
{

    switch(_signal)
    {
    case SIGHUP:     // shell termination
    case SIGINT:     // Ctrl-c
    case SIGTERM:    // "kill" from shell
    case SIGKILL:
    case SIGSEGV:
        if(__IS_GAZEBO)
            lanHandler->RBLanClose();
        else
            canHandler->Finish();
        usleep(1000*1000);

        for(int i=1; i<_NO_OF_AL; i++){
            pmHandler->CloseAL(i);
            //            cout<<"_NO_OF_AL :" <<_NO_OF_AL << endl;
        }
        IS_WORKING = false;
        //        std::cout << "daemon CatchSignals " << std::endl;


        break;
    }
    usleep(1000*1000);
}

void CheckArguments(int argc, char *argv[]){
    int opt = 0;
    int extrafinger = 0;
    while((opt = getopt(argc, argv, "g:f:r:e:")) != -1){
        switch(opt){
        case 'g':
            if(strcmp(optarg, "true")==0 || strcmp(optarg, "TRUE")==0){
                __IS_GAZEBO = true;
            }else if(strcmp(optarg, "false")==0 || strcmp(optarg, "FALSE")==0){
                __IS_GAZEBO = false;
            }else{
                FILE_LOG(logERROR) << opt;
                FILE_LOG(logERROR) << "Invalid option for Gazebo";
                FILE_LOG(logERROR) << "Valid options are \"true\", \"TRUE\", \"false\", \"FALSE\"";
                FILE_LOG(logERROR) << "Use default value";
            }
            break;
        case 'f':
            if(strcmp(optarg, "true")==0 || strcmp(optarg, "TRUE")==0){
                __IS_FOG = true;
            }else if(strcmp(optarg, "false")==0 || strcmp(optarg, "FALSE")==0){
                __IS_FOG = false;
            }else{
                FILE_LOG(logERROR) << opt;
                FILE_LOG(logERROR) << "Invalid option for FOG";
                FILE_LOG(logERROR) << "Valid options are \"true\", \"TRUE\", \"false\", \"FALSE\"";
                FILE_LOG(logERROR) << "Use default value";
            }
            break;
        case 'r':
            if(strcmp(optarg, "true")==0 || strcmp(optarg, "TRUE")==0){
                __IS_ROS = true;
            }else if(strcmp(optarg, "false")==0 || strcmp(optarg, "FALSE")==0){
                __IS_ROS = false;
            }else{
                FILE_LOG(logERROR) << opt;
                FILE_LOG(logERROR) << "Invalid option for ROS";
                FILE_LOG(logERROR) << "Valid options are \"true\", \"TRUE\", \"false\", \"FALSE\"";
                FILE_LOG(logERROR) << "Use default value";
            }
            break;
        case '?':
            if(optopt == 'g'){
                FILE_LOG(logERROR) << "Option for Gazebo";
                FILE_LOG(logERROR) << "Valid options are \"true\", \"TRUE\", \"false\", \"FALSE\"";
            }else if(optopt == 'f'){
                FILE_LOG(logERROR) << "Option for FOG";
                FILE_LOG(logERROR) << "Valid options are \"true\", \"TRUE\", \"false\", \"FALSE\"";
            }
        }
    }

    cout << endl;
    FILE_LOG(logERROR) << "=========Daemon Setting==========";
    FILE_LOG(logWARNING) << argv[0];
    if(__IS_GAZEBO)     FILE_LOG(logWARNING) << "Daemon for Gazebo";
    else                FILE_LOG(logWARNING) << "Daemon for Robot";
    if(__IS_FOG)        FILE_LOG(logWARNING) << "FOG is used";
    else                FILE_LOG(logWARNING) << "FOG is not used";
    if(__IS_ROS)        FILE_LOG(logWARNING) << "ROS is used";
    else                FILE_LOG(logWARNING) << "ROS is not used";
    FILE_LOG(logERROR) << "=================================";
    cout << endl;
}

int main(int argc, char *argv[])
{

    // Copyright
    cout << endl;
    cout << " \033[31m######################################################################\n";
    cout << " #                                                                    #\n";
    cout << " #  PODO Version 2.2                                                  #\n";
    cout << " #  Copyright 2016 Rainbow Robotics Co.                               #\n";
    cout << " #                                                                    #\n";
    cout << " #  Main developer: Jeongsoo Lim                                      #\n";
    cout << " #  E-mail: yjs0497@kaist.ac.kr                                       #\n";
    cout << " #                                                                    #\n";
    cout << " #  We touch the core!                                                #\n";
    cout << " #                                                                    #\n";
    cout << " ######################################################################\n";

    // Termination signal
    signal(SIGTERM, CatchSignals);       // "kill" from shell
    signal(SIGINT,  CatchSignals);       // Ctrl-c
    signal(SIGHUP,  CatchSignals);       // shell termination
    signal(SIGKILL, CatchSignals);
    signal(SIGSEGV, CatchSignals);

    // Block memory swapping
    mlockall(MCL_CURRENT|MCL_FUTURE);

    CheckArguments(argc, argv);

    if(RBCore_Initialize() == false){
        FILE_LOG(logERROR) << "Core Initialization Failed..";
        return 0;
    }

    // Parameters Initialization =========================================================
    dec(cout); // decimal number print out
    // MPC parameters setting (Pump Control)
    sharedREF->N_PrevPump = 20;
    sharedREF->dT_PrevPump = 0.100;
    sharedREF->Flag_PumpControlMPC = false;
    for(int i=0;i<=MAX_PREVIEW;i++) {
        for(int j=0;j<MAX_MC;j++) {
            sharedREF->LoadPressureReference_Future[i][j] = 35.0;
            sharedREF->ActFlowrateReference_Future[i][j] = 0.0;
        }
    }
    sharedSEN->PUMP[0].CurrentSettingVoltage = 100.0;
    // ===================================================================================

    while(IS_WORKING){
        usleep(100*1000);

        switch(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND){
        case DAEMON4LIGHT_PROCESS_CREATE:
        {
            FILE_LOG(logINFO) << "CMD: DAEMON4LIGHT_PROCESS_CREATE";
            pmHandler->OpenAL(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_INT[0]);
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }
        case DAEMON4LIGHT_PROCESS_KILL:
        {
            FILE_LOG(logINFO) << "CMD: DAEMON4LIGHT_PROCESS_KILL";
            pmHandler->CloseAL(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_INT[0]);
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }

        case DAEMON4LIGHT_INIT_CHECK_DEVICE:
        {
            FILE_LOG(logINFO) << "CMD: DAEMON4LIGHT_INIT_CHECK_DEVICE";
            if(__IS_GAZEBO) {FILE_LOG(logINFO) << "Gazebo doesn't need DAEMON_INIT_CHECK_DEVICE";}
            else{
                if(IS_CAN_OK)   {RBCMD_InitCheckDevice();}
                else            {FILE_LOG(logWARNING) << "CAN device not set";}
            }
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }
        case DAEMON4LIGHT_CAN_CHANNEL_ARRANGE:
        {
            FILE_LOG(logINFO) << "CMD: DAEMON4LIGHT_CAN_CHANNEL_ARRANGE";
            if(__IS_GAZEBO) {FILE_LOG(logINFO) << "Gazebo doesn't need DAEMON_INIT_CHECK_DEVICE";}
            else{
                if(IS_CAN_OK)   {RBCMD_CAN_Channel_Arrange();}
                else            {FILE_LOG(logWARNING) << "CAN device not set";}
            }
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }

        case DAEMON4LIGHT_SAVEDATA:
        {
            if(sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0]) {
                save_Flag = true;
                FILE_LOG(logWARNING) << "SAVING DAEMON DATA START!!";
            } else {
                save_Flag = false;
                save_File();
                FILE_LOG(logERROR) << "SAVING DAEMON DATA END!!";
            }

            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }

        case DAEMON4LIGHT_MOTION_ENCODER_ZERO:
        {
            FILE_LOG(logINFO) << "CMD: DAEMON4LIGHT_MOTION_ENCODER_ZERO";
            if(__IS_GAZEBO) {FILE_LOG(logINFO) << "Gazebo doesn't need DAEMON_SENSOR_ENCODER_RESET";}
            else{
                if(IS_CAN_OK)   {RBCMD_EncoderZero();}
                else            {FILE_LOG(logWARNING) << "CAN device not set";}
            }
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }
        case DAEMON4LIGHT_MOTION_FET_ONOFF:
        {
            FILE_LOG(logINFO) << "CMD: DAEMON_INIT_FET_ONOFF";
            if(__IS_GAZEBO) {FILE_LOG(logINFO) << "Gazebo doesn't need DAEMON_INIT_FET_ONOFF";}
            else{
                if(IS_CAN_OK)   {RBCMD_InitFETOnOff();}
                else            {FILE_LOG(logWARNING) << "CAN device not set";}
            }
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }
        case DAEMON4LIGHT_MOTION_REF_ONOFF:
        {
            FILE_LOG(logINFO) << "CMD: DAEMON_MOTION_REF_ONOFF";
            if(IS_CAN_OK)   {
                RBCMD_MotionRefOnOff();
            } else {
                FILE_LOG(logWARNING) << "CAN device not set";
            }
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }

        case DAEMON4LIGHT_MOTION_BOARDTEST:
        {
            FILE_LOG(logINFO) << "CMD: DAEMON4LIGHT_MOTION_BOARDTEST";
            RBCMD_HCB_BoardTest();
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }

        case DAEMON4LIGHT_MOTION_BOARDTEST_ERRORRESET:
        {
            FILE_LOG(logINFO) << "CMD: DAEMON4LIGHT_MOTION_BOARDTEST_REF";
            RBCMD_HCB_BoardTest_ErrorClear();
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }

        case DAEMON4LIGHT_MOTION_TORQUEFORCE_NULLING:
        {
            FILE_LOG(logINFO) << "CMD: DAEMON4LIGHT_MOTION_TORQUEFORCE_NULLING";
            if(__IS_GAZEBO) {FILE_LOG(logINFO) << "Gazebo doesn't need DAEMON4LIGHT_MOTION_PRES_NULLING";}
            else{
                if(IS_CAN_OK)   {RBCMD_HCB_TorqueForce_Nulling();}
                else            {FILE_LOG(logWARNING) << "CAN device not set";}
            }
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }

        case DAEMON4LIGHT_MOTION_PRES_NULLING:
        {
            FILE_LOG(logINFO) << "CMD: DAEMON4LIGHT_MOTION_PRES_NULLING";
            if(__IS_GAZEBO) {FILE_LOG(logINFO) << "Gazebo doesn't need DAEMON4LIGHT_MOTION_PRES_NULLING";}
            else{
                if(IS_CAN_OK)   {RBCMD_HCB_Pres_Nulling();}
                else            {FILE_LOG(logWARNING) << "CAN device not set";}
            }
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }

        case DAEMON4LIGHT_MOTION_REF_RESET:
        {
            FILE_LOG(logINFO) << "CMD: DAEMON4LIGHT_MOTION_REF_RESET";
            if(__IS_GAZEBO) {FILE_LOG(logINFO) << "Gazebo doesn't need DAEMON4LIGHT_MOTION_PRES_NULLING";}
            else{
                if(IS_CAN_OK)   {RBCMD_HCB_AllReference_Reset();}
                else            {FILE_LOG(logWARNING) << "CAN device not set";}
            }
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }

        case DAEMON4LIGHT_MOTION_REQUEST_ENABLE_DISABLE:
        {
            //            FILE_LOG(logINFO) << "CMD: DAEMON4LIGHT_MOTION_REQUEST_ENABLE_DISABLE";
            RBCMD_HCB_RequestEnableDisable();
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }
        case DAEMON4LIGHT_MOTION_ASK_EVERYTHING:
        {
            FILE_LOG(logINFO) << "CMD: DAEMON4LIGHT_MOTION_ASK_EVERYTHING";
            RBCMD_HCB_AskEverything();
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }

        case DAEMON4LIGHT_MOTION_SET_EVERYTHING:
        {
            FILE_LOG(logINFO) << "CMD: DAEMON4LIGT_MOTION_SET_EVERYTHING";
            RBCMD_HCB_SetEverything();
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }

        case DAEMON4LIGHT_INIT_CAN_RESET:
        {
            FILE_LOG(logINFO) << "CMD: DAEMON4LIGHT_INIT_CAN_RESET";
            canHandler->RBResetCAN();
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }

        case DAEMON4LIGHT_MOTION_SET_BNO:
        {
            FILE_LOG(logINFO) << "CMD: DAEMON4LIGT_MOTION_SET_BNO";
            RBCMD_HCB_SetBNO();
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }

        case DAEMON4LIGHT_MOTION_CHANGE_POSorTOR:
        {
            // Position Ctrl or Force Ctrl
            for(int i=0;i<12;i++) {
                sharedREF->PosOrFor_Selection[i] = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[i];
                if(sharedREF->PosOrFor_Selection[i]!=sharedREF->PosOrFor_Selection_last[i]) FILE_LOG(logINFO) << "Joint("<< i <<") Control Mode Change!";
            }
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }

        case DAEMON4LIGHT_MOTION_CHANGE_SUPPLYPRES_MODE:
        {
            // Constant Supply Pressure (0) or Variable Supply Pressure (1)
            sharedREF->PumpSupplyPressureChange = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
            for(int i=0;i<_NO_OF_MC;i++) {
                _DEV_MC[i].HCB_CMD_VariableSupplyPressure(sharedREF->PumpSupplyPressureChange);
            }
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }

        // Pump Command //////////////////////////////////////////////////

        case DAEMON4LIGHT_PUMP_ASK_STATUS:
        {
            FILE_LOG(logINFO) << "CMD: DAEMON4LIGHT_PUMP_ASK_STATUS";
            int CMD_TYPE = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];

            if(CMD_TYPE == 81) _DEV_PC[0].ASK_SPEED_REF();
            if(CMD_TYPE == 82) _DEV_PC[0].ASK_CONTROL_MODE_ONOFF();
            if(CMD_TYPE == 90) _DEV_PC[0].ASK_PUMPDATA_REQUEST_FLAG();

            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }

        case DAEMON4LIGHT_PUMP_SEND_COMMAND:
        {
            FILE_LOG(logINFO) << "CMD: DAEMON4LIGHT_PUMP_SEND_COMMAND";
            int CMD_TYPE = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_INT[0];
            int CMD_DATA = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_INT[1];

            if(CMD_TYPE == 182) {
                sharedSEN->PUMP[0].CurrentSettingVoltage = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_DOUBLE[0];
                _DEV_PC[0].CMD_CONTROL_MODE_ON();
            }
            if(CMD_TYPE == 183) _DEV_PC[0].CMD_CONTROL_MODE_OFF();
            if(CMD_TYPE == 188) _DEV_PC[0].CMD_PRESSURE_NULL();
            if(CMD_TYPE == 190) _DEV_PC[0].CMD_PUMPDATA_REQUEST_FLAG(CMD_DATA);

            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }

        case DAEMON4LIGHT_PUMP_MANUAL_REFSPEED_SET:
        {
            FILE_LOG(logINFO) << "CMD: DAEMON4LIGHT_PUMP_MANUAL_REFSPEED_SET";
            int REF_SPEED = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_INT[0];

            int REF_SPEED_MIN = 200;
            int REF_SPEED_MAX = (int)(0.95*sharedSEN->PUMP[0].CurrentSettingVoltage/100.0*3000.0);

            if(REF_SPEED > REF_SPEED_MAX) {
                REF_SPEED = REF_SPEED_MAX;
                FILE_LOG(logWARNING) << "Pump speed reference is saturated! (Maximum : " << REF_SPEED_MAX << " rpm)";
            } else if (REF_SPEED < REF_SPEED_MIN) {
                REF_SPEED = REF_SPEED_MIN;
                FILE_LOG(logWARNING) << "Pump speed reference is saturated! (Minimum : " << REF_SPEED_MIN << " rpm)";
            }
            FILE_LOG(logINFO) << "Ref Pump Speed : " << REF_SPEED;

            _DEV_PC[0].CMD_SPEED_REF(REF_SPEED);
            sharedREF->PumpVelocityReference[0] = (double)REF_SPEED;

            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }

        case DAEMON4LIGHT_PUMP_ACTIVE_VELOCITY_CONTROL_ONOFF:
        {
            FILE_LOG(logINFO) << "CMD: DAEMON4LIGHT_PUMP_ACTIVE_VELOCITY_CONTROL_ONOFF";
            int ONOFF = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_INT[0];
            _DEV_PC[0].ActivePumpControl_ONOFF(ONOFF);

            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }

        // Sensor Setting //////////////////////////////////////////////////

            //        case DAEMON4LIGHT_SENSOR_SENSOR_ONOFF:
            //        {
            //            FILE_LOG(logINFO) << "CMD: DAEMON_SENSOR_SENSOR_ONOFF";
            //            if(__IS_GAZEBO) {FILE_LOG(logINFO) << "Gazebo doesn't need DAEMON_SENSOR_SENSOR_ONOFF";}
            //            else{
            //                if(IS_CAN_OK)   {RBCMD_SensorOnOff();}
            //                else            {FILE_LOG(logWARNING) << "CAN device not set";}
            //            }
            //            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            //            break;
            //        }

        case DAEMON4LIGHT_SENSOR_FT_ONOFF:
        {
            FILE_LOG(logINFO) << "CMD: DAEMON4LIGHT_SENSOR_FT_ONOFF";
            if(__IS_GAZEBO) {FILE_LOG(logINFO) << "Gazebo doesn't need DAEMON_SENSOR_IMU_OFFSET_SET";}
            else{
                if(IS_CAN_OK)   {RBCMD_SensorFTOnOff();}
                else            {FILE_LOG(logWARNING) << "CAN device not set";}
            }
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }
        case DAEMON4LIGHT_SENSOR_FT_NULL:
        {
            FILE_LOG(logINFO) << "CMD: DAEMON4LIGHT_SENSOR_FT_ONOFF";
            if(__IS_GAZEBO) {FILE_LOG(logINFO) << "Gazebo doesn't need DAEMON_SENSOR_IMU_OFFSET_SET";}
            else{
                if(IS_CAN_OK)   {RBCMD_SensorFTNull();}
                else            {FILE_LOG(logWARNING) << "CAN device not set";}
            }
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }
        case DAEMON4LIGHT_SENSOR_IMU_ONOFF:
        {
            FILE_LOG(logINFO) << "CMD: DAEMON4LIGHT_SENSOR_IMU_ONOFF";
            if(__IS_GAZEBO) {FILE_LOG(logINFO) << "Gazebo doesn't need DAEMON_SENSOR_IMU_OFFSET_SET";}
            else{
                if(IS_CAN_OK)   {RBCMD_SensorIMUONOFF();}
                else            {FILE_LOG(logWARNING) << "CAN device not set";}
            }
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }
        case DAEMON4LIGHT_SENSOR_IMU_NULL:
        {
            FILE_LOG(logINFO) << "CMD: DAEMON4LIGHT_SENSOR_IMU_ONOFF";
            if(__IS_GAZEBO) {FILE_LOG(logINFO) << "Gazebo doesn't need DAEMON_SENSOR_IMU_NULL";}
            else{
                if(IS_CAN_OK)   {RBCMD_SensorIMUNull();}
                else            {FILE_LOG(logWARNING) << "CAN device not set";}
            }
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }
        case DAEMON4LIGHT_SENSOR_IMU_FIND_OFFSET:
        {
            FILE_LOG(logINFO) << "CMD: DAEMON4LIGHT_SENSOR_IMU_FIND_OFFSET";
            if(__IS_GAZEBO) {FILE_LOG(logINFO) << "Gazebo doesn't need DAEMON4LIGHT_SENSOR_IMU_FIND_OFFSET";}
            else{
                if(IS_CAN_OK)   {RBCMD_SensorIMUOffsetSetting();}
                else            {FILE_LOG(logWARNING) << "CAN device not set";}
            }
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }

        case DAEMON4LIGHT_ENC_ZERO:
        {
            FILE_LOG(logINFO) << "CMD: DAEMON4LIGHT_ENC_ZERO";
            RBCMD_EncZero();
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }
        case DAEMON4LIGHT_FINDHOME:
        {
            FILE_LOG(logINFO) << "CMD: DAEMON4LIGHT_HOME_POS";
            RBCMD_FindHome();
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }

        case DAEMON4LIGHT_SIMLATION_MODE_ONOFF:
        {
            FILE_LOG(logINFO) << "CMD: DAEMON4LIGHT_SIMLATION_MODE_ONOFF";
            // __IS_CHOREONOID == 0 : Robot mode
            // __IS_CHOREONOID == 1 : Simulation(choreonoid) mode
            sharedREF->Simulation_DataEnable = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_INT[0];

            if (sharedREF->Simulation_DataEnable == 0) FILE_LOG(logINFO) << "MODE : ROBOT";
            else if (sharedREF->Simulation_DataEnable == 1)  FILE_LOG(logINFO) << "MODE : CHOREONOID";
            sharedCMD->COMMAND[RBCORE_PODO_NO].USER_COMMAND = DAEMON4LIGHT_NO_ACT;
            break;
        }

        }


        // Command acceptance check for AL -------------------------------
        //        for(int i=2; i<_NO_OF_AL; i++){ // i=0 Daemon, i=1 PODOLAN
        //            if(sharedCMD->COMMAND[i].USER_COMMAND != 0 &&
        //                sharedCMD->COMMAND[i].USER_COMMAND != 100 &&
        //                sharedCMD->CommandAccept[i] == false){ // NO_ACT should be 0 or 100
        //                    _ALCommandCnt[i]++;
        //            }else{
        //                _ALCommandCnt[i] = 0;
        //            }

        //            // AL didn't accept command for (100ms X 5 = 500ms)
        //            if(_ALCommandCnt[i] > 5){
        //                sharedCMD->ErrorInform2GUI |= (1<<i);
        //                sharedCMD->CommandAccept[i] = true;
        //            }
        //        }
        // ----------------------------------------------------------------
    }


    RBCore_Termination();

    usleep(1000*1000);
    return 0;
}

void RBCore_RTThreadCon(void *)
{

    rt_task_set_periodic(NULL, TM_NOW, RT_TIMER_PERIOD_MS*1000*1000); // 1000000ns = 1000us = 1ms

    RTIME   th_start, th_stop;
    int     waitCnt, waitOK;
    double  timeGap = 0.0;
    double  finalJointPosRef[MOTOR_2CH];
    double  finalJointVelRef[MOTOR_2CH];
    double  finalJointFTRef[MOTOR_2CH];
    double  finalValvePosRef[MOTOR_2CH];

    double  finalPumpVelocityRef = 0.0;

    //reference set zero
    for(int i=0; i<_NO_OF_MC; i++){
        for(int j=0; j<MAX_JOINT; j++){
            sharedREF->AngleReference[0][i][j] = 0.0;
            sharedREF->AngVelReference[0][i][j] = 0.0;
            sharedREF->TorqueReference[0][i][j] = 0.0;
            sharedREF->ActPosReference[0][i][j] = 0.0;
            sharedREF->ActVelReference[0][i][j] = 0.0;
            sharedREF->ActForceReference[0][i][j] = 0.0;
            _DEV_MC[i].Joints[j].HCB_Ref.ReferencePosition = 0.0; // Actuator Position (Unit : mm or deg)
            _DEV_MC[i].Joints[j].HCB_Ref.ReferenceVelocity = 0.0; // Actuator Velocity (Unit : mm/s or deg/s)
            _DEV_MC[i].Joints[j].HCB_Ref.ReferenceForceTorque = 0.0; // Actuator Force or Torque (Unit : N or Nm)
            _DEV_MC[i].HCB_SendReference_PosVel(false);
        }
    }


    while(IS_WORKING)
    {
        rt_task_wait_period(NULL);

        // Read Sensor & Encoder =====================================
        if(canHandler->IsWorking()){

            if(sharedREF->Simulation_DataEnable==0) // robot mode (by BUYOUN)
            {
                THREAD_UpdateHCBInfo();
                THREAD_ReadEncoder();
                THREAD_ReadValvePos();
                THREAD_ReadOtherInfo();

                THREAD_ReadNewIMU();
                THREAD_ReadFT();

                THREAD_ReadPumpData();

                sharedSEN->CAN_Enabled = _CANOUT_ENABLED;
                //                sharedSEN->SEN_Enabled = _SENSOR_ENABLED;
            }

            sharedSEN->IMU[0].NO_RESPONSE_CNT = NO_RESPONSE_CNT;
        }

        if(sharedREF->Simulation_DataEnable==1) // simulation mode (by BUYOUN)
        {
            // Joint angle and torque sensor update
            for(int i=0; i<_NO_OF_MC; i++) {
                for(int j=0; j<MAX_JOINT; j++) {
                    sharedSEN->ENCODER[i][j].CurrentAngle = sharedREF->Simulation_AngleSensor[i];
                    sharedSEN->ENCODER[i][j].CurrentAngVel = sharedREF->Simulation_VelocitySensor[i];
                    sharedSEN->ENCODER[i][j].CurrentTorque = sharedREF->Simulation_TorqueSensor[i];
                }
            }

            sharedSEN->IMU[0].Wx_B  = sharedREF->Simulation_IMU[0].Wx_B;
            sharedSEN->IMU[0].Wy_B  = sharedREF->Simulation_IMU[0].Wy_B;
            sharedSEN->IMU[0].Wz_B  = sharedREF->Simulation_IMU[0].Wz_B;

            sharedSEN->IMU[0].Ax_B  = sharedREF->Simulation_IMU[0].Ax_B;
            sharedSEN->IMU[0].Ay_B  = sharedREF->Simulation_IMU[0].Ay_B;
            sharedSEN->IMU[0].Az_B  = sharedREF->Simulation_IMU[0].Az_B;

            // By Simulation Data
            double q0 = sharedREF->Simulation_IMU[0].Q[0];
            double q1 = sharedREF->Simulation_IMU[0].Q[1];
            double q2 = sharedREF->Simulation_IMU[0].Q[2];
            double q3 = sharedREF->Simulation_IMU[0].Q[3];

            double psi = atan2(2 * (q1*q2 + q0*q3), 1 - 2 * (q2*q2 + q3*q3));    // z-axis
            double theta = -1 * asin(2 * (q1*q3 - q0*q2));                       // y'-axis
            double phi = atan2(2 * (q2*q3 + q0*q1), 1 - 2 * (q1*q1 + q2*q2));    // x''-axis

            Matrix3_InEKF R; // Pelvis frame orientation w.r.t the global frame.
            R.M[0][0] = 1.0-2.0*(q2*q2+q3*q3);
            R.M[0][1] = 2.0*(q1*q2-q3*q0);
            R.M[0][2] = 2.0*(q0*q2+q1*q3);
            R.M[1][0] = 2.0*(q1*q2+q3*q0);
            R.M[1][1] = 1.0-2.0*(q1*q1+q3*q3);
            R.M[1][2] = 2.0*(q2*q3-q0*q1);
            R.M[2][0] = 2.0*(q1*q3-q0*q2);
            R.M[2][1] = 2.0*(q0*q1+q2*q3);
            R.M[2][2] = 1.0-2.0*(q2*q2+q1*q1);

            // Projection to Yaw rotation zero
            bool Flag_YawZero = true;
            if(Flag_YawZero) {
                if(fabs(sqrt(R.M[1][0]*R.M[1][0]+R.M[0][0]*R.M[0][0]))<1e-6) {
                    if(R.M[0][0] < 0.0) {
                        sharedSEN->IMU[0].Roll = 0.0;
                        sharedSEN->IMU[0].Pitch = 90.0*R2D;
                        sharedSEN->IMU[0].Yaw = 0.0;
                        Matrix3_InEKF R_yawzero(0.0, 0.0, 1.0,
                                                0.0, 1.0, 0.0,
                                                1.0, 0.0, 0.0);

                        Vector3_InEKF Uw(sharedSEN->IMU[0].Wx_B, sharedSEN->IMU[0].Wy_B, sharedSEN->IMU[0].Wz_B);
                        Vector3_InEKF W_G = R_yawzero*Uw;
                        sharedSEN->IMU[0].Wx_G = W_G.v[0];
                        sharedSEN->IMU[0].Wy_G = W_G.v[1];
                        sharedSEN->IMU[0].Wz_G = W_G.v[2];

                        Vector3_InEKF Ua(sharedSEN->IMU[0].Ax_B, sharedSEN->IMU[0].Ay_B, sharedSEN->IMU[0].Az_B);
                        Vector3_InEKF A_G = R_yawzero*Ua;
                        sharedSEN->IMU[0].Ax_G = A_G.v[0];
                        sharedSEN->IMU[0].Ay_G = A_G.v[1];
                        sharedSEN->IMU[0].Az_G = A_G.v[2];
                    } else {
                        sharedSEN->IMU[0].Roll = 0.0;
                        sharedSEN->IMU[0].Pitch = -90.0*R2D;
                        sharedSEN->IMU[0].Yaw = 0.0;
                        Matrix3_InEKF R_yawzero(0.0, 0.0, -1.0,
                                                0.0, 1.0, 0.0,
                                                -1.0, 0.0, 0.0);

                        Vector3_InEKF Uw(sharedSEN->IMU[0].Wx_B, sharedSEN->IMU[0].Wy_B, sharedSEN->IMU[0].Wz_B);
                        Vector3_InEKF W_G = R_yawzero*Uw;
                        sharedSEN->IMU[0].Wx_G = W_G.v[0];
                        sharedSEN->IMU[0].Wy_G = W_G.v[1];
                        sharedSEN->IMU[0].Wz_G = W_G.v[2];

                        Vector3_InEKF Ua(sharedSEN->IMU[0].Ax_B, sharedSEN->IMU[0].Ay_B, sharedSEN->IMU[0].Az_B);
                        Vector3_InEKF A_G = R_yawzero*Ua;
                        sharedSEN->IMU[0].Ax_G = A_G.v[0];
                        sharedSEN->IMU[0].Ay_G = A_G.v[1];
                        sharedSEN->IMU[0].Az_G = A_G.v[2];
                    }
                } else {
                    double psi = atan2(R.M[1][0], R.M[0][0]);   // z-axis (Yaw)
        //            double theta = -asin(R.M[2][0]);            // y'-axis (Pitch)
        //            double phi = atan2(R.M[2][1], R.M[2][2]);   // x''-axis (Roll)
                    Matrix3_InEKF R_yaw(cos(psi), -sin(psi), 0.0,
                                        sin(psi), cos(psi) , 0.0,
                                        0.0     , 0.0      , 1.0);
                    Matrix3_InEKF R_yawzero = R_yaw.Transpose()*R;

                    sharedSEN->IMU[0].Q[0] = sqrt(1.0+R_yawzero.M[0][0]+R_yawzero.M[1][1]+R_yawzero.M[2][2])/2.0;
                    sharedSEN->IMU[0].Q[1] = (R_yawzero.M[2][1]-R_yawzero.M[1][2])/4.0/sharedSEN->IMU[0].Q[0];
                    sharedSEN->IMU[0].Q[2] = (R_yawzero.M[0][2]-R_yawzero.M[2][0])/4.0/sharedSEN->IMU[0].Q[0];
                    sharedSEN->IMU[0].Q[3] = (R_yawzero.M[1][0]-R_yawzero.M[0][1])/4.0/sharedSEN->IMU[0].Q[0];

                    sharedSEN->IMU[0].Roll = atan2(-R_yawzero.M[1][2],R_yawzero.M[1][1]);
                    sharedSEN->IMU[0].Pitch = atan2(-R_yawzero.M[2][0],R_yawzero.M[0][0]);
                    sharedSEN->IMU[0].Yaw = 0.0;

                    Vector3_InEKF Uw(sharedSEN->IMU[0].Wx_B, sharedSEN->IMU[0].Wy_B, sharedSEN->IMU[0].Wz_B);
                    Vector3_InEKF W_G = R_yawzero*Uw;
                    sharedSEN->IMU[0].Wx_G = W_G.v[0];
                    sharedSEN->IMU[0].Wy_G = W_G.v[1];
                    sharedSEN->IMU[0].Wz_G = W_G.v[2];

                    Vector3_InEKF Ua(sharedSEN->IMU[0].Ax_B, sharedSEN->IMU[0].Ay_B, sharedSEN->IMU[0].Az_B);
                    Vector3_InEKF A_G = R_yawzero*Ua;
                    sharedSEN->IMU[0].Ax_G = A_G.v[0];
                    sharedSEN->IMU[0].Ay_G = A_G.v[1];
                    sharedSEN->IMU[0].Az_G = A_G.v[2];
                }
            } else {
                sharedSEN->IMU[0].Q[0] = q0;
                sharedSEN->IMU[0].Q[1] = q1;
                sharedSEN->IMU[0].Q[2] = q2;
                sharedSEN->IMU[0].Q[3] = q3;

                sharedSEN->IMU[0].Yaw = psi;
                sharedSEN->IMU[0].Pitch = theta;
                sharedSEN->IMU[0].Roll = phi;

                Vector3_InEKF Uw(sharedSEN->IMU[0].Wx_B, sharedSEN->IMU[0].Wy_B, sharedSEN->IMU[0].Wz_B);
                Vector3_InEKF W_G = R*Uw;
                sharedSEN->IMU[0].Wx_G = W_G.v[0];
                sharedSEN->IMU[0].Wy_G = W_G.v[1];
                sharedSEN->IMU[0].Wz_G = W_G.v[2];

                Vector3_InEKF Ua(sharedSEN->IMU[0].Ax_B, sharedSEN->IMU[0].Ay_B, sharedSEN->IMU[0].Az_B);
                Vector3_InEKF A_G = R*Ua;
                sharedSEN->IMU[0].Ax_G = A_G.v[0];
                sharedSEN->IMU[0].Ay_G = A_G.v[1];
                sharedSEN->IMU[0].Az_G = A_G.v[2];
            }

            // Ankle FT Sensor
            sharedSEN->FT[0].Mx = sharedREF->Simulation_FT[0].Mx;
            sharedSEN->FT[0].My = sharedREF->Simulation_FT[0].My;
            sharedSEN->FT[0].Mz = sharedREF->Simulation_FT[0].Mz;
            sharedSEN->FT[0].Fx = sharedREF->Simulation_FT[0].Fx;
            sharedSEN->FT[0].Fy = sharedREF->Simulation_FT[0].Fy;
            sharedSEN->FT[0].Fz = sharedREF->Simulation_FT[0].Fz;
            sharedSEN->FT[1].Mx = sharedREF->Simulation_FT[1].Mx;
            sharedSEN->FT[1].My = sharedREF->Simulation_FT[1].My;
            sharedSEN->FT[1].Mz = sharedREF->Simulation_FT[1].Mz;
            sharedSEN->FT[1].Fx = sharedREF->Simulation_FT[1].Fx;
            sharedSEN->FT[1].Fy = sharedREF->Simulation_FT[1].Fy;
            sharedSEN->FT[1].Fz = sharedREF->Simulation_FT[1].Fz;

        }
        // ===========================================================


        // Change Flag ===============================================
        for(int i=0; i<_NO_OF_MC; i++){
            for(int j=0; j<1; j++){
                sharedCMD->ACK_SIGNAL[i][j] = false;
            }
        }
        for(int i=0; i<_NO_OF_AL; i++){
            sharedCMD->SYNC_SIGNAL[i] = true;
        }
        // ===========================================================

        // Wait Reference ============================================
        th_start = rt_timer_read();
        waitCnt = 0;
        waitOK = false;
        while(1){
            // check the all WriteDoneFlag are enabled
            int notRead = 0;
            for(int i=0; i<_NO_OF_MC; i++){
                for(int j=0; j<1; j++){
                    if(sharedCMD->ACK_SIGNAL[i][j] == false && sharedCMD->MotionOwner[i][j] != RBCORE_PODO_NO){
                        notRead++;
                    }
                }
            }
            if(notRead == 0){
                th_stop = rt_timer_read();
                timeGap = (double)(th_stop - th_start) / (double)1000000.0;
                waitOK = true;
                break;
            }

            if(waitCnt%500 == 0){
                th_stop = rt_timer_read();
                timeGap = (double)(th_stop - th_start) / (double)1000000.0;
                if(timeGap > 2.5){
                    waitOK = false;
                    //                                        FILE_LOG(logWARNING) << "Over 2.5msec";
                    break;
                }
            }
            waitCnt++;
            usleep(2);
        }
        // ===========================================================

        // Write CAN Reference =======================================

        // Motor(Valve) Controller Reference
        for(int i=0; i<_NO_OF_MC; i++){

            for(int j=0; j<MAX_JOINT; j++){
                int motionOwner = sharedCMD->MotionOwner[i][j];

                sharedSEN->ENCODER[i][j].CurrentRefAngle = sharedREF->AngleReference[motionOwner][i][j];
                sharedSEN->ENCODER[i][j].CurrentRefAngVel = sharedREF->AngVelReference[motionOwner][i][j];
                sharedSEN->ENCODER[i][j].CurrentRefTorque = sharedREF->TorqueReference[motionOwner][i][j];

                finalJointPosRef[j] = sharedREF->ActPosReference[motionOwner][i][j];
                _DEV_MC[i].Joints[j].HCB_Ref.ReferencePosition = finalJointPosRef[j]; // Actuator Position (Unit : mm or deg)
                sharedSEN->ENCODER[i][j].CurrentRefActPos = finalJointPosRef[j];

                finalJointVelRef[j] = sharedREF->ActVelReference[motionOwner][i][j];
                _DEV_MC[i].Joints[j].HCB_Ref.ReferenceVelocity = finalJointVelRef[j]; // Actuator Velocity (Unit : mm/s or deg/s)
                sharedSEN->ENCODER[i][j].CurrentRefActVel = finalJointVelRef[j];

                finalJointFTRef[j] = sharedREF->ActForceReference[motionOwner][i][j];
                _DEV_MC[i].Joints[j].HCB_Ref.ReferenceForceTorque = finalJointFTRef[j]; // Actuator Force or Torque (Unit : N or Nm)
                sharedSEN->ENCODER[i][j].CurrentRefActForce = finalJointFTRef[j];

                finalValvePosRef[j] = sharedREF->ValvePosReference[motionOwner][i][j];
                _DEV_MC[i].Joints[j].HCB_Ref.ReferenceValvePos = finalValvePosRef[j]; // Valve Position (Unit : 0~10000uA)
                sharedSEN->ENCODER[i][j].CurrentRefValvePos = finalValvePosRef[j];

                _DEV_MC[i].Joints[j].HCB_Ref.ReferencePumpPressure = sharedREF->PumpPressureReference[0];

                // Send Reference Data to Board
                if(canHandler->IsWorking() &&  _DEV_MC[i].ConnectionStatus) { // Only when CAN Connection is Enabled

                    switch(sharedREF->ValveCtrlMode_Command[i]) {
                    case ValveControlMode_Null: // Valve Control Mode : Null
                    {
                        cout << "\033[1;34m Valve Control Mode (Board ID : " << _DEV_MC[i].BOARD_ID << ") : Null \033[0m" << endl;
                        _DEV_MC[i].HCB_CMD_ControlMode(0);
                        sharedREF->ValveCtrlMode_Command[i] = ValveControlMode_Noact;
                        sharedREF->ValveCtrlMode[i] = ValveControlMode_Null;
                        break;
                    }
                    case ValveControlMode_Opening: // Valve Control Mode : Valve Opening (openloop)
                    {
                        cout << "\033[1;34m Valve Control Mode (Board ID : " << _DEV_MC[i].BOARD_ID << ") : Open Loop \033[0m" << endl;
                        _DEV_MC[i].HCB_CMD_ControlMode(1);
                        sharedREF->ValveCtrlMode_Command[i] = ValveControlMode_Noact;
                        sharedREF->ValveCtrlMode[i] = ValveControlMode_Opening;
                        break;
                    }
                    case ValveControlMode_PosOrFor: // Valve Control Mode : Position or Force Control
                    {
                        cout << "\033[1;34m Valve Control Mode (Board ID : " << _DEV_MC[i].BOARD_ID << ") : Position or Force \033[0m" << endl;
                        _DEV_MC[i].HCB_CMD_ControlMode(2);
                        sharedREF->ValveCtrlMode_Command[i] = ValveControlMode_Noact;
                        sharedREF->ValveCtrlMode[i] = ValveControlMode_PosOrFor;
                        break;
                    }
                    default:
                        break;
                    }

                    switch(sharedREF->ValveCtrlMode[i]) {
                    case ValveControlMode_Null: // Valve Control Mode : Null
                    {
                        break;
                    }
                    case ValveControlMode_Opening: // Valve Control Mode : Valve Opening (openloop)
                    {
                        _DEV_MC[i].HCB_SendReference_ValvePos();
                        break;
                    }
                    case ValveControlMode_PosOrFor: // Valve Control Mode : Position or Force Control
                    {
                        // Joint Stiffness and Damping Change
                        if(sharedREF->StiffnDampChange[i] == true) {
                            if(canHandler->IsWorking() &&  _DEV_MC[i].ConnectionStatus){ // Only when CAN Connection is Enabled
                                _DEV_MC[i].HCB_CMD_JointSpringDamper(sharedREF->ActuatorStiffness[i],sharedREF->ActuatorDamping[i]);
                            }
                            sharedREF->StiffnDampChange[i] = false;
                        }

                        // Control Method Changed
                        if(sharedREF->PosOrFor_Selection[i] != sharedREF->PosOrFor_Selection_last[i]) {
                            if(canHandler->IsWorking() && _DEV_MC[i].ConnectionStatus){ // Only when CAN Connection is Enabled
                                _DEV_MC[i].HCB_CMD_ControlMethodTransition(sharedREF->PosOrFor_Selection[i]);
                            }
                        }
                        sharedREF->PosOrFor_Selection_last[i] = sharedREF->PosOrFor_Selection[i];

                        bool SupplyPressureChangeOnOff = sharedREF->PumpSupplyPressureChange;
                        _DEV_MC[i].HCB_SendReference_PosVel(SupplyPressureChangeOnOff);
                        break;
                    }
                    case ValveControlMode_FindHome: // Valve Control Mode : Find Home
                    {
//                        bool _FINISH = _DEV_MC[i].HCB_Read_FindHomeDone();
//                        if(_FINISH) {
//                            sharedREF->ValveCtrlMode[i] = ValveControlMode_PosOrFor;
//                            sharedREF->PosOrFor_Selection[i] = JointControlMode_Position;
//                            sharedREF->PosOrFor_Selection_last[i] = JointControlMode_Position;
//                        }
                        break;
                    }
                    default:
                        break;
                    }
                }
            }
        }

        // Pump Controller Reference
        for(int i=0;i<_NO_OF_PC; i++)
        {
            finalPumpVelocityRef = sharedREF->PumpVelocityReference[i];
            _DEV_PC[i].ReferencePumpVelocity = finalPumpVelocityRef;
            sharedSEN->PUMP[i].CurrentRefVelocity = finalPumpVelocityRef;

            sharedSEN->PUMP[i].CurrentRefPressure = sharedREF->PumpPressureReference[i];

            _DEV_PC[i].Send_ReferenceSpeed();
        }

        for(int i=0;i<_NO_OF_AL; i++)
        {
            sharedREF->NO_RESPONSE_CNT[i] += 1;
        }
        if(sharedREF->NO_RESPONSE_CNT[4]>20)//JUMP
        {
            for(int i=0;i<_NO_OF_MC;i++)
            {
                if(canHandler->IsWorking() && _CANOUT_ENABLED && _DEV_MC[i].ConnectionStatus)
                {
                    //                    if(_DEV_MC[i].Joints[0].ControlMode==CONTROL_MODE_CUR)
                    //                    {
                    //                       _DEV_MC[i].RBJoint_EnableFeedbackControlDirectly(0,false);//disable
                    //                        _DEV_MC[i].Joints[0].ControlMode=CONTROL_MODE_DISABLED;
                    //                        cout<<"AL NOT RESPONDING DISABLED "<< i <<endl;
                    //                    }
                }
            }
        }

        // Variable Display for debuging.
        _ThreadCnt++;
        if (_ThreadCnt == 500) {
//            for(int i=0;i<12;i++) {
//                int16_t temp1 = _DEV_MC[0].Joints[0].HCB_Data.CurrentTempData1;
//                int16_t temp2 = _DEV_MC[0].Joints[0].HCB_Data.CurrentTempData2;
//                FILE_LOG(logDEBUG3) << "K : (" << i << ") : "<< temp1;
//                FILE_LOG(logDEBUG3) << "D : (" << i << ") : "<< temp2;
//            }


//            FILE_LOG(logDEBUG3) << "Ps : " << sharedREF->PumpPressureReference[0];
//            FILE_LOG(logDEBUG3) << "Ps : " << _DEV_MC[0].Joints[0].HCB_Ref.ReferencePumpPressure;
//            FILE_LOG(logDEBUG3) << "Change? : " << sharedREF->PumpSupplyPressureChange;

            _ThreadCnt = 0;
        }

        // Request Sensor  ===========================================
        if(canHandler->IsWorking()){
            //            THREAD_RequestSensor();
        }

        if(save_Flag == true){
            save_PutData(save_Index);
            save_Index++;
            if(save_Index == (SAVENUM-1))
                save_Index=0;
        }


        if(WaitTime>0) WaitDisplay();
        // ===========================================================
    }
    FILE_LOG(logERROR) << "RTthread will be terminated..";
}

void save_PutData(unsigned int cur_Index)
{
    // System Period
    save_Buf[0][cur_Index] = (double)RT_TIMER_PERIOD_MS/1000.0;

    // ankle linear position(unit:mm) data
    save_Buf[1][cur_Index] = sharedSEN->ENCODER[LAP][0].CurrentRefActPos;
    save_Buf[2][cur_Index] = sharedSEN->ENCODER[LAP][0].CurrentActPos;
    save_Buf[3][cur_Index] = sharedSEN->ENCODER[LAR][0].CurrentRefActPos;
    save_Buf[4][cur_Index] = sharedSEN->ENCODER[LAR][0].CurrentActPos;

    //    save_Buf[1][cur_Index] = 102.0;
    //    save_Buf[2][cur_Index] = 103.0;
}

void SetWaitTime(int T) {
    WaitTime = T;
}
void WaitDisplay(void){
    // just display waiting time (buyoun)
    if(WaitTime%1000 == 0) {
        FILE_LOG(logWARNING) << "Wait for " << WaitTime/1000 << "sec.";
    } WaitTime = WaitTime - RT_TIMER_PERIOD_MS;
    if(WaitTime==0){
        FILE_LOG(logSUCCESS) << "The process is done!!!";
    }
}


// =================================================================================================
// =================================================================================================
// =================================================================================================

void *RBCore_NRTThreadCon(void *)
{
    DRC_GAZEBO_SENSOR   GazeboSensor;
    DRC_GAZEBO_JOINT    GazeboJoint;
    char                *GazeboJoint_and_Type;
    DRC_GAZEBO_GO_CMD   GazeboGain;
    char                *GazeboGain_and_Type;

    GazeboJoint_and_Type = (char*)malloc(sizeof(DRC_GAZEBO_JOINT) + sizeof(int));
    GazeboGain_and_Type = (char*)malloc(sizeof(DRC_GAZEBO_GO_CMD) + sizeof(int));

    RTIME   th_start, th_stop;
    int     waitCnt, waitOK;
    double  timeGap = 0.0;
    double  finalJointRef[MOTOR_2CH];
    //    double  finalJointPWMRef[MOTOR_2CH];
    //    double  finalJointCurrentRef[MOTOR_2CH];

    while(IS_WORKING)
    {
        usleep(10);
        if(lanHandler->ConnectionStatus){
            if(lanHandler->RBLanReadData((char*)(&GazeboSensor), sizeof(GazeboSensor), 0x00) == LAN_NEW_DATA){

                sharedSEN->Sim_Time_sec = GazeboSensor.Sim_Time.sec;
                sharedSEN->Sim_Time_nsec = GazeboSensor.Sim_Time.nsec;

                // Read Sensor & Encoder =====================================
                for(int i=0; i<_NO_OF_FT; i++){
                    sharedSEN->FT[i].Fx = GazeboSensor.FTSensor[i].force[0];
                    sharedSEN->FT[i].Fy = GazeboSensor.FTSensor[i].force[1];
                    sharedSEN->FT[i].Fz = GazeboSensor.FTSensor[i].force[2];
                    sharedSEN->FT[i].Mx = GazeboSensor.FTSensor[i].torque[0];
                    sharedSEN->FT[i].My = GazeboSensor.FTSensor[i].torque[1];
                    sharedSEN->FT[i].Mz = GazeboSensor.FTSensor[i].torque[2];
                }
                for(int i=0; i<_NO_OF_IMU; i++){
                    sharedSEN->IMU[i].Roll      = GazeboSensor.IMUSensor[0];
                    sharedSEN->IMU[i].Pitch     = GazeboSensor.IMUSensor[1];
                    sharedSEN->IMU[i].Yaw       = GazeboSensor.IMUSensor[2];
                    sharedSEN->IMU[i].Wx_B      = GazeboSensor.IMUSensor[3];
                    sharedSEN->IMU[i].Wy_B      = GazeboSensor.IMUSensor[4];
                    sharedSEN->IMU[i].Wz_B      = GazeboSensor.IMUSensor[5];
                    sharedSEN->IMU[i].Ax_B      = GazeboSensor.IMUSensor[6];
                    sharedSEN->IMU[i].Ay_B      = GazeboSensor.IMUSensor[7];
                    sharedSEN->IMU[i].Az_B      = GazeboSensor.IMUSensor[8];
                }

                double tempDouble;
                for(int i=0; i<NO_OF_JOINTS; i++){
                    tempDouble = sharedSEN->ENCODER[MC_GetID(i)][MC_GetCH(i)].CurrentAngle;
                    sharedSEN->ENCODER[MC_GetID(i)][MC_GetCH(i)].CurrentAngle = GazeboSensor.JointCurrentPosition[i];
                    sharedSEN->ENCODER[MC_GetID(i)][MC_GetCH(i)].CurrentAngVel = (sharedSEN->ENCODER[MC_GetID(i)][MC_GetCH(i)].CurrentAngle - tempDouble) / (double)RT_TIMER_PERIOD_MS * 1000.0;
                }


                sharedSEN->FOG.Roll     = sharedSEN->IMU[0].Roll;
                sharedSEN->FOG.Pitch    = sharedSEN->IMU[0].Pitch;
                sharedSEN->FOG.Yaw      = sharedSEN->IMU[0].Yaw;
                sharedSEN->FOG.RollVel     = sharedSEN->IMU[0].Wx_B*RBCORE_PI/180.;
                sharedSEN->FOG.PitchVel    = sharedSEN->IMU[0].Wy_B*RBCORE_PI/180.;
                sharedSEN->FOG.YawVel      = sharedSEN->IMU[0].Wz_B*RBCORE_PI/180.0;

                sharedSEN->CAN_Enabled = _CANOUT_ENABLED;
                sharedSEN->SEN_Enabled = _SENSOR_ENABLED;
                // ===========================================================

                // Change Flag ===============================================
                for(int i=0; i<_NO_OF_MC; i++){
                    for(int j=0; j<1; j++){
                        sharedCMD->ACK_SIGNAL[i][j] = false;
                    }
                }
                for(int i=0; i<_NO_OF_AL; i++){
                    sharedCMD->SYNC_SIGNAL[i] = true;
                }
                // ===========================================================

                // Wait Reference ============================================
                th_start = rt_timer_read();
                waitCnt = 0;
                waitOK = false;
                while(1){
                    // check the all WriteDoneFlag are enabled
                    int notRead = 0;
                    for(int i=0; i<_NO_OF_MC; i++){
                        for(int j=0; j<1; j++){
                            if(sharedCMD->ACK_SIGNAL[i][j] == false && sharedCMD->MotionOwner[i][j] != RBCORE_PODO_NO){
                                notRead++;
                            }
                        }
                    }
                    if(notRead == 0){
                        th_stop = rt_timer_read();
                        timeGap = (double)(th_stop - th_start) / (double)1000000.0;
                        waitOK = true;
                        break;
                    }

                    if(waitCnt%500 == 0){
                        th_stop = rt_timer_read();
                        timeGap = (double)(th_stop - th_start) / (double)1000000.0;
                        if(timeGap > 2.5){
                            waitOK = false;
                            FILE_LOG(logWARNING) << "Over 2.5msec";
                            break;
                        }
                    }
                    waitCnt++;
                    usleep(2);
                }
                // ===========================================================

                // Write LAN Reference =======================================
                for(int i=0; i<_NO_OF_MC; i++){
                    for(int j=0; j<1; j++){
                        int motionOwner = sharedCMD->MotionOwner[i][j];

                        finalJointRef[j] = sharedREF->AngleReference[motionOwner][i][j];
                        _DEV_MC[i].Joints[j].HCB_Ref.ReferencePosition = finalJointRef[j];
                        sharedSEN->ENCODER[i][j].CurrentRefAngle = finalJointRef[j];

                        //                        finalJointPWMRef[j] = sharedREF->JointFFpwm[motionOwner][i][j];
                        //                        _DEV_MC[i].Joints[j].RefPWM = finalJointPWMRef[j];
                        //                        _DEV_MC[i].Joints[j].RefFFpwm = finalJointPWMRef[j];
                        //                        sharedSEN->ENCODER[i][j].PWMffout = finalJointPWMRef[j];

                        //                        finalJointCurrentRef[j] = sharedREF->COCOAQCurrent_REF[motionOwner][i];
                        //                        _DEV_MC[i].Joints[j].RefQCurrent = finalJointCurrentRef[j];
                        //                        sharedSEN->ENCODER[i][j].CurrentQCurrentReference = finalJointCurrentRef[j];
                    }
                }
                for(int i=0; i<NO_OF_JOINTS; i++){
                    GazeboJoint.JointReference[i] = _DEV_MC[MC_GetID(i)].Joints[MC_GetCH(i)].HCB_Ref.ReferencePosition;
                }
                int type = GAZEBO_TYPE_JOINT;
                //memcpy(GazeboJoint_and_Type, &type, sizeof(int));
                //memcpy(&(GazeboJoint_and_Type[sizeof(int)]), &GazeboJoint, sizeof(DRC_GAZEBO_JOINT));

                lanHandler->RBLanWriteData(&type, sizeof(int));
                lanHandler->RBLanWriteData(&GazeboJoint, sizeof(DRC_GAZEBO_JOINT));
                //lanHandler->RBLanWriteData(GazeboJoint_and_Type, sizeof(GazeboJoint_and_Type));
                // ===========================================================

                for(int i=0; i<MAX_MANUAL_CAN; i++){
                    if(sharedCMD->ManualCAN[i].status == MANUALCAN_NEW){
                        sharedCMD->ManualCAN[i].status = MANUALCAN_WRITING;

                        int id = sharedCMD->ManualCAN[i].id;
                        int dlc = sharedCMD->ManualCAN[i].dlc;
                        int bno = -1;
                        for(int j=0; j<_NO_OF_MC; j++){
                            if(_DEV_MC[j].ID_SEND_GENERAL == id){
                                bno = j;
                                break;
                            }
                        }

                        if(bno >= 0){
                            if(dlc > 0){
                                if(sharedCMD->ManualCAN[i].data[0] == 0x6F){    // Gain Override
                                    int ch = sharedCMD->ManualCAN[i].data[1]-1;
                                    int joint = -1;
                                    for(int j=0; j<NO_OF_JOINTS; j++){
                                        if(MC_GetID(j) == bno && MC_GetCH(j) == ch){
                                            joint = j;
                                            break;
                                        }
                                    }
                                    //FILE_LOG(logWARNING) << "ManualCAN JOINT: " << joint;
                                    if(joint >= 0){
                                        int gain = sharedCMD->ManualCAN[i].data[2];
                                        int timeMS = (int)(sharedCMD->ManualCAN[i].data[3] | (sharedCMD->ManualCAN[i].data[4] << 8));

                                        int type = GAZEBO_TYPE_GAINOVERRIDE;
                                        GazeboGain.gain = gain;
                                        GazeboGain.joint = joint;
                                        GazeboGain.timeMs = timeMS;
                                        FILE_LOG(logWARNING) << "GainOverride: " << joint << " , " << gain << ", " << timeMS;
                                        lanHandler->RBLanWriteData(&type, sizeof(int));
                                        lanHandler->RBLanWriteData(&GazeboGain, sizeof(DRC_GAZEBO_GO_CMD));
                                    }
                                }else if(sharedCMD->ManualCAN[i].data[0] == 0x11){ // Home
                                    int ch = sharedCMD->ManualCAN[i].data[1]-1;
                                    int joint = -1;
                                    if(ch == -1){
                                        FILE_LOG(logWARNING) << "Only support single channel";
                                    }else{
                                        for(int j=0; j<NO_OF_JOINTS; j++){
                                            if(MC_GetID(j) == bno && MC_GetCH(j) == ch){
                                                joint = j;
                                                break;
                                            }
                                        }
                                        if(joint >= 0){
                                            int type = GAZEBO_TYPE_HOME;
                                            FILE_LOG(logWARNING) << "Find Home: " << joint;
                                            lanHandler->RBLanWriteData(&type, sizeof(int));
                                            lanHandler->RBLanWriteData(&joint, sizeof(int));
                                        }
                                    }
                                }
                            }

                        }
                        sharedCMD->ManualCAN[i].status = MANUALCAN_EMPTY;
                    }
                }
            }
        }else{ // without Gazebo connection
            // No Sensor Data

            // Change Flag ===============================================
            for(int i=0; i<_NO_OF_MC; i++){
                for(int j=0; j<1; j++){
                    sharedCMD->ACK_SIGNAL[i][j] = false;
                }
            }
            for(int i=0; i<_NO_OF_AL; i++){
                sharedCMD->SYNC_SIGNAL[i] = true;
            }
            // ===========================================================

            // Wait Reference ============================================
            th_start = rt_timer_read();
            waitCnt = 0;
            waitOK = false;
            while(1){
                // check the all WriteDoneFlag are enabled
                int notRead = 0;
                for(int i=0; i<_NO_OF_MC; i++){
                    for(int j=0; j<1; j++){
                        if(sharedCMD->ACK_SIGNAL[i][j] == false && sharedCMD->MotionOwner[i][j] != RBCORE_PODO_NO){
                            notRead++;
                        }
                    }
                }
                if(notRead == 0){
                    th_stop = rt_timer_read();
                    timeGap = (double)(th_stop - th_start) / (double)1000000.0;
                    waitOK = true;
                    break;
                }

                if(waitCnt%500 == 0){
                    th_stop = rt_timer_read();
                    timeGap = (double)(th_stop - th_start) / (double)1000000.0;
                    if(timeGap > 2.5){
                        waitOK = false;
                        FILE_LOG(logWARNING) << "Over 2.5msec";
                        break;
                    }
                }
                waitCnt++;
                usleep(2);
            }
            // ===========================================================

            // Move Reference ============================================
            for(int i=0; i<_NO_OF_MC; i++){
                for(int j=0; j<1; j++){
                    int motionOwner = sharedCMD->MotionOwner[i][j];

                    finalJointRef[j] = sharedREF->AngleReference[motionOwner][i][j];
                    _DEV_MC[i].Joints[j].HCB_Ref.ReferencePosition = finalJointRef[j];
                    sharedSEN->ENCODER[i][j].CurrentRefAngle = finalJointRef[j];
                }
            }
            // ===========================================================

            // No Manual Control

            usleep(5*1000);
        }
    }
    return NULL;
}

// =================================================================================================
// =================================================================================================
// =================================================================================================

void save_File()
{
    FILE* fp;
    unsigned int i, j;
    fp = fopen("LIGHT_Daemon_Data.txt", "w");

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
}

void RBCMD_InitCheckDevice(){
    DO_CANCHECK = true;

    for(int i=0; i<_NO_OF_MC; i++){
        _DEV_MC[i].HCB_CANCheck();
        usleep(10*1000);
        for(int j=0; j<MOTOR_2CH; j++){
            sharedSEN->ENCODER[i][j].BoardConnection = _DEV_MC[i].ConnectionStatus;
        }
    }
    for(int i=0; i<_NO_OF_PC; i++){
        _DEV_PC[i].CANCheck();
        sharedSEN->PUMP[i].BoardConnection = _DEV_PC[i].ConnectionStatus;
    }
    for(int i=0; i<_NO_OF_IMU; i++){
        _DEV_IMU[i].CANCheck();
        sharedSEN->IMU[i].BoardConnection = _DEV_IMU[i].ConnectionStatus;
    }
    for(int i=0; i<_NO_OF_FT; i++){
        _DEV_FT[i].CANCheck();
        sharedSEN->FT[i].BoardConnection = _DEV_FT[i].ConnectionStatus;
    }

    //    for(int i=0; i<_NO_OF_SP; i++){
    //        _DEV_SP[i].RBBoard_CANCheck(RT_TIMER_PERIOD_MS);
    //        sharedSEN->SP[i].BoardConnection = _DEV_SP[i].ConnectionStatus;
    //    }
    //    for(int i=0; i<_NO_OF_OF; i++){
    //        _DEV_OF[i].RBBoard_CANCheck(RT_TIMER_PERIOD_MS);
    //        sharedSEN->OF.BoardConnection = _DEV_OF[i].ConnectionStatus;
    //    }
    DO_CANCHECK = false;

}

void RBCMD_CAN_Channel_Arrange(){
    DO_CANCHECK = true;

    // Motion Controller CAN Channel Arrangement
    for(int i=0; i<_NO_OF_MC; i++){
        _DEV_MC[i].HCB_CANChannel_Arrange();
        usleep(100*1000);
        for(int j=0; j<MOTOR_2CH; j++){
            sharedSEN->ENCODER[i][j].BoardConnection = _DEV_MC[i].ConnectionStatus;
        }
    }

    // Pump Controller CAN Channel Arrangement
    for(int i=0; i<_NO_OF_PC; i++){
        _DEV_PC[i].CANChannel_Arrange();
        usleep(100*1000);
        sharedSEN->PUMP[i].BoardConnection = _DEV_PC[i].ConnectionStatus;
    }

    // IMU Controller CAN Channel Arrangement
    for(int i=0; i<_NO_OF_IMU; i++){
        _DEV_IMU[i].CANChannel_Arrange();
        usleep(100*1000);
        sharedSEN->IMU[i].BoardConnection = _DEV_IMU[i].ConnectionStatus;
    }

    // IMU Controller CAN Channel Arrangement
    for(int i=0; i<_NO_OF_FT; i++){
        _DEV_FT[i].CANChannel_Arrange();
        usleep(100*1000);
        sharedSEN->FT[i].BoardConnection = _DEV_FT[i].ConnectionStatus;
    }

    RBDataBase DB;
    DB.SetFilename("Core_Config.db");

    std::cout << " \n\n========================================================\n";
    std::cout << "           [CAN Channel Arrangement Result]        \n";
    // Motion Controller DB Update
    for(int i=0; i<_NO_OF_MC; i++){
        int BOARD_ID = _DEV_MC[i].BOARD_ID;
        int CAN_CH = _DEV_MC[i].CAN_CHANNEL;

        if (_DEV_MC[i].ConnectionStatus) {
            if(DB.UpdateDB_CAN_Channel_MC(BOARD_ID, CAN_CH) == false){
                std::cout << "DB update fail!! \n";
            }
            std::cout << ">>> Board(" << BOARD_ID << ": " << _DEV_MC[i].BOARD_NAME.toStdString().data() << ") is \033[32mconnected \033[0mto channel [" << CAN_CH << "] \n";
        } else {
            if(DB.UpdateDB_CAN_Channel_MC(BOARD_ID, 0) == false){
                std::cout << "DB update fail!! \n";
            }
            std::cout << ">>> Board(" << _DEV_MC[i].BOARD_ID << ": " << _DEV_MC[i].BOARD_NAME.toStdString().data() << ") is \033[31mnot connected.\033[0m \n";
        }
    }
    // Pump Controller DB Update
    for(int i=0; i<_NO_OF_PC; i++){
        int BOARD_ID = _DEV_PC[i].BOARD_ID;
        int CAN_CH = _DEV_PC[i].CAN_CHANNEL;

        if (_DEV_PC[i].ConnectionStatus) {
            if(DB.UpdateDB_CAN_Channel_PC(BOARD_ID, CAN_CH) == false){
                std::cout << "DB update fail!! \n";
            }
            std::cout << ">>> Board(" << BOARD_ID << ": PUMP) is \033[32mconnected \033[0mto channel [" << CAN_CH << "] \n";
        } else {
            if(DB.UpdateDB_CAN_Channel_PC(BOARD_ID, 0) == false){
                std::cout << "DB update fail!! \n";
            }
            std::cout << ">>> Board(" << _DEV_PC[i].BOARD_ID << ": PUMP) is \033[31mnot connected.\033[0m \n";
        }
    }
    // IMU Controller DB Update
    for(int i=0; i<_NO_OF_IMU; i++){
        int BOARD_ID = _DEV_IMU[i].BOARD_ID;
        int CAN_CH = _DEV_IMU[i].CAN_CHANNEL;

        if (_DEV_IMU[i].ConnectionStatus) {
            if(DB.UpdateDB_CAN_Channel_IMU(BOARD_ID, CAN_CH) == false){
                std::cout << "DB update fail!! \n";
            }
            std::cout << ">>> Board(" << BOARD_ID << ": IMU) is \033[32mconnected \033[0mto channel [" << CAN_CH << "] \n";
        } else {
            if(DB.UpdateDB_CAN_Channel_IMU(BOARD_ID, 0) == false){
                std::cout << "DB update fail!! \n";
            }
            std::cout << ">>> Board(" << _DEV_IMU[i].BOARD_ID << ": IMU) is \033[31mnot connected.\033[0m \n";
        }
    }
    // FT Controller DB Update
    for(int i=0; i<_NO_OF_FT; i++){
        int BOARD_ID = _DEV_FT[i].BOARD_ID;
        int CAN_CH = _DEV_FT[i].CAN_CHANNEL;

        if (_DEV_FT[i].ConnectionStatus) {
            if(DB.UpdateDB_CAN_Channel_FT(BOARD_ID, CAN_CH) == false){
                std::cout << "DB update fail!! \n";
            }
            std::cout << ">>> Board(" << BOARD_ID << ": "<< _DEV_FT[i].BOARD_NAME.toStdString().data() << ") is \033[32mconnected \033[0mto channel [" << CAN_CH << "] \n";
        } else {
            if(DB.UpdateDB_CAN_Channel_FT(BOARD_ID, 0) == false){
                std::cout << "DB update fail!! \n";
            }
            std::cout << ">>> Board(" << _DEV_FT[i].BOARD_ID << ": " << _DEV_FT[i].BOARD_NAME.toStdString().data() << ") is \033[31mnot connected.\033[0m \n";
        }
    }
    std::cout << " ========================================================\n\n";

    DO_CANCHECK = false;
}

void RBCMD_EncoderZero(){
    int id = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
    int ch = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1];

    if(id == -1){ // All
        for(int i=0; i<_NO_OF_MC; i++){
            for(int j=0; j<MAX_JOINT; j++){
                _DEV_MC[i].HCB_CMD_EncoderZero();
                sharedCMD->MotionOwner[i][j] = RBCORE_PODO_NO;

                for(int k=0; k<_NO_OF_AL; k++){
                    sharedREF->AngleReference[k][i][j] = 0.0;
                }
                _DEV_MC[i].Joints[j].HCB_Ref.ReferencePosition = 0.0;
                _DEV_MC[i].Joints[j].HCB_Ref.ReferenceVelocity = 0.0;
                _DEV_MC[i].MoveJoints[j].RefAngleCurrent = 0.0;
            }
        }
    }else{  // Each
        _DEV_MC[id].HCB_CMD_EncoderZero();
        sharedCMD->MotionOwner[id][ch] = RBCORE_PODO_NO;
        for(int k=0; k<_NO_OF_AL; k++){
            sharedREF->AngleReference[k][id][ch] = 0.0;
        }
        _DEV_MC[id].Joints[ch].HCB_Ref.ReferencePosition = 0.0;
        _DEV_MC[id].Joints[ch].HCB_Ref.ReferenceVelocity = 0.0;
        _DEV_MC[id].MoveJoints[ch].RefAngleCurrent = 0.0;
    }
}
void RBCMD_InitFETOnOff(){
    int id = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
    int ch = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1];
    int onoff = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2];   // 0->off
    if(onoff != 0) onoff = 1;

    if(id == -1){ // All
        if(onoff == 1) {
            FILE_LOG(logINFO) << "ALL FET WILL TURN ON!";
        } else {
            FILE_LOG(logINFO) << "ALL FET WILL TURN OFF...";
        }

        for(int i=0; i<_NO_OF_MC; i++){
            for(int j=0; j<MAX_JOINT; j++){
                _DEV_MC[i].HCB_CMD_FETOnOff(onoff);
            }
        }
    }else{  // Each
        _DEV_MC[id].HCB_CMD_FETOnOff(onoff);
    }
}

void RBCMD_MotionRefOnOff()
{
    int Target = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
    int type = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1];

    if(Target == -1) { // to all joints
        for(int i=0; i<_NO_OF_MC; i++){
            sharedREF->ValveCtrlMode_Command[i] = type;
        }
    } else if (Target==RAP||Target==RAR) {
        sharedREF->ValveCtrlMode_Command[RAP] = type;
        sharedREF->ValveCtrlMode_Command[RAR] = type;
    } else if (Target==LAP||Target==LAR) {
        sharedREF->ValveCtrlMode_Command[LAP] = type;
        sharedREF->ValveCtrlMode_Command[LAR] = type;
    } else {
        sharedREF->ValveCtrlMode_Command[Target] = type;
    }
}

void RBCMD_SensorFTOnOff(){
    char ONOFF = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
    for(int i=0; i<_NO_OF_FT; i++){
        _DEV_FT[i].RBFT_RequestONOFF(ONOFF);
    }
}
void RBCMD_SensorFTNull(){
    for(int i=0; i<_NO_OF_FT; i++){
        _DEV_FT[i].RBFT_Nulling(0);
    }
}

void RBCMD_SensorIMUONOFF(){
    char ONOFF = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
    _DEV_IMU[0].RBIMU_RequestONOFF(ONOFF);
}

void RBCMD_SensorIMUNull(){
    _DEV_IMU[0].RBIMU_Nulling();
}


void RBCMD_SensorIMUOffsetSetting()
{
    double t = 0.0;
    int cnt = 0;
    double roll_sum = 0.0;
    double pitch_sum = 0.0;

    while(t<5.0) {
        cnt++;
        roll_sum += sharedSEN->IMU[0].Roll;
        pitch_sum += sharedSEN->IMU[0].Pitch;
        usleep(1000); // 1msec
        t += 0.001;
        if(cnt%1000 == 0) { FILE_LOG(logWARNING) << "IMU Offset Setting... Remaining " << (5-cnt/1000) << "sec.";}
    }

    _DEV_IMU[0].ROLL_OFFSET = roll_sum/(double)cnt;
    _DEV_IMU[0].PITCH_OFFSET = pitch_sum/(double)cnt;

    FILE_LOG(logSUCCESS) << "IMU offset is found! ";
    FILE_LOG(logSUCCESS) << "Roll offset : " << _DEV_IMU[0].ROLL_OFFSET*R2D << "(deg)";
    FILE_LOG(logSUCCESS) << "Pitch offset : " << _DEV_IMU[0].PITCH_OFFSET*R2D << "(deg)";
}


// ========================================================================


void RBCMD_MotionSetPWM(){
    int id = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
    int ch = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1];
    double PWM = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_DOUBLE[0];

    sharedCMD->MotionOwner[id][ch] = RBCORE_PODO_NO;
}

void RBCMD_MotionErrorClear(){
    //    int id = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
    //    int ch = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1];
    //    int mode = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2];
    //    if(mode != 0) mode = 1;

    ErrorClearStart = true;

    //    if(mode == 0){  // just error clear
    //        if(id == -1){ // All
    //            for(int i=0; i<_NO_OF_MC; i++){
    //                for(int j=0; j<1; j++){
    //                    _DEV_MC[i].RBJoint_ClearErrorFlag(j+1);
    //                }
    //            }
    //        }
    //        else{  // Each
    //            _DEV_MC[id].RBJoint_ClearErrorFlag(ch+1);
    //        }
    //    }
    //    else{  // error clear + joint recovery
    //           // reference out disable & get motion ownership for all joints
    //        for(int i=0; i<_NO_OF_MC; i++){
    //            _DEV_MC[i].RBBoard_ReferenceOutEnable(false);
    //            for(int j=0; j<1; j++){
    //                sharedCMD->MotionOwner[i][j] = RBCORE_PODO_NO;
    //            }
    //        }

    //        // encoder enable
    //        for(int i=0; i<_NO_OF_MC; i++){
    //            _DEV_MC[i].RBBoard_RequestEncoder(1);
    //        }

    //        // sleep for encoder read
    //        usleep(30*1000);

    //        for(int i=0; i<_NO_OF_MC; i++){
    //            for(int j=0; j<1; j++){
    //                // update reference with encoder (exception RWH LWH RHAND LHAND)
    //                if((i == 4 && j == 1) || (i == 10 && j == 1) || (i == 21 && j == 1) || (i == 22 && j == 1)){
    //                    _DEV_MC[i].MoveJoints[j].RefAngleCurrent = 0.0;
    //                }else{
    //                    _DEV_MC[i].MoveJoints[j].RefAngleCurrent = sharedSEN->ENCODER[i][j].CurrentPosition;
    //                }
    //            }
    //        }

    //        if(id == -1){ // All
    //            for(int i=0; i<_NO_OF_MC; i++){
    //                for(int j=0; j<1; j++){
    //                    // error clear
    //                    _DEV_MC[i].RBJoint_ClearErrorFlag(j+1);

    //                    // FET on & CTRL on
    //                    _DEV_MC[i].RBJoint_EnableFETDriver(j+1, true);
    //                    _DEV_MC[i].RBJoint_EnableFeedbackControl(j+1, true);
    //                }
    //            }
    //        }else{  // Each
    //            // error clear
    //            _DEV_MC[id].RBJoint_ClearErrorFlag(ch+1);

    //            // FET on & CTRL on
    //            _DEV_MC[id].RBJoint_EnableFETDriver(ch+1, true);
    //            _DEV_MC[id].RBJoint_EnableFeedbackControl(ch+1, true);
    //        }

    //        // wait for settling
    //        usleep(10*1000);

    //        // reference out enable
    //        for(int i=0; i<_NO_OF_MC; i++){
    //            for(int j=0; j<1; j++){
    //                _DEV_MC[i].RBBoard_ReferenceOutEnable(true);
    //            }
    //        }
    //    }
    ErrorClearStart = false;
}

void RBCMD_CANEnableDisable(){
    _CANOUT_ENABLED = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
}

void RBCMD_HCB_RequestEnableDisable(void)
{
    int id = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
    int ch = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1];
    bool enable = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[2];

    int type = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[3];

    if(id==-1)
    {
        if(enable) cout << "Request to Get Data from All Board." << endl;
        else cout << "Stop Getting Data from All Board." << endl;

        for(int i = 0;i<_NO_OF_MC; i++)
        {

            if(sharedSEN->ENCODER[i][0].BoardConnection)
            {
                switch (type)
                {
                case 2:
                    cout << "Data Type : Position & Velocity" << endl;
                    _DEV_MC[i].HCB_CMD_Request_PosVel(enable);
                    usleep(10*1000);
                    break;
                case 3:
                    cout << "Data Type : Valve Position" << endl;
                    _DEV_MC[i].HCB_CMD_Request_ValvePos(enable);
                    usleep(10*1000);
                    break;
                case 4:
                    cout << "Data Type : Other Information" << endl;
                    _DEV_MC[i].HCB_CMD_Request_OtherInfo(enable);
                    usleep(10*1000);
                    break;
                case -1:
                    cout << "Data Type : ALL DATA" << endl;
                    _DEV_MC[i].HCB_CMD_Request_PosVel(enable);
                    usleep(10*1000);
                    _DEV_MC[i].HCB_CMD_Request_ValvePos(enable);
                    usleep(10*1000);
                    _DEV_MC[i].HCB_CMD_Request_OtherInfo(enable);
                    usleep(10*1000);
                    break;
                default:
                    break;
                }
            } else {
                FILE_LOG(logWARNING) << "BoardConnection(" << i << ") is not checked yet.";
            }

        }
    }
    else
    {
        if(enable) cout << "Request to Get Data from Board(" << id <<")"<< endl;
        else cout << "Stop Getting Data from Board(" << id <<")"<< endl;
        switch (type)
        {
        case 2:
            cout << "Data Type : Position & Velocity" << endl;
            _DEV_MC[id].HCB_CMD_Request_PosVel(enable);
            usleep(10*1000);
            break;
        case 3:
            cout << "Data Type : Valve Position" << endl;
            _DEV_MC[id].HCB_CMD_Request_ValvePos(enable);
            usleep(10*1000);
            break;
        case 4:
            cout << "Data Type : Other Information" << endl;
            _DEV_MC[id].HCB_CMD_Request_OtherInfo(enable);
            usleep(10*1000);
            break;
        case -1:
            cout << "Data Type : ALL DATA" << endl;
            _DEV_MC[id].HCB_CMD_Request_PosVel(enable);
            usleep(10*1000);
            _DEV_MC[id].HCB_CMD_Request_ValvePos(enable);
            usleep(10*1000);
            _DEV_MC[id].HCB_CMD_Request_OtherInfo(enable);
            usleep(10*1000);
            break;
        default:
            break;
        }
    }
}

void RBCMD_HCB_SetEverything()
{
    int _BN = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];

    HCB_INFO H = sharedCMD->COMMAND[RBCORE_PODO_NO].HCB_Info;

    if(sharedSEN->ENCODER[_BN][0].HCB_Info.OPERATION_MODE != H.OPERATION_MODE
            || sharedSEN->ENCODER[_BN][0].HCB_Info.SENSING_MODE != H.SENSING_MODE
            || sharedSEN->ENCODER[_BN][0].HCB_Info.CURRENT_CONTROL_MODE != H.CURRENT_CONTROL_MODE)
    {
        _DEV_MC[_BN].HCB_CMD_BoardOperationMode(H.OPERATION_MODE,H.SENSING_MODE,H.CURRENT_CONTROL_MODE);
        usleep(10*1000);
    }

    if(sharedSEN->ENCODER[_BN][0].HCB_Info.CAN_FREQ != H.CAN_FREQ)
    {
        _DEV_MC[_BN].HCB_CMD_CANFrequency(H.CAN_FREQ);
        usleep(10*1000);
    }

    //    _DEV_MC[_BN].HCB_CMD_ControlMode(H.CONTROL_MODE);
    //    usleep(10*1000);

    if(sharedSEN->ENCODER[_BN][0].HCB_Info.JOINTENC_DIRECTION != H.JOINTENC_DIRECTION)
    {
        _DEV_MC[_BN].HCB_CMD_JointEncDir(H.JOINTENC_DIRECTION);
        usleep(10*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.VALVEINPUT_DIRECTION != H.VALVEINPUT_DIRECTION)
    {
        _DEV_MC[_BN].HCB_CMD_ValveInputDir(H.VALVEINPUT_DIRECTION);
        usleep(10*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.VALVEENC_DIRECTION != H.VALVEENC_DIRECTION)
    {
        _DEV_MC[_BN].HCB_CMD_ValveEncDir(H.VALVEENC_DIRECTION);
        usleep(10*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.BOARD_IN_VOLTAGE != H.BOARD_IN_VOLTAGE)
    {
        _DEV_MC[_BN].HCB_CMD_BoardInputVoltage(H.BOARD_IN_VOLTAGE);
        usleep(10*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.BOARD_OPER_VOLTAGE != H.BOARD_OPER_VOLTAGE)
    {
        _DEV_MC[_BN].HCB_CMD_ValveOperationVoltage(H.BOARD_OPER_VOLTAGE);
        usleep(10*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.VARIABLE_SUPPLYPRES_ONOFF != H.VARIABLE_SUPPLYPRES_ONOFF)
    {
        _DEV_MC[_BN].HCB_CMD_VariableSupplyPressure(H.VARIABLE_SUPPLYPRES_ONOFF);
        usleep(10*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.VALVE_P_KP != H.VALVE_P_KP)
    {
        _DEV_MC[_BN].HCB_CMD_PIDGain(0,H.VALVE_P_KP,H.VALVE_P_KI,H.VALVE_P_KD);
        usleep(10*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.VALVE_P_KI != H.VALVE_P_KI)
    {
        _DEV_MC[_BN].HCB_CMD_PIDGain(0,H.VALVE_P_KP,H.VALVE_P_KI,H.VALVE_P_KD);
        usleep(10*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.VALVE_P_KD != H.VALVE_P_KD)
    {
        _DEV_MC[_BN].HCB_CMD_PIDGain(0,H.VALVE_P_KP,H.VALVE_P_KI,H.VALVE_P_KD);
        usleep(10*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.JOINT_P_KP != H.JOINT_P_KP)
    {
        _DEV_MC[_BN].HCB_CMD_PIDGain(1,H.JOINT_P_KP,H.JOINT_P_KI,H.JOINT_P_KD);
        usleep(10*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.JOINT_P_KI != H.JOINT_P_KI)
    {
        _DEV_MC[_BN].HCB_CMD_PIDGain(1,H.JOINT_P_KP,H.JOINT_P_KI,H.JOINT_P_KD);
        usleep(10*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.JOINT_P_KD != H.JOINT_P_KD)
    {
        _DEV_MC[_BN].HCB_CMD_PIDGain(1,H.JOINT_P_KP,H.JOINT_P_KI,H.JOINT_P_KD);
        usleep(10*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.JOINT_F_KP != H.JOINT_F_KP)
    {
        _DEV_MC[_BN].HCB_CMD_PIDGain(2,H.JOINT_F_KP,H.JOINT_F_KI,H.JOINT_F_KD);
        usleep(10*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.JOINT_F_KI != H.JOINT_F_KI)
    {
        _DEV_MC[_BN].HCB_CMD_PIDGain(2,H.JOINT_F_KP,H.JOINT_F_KI,H.JOINT_F_KD);
        usleep(10*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.JOINT_F_KD != H.JOINT_F_KD)
    {
        _DEV_MC[_BN].HCB_CMD_PIDGain(2,H.JOINT_F_KP,H.JOINT_F_KI,H.JOINT_F_KD);
        usleep(10*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.JOINT_SPRING != H.JOINT_SPRING)
    {
        _DEV_MC[_BN].HCB_CMD_JointSpringDamper(H.JOINT_SPRING,H.JOINT_DAMPER);
        usleep(10*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.JOINT_DAMPER != H.JOINT_DAMPER)
    {
        _DEV_MC[_BN].HCB_CMD_JointSpringDamper(H.JOINT_SPRING,H.JOINT_DAMPER);
        usleep(10*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.VALVE_CENTER_POS != H.VALVE_CENTER_POS
            || sharedSEN->ENCODER[_BN][0].HCB_Info.VALVE_DZ_PLUS != H.VALVE_DZ_PLUS
            || sharedSEN->ENCODER[_BN][0].HCB_Info.VALVE_DZ_MINUS != H.VALVE_DZ_MINUS)
    {
        _DEV_MC[_BN].HCB_CMD_ValveDeadzoneNCenterPos(H.VALVE_CENTER_POS,H.VALVE_DZ_PLUS,H.VALVE_DZ_MINUS);
        usleep(10*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.VEL_COMPENSATION_K != H.VEL_COMPENSATION_K)
    {
        _DEV_MC[_BN].HCB_CMD_VelCompensationGain(H.VEL_COMPENSATION_K);
        usleep(10*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.ACTUATOR_COMPLIANCE_K != H.ACTUATOR_COMPLIANCE_K)
    {
        _DEV_MC[_BN].HCB_CMD_ComplianceGain(H.ACTUATOR_COMPLIANCE_K);
        usleep(10*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.VALVE_FEEDFORWARD != H.VALVE_FEEDFORWARD)
    {
        _DEV_MC[_BN].HCB_CMD_ValveFeedforward(H.VALVE_FEEDFORWARD);
        usleep(10*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.BULK_MODULUS != H.BULK_MODULUS)
    {
        _DEV_MC[_BN].HCB_CMD_BulkModulus(H.BULK_MODULUS);
        usleep(10*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.VOL_A != H.VOL_A)
    {
        _DEV_MC[_BN].HCB_CMD_ChamberVolume(H.VOL_A,H.VOL_B);
        usleep(10*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.VOL_B != H.VOL_B)
    {
        _DEV_MC[_BN].HCB_CMD_ChamberVolume(H.VOL_A,H.VOL_B);
        usleep(10*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.PIS_AREA_A != H.PIS_AREA_A)
    {
        _DEV_MC[_BN].HCB_CMD_PistonArea(H.PIS_AREA_A,H.PIS_AREA_B);
        usleep(10*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.PIS_AREA_B != H.PIS_AREA_B)
    {
        _DEV_MC[_BN].HCB_CMD_PistonArea(H.PIS_AREA_A,H.PIS_AREA_B);
        usleep(10*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.SUP_PRES != H.SUP_PRES)
    {
        _DEV_MC[_BN].HCB_CMD_SupplyAndReturnPressure(H.SUP_PRES,0);
//        _DEV_MC[_BN].HCB_CMD_SupplyAndReturnPressure(H.SUP_PRES,H.RET_PRES);
        usleep(10*1000);
    }
//    if(sharedSEN->ENCODER[_BN][0].HCB_Info.RET_PRES != H.RET_PRES)
//    {
//        _DEV_MC[_BN].HCB_CMD_SupplyAndReturnPressure(H.SUP_PRES,H.RET_PRES);
//        usleep(10*1000);
//    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.JOINTENC_LIMIT_MINUS != H.JOINTENC_LIMIT_MINUS)
    {
        _DEV_MC[_BN].HCB_CMD_JointEncLimit(H.JOINTENC_LIMIT_MINUS,H.JOINTENC_LIMIT_PLUS);
        usleep(10*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.JOINTENC_LIMIT_PLUS != H.JOINTENC_LIMIT_PLUS)
    {
        _DEV_MC[_BN].HCB_CMD_JointEncLimit(H.JOINTENC_LIMIT_MINUS,H.JOINTENC_LIMIT_PLUS);
        usleep(10*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.PIS_STROKE != H.PIS_STROKE)
    {
        _DEV_MC[_BN].HCB_CMD_PistonStroke(H.PIS_STROKE);
        usleep(10*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.VALVEPOS_LIMIT_MINUS != H.VALVEPOS_LIMIT_MINUS)
    {
        _DEV_MC[_BN].HCB_CMD_ValvePositionLimit(H.VALVEPOS_LIMIT_MINUS,H.VALVEPOS_LIMIT_PLUS);
        usleep(10*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.VALVEPOS_LIMIT_PLUS != H.VALVEPOS_LIMIT_PLUS)
    {
        _DEV_MC[_BN].HCB_CMD_ValvePositionLimit(H.VALVEPOS_LIMIT_MINUS,H.VALVEPOS_LIMIT_PLUS);
        usleep(10*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.JOINTENC_PPP != H.JOINTENC_PPP)
    {
        _DEV_MC[_BN].HCB_CMD_EncoderPulsePerPosition(H.JOINTENC_PPP);
        usleep(10*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.FORCESEN_PPF != H.FORCESEN_PPF)
    {
        _DEV_MC[_BN].HCB_CMD_SensorPulsePerForceTorque(H.FORCESEN_PPF);
        usleep(10*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.PRESSEN_PPP_A != H.PRESSEN_PPP_A ||
            sharedSEN->ENCODER[_BN][0].HCB_Info.PRESSEN_PPP_B != H.PRESSEN_PPP_B)
    {
        _DEV_MC[_BN].HCB_CMD_SensorPulsePerPressure(H.PRESSEN_PPP_A, H.PRESSEN_PPP_B);
        usleep(10*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.CONST_FRIC != H.CONST_FRIC)
    {
        _DEV_MC[_BN].HCB_CMD_ConstantFriction(H.CONST_FRIC);
        usleep(10*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.HOMEPOS_OFFSET != H.HOMEPOS_OFFSET)
    {
        _DEV_MC[_BN].HCB_CMD_HomeposOffset(H.HOMEPOS_OFFSET);
        usleep(10*1000);
    }
    if(sharedSEN->ENCODER[_BN][0].HCB_Info.HOMEPOS_VALVE_OPENING != H.HOMEPOS_VALVE_OPENING)
    {
        _DEV_MC[_BN].HCB_CMD_HomeposValveOpening(H.HOMEPOS_VALVE_OPENING);
        usleep(10*1000);
    }

    _DEV_MC[_BN].Joints[0].HCB_Info = H;
    FILE_LOG(logSUCCESS) << "Board(" << _BN << "), Parameter Setting is Done!";

}

void RBCMD_HCB_SetBNO()
{
    int _BN = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
    int target_BNO = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[1];

    _DEV_MC[_BN].HCB_CMD_BoardNumber(target_BNO);
    usleep(5*1000);
}


void RBCMD_EncZero(){
    int _BN = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
    if(_BN == -1) { // all joint
        for(int i = 0;i<_NO_OF_MC; i++)
        {
            _DEV_MC[i].HCB_CMD_EncZero();
            usleep(10*1000);
        }
    } else {
        _DEV_MC[_BN].HCB_CMD_EncZero();
        usleep(10*1000);
    }
}

void RBCMD_FindHome(void)
{
    int _BN = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];

    if(_BN == -1) {
        for(int i = 0;i<12; i++)
        {
            FindHomeOperation(i);
            sharedREF->PosOrFor_Selection[i] = JointControlMode_Position; // position control mode
        }
    }
    else if(_BN == -2){ // LIGHT Findhome process 1st stage
        FindHomeOperation(WST);
        FindHomeOperation(RAP);
        FindHomeOperation(RAR);
        FindHomeOperation(LAP);
        FindHomeOperation(LAR);
        sharedREF->PosOrFor_Selection[WST] = JointControlMode_Position;
        sharedREF->PosOrFor_Selection[RAP] = JointControlMode_Position;
        sharedREF->PosOrFor_Selection[RAR] = JointControlMode_Position;
        sharedREF->PosOrFor_Selection[LAP] = JointControlMode_Position;
        sharedREF->PosOrFor_Selection[LAR] = JointControlMode_Position;
    }
    else if(_BN == -3){ // LIGHT Findhome process 2nd stage
        FindHomeOperation(RHR);
        FindHomeOperation(RHY);
        FindHomeOperation(RHP);
        FindHomeOperation(RKN);
        FindHomeOperation(LHR);
        FindHomeOperation(LHY);
        FindHomeOperation(LHP);
        FindHomeOperation(LKN);
        sharedREF->PosOrFor_Selection[RHR] = JointControlMode_Position;
        sharedREF->PosOrFor_Selection[RHY] = JointControlMode_Position;
        sharedREF->PosOrFor_Selection[RHP] = JointControlMode_Position;
        sharedREF->PosOrFor_Selection[RKN] = JointControlMode_Position;
        sharedREF->PosOrFor_Selection[LHR] = JointControlMode_Position;
        sharedREF->PosOrFor_Selection[LHY] = JointControlMode_Position;
        sharedREF->PosOrFor_Selection[LHP] = JointControlMode_Position;
        sharedREF->PosOrFor_Selection[LKN] = JointControlMode_Position;
    }
    else if(_BN == RAP||_BN == RAR){
        FindHomeOperation(RAP);
        FindHomeOperation(RAR);
        sharedREF->PosOrFor_Selection[RAP] = JointControlMode_Position;
        sharedREF->PosOrFor_Selection[RAR] = JointControlMode_Position;
    }
    else if(_BN == LAP||_BN == LAR){
        FindHomeOperation(LAP);
        FindHomeOperation(LAR);
        sharedREF->PosOrFor_Selection[LAP] = JointControlMode_Position;
        sharedREF->PosOrFor_Selection[LAR] = JointControlMode_Position;
    } else {
        FindHomeOperation(_BN);
        sharedREF->PosOrFor_Selection[_BN] = JointControlMode_Position;
    }
}

void FindHomeOperation(int _BN)
{
    for(int k=0; k<_NO_OF_AL; k++){
        sharedREF->AngleReference[k][_BN][0] = 0.0;
        sharedREF->AngVelReference[k][_BN][0] = 0.0;
        sharedREF->ActPosReference[k][_BN][0] = 0.0;
        sharedREF->ActVelReference[k][_BN][0] = 0.0;
    }
    sharedSEN->ENCODER[_BN][0].CurrentRefAngle = 0.0;
    sharedSEN->ENCODER[_BN][0].CurrentRefAngVel = 0.0;
    sharedSEN->ENCODER[_BN][0].CurrentRefActPos = 0.0;
    sharedSEN->ENCODER[_BN][0].CurrentRefActVel = 0.0;
    _DEV_MC[_BN].Joints[0].HCB_Ref.ReferencePosition = 0.0;
    _DEV_MC[_BN].Joints[0].HCB_Ref.ReferenceVelocity = 0.0;

    // Position reference sending off
    sharedREF->ValveCtrlMode[_BN] = ValveControlMode_FindHome;

    _DEV_MC[_BN].HCB_CMD_ControlMode(22);
    usleep(100);
}


void RBCMD_HCB_AskEverything()
{
    int _BN = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];

    _DEV_MC[_BN].HCB_ASK_BoardOperationMode();
    usleep(10*1000);
    _DEV_MC[_BN].HCB_ASK_CANFrequency();
    usleep(10*1000);
    _DEV_MC[_BN].HCB_ASK_ControlMode();
    usleep(10*1000);
    _DEV_MC[_BN].HCB_ASK_JointEncDir();
    usleep(10*1000);
    _DEV_MC[_BN].HCB_ASK_ValveInputDir();
    usleep(10*1000);
    _DEV_MC[_BN].HCB_ASK_ValveEncDir();
    usleep(10*1000);
    _DEV_MC[_BN].HCB_ASK_BoardInputVoltage();
    usleep(10*1000);
    _DEV_MC[_BN].HCB_ASK_ValveOperationVoltage();
    usleep(10*1000);
    _DEV_MC[_BN].HCB_ASK_VariableSupplyPressure();
    usleep(10*1000);

    _DEV_MC[_BN].HCB_ASK_PIDGain(0);
    usleep(10*1000);
    _DEV_MC[_BN].HCB_ASK_PIDGain(1);
    usleep(10*1000);
    _DEV_MC[_BN].HCB_ASK_PIDGain(2);
    usleep(10*1000);
    _DEV_MC[_BN].HCB_ASK_PIDGain(3);
    usleep(10*1000);
    _DEV_MC[_BN].HCB_ASK_ValveDeadzoneNCenterPos();
    usleep(10*1000);
    _DEV_MC[_BN].HCB_ASK_VelCompensationGain();
    usleep(10*1000);
    _DEV_MC[_BN].HCB_ASK_ComplianceGain();
    usleep(10*1000);
    _DEV_MC[_BN].HCB_ASK_ValveFeedforward();
    usleep(10*1000);
    _DEV_MC[_BN].HCB_ASK_BulkModulus();
    usleep(10*1000);
    _DEV_MC[_BN].HCB_ASK_ChamberVolume();
    usleep(10*1000);
    _DEV_MC[_BN].HCB_ASK_PistonArea();
    usleep(10*1000);
    _DEV_MC[_BN].HCB_ASK_SupplyAndReturnPressure();
    usleep(10*1000);
    _DEV_MC[_BN].HCB_ASK_JointEncLimit();
    usleep(10*1000);
    _DEV_MC[_BN].HCB_ASK_PistonStroke();
    usleep(10*1000);
    _DEV_MC[_BN].HCB_ASK_ValvePositionLimit();
    usleep(10*1000);
    _DEV_MC[_BN].HCB_ASK_EncoderPulsePerPosition();
    usleep(10*1000);
    _DEV_MC[_BN].HCB_ASK_SensorPulsePerForceTorque();
    usleep(10*1000);
    _DEV_MC[_BN].HCB_ASK_SensorPulsePerPressure();
    usleep(10*1000);
    _DEV_MC[_BN].HCB_ASK_ConstantFriction();
    usleep(10*1000);
    _DEV_MC[_BN].HCB_ASK_HomeposOffset();
    usleep(10*1000);
    _DEV_MC[_BN].HCB_ASK_HomeposValveOpening();
    usleep(10*1000);
    _DEV_MC[_BN].HCB_ASK_VALVE_GAIN_PLUS();
    usleep(10*1000);
    _DEV_MC[_BN].HCB_ASK_VALVE_GAIN_MINUS();
    usleep(10*1000);
}

void RBCMD_HCB_BoardTest(void) {
    int _BN = 0;
    _DEV_MC[_BN].HCB_InformationCheck();
}

void RBCMD_HCB_BoardTest_ErrorClear() {
    int _BN = 0;
    _DEV_MC[_BN].HCB_CMD_ErrorClear();
}

void RBCMD_HCB_TorqueForce_Nulling(void) {
    int _BNO = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
    if(_BNO == -1) {
        FILE_LOG(logSUCCESS) << "All Torque(Force) Sensor Nulling!";
        for(int i=0; i<_NO_OF_MC; i++)  {
            _DEV_MC[i].HCB_CMD_ControlMode(20); // (force sensor or pressure sensor) nulling mode
        }
    } else {
        FILE_LOG(logSUCCESS) << "Board (" << _BNO <<") Torque(Force) Sensor Nulling!";
        _DEV_MC[_BNO].HCB_CMD_ControlMode(20); // (force sensor or pressure sensor) nulling mode
    }
    SetWaitTime(5000); // WaitDisplay ON for 5sec
}

void RBCMD_HCB_Pres_Nulling(void) {
    int _BNO = sharedCMD->COMMAND[RBCORE_PODO_NO].USER_PARA_CHAR[0];
    if(_BNO == -1) {
        FILE_LOG(logSUCCESS) << "All Pressure Sensor Nulling!";
        for(int i=0; i<_NO_OF_MC; i++)  {
            _DEV_MC[i].HCB_CMD_ControlMode(24); // pressure sensor nulling mode
        }
    } else {
        FILE_LOG(logSUCCESS) << "Board (" << _BNO <<") Pressure Sensor Nulling!";
        _DEV_MC[_BNO].HCB_CMD_ControlMode(24); // pressure sensor nulling mode
    }
    SetWaitTime(5000); // WaitDisplay ON for 5sec
}

void RBCMD_HCB_AllReference_Reset(void) {
    FILE_LOG(logWARNING) << "All Position & Torque Reference Reset!";
    bool SupplyPressureChangeOnOff = sharedREF->PumpSupplyPressureChange;
    for(int i=0; i<_NO_OF_MC; i++)  {
        if(_DEV_MC[i].HCB_ResetReference_PosVel(SupplyPressureChangeOnOff))
        {
            FILE_LOG(logINFO) << "Borad(" << i << ") Reference Reset.";

        }
    }
}

//==============================================================================
int RBCore_Initialize(void){
    cout << endl;
    FILE_LOG(logERROR) << "==========Initializing===========";

    IS_WORKING = true;

    // Shared Memory initialize
    if(RBCore_SMInitialize() == false)
        return false;

    // Load RBCore configuration file
    if(RBCore_DBInitialize() == false)
        return false;

    // CAN & LAN Communcation initialize
    if(__IS_GAZEBO){
        if(RBCore_LANInitialize() == false){
            FILE_LOG(logERROR) << "KK";
            return false;
        }
    }else{
        RBCore_CANInitialize();
    }


    // Real-time thread initialize
    if(RBCore_ThreadInitialize() == false)
        return false;


    // Process Manager initialize
    if(RBCore_PMInitialize() == false)
        return false;


    // FOG [RS-232] initialize
    __IS_FOG = false;
    if(!__IS_GAZEBO && __IS_FOG){
        if(_DEV_FOG.RBOpenPort(B460800) != FOG_PORT_SUCCESS){
            IS_RS232_OK = false;
            return false;
        }else{
            IS_RS232_OK = true;
        }
    }

    FILE_LOG(logERROR) << "=================================";
    cout << endl;

    sharedSEN->PODO_AL_WORKING[RBCORE_PODO_NO] = true;
    IS_WORKING = true;
    return true;
}
//---------------
int RBCore_SMInitialize(){
    shm_unlink(RBCORE_SHM_NAME_REFERENCE);
    shm_unlink(RBCORE_SHM_NAME_SENSOR);
    shm_unlink(RBCORE_SHM_NAME_COMMAND);

    int shmFD;
    // Core Shared Memory Creation [Reference]==================================
    shmFD = shm_open(RBCORE_SHM_NAME_REFERENCE, O_CREAT|O_RDWR, 0666);
    if(shmFD == -1){
        FILE_LOG(logERROR) << "Fail to open core shared memory [Reference]";
        return -1;
    }else{
        if(ftruncate(shmFD, sizeof(RBCORE_SHM_REFERENCE)) == -1){
            FILE_LOG(logERROR) << "Fail to truncate core shared memory [Reference]";
            return -1;
        }else{
            sharedREF = (pRBCORE_SHM_REFERENCE)mmap(0, sizeof(RBCORE_SHM_REFERENCE), PROT_READ|PROT_WRITE, MAP_SHARED, shmFD, 0);
            if(sharedREF == (void*)-1){
                FILE_LOG(logERROR) << "Fail to mapping core shared memory [Reference]";
                return -1;
            }
        }
    }
    FILE_LOG(logSUCCESS) << "Core shared memory creation = OK [Reference]";
    // =========================================================================

    // Core Shared Memory Creation [Sensor]=====================================
    shmFD = shm_open(RBCORE_SHM_NAME_SENSOR, O_CREAT|O_RDWR, 0666);
    if(shmFD == -1){
        FILE_LOG(logERROR) << "Fail to open core shared memory [Sensor]";
        return -1;
    }else{
        if(ftruncate(shmFD, sizeof(RBCORE_SHM_SENSOR)) == -1){
            FILE_LOG(logERROR) << "Fail to truncate core shared memory [Sensor]";
            return -1;
        }else{
            sharedSEN = (pRBCORE_SHM_SENSOR)mmap(0, sizeof(RBCORE_SHM_SENSOR), PROT_READ|PROT_WRITE, MAP_SHARED, shmFD, 0);
            if(sharedSEN == (void*)-1){
                FILE_LOG(logERROR) << "Fail to mapping core shared memory [Sensor]";
                return -1;
            }
        }
    }
    FILE_LOG(logSUCCESS) << "Core shared memory creation = OK [Sensor]";
    // =========================================================================

    // Core Shared Memory Creation [Command]====================================
    shmFD = shm_open(RBCORE_SHM_NAME_COMMAND, O_CREAT|O_RDWR, 0666);
    if(shmFD == -1){
        FILE_LOG(logERROR) << "Fail to open core shared memory [Command]";
        return -1;
    }else{
        if(ftruncate(shmFD, sizeof(RBCORE_SHM_COMMAND)) == -1){
            FILE_LOG(logERROR) << "Fail to truncate core shared memory [Command]";
            return -1;
        }else{
            sharedCMD = (pRBCORE_SHM_COMMAND)mmap(0, sizeof(RBCORE_SHM_COMMAND), PROT_READ|PROT_WRITE, MAP_SHARED, shmFD, 0);
            if(sharedCMD == (void*)-1){
                FILE_LOG(logERROR) << "Fail to mapping core shared memory [Command]";
                return -1;
            }
        }
    }
    FILE_LOG(logSUCCESS) << "Core shared memory creation = OK [Command]";
    // =========================================================================

    return true;
}
//---------------
int RBCore_DBInitialize(){
    RBDataBase DB;
    DB.SetFilename("Core_Config.db");
    if(DB.OpenDB() == false){
        FILE_LOG(logERROR) << "Fail to load database file";
        return false;
    }

    _VERSION        = RBDataBase::_DB_GENERAL.VERSION;
    _NO_OF_AL       = RBDataBase::_DB_GENERAL.NO_OF_AL;
    _NO_OF_COMM_CH  = RBDataBase::_DB_GENERAL.NO_OF_COMM_CH;
    _NO_OF_MC       = RBDataBase::_DB_GENERAL.NO_OF_MC;
    _NO_OF_PC       = RBDataBase::_DB_GENERAL.NO_OF_PC;
    _NO_OF_FT       = RBDataBase::_DB_GENERAL.NO_OF_FT;
    _NO_OF_IMU      = RBDataBase::_DB_GENERAL.NO_OF_IMU;
    _NO_OF_SP       = RBDataBase::_DB_GENERAL.NO_OF_SP;
    _NO_OF_OF       = RBDataBase::_DB_GENERAL.NO_OF_OF;

    FILE_LOG(logSUCCESS) << "Core load database = OK";

    std::cout << "----------------------------------" << std::endl;
    std::cout << "     VERSION       : " << _VERSION << std::endl;
    std::cout << "     NO_OF_AL      : " << _NO_OF_AL << std::endl;
    std::cout << "     NO_OF_COMM_CH : " << _NO_OF_COMM_CH << std::endl;
    std::cout << "     NO_OF_MC      : " << _NO_OF_MC << std::endl;
    std::cout << "     NO_OF_PC      : " << _NO_OF_PC << std::endl;
    std::cout << "     NO_OF_FT      : " << _NO_OF_FT << std::endl;
    std::cout << "     NO_OF_IMU     : " << _NO_OF_IMU << std::endl;
    std::cout << "     NO_OF_SP      : " << _NO_OF_SP << std::endl;
    std::cout << "     NO_OF_OF      : " << _NO_OF_OF << std::endl;
    std::cout << "----------------------------------" << std::endl;

    for(int i=0; i<_NO_OF_MC; i++)   _DEV_MC[i].HCB_GetDBData(RBDataBase::_DB_MC[i]);
    for(int i=0; i<_NO_OF_PC; i++)   _DEV_PC[i].GetDBData(RBDataBase::_DB_PC[i]);
    for(int i=0; i<_NO_OF_FT; i++)   _DEV_FT[i].RBBoard_GetDBData(RBDataBase::_DB_FT[i]);
    for(int i=0; i<_NO_OF_IMU; i++)  _DEV_IMU[i].RBBoard_GetDBData(RBDataBase::_DB_IMU[i]);
    for(int i=0; i<_NO_OF_SP; i++)   _DEV_SP[i].RBBoard_GetDBData(RBDataBase::_DB_SP[i]);
    for(int i=0; i<_NO_OF_OF; i++)   _DEV_OF[i].RBBoard_GetDBData(RBDataBase::_DB_OF[i]);

    return true;
}
//---------------
int RBCore_CANInitialize(){
    canHandler = new RBCAN(_NO_OF_COMM_CH);

    if(canHandler->IsWorking() == false){
        IS_CAN_OK = false;
        return false;
    }else{
        _CANOUT_ENABLED = true;

        for(int i=0; i<_NO_OF_MC; i++)   _DEV_MC[i].HCB_AddCANMailBox_RCVDATA(); // Hydraulic Actuator Controller
        for(int i=0; i<_NO_OF_PC; i++)   _DEV_PC[i].AddCANMailBox_RCVDATA(); // Hydraulic Pump Controller
        for(int i=0; i<_NO_OF_FT; i++)   _DEV_FT[i].RBFT_AddCANMailBox();
        for(int i=0; i<_NO_OF_IMU; i++)  _DEV_IMU[i].RBIMU_AddCANMailBox();
        for(int i=0; i<_NO_OF_SP; i++)   _DEV_SP[i].RBSP_AddCANMailBox();
        for(int i=0; i<_NO_OF_OF; i++)   _DEV_OF[i].RBOF_AddCANMailBox();
        IS_CAN_OK = true;
        return true;
    }
}
//---------------
int RBCore_LANInitialize(){
    lanHandler = new RBRawLAN();
    if(lanHandler->RBLanOpen(8888) == false){
        FILE_LOG(logSUCCESS) << "qq";
        return false;
    }
    FILE_LOG(logSUCCESS) << "lan success";
    return true;
}
//---------------
int RBCore_ThreadInitialize(){
    if(__IS_GAZEBO){
        int theadID = pthread_create(&nrtTaskCon, NULL, &RBCore_NRTThreadCon, NULL);
        if(theadID < 0){
            FILE_LOG(logERROR) << "Fail to create core non real-time thread";
            return false;
        }
        FILE_LOG(logSUCCESS) << "Core non real-time thread start = OK";
    }else{
        if(rt_task_create(&rtTaskCon, "RBCORE_TASK", 0, 99, 0) == 0){
            cpu_set_t aCPU;
            CPU_ZERO(&aCPU);
            CPU_SET(0, &aCPU);
            if(rt_task_set_affinity(&rtTaskCon, &aCPU) != 0){
                FILE_LOG(logWARNING) << "Core real-time thread set affinity CPU failed..";
            }
            if(rt_task_start(&rtTaskCon, &RBCore_RTThreadCon, NULL) == 0){
                FILE_LOG(logSUCCESS) << "Core real-time thread start = OK";
            }else{
                FILE_LOG(logERROR) << "Core real-time thread start = FAIL";
                return false;
            }
        }else{
            FILE_LOG(logERROR) << "Fail to create core real-time thread";
            return false;
        }
    }

    return true;
}
//---------------
int RBCore_PMInitialize(){
    pmHandler = new RBProcessManager();

    int ret = pmHandler->OpenAL(1);
    if(ret == -99){
        IS_CHILD = true;
        IS_WORKING = false;
        return false;
    }
    return true;
}
//---------------
int RBCore_Termination(){
    if(IS_CHILD)
        return true;

    rt_task_delete(&rtTaskCon);
    shm_unlink(RBCORE_SHM_NAME_REFERENCE);
    shm_unlink(RBCORE_SHM_NAME_SENSOR);
    shm_unlink(RBCORE_SHM_NAME_COMMAND);
    FILE_LOG(logERROR) << "RBCore will be terminated..";
    //    exit(0);
    return true;
}
//==============================================================================

void THREAD_UpdateHCBInfo()
{
    for(int i=0; i<_NO_OF_MC; i++){
        if(!DO_CANCHECK) {
            //            _DEV_MC[i].HCB_SEE_Information();
        }
        for(int j=0; j<MAX_JOINT; j++){
            sharedSEN->ENCODER[i][j].HCB_Info = _DEV_MC[i].Joints[j].HCB_Info;
        }
    }
}

double dRAP_oo = 0.0;
double dRAP_o = 0.0;
double dRAR_oo = 0.0;
double dRAR_o = 0.0;
double dLAP_oo = 0.0;
double dLAP_o = 0.0;
double dLAR_oo = 0.0;
double dLAR_o = 0.0;

void THREAD_ReadEncoder(){
    for(int i=0; i<_NO_OF_MC; i++){
        if(_DEV_MC[i].Joints[0].HCB_Info.REQUEST_ONOFF_PosVel) {
            if(_DEV_MC[i].HCB_Read_EncoderData())
            {
                for(int j=0; j<MAX_JOINT; j++)
                {
                    sharedSEN->ENCODER[i][j].CurrentActPos = _DEV_MC[i].Joints[j].HCB_Data.CurrentPosition;
                    sharedSEN->ENCODER[i][j].CurrentActVel = _DEV_MC[i].Joints[j].HCB_Data.CurrentVelocity;
                    sharedSEN->ENCODER[i][j].CurrentActForce = _DEV_MC[i].Joints[j].HCB_Data.CurrentForce;

                    if(_DEV_MC[i].ACTUATOR_TYPE == "LIN")
                    {
                        // ACTUATOR_TYPE : Linear Actuator
                        if(_DEV_MC[i].BOARD_NAME == "HCB_RHP"||_DEV_MC[i].BOARD_NAME == "HCB_LHP") {
                            double x = sharedSEN->ENCODER[i][j].CurrentActPos*0.001;
                            double dx = sharedSEN->ENCODER[i][j].CurrentActVel*0.001;
                            double F = sharedSEN->ENCODER[i][j].CurrentActForce;
                            double theta, dtheta, T;
                            Actuator2Joint_HipPitch(x, dx, F, theta, dtheta, T);
                            sharedSEN->ENCODER[i][j].CurrentAngle  = theta*R2D;
                            sharedSEN->ENCODER[i][j].CurrentAngVel = dtheta*R2D;
                            sharedSEN->ENCODER[i][j].CurrentTorque = T;
                        } else if(_DEV_MC[i].BOARD_NAME == "HCB_RKN"||_DEV_MC[i].BOARD_NAME == "HCB_LKN") {
                            double x = sharedSEN->ENCODER[i][j].CurrentActPos*0.001; // mm >> m
                            double dx = sharedSEN->ENCODER[i][j].CurrentActVel*0.001; // mm >> m
                            double F = sharedSEN->ENCODER[i][j].CurrentActForce;
                            double theta, dtheta, T;
                            Actuator2Joint_Knee(x, dx, F, theta, dtheta, T);
                            sharedSEN->ENCODER[i][j].CurrentAngle  = theta*R2D;
                            sharedSEN->ENCODER[i][j].CurrentAngVel = dtheta*R2D;
                            sharedSEN->ENCODER[i][j].CurrentTorque = T;
                        } else if(_DEV_MC[i].BOARD_NAME == "HCB_RA_1"||_DEV_MC[i].BOARD_NAME == "HCB_RA_2") {
                            double theta_init_guess = 40.0*D2R;//sharedSEN->ENCODER[LAP][0].CurrentAngle*D2R;
                            double phi_init_guess = 0.0*D2R;//sharedSEN->ENCODER[LAR][0].CurrentAngle*D2R;
                            double x1 = sharedSEN->ENCODER[RAP][0].CurrentActPos*0.001;
                            double x2 = sharedSEN->ENCODER[RAR][0].CurrentActPos*0.001;
                            double dx1 = sharedSEN->ENCODER[RAP][0].CurrentActVel*0.001;
                            double dx2 = sharedSEN->ENCODER[RAR][0].CurrentActVel*0.001;
                            double F1 = sharedSEN->ENCODER[RAP][0].CurrentActForce;
                            double F2 = sharedSEN->ENCODER[RAR][0].CurrentActForce;

                            double theta_new, phi_new;
                            double dtheta, dphi;
                            double T_pitch, T_roll;
                            Actuator2Joint_Ankle(theta_init_guess,phi_init_guess,
                                                 x1,x2,dx1,dx2,F1,F2,
                                                 theta_new,phi_new,dtheta,dphi,T_pitch, T_roll);

                            sharedSEN->ENCODER[RAP][0].CurrentAngle = theta_new*R2D;
                            sharedSEN->ENCODER[RAR][0].CurrentAngle = phi_new*R2D;
                            sharedSEN->ENCODER[RAP][0].CurrentAngVel = dtheta*R2D;
                            sharedSEN->ENCODER[RAR][0].CurrentAngVel = dphi*R2D;
                            sharedSEN->ENCODER[RAP][0].CurrentTorque = T_pitch;
                            sharedSEN->ENCODER[RAR][0].CurrentTorque = T_roll;
                        } else if(_DEV_MC[i].BOARD_NAME == "HCB_LA_1"||_DEV_MC[i].BOARD_NAME == "HCB_LA_2") {
                            double theta_init_guess = 40.0*D2R;//sharedSEN->ENCODER[LAP][0].CurrentAngle*D2R;
                            double phi_init_guess = 0.0*D2R;//sharedSEN->ENCODER[LAR][0].CurrentAngle*D2R;
                            double x1 = sharedSEN->ENCODER[LAP][0].CurrentActPos*0.001;
                            double x2 = sharedSEN->ENCODER[LAR][0].CurrentActPos*0.001;
                            double dx1 = sharedSEN->ENCODER[LAP][0].CurrentActVel*0.001;
                            double dx2 = sharedSEN->ENCODER[LAR][0].CurrentActVel*0.001;
                            double F1 = sharedSEN->ENCODER[LAP][0].CurrentActForce;
                            double F2 = sharedSEN->ENCODER[LAR][0].CurrentActForce;

                            double theta_new, phi_new;
                            double dtheta, dphi;
                            double T_pitch, T_roll;
                            Actuator2Joint_Ankle(theta_init_guess,phi_init_guess,
                                                 x1,x2,dx1,dx2,F1,F2,
                                                 theta_new,phi_new,dtheta,dphi,T_pitch, T_roll);

                            sharedSEN->ENCODER[LAP][0].CurrentAngle = theta_new*R2D;
                            sharedSEN->ENCODER[LAR][0].CurrentAngle = phi_new*R2D;
                            sharedSEN->ENCODER[LAP][0].CurrentAngVel = dtheta*R2D;
                            sharedSEN->ENCODER[LAR][0].CurrentAngVel = dphi*R2D;
                            sharedSEN->ENCODER[LAP][0].CurrentTorque = T_pitch;
                            sharedSEN->ENCODER[LAR][0].CurrentTorque = T_roll;
                        } else {
                            FILE_LOG(logERROR) << "Not Cylinder TYPE!!";
                        }
                    } else if (_DEV_MC[i].ACTUATOR_TYPE == "ROT") {
                        // ACTUATOR_TYPE : Rotary Actuator
                        sharedSEN->ENCODER[i][j].CurrentAngle = sharedSEN->ENCODER[i][j].CurrentActPos;
                        sharedSEN->ENCODER[i][j].CurrentAngVel = sharedSEN->ENCODER[i][j].CurrentActVel;
                        sharedSEN->ENCODER[i][j].CurrentTorque = sharedSEN->ENCODER[i][j].CurrentActForce;
                    } else {
                        FILE_LOG(logERROR) << "Not Defined ACTUATOR TYPE!!";
                    }
                }
                sharedSEN->ENCODER[i][0].NO_RESPONSE_CNT = 0;
            }
            else
            {
                sharedSEN->ENCODER[i][0].NO_RESPONSE_CNT++;
            }
        }
    }
}

void THREAD_ReadValvePos(){

    for(int i=0; i<_NO_OF_MC; i++){

        if(_DEV_MC[i].Joints[0].HCB_Info.REQUEST_ONOFF_ValvePos){
            if(_DEV_MC[i].HCB_Read_ValvePosData())
            {
                for(int j=0; j<MAX_JOINT; j++){
                    sharedSEN->ENCODER[i][j].CurrentValvePos = _DEV_MC[i].Joints[j].HCB_Data.CurrentValvePos;
                }
            }
        }

    }
}

void THREAD_ReadOtherInfo(){

    for(int i=0; i<_NO_OF_MC; i++){
        if(_DEV_MC[i].Joints[0].HCB_Info.REQUEST_ONOFF_OtherInfo){
            if(_DEV_MC[i].HCB_Read_OtherInfoData())
            {

                for(int j=0; j<MAX_JOINT; j++){
                    sharedSEN->ENCODER[i][j].CurrentPressureA = _DEV_MC[i].Joints[j].HCB_Data.CurrentPressureA;
                    sharedSEN->ENCODER[i][j].CurrentPressureB = _DEV_MC[i].Joints[j].HCB_Data.CurrentPressureB;
                    sharedSEN->ENCODER[i][j].CurrentTempData1 = _DEV_MC[i].Joints[j].HCB_Data.CurrentTempData1;
                    sharedSEN->ENCODER[i][j].CurrentTempData2 = _DEV_MC[i].Joints[j].HCB_Data.CurrentTempData2;
                }
            }
        }

    }
}

void THREAD_ReadFT(){
    for(int i=0; i<_NO_OF_FT; i++){
        _DEV_FT[i].RBFT_ReadData();
        sharedSEN->FT[i].Fx     = -_DEV_FT[i].FX;
        sharedSEN->FT[i].Fy     = _DEV_FT[i].FY;
        sharedSEN->FT[i].Fz     = _DEV_FT[i].FZ;
        sharedSEN->FT[i].Mx     = _DEV_FT[i].MX;
        sharedSEN->FT[i].My     = -_DEV_FT[i].MY;
        sharedSEN->FT[i].Mz     = -_DEV_FT[i].MZ;
    }
}


float local_wx_o,local_ax_o;
float local_wy_o,local_ay_o;
float local_wz_o,local_az_o;
float local_wx_oo,local_ax_oo;
float local_wy_oo,local_ay_oo;
float local_wz_oo,local_az_oo;

float local_wx_f,local_ax_f;
float local_wy_f,local_ay_f;
float local_wz_f,local_az_f;
float local_wx_fo,local_ax_fo;
float local_wy_fo,local_ay_fo;
float local_wz_fo,local_az_fo;
float local_wx_foo,local_ax_foo;
float local_wy_foo,local_ay_foo;
float local_wz_foo,local_az_foo;

void THREAD_ReadNewIMU(void){

    if(_DEV_IMU[0].ConnectionStatus == true) {

        float local_wx,local_ax;
        float local_wy,local_ay;
        float local_wz,local_az;

        /// Local Angular Velocity Receive
        RBCAN_MB mb2;
        mb2.channel = _DEV_IMU[0].CAN_CHANNEL;
        mb2.id = _DEV_IMU[0].ID_RCV_DATA_LOCAL_X;
        canHandler->RBCAN_ReadData(&mb2);
        if(mb2.status != RBCAN_NODATA){
            memcpy(&local_wx,mb2.data,4);   // data[0]~data[3]
            memcpy(&local_ax,mb2.data+4,4); // data[4]~data[7]
            mb2.status = RBCAN_NODATA;
        } else {
            local_wx = local_wx_o;
            local_ax = local_ax_o;
        }

        RBCAN_MB mb3;
        mb3.channel = _DEV_IMU[0].CAN_CHANNEL;
        mb3.id = _DEV_IMU[0].ID_RCV_DATA_LOCAL_Y;
        canHandler->RBCAN_ReadData(&mb3);
        if(mb3.status != RBCAN_NODATA){
            memcpy(&local_wy,mb3.data,4);   // data[0]~data[3]
            memcpy(&local_ay,mb3.data+4,4); // data[4]~data[7]
            mb3.status = RBCAN_NODATA;
        } else {
            local_wy = local_wy_o;
            local_ay = local_ay_o;
        }

        RBCAN_MB mb4;
        mb4.channel = _DEV_IMU[0].CAN_CHANNEL;
        mb4.id = _DEV_IMU[0].ID_RCV_DATA_LOCAL_Z;
        canHandler->RBCAN_ReadData(&mb4);
        if(mb4.status != RBCAN_NODATA){
            memcpy(&local_wz,mb4.data,4);   // data[0]~data[3]
            memcpy(&local_az,mb4.data+4,4); // data[4]~data[7]
            mb4.status = RBCAN_NODATA;
        } else {
            local_wz = local_wz_o;
            local_az = local_az_o;
        }

//        // Low pass filter (or notch filter) for pump vibration
//        if(sharedSEN->PUMP[0].CurrentVelocity < 600.0) {
//            local_wx_f = local_wx;
//            local_wy_f = local_wy;
//            local_wz_f = local_wz;
//            local_ax_f = local_ax;
//            local_ay_f = local_ay;
//            local_az_f = local_az;
//        } else {
//            double wdt_notch = 2.0*3.141592*sharedSEN->PUMP[0].CurrentVelocity/60.0*(double)RT_TIMER_PERIOD_MS/1000.0;
//            double Q_notch = 0.3;
//            double a1_notch = 1.0 + wdt_notch/Q_notch + wdt_notch*wdt_notch;
//            double a2_notch = -2.0 - wdt_notch/Q_notch;
//            double a3_notch = 1.0;
//            double b1_notch = 1.0 + wdt_notch*wdt_notch;
//            double b2_notch = -2.0;
//            double b3_notch = 1.0;

//            local_wx_f = (b1_notch*local_wx + b2_notch*local_wx_o + b3_notch*local_wx_oo - a2_notch*local_wx_fo - a3_notch*local_wx_foo)/a1_notch;
//            local_wy_f = (b1_notch*local_wy + b2_notch*local_wy_o + b3_notch*local_wy_oo - a2_notch*local_wy_fo - a3_notch*local_wy_foo)/a1_notch;
//            local_wz_f = (b1_notch*local_wz + b2_notch*local_wz_o + b3_notch*local_wz_oo - a2_notch*local_wz_fo - a3_notch*local_wz_foo)/a1_notch;
//            local_ax_f = (b1_notch*local_ax + b2_notch*local_ax_o + b3_notch*local_ax_oo - a2_notch*local_ax_fo - a3_notch*local_ax_foo)/a1_notch;
//            local_ay_f = (b1_notch*local_ay + b2_notch*local_ay_o + b3_notch*local_ay_oo - a2_notch*local_ay_fo - a3_notch*local_ay_foo)/a1_notch;
//            local_az_f = (b1_notch*local_az + b2_notch*local_az_o + b3_notch*local_az_oo - a2_notch*local_az_fo - a3_notch*local_az_foo)/a1_notch;
//        }

//        double w_cut = 2.0*PI*1.0;
//        double SYS_FREQ = RT_TIMER_FREQ;
//        double alpha1 = -(SYS_FREQ*SYS_FREQ)/(SYS_FREQ*SYS_FREQ+sqrt(2.0)*w_cut*SYS_FREQ+w_cut*w_cut);
//        double alpha2 = (2.0*SYS_FREQ*SYS_FREQ+sqrt(2.0)*w_cut*SYS_FREQ)/(SYS_FREQ*SYS_FREQ+sqrt(2.0)*w_cut*SYS_FREQ+w_cut*w_cut);
//        double alpha3 = (w_cut*w_cut)/(SYS_FREQ*SYS_FREQ+sqrt(2.0)*w_cut*SYS_FREQ+w_cut*w_cut);

//        local_wx_f = alpha1*local_wx_oo + alpha2*local_wx_o + alpha3*local_wx;
//        local_wy_f = alpha1*local_wy_oo + alpha2*local_wy_o + alpha3*local_wy;
//        local_wz_f = alpha1*local_wz_oo + alpha2*local_wz_o + alpha3*local_wz;
//        local_ax_f = alpha1*local_ax_oo + alpha2*local_ax_o + alpha3*local_ax;
//        local_ay_f = alpha1*local_ay_oo + alpha2*local_ay_o + alpha3*local_ay;
//        local_az_f = alpha1*local_az_oo + alpha2*local_az_o + alpha3*local_az;

        double fcut_wp = 12.0;
        double alpha_wp = 1.0/(1.0+2.0*PI*fcut_wp*(double)RT_TIMER_PERIOD_MS/1000.0);

        local_wx_f = alpha_wp*local_wx_f + (1.0-alpha_wp)*local_wx;
        local_wy_f = alpha_wp*local_wy_f + (1.0-alpha_wp)*local_wy;
        local_wz_f = alpha_wp*local_wz_f + (1.0-alpha_wp)*local_wz;
        local_ax_f = alpha_wp*local_ax_f + (1.0-alpha_wp)*local_ax;
        local_ay_f = alpha_wp*local_ay_f + (1.0-alpha_wp)*local_ay;
        local_az_f = alpha_wp*local_az_f + (1.0-alpha_wp)*local_az;

        local_wx_oo = local_wx_o;
        local_wy_oo = local_wy_o;
        local_wz_oo = local_wz_o;
        local_ax_oo = local_ax_o;
        local_ay_oo = local_ay_o;
        local_az_oo = local_az_o;
        local_wx_o = local_wx;
        local_wy_o = local_wy;
        local_wz_o = local_wz;
        local_ax_o = local_ax;
        local_ay_o = local_ay;
        local_az_o = local_az;

        local_wx_foo = local_wx_fo;
        local_wy_foo = local_wy_fo;
        local_wz_foo = local_wz_fo;
        local_ax_foo = local_ax_fo;
        local_ay_foo = local_ay_fo;
        local_az_foo = local_az_fo;
        local_wx_fo = local_wx_f;
        local_wy_fo = local_wy_f;
        local_wz_fo = local_wz_f;
        local_ax_fo = local_ax_f;
        local_ay_fo = local_ay_f;
        local_az_fo = local_az_f;


        // reversed IMU with sponge (210830)
        sharedSEN->IMU[0].Wx_B      = (double)local_wy_f*D2R; // Unit : deg/s > rad/s
        sharedSEN->IMU[0].Wy_B      = (double)local_wx_f*D2R; // Unit : deg/s > rad/s
        sharedSEN->IMU[0].Wz_B      = (double)-local_wz_f*D2R; // Unit : deg/s > rad/s
        sharedSEN->IMU[0].Ax_B      = (double)local_ay_f*9.81; // Unit : (normalized g) > m/s^2
        sharedSEN->IMU[0].Ay_B      = (double)local_ax_f*9.81; // Unit : (normalized g) > m/s^2
        sharedSEN->IMU[0].Az_B      = (double)-local_az_f*9.81; // Unit : (normalized g) > m/s^2

        // for LIGHT2
//        sharedSEN->IMU[0].Wx_B      = (double)local_wy_f*D2R; // Unit : deg/s > rad/s
//        sharedSEN->IMU[0].Wy_B      = (double)-local_wx_f*D2R; // Unit : deg/s > rad/s
//        sharedSEN->IMU[0].Wz_B      = (double)local_wz_f*D2R; // Unit : deg/s > rad/s
//        sharedSEN->IMU[0].Ax_B      = (double)local_ay_f*9.81; // Unit : (normalized g) > m/s^2
//        sharedSEN->IMU[0].Ay_B      = (double)-local_ax_f*9.81; // Unit : (normalized g) > m/s^2
//        sharedSEN->IMU[0].Az_B      = (double)local_az_f*9.81; // Unit : (normalized g) > m/s^2

        // Invarient Extended Kalman Filter
        Vector3_InEKF Uw(sharedSEN->IMU[0].Wx_B, sharedSEN->IMU[0].Wy_B, sharedSEN->IMU[0].Wz_B);
        Vector3_InEKF Ua(sharedSEN->IMU[0].Ax_B, sharedSEN->IMU[0].Ay_B, sharedSEN->IMU[0].Az_B);
        InEKF_IMU.Update(Uw,Ua);
        Matrix3_InEKF R = InEKF_IMU.R;

        // IMU Offset Compensation
        double roll_off = _DEV_IMU[0].ROLL_OFFSET;
        double pitch_off = _DEV_IMU[0].PITCH_OFFSET;
        Matrix3_InEKF R_Offset_Roll(1.0,            0.0,            0.0,
                                    0.0,  cos(roll_off), -sin(roll_off),
                                    0.0,  sin(roll_off),  cos(roll_off));
        Matrix3_InEKF R_Offset_Pitch( cos(pitch_off), 0.0, sin(pitch_off),
                                                 0.0, 1.0,            0.0,
                                     -sin(pitch_off), 0.0, cos(pitch_off));
        R = R_Offset_Roll.Transpose()*R_Offset_Pitch.Transpose()*R;

        // Projection to Yaw rotation angle zero
        bool Flag_YawZero = true;
        if(Flag_YawZero) {
            if(fabs(sqrt(R.M[1][0]*R.M[1][0]+R.M[0][0]*R.M[0][0]))<1e-6) {
                if(R.M[0][0] < 0.0) {
                    sharedSEN->IMU[0].Roll = 0.0;
                    sharedSEN->IMU[0].Pitch = 90.0*D2R;
                    sharedSEN->IMU[0].Yaw = 0.0;
                    Matrix3_InEKF R_yawzero(0.0, 0.0, 1.0,
                                            0.0, 1.0, 0.0,
                                            1.0, 0.0, 0.0);

                    Vector3_InEKF W_G = R_yawzero*Uw;
                    sharedSEN->IMU[0].Wx_G = W_G.v[0];
                    sharedSEN->IMU[0].Wy_G = W_G.v[1];
                    sharedSEN->IMU[0].Wz_G = W_G.v[2];

                    Vector3_InEKF A_G = R_yawzero*Ua;
                    sharedSEN->IMU[0].Ax_G = A_G.v[0];
                    sharedSEN->IMU[0].Ay_G = A_G.v[1];
                    sharedSEN->IMU[0].Az_G = A_G.v[2];
                } else {
                    sharedSEN->IMU[0].Roll = 0.0;
                    sharedSEN->IMU[0].Pitch = -90.0*D2R;
                    sharedSEN->IMU[0].Yaw = 0.0;
                    Matrix3_InEKF R_yawzero(0.0, 0.0, -1.0,
                                            0.0, 1.0, 0.0,
                                            -1.0, 0.0, 0.0);

                    Vector3_InEKF W_G = R_yawzero*Uw;
                    sharedSEN->IMU[0].Wx_G = W_G.v[0];
                    sharedSEN->IMU[0].Wy_G = W_G.v[1];
                    sharedSEN->IMU[0].Wz_G = W_G.v[2];

                    Vector3_InEKF A_G = R_yawzero*Ua;
                    sharedSEN->IMU[0].Ax_G = A_G.v[0];
                    sharedSEN->IMU[0].Ay_G = A_G.v[1];
                    sharedSEN->IMU[0].Az_G = A_G.v[2];
                }
            } else {
                double psi = atan2(R.M[1][0], R.M[0][0]);   // z-axis (Yaw)
    //            double theta = -asin(R.M[2][0]);            // y'-axis (Pitch)
    //            double phi = atan2(R.M[2][1], R.M[2][2]);   // x''-axis (Roll)
                Matrix3_InEKF R_yaw(cos(psi), -sin(psi), 0.0,
                                    sin(psi), cos(psi) , 0.0,
                                    0.0     , 0.0      , 1.0);
                Matrix3_InEKF R_yawzero = R_yaw.Transpose()*R;

                sharedSEN->IMU[0].Q[0] = sqrt(1.0+R_yawzero.M[0][0]+R_yawzero.M[1][1]+R_yawzero.M[2][2])/2.0;
                sharedSEN->IMU[0].Q[1] = (R_yawzero.M[2][1]-R_yawzero.M[1][2])/4.0/sharedSEN->IMU[0].Q[0];
                sharedSEN->IMU[0].Q[2] = (R_yawzero.M[0][2]-R_yawzero.M[2][0])/4.0/sharedSEN->IMU[0].Q[0];
                sharedSEN->IMU[0].Q[3] = (R_yawzero.M[1][0]-R_yawzero.M[0][1])/4.0/sharedSEN->IMU[0].Q[0];

                sharedSEN->IMU[0].Roll = atan2(-R_yawzero.M[1][2],R_yawzero.M[1][1]);
                sharedSEN->IMU[0].Pitch = atan2(-R_yawzero.M[2][0],R_yawzero.M[0][0]);
                sharedSEN->IMU[0].Yaw = 0.0;

                Vector3_InEKF W_G = R_yawzero*Uw;
                sharedSEN->IMU[0].Wx_G = W_G.v[0];
                sharedSEN->IMU[0].Wy_G = W_G.v[1];
                sharedSEN->IMU[0].Wz_G = W_G.v[2];

                Vector3_InEKF A_G = R_yawzero*Ua;
                sharedSEN->IMU[0].Ax_G = A_G.v[0];
                sharedSEN->IMU[0].Ay_G = A_G.v[1];
                sharedSEN->IMU[0].Az_G = A_G.v[2];
            }
        } else {
            sharedSEN->IMU[0].Q[0] = sqrt(1.0+R.M[0][0]+R.M[1][1]+R.M[2][2])/2.0;
            sharedSEN->IMU[0].Q[1] = (R.M[2][1]-R.M[1][2])/4.0/sharedSEN->IMU[0].Q[0];
            sharedSEN->IMU[0].Q[2] = (R.M[0][2]-R.M[2][0])/4.0/sharedSEN->IMU[0].Q[0];
            sharedSEN->IMU[0].Q[3] = (R.M[1][0]-R.M[0][1])/4.0/sharedSEN->IMU[0].Q[0];

            double q0 = sharedSEN->IMU[0].Q[0];
            double q1 = sharedSEN->IMU[0].Q[1];
            double q2 = sharedSEN->IMU[0].Q[2];
            double q3 = sharedSEN->IMU[0].Q[3];
            sharedSEN->IMU[0].Yaw   = atan2(2.0*(q1*q2 + q0*q3), 1.0-2.0*(q2*q2 + q3*q3)); // z-axis
            sharedSEN->IMU[0].Pitch = -asin(2.0*(q1*q3 - q0*q2));                          // y'-axis
            sharedSEN->IMU[0].Roll  = atan2(2.0*(q2*q3 + q0*q1), 1.0-2.0*(q1*q1 + q2*q2)); // x''-axis

            Vector3_InEKF W_G = R*Uw;
            sharedSEN->IMU[0].Wx_G = W_G.v[0];
            sharedSEN->IMU[0].Wy_G = W_G.v[1];
            sharedSEN->IMU[0].Wz_G = W_G.v[2];

            Vector3_InEKF A_G = R*Ua;
            sharedSEN->IMU[0].Ax_G = A_G.v[0];
            sharedSEN->IMU[0].Ay_G = A_G.v[1];
            sharedSEN->IMU[0].Az_G = A_G.v[2];
        }
    }
}

void THREAD_ReadPumpData(){
    if(_DEV_PC[0].REQUEST_ONOFF_DATA == true) {
        if(_DEV_PC[0].Read_PumpData())
        {
            double alpha_velo = 1.0/(1.0+RT_TIMER_FREQ/(2.0*3.141592*30.0));
            double alpha_pres = 1.0/(1.0+RT_TIMER_FREQ/(2.0*3.141592*5.0));
            double alpha_temp = 1.0/(1.0+RT_TIMER_FREQ/(2.0*3.141592*10.0));
            sharedSEN->PUMP[0].CurrentVelocity      = (1.0-alpha_velo)*sharedSEN->PUMP[0].CurrentVelocity + alpha_velo*_DEV_PC[0].CurrentVelocity;
            sharedSEN->PUMP[0].CurrentPressure      = (1.0-alpha_pres)*sharedSEN->PUMP[0].CurrentPressure + alpha_pres*_DEV_PC[0].CurrentPressure;
            sharedSEN->PUMP[0].CurrentTemperature   = (1.0-alpha_temp)*sharedSEN->PUMP[0].CurrentTemperature + alpha_temp*_DEV_PC[0].CurrentTemperature;

            sharedSEN->PUMP[0].CurrentHydraPower    = (sharedSEN->PUMP[0].CurrentPressure*100.0)*(sharedSEN->PUMP[0].CurrentVelocity/3000.0*14.0/60.0);
            double temp_HydraPower = 0.0;
            for(int i=0;i<MAX_MC;i++) {
                temp_HydraPower += sharedSEN->ENCODER[i][0].CurrentActVel*D2R*sharedSEN->ENCODER[i][0].CurrentActForce;
            }
            sharedSEN->PUMP[0].CurrentMechaPower    = temp_HydraPower;
        }
    }
}
