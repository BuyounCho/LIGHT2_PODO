#include "PumpController_BasicFunctions.h"
#include "PumpControl_ControlFunctions.h"

extern pRBCORE_SHM_COMMAND     sharedCMD;
extern pRBCORE_SHM_REFERENCE   sharedREF;
extern pRBCORE_SHM_SENSOR      sharedSEN;
//extern pUSER_SHM                userData;

PumpControl_QP  QP_PsCtrl(SolverIsQPSwift);
extern double wp_ref_max;
extern double wp_ref_min;
extern double Ps_des[MAX_PREVIEW+1];

void PumpPressure_Dynamics(double Ps_now, double wp_now, double wdp, double Q_act, double dT, double& Ps_next, double& wp_next)
{
    if(wdp > wp_ref_max/60.0*2.0*PI) {
        wdp = wp_ref_max/60.0*2.0*PI;
    } else if (wdp < wp_ref_min/60.0*2.0*PI) {
        wdp = wp_ref_min/60.0*2.0*PI;
    }

    double Ps = Ps_now;
    double wp = wp_now;

    double k = n_gas/(pow(P_pre,(1/n_gas))*V_pre);

    double q = OutputFlowPerRev / (2.0 * PI);  // Pump Displacement [L/rad]
    double n_act = 13.0;                    // Number of actuators

    int integration_grid = 10;
    for (int i = 0; i < integration_grid; i++) {
        double Q_pump = q * wp * 60.0;
        double Q_leak = n_act * K_leak * Ps;

        double dPs = (k * pow(Ps, (1.0 / n_gas)) * Ps * (Q_pump - Q_act - Q_leak) / 60.0); // [bar/s]

        // Integration
        Ps = Ps + dPs * dT / (double)integration_grid;
        double T_delay = 0.05;
        wp = (1.0/(1.0+2.0*PI*dT/(double)integration_grid/T_delay))*wp + (1.0-1.0/(1.0+2.0*PI*dT/(double)integration_grid/T_delay))*wdp;
    }

    Ps_next = Ps;
    wp_next = wp;

}

void PumpPressureController_LinearMPC_NoDelay_Formulation(double Ps_now, double Ulast,
                                                          VectorNd Ps_des_window, VectorNd Qact_des_window)
{
    int N = sharedREF->N_PrevPump;
    double dT = sharedREF->dT_PrevPump;

    int n_X = N;
    int n_U = 2*N;
    int n_var = n_X + n_U;
    VectorNd X = VectorNd::Zero(n_X);
    VectorNd U = VectorNd::Zero(n_U);
    VectorNd Z = VectorNd::Zero(n_var); // [X;U]
    QP_PsCtrl.n_X = n_X;
    QP_PsCtrl.n_U = n_U;
    QP_PsCtrl.n_var = n_var;

    MatrixNd A_temp;
    VectorNd B_temp;

    QP_PsCtrl.N_window = N;
    QP_PsCtrl.dT_window = dT;

    //////// Cost Functions /////////////////////////////////////////////////////////////////////////
    /// 1) Supply Pressure Condition (Ps > Ps,ref)  [Soft Constraint]
    /// 2) Power minimization
    ///    W_loss = (mu*wp+fc)*wp + KL*Ps*Ps
    /////////////////////////////////////////////////////////////////////////////////////////////////

    // Cost Function Weight
    double W_Tracking = 100.0;
    double W_SpeedLossMin = 1.0;
    double W_LeakLossMin = 1.0;
//    double W_SpeedChangeMinInit = 1e-4;
    double W_SpeedChangeMin = 0.05; // 0.03r
    double W_PressureChangeMin = 0.15*(Ps_now/Ps_min)*(Ps_now/Ps_min)*(Ps_now/Ps_min); // 0.001

    int n_cost = 0;
    int idx_cost = 0;
    int temp_size_cost = 0;
    MatrixNd A_cost;
    VectorNd B_cost;
    double W_temp = 0.0;

    // 1. Supply Pressure Tracking + Penalty
    W_temp = W_Tracking;
    temp_size_cost = N;
    Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
    for(int i=0;i<N;i++) {
        A_temp(i,i) = 1.0;
        A_temp(i,n_X+2*i+1) = -1.0;
        B_temp(i) = Ps_des_window(i+1);
    }
    Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
    idx_cost += temp_size_cost;

    // 2. Power Loss Minimization [Watt]
    W_temp = W_SpeedLossMin;
    temp_size_cost = N;
    Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
    for(int i=0;i<N;i++) {
        double mu = 0.0051166;
        double fc = 0.56983;
        A_temp(i,n_X+2*i) = sqrt(mu);
        B_temp(i) = -fc/(2.0*sqrt(mu));
    }
    Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
    idx_cost += temp_size_cost;

    // 3. Leakage Minimization [Watt]
    W_temp = W_LeakLossMin;
    temp_size_cost = N;
    Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
    for(int i=0;i<N;i++) {
        double KL = 0.005;//0.014642;
        A_temp(i,i) = sqrt(KL);
    }
    Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
    idx_cost += temp_size_cost;

//    // 4-1. Pump Speed Change Minimization, Initial Step (Acceleration)
//    W_temp = W_SpeedChangeMinInit;
//    temp_size_cost = 1;
//    Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
//    A_temp(0,n_X) = 1.0/SYS_DT;
//    B_temp(0) = wdp0/SYS_DT;
//    Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
//    idx_cost += temp_size_cost;

    // 4-2. Pump Speed Change Minimization (Acceleration)
    W_temp = W_SpeedChangeMin;
    temp_size_cost = N-1;
    Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
    for(int i=0;i<N-1;i++) {
        A_temp(i,n_X+2*i) = -1.0/dT;
        A_temp(i,n_X+2*(i+1)) = 1.0/dT;
        B_temp(i) = 0.0;
    }
    Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
    idx_cost += temp_size_cost;

    // 5-1. Pressure Change Minimization
    W_temp = W_PressureChangeMin;
    temp_size_cost = N-1;
    Resize_TempMatrix(temp_size_cost,n_var,A_temp,B_temp);
    for(int i=0;i<N-1;i++) {
        A_temp(i,i) = -1.0/dT;
        A_temp(i,i+1) = 1.0/dT;
        B_temp(i) = 0.0;
    }
    Matrix4QP_Addition(A_cost,B_cost,W_temp*A_temp,W_temp*B_temp);
    idx_cost += temp_size_cost;


    n_cost = idx_cost;

    //////// Equality Constraints (Ax=B) /////////////////////////////////////////////////////////////////

    /// 1) Model Predictive Control :
    /// X_MPC = [x_k+1; x_k+2; ...; x_k+N]
    /// A_MPC = [A^1; A^2; ...; A^N]
    /// B_MPC = [B 0 .. 0; A*B B 0 .. 0; A*A*B A*B B 0 .. 0; ... ; A^(N-1)*B .. A*B B]
    /// U_MPC = [x_vel_ref(k+1);x_acc_ref(k+1);...;x_vel_ref(k+N);x_acc_ref(k+N)]
    ///    >> X_MPC = A_MPC*_xnow + B_MPC*U_MPC;
    ///    >> X_MPC - B_MPC*U_MPC = -A_MPC*_xnow;
    ///    >> [I -B_MPC]*[X_MPC;U_MPC] = -A_MPC*_xnow;
    /////////////////////////////////////////////////////////////////////////////////////////////////

    cout << Qact_des_window.transpose() << endl;

    int n_equ =0;
    int idx_equ = 0;
    int temp_size_equ = 0;
    MatrixNd A_equ;
    VectorNd B_equ;

    double k = n_gas/(pow(P_pre,(1/n_gas))*V_pre);

    double q_p = OutputFlowPerRev / (2.0 * PI);  // Pump Displacement [L/rad]
    double n_act = 13.0; // Number of actuators
    double Ps0 = Ps_now;
    double wdp0 = 0.0*Ulast;

    MatrixNd Ac = MatrixNd::Zero(1,1);
    MatrixNd Bc = MatrixNd::Zero(1,2);
    MatrixNd Ec = MatrixNd::Zero(1,1);
    Ac(0,0) = (k*pow(Ps0,(1/n_gas))*(((n_gas+1)/n_gas)*(q_p*wdp0*60.0-Qact_des_window(0))-((2*n_gas+1)/n_gas)*(n_act*K_leak*Ps0)))/60.0;
    Bc(0,0) = (k*pow(Ps0,(n_gas+1)/n_gas)*q_p*60.0)/60.0;
    Ec(0,0) = -k*pow(Ps0,(n_gas+1)/n_gas)*((n_gas+1)/n_gas*(q_p*wdp0*60.0-n_act*K_leak*Ps0)-Qact_des_window(0)/n_gas)/60.0;

    MatrixNd Ad = MatrixNd::Identity(1,1)+dT*Ac;
    MatrixNd Bd = dT*Bc;
    MatrixNd Ed = dT*Ec;

    MatrixNd A_MPC = MatrixNd::Zero(N,1);
    MatrixNd B_MPC = MatrixNd::Zero(N,2*N);
    MatrixNd E_MPC = MatrixNd::Zero(N,1);

    A_MPC.block(0,0,1,1) = Ad;
    B_MPC.block(0,0,1,2) = Bd;
    E_MPC.block(0,0,1,1) = Ed;

    for(int i=1;i<N;i++) {
        Ac = MatrixNd::Zero(1,1);
        Bc = MatrixNd::Zero(1,2);
        Ec = MatrixNd::Zero(1,1);
        Ac(0,0) = (k*pow(Ps0,1/n_gas)*(((n_gas+1)/n_gas)*(q_p*wdp0*60.0-Qact_des_window(i))-((2*n_gas+1)/n_gas)*(n_act*K_leak*Ps0)))/60.0;
        Bc(0,0) = (k*pow(Ps0,(n_gas+1)/n_gas)*q_p*60.0)/60.0;
        Ec(0,0) = -k*pow(Ps0,(n_gas+1)/n_gas)*((n_gas+1)/n_gas*(q_p*wdp0*60.0-n_act*K_leak*Ps0)-Qact_des_window(i)/n_gas)/60.0;

        Ad = MatrixNd::Identity(1,1)+dT*Ac;
        Bd = dT*Bc;
        Ed = dT*Ec;

        A_MPC.block(i,0,1,1) = Ad*A_MPC.block((i-1),0,1,1);

        for(int j=0;j<i;j++) {
            B_MPC.block(i,2*j,1,2) = Ad*B_MPC.block((i-1),2*j,1,2);
        }
        B_MPC.block(i,2*i,1,2) = Bd;

        E_MPC.block(i,0,1,1) = Ad*E_MPC.block((i-1),0,1,1)+Ed;
    }

    temp_size_equ = N;
    Resize_TempMatrix(temp_size_equ,n_var,A_temp,B_temp);
    A_temp.block(0,0,N,N) = MatrixNd::Identity(N,N);
    A_temp.block(0,N,N,2*N) = -B_MPC;
    B_temp.block(0,0,N,1) = A_MPC*Ps_now + E_MPC;
    Matrix4QP_Addition(A_equ,B_equ,A_temp,B_temp);
    idx_equ += temp_size_equ;

    n_equ = idx_equ;

    //////// Inequality Constraints (Aq<=B) /////////////////////////////////////////////////////////
    /// 1) Pump Speed Constraints
    /// 2) Pump Acceleration Constraints
    /// 2) Delta Limit
    /////////////////////////////////////////////////////////////////////////////////////////////////

    int n_inequ = 0;
    int idx_inequ = 0;
    int temp_size_inequ = 0;
    MatrixNd A_inequ;
    VectorNd B_inequ;

    // 1) Pump Speed Constraints
    double wp_lb = wp_ref_min/60.0*2.0*PI; // [rad/s]
    double wp_ub = wp_ref_max/60.0*2.0*PI; // [rad/s]
    temp_size_inequ = 2*N;
    Resize_TempMatrix(temp_size_inequ,n_var,A_temp,B_temp);
    for(int i=0;i<N;i++) {
        A_temp(i,n_X+2*i) = -1.0;
        B_temp(i) = -wp_lb;
        A_temp(i+N,n_X+2*i) = 1.0;
        B_temp(i+N) = wp_ub;
    }
    Matrix4QP_Addition(A_inequ,B_inequ,A_temp,B_temp);
    idx_inequ += temp_size_inequ;

//    // 2) Pump Acceleration Constraints
//    double dwp_lb = -1000.0; // [rad/s^2]
//    double dwp_ub = 1000.0; // [rad/s^2]
//    temp_size_inequ = 2*(N-1);
//    Resize_TempMatrix(temp_size_inequ,n_var,A_temp,B_temp);
//    for(int i=0;i<N-1;i++) {
//        A_temp(i,n_X+2*i) = -1.0;
//        A_temp(i,n_X+2*(i-1)) = 1.0;
//        B_temp(i) = -(dwp_lb*dT);
//    }
//    for(int i=0;i<N-1;i++) {
//        A_temp(i+N-1,n_X+2*i) = 1.0;
//        A_temp(i+N-1,n_X+2*(i-1)) = -1.0;
//        B_temp(i+N-1) = (dwp_ub*dT);
//    }
////    A_temp(2*N-2,n_X) = 1.0;
////    B_temp(2*N-2) = (dwp_ub*SYS_DT+wdp0);
////    A_temp(2*N-1,n_X) = -1.0;
////    B_temp(2*N-1) = -(dwp_lb*SYS_DT+wdp0);
//    Matrix4QP_Addition(A_inequ,B_inequ,A_temp,B_temp);
//    idx_inequ += temp_size_inequ;

    // 3) Delta Limit
    double delta_lb = 0.0;
    temp_size_inequ = N;
    Resize_TempMatrix(temp_size_inequ,n_var,A_temp,B_temp);
    for(int i=0;i<N;i++) {
        A_temp(i,n_X+2*i+1) = -1.0;
        B_temp(i) = -delta_lb;
    }
    Matrix4QP_Addition(A_inequ,B_inequ,A_temp,B_temp);
    idx_inequ += temp_size_inequ;

    n_inequ = idx_inequ;

    QP_PsCtrl.setNums(n_var,n_cost,n_equ,n_inequ);
    QP_PsCtrl.make_COST(A_cost, B_cost);
    QP_PsCtrl.make_EQ(A_equ,B_equ);
    QP_PsCtrl.make_IEQ(A_inequ,B_inequ);

}


void PumpPressureController_LinearMPC_Solve(VectorNd& Uout)
{

    int N = QP_PsCtrl.N_window;
    VectorNd Ps_window = VectorNd::Zero(N);
    VectorNd wdp_window = VectorNd::Zero(N);
//    VectorNd Ps_window = VectorNd::Zero(N);
//    VectorNd wp_window = VectorNd::Zero(N);
//    VectorNd dwp_window = VectorNd::Zero(N);

    switch(QP_PsCtrl.WhichSolver()) {
    case SolverIsQuadProg:
    {
        // 1. Solve the problem.
        VectorNd QP_sol = QP_PsCtrl.solve_QP();
        for(int i=0;i<N;i++) {
            Ps_window(i) = QP_sol(i);
            wdp_window(i) = QP_sol(2*i+N);
//            Ps_window(i) = QP_sol(2*i);
//            wp_window(i) = QP_sol(2*i+1);
//            dwp_window(i) = QP_sol(2*i+2*N);

        }
        break;
    }
    case SolverIsQPSwift:
    {
        qp_int n_var = QP_PsCtrl.NUMCOLS;
        qp_int n_equ = QP_PsCtrl.NUMEQ;
        qp_int n_inequ = QP_PsCtrl.NUMINEQ;

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
        MatrixNd P = QP_PsCtrl.P_cost;
        VectorNd q = QP_PsCtrl.Q_cost;
        P_nnz = QP_PsCtrl.GetNumberOfNonZero_QPswift(P);
        P_x = new qp_real[P_nnz];
        P_i = new qp_int[P_nnz];
        P_p = new qp_int[n_var+1];
        QP_PsCtrl.ConvertMatrixA_Full2CCS_QPswift(P,P_x,P_i,P_p);
        q_x = new qp_real[n_var];
        QP_PsCtrl.ConvertVector2Array_QPswift(q,n_var,q_x);

        // 1-1. Convert equality constraint to sparse form .
        if(n_equ > 0) {
            MatrixNd A = QP_PsCtrl.A_equ;
            MatrixNd b = QP_PsCtrl.B_equ;
            A_nnz = QP_PsCtrl.GetNumberOfNonZero_QPswift(A);
            A_x = new qp_real[A_nnz];
            A_i = new qp_int[A_nnz];
            A_p = new qp_int[n_var+1];
            QP_PsCtrl.ConvertMatrixA_Full2CCS_QPswift(A,A_x,A_i,A_p);
            b_x = new qp_real[n_equ];
            QP_PsCtrl.ConvertVector2Array_QPswift(b,n_equ,b_x);
        }

        // 1-2. Convert inequality constraint to sparse form .
        if(n_inequ > 0) {
            MatrixNd G = QP_PsCtrl.A_inequ;
            MatrixNd h = QP_PsCtrl.B_inequ;
            G_nnz = QP_PsCtrl.GetNumberOfNonZero_QPswift(G);
            G_x = new qp_real[G_nnz];
            G_i = new qp_int[G_nnz];
            G_p = new qp_int[n_var+1];
            QP_PsCtrl.ConvertMatrixA_Full2CCS_QPswift(G,G_x,G_i,G_p);
            h_x = new qp_real[n_inequ];
            QP_PsCtrl.ConvertVector2Array_QPswift(h,n_inequ,h_x);
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
//            PRINT("Solve Time     : %f ms\n", (myQP->stats->tsolve + myQP->stats->tsetup)*1000.0);
//            PRINT("KKT_Solve Time : %f ms\n", myQP->stats->kkt_time*1000.0);
//            PRINT("LDL Time       : %f ms\n", myQP->stats->ldl_numeric*1000.0);
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
        QP_PsCtrl.ConvertArray2Vector_QPswift(myQP->x, n_var, QP_sol);
        for(int i=0;i<N;i++) {
            Ps_window(i) = QP_sol(i);
            wdp_window(i) = QP_sol(2*i+N);
//            Ps_window(i) = QP_sol(2*i);
//            wp_window(i) = QP_sol(2*i+1);
//            dwp_window(i) = QP_sol(2*i+2*N);

        }

//        cout << "Ps : \n" << Ps_window << endl;
//        cout << "wp : \n" << wp_window << endl;

        // 3. destruction allocated memory
        QP_CLEANUP(myQP);
        delete []P_x;  delete []P_i;  delete []P_p;  delete []q_x;
        delete []A_x;  delete []A_i;  delete []A_p;  delete []b_x;
        delete []G_x;  delete []G_i;  delete []G_p;  delete []h_x;
        break;
    }
    default:
        FILE_LOG(logERROR) << "[Pattern Generation X Error] QP Solver is not set!";
    }

    Uout = wdp_window;
//    Uout = dwp_window;
}

void Resize_TempMatrix(int m, int n, MatrixNd& A, VectorNd& B)
{
    A.resize(m,n);
    A = MatrixNd::Zero(m,n);
    B.resize(m,1);
    B = MatrixNd::Zero(m,1);
}

void Matrix4QP_Addition(MatrixNd& A_base, VectorNd& B_base, MatrixNd A_new, VectorNd B_new)
{
    int m_now = A_base.rows();
    int n_new = A_new.cols();
    int m_new = A_new.rows();

    A_base.conservativeResize(m_now+m_new,n_new);
    B_base.conservativeResize(m_now+m_new,1);
    A_base.block(m_now,0,m_new,n_new) = A_new;
    B_base.block(m_now,0,m_new,1) = B_new;
}

