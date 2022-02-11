#include "../ALPrograms/LIGHTWalking/LIGHT_info.h"
#include "../../share/Headers/RBSharedMemory.h"
#include "HydraulicActuatorDataConverting.h"

extern pRBCORE_SHM_COMMAND     sharedCMD;
extern pRBCORE_SHM_REFERENCE   sharedREF;
extern pRBCORE_SHM_SENSOR      sharedSEN;

// Converting Joint positon data

INFO_LIGHT _LIGHT_Info;

///////////////////////////////////////////////
///   [ Cylinder Info >> Joint Info ]
/// ///////////////////////////////////////////

// 1. Hip Joint
void Actuator2Joint_HipPitch(double S, double dS, double F,
                                double &theta, double &dtheta, double &T)
{
    // input : relative cylinder length (0~stroke) [m]
    // output : relative hip angle (0~fullangle) [rad]

    double l0 = _LIGHT_Info.HipLink.l0;
    double l1 = _LIGHT_Info.HipLink.l1;
    double alpha = _LIGHT_Info.HipLink.alpha;

    S = S + _LIGHT_Info.HipLink.x_off;
    if (S >= l0+l1-0.001) { S = l0+l1-0.001; }
    else if (S <= l0-l1+0.001) { S = l0-l1+0.001; }

    theta = acos((l0*l0+l1*l1-S*S)/(2.0*l0*l1));

    double temp_dS = 1.0;
    double J = S*temp_dS/(l0*l1*sin(theta));

    dtheta = J*dS;
    T = F/J;

    theta = alpha - theta;
    dtheta = -dtheta;
    T = -T;
}

// 2. Knee Joint
void Actuator2Joint_Knee(double S, double dS, double F,
                         double &theta, double &dtheta, double &T)
{
    // input : relative cylinder length (0~stroke) [m]
    // output : relative knee angle (0~fullangle) [rad]

    double l0 = _LIGHT_Info.KneeLink.l0;
    double beta = _LIGHT_Info.KneeLink.beta;

    double d1 = _LIGHT_Info.KneeLink.d1;
    double d2 = _LIGHT_Info.KneeLink.d2;
    double d3 = _LIGHT_Info.KneeLink.d3;
    double d4 = _LIGHT_Info.KneeLink.d4;
    double delta = _LIGHT_Info.KneeLink.delta;

    S = S + _LIGHT_Info.KneeLink.x_off;
    if (S >= l0+d2-0.001) { S = l0+d2-0.001; }
    else if (S <= l0-d2+0.001)  { S = l0-d2+0.001; }

    double phi = PI-(acos((l0*l0+d2*d2-S*S)/(2.0*l0*d2))-beta);
    double Ls = sqrt(d1*d1+d2*d2-2.0*d1*d2*cos(phi));

    double theta_d4 = acos((Ls*Ls+d3*d3-d4*d4)/(2.0*Ls*d3));
    double theta_d2 = acos((Ls*Ls+d1*d1-d2*d2)/(2.0*Ls*d1));
    theta = theta_d2 + theta_d4;

    double temp_dS = 1.0;
    double dphi = -S*temp_dS/(l0*d2*sin(PI+beta-phi));
    double dLs = d1*d2/Ls*sin(phi)*dphi;
    double dtheta_d4 = (d3*cos(theta_d4)-Ls)/(Ls*d3*sin(theta_d4))*dLs;
    double dtheta_d2 = (d1*cos(theta_d2)-Ls)/(Ls*d1*sin(theta_d2))*dLs;
    double J = dtheta_d2 + dtheta_d4;

    dtheta = J*dS;
    T = F/J;

    theta = delta - theta;
    dtheta = -dtheta;
    T = -T;
}


// 3. Ankle Joint
void Actuator2Joint_Ankle(double theta_old, double phi_old,
                          double x_R, double x_L, double dx_R, double dx_L, double F_R, double F_L,
                          double& theta_new, double& phi_new, double& dtheta, double& dphi, double& T_p, double& T_r)
{
    // [Input]
    // theta : Initial pitch angle [rad]
    // phi : Initial roll angle [rad]
    // x_R : current right sensor position [m]
    // x_L : current left sensor position [m]

    // [Output]
    // new_theta : updated pitch angle [rad]
    // new_phi : updated roll angle [rad]

    if (x_R <= -0.005) { x_R = -0.005; }
    else if (x_R >= 0.889)  { x_R = 0.0889; }
    if (x_L <= -0.005) { x_L = -0.005; }
    else if (x_L >= 0.889)  { x_L = 0.889; }

    // Point O : Ankle Joint Center
    // Point P : Joint Link(Universal Joint) Center
    // Point Q1 : Right Link(connected to cylinder) Center
    // Point Q2 : Left Link(connected to cylinder) Center
    // Point C1 : Right Cylinder base
    // Point C2 : Left Cylinder base
    Vector3d OP = _LIGHT_Info.AnkleLink.OP;
    Vector3d PQ1 = _LIGHT_Info.AnkleLink.PQ_R; // right : 1
    Vector3d PQ2 = _LIGHT_Info.AnkleLink.PQ_L; // left : 2
    Vector3d OC1 = _LIGHT_Info.AnkleLink.OC_R;
    Vector3d OC2 = _LIGHT_Info.AnkleLink.OC_L;
    double alpha = _LIGHT_Info.AnkleLink.alpha; // inclined cylinder angle
    double l = _LIGHT_Info.AnkleLink.l;
    double x1_off = _LIGHT_Info.AnkleLink.x_off_R;
    double x2_off = _LIGHT_Info.AnkleLink.x_off_L;
    double theta_off = _LIGHT_Info.AnkleLink.theta_off;
    Vector3d p = Vector3d(sin(alpha), 0, -cos(alpha));

    double theta_update = theta_old - theta_off;
    double phi_update = phi_old;

    int CNT_ANKLE_KINEMATICS = 0;
    double w2 = 1000.0;
    while((fabs(w2)>1e-6)&&(CNT_ANKLE_KINEMATICS<50)) {
        CNT_ANKLE_KINEMATICS++;

        Matrix3d R_p = Matrix3d(cos(theta_update),      0.0, sin(theta_update),
                                              0.0,      1.0,               0.0,
                               -sin(theta_update),      0.0, cos(theta_update));
        Matrix3d R_r = Matrix3d(       1.0,             0.0,                0.0,
                                       0.0, cos(phi_update),   -sin(phi_update),
                                       0.0, sin(phi_update),    cos(phi_update));
        Vector3d OQ1 = R_p*(R_r*PQ1+OP);
        Vector3d OQ2 = R_p*(R_r*PQ2+OP);

        Vector3d C1Q1 = OQ1-OC1;
        double C1Q1_Proj = p.dot(C1Q1);
        double C1Q1_off = C1Q1.dot(C1Q1)-C1Q1_Proj*C1Q1_Proj;
        double x1 = C1Q1_Proj-sqrt(l*l-C1Q1_off);

        Vector3d C2Q2 = OQ2-OC2;
        double C2Q2_Proj = p.dot(C2Q2);
        double C2Q2_off = C2Q2.dot(C2Q2)-C2Q2_Proj*C2Q2_Proj;
        double x2 = C2Q2_Proj-sqrt(l*l-C2Q2_off);

        Matrix3d dR_p = Matrix3d(-sin(theta_update),     0.0, cos(theta_update),
                                                0.0,     0.0,               0.0,
                                 -cos(theta_update),     0.0,-sin(theta_update));
        Matrix3d dR_r = Matrix3d(        0.0,              0.0,              0.0,
                                         0.0, -sin(phi_update), -cos(phi_update),
                                         0.0,  cos(phi_update), -sin(phi_update));

        Vector3d dC1Q1_1 = dR_p*(R_r*PQ1+OP);
        Vector3d dC1Q1_2 = R_p*(dR_r*PQ1);
        MatrixNd J_dC1Q1(3,2);
        J_dC1Q1.block(0,0,3,1) = dC1Q1_1;
        J_dC1Q1.block(0,1,3,1) = dC1Q1_2;
        MatrixNd J_x1 = 1.0/(x1-p.dot(C1Q1))*(x1*p.transpose()-C1Q1.transpose())*J_dC1Q1;

        Vector3d dC2Q2_1 = dR_p*(R_r*PQ2+OP);
        Vector3d dC2Q2_2 = R_p*(dR_r*PQ2);
        MatrixNd J_dC2Q2(3,2);
        J_dC2Q2.block(0,0,3,1) = dC2Q2_1;
        J_dC2Q2.block(0,1,3,1) = dC2Q2_2;
        MatrixNd J_x2 = 1.0/(x2-p.dot(C2Q2))*(x2*p.transpose()-C2Q2.transpose())*J_dC2Q2;

        // Jacobian for joint angular velocity >> cylinder velocity (v = J*thetadot)
        MatrixNd J(2,2);
        J.block(0,0,1,2) = J_x1; // for pitch joint velocity
        J.block(1,0,1,2) = J_x2; // for roll joint velocity

        VectorNd dx(2);
        dx(0) = x_R-(x1-x1_off); // Cylinder length error
        dx(1) = x_L-(x2-x2_off);
        VectorNd w = J.inverse()*dx;
        double alpha = 0.7; // update gain
        theta_update += alpha*w(0);
        phi_update += alpha*w(1);

        w2 = sqrt(w.dot(w)); // angle error magnitude
                       // if w2 is small enough, iteration ends.

        if(fabs(w2)<=1e-6) {
            VectorNd dx(2);
            dx(0) = dx_R;
            dx(1) = dx_L;
            VectorNd w = J.inverse()*dx;
            dtheta = w(0);
            dphi = w(1);

            VectorNd F(2);
            F(0) = F_R;
            F(1) = F_L;
            VectorNd Tau = J.transpose()*F;
            T_p = Tau(0);
            T_r = Tau(1);
        }
    }
    if(CNT_ANKLE_KINEMATICS >= 50) {
        std::cout << "Ankle Kinematics is not converged, w2 : " << w2 << std::endl;
        dtheta = 0.0;
        dphi = 0.0;
        T_p = 0.0;
        T_r = 0.0;
    }

    theta_new = theta_update + theta_off;
    phi_new = phi_update;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

void  Rotary2Joint_QuadKnee(double S, double dS, double F,
                            double &theta, double &dtheta, double &T)
{
    S = S + 37.03*D2R;
    if(S < 36.03*D2R) S = 36.03*D2R;
    else if (S > 138.03*D2R) S = 138.03*D2R;

    double d1 = 0.350;
    double d2 = 0.060;
    double d3 = 0.350;
    double d4 = 0.058;
    double delta = (20.0)*D2R; // knee angle offset
    double gamma = (28.0)*D2R; // knee ~ d4 link angle

    double Ls = sqrt(d1*d1+d2*d2-2.0*d1*d2*cos(S)); // cos(PI-(theta+delta+gamma)) = -cos(theta+delta+gamma)

    double phi2 = acos((Ls*Ls+d4*d4-d3*d3)/(2.0*Ls*d4));
    double phi1 = acos((Ls*Ls+d1*d1-d2*d2)/(2.0*Ls*d1));

    theta = PI - (phi2 - phi1 + gamma) - delta;

    // Jacobian for actuator > joint
    double dLs = d1*d4*sin(S)/Ls;
    double dphi2 = (d4*cos(phi2)-Ls)*dLs/(d4*Ls*sin(phi2));
    double dphi1 = (d1*cos(phi1)-Ls)*dLs/(d1*Ls*sin(phi1));
    double J = dphi2 - dphi1;

    dtheta = J*dS;
    T = F/J;

    theta = -theta;
    dtheta = -dtheta;
    T = -T;
}
