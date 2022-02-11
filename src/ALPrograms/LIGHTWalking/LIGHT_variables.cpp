#include <math.h>
#include "../../../share/Headers/JointInformation.h"
#include "LIGHT_var_and_func.h"
#include "rbdl/rbdl.h"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

/********************************************
 * Constant Values
 ********************************************/
#define    PI   3.1415927f
#define    R2D  57.2957914f
#define    D2R  0.01745329f
#define    g_const    9.81


/********************************************
 * LIPM dynamics parameter
 ********************************************/
double      Pelvis_BaseHeight = 0.70;
double      HeightDiff_CoM2Pel = -0.10;
double      wn_LIPM = sqrt(g_const/(Pelvis_BaseHeight-HeightDiff_CoM2Pel));


/********************************************
 * ETC
 ********************************************/
Vector3d zv = Vector3d::Zero();
Matrix3d I3 = Matrix3d::Identity();


