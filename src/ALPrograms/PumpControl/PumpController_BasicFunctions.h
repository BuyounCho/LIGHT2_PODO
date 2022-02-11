#ifndef PUMPCONTROLLER_BASICFUNCTION_H
#define PUMPCONTROLLER_BASICFUNCTION_H

#include <iostream>
#include <cmath>
#include <chrono>

#include "RBLog.h"
#include "RBSharedMemory.h"
#include "JointInformation.h"
#include "rbdl/rbdl.h"

using namespace std;
using namespace std::chrono;

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

#define    SYS_DT  0.004
#define    SYS_FREQ  250.0

#define     MPatoBAR        10.0   // MPa >> bar
#define     MLPStoLPM       0.06   // mm^2*m/s >> L/min
#define     BARtoPA         100000.0
#define     PI              3.141592
#define     D2R             (PI/180.0)
#define     R2D             (180.0/PI)

#define     PUMPSPEED_MAX   2200.0 // @ 100V voltage input
#define     PUMPSPEED_MIN   150.0

#define     K_leak          (0.20/100.0)   // leakage constant [L/min/bar]
#define     n_gas           1.2         // polytropic coefficient
#define     V_pre           0.16         // Pre-charged gas volume [L]
#define     P_pre           30.0         // Pre-charged gas pressure [bar]
#define     P_ModeChange    P_pre
#define     Ps_min          (P_pre+5.0)    // Allowable maximum pressure at accumulator
#define     Ps_max          210.0         // Allowable maximum pressure at accumulator
#define     Ps_margin       5.0

#define     OutputFlowPerRev    (14.0/3000)   // 14.0/3000 [L/rev]

void PrintHere(int n);
void linear_trajectory_pressure(double tf, double tn, double Pnow, double Pfin, double &Pnext, double &dPnext);


#endif // PUMPCONTROLLER_BASICFUNCTION_H
