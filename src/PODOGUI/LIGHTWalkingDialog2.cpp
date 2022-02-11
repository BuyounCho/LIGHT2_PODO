#include "LIGHTWalkingDialog2.h"
#include "ui_LIGHTWalkingDialog2.h"
#include "../../src/ALPrograms/LIGHTWalking/LIGHT_commands.h"
#include "JoyStick/joystickclass.h"
#include "JoyStick/joystickvariable.h"

#include "BasicFiles/PODOALDialog.h"
#include <iostream>
using namespace std;

int DeadZoneFunc(int& target, int _DZ)
{
    if(target > _DZ) {
        target = target - _DZ;
    } else if(target < -_DZ) {
        target = target + _DZ;
    } else {
        target = 0;
    }
}

LIGHTWalkingDialog2::LIGHTWalkingDialog2(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::LIGHTWalkingDialog2)
{
    ui->setupUi(this);

    JoyStick = new RBJoystick();
    JoyStick->ConnectJoy();
    Flag_JoyStick_OnOff = false;

    palette = new QPalette();
    palette->setColor(QPalette::Base,Qt::red);
    ui->LAMP_JOYSTICK->setPalette(*palette);
    palette->setColor(QPalette::Window,Qt::black);
    ui->LAMP_JOYSTICK->setPalette(*palette);

    ALNum = PODOALDialog::GetALNumFromFileName("LIGHTWalking");
    displayTimer = new QTimer(this);
    connect(displayTimer, SIGNAL(timeout()), this, SLOT(UpdateIMUData()));
    connect(displayTimer, SIGNAL(timeout()), this, SLOT(UpdateJoyStickData()));
    displayTimer->start(100);//100ms

}

LIGHTWalkingDialog2::~LIGHTWalkingDialog2()
{
    delete palette;
    delete ui;
}

void LIGHTWalkingDialog2::UpdateIMUData(){
    ui->LE_IMU_ROLL->setText(QString().sprintf("%.2f", PODO_DATA.CoreSEN.IMU[0].Roll*R2D));
    ui->LE_IMU_PITCH->setText(QString().sprintf("%.2f", PODO_DATA.CoreSEN.IMU[0].Pitch*R2D));
    ui->LE_IMU_YAW->setText(QString().sprintf("%.2f", PODO_DATA.CoreSEN.IMU[0].Yaw*R2D));

    if(PODO_DATA.CoreREF.Simulation_DataEnable == 0) { // Data Receiving from Robot
        ui->LE_ROBOT_OR_SIM->setText(QString().sprintf("%s", "Robot"));
        QPalette *palette = new QPalette();
        palette->setColor(QPalette::Text,Qt::red);
        ui->LE_ROBOT_OR_SIM->setPalette(*palette);
    } else if (PODO_DATA.CoreREF.Simulation_DataEnable == 1) {  // Data Receiving from Simulation
        ui->LE_ROBOT_OR_SIM->setText(QString().sprintf("%s", "Simulation"));
        QPalette *palette = new QPalette();
        palette->setColor(QPalette::Text,Qt::blue);
        ui->LE_ROBOT_OR_SIM->setPalette(*palette);
    }
}

void LIGHTWalkingDialog2::on_BTN_JOYSTICK_ENABLE_clicked()
{
    Flag_JoyStick_OnOff = true;
}

void LIGHTWalkingDialog2::on_BTN_JOYSTICK_DISABLE_clicked()
{
    Flag_JoyStick_OnOff = false;
}

void LIGHTWalkingDialog2::UpdateJoyStickData()
{
    // JoyStick Input
    if(Flag_JoyStick_OnOff==true) {
        GetJoystickData();
        ui->LE_JOYSTICK_COMMAND_X->setText(QString().sprintf("%d", JOY_LJOG_UD));
        ui->LE_JOYSTICK_COMMAND_Y->setText(QString().sprintf("%d", JOY_LJOG_RL));
        ui->LE_JOYSTICK_COMMAND_YAW->setText(QString().sprintf("%d", JOY_RJOG_RL));
//        ui->LE_JOYSTICK_BUTTON_A->setText(QString().sprintf("%d", JOY_START));
//        ui->LE_JOYSTICK_BUTTON_X->setText(QString().sprintf("%d", JOY_BACK));
    } else {
        InitJoystickData();
        palette->setColor(QPalette::Base,Qt::red);
        ui->LAMP_JOYSTICK->setPalette(*palette);
    }
}

void LIGHTWalkingDialog2::GetJoystickData(void)
{
    if(JoyStick->isConnected() == true){
        palette->setColor(QPalette::Base,Qt::green);
        ui->LAMP_JOYSTICK->setPalette(*palette);
    } else if(JoyStick->isConnected() == false){
        palette->setColor(QPalette::Base,Qt::yellow);
        ui->LAMP_JOYSTICK->setPalette(*palette);
        printf("JoyStick connection failure...!!!\n");
        return;
    }

    // Button Data (Checkable with jstest-gtk)
    JOY_BACK        = JoyStick->JoyButton[8];
    JOY_START       = JoyStick->JoyButton[9];

    JOY_LJOG_RL     = -JoyStick->JoyAxis[0];
    JOY_LJOG_UD     = -JoyStick->JoyAxis[1];
    JOY_RJOG_RL     = -JoyStick->JoyAxis[2];
    JOY_RJOG_UD     = -JoyStick->JoyAxis[3];

    // Give Deadzone
    double th_deadzone = 0.0;
    DeadZoneFunc(JOY_LJOG_RL, th_deadzone);
    DeadZoneFunc(JOY_LJOG_UD, th_deadzone);
    DeadZoneFunc(JOY_RJOG_RL, th_deadzone);
    DeadZoneFunc(JOY_RJOG_UD, th_deadzone);

    double Command_X = ((double)JOY_LJOG_UD)/(32768.0-th_deadzone);
    double Command_Y = ((double)JOY_LJOG_RL)/(32768.0-th_deadzone);
    double Command_Yaw = ((double)(JOY_RJOG_RL))/(32768.0);

    USER_COMMAND cmd;
    if(JOY_BACK == true) {
        pLAN->G2MData->JoyStick_OnOff = 2;
    } else {
        pLAN->G2MData->JoyStick_OnOff = 1;
    }
    pLAN->G2MData->JoyStick_Command_X = Command_X;
    pLAN->G2MData->JoyStick_Command_Y = Command_Y;
    pLAN->G2MData->JoyStick_Command_Yaw = Command_Yaw;
    pLAN->G2MData->JoyStick_Command_StartWalk = JOY_START;
    pLAN->G2MData->JoyStick_Command_StopWalk = JOY_BACK;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);

    if(JOY_BACK == true) {
        Flag_JoyStick_OnOff = false;
        pLAN->G2MData->JoyStick_OnOff = 0;
        pLAN->G2MData->JoyStick_Command_X = 0.0;
        pLAN->G2MData->JoyStick_Command_Y = 0.0;
        pLAN->G2MData->JoyStick_Command_Yaw = 0.0;
        pLAN->G2MData->JoyStick_Command_StartWalk = 0;
        pLAN->G2MData->JoyStick_Command_StopWalk = 0;
        JOY_BACK = 0;
    }
}

void LIGHTWalkingDialog2::InitJoystickData(void)
{
    JOY_LJOG_RL=JOY_LJOG_UD=0;
    JOY_RJOG_RL=JOY_RJOG_UD=0;
    JOY_BACK=JOY_START=0;
}


void LIGHTWalkingDialog2::on_RBTN_OPERATIONMODE_CHOREONOID_toggled(bool checked)
{
    USER_COMMAND cmd;
    if(ui->RBTN_OPERATIONMODE_ROBOT->isChecked()) cmd.COMMAND_DATA.USER_PARA_INT[0] = 0;
    else if(ui->RBTN_OPERATIONMODE_CHOREONOID->isChecked()) cmd.COMMAND_DATA.USER_PARA_INT[0] = 1;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_SIMLATION_MODE_ONOFF;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog2::on_BTN_IMU_ENABLE_clicked()
{
    // new imu enable
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_SENSOR_IMU_ONOFF;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog2::on_BTN_IMU_DISABLE_clicked()
{
    // new imu enable
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 1;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_SENSOR_IMU_ONOFF;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog2::on_BTN_IMU_NULL_clicked()
{
    // new imu reset angle
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_SENSOR_IMU_NULL;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog2::on_BTN_IMU_FIND_OFFSET_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_SENSOR_IMU_FIND_OFFSET;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog2::on_BTN_ROBOT_CONTROLMETHOD_CHANGE_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = ui->CBX_ROBOT_CONTROLMETHOD_RHR->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ui->CBX_ROBOT_CONTROLMETHOD_RHY->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = ui->CBX_ROBOT_CONTROLMETHOD_RHP->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[3] = ui->CBX_ROBOT_CONTROLMETHOD_RKN->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[4] = ui->CBX_ROBOT_CONTROLMETHOD_RANK->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[5] = ui->CBX_ROBOT_CONTROLMETHOD_RANK->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[6] = ui->CBX_ROBOT_CONTROLMETHOD_LHR->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[7] = ui->CBX_ROBOT_CONTROLMETHOD_LHY->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[8] = ui->CBX_ROBOT_CONTROLMETHOD_LHP->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[9] = ui->CBX_ROBOT_CONTROLMETHOD_LKN->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[10] = ui->CBX_ROBOT_CONTROLMETHOD_LANK->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[11] = ui->CBX_ROBOT_CONTROLMETHOD_LANK->isChecked();

    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_MOTION_CHANGE_POSorTOR;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog2::on_BTN_ROBOT_CONTROLMETHOD_CHANGE_TO_POS_clicked()
{
    ui->CBX_ROBOT_CONTROLMETHOD_RHR->setChecked(0);
//    ui->CBX_ROBOT_CONTROLMETHOD_RHY->setChecked(0);
    ui->CBX_ROBOT_CONTROLMETHOD_RHP->setChecked(0);
    ui->CBX_ROBOT_CONTROLMETHOD_RKN->setChecked(0);
    ui->CBX_ROBOT_CONTROLMETHOD_RANK->setChecked(0);
    ui->CBX_ROBOT_CONTROLMETHOD_LHR->setChecked(0);
//    ui->CBX_ROBOT_CONTROLMETHOD_LHY->setChecked(0);
    ui->CBX_ROBOT_CONTROLMETHOD_LHP->setChecked(0);
    ui->CBX_ROBOT_CONTROLMETHOD_LKN->setChecked(0);
    ui->CBX_ROBOT_CONTROLMETHOD_LANK->setChecked(0);

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = ui->CBX_ROBOT_CONTROLMETHOD_RHR->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ui->CBX_ROBOT_CONTROLMETHOD_RHY->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = ui->CBX_ROBOT_CONTROLMETHOD_RHP->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[3] = ui->CBX_ROBOT_CONTROLMETHOD_RKN->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[4] = ui->CBX_ROBOT_CONTROLMETHOD_RANK->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[5] = ui->CBX_ROBOT_CONTROLMETHOD_RANK->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[6] = ui->CBX_ROBOT_CONTROLMETHOD_LHR->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[7] = ui->CBX_ROBOT_CONTROLMETHOD_LHY->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[8] = ui->CBX_ROBOT_CONTROLMETHOD_LHP->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[9] = ui->CBX_ROBOT_CONTROLMETHOD_LKN->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[10] = ui->CBX_ROBOT_CONTROLMETHOD_LANK->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[11] = ui->CBX_ROBOT_CONTROLMETHOD_LANK->isChecked();

    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_MOTION_CHANGE_POSorTOR;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog2::on_BTN_ROBOT_CONTROLMETHOD_CHANGE_TO_TOR_clicked()
{
    ui->CBX_ROBOT_CONTROLMETHOD_RHR->setChecked(1);
//    ui->CBX_ROBOT_CONTROLMETHOD_RHY->setChecked(1);
    ui->CBX_ROBOT_CONTROLMETHOD_RHP->setChecked(1);
    ui->CBX_ROBOT_CONTROLMETHOD_RKN->setChecked(1);
    ui->CBX_ROBOT_CONTROLMETHOD_RANK->setChecked(1);
    ui->CBX_ROBOT_CONTROLMETHOD_LHR->setChecked(1);
//    ui->CBX_ROBOT_CONTROLMETHOD_LHY->setChecked(1);
    ui->CBX_ROBOT_CONTROLMETHOD_LHP->setChecked(1);
    ui->CBX_ROBOT_CONTROLMETHOD_LKN->setChecked(1);
    ui->CBX_ROBOT_CONTROLMETHOD_LANK->setChecked(1);

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = ui->CBX_ROBOT_CONTROLMETHOD_RHR->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = ui->CBX_ROBOT_CONTROLMETHOD_RHY->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[2] = ui->CBX_ROBOT_CONTROLMETHOD_RHP->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[3] = ui->CBX_ROBOT_CONTROLMETHOD_RKN->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[4] = ui->CBX_ROBOT_CONTROLMETHOD_RANK->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[5] = ui->CBX_ROBOT_CONTROLMETHOD_RANK->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[6] = ui->CBX_ROBOT_CONTROLMETHOD_LHR->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[7] = ui->CBX_ROBOT_CONTROLMETHOD_LHY->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[8] = ui->CBX_ROBOT_CONTROLMETHOD_LHP->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[9] = ui->CBX_ROBOT_CONTROLMETHOD_LKN->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[10] = ui->CBX_ROBOT_CONTROLMETHOD_LANK->isChecked();
    cmd.COMMAND_DATA.USER_PARA_CHAR[11] = ui->CBX_ROBOT_CONTROLMETHOD_LANK->isChecked();

    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_MOTION_CHANGE_POSorTOR;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog2::on_BTN_COM_SMOOTH_MOTION_GO_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIGHT_COM_MOVING_RDSP_SMOOTH;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog2::on_BTN_COM_SMOOTH_MOTION_GO_LDSP_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIGHT_COM_MOVING_LDSP_SMOOTH;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog2::on_BTN_RFSWINGUP_GO_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIGHT_RFSWINGUP_DYNAMIC;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog2::on_BTN_RFSWINGDOWN_GO_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIGHT_RFSWINGDOWN_DYNAMIC;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog2::on_BTN_LFSWINGUP_GO_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIGHT_LFSWINGUP_DYNAMIC;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog2::on_BTN_LFSWINGDOWN_GO_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIGHT_LFSWINGDOWN_DYNAMIC;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog2::on_BTN_DYNAMICWALKING_GO_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->CON_DYNAMICWALKING_STEP_TIME->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->CON_DYNAMICWALKING_DSP_TIME->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->CON_DYNAMICWALKING_SWAY_LENGTH->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_INT[0] = ui->CON_DYNAMICWALKING_STEP_NUMBER->text().toInt();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[3] = ui->CON_DYNAMICWALKING_OFFSET_X->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[4] = ui->CON_DYNAMICWALKING_OFFSET_Y->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[5] = ui->CON_DYNAMICWALKING_OFFSET_YAW->text().toDouble();

    cmd.COMMAND_DATA.USER_PARA_DOUBLE[6] = ui->CON_DYNAMICWALKING_X_STEP->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[7] = ui->CON_DYNAMICWALKING_Y_STEP->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[8] = ui->CON_DYNAMICWALKING_YAW_ROTATION->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[9] = ui->CON_DYNAMICWALKING_STANCE_OFFSET->text().toDouble();

    cmd.COMMAND_DATA.USER_COMMAND = LIGHT_WALK;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}


void LIGHTWalkingDialog2::on_BTN_DYNAMICWALKING_JOYSTICK_GO_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[0] = ui->CON_DYNAMICWALKING_STEP_TIME->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[1] = ui->CON_DYNAMICWALKING_DSP_TIME->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[2] = ui->CON_DYNAMICWALKING_SWAY_LENGTH->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[3] = ui->CON_DYNAMICWALKING_OFFSET_X->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[4] = ui->CON_DYNAMICWALKING_OFFSET_Y->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[5] = ui->CON_DYNAMICWALKING_OFFSET_YAW->text().toDouble();
    cmd.COMMAND_DATA.USER_PARA_DOUBLE[6] = ui->CON_DYNAMICWALKING_STANCE_OFFSET->text().toDouble();

    cmd.COMMAND_DATA.USER_COMMAND = LIGHT_WALK_withJOYSTICK;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog2::on_BTN_SAVE_START_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = true; // save flag on!
    cmd.COMMAND_DATA.USER_COMMAND = LIGHT_DATA_SAVE;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog2::on_BTN_SAVE_END_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = false; // save flag off
    string filename = ui->CON_SAVE_FILENAME->text().toStdString(); // filename string
    if(filename.empty()) {
        cmd.COMMAND_DATA.USER_PARA_CHAR[0] = NULL;
    } else {
        strcpy(cmd.COMMAND_DATA.USER_PARA_CHAR, filename.c_str());
    }
    cmd.COMMAND_DATA.USER_COMMAND = LIGHT_DATA_SAVE;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog2::on_BTN_FULLTASK_GO_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIGHT_FULLTASK;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void LIGHTWalkingDialog2::on_CBX_FULLTASK_UPDATE_PUMPREF_toggled(bool checked)
{
    if(checked == true){
        palette->setColor(QPalette::Base,Qt::green);
        ui->LAMP_FULLTASK_UPDATE_PUMPREF->setPalette(*palette);
    } else if(checked == false){
        palette->setColor(QPalette::Base,Qt::red);
        ui->LAMP_FULLTASK_UPDATE_PUMPREF->setPalette(*palette);
    }

    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = LIGHT_SUPPLYPRESSURE_SETTING;
    cmd.COMMAND_DATA.USER_PARA_INT[0] = _OPERATION_LIGHT_FULL_SCINARIO;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = checked;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}
