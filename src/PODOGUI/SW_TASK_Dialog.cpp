#include "SW_TASK_Dialog.h"
#include "ui_SW_TASK_Dialog.h"

#include <QMessageBox>
#include <qvector2d.h>

#include "BasicFiles/PODOALDialog.h"

#include <iostream>
#include <iomanip>
using namespace std;

SW_TASK_Dialog::SW_TASK_Dialog(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::SW_TASK_Dialog)
{
    ui->setupUi(this);

    ALNum = PODOALDialog::GetALNumFromFileName("SW_TASK");

//    connect(pLAN, SIGNAL(NewPODOData()), this, SLOT(UpdateSettings()));
//    connect(displayTimer, SIGNAL(timeout()), this, SLOT(UpdateSettings()));
}

typedef enum SW_TASK_COMMAND_SET{
    SW_TASK_NO_ACT = 0,
    // For process handle
    SW_TASK_TASK1,
    SW_TASK_TASK2,

}SW_TASK_COMMAND;

SW_TASK_Dialog::~SW_TASK_Dialog()
{
    delete ui;
}

//void SW_TASK_Dialog::UpdateSettings(){
//    int mcId, mcCh, row;
//    QTableWidget *tw;
//    QString str;

//        ui->customPlot->addGraph();
//        ui->customPlot->graph(0)->setPen(QPen(Qt::blue)); // line color blue for first graph
////        ui->customPlot->graph(0)->setBrush(QBrush(QColor(0, 0, 255, 20))); // first graph will be filled with translucent blue
//        ui->customPlot->addGraph();
//        ui->customPlot->graph(1)->setPen(QPen(Qt::red)); // line color red for second graph
//        ui->customPlot->xAxis->setLabel("Valve Voltage (V)");
//        ui->customPlot->yAxis->setLabel("flow rate (lpm)");
//        ui->customPlot->xAxis->setRange(-5.5, 5.5);
//        ui->customPlot->yAxis->setRange(-20, 20);
//        ui->customPlot->xAxis->setVisible(true);
//        ui->customPlot->yAxis->setVisible(true);
//        ui->customPlot->replot();

//    QVector<double> V(12);
//    QVector<double> Valv_Gain(12);
//    for (int j=0; j<5; ++j)
//    {
//        V[4-j]=-j-1;
//        V[j+7]=j+1;
//        Valv_Gain[4-j]=PODO_DATA.CoreSEN.ENCODER[0][0].HCB_Info.VALVE_GAIN_MINUS[j]*(double)(-j-1);
//        Valv_Gain[j+7]=PODO_DATA.CoreSEN.ENCODER[0][0].HCB_Info.VALVE_GAIN_PLUS[j]*(double)(j+1);
//    }
//    V[5] = (double)PODO_DATA.CoreSEN.ENCODER[0][0].HCB_Info.VALVE_DZ_MINUS * 0.001;
//    V[6] = (double)PODO_DATA.CoreSEN.ENCODER[0][0].HCB_Info.VALVE_DZ_PLUS * 0.001;

//    Valv_Gain[5]=0;
//    Valv_Gain[6]=0;

//    ui->customPlot->graph(0)->setData(V, Valv_Gain);
////    ui->customPlot->graph(1)->setData(x_time, Output_tor);
//    ui->customPlot->replot();


//}

void SW_TASK_Dialog::on_BTN_FT_NULL_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_SENSOR_FT_NULL;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);
}

void SW_TASK_Dialog::on_BTN_MOVE_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 0; //BNO
    cmd.COMMAND_DATA.USER_PARA_INT[0] = ui->LE_1000->text().toInt(); // Command to all joints
    cmd.COMMAND_DATA.USER_COMMAND = SW_TASK_TASK2;
    cmd.COMMAND_TARGET = ALNum;
    pLAN->SendCommand(cmd);

}

void SW_TASK_Dialog::on_BTN_PWM_ON_clicked()
{
    USER_COMMAND cmd;
    cmd.COMMAND_DATA.USER_PARA_CHAR[0] = 5; //ref type
    cmd.COMMAND_DATA.USER_PARA_CHAR[1] = 1; // on
    cmd.COMMAND_DATA.USER_COMMAND = DAEMON4LIGHT_MOTION_REF_ONOFF;
    cmd.COMMAND_TARGET = RBCORE_PODO_NO;
    pLAN->SendCommand(cmd);
}

void SW_TASK_Dialog::on_BTN_SHOW_clicked()
{
    int BNO = 0;
    ui->LE_VALV_GAIN_PLUS_1->setText(QString().sprintf("%f",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.VALVE_GAIN_PLUS[0]));
    ui->LE_VALV_GAIN_PLUS_2->setText(QString().sprintf("%f",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.VALVE_GAIN_PLUS[1]));
    ui->LE_VALV_GAIN_PLUS_3->setText(QString().sprintf("%f",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.VALVE_GAIN_PLUS[2]));
    ui->LE_VALV_GAIN_PLUS_4->setText(QString().sprintf("%f",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.VALVE_GAIN_PLUS[3]));
    ui->LE_VALV_GAIN_PLUS_5->setText(QString().sprintf("%f",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.VALVE_GAIN_PLUS[4]));

    ui->LE_VALV_GAIN_MINUS_1->setText(QString().sprintf("%f",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.VALVE_GAIN_MINUS[0]));
    ui->LE_VALV_GAIN_MINUS_2->setText(QString().sprintf("%f",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.VALVE_GAIN_MINUS[1]));
    ui->LE_VALV_GAIN_MINUS_3->setText(QString().sprintf("%f",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.VALVE_GAIN_MINUS[2]));
    ui->LE_VALV_GAIN_MINUS_4->setText(QString().sprintf("%f",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.VALVE_GAIN_MINUS[3]));
    ui->LE_VALV_GAIN_MINUS_5->setText(QString().sprintf("%f",PODO_DATA.CoreSEN.ENCODER[BNO][0].HCB_Info.VALVE_GAIN_MINUS[4]));
}
