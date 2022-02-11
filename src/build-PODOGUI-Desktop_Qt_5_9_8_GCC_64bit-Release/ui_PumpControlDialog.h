/********************************************************************************
** Form generated from reading UI file 'PumpControlDialog.ui'
**
** Created by: Qt User Interface Compiler version 5.9.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PUMPCONTROLDIALOG_H
#define UI_PUMPCONTROLDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_PumpControlDialog
{
public:
    QPushButton *BTN_SEND_DUTY;
    QLineEdit *CON_DUTY;
    QPushButton *BTN_SEND_DUTY_ZERO;
    QPushButton *BTN_SEND_PRESSURE_NULL;
    QLabel *label_4;
    QLabel *label_5;
    QLineEdit *EDIT_PUMP_PRESSURE;
    QLabel *label_6;
    QLabel *label_7;
    QLineEdit *EDIT_PUMP_DUTYREFERENCE;
    QPushButton *BTN_FINDHOME_ALL;
    QLineEdit *EDIT_PUMP_VELOCITY;
    QPushButton *BTN_SEND_PUMPDATA_REQUEST_ON;
    QLabel *label_14;
    QLabel *label_15;
    QPushButton *BTN_SEND_PUMPDATA_REQUEST_OFF;
    QGroupBox *GB_JointSelection_9;
    QPushButton *BTN_SAVE_START;
    QPushButton *BTN_SAVE_STOP;
    QLabel *label_31;
    QLineEdit *CON_SAVE_FILENAME;
    QPushButton *BTN_SEND_PUMP_CONTROL_DISABLE;
    QPushButton *BTN_SEND_PUMP_CONTROL_ENABLE;
    QLabel *label_16;
    QLineEdit *EDIT_PUMP_TEMPERATURE;
    QLabel *label_17;
    QPushButton *BTN_TORQUEFORCE_NULLING;
    QLineEdit *CON_ACTIVE_DUTYCONTROL_SINEWAVE_MAG;
    QPushButton *BTN_ACTIVE_DUTYCONTROL_SINEWAVE_GO;
    QLabel *label_18;
    QLabel *label_19;
    QLineEdit *CON_ACTIVE_DUTYCONTROL_SINEWAVE_PER;
    QLabel *label_20;
    QLabel *label_21;
    QLineEdit *CON_ACTIVE_DUTYCONTROL_SINEWAVE_NUM;
    QLabel *label_23;
    QLabel *label_9;
    QLabel *label_22;
    QLineEdit *EDIT_PUMP_PRESSURE_REF;
    QLabel *label_24;
    QLabel *label_26;
    QLineEdit *CON_LINEARCHANGE_DES_PRES;
    QPushButton *BTN_LINEARCHANGE_GO;
    QGroupBox *GB_JointSelection_10;
    QPushButton *BTN_ACTIVE_DUTYCONTROL_ON;
    QLabel *label_10;
    QPushButton *BTN_ACTIVE_DUTYCONTROL_OFF;
    QLineEdit *CON_ACTIVE_DUTYCONTROL_INPUT_VOLTAGE;
    QLabel *label_11;
    QFrame *line;
    QGroupBox *GB_JointSelection_11;
    QPushButton *BTN_PUMPCONTROL_MPC_ON;
    QPushButton *BTN_PUMPCONTROL_MPC_OFF;
    QGroupBox *GB_JointSelection_12;
    QRadioButton *RBTN_PS_REF_SM;
    QLabel *label_32;
    QRadioButton *RBTN_PS_REF_THISAL;
    QLabel *label_25;
    QLabel *label_27;
    QLineEdit *CON_LINEARCHANGE_TIME;
    QGroupBox *GB_JointSelection_13;
    QRadioButton *RBTN_VARIABLE_PRES_MODE;
    QLabel *label_34;
    QRadioButton *RBTN_CONST_PRES_MODE;

    void setupUi(QWidget *PumpControlDialog)
    {
        if (PumpControlDialog->objectName().isEmpty())
            PumpControlDialog->setObjectName(QStringLiteral("PumpControlDialog"));
        PumpControlDialog->resize(780, 780);
        BTN_SEND_DUTY = new QPushButton(PumpControlDialog);
        BTN_SEND_DUTY->setObjectName(QStringLiteral("BTN_SEND_DUTY"));
        BTN_SEND_DUTY->setGeometry(QRect(504, 211, 91, 41));
        CON_DUTY = new QLineEdit(PumpControlDialog);
        CON_DUTY->setObjectName(QStringLiteral("CON_DUTY"));
        CON_DUTY->setGeometry(QRect(444, 211, 51, 41));
        CON_DUTY->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        BTN_SEND_DUTY_ZERO = new QPushButton(PumpControlDialog);
        BTN_SEND_DUTY_ZERO->setObjectName(QStringLiteral("BTN_SEND_DUTY_ZERO"));
        BTN_SEND_DUTY_ZERO->setGeometry(QRect(504, 260, 91, 41));
        BTN_SEND_PRESSURE_NULL = new QPushButton(PumpControlDialog);
        BTN_SEND_PRESSURE_NULL->setObjectName(QStringLiteral("BTN_SEND_PRESSURE_NULL"));
        BTN_SEND_PRESSURE_NULL->setEnabled(true);
        BTN_SEND_PRESSURE_NULL->setGeometry(QRect(20, 180, 101, 41));
        label_4 = new QLabel(PumpControlDialog);
        label_4->setObjectName(QStringLiteral("label_4"));
        label_4->setGeometry(QRect(444, 16, 111, 20));
        QFont font;
        font.setPointSize(9);
        label_4->setFont(font);
        label_4->setAlignment(Qt::AlignCenter);
        label_5 = new QLabel(PumpControlDialog);
        label_5->setObjectName(QStringLiteral("label_5"));
        label_5->setGeometry(QRect(573, 70, 31, 21));
        QFont font1;
        font1.setPointSize(12);
        label_5->setFont(font1);
        label_5->setAlignment(Qt::AlignCenter);
        EDIT_PUMP_PRESSURE = new QLineEdit(PumpControlDialog);
        EDIT_PUMP_PRESSURE->setObjectName(QStringLiteral("EDIT_PUMP_PRESSURE"));
        EDIT_PUMP_PRESSURE->setGeometry(QRect(446, 40, 121, 51));
        QFont font2;
        font2.setPointSize(22);
        EDIT_PUMP_PRESSURE->setFont(font2);
        EDIT_PUMP_PRESSURE->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        EDIT_PUMP_PRESSURE->setReadOnly(true);
        label_6 = new QLabel(PumpControlDialog);
        label_6->setObjectName(QStringLiteral("label_6"));
        label_6->setGeometry(QRect(374, 250, 71, 16));
        label_6->setFont(font1);
        label_6->setAlignment(Qt::AlignCenter);
        label_7 = new QLabel(PumpControlDialog);
        label_7->setObjectName(QStringLiteral("label_7"));
        label_7->setGeometry(QRect(266, 198, 131, 20));
        label_7->setFont(font);
        label_7->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        EDIT_PUMP_DUTYREFERENCE = new QLineEdit(PumpControlDialog);
        EDIT_PUMP_DUTYREFERENCE->setObjectName(QStringLiteral("EDIT_PUMP_DUTYREFERENCE"));
        EDIT_PUMP_DUTYREFERENCE->setGeometry(QRect(266, 220, 121, 51));
        EDIT_PUMP_DUTYREFERENCE->setFont(font2);
        EDIT_PUMP_DUTYREFERENCE->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        EDIT_PUMP_DUTYREFERENCE->setReadOnly(true);
        BTN_FINDHOME_ALL = new QPushButton(PumpControlDialog);
        BTN_FINDHOME_ALL->setObjectName(QStringLiteral("BTN_FINDHOME_ALL"));
        BTN_FINDHOME_ALL->setGeometry(QRect(630, 20, 121, 261));
        QFont font3;
        font3.setPointSize(10);
        BTN_FINDHOME_ALL->setFont(font3);
        EDIT_PUMP_VELOCITY = new QLineEdit(PumpControlDialog);
        EDIT_PUMP_VELOCITY->setObjectName(QStringLiteral("EDIT_PUMP_VELOCITY"));
        EDIT_PUMP_VELOCITY->setGeometry(QRect(268, 130, 121, 51));
        EDIT_PUMP_VELOCITY->setFont(font2);
        EDIT_PUMP_VELOCITY->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        EDIT_PUMP_VELOCITY->setReadOnly(true);
        BTN_SEND_PUMPDATA_REQUEST_ON = new QPushButton(PumpControlDialog);
        BTN_SEND_PUMPDATA_REQUEST_ON->setObjectName(QStringLiteral("BTN_SEND_PUMPDATA_REQUEST_ON"));
        BTN_SEND_PUMPDATA_REQUEST_ON->setGeometry(QRect(20, 130, 101, 41));
        QFont font4;
        font4.setPointSize(8);
        BTN_SEND_PUMPDATA_REQUEST_ON->setFont(font4);
        label_14 = new QLabel(PumpControlDialog);
        label_14->setObjectName(QStringLiteral("label_14"));
        label_14->setGeometry(QRect(268, 107, 121, 20));
        label_14->setFont(font);
        label_14->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        label_15 = new QLabel(PumpControlDialog);
        label_15->setObjectName(QStringLiteral("label_15"));
        label_15->setGeometry(QRect(396, 160, 30, 21));
        label_15->setFont(font1);
        label_15->setAlignment(Qt::AlignCenter);
        BTN_SEND_PUMPDATA_REQUEST_OFF = new QPushButton(PumpControlDialog);
        BTN_SEND_PUMPDATA_REQUEST_OFF->setObjectName(QStringLiteral("BTN_SEND_PUMPDATA_REQUEST_OFF"));
        BTN_SEND_PUMPDATA_REQUEST_OFF->setGeometry(QRect(130, 130, 101, 41));
        BTN_SEND_PUMPDATA_REQUEST_OFF->setFont(font4);
        GB_JointSelection_9 = new QGroupBox(PumpControlDialog);
        GB_JointSelection_9->setObjectName(QStringLiteral("GB_JointSelection_9"));
        GB_JointSelection_9->setGeometry(QRect(16, 10, 221, 101));
        QFont font5;
        font5.setBold(true);
        font5.setWeight(75);
        GB_JointSelection_9->setFont(font5);
        GB_JointSelection_9->setStyleSheet(QLatin1String("QGroupBox {\n"
"    border: 1px solid gray;\n"
"    border-radius: 5px;\n"
"    margin-top: 7px;\n"
"}\n"
"\n"
"QGroupBox::title {\n"
"    subcontrol-origin: margin;\n"
"    left: 8px;\n"
"    padding: 0 3px 0 3px;\n"
"}\n"
""));
        BTN_SAVE_START = new QPushButton(GB_JointSelection_9);
        BTN_SAVE_START->setObjectName(QStringLiteral("BTN_SAVE_START"));
        BTN_SAVE_START->setGeometry(QRect(20, 20, 81, 31));
        BTN_SAVE_STOP = new QPushButton(GB_JointSelection_9);
        BTN_SAVE_STOP->setObjectName(QStringLiteral("BTN_SAVE_STOP"));
        BTN_SAVE_STOP->setGeometry(QRect(120, 20, 81, 31));
        label_31 = new QLabel(GB_JointSelection_9);
        label_31->setObjectName(QStringLiteral("label_31"));
        label_31->setGeometry(QRect(17, 65, 61, 20));
        label_31->setFont(font);
        label_31->setAlignment(Qt::AlignCenter);
        CON_SAVE_FILENAME = new QLineEdit(GB_JointSelection_9);
        CON_SAVE_FILENAME->setObjectName(QStringLiteral("CON_SAVE_FILENAME"));
        CON_SAVE_FILENAME->setGeometry(QRect(83, 60, 121, 31));
        CON_SAVE_FILENAME->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        BTN_SEND_PUMP_CONTROL_DISABLE = new QPushButton(PumpControlDialog);
        BTN_SEND_PUMP_CONTROL_DISABLE->setObjectName(QStringLiteral("BTN_SEND_PUMP_CONTROL_DISABLE"));
        BTN_SEND_PUMP_CONTROL_DISABLE->setGeometry(QRect(130, 240, 101, 41));
        BTN_SEND_PUMP_CONTROL_DISABLE->setFont(font4);
        BTN_SEND_PUMP_CONTROL_ENABLE = new QPushButton(PumpControlDialog);
        BTN_SEND_PUMP_CONTROL_ENABLE->setObjectName(QStringLiteral("BTN_SEND_PUMP_CONTROL_ENABLE"));
        BTN_SEND_PUMP_CONTROL_ENABLE->setGeometry(QRect(20, 240, 101, 41));
        BTN_SEND_PUMP_CONTROL_ENABLE->setFont(font4);
        label_16 = new QLabel(PumpControlDialog);
        label_16->setObjectName(QStringLiteral("label_16"));
        label_16->setGeometry(QRect(396, 70, 31, 21));
        label_16->setFont(font1);
        label_16->setAlignment(Qt::AlignCenter);
        EDIT_PUMP_TEMPERATURE = new QLineEdit(PumpControlDialog);
        EDIT_PUMP_TEMPERATURE->setObjectName(QStringLiteral("EDIT_PUMP_TEMPERATURE"));
        EDIT_PUMP_TEMPERATURE->setGeometry(QRect(269, 41, 121, 51));
        EDIT_PUMP_TEMPERATURE->setFont(font2);
        EDIT_PUMP_TEMPERATURE->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        EDIT_PUMP_TEMPERATURE->setReadOnly(true);
        label_17 = new QLabel(PumpControlDialog);
        label_17->setObjectName(QStringLiteral("label_17"));
        label_17->setGeometry(QRect(269, 17, 131, 20));
        label_17->setFont(font);
        label_17->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        BTN_TORQUEFORCE_NULLING = new QPushButton(PumpControlDialog);
        BTN_TORQUEFORCE_NULLING->setObjectName(QStringLiteral("BTN_TORQUEFORCE_NULLING"));
        BTN_TORQUEFORCE_NULLING->setGeometry(QRect(20, 290, 101, 41));
        BTN_TORQUEFORCE_NULLING->setFont(font4);
        CON_ACTIVE_DUTYCONTROL_SINEWAVE_MAG = new QLineEdit(PumpControlDialog);
        CON_ACTIVE_DUTYCONTROL_SINEWAVE_MAG->setObjectName(QStringLiteral("CON_ACTIVE_DUTYCONTROL_SINEWAVE_MAG"));
        CON_ACTIVE_DUTYCONTROL_SINEWAVE_MAG->setGeometry(QRect(100, 518, 41, 21));
        CON_ACTIVE_DUTYCONTROL_SINEWAVE_MAG->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        BTN_ACTIVE_DUTYCONTROL_SINEWAVE_GO = new QPushButton(PumpControlDialog);
        BTN_ACTIVE_DUTYCONTROL_SINEWAVE_GO->setObjectName(QStringLiteral("BTN_ACTIVE_DUTYCONTROL_SINEWAVE_GO"));
        BTN_ACTIVE_DUTYCONTROL_SINEWAVE_GO->setGeometry(QRect(40, 600, 121, 41));
        label_18 = new QLabel(PumpControlDialog);
        label_18->setObjectName(QStringLiteral("label_18"));
        label_18->setGeometry(QRect(140, 545, 31, 20));
        label_18->setFont(font);
        label_18->setAlignment(Qt::AlignCenter);
        label_19 = new QLabel(PumpControlDialog);
        label_19->setObjectName(QStringLiteral("label_19"));
        label_19->setGeometry(QRect(10, 518, 91, 20));
        label_19->setFont(font);
        label_19->setAlignment(Qt::AlignCenter);
        CON_ACTIVE_DUTYCONTROL_SINEWAVE_PER = new QLineEdit(PumpControlDialog);
        CON_ACTIVE_DUTYCONTROL_SINEWAVE_PER->setObjectName(QStringLiteral("CON_ACTIVE_DUTYCONTROL_SINEWAVE_PER"));
        CON_ACTIVE_DUTYCONTROL_SINEWAVE_PER->setGeometry(QRect(100, 544, 41, 21));
        CON_ACTIVE_DUTYCONTROL_SINEWAVE_PER->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label_20 = new QLabel(PumpControlDialog);
        label_20->setObjectName(QStringLiteral("label_20"));
        label_20->setGeometry(QRect(40, 544, 61, 20));
        label_20->setFont(font);
        label_20->setAlignment(Qt::AlignCenter);
        label_21 = new QLabel(PumpControlDialog);
        label_21->setObjectName(QStringLiteral("label_21"));
        label_21->setGeometry(QRect(140, 519, 31, 20));
        label_21->setFont(font);
        label_21->setAlignment(Qt::AlignCenter);
        CON_ACTIVE_DUTYCONTROL_SINEWAVE_NUM = new QLineEdit(PumpControlDialog);
        CON_ACTIVE_DUTYCONTROL_SINEWAVE_NUM->setObjectName(QStringLiteral("CON_ACTIVE_DUTYCONTROL_SINEWAVE_NUM"));
        CON_ACTIVE_DUTYCONTROL_SINEWAVE_NUM->setGeometry(QRect(100, 570, 41, 21));
        CON_ACTIVE_DUTYCONTROL_SINEWAVE_NUM->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label_23 = new QLabel(PumpControlDialog);
        label_23->setObjectName(QStringLiteral("label_23"));
        label_23->setGeometry(QRect(30, 570, 71, 20));
        label_23->setFont(font);
        label_23->setAlignment(Qt::AlignCenter);
        label_9 = new QLabel(PumpControlDialog);
        label_9->setObjectName(QStringLiteral("label_9"));
        label_9->setGeometry(QRect(443, 108, 141, 20));
        label_9->setFont(font);
        label_9->setAlignment(Qt::AlignCenter);
        label_22 = new QLabel(PumpControlDialog);
        label_22->setObjectName(QStringLiteral("label_22"));
        label_22->setGeometry(QRect(573, 160, 31, 21));
        label_22->setFont(font1);
        label_22->setAlignment(Qt::AlignCenter);
        EDIT_PUMP_PRESSURE_REF = new QLineEdit(PumpControlDialog);
        EDIT_PUMP_PRESSURE_REF->setObjectName(QStringLiteral("EDIT_PUMP_PRESSURE_REF"));
        EDIT_PUMP_PRESSURE_REF->setGeometry(QRect(446, 130, 121, 51));
        EDIT_PUMP_PRESSURE_REF->setFont(font2);
        EDIT_PUMP_PRESSURE_REF->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        EDIT_PUMP_PRESSURE_REF->setReadOnly(true);
        label_24 = new QLabel(PumpControlDialog);
        label_24->setObjectName(QStringLiteral("label_24"));
        label_24->setGeometry(QRect(192, 521, 71, 41));
        label_24->setFont(font);
        label_24->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        label_26 = new QLabel(PumpControlDialog);
        label_26->setObjectName(QStringLiteral("label_26"));
        label_26->setGeometry(QRect(311, 540, 31, 20));
        label_26->setFont(font);
        label_26->setAlignment(Qt::AlignCenter);
        CON_LINEARCHANGE_DES_PRES = new QLineEdit(PumpControlDialog);
        CON_LINEARCHANGE_DES_PRES->setObjectName(QStringLiteral("CON_LINEARCHANGE_DES_PRES"));
        CON_LINEARCHANGE_DES_PRES->setGeometry(QRect(261, 537, 51, 21));
        CON_LINEARCHANGE_DES_PRES->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        BTN_LINEARCHANGE_GO = new QPushButton(PumpControlDialog);
        BTN_LINEARCHANGE_GO->setObjectName(QStringLiteral("BTN_LINEARCHANGE_GO"));
        BTN_LINEARCHANGE_GO->setEnabled(true);
        BTN_LINEARCHANGE_GO->setGeometry(QRect(199, 600, 131, 41));
        GB_JointSelection_10 = new QGroupBox(PumpControlDialog);
        GB_JointSelection_10->setObjectName(QStringLiteral("GB_JointSelection_10"));
        GB_JointSelection_10->setGeometry(QRect(16, 370, 221, 131));
        GB_JointSelection_10->setFont(font5);
        GB_JointSelection_10->setStyleSheet(QLatin1String("QGroupBox {\n"
"    border: 1px solid gray;\n"
"    border-radius: 5px;\n"
"    margin-top: 7px;\n"
"}\n"
"\n"
"QGroupBox::title {\n"
"    subcontrol-origin: margin;\n"
"    left: 8px;\n"
"    padding: 0 3px 0 3px;\n"
"}\n"
""));
        BTN_ACTIVE_DUTYCONTROL_ON = new QPushButton(GB_JointSelection_10);
        BTN_ACTIVE_DUTYCONTROL_ON->setObjectName(QStringLiteral("BTN_ACTIVE_DUTYCONTROL_ON"));
        BTN_ACTIVE_DUTYCONTROL_ON->setGeometry(QRect(15, 70, 91, 41));
        label_10 = new QLabel(GB_JointSelection_10);
        label_10->setObjectName(QStringLiteral("label_10"));
        label_10->setGeometry(QRect(32, 30, 101, 20));
        label_10->setFont(font);
        label_10->setAlignment(Qt::AlignCenter);
        BTN_ACTIVE_DUTYCONTROL_OFF = new QPushButton(GB_JointSelection_10);
        BTN_ACTIVE_DUTYCONTROL_OFF->setObjectName(QStringLiteral("BTN_ACTIVE_DUTYCONTROL_OFF"));
        BTN_ACTIVE_DUTYCONTROL_OFF->setGeometry(QRect(115, 70, 91, 41));
        CON_ACTIVE_DUTYCONTROL_INPUT_VOLTAGE = new QLineEdit(GB_JointSelection_10);
        CON_ACTIVE_DUTYCONTROL_INPUT_VOLTAGE->setObjectName(QStringLiteral("CON_ACTIVE_DUTYCONTROL_INPUT_VOLTAGE"));
        CON_ACTIVE_DUTYCONTROL_INPUT_VOLTAGE->setGeometry(QRect(132, 30, 41, 21));
        CON_ACTIVE_DUTYCONTROL_INPUT_VOLTAGE->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label_11 = new QLabel(GB_JointSelection_10);
        label_11->setObjectName(QStringLiteral("label_11"));
        label_11->setGeometry(QRect(171, 31, 21, 20));
        label_11->setFont(font);
        label_11->setAlignment(Qt::AlignCenter);
        line = new QFrame(PumpControlDialog);
        line->setObjectName(QStringLiteral("line"));
        line->setGeometry(QRect(40, 346, 691, 10));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);
        GB_JointSelection_11 = new QGroupBox(PumpControlDialog);
        GB_JointSelection_11->setObjectName(QStringLiteral("GB_JointSelection_11"));
        GB_JointSelection_11->setGeometry(QRect(246, 370, 111, 131));
        GB_JointSelection_11->setFont(font5);
        GB_JointSelection_11->setStyleSheet(QLatin1String("QGroupBox {\n"
"    border: 1px solid gray;\n"
"    border-radius: 5px;\n"
"    margin-top: 7px;\n"
"}\n"
"\n"
"QGroupBox::title {\n"
"    subcontrol-origin: margin;\n"
"    left: 8px;\n"
"    padding: 0 3px 0 3px;\n"
"}\n"
""));
        BTN_PUMPCONTROL_MPC_ON = new QPushButton(GB_JointSelection_11);
        BTN_PUMPCONTROL_MPC_ON->setObjectName(QStringLiteral("BTN_PUMPCONTROL_MPC_ON"));
        BTN_PUMPCONTROL_MPC_ON->setGeometry(QRect(10, 25, 91, 41));
        BTN_PUMPCONTROL_MPC_OFF = new QPushButton(GB_JointSelection_11);
        BTN_PUMPCONTROL_MPC_OFF->setObjectName(QStringLiteral("BTN_PUMPCONTROL_MPC_OFF"));
        BTN_PUMPCONTROL_MPC_OFF->setGeometry(QRect(10, 75, 91, 41));
        GB_JointSelection_12 = new QGroupBox(PumpControlDialog);
        GB_JointSelection_12->setObjectName(QStringLiteral("GB_JointSelection_12"));
        GB_JointSelection_12->setGeometry(QRect(365, 370, 121, 131));
        GB_JointSelection_12->setFont(font5);
        GB_JointSelection_12->setStyleSheet(QLatin1String("QGroupBox {\n"
"    border: 1px solid gray;\n"
"    border-radius: 5px;\n"
"    margin-top: 7px;\n"
"}\n"
"\n"
"QGroupBox::title {\n"
"    subcontrol-origin: margin;\n"
"    left: 8px;\n"
"    padding: 0 3px 0 3px;\n"
"}\n"
""));
        RBTN_PS_REF_SM = new QRadioButton(GB_JointSelection_12);
        RBTN_PS_REF_SM->setObjectName(QStringLiteral("RBTN_PS_REF_SM"));
        RBTN_PS_REF_SM->setGeometry(QRect(10, 93, 111, 20));
        label_32 = new QLabel(GB_JointSelection_12);
        label_32->setObjectName(QStringLiteral("label_32"));
        label_32->setGeometry(QRect(16, 17, 91, 41));
        QFont font6;
        font6.setPointSize(10);
        font6.setBold(true);
        font6.setWeight(75);
        label_32->setFont(font6);
        label_32->setAlignment(Qt::AlignCenter);
        RBTN_PS_REF_THISAL = new QRadioButton(GB_JointSelection_12);
        RBTN_PS_REF_THISAL->setObjectName(QStringLiteral("RBTN_PS_REF_THISAL"));
        RBTN_PS_REF_THISAL->setGeometry(QRect(10, 70, 81, 20));
        RBTN_PS_REF_THISAL->setChecked(true);
        label_25 = new QLabel(PumpControlDialog);
        label_25->setObjectName(QStringLiteral("label_25"));
        label_25->setGeometry(QRect(212, 563, 41, 21));
        label_25->setFont(font);
        label_25->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        label_27 = new QLabel(PumpControlDialog);
        label_27->setObjectName(QStringLiteral("label_27"));
        label_27->setGeometry(QRect(310, 567, 31, 20));
        label_27->setFont(font);
        label_27->setAlignment(Qt::AlignCenter);
        CON_LINEARCHANGE_TIME = new QLineEdit(PumpControlDialog);
        CON_LINEARCHANGE_TIME->setObjectName(QStringLiteral("CON_LINEARCHANGE_TIME"));
        CON_LINEARCHANGE_TIME->setGeometry(QRect(260, 564, 51, 21));
        CON_LINEARCHANGE_TIME->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        GB_JointSelection_13 = new QGroupBox(PumpControlDialog);
        GB_JointSelection_13->setObjectName(QStringLiteral("GB_JointSelection_13"));
        GB_JointSelection_13->setGeometry(QRect(494, 370, 121, 131));
        GB_JointSelection_13->setFont(font5);
        GB_JointSelection_13->setStyleSheet(QLatin1String("QGroupBox {\n"
"    border: 1px solid gray;\n"
"    border-radius: 5px;\n"
"    margin-top: 7px;\n"
"}\n"
"\n"
"QGroupBox::title {\n"
"    subcontrol-origin: margin;\n"
"    left: 8px;\n"
"    padding: 0 3px 0 3px;\n"
"}\n"
""));
        RBTN_VARIABLE_PRES_MODE = new QRadioButton(GB_JointSelection_13);
        RBTN_VARIABLE_PRES_MODE->setObjectName(QStringLiteral("RBTN_VARIABLE_PRES_MODE"));
        RBTN_VARIABLE_PRES_MODE->setGeometry(QRect(18, 93, 91, 20));
        label_34 = new QLabel(GB_JointSelection_13);
        label_34->setObjectName(QStringLiteral("label_34"));
        label_34->setGeometry(QRect(0, 17, 121, 41));
        label_34->setFont(font6);
        label_34->setAlignment(Qt::AlignCenter);
        RBTN_CONST_PRES_MODE = new QRadioButton(GB_JointSelection_13);
        RBTN_CONST_PRES_MODE->setObjectName(QStringLiteral("RBTN_CONST_PRES_MODE"));
        RBTN_CONST_PRES_MODE->setGeometry(QRect(18, 70, 81, 20));
        RBTN_CONST_PRES_MODE->setChecked(true);

        retranslateUi(PumpControlDialog);

        QMetaObject::connectSlotsByName(PumpControlDialog);
    } // setupUi

    void retranslateUi(QWidget *PumpControlDialog)
    {
        PumpControlDialog->setWindowTitle(QApplication::translate("PumpControlDialog", "Form", Q_NULLPTR));
        BTN_SEND_DUTY->setText(QApplication::translate("PumpControlDialog", "Send Duty", Q_NULLPTR));
        CON_DUTY->setText(QApplication::translate("PumpControlDialog", "200", Q_NULLPTR));
        BTN_SEND_DUTY_ZERO->setText(QApplication::translate("PumpControlDialog", "Duty Zero", Q_NULLPTR));
        BTN_SEND_PRESSURE_NULL->setText(QApplication::translate("PumpControlDialog", "Press. Null", Q_NULLPTR));
        label_4->setText(QApplication::translate("PumpControlDialog", "Supply Pressure :", Q_NULLPTR));
        label_5->setText(QApplication::translate("PumpControlDialog", "bar", Q_NULLPTR));
        EDIT_PUMP_PRESSURE->setText(QString());
        label_6->setText(QApplication::translate("PumpControlDialog", "rpm", Q_NULLPTR));
        label_7->setText(QApplication::translate("PumpControlDialog", "Pump Speed Ref.", Q_NULLPTR));
        BTN_FINDHOME_ALL->setText(QApplication::translate("PumpControlDialog", "FindHome\n"
"\n"
"All Joint", Q_NULLPTR));
        BTN_SEND_PUMPDATA_REQUEST_ON->setText(QApplication::translate("PumpControlDialog", "Pump Data\n"
"Request ON", Q_NULLPTR));
        label_14->setText(QApplication::translate("PumpControlDialog", "Pump Speed :", Q_NULLPTR));
        label_15->setText(QApplication::translate("PumpControlDialog", "rpm", Q_NULLPTR));
        BTN_SEND_PUMPDATA_REQUEST_OFF->setText(QApplication::translate("PumpControlDialog", "Pump Data\n"
"Request OFF", Q_NULLPTR));
        GB_JointSelection_9->setTitle(QApplication::translate("PumpControlDialog", "Data Save", Q_NULLPTR));
        BTN_SAVE_START->setText(QApplication::translate("PumpControlDialog", "Save Start", Q_NULLPTR));
        BTN_SAVE_STOP->setText(QApplication::translate("PumpControlDialog", "Save Stop", Q_NULLPTR));
        label_31->setText(QApplication::translate("PumpControlDialog", "Subtitle :", Q_NULLPTR));
        CON_SAVE_FILENAME->setText(QString());
        BTN_SEND_PUMP_CONTROL_DISABLE->setText(QApplication::translate("PumpControlDialog", "Pump Control\n"
"Disable", Q_NULLPTR));
        BTN_SEND_PUMP_CONTROL_ENABLE->setText(QApplication::translate("PumpControlDialog", "Pump Control\n"
"Enable", Q_NULLPTR));
        label_16->setText(QApplication::translate("PumpControlDialog", "deg", Q_NULLPTR));
        EDIT_PUMP_TEMPERATURE->setText(QString());
        label_17->setText(QApplication::translate("PumpControlDialog", "Wire Temperature :", Q_NULLPTR));
        BTN_TORQUEFORCE_NULLING->setText(QApplication::translate("PumpControlDialog", "Force/Pressure\n"
"Sensor Nulling", Q_NULLPTR));
        CON_ACTIVE_DUTYCONTROL_SINEWAVE_MAG->setText(QApplication::translate("PumpControlDialog", "40", Q_NULLPTR));
        BTN_ACTIVE_DUTYCONTROL_SINEWAVE_GO->setText(QApplication::translate("PumpControlDialog", "SineWave \n"
"Go", Q_NULLPTR));
        label_18->setText(QApplication::translate("PumpControlDialog", "sec", Q_NULLPTR));
        label_19->setText(QApplication::translate("PumpControlDialog", "Magnitude :", Q_NULLPTR));
        CON_ACTIVE_DUTYCONTROL_SINEWAVE_PER->setText(QApplication::translate("PumpControlDialog", "2.0", Q_NULLPTR));
        label_20->setText(QApplication::translate("PumpControlDialog", "Period :", Q_NULLPTR));
        label_21->setText(QApplication::translate("PumpControlDialog", "bar", Q_NULLPTR));
        CON_ACTIVE_DUTYCONTROL_SINEWAVE_NUM->setText(QApplication::translate("PumpControlDialog", "5", Q_NULLPTR));
        label_23->setText(QApplication::translate("PumpControlDialog", "Number :", Q_NULLPTR));
        label_9->setText(QApplication::translate("PumpControlDialog", "Supply Pressure Ref. :", Q_NULLPTR));
        label_22->setText(QApplication::translate("PumpControlDialog", "bar", Q_NULLPTR));
        EDIT_PUMP_PRESSURE_REF->setText(QString());
        label_24->setText(QApplication::translate("PumpControlDialog", "Desired\n"
"Pressure :", Q_NULLPTR));
        label_26->setText(QApplication::translate("PumpControlDialog", "bar", Q_NULLPTR));
        CON_LINEARCHANGE_DES_PRES->setText(QApplication::translate("PumpControlDialog", "60", Q_NULLPTR));
        BTN_LINEARCHANGE_GO->setText(QApplication::translate("PumpControlDialog", "Linearly Change \n"
"Go", Q_NULLPTR));
        GB_JointSelection_10->setTitle(QApplication::translate("PumpControlDialog", "Active Control On/Off", Q_NULLPTR));
        BTN_ACTIVE_DUTYCONTROL_ON->setText(QApplication::translate("PumpControlDialog", "Active Ctrl. \n"
"On", Q_NULLPTR));
        label_10->setText(QApplication::translate("PumpControlDialog", "Input Voltage :", Q_NULLPTR));
        BTN_ACTIVE_DUTYCONTROL_OFF->setText(QApplication::translate("PumpControlDialog", "Active Ctrl. \n"
"Off", Q_NULLPTR));
        CON_ACTIVE_DUTYCONTROL_INPUT_VOLTAGE->setText(QApplication::translate("PumpControlDialog", "60", Q_NULLPTR));
        label_11->setText(QApplication::translate("PumpControlDialog", "V", Q_NULLPTR));
        GB_JointSelection_11->setTitle(QApplication::translate("PumpControlDialog", "Linear MPC", Q_NULLPTR));
        BTN_PUMPCONTROL_MPC_ON->setText(QApplication::translate("PumpControlDialog", "Linear MPC \n"
"On", Q_NULLPTR));
        BTN_PUMPCONTROL_MPC_OFF->setText(QApplication::translate("PumpControlDialog", "Linear MPC \n"
"Off", Q_NULLPTR));
        GB_JointSelection_12->setTitle(QString());
        RBTN_PS_REF_SM->setText(QApplication::translate("PumpControlDialog", "Shared Mem.", Q_NULLPTR));
        label_32->setText(QApplication::translate("PumpControlDialog", "Pressure\n"
"Reference", Q_NULLPTR));
        RBTN_PS_REF_THISAL->setText(QApplication::translate("PumpControlDialog", "This AL", Q_NULLPTR));
        label_25->setText(QApplication::translate("PumpControlDialog", "Time : ", Q_NULLPTR));
        label_27->setText(QApplication::translate("PumpControlDialog", "sec", Q_NULLPTR));
        CON_LINEARCHANGE_TIME->setText(QApplication::translate("PumpControlDialog", "5.0", Q_NULLPTR));
        GB_JointSelection_13->setTitle(QString());
        RBTN_VARIABLE_PRES_MODE->setText(QApplication::translate("PumpControlDialog", "Variable", Q_NULLPTR));
        label_34->setText(QApplication::translate("PumpControlDialog", "Supply Pres.\n"
" at Ctrl. Board", Q_NULLPTR));
        RBTN_CONST_PRES_MODE->setText(QApplication::translate("PumpControlDialog", "Constant", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class PumpControlDialog: public Ui_PumpControlDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PUMPCONTROLDIALOG_H
