/********************************************************************************
** Form generated from reading UI file 'SH_TASK_Dialog.ui'
**
** Created by: Qt User Interface Compiler version 5.9.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SH_TASK_DIALOG_H
#define UI_SH_TASK_DIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_SH_TASK_Dialog
{
public:
    QGroupBox *GB_JointSelection_2;
    QWidget *layoutWidget;
    QGridLayout *gridLayout;
    QRadioButton *RB_RKN;
    QRadioButton *RB_RHR;
    QRadioButton *RB_LHP;
    QRadioButton *RB_RAR;
    QRadioButton *RB_RHY;
    QRadioButton *RB_LHR;
    QRadioButton *RB_LAP;
    QRadioButton *RB_LAR;
    QRadioButton *RB_RAP;
    QRadioButton *RB_LKN;
    QRadioButton *RB_LHY;
    QRadioButton *RB_RHP;
    QRadioButton *RB_WST;
    QGroupBox *GB_JointSelection_5;
    QLabel *label_50;
    QLabel *label_52;
    QLabel *label_53;
    QLabel *label_49;
    QLabel *label_39;
    QLabel *label_51;
    QLineEdit *LE_POSITION_CONTROL_SINEWAVE_NUM;
    QLabel *label_41;
    QPushButton *BTN_GOTO_RANDOM_POSITION;
    QLineEdit *LE_POSITION_CONTROL_SINEWAVE_MAG;
    QLineEdit *LE_GOTO_POSITION_POSITION;
    QLineEdit *LE_POSITION_CONTROL_SINEWAVE_PERIOD;
    QPushButton *BTN_POSITION_CONTROL_SINEWAVE_GO;
    QLineEdit *LE_GOTO_POSITION_TIME;
    QPushButton *BTN_LIGHT_GOTO_POSITION;
    QLabel *label_40;
    QLineEdit *LE_GOTO_RANDOM_POSITION_TIME;
    QGroupBox *GB_JointSelection_6;
    QLabel *label_42;
    QPushButton *BTN_TORQUE_CONTROL_STOP;
    QPushButton *BTN_TORQUE_CONTROL_GO;
    QLineEdit *LE_REF_TORQUE;
    QLabel *label_66;
    QGroupBox *GB_JointSelection_7;
    QLabel *label_33;
    QLabel *label_36;
    QPushButton *BTN_OPENLOOP_SINE_GO;
    QLineEdit *LE_OPENLOOP_SINE_NUM;
    QLabel *label_34;
    QLabel *label_35;
    QPushButton *BTN_LIGHT_FINDHOME_MINUS_END;
    QLineEdit *LE_OPENLOOP_SINE_TIME;
    QPushButton *BTN_LIGHT_FINDHOME_PLUS_END;
    QLineEdit *LE_OPENLOOP_SINE_MAG;
    QLineEdit *LE_CONST_OPEN_VALUE;
    QPushButton *BTN_CONST_OPEN_NEG;
    QPushButton *BTN_CONST_OPEN_POS;
    QLabel *label_37;
    QGroupBox *GB_JointSelection_8;
    QLabel *label_57;
    QLineEdit *EDIT_LIGHT_VALVEID_OPEN_RESOL;
    QLineEdit *EDIT_LIGHT_VALVEID_OPEN_MIN;
    QLabel *label_48;
    QLabel *label_55;
    QLineEdit *EDIT_LIGHT_VALVEID_OPEN_MAX;
    QPushButton *BTN_LIGHT_ANKLE_VALVE_IDENTIFICATION;
    QPushButton *BTN_LIGHT_ANKLE_VALVE_IDENTIFICATION_NEG;
    QLineEdit *EDIT_LIGHT_VALVEID_PRES_MIN;
    QLineEdit *EDIT_LIGHT_VALVEID_PRES_RESOL;
    QLineEdit *EDIT_LIGHT_VALVEID_PRES_MAX;
    QLabel *label_60;
    QLabel *label_54;
    QLabel *label_56;
    QGroupBox *GB_JointSelection_9;
    QPushButton *BTN_SAVE_START;
    QPushButton *BTN_SAVE_STOP;
    QLineEdit *CON_SAVE_FILENAME;
    QLabel *label_59;
    QGroupBox *groupBox_4;
    QPushButton *BTN_ROBOT_CONTROLMETHOD_CHANGE_TO_POS;
    QCheckBox *checkBox;
    QCheckBox *checkBox_2;
    QCheckBox *CBX_ROBOT_CONTROLMETHOD_RHP;
    QCheckBox *CBX_ROBOT_CONTROLMETHOD_RKN;
    QCheckBox *CBX_ROBOT_CONTROLMETHOD_RANK;
    QCheckBox *checkBox_6;
    QCheckBox *CBX_ROBOT_CONTROLMETHOD_LANK;
    QCheckBox *CBX_ROBOT_CONTROLMETHOD_LHP;
    QCheckBox *checkBox_9;
    QCheckBox *CBX_ROBOT_CONTROLMETHOD_LKN;
    QPushButton *BTN_ROBOT_CONTROLMETHOD_CHANGE;
    QPushButton *BTN_ROBOT_CONTROLMETHOD_CHANGE_TO_TOR;
    QLabel *label_43;
    QGroupBox *GB_JointSelection_10;
    QLabel *label_44;
    QPushButton *LE_VARIABLE_SUPPLY_SWING_STOP;
    QPushButton *LE_VARIABLE_SUPPLY_SWING_GO;
    QLineEdit *LE_VARIABLE_SUPPLY_SWING_MASS;
    QLabel *label_67;
    QLabel *label_45;
    QLabel *label_68;
    QLineEdit *LE_VARIABLE_SUPPLY_SWING_MAG;
    QLabel *label_46;
    QLineEdit *LE_VARIABLE_SUPPLY_SWING_NUM;
    QLabel *label_47;
    QLabel *label_70;
    QLineEdit *LE_VARIABLE_SUPPLY_SWING_PER;
    QLabel *label_58;
    QLabel *label_71;

    void setupUi(QWidget *SH_TASK_Dialog)
    {
        if (SH_TASK_Dialog->objectName().isEmpty())
            SH_TASK_Dialog->setObjectName(QStringLiteral("SH_TASK_Dialog"));
        SH_TASK_Dialog->resize(780, 780);
        GB_JointSelection_2 = new QGroupBox(SH_TASK_Dialog);
        GB_JointSelection_2->setObjectName(QStringLiteral("GB_JointSelection_2"));
        GB_JointSelection_2->setGeometry(QRect(19, 20, 151, 201));
        QFont font;
        font.setBold(true);
        font.setWeight(75);
        GB_JointSelection_2->setFont(font);
        GB_JointSelection_2->setStyleSheet(QLatin1String("QGroupBox {\n"
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
        layoutWidget = new QWidget(GB_JointSelection_2);
        layoutWidget->setObjectName(QStringLiteral("layoutWidget"));
        layoutWidget->setGeometry(QRect(20, 17, 111, 178));
        QFont font1;
        font1.setPointSize(9);
        layoutWidget->setFont(font1);
        gridLayout = new QGridLayout(layoutWidget);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        RB_RKN = new QRadioButton(layoutWidget);
        RB_RKN->setObjectName(QStringLiteral("RB_RKN"));
        RB_RKN->setFont(font1);

        gridLayout->addWidget(RB_RKN, 3, 0, 1, 1);

        RB_RHR = new QRadioButton(layoutWidget);
        RB_RHR->setObjectName(QStringLiteral("RB_RHR"));
        RB_RHR->setFont(font1);
        RB_RHR->setChecked(true);

        gridLayout->addWidget(RB_RHR, 0, 0, 1, 1);

        RB_LHP = new QRadioButton(layoutWidget);
        RB_LHP->setObjectName(QStringLiteral("RB_LHP"));
        RB_LHP->setFont(font1);

        gridLayout->addWidget(RB_LHP, 2, 1, 1, 1);

        RB_RAR = new QRadioButton(layoutWidget);
        RB_RAR->setObjectName(QStringLiteral("RB_RAR"));
        RB_RAR->setFont(font1);

        gridLayout->addWidget(RB_RAR, 5, 0, 1, 1);

        RB_RHY = new QRadioButton(layoutWidget);
        RB_RHY->setObjectName(QStringLiteral("RB_RHY"));
        RB_RHY->setFont(font1);
        RB_RHY->setChecked(false);

        gridLayout->addWidget(RB_RHY, 1, 0, 1, 1);

        RB_LHR = new QRadioButton(layoutWidget);
        RB_LHR->setObjectName(QStringLiteral("RB_LHR"));
        RB_LHR->setFont(font1);
        RB_LHR->setChecked(false);

        gridLayout->addWidget(RB_LHR, 0, 1, 1, 1);

        RB_LAP = new QRadioButton(layoutWidget);
        RB_LAP->setObjectName(QStringLiteral("RB_LAP"));
        RB_LAP->setFont(font1);

        gridLayout->addWidget(RB_LAP, 4, 1, 1, 1);

        RB_LAR = new QRadioButton(layoutWidget);
        RB_LAR->setObjectName(QStringLiteral("RB_LAR"));
        RB_LAR->setFont(font1);

        gridLayout->addWidget(RB_LAR, 5, 1, 1, 1);

        RB_RAP = new QRadioButton(layoutWidget);
        RB_RAP->setObjectName(QStringLiteral("RB_RAP"));
        RB_RAP->setFont(font1);

        gridLayout->addWidget(RB_RAP, 4, 0, 1, 1);

        RB_LKN = new QRadioButton(layoutWidget);
        RB_LKN->setObjectName(QStringLiteral("RB_LKN"));
        RB_LKN->setFont(font1);

        gridLayout->addWidget(RB_LKN, 3, 1, 1, 1);

        RB_LHY = new QRadioButton(layoutWidget);
        RB_LHY->setObjectName(QStringLiteral("RB_LHY"));
        RB_LHY->setFont(font1);

        gridLayout->addWidget(RB_LHY, 1, 1, 1, 1);

        RB_RHP = new QRadioButton(layoutWidget);
        RB_RHP->setObjectName(QStringLiteral("RB_RHP"));
        RB_RHP->setFont(font1);

        gridLayout->addWidget(RB_RHP, 2, 0, 1, 1);

        RB_WST = new QRadioButton(layoutWidget);
        RB_WST->setObjectName(QStringLiteral("RB_WST"));
        RB_WST->setFont(font1);

        gridLayout->addWidget(RB_WST, 6, 0, 1, 1);

        GB_JointSelection_5 = new QGroupBox(SH_TASK_Dialog);
        GB_JointSelection_5->setObjectName(QStringLiteral("GB_JointSelection_5"));
        GB_JointSelection_5->setGeometry(QRect(250, 358, 211, 361));
        GB_JointSelection_5->setFont(font);
        GB_JointSelection_5->setStyleSheet(QLatin1String("QGroupBox {\n"
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
        label_50 = new QLabel(GB_JointSelection_5);
        label_50->setObjectName(QStringLiteral("label_50"));
        label_50->setGeometry(QRect(178, 230, 21, 21));
        label_50->setAlignment(Qt::AlignCenter);
        label_52 = new QLabel(GB_JointSelection_5);
        label_52->setObjectName(QStringLiteral("label_52"));
        label_52->setGeometry(QRect(21, 276, 91, 21));
        label_52->setAlignment(Qt::AlignCenter);
        label_53 = new QLabel(GB_JointSelection_5);
        label_53->setObjectName(QStringLiteral("label_53"));
        label_53->setGeometry(QRect(21, 253, 91, 21));
        label_53->setAlignment(Qt::AlignCenter);
        label_49 = new QLabel(GB_JointSelection_5);
        label_49->setObjectName(QStringLiteral("label_49"));
        label_49->setGeometry(QRect(12, 231, 101, 21));
        label_49->setAlignment(Qt::AlignCenter);
        label_39 = new QLabel(GB_JointSelection_5);
        label_39->setObjectName(QStringLiteral("label_39"));
        label_39->setGeometry(QRect(21, 25, 91, 20));
        label_39->setAlignment(Qt::AlignCenter);
        label_51 = new QLabel(GB_JointSelection_5);
        label_51->setObjectName(QStringLiteral("label_51"));
        label_51->setGeometry(QRect(173, 253, 31, 21));
        label_51->setAlignment(Qt::AlignCenter);
        LE_POSITION_CONTROL_SINEWAVE_NUM = new QLineEdit(GB_JointSelection_5);
        LE_POSITION_CONTROL_SINEWAVE_NUM->setObjectName(QStringLiteral("LE_POSITION_CONTROL_SINEWAVE_NUM"));
        LE_POSITION_CONTROL_SINEWAVE_NUM->setGeometry(QRect(121, 277, 51, 20));
        QFont font2;
        font2.setPointSize(7);
        LE_POSITION_CONTROL_SINEWAVE_NUM->setFont(font2);
        LE_POSITION_CONTROL_SINEWAVE_NUM->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label_41 = new QLabel(GB_JointSelection_5);
        label_41->setObjectName(QStringLiteral("label_41"));
        label_41->setGeometry(QRect(32, 149, 71, 20));
        label_41->setAlignment(Qt::AlignCenter);
        BTN_GOTO_RANDOM_POSITION = new QPushButton(GB_JointSelection_5);
        BTN_GOTO_RANDOM_POSITION->setObjectName(QStringLiteral("BTN_GOTO_RANDOM_POSITION"));
        BTN_GOTO_RANDOM_POSITION->setGeometry(QRect(28, 181, 151, 31));
        LE_POSITION_CONTROL_SINEWAVE_MAG = new QLineEdit(GB_JointSelection_5);
        LE_POSITION_CONTROL_SINEWAVE_MAG->setObjectName(QStringLiteral("LE_POSITION_CONTROL_SINEWAVE_MAG"));
        LE_POSITION_CONTROL_SINEWAVE_MAG->setGeometry(QRect(121, 254, 51, 20));
        LE_POSITION_CONTROL_SINEWAVE_MAG->setFont(font2);
        LE_POSITION_CONTROL_SINEWAVE_MAG->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_GOTO_POSITION_POSITION = new QLineEdit(GB_JointSelection_5);
        LE_GOTO_POSITION_POSITION->setObjectName(QStringLiteral("LE_GOTO_POSITION_POSITION"));
        LE_GOTO_POSITION_POSITION->setGeometry(QRect(37, 49, 61, 31));
        LE_GOTO_POSITION_POSITION->setFont(font2);
        LE_POSITION_CONTROL_SINEWAVE_PERIOD = new QLineEdit(GB_JointSelection_5);
        LE_POSITION_CONTROL_SINEWAVE_PERIOD->setObjectName(QStringLiteral("LE_POSITION_CONTROL_SINEWAVE_PERIOD"));
        LE_POSITION_CONTROL_SINEWAVE_PERIOD->setGeometry(QRect(121, 231, 51, 20));
        LE_POSITION_CONTROL_SINEWAVE_PERIOD->setFont(font2);
        LE_POSITION_CONTROL_SINEWAVE_PERIOD->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        BTN_POSITION_CONTROL_SINEWAVE_GO = new QPushButton(GB_JointSelection_5);
        BTN_POSITION_CONTROL_SINEWAVE_GO->setObjectName(QStringLiteral("BTN_POSITION_CONTROL_SINEWAVE_GO"));
        BTN_POSITION_CONTROL_SINEWAVE_GO->setGeometry(QRect(30, 303, 151, 41));
        LE_GOTO_POSITION_TIME = new QLineEdit(GB_JointSelection_5);
        LE_GOTO_POSITION_TIME->setObjectName(QStringLiteral("LE_GOTO_POSITION_TIME"));
        LE_GOTO_POSITION_TIME->setGeometry(QRect(114, 49, 61, 31));
        LE_GOTO_POSITION_TIME->setFont(font2);
        BTN_LIGHT_GOTO_POSITION = new QPushButton(GB_JointSelection_5);
        BTN_LIGHT_GOTO_POSITION->setObjectName(QStringLiteral("BTN_LIGHT_GOTO_POSITION"));
        BTN_LIGHT_GOTO_POSITION->setGeometry(QRect(30, 89, 151, 31));
        label_40 = new QLabel(GB_JointSelection_5);
        label_40->setObjectName(QStringLiteral("label_40"));
        label_40->setGeometry(QRect(110, 25, 71, 20));
        label_40->setAlignment(Qt::AlignCenter);
        LE_GOTO_RANDOM_POSITION_TIME = new QLineEdit(GB_JointSelection_5);
        LE_GOTO_RANDOM_POSITION_TIME->setObjectName(QStringLiteral("LE_GOTO_RANDOM_POSITION_TIME"));
        LE_GOTO_RANDOM_POSITION_TIME->setGeometry(QRect(112, 143, 61, 31));
        LE_GOTO_RANDOM_POSITION_TIME->setFont(font2);
        GB_JointSelection_6 = new QGroupBox(SH_TASK_Dialog);
        GB_JointSelection_6->setObjectName(QStringLiteral("GB_JointSelection_6"));
        GB_JointSelection_6->setGeometry(QRect(480, 358, 211, 141));
        GB_JointSelection_6->setFont(font);
        GB_JointSelection_6->setStyleSheet(QLatin1String("QGroupBox {\n"
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
        label_42 = new QLabel(GB_JointSelection_6);
        label_42->setObjectName(QStringLiteral("label_42"));
        label_42->setGeometry(QRect(10, 30, 111, 21));
        label_42->setAlignment(Qt::AlignCenter);
        BTN_TORQUE_CONTROL_STOP = new QPushButton(GB_JointSelection_6);
        BTN_TORQUE_CONTROL_STOP->setObjectName(QStringLiteral("BTN_TORQUE_CONTROL_STOP"));
        BTN_TORQUE_CONTROL_STOP->setGeometry(QRect(20, 96, 151, 31));
        BTN_TORQUE_CONTROL_GO = new QPushButton(GB_JointSelection_6);
        BTN_TORQUE_CONTROL_GO->setObjectName(QStringLiteral("BTN_TORQUE_CONTROL_GO"));
        BTN_TORQUE_CONTROL_GO->setGeometry(QRect(20, 60, 151, 31));
        LE_REF_TORQUE = new QLineEdit(GB_JointSelection_6);
        LE_REF_TORQUE->setObjectName(QStringLiteral("LE_REF_TORQUE"));
        LE_REF_TORQUE->setGeometry(QRect(129, 30, 51, 20));
        LE_REF_TORQUE->setFont(font2);
        LE_REF_TORQUE->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label_66 = new QLabel(GB_JointSelection_6);
        label_66->setObjectName(QStringLiteral("label_66"));
        label_66->setGeometry(QRect(180, 30, 21, 21));
        label_66->setAlignment(Qt::AlignCenter);
        GB_JointSelection_7 = new QGroupBox(SH_TASK_Dialog);
        GB_JointSelection_7->setObjectName(QStringLiteral("GB_JointSelection_7"));
        GB_JointSelection_7->setGeometry(QRect(19, 358, 211, 361));
        GB_JointSelection_7->setFont(font);
        GB_JointSelection_7->setStyleSheet(QLatin1String("QGroupBox {\n"
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
        label_33 = new QLabel(GB_JointSelection_7);
        label_33->setObjectName(QStringLiteral("label_33"));
        label_33->setGeometry(QRect(29, 218, 91, 21));
        label_33->setAlignment(Qt::AlignCenter);
        label_36 = new QLabel(GB_JointSelection_7);
        label_36->setObjectName(QStringLiteral("label_36"));
        label_36->setGeometry(QRect(171, 217, 21, 21));
        label_36->setAlignment(Qt::AlignCenter);
        BTN_OPENLOOP_SINE_GO = new QPushButton(GB_JointSelection_7);
        BTN_OPENLOOP_SINE_GO->setObjectName(QStringLiteral("BTN_OPENLOOP_SINE_GO"));
        BTN_OPENLOOP_SINE_GO->setGeometry(QRect(28, 290, 151, 41));
        LE_OPENLOOP_SINE_NUM = new QLineEdit(GB_JointSelection_7);
        LE_OPENLOOP_SINE_NUM->setObjectName(QStringLiteral("LE_OPENLOOP_SINE_NUM"));
        LE_OPENLOOP_SINE_NUM->setGeometry(QRect(119, 264, 51, 20));
        LE_OPENLOOP_SINE_NUM->setFont(font2);
        LE_OPENLOOP_SINE_NUM->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label_34 = new QLabel(GB_JointSelection_7);
        label_34->setObjectName(QStringLiteral("label_34"));
        label_34->setGeometry(QRect(28, 240, 91, 21));
        label_34->setAlignment(Qt::AlignCenter);
        label_35 = new QLabel(GB_JointSelection_7);
        label_35->setObjectName(QStringLiteral("label_35"));
        label_35->setGeometry(QRect(28, 263, 91, 21));
        label_35->setAlignment(Qt::AlignCenter);
        BTN_LIGHT_FINDHOME_MINUS_END = new QPushButton(GB_JointSelection_7);
        BTN_LIGHT_FINDHOME_MINUS_END->setObjectName(QStringLiteral("BTN_LIGHT_FINDHOME_MINUS_END"));
        BTN_LIGHT_FINDHOME_MINUS_END->setEnabled(false);
        BTN_LIGHT_FINDHOME_MINUS_END->setGeometry(QRect(25, 157, 81, 41));
        LE_OPENLOOP_SINE_TIME = new QLineEdit(GB_JointSelection_7);
        LE_OPENLOOP_SINE_TIME->setObjectName(QStringLiteral("LE_OPENLOOP_SINE_TIME"));
        LE_OPENLOOP_SINE_TIME->setGeometry(QRect(119, 218, 51, 20));
        LE_OPENLOOP_SINE_TIME->setFont(font2);
        LE_OPENLOOP_SINE_TIME->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        BTN_LIGHT_FINDHOME_PLUS_END = new QPushButton(GB_JointSelection_7);
        BTN_LIGHT_FINDHOME_PLUS_END->setObjectName(QStringLiteral("BTN_LIGHT_FINDHOME_PLUS_END"));
        BTN_LIGHT_FINDHOME_PLUS_END->setEnabled(false);
        BTN_LIGHT_FINDHOME_PLUS_END->setGeometry(QRect(107, 157, 81, 41));
        LE_OPENLOOP_SINE_MAG = new QLineEdit(GB_JointSelection_7);
        LE_OPENLOOP_SINE_MAG->setObjectName(QStringLiteral("LE_OPENLOOP_SINE_MAG"));
        LE_OPENLOOP_SINE_MAG->setGeometry(QRect(119, 241, 51, 20));
        LE_OPENLOOP_SINE_MAG->setFont(font2);
        LE_OPENLOOP_SINE_MAG->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_CONST_OPEN_VALUE = new QLineEdit(GB_JointSelection_7);
        LE_CONST_OPEN_VALUE->setObjectName(QStringLiteral("LE_CONST_OPEN_VALUE"));
        LE_CONST_OPEN_VALUE->setGeometry(QRect(37, 69, 51, 31));
        LE_CONST_OPEN_VALUE->setFont(font2);
        BTN_CONST_OPEN_NEG = new QPushButton(GB_JointSelection_7);
        BTN_CONST_OPEN_NEG->setObjectName(QStringLiteral("BTN_CONST_OPEN_NEG"));
        BTN_CONST_OPEN_NEG->setEnabled(true);
        BTN_CONST_OPEN_NEG->setGeometry(QRect(100, 100, 81, 31));
        BTN_CONST_OPEN_POS = new QPushButton(GB_JointSelection_7);
        BTN_CONST_OPEN_POS->setObjectName(QStringLiteral("BTN_CONST_OPEN_POS"));
        BTN_CONST_OPEN_POS->setEnabled(true);
        BTN_CONST_OPEN_POS->setGeometry(QRect(100, 69, 81, 31));
        label_37 = new QLabel(GB_JointSelection_7);
        label_37->setObjectName(QStringLiteral("label_37"));
        label_37->setGeometry(QRect(-6, 20, 221, 51));
        label_37->setAlignment(Qt::AlignCenter);
        GB_JointSelection_8 = new QGroupBox(SH_TASK_Dialog);
        GB_JointSelection_8->setObjectName(QStringLiteral("GB_JointSelection_8"));
        GB_JointSelection_8->setGeometry(QRect(20, 228, 461, 121));
        GB_JointSelection_8->setFont(font);
        GB_JointSelection_8->setStyleSheet(QLatin1String("QGroupBox {\n"
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
        label_57 = new QLabel(GB_JointSelection_8);
        label_57->setObjectName(QStringLiteral("label_57"));
        label_57->setGeometry(QRect(0, 27, 71, 21));
        label_57->setFont(font2);
        label_57->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        EDIT_LIGHT_VALVEID_OPEN_RESOL = new QLineEdit(GB_JointSelection_8);
        EDIT_LIGHT_VALVEID_OPEN_RESOL->setObjectName(QStringLiteral("EDIT_LIGHT_VALVEID_OPEN_RESOL"));
        EDIT_LIGHT_VALVEID_OPEN_RESOL->setGeometry(QRect(80, 51, 51, 20));
        EDIT_LIGHT_VALVEID_OPEN_RESOL->setFont(font2);
        EDIT_LIGHT_VALVEID_OPEN_RESOL->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        EDIT_LIGHT_VALVEID_OPEN_MIN = new QLineEdit(GB_JointSelection_8);
        EDIT_LIGHT_VALVEID_OPEN_MIN->setObjectName(QStringLiteral("EDIT_LIGHT_VALVEID_OPEN_MIN"));
        EDIT_LIGHT_VALVEID_OPEN_MIN->setGeometry(QRect(80, 28, 51, 20));
        EDIT_LIGHT_VALVEID_OPEN_MIN->setFont(font2);
        EDIT_LIGHT_VALVEID_OPEN_MIN->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label_48 = new QLabel(GB_JointSelection_8);
        label_48->setObjectName(QStringLiteral("label_48"));
        label_48->setGeometry(QRect(20, 74, 51, 21));
        label_48->setFont(font2);
        label_48->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label_55 = new QLabel(GB_JointSelection_8);
        label_55->setObjectName(QStringLiteral("label_55"));
        label_55->setGeometry(QRect(9, 50, 61, 21));
        label_55->setFont(font2);
        label_55->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        EDIT_LIGHT_VALVEID_OPEN_MAX = new QLineEdit(GB_JointSelection_8);
        EDIT_LIGHT_VALVEID_OPEN_MAX->setObjectName(QStringLiteral("EDIT_LIGHT_VALVEID_OPEN_MAX"));
        EDIT_LIGHT_VALVEID_OPEN_MAX->setGeometry(QRect(80, 74, 51, 20));
        EDIT_LIGHT_VALVEID_OPEN_MAX->setFont(font2);
        EDIT_LIGHT_VALVEID_OPEN_MAX->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        BTN_LIGHT_ANKLE_VALVE_IDENTIFICATION = new QPushButton(GB_JointSelection_8);
        BTN_LIGHT_ANKLE_VALVE_IDENTIFICATION->setObjectName(QStringLiteral("BTN_LIGHT_ANKLE_VALVE_IDENTIFICATION"));
        BTN_LIGHT_ANKLE_VALVE_IDENTIFICATION->setEnabled(true);
        BTN_LIGHT_ANKLE_VALVE_IDENTIFICATION->setGeometry(QRect(347, 20, 101, 41));
        BTN_LIGHT_ANKLE_VALVE_IDENTIFICATION_NEG = new QPushButton(GB_JointSelection_8);
        BTN_LIGHT_ANKLE_VALVE_IDENTIFICATION_NEG->setObjectName(QStringLiteral("BTN_LIGHT_ANKLE_VALVE_IDENTIFICATION_NEG"));
        BTN_LIGHT_ANKLE_VALVE_IDENTIFICATION_NEG->setEnabled(true);
        BTN_LIGHT_ANKLE_VALVE_IDENTIFICATION_NEG->setGeometry(QRect(347, 70, 101, 41));
        EDIT_LIGHT_VALVEID_PRES_MIN = new QLineEdit(GB_JointSelection_8);
        EDIT_LIGHT_VALVEID_PRES_MIN->setObjectName(QStringLiteral("EDIT_LIGHT_VALVEID_PRES_MIN"));
        EDIT_LIGHT_VALVEID_PRES_MIN->setGeometry(QRect(211, 28, 51, 20));
        EDIT_LIGHT_VALVEID_PRES_MIN->setFont(font2);
        EDIT_LIGHT_VALVEID_PRES_MIN->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        EDIT_LIGHT_VALVEID_PRES_RESOL = new QLineEdit(GB_JointSelection_8);
        EDIT_LIGHT_VALVEID_PRES_RESOL->setObjectName(QStringLiteral("EDIT_LIGHT_VALVEID_PRES_RESOL"));
        EDIT_LIGHT_VALVEID_PRES_RESOL->setGeometry(QRect(211, 51, 51, 20));
        EDIT_LIGHT_VALVEID_PRES_RESOL->setFont(font2);
        EDIT_LIGHT_VALVEID_PRES_RESOL->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        EDIT_LIGHT_VALVEID_PRES_MAX = new QLineEdit(GB_JointSelection_8);
        EDIT_LIGHT_VALVEID_PRES_MAX->setObjectName(QStringLiteral("EDIT_LIGHT_VALVEID_PRES_MAX"));
        EDIT_LIGHT_VALVEID_PRES_MAX->setGeometry(QRect(211, 74, 51, 20));
        EDIT_LIGHT_VALVEID_PRES_MAX->setFont(font2);
        EDIT_LIGHT_VALVEID_PRES_MAX->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label_60 = new QLabel(GB_JointSelection_8);
        label_60->setObjectName(QStringLiteral("label_60"));
        label_60->setGeometry(QRect(131, 27, 71, 21));
        label_60->setFont(font2);
        label_60->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label_54 = new QLabel(GB_JointSelection_8);
        label_54->setObjectName(QStringLiteral("label_54"));
        label_54->setGeometry(QRect(151, 74, 51, 21));
        label_54->setFont(font2);
        label_54->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label_56 = new QLabel(GB_JointSelection_8);
        label_56->setObjectName(QStringLiteral("label_56"));
        label_56->setGeometry(QRect(140, 50, 61, 21));
        label_56->setFont(font2);
        label_56->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        GB_JointSelection_9 = new QGroupBox(SH_TASK_Dialog);
        GB_JointSelection_9->setObjectName(QStringLiteral("GB_JointSelection_9"));
        GB_JointSelection_9->setGeometry(QRect(490, 228, 201, 121));
        GB_JointSelection_9->setFont(font);
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
        BTN_SAVE_START->setGeometry(QRect(16, 25, 81, 41));
        BTN_SAVE_STOP = new QPushButton(GB_JointSelection_9);
        BTN_SAVE_STOP->setObjectName(QStringLiteral("BTN_SAVE_STOP"));
        BTN_SAVE_STOP->setGeometry(QRect(106, 25, 81, 41));
        CON_SAVE_FILENAME = new QLineEdit(GB_JointSelection_9);
        CON_SAVE_FILENAME->setObjectName(QStringLiteral("CON_SAVE_FILENAME"));
        CON_SAVE_FILENAME->setGeometry(QRect(69, 74, 121, 31));
        QFont font3;
        font3.setPointSize(8);
        CON_SAVE_FILENAME->setFont(font3);
        CON_SAVE_FILENAME->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label_59 = new QLabel(GB_JointSelection_9);
        label_59->setObjectName(QStringLiteral("label_59"));
        label_59->setGeometry(QRect(6, 80, 61, 21));
        label_59->setFont(font3);
        label_59->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        groupBox_4 = new QGroupBox(SH_TASK_Dialog);
        groupBox_4->setObjectName(QStringLiteral("groupBox_4"));
        groupBox_4->setGeometry(QRect(200, 32, 481, 151));
        groupBox_4->setAlignment(Qt::AlignCenter);
        BTN_ROBOT_CONTROLMETHOD_CHANGE_TO_POS = new QPushButton(groupBox_4);
        BTN_ROBOT_CONTROLMETHOD_CHANGE_TO_POS->setObjectName(QStringLiteral("BTN_ROBOT_CONTROLMETHOD_CHANGE_TO_POS"));
        BTN_ROBOT_CONTROLMETHOD_CHANGE_TO_POS->setGeometry(QRect(30, 110, 201, 31));
        checkBox = new QCheckBox(groupBox_4);
        checkBox->setObjectName(QStringLiteral("checkBox"));
        checkBox->setEnabled(false);
        checkBox->setGeometry(QRect(20, 30, 51, 20));
        checkBox_2 = new QCheckBox(groupBox_4);
        checkBox_2->setObjectName(QStringLiteral("checkBox_2"));
        checkBox_2->setEnabled(false);
        checkBox_2->setGeometry(QRect(80, 30, 51, 20));
        CBX_ROBOT_CONTROLMETHOD_RHP = new QCheckBox(groupBox_4);
        CBX_ROBOT_CONTROLMETHOD_RHP->setObjectName(QStringLiteral("CBX_ROBOT_CONTROLMETHOD_RHP"));
        CBX_ROBOT_CONTROLMETHOD_RHP->setGeometry(QRect(140, 30, 51, 20));
        CBX_ROBOT_CONTROLMETHOD_RKN = new QCheckBox(groupBox_4);
        CBX_ROBOT_CONTROLMETHOD_RKN->setObjectName(QStringLiteral("CBX_ROBOT_CONTROLMETHOD_RKN"));
        CBX_ROBOT_CONTROLMETHOD_RKN->setGeometry(QRect(200, 30, 51, 20));
        CBX_ROBOT_CONTROLMETHOD_RANK = new QCheckBox(groupBox_4);
        CBX_ROBOT_CONTROLMETHOD_RANK->setObjectName(QStringLiteral("CBX_ROBOT_CONTROLMETHOD_RANK"));
        CBX_ROBOT_CONTROLMETHOD_RANK->setGeometry(QRect(260, 30, 91, 20));
        checkBox_6 = new QCheckBox(groupBox_4);
        checkBox_6->setObjectName(QStringLiteral("checkBox_6"));
        checkBox_6->setEnabled(false);
        checkBox_6->setGeometry(QRect(20, 60, 51, 20));
        CBX_ROBOT_CONTROLMETHOD_LANK = new QCheckBox(groupBox_4);
        CBX_ROBOT_CONTROLMETHOD_LANK->setObjectName(QStringLiteral("CBX_ROBOT_CONTROLMETHOD_LANK"));
        CBX_ROBOT_CONTROLMETHOD_LANK->setGeometry(QRect(260, 60, 91, 20));
        CBX_ROBOT_CONTROLMETHOD_LHP = new QCheckBox(groupBox_4);
        CBX_ROBOT_CONTROLMETHOD_LHP->setObjectName(QStringLiteral("CBX_ROBOT_CONTROLMETHOD_LHP"));
        CBX_ROBOT_CONTROLMETHOD_LHP->setGeometry(QRect(140, 60, 51, 20));
        checkBox_9 = new QCheckBox(groupBox_4);
        checkBox_9->setObjectName(QStringLiteral("checkBox_9"));
        checkBox_9->setEnabled(false);
        checkBox_9->setGeometry(QRect(80, 60, 51, 20));
        CBX_ROBOT_CONTROLMETHOD_LKN = new QCheckBox(groupBox_4);
        CBX_ROBOT_CONTROLMETHOD_LKN->setObjectName(QStringLiteral("CBX_ROBOT_CONTROLMETHOD_LKN"));
        CBX_ROBOT_CONTROLMETHOD_LKN->setGeometry(QRect(200, 60, 51, 20));
        BTN_ROBOT_CONTROLMETHOD_CHANGE = new QPushButton(groupBox_4);
        BTN_ROBOT_CONTROLMETHOD_CHANGE->setObjectName(QStringLiteral("BTN_ROBOT_CONTROLMETHOD_CHANGE"));
        BTN_ROBOT_CONTROLMETHOD_CHANGE->setGeometry(QRect(365, 35, 101, 41));
        BTN_ROBOT_CONTROLMETHOD_CHANGE_TO_TOR = new QPushButton(groupBox_4);
        BTN_ROBOT_CONTROLMETHOD_CHANGE_TO_TOR->setObjectName(QStringLiteral("BTN_ROBOT_CONTROLMETHOD_CHANGE_TO_TOR"));
        BTN_ROBOT_CONTROLMETHOD_CHANGE_TO_TOR->setGeometry(QRect(250, 110, 201, 31));
        label_43 = new QLabel(groupBox_4);
        label_43->setObjectName(QStringLiteral("label_43"));
        label_43->setGeometry(QRect(50, 86, 381, 16));
        label_43->setAlignment(Qt::AlignCenter);
        GB_JointSelection_10 = new QGroupBox(SH_TASK_Dialog);
        GB_JointSelection_10->setObjectName(QStringLiteral("GB_JointSelection_10"));
        GB_JointSelection_10->setGeometry(QRect(480, 508, 211, 211));
        QFont font4;
        font4.setPointSize(8);
        font4.setBold(true);
        font4.setWeight(75);
        GB_JointSelection_10->setFont(font4);
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
        label_44 = new QLabel(GB_JointSelection_10);
        label_44->setObjectName(QStringLiteral("label_44"));
        label_44->setGeometry(QRect(40, 30, 71, 21));
        label_44->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_VARIABLE_SUPPLY_SWING_STOP = new QPushButton(GB_JointSelection_10);
        LE_VARIABLE_SUPPLY_SWING_STOP->setObjectName(QStringLiteral("LE_VARIABLE_SUPPLY_SWING_STOP"));
        LE_VARIABLE_SUPPLY_SWING_STOP->setEnabled(false);
        LE_VARIABLE_SUPPLY_SWING_STOP->setGeometry(QRect(110, 160, 71, 31));
        LE_VARIABLE_SUPPLY_SWING_GO = new QPushButton(GB_JointSelection_10);
        LE_VARIABLE_SUPPLY_SWING_GO->setObjectName(QStringLiteral("LE_VARIABLE_SUPPLY_SWING_GO"));
        LE_VARIABLE_SUPPLY_SWING_GO->setEnabled(false);
        LE_VARIABLE_SUPPLY_SWING_GO->setGeometry(QRect(30, 160, 71, 31));
        LE_VARIABLE_SUPPLY_SWING_MASS = new QLineEdit(GB_JointSelection_10);
        LE_VARIABLE_SUPPLY_SWING_MASS->setObjectName(QStringLiteral("LE_VARIABLE_SUPPLY_SWING_MASS"));
        LE_VARIABLE_SUPPLY_SWING_MASS->setGeometry(QRect(120, 30, 51, 20));
        LE_VARIABLE_SUPPLY_SWING_MASS->setFont(font2);
        LE_VARIABLE_SUPPLY_SWING_MASS->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label_67 = new QLabel(GB_JointSelection_10);
        label_67->setObjectName(QStringLiteral("label_67"));
        label_67->setGeometry(QRect(180, 30, 21, 21));
        label_67->setAlignment(Qt::AlignCenter);
        label_45 = new QLabel(GB_JointSelection_10);
        label_45->setObjectName(QStringLiteral("label_45"));
        label_45->setGeometry(QRect(11, 68, 111, 21));
        label_45->setAlignment(Qt::AlignCenter);
        label_68 = new QLabel(GB_JointSelection_10);
        label_68->setObjectName(QStringLiteral("label_68"));
        label_68->setGeometry(QRect(177, 60, 21, 21));
        label_68->setAlignment(Qt::AlignCenter);
        LE_VARIABLE_SUPPLY_SWING_MAG = new QLineEdit(GB_JointSelection_10);
        LE_VARIABLE_SUPPLY_SWING_MAG->setObjectName(QStringLiteral("LE_VARIABLE_SUPPLY_SWING_MAG"));
        LE_VARIABLE_SUPPLY_SWING_MAG->setGeometry(QRect(120, 60, 51, 20));
        LE_VARIABLE_SUPPLY_SWING_MAG->setFont(font2);
        LE_VARIABLE_SUPPLY_SWING_MAG->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label_46 = new QLabel(GB_JointSelection_10);
        label_46->setObjectName(QStringLiteral("label_46"));
        label_46->setGeometry(QRect(0, 58, 111, 21));
        label_46->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        LE_VARIABLE_SUPPLY_SWING_NUM = new QLineEdit(GB_JointSelection_10);
        LE_VARIABLE_SUPPLY_SWING_NUM->setObjectName(QStringLiteral("LE_VARIABLE_SUPPLY_SWING_NUM"));
        LE_VARIABLE_SUPPLY_SWING_NUM->setGeometry(QRect(120, 119, 51, 20));
        LE_VARIABLE_SUPPLY_SWING_NUM->setFont(font2);
        LE_VARIABLE_SUPPLY_SWING_NUM->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label_47 = new QLabel(GB_JointSelection_10);
        label_47->setObjectName(QStringLiteral("label_47"));
        label_47->setGeometry(QRect(10, 89, 101, 21));
        label_47->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label_70 = new QLabel(GB_JointSelection_10);
        label_70->setObjectName(QStringLiteral("label_70"));
        label_70->setGeometry(QRect(177, 89, 21, 21));
        label_70->setAlignment(Qt::AlignCenter);
        LE_VARIABLE_SUPPLY_SWING_PER = new QLineEdit(GB_JointSelection_10);
        LE_VARIABLE_SUPPLY_SWING_PER->setObjectName(QStringLiteral("LE_VARIABLE_SUPPLY_SWING_PER"));
        LE_VARIABLE_SUPPLY_SWING_PER->setGeometry(QRect(120, 89, 51, 20));
        LE_VARIABLE_SUPPLY_SWING_PER->setFont(font2);
        LE_VARIABLE_SUPPLY_SWING_PER->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label_58 = new QLabel(GB_JointSelection_10);
        label_58->setObjectName(QStringLiteral("label_58"));
        label_58->setGeometry(QRect(0, 117, 111, 21));
        label_58->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);
        label_71 = new QLabel(GB_JointSelection_10);
        label_71->setObjectName(QStringLiteral("label_71"));
        label_71->setGeometry(QRect(176, 118, 21, 21));
        label_71->setAlignment(Qt::AlignCenter);

        retranslateUi(SH_TASK_Dialog);

        QMetaObject::connectSlotsByName(SH_TASK_Dialog);
    } // setupUi

    void retranslateUi(QWidget *SH_TASK_Dialog)
    {
        SH_TASK_Dialog->setWindowTitle(QApplication::translate("SH_TASK_Dialog", "Form", Q_NULLPTR));
        GB_JointSelection_2->setTitle(QApplication::translate("SH_TASK_Dialog", "Joint Select", Q_NULLPTR));
        RB_RKN->setText(QApplication::translate("SH_TASK_Dialog", "RKN", Q_NULLPTR));
        RB_RHR->setText(QApplication::translate("SH_TASK_Dialog", "RHR", Q_NULLPTR));
        RB_LHP->setText(QApplication::translate("SH_TASK_Dialog", "LHP", Q_NULLPTR));
        RB_RAR->setText(QApplication::translate("SH_TASK_Dialog", "RAR", Q_NULLPTR));
        RB_RHY->setText(QApplication::translate("SH_TASK_Dialog", "RHY", Q_NULLPTR));
        RB_LHR->setText(QApplication::translate("SH_TASK_Dialog", "LHR", Q_NULLPTR));
        RB_LAP->setText(QApplication::translate("SH_TASK_Dialog", "LAP", Q_NULLPTR));
        RB_LAR->setText(QApplication::translate("SH_TASK_Dialog", "LAR", Q_NULLPTR));
        RB_RAP->setText(QApplication::translate("SH_TASK_Dialog", "RAP", Q_NULLPTR));
        RB_LKN->setText(QApplication::translate("SH_TASK_Dialog", "LKN", Q_NULLPTR));
        RB_LHY->setText(QApplication::translate("SH_TASK_Dialog", "LHY", Q_NULLPTR));
        RB_RHP->setText(QApplication::translate("SH_TASK_Dialog", "RHP", Q_NULLPTR));
        RB_WST->setText(QApplication::translate("SH_TASK_Dialog", "WST", Q_NULLPTR));
        GB_JointSelection_5->setTitle(QApplication::translate("SH_TASK_Dialog", "Position Control", Q_NULLPTR));
        label_50->setText(QApplication::translate("SH_TASK_Dialog", "sec", Q_NULLPTR));
        label_52->setText(QApplication::translate("SH_TASK_Dialog", "SineWave Num", Q_NULLPTR));
        label_53->setText(QApplication::translate("SH_TASK_Dialog", "SineWave Mag", Q_NULLPTR));
        label_49->setText(QApplication::translate("SH_TASK_Dialog", "SineWave Period", Q_NULLPTR));
        label_39->setText(QApplication::translate("SH_TASK_Dialog", "Position(deg)", Q_NULLPTR));
        label_51->setText(QApplication::translate("SH_TASK_Dialog", "deg", Q_NULLPTR));
        LE_POSITION_CONTROL_SINEWAVE_NUM->setText(QApplication::translate("SH_TASK_Dialog", "5.0", Q_NULLPTR));
        label_41->setText(QApplication::translate("SH_TASK_Dialog", "Time(sec)", Q_NULLPTR));
        BTN_GOTO_RANDOM_POSITION->setText(QApplication::translate("SH_TASK_Dialog", "Random Position", Q_NULLPTR));
        LE_POSITION_CONTROL_SINEWAVE_MAG->setText(QApplication::translate("SH_TASK_Dialog", "30.0", Q_NULLPTR));
        LE_GOTO_POSITION_POSITION->setText(QApplication::translate("SH_TASK_Dialog", "10.0", Q_NULLPTR));
        LE_POSITION_CONTROL_SINEWAVE_PERIOD->setText(QApplication::translate("SH_TASK_Dialog", "1.0", Q_NULLPTR));
        BTN_POSITION_CONTROL_SINEWAVE_GO->setText(QApplication::translate("SH_TASK_Dialog", "Position Sinewave Go", Q_NULLPTR));
        LE_GOTO_POSITION_TIME->setText(QApplication::translate("SH_TASK_Dialog", "3.0", Q_NULLPTR));
        BTN_LIGHT_GOTO_POSITION->setText(QApplication::translate("SH_TASK_Dialog", "Go to Position", Q_NULLPTR));
        label_40->setText(QApplication::translate("SH_TASK_Dialog", "Time(sec)", Q_NULLPTR));
        LE_GOTO_RANDOM_POSITION_TIME->setText(QApplication::translate("SH_TASK_Dialog", "5.0", Q_NULLPTR));
        GB_JointSelection_6->setTitle(QApplication::translate("SH_TASK_Dialog", "Torque Control", Q_NULLPTR));
        label_42->setText(QApplication::translate("SH_TASK_Dialog", "Reference Torque", Q_NULLPTR));
        BTN_TORQUE_CONTROL_STOP->setText(QApplication::translate("SH_TASK_Dialog", "Torque Control STOP", Q_NULLPTR));
        BTN_TORQUE_CONTROL_GO->setText(QApplication::translate("SH_TASK_Dialog", "Torque Control GO!", Q_NULLPTR));
        LE_REF_TORQUE->setText(QApplication::translate("SH_TASK_Dialog", "10", Q_NULLPTR));
        label_66->setText(QApplication::translate("SH_TASK_Dialog", "N", Q_NULLPTR));
        GB_JointSelection_7->setTitle(QApplication::translate("SH_TASK_Dialog", "Openloop Opening", Q_NULLPTR));
        label_33->setText(QApplication::translate("SH_TASK_Dialog", "Opening Time", Q_NULLPTR));
        label_36->setText(QApplication::translate("SH_TASK_Dialog", "ms", Q_NULLPTR));
        BTN_OPENLOOP_SINE_GO->setText(QApplication::translate("SH_TASK_Dialog", "Openloop Sinewave Go", Q_NULLPTR));
        LE_OPENLOOP_SINE_NUM->setText(QApplication::translate("SH_TASK_Dialog", "5.0", Q_NULLPTR));
        label_34->setText(QApplication::translate("SH_TASK_Dialog", "Opening Mag", Q_NULLPTR));
        label_35->setText(QApplication::translate("SH_TASK_Dialog", "Opening Num", Q_NULLPTR));
        BTN_LIGHT_FINDHOME_MINUS_END->setText(QApplication::translate("SH_TASK_Dialog", "FindHome \n"
"(Minus End)", Q_NULLPTR));
        LE_OPENLOOP_SINE_TIME->setText(QApplication::translate("SH_TASK_Dialog", "1000.0", Q_NULLPTR));
        BTN_LIGHT_FINDHOME_PLUS_END->setText(QApplication::translate("SH_TASK_Dialog", "FindHome\n"
"(Plus End)", Q_NULLPTR));
        LE_OPENLOOP_SINE_MAG->setText(QApplication::translate("SH_TASK_Dialog", "1000.0", Q_NULLPTR));
        LE_CONST_OPEN_VALUE->setText(QApplication::translate("SH_TASK_Dialog", "300", Q_NULLPTR));
        BTN_CONST_OPEN_NEG->setText(QApplication::translate("SH_TASK_Dialog", "Move -", Q_NULLPTR));
        BTN_CONST_OPEN_POS->setText(QApplication::translate("SH_TASK_Dialog", "Move", Q_NULLPTR));
        label_37->setText(QApplication::translate("SH_TASK_Dialog", "Const Open\n"
"(Range : -10000~10000)", Q_NULLPTR));
        GB_JointSelection_8->setTitle(QApplication::translate("SH_TASK_Dialog", "Valve Identification", Q_NULLPTR));
        label_57->setText(QApplication::translate("SH_TASK_Dialog", "Min. Open", Q_NULLPTR));
        EDIT_LIGHT_VALVEID_OPEN_RESOL->setText(QApplication::translate("SH_TASK_Dialog", "100", Q_NULLPTR));
        EDIT_LIGHT_VALVEID_OPEN_MIN->setText(QApplication::translate("SH_TASK_Dialog", "-500", Q_NULLPTR));
        label_48->setText(QApplication::translate("SH_TASK_Dialog", "Max. Open", Q_NULLPTR));
        label_55->setText(QApplication::translate("SH_TASK_Dialog", "Resol. Open", Q_NULLPTR));
        EDIT_LIGHT_VALVEID_OPEN_MAX->setText(QApplication::translate("SH_TASK_Dialog", "5000", Q_NULLPTR));
        BTN_LIGHT_ANKLE_VALVE_IDENTIFICATION->setText(QApplication::translate("SH_TASK_Dialog", "Valve I.D.\n"
"(Minus>Plus)", Q_NULLPTR));
        BTN_LIGHT_ANKLE_VALVE_IDENTIFICATION_NEG->setText(QApplication::translate("SH_TASK_Dialog", "Valve I.D.\n"
"(Plus>Minus)", Q_NULLPTR));
        EDIT_LIGHT_VALVEID_PRES_MIN->setText(QApplication::translate("SH_TASK_Dialog", "50", Q_NULLPTR));
        EDIT_LIGHT_VALVEID_PRES_RESOL->setText(QApplication::translate("SH_TASK_Dialog", "10", Q_NULLPTR));
        EDIT_LIGHT_VALVEID_PRES_MAX->setText(QApplication::translate("SH_TASK_Dialog", "80", Q_NULLPTR));
        label_60->setText(QApplication::translate("SH_TASK_Dialog", "Min. Pres", Q_NULLPTR));
        label_54->setText(QApplication::translate("SH_TASK_Dialog", "Max. Pres", Q_NULLPTR));
        label_56->setText(QApplication::translate("SH_TASK_Dialog", "Resol. Pres", Q_NULLPTR));
        GB_JointSelection_9->setTitle(QApplication::translate("SH_TASK_Dialog", "Data Save", Q_NULLPTR));
        BTN_SAVE_START->setText(QApplication::translate("SH_TASK_Dialog", "Save Start", Q_NULLPTR));
        BTN_SAVE_STOP->setText(QApplication::translate("SH_TASK_Dialog", "Save Stop", Q_NULLPTR));
        CON_SAVE_FILENAME->setText(QString());
        label_59->setText(QApplication::translate("SH_TASK_Dialog", "Subtitle : ", Q_NULLPTR));
        groupBox_4->setTitle(QApplication::translate("SH_TASK_Dialog", "[ Robot(LIGHT) Control Mode ]", Q_NULLPTR));
        BTN_ROBOT_CONTROLMETHOD_CHANGE_TO_POS->setText(QApplication::translate("SH_TASK_Dialog", "ALL Joint Change to Pos. Ctrl.", Q_NULLPTR));
        checkBox->setText(QApplication::translate("SH_TASK_Dialog", "RHY", Q_NULLPTR));
        checkBox_2->setText(QApplication::translate("SH_TASK_Dialog", "RHR", Q_NULLPTR));
        CBX_ROBOT_CONTROLMETHOD_RHP->setText(QApplication::translate("SH_TASK_Dialog", "RHP", Q_NULLPTR));
        CBX_ROBOT_CONTROLMETHOD_RKN->setText(QApplication::translate("SH_TASK_Dialog", "RKN", Q_NULLPTR));
        CBX_ROBOT_CONTROLMETHOD_RANK->setText(QApplication::translate("SH_TASK_Dialog", "RANK(Both)", Q_NULLPTR));
        checkBox_6->setText(QApplication::translate("SH_TASK_Dialog", "LHY", Q_NULLPTR));
        CBX_ROBOT_CONTROLMETHOD_LANK->setText(QApplication::translate("SH_TASK_Dialog", "LANK(Both)", Q_NULLPTR));
        CBX_ROBOT_CONTROLMETHOD_LHP->setText(QApplication::translate("SH_TASK_Dialog", "LHP", Q_NULLPTR));
        checkBox_9->setText(QApplication::translate("SH_TASK_Dialog", "LHR", Q_NULLPTR));
        CBX_ROBOT_CONTROLMETHOD_LKN->setText(QApplication::translate("SH_TASK_Dialog", "LKN", Q_NULLPTR));
        BTN_ROBOT_CONTROLMETHOD_CHANGE->setText(QApplication::translate("SH_TASK_Dialog", "Mode Change", Q_NULLPTR));
        BTN_ROBOT_CONTROLMETHOD_CHANGE_TO_TOR->setText(QApplication::translate("SH_TASK_Dialog", "ALL Joint Change to Tor. Ctrl.", Q_NULLPTR));
        label_43->setText(QApplication::translate("SH_TASK_Dialog", "(Checked :Unchecked : Position Control,  Torque Control)", Q_NULLPTR));
        GB_JointSelection_10->setTitle(QApplication::translate("SH_TASK_Dialog", "Variable Supply Pressure Exp.", Q_NULLPTR));
        label_44->setText(QApplication::translate("SH_TASK_Dialog", "Swing Mass", Q_NULLPTR));
        LE_VARIABLE_SUPPLY_SWING_STOP->setText(QApplication::translate("SH_TASK_Dialog", "Stop!", Q_NULLPTR));
        LE_VARIABLE_SUPPLY_SWING_GO->setText(QApplication::translate("SH_TASK_Dialog", "Go!", Q_NULLPTR));
        LE_VARIABLE_SUPPLY_SWING_MASS->setText(QApplication::translate("SH_TASK_Dialog", "10", Q_NULLPTR));
        label_67->setText(QApplication::translate("SH_TASK_Dialog", "kg", Q_NULLPTR));
        label_45->setText(QString());
        label_68->setText(QApplication::translate("SH_TASK_Dialog", "deg", Q_NULLPTR));
        LE_VARIABLE_SUPPLY_SWING_MAG->setText(QApplication::translate("SH_TASK_Dialog", "30", Q_NULLPTR));
        label_46->setText(QApplication::translate("SH_TASK_Dialog", "Swing Magnitude", Q_NULLPTR));
        LE_VARIABLE_SUPPLY_SWING_NUM->setText(QApplication::translate("SH_TASK_Dialog", "3", Q_NULLPTR));
        label_47->setText(QApplication::translate("SH_TASK_Dialog", "Swing Period", Q_NULLPTR));
        label_70->setText(QApplication::translate("SH_TASK_Dialog", "sec", Q_NULLPTR));
        LE_VARIABLE_SUPPLY_SWING_PER->setText(QApplication::translate("SH_TASK_Dialog", "5", Q_NULLPTR));
        label_58->setText(QApplication::translate("SH_TASK_Dialog", "Swing Number", Q_NULLPTR));
        label_71->setText(QApplication::translate("SH_TASK_Dialog", "rep", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class SH_TASK_Dialog: public Ui_SH_TASK_Dialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SH_TASK_DIALOG_H
