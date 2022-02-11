/********************************************************************************
** Form generated from reading UI file 'HCBSettingDialog.ui'
**
** Created by: Qt User Interface Compiler version 5.9.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_HCBSETTINGDIALOG_H
#define UI_HCBSETTINGDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_HCBSettingDialog
{
public:
    QGroupBox *GB_JointSelection;
    QPushButton *BTN_CAN_CHECK;
    QPushButton *BTN_FET_ONOFF;
    QPushButton *BTN_FET_ONOFF_2;
    QPushButton *BTN_CAN_CHANNEL_ARRANGE;
    QGroupBox *GB_JointSelection_2;
    QPushButton *BTN_ASK_EVERYTHING;
    QPushButton *BTN_ENC_ZERO;
    QPushButton *BTN_FINDHOME_ALL;
    QPushButton *BTN_FINDHOME_ONEJOINT;
    QWidget *layoutWidget;
    QGridLayout *gridLayout;
    QRadioButton *RB_RHR;
    QRadioButton *RB_LHR;
    QRadioButton *RB_RHY;
    QRadioButton *RB_LHY;
    QRadioButton *RB_RHP;
    QRadioButton *RB_LHP;
    QRadioButton *RB_RKN;
    QRadioButton *RB_LKN;
    QRadioButton *RB_RAP;
    QRadioButton *RB_LAP;
    QRadioButton *RB_RAR;
    QRadioButton *RB_LAR;
    QRadioButton *RB_WST;
    QPushButton *BTN_FINDHOME_SECOND_STAGE;
    QPushButton *BTN_FINDHOME_FIRST_STAGE;
    QGroupBox *GB_JointSelection_3;
    QPushButton *BTN_PARAMETER_SET_ALL;
    QPushButton *BTN_SHOW_CURRENT_PARA;
    QLineEdit *LE_000;
    QLabel *label_2;
    QLabel *label_3;
    QLabel *label_4;
    QLabel *label_5;
    QLabel *label_6;
    QLabel *label_7;
    QLabel *label_8;
    QLabel *label_9;
    QLabel *label_10;
    QLabel *label_11;
    QLabel *label_12;
    QLabel *label_13;
    QLabel *label_14;
    QLabel *label_15;
    QLabel *label_16;
    QLabel *label_17;
    QLabel *label_18;
    QLabel *label_20;
    QLabel *label_21;
    QLabel *label_22;
    QLabel *label_23;
    QLabel *label_24;
    QLabel *label_25;
    QLabel *label_26;
    QLabel *label_27;
    QLabel *label_28;
    QLabel *label_29;
    QLineEdit *LE_001;
    QLineEdit *LE_002;
    QLineEdit *LE_003;
    QLineEdit *LE_004;
    QLineEdit *LE_005;
    QLineEdit *LE_006;
    QLineEdit *LE_007;
    QLabel *label_30;
    QLabel *label_31;
    QLabel *label_32;
    QLineEdit *LE_008;
    QLineEdit *LE_009;
    QLineEdit *LE_010;
    QLineEdit *LE_011;
    QLineEdit *LE_012;
    QLabel *label_33;
    QLabel *label_34;
    QLabel *label_35;
    QLineEdit *LE_100;
    QLineEdit *LE_101;
    QLineEdit *LE_103;
    QLineEdit *LE_104;
    QLineEdit *LE_105;
    QLineEdit *LE_106;
    QLineEdit *LE_107;
    QLineEdit *LE_108;
    QLineEdit *LE_109;
    QLineEdit *LE_110;
    QLineEdit *LE_111;
    QLineEdit *LE_112;
    QLineEdit *LE_113_A;
    QLineEdit *LE_200;
    QLineEdit *LE_300;
    QLineEdit *LE_400;
    QLineEdit *LE_401;
    QLineEdit *LE_201;
    QLineEdit *LE_301;
    QLineEdit *LE_402;
    QLineEdit *LE_302;
    QLineEdit *LE_202;
    QLineEdit *LE_013;
    QLineEdit *LE_102;
    QLineEdit *LE_0001;
    QLabel *label_36;
    QLineEdit *LE_115;
    QLabel *label_37;
    QLineEdit *LE_116;
    QLabel *label_38;
    QLineEdit *LE_113_B;
    QLineEdit *LE_014;
    QLineEdit *LE_114;
    QLineEdit *LE_501;
    QLineEdit *LE_500;
    QLabel *label_39;
    QLabel *label_40;
    QLabel *label_41;
    QLabel *label_42;
    QLabel *label_43;
    QLabel *label_44;
    QLabel *label_45;
    QLabel *label_46;
    QLabel *label_47;
    QLabel *label_48;
    QLabel *label_49;
    QLabel *label_50;
    QLineEdit *LE_0002;
    QLabel *label_19;
    QLabel *label_51;
    QLabel *label_52;
    QLineEdit *LE_117;
    QGroupBox *GB_JointSelection_6;
    QPushButton *BTN_VALVEPOS_ENABLE;
    QPushButton *BTN_ENC_ENABLE;
    QPushButton *BTN_DISABLE_ALL;
    QFrame *line;
    QPushButton *BTN_SOMETHING_ENABLE;
    QPushButton *BTN_BOARD_TEST_ERRORRESET;
    QPushButton *BTN_SET_BNO;
    QLineEdit *LE_TARGET_BNO;
    QPushButton *BTN_PRESSURE_NULLING;
    QPushButton *BTN_TORQUEFORCE_NULLING;
    QPushButton *BTN_TORQUEFORCE_NULLING_ONE;
    QPushButton *BTN_PRESSURE_NULLING_ONE;

    void setupUi(QWidget *HCBSettingDialog)
    {
        if (HCBSettingDialog->objectName().isEmpty())
            HCBSettingDialog->setObjectName(QStringLiteral("HCBSettingDialog"));
        HCBSettingDialog->resize(780, 780);
        GB_JointSelection = new QGroupBox(HCBSettingDialog);
        GB_JointSelection->setObjectName(QStringLiteral("GB_JointSelection"));
        GB_JointSelection->setGeometry(QRect(10, 10, 511, 61));
        QFont font;
        font.setBold(true);
        font.setWeight(75);
        GB_JointSelection->setFont(font);
        GB_JointSelection->setStyleSheet(QLatin1String("QGroupBox {\n"
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
        BTN_CAN_CHECK = new QPushButton(GB_JointSelection);
        BTN_CAN_CHECK->setObjectName(QStringLiteral("BTN_CAN_CHECK"));
        BTN_CAN_CHECK->setGeometry(QRect(130, 20, 101, 31));
        BTN_FET_ONOFF = new QPushButton(GB_JointSelection);
        BTN_FET_ONOFF->setObjectName(QStringLiteral("BTN_FET_ONOFF"));
        BTN_FET_ONOFF->setGeometry(QRect(240, 20, 101, 31));
        BTN_FET_ONOFF_2 = new QPushButton(GB_JointSelection);
        BTN_FET_ONOFF_2->setObjectName(QStringLiteral("BTN_FET_ONOFF_2"));
        BTN_FET_ONOFF_2->setGeometry(QRect(350, 20, 91, 31));
        BTN_CAN_CHANNEL_ARRANGE = new QPushButton(GB_JointSelection);
        BTN_CAN_CHANNEL_ARRANGE->setObjectName(QStringLiteral("BTN_CAN_CHANNEL_ARRANGE"));
        BTN_CAN_CHANNEL_ARRANGE->setGeometry(QRect(10, 20, 111, 31));
        GB_JointSelection_2 = new QGroupBox(HCBSettingDialog);
        GB_JointSelection_2->setObjectName(QStringLiteral("GB_JointSelection_2"));
        GB_JointSelection_2->setGeometry(QRect(10, 76, 511, 181));
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
        BTN_ASK_EVERYTHING = new QPushButton(GB_JointSelection_2);
        BTN_ASK_EVERYTHING->setObjectName(QStringLiteral("BTN_ASK_EVERYTHING"));
        BTN_ASK_EVERYTHING->setGeometry(QRect(140, 20, 101, 41));
        QFont font1;
        font1.setPointSize(10);
        BTN_ASK_EVERYTHING->setFont(font1);
        BTN_ASK_EVERYTHING->setStyleSheet(QStringLiteral(""));
        BTN_ENC_ZERO = new QPushButton(GB_JointSelection_2);
        BTN_ENC_ZERO->setObjectName(QStringLiteral("BTN_ENC_ZERO"));
        BTN_ENC_ZERO->setGeometry(QRect(380, 20, 121, 41));
        QFont font2;
        font2.setPointSize(8);
        BTN_ENC_ZERO->setFont(font2);
        BTN_ENC_ZERO->setStyleSheet(QStringLiteral(""));
        BTN_FINDHOME_ALL = new QPushButton(GB_JointSelection_2);
        BTN_FINDHOME_ALL->setObjectName(QStringLiteral("BTN_FINDHOME_ALL"));
        BTN_FINDHOME_ALL->setGeometry(QRect(380, 120, 121, 41));
        BTN_FINDHOME_ALL->setFont(font2);
        BTN_FINDHOME_ALL->setStyleSheet(QStringLiteral(""));
        BTN_FINDHOME_ONEJOINT = new QPushButton(GB_JointSelection_2);
        BTN_FINDHOME_ONEJOINT->setObjectName(QStringLiteral("BTN_FINDHOME_ONEJOINT"));
        BTN_FINDHOME_ONEJOINT->setGeometry(QRect(380, 70, 121, 41));
        BTN_FINDHOME_ONEJOINT->setFont(font2);
        BTN_FINDHOME_ONEJOINT->setStyleSheet(QStringLiteral(""));
        layoutWidget = new QWidget(GB_JointSelection_2);
        layoutWidget->setObjectName(QStringLiteral("layoutWidget"));
        layoutWidget->setGeometry(QRect(21, 21, 168, 152));
        gridLayout = new QGridLayout(layoutWidget);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        RB_RHR = new QRadioButton(layoutWidget);
        RB_RHR->setObjectName(QStringLiteral("RB_RHR"));
        RB_RHR->setChecked(false);

        gridLayout->addWidget(RB_RHR, 0, 0, 1, 1);

        RB_LHR = new QRadioButton(layoutWidget);
        RB_LHR->setObjectName(QStringLiteral("RB_LHR"));
        RB_LHR->setChecked(false);

        gridLayout->addWidget(RB_LHR, 0, 1, 1, 1);

        RB_RHY = new QRadioButton(layoutWidget);
        RB_RHY->setObjectName(QStringLiteral("RB_RHY"));
        RB_RHY->setChecked(false);

        gridLayout->addWidget(RB_RHY, 1, 0, 1, 1);

        RB_LHY = new QRadioButton(layoutWidget);
        RB_LHY->setObjectName(QStringLiteral("RB_LHY"));

        gridLayout->addWidget(RB_LHY, 1, 1, 1, 1);

        RB_RHP = new QRadioButton(layoutWidget);
        RB_RHP->setObjectName(QStringLiteral("RB_RHP"));

        gridLayout->addWidget(RB_RHP, 2, 0, 1, 1);

        RB_LHP = new QRadioButton(layoutWidget);
        RB_LHP->setObjectName(QStringLiteral("RB_LHP"));

        gridLayout->addWidget(RB_LHP, 2, 1, 1, 1);

        RB_RKN = new QRadioButton(layoutWidget);
        RB_RKN->setObjectName(QStringLiteral("RB_RKN"));

        gridLayout->addWidget(RB_RKN, 3, 0, 1, 1);

        RB_LKN = new QRadioButton(layoutWidget);
        RB_LKN->setObjectName(QStringLiteral("RB_LKN"));

        gridLayout->addWidget(RB_LKN, 3, 1, 1, 1);

        RB_RAP = new QRadioButton(layoutWidget);
        RB_RAP->setObjectName(QStringLiteral("RB_RAP"));

        gridLayout->addWidget(RB_RAP, 4, 0, 1, 1);

        RB_LAP = new QRadioButton(layoutWidget);
        RB_LAP->setObjectName(QStringLiteral("RB_LAP"));

        gridLayout->addWidget(RB_LAP, 4, 1, 1, 1);

        RB_RAR = new QRadioButton(layoutWidget);
        RB_RAR->setObjectName(QStringLiteral("RB_RAR"));

        gridLayout->addWidget(RB_RAR, 5, 0, 1, 1);

        RB_LAR = new QRadioButton(layoutWidget);
        RB_LAR->setObjectName(QStringLiteral("RB_LAR"));

        gridLayout->addWidget(RB_LAR, 5, 1, 1, 1);

        RB_WST = new QRadioButton(layoutWidget);
        RB_WST->setObjectName(QStringLiteral("RB_WST"));
        RB_WST->setChecked(true);

        gridLayout->addWidget(RB_WST, 5, 2, 1, 1);

        BTN_FINDHOME_SECOND_STAGE = new QPushButton(GB_JointSelection_2);
        BTN_FINDHOME_SECOND_STAGE->setObjectName(QStringLiteral("BTN_FINDHOME_SECOND_STAGE"));
        BTN_FINDHOME_SECOND_STAGE->setGeometry(QRect(260, 100, 111, 51));
        QFont font3;
        font3.setPointSize(9);
        BTN_FINDHOME_SECOND_STAGE->setFont(font3);
        BTN_FINDHOME_SECOND_STAGE->setStyleSheet(QStringLiteral(""));
        BTN_FINDHOME_FIRST_STAGE = new QPushButton(GB_JointSelection_2);
        BTN_FINDHOME_FIRST_STAGE->setObjectName(QStringLiteral("BTN_FINDHOME_FIRST_STAGE"));
        BTN_FINDHOME_FIRST_STAGE->setGeometry(QRect(260, 40, 111, 51));
        BTN_FINDHOME_FIRST_STAGE->setFont(font3);
        BTN_FINDHOME_FIRST_STAGE->setStyleSheet(QStringLiteral(""));
        BTN_ENC_ZERO->raise();
        BTN_FINDHOME_ALL->raise();
        BTN_FINDHOME_ONEJOINT->raise();
        layoutWidget->raise();
        BTN_ASK_EVERYTHING->raise();
        BTN_FINDHOME_SECOND_STAGE->raise();
        BTN_FINDHOME_FIRST_STAGE->raise();
        GB_JointSelection_3 = new QGroupBox(HCBSettingDialog);
        GB_JointSelection_3->setObjectName(QStringLiteral("GB_JointSelection_3"));
        GB_JointSelection_3->setGeometry(QRect(10, 330, 751, 431));
        GB_JointSelection_3->setFont(font);
        GB_JointSelection_3->setStyleSheet(QLatin1String("QGroupBox {\n"
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
        BTN_PARAMETER_SET_ALL = new QPushButton(GB_JointSelection_3);
        BTN_PARAMETER_SET_ALL->setObjectName(QStringLiteral("BTN_PARAMETER_SET_ALL"));
        BTN_PARAMETER_SET_ALL->setEnabled(true);
        BTN_PARAMETER_SET_ALL->setGeometry(QRect(550, 360, 161, 31));
        BTN_SHOW_CURRENT_PARA = new QPushButton(GB_JointSelection_3);
        BTN_SHOW_CURRENT_PARA->setObjectName(QStringLiteral("BTN_SHOW_CURRENT_PARA"));
        BTN_SHOW_CURRENT_PARA->setGeometry(QRect(10, 20, 131, 31));
        LE_000 = new QLineEdit(GB_JointSelection_3);
        LE_000->setObjectName(QStringLiteral("LE_000"));
        LE_000->setGeometry(QRect(150, 60, 91, 16));
        QFont font4;
        font4.setPointSize(7);
        LE_000->setFont(font4);
        label_2 = new QLabel(GB_JointSelection_3);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setGeometry(QRect(10, 60, 111, 16));
        label_3 = new QLabel(GB_JointSelection_3);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setGeometry(QRect(10, 120, 111, 16));
        label_4 = new QLabel(GB_JointSelection_3);
        label_4->setObjectName(QStringLiteral("label_4"));
        label_4->setGeometry(QRect(10, 140, 111, 16));
        label_5 = new QLabel(GB_JointSelection_3);
        label_5->setObjectName(QStringLiteral("label_5"));
        label_5->setGeometry(QRect(10, 160, 111, 16));
        label_6 = new QLabel(GB_JointSelection_3);
        label_6->setObjectName(QStringLiteral("label_6"));
        label_6->setGeometry(QRect(10, 180, 111, 16));
        label_7 = new QLabel(GB_JointSelection_3);
        label_7->setObjectName(QStringLiteral("label_7"));
        label_7->setGeometry(QRect(10, 200, 111, 16));
        label_8 = new QLabel(GB_JointSelection_3);
        label_8->setObjectName(QStringLiteral("label_8"));
        label_8->setGeometry(QRect(10, 220, 111, 16));
        label_9 = new QLabel(GB_JointSelection_3);
        label_9->setObjectName(QStringLiteral("label_9"));
        label_9->setGeometry(QRect(10, 240, 141, 16));
        label_10 = new QLabel(GB_JointSelection_3);
        label_10->setObjectName(QStringLiteral("label_10"));
        label_10->setGeometry(QRect(590, 30, 111, 16));
        label_11 = new QLabel(GB_JointSelection_3);
        label_11->setObjectName(QStringLiteral("label_11"));
        label_11->setGeometry(QRect(590, 115, 111, 16));
        label_12 = new QLabel(GB_JointSelection_3);
        label_12->setObjectName(QStringLiteral("label_12"));
        label_12->setGeometry(QRect(590, 200, 111, 16));
        label_13 = new QLabel(GB_JointSelection_3);
        label_13->setObjectName(QStringLiteral("label_13"));
        label_13->setGeometry(QRect(10, 280, 111, 16));
        label_14 = new QLabel(GB_JointSelection_3);
        label_14->setObjectName(QStringLiteral("label_14"));
        label_14->setGeometry(QRect(10, 320, 111, 16));
        label_15 = new QLabel(GB_JointSelection_3);
        label_15->setObjectName(QStringLiteral("label_15"));
        label_15->setGeometry(QRect(250, 80, 151, 16));
        label_16 = new QLabel(GB_JointSelection_3);
        label_16->setObjectName(QStringLiteral("label_16"));
        label_16->setGeometry(QRect(250, 100, 161, 16));
        label_17 = new QLabel(GB_JointSelection_3);
        label_17->setObjectName(QStringLiteral("label_17"));
        label_17->setGeometry(QRect(250, 120, 121, 16));
        label_18 = new QLabel(GB_JointSelection_3);
        label_18->setObjectName(QStringLiteral("label_18"));
        label_18->setGeometry(QRect(250, 140, 161, 16));
        label_20 = new QLabel(GB_JointSelection_3);
        label_20->setObjectName(QStringLiteral("label_20"));
        label_20->setGeometry(QRect(250, 40, 91, 16));
        label_21 = new QLabel(GB_JointSelection_3);
        label_21->setObjectName(QStringLiteral("label_21"));
        label_21->setGeometry(QRect(250, 160, 101, 16));
        label_22 = new QLabel(GB_JointSelection_3);
        label_22->setObjectName(QStringLiteral("label_22"));
        label_22->setGeometry(QRect(250, 200, 91, 16));
        label_23 = new QLabel(GB_JointSelection_3);
        label_23->setObjectName(QStringLiteral("label_23"));
        label_23->setGeometry(QRect(250, 240, 81, 16));
        label_24 = new QLabel(GB_JointSelection_3);
        label_24->setObjectName(QStringLiteral("label_24"));
        label_24->setGeometry(QRect(250, 260, 131, 16));
        label_25 = new QLabel(GB_JointSelection_3);
        label_25->setObjectName(QStringLiteral("label_25"));
        label_25->setGeometry(QRect(10, 380, 151, 16));
        label_26 = new QLabel(GB_JointSelection_3);
        label_26->setObjectName(QStringLiteral("label_26"));
        label_26->setGeometry(QRect(10, 400, 131, 16));
        label_27 = new QLabel(GB_JointSelection_3);
        label_27->setObjectName(QStringLiteral("label_27"));
        label_27->setGeometry(QRect(250, 300, 161, 16));
        label_28 = new QLabel(GB_JointSelection_3);
        label_28->setObjectName(QStringLiteral("label_28"));
        label_28->setGeometry(QRect(250, 340, 161, 16));
        label_29 = new QLabel(GB_JointSelection_3);
        label_29->setObjectName(QStringLiteral("label_29"));
        label_29->setGeometry(QRect(250, 180, 101, 16));
        LE_001 = new QLineEdit(GB_JointSelection_3);
        LE_001->setObjectName(QStringLiteral("LE_001"));
        LE_001->setGeometry(QRect(150, 120, 91, 16));
        LE_001->setFont(font4);
        LE_002 = new QLineEdit(GB_JointSelection_3);
        LE_002->setObjectName(QStringLiteral("LE_002"));
        LE_002->setGeometry(QRect(150, 140, 91, 16));
        LE_002->setFont(font4);
        LE_003 = new QLineEdit(GB_JointSelection_3);
        LE_003->setObjectName(QStringLiteral("LE_003"));
        LE_003->setGeometry(QRect(150, 160, 91, 16));
        LE_003->setFont(font4);
        LE_004 = new QLineEdit(GB_JointSelection_3);
        LE_004->setObjectName(QStringLiteral("LE_004"));
        LE_004->setGeometry(QRect(150, 180, 91, 16));
        LE_004->setFont(font4);
        LE_005 = new QLineEdit(GB_JointSelection_3);
        LE_005->setObjectName(QStringLiteral("LE_005"));
        LE_005->setGeometry(QRect(150, 200, 91, 16));
        LE_005->setFont(font4);
        LE_006 = new QLineEdit(GB_JointSelection_3);
        LE_006->setObjectName(QStringLiteral("LE_006"));
        LE_006->setGeometry(QRect(150, 220, 91, 16));
        LE_006->setFont(font4);
        LE_007 = new QLineEdit(GB_JointSelection_3);
        LE_007->setObjectName(QStringLiteral("LE_007"));
        LE_007->setGeometry(QRect(150, 240, 91, 16));
        LE_007->setFont(font4);
        label_30 = new QLabel(GB_JointSelection_3);
        label_30->setObjectName(QStringLiteral("label_30"));
        label_30->setGeometry(QRect(10, 300, 111, 16));
        label_31 = new QLabel(GB_JointSelection_3);
        label_31->setObjectName(QStringLiteral("label_31"));
        label_31->setGeometry(QRect(10, 340, 111, 16));
        label_32 = new QLabel(GB_JointSelection_3);
        label_32->setObjectName(QStringLiteral("label_32"));
        label_32->setGeometry(QRect(10, 360, 111, 16));
        LE_008 = new QLineEdit(GB_JointSelection_3);
        LE_008->setObjectName(QStringLiteral("LE_008"));
        LE_008->setGeometry(QRect(150, 280, 91, 16));
        LE_008->setFont(font4);
        LE_009 = new QLineEdit(GB_JointSelection_3);
        LE_009->setObjectName(QStringLiteral("LE_009"));
        LE_009->setGeometry(QRect(150, 300, 91, 16));
        LE_009->setFont(font4);
        LE_010 = new QLineEdit(GB_JointSelection_3);
        LE_010->setObjectName(QStringLiteral("LE_010"));
        LE_010->setGeometry(QRect(150, 320, 91, 16));
        LE_010->setFont(font4);
        LE_011 = new QLineEdit(GB_JointSelection_3);
        LE_011->setObjectName(QStringLiteral("LE_011"));
        LE_011->setGeometry(QRect(150, 340, 91, 16));
        LE_011->setFont(font4);
        LE_012 = new QLineEdit(GB_JointSelection_3);
        LE_012->setObjectName(QStringLiteral("LE_012"));
        LE_012->setGeometry(QRect(150, 360, 91, 16));
        LE_012->setFont(font4);
        label_33 = new QLabel(GB_JointSelection_3);
        label_33->setObjectName(QStringLiteral("label_33"));
        label_33->setGeometry(QRect(250, 60, 91, 16));
        label_34 = new QLabel(GB_JointSelection_3);
        label_34->setObjectName(QStringLiteral("label_34"));
        label_34->setGeometry(QRect(250, 220, 91, 16));
        label_35 = new QLabel(GB_JointSelection_3);
        label_35->setObjectName(QStringLiteral("label_35"));
        label_35->setGeometry(QRect(250, 280, 131, 16));
        LE_100 = new QLineEdit(GB_JointSelection_3);
        LE_100->setObjectName(QStringLiteral("LE_100"));
        LE_100->setGeometry(QRect(410, 40, 91, 16));
        LE_100->setFont(font4);
        LE_101 = new QLineEdit(GB_JointSelection_3);
        LE_101->setObjectName(QStringLiteral("LE_101"));
        LE_101->setGeometry(QRect(410, 60, 91, 16));
        LE_101->setFont(font4);
        LE_103 = new QLineEdit(GB_JointSelection_3);
        LE_103->setObjectName(QStringLiteral("LE_103"));
        LE_103->setGeometry(QRect(410, 100, 91, 16));
        LE_103->setFont(font4);
        LE_104 = new QLineEdit(GB_JointSelection_3);
        LE_104->setObjectName(QStringLiteral("LE_104"));
        LE_104->setGeometry(QRect(410, 120, 91, 16));
        LE_104->setFont(font4);
        LE_105 = new QLineEdit(GB_JointSelection_3);
        LE_105->setObjectName(QStringLiteral("LE_105"));
        LE_105->setGeometry(QRect(410, 140, 91, 16));
        LE_105->setFont(font4);
        LE_106 = new QLineEdit(GB_JointSelection_3);
        LE_106->setObjectName(QStringLiteral("LE_106"));
        LE_106->setGeometry(QRect(410, 160, 91, 16));
        LE_106->setFont(font4);
        LE_107 = new QLineEdit(GB_JointSelection_3);
        LE_107->setObjectName(QStringLiteral("LE_107"));
        LE_107->setGeometry(QRect(410, 180, 91, 16));
        LE_107->setFont(font4);
        LE_108 = new QLineEdit(GB_JointSelection_3);
        LE_108->setObjectName(QStringLiteral("LE_108"));
        LE_108->setGeometry(QRect(410, 200, 91, 16));
        LE_108->setFont(font4);
        LE_109 = new QLineEdit(GB_JointSelection_3);
        LE_109->setObjectName(QStringLiteral("LE_109"));
        LE_109->setGeometry(QRect(410, 220, 91, 16));
        LE_109->setFont(font4);
        LE_110 = new QLineEdit(GB_JointSelection_3);
        LE_110->setObjectName(QStringLiteral("LE_110"));
        LE_110->setGeometry(QRect(410, 240, 91, 16));
        LE_110->setFont(font4);
        LE_111 = new QLineEdit(GB_JointSelection_3);
        LE_111->setObjectName(QStringLiteral("LE_111"));
        LE_111->setGeometry(QRect(410, 260, 91, 16));
        LE_111->setFont(font4);
        LE_112 = new QLineEdit(GB_JointSelection_3);
        LE_112->setObjectName(QStringLiteral("LE_112"));
        LE_112->setGeometry(QRect(410, 280, 91, 16));
        LE_112->setFont(font4);
        LE_113_A = new QLineEdit(GB_JointSelection_3);
        LE_113_A->setObjectName(QStringLiteral("LE_113_A"));
        LE_113_A->setGeometry(QRect(410, 300, 91, 16));
        LE_113_A->setFont(font4);
        LE_200 = new QLineEdit(GB_JointSelection_3);
        LE_200->setObjectName(QStringLiteral("LE_200"));
        LE_200->setGeometry(QRect(590, 50, 91, 16));
        LE_200->setFont(font4);
        LE_300 = new QLineEdit(GB_JointSelection_3);
        LE_300->setObjectName(QStringLiteral("LE_300"));
        LE_300->setGeometry(QRect(590, 135, 91, 16));
        LE_300->setFont(font4);
        LE_400 = new QLineEdit(GB_JointSelection_3);
        LE_400->setObjectName(QStringLiteral("LE_400"));
        LE_400->setGeometry(QRect(590, 220, 91, 16));
        LE_400->setFont(font4);
        LE_401 = new QLineEdit(GB_JointSelection_3);
        LE_401->setObjectName(QStringLiteral("LE_401"));
        LE_401->setGeometry(QRect(590, 240, 91, 16));
        LE_401->setFont(font4);
        LE_201 = new QLineEdit(GB_JointSelection_3);
        LE_201->setObjectName(QStringLiteral("LE_201"));
        LE_201->setGeometry(QRect(590, 70, 91, 16));
        LE_201->setFont(font4);
        LE_301 = new QLineEdit(GB_JointSelection_3);
        LE_301->setObjectName(QStringLiteral("LE_301"));
        LE_301->setGeometry(QRect(590, 155, 91, 16));
        LE_301->setFont(font4);
        LE_402 = new QLineEdit(GB_JointSelection_3);
        LE_402->setObjectName(QStringLiteral("LE_402"));
        LE_402->setGeometry(QRect(590, 260, 91, 16));
        LE_402->setFont(font4);
        LE_302 = new QLineEdit(GB_JointSelection_3);
        LE_302->setObjectName(QStringLiteral("LE_302"));
        LE_302->setGeometry(QRect(590, 175, 91, 16));
        LE_302->setFont(font4);
        LE_202 = new QLineEdit(GB_JointSelection_3);
        LE_202->setObjectName(QStringLiteral("LE_202"));
        LE_202->setGeometry(QRect(590, 90, 91, 16));
        LE_202->setFont(font4);
        LE_013 = new QLineEdit(GB_JointSelection_3);
        LE_013->setObjectName(QStringLiteral("LE_013"));
        LE_013->setGeometry(QRect(150, 380, 91, 16));
        LE_013->setFont(font4);
        LE_102 = new QLineEdit(GB_JointSelection_3);
        LE_102->setObjectName(QStringLiteral("LE_102"));
        LE_102->setGeometry(QRect(410, 80, 91, 16));
        LE_102->setFont(font4);
        LE_0001 = new QLineEdit(GB_JointSelection_3);
        LE_0001->setObjectName(QStringLiteral("LE_0001"));
        LE_0001->setGeometry(QRect(150, 80, 91, 16));
        LE_0001->setFont(font4);
        label_36 = new QLabel(GB_JointSelection_3);
        label_36->setObjectName(QStringLiteral("label_36"));
        label_36->setGeometry(QRect(250, 360, 161, 16));
        LE_115 = new QLineEdit(GB_JointSelection_3);
        LE_115->setObjectName(QStringLiteral("LE_115"));
        LE_115->setGeometry(QRect(410, 360, 91, 16));
        LE_115->setFont(font4);
        label_37 = new QLabel(GB_JointSelection_3);
        label_37->setObjectName(QStringLiteral("label_37"));
        label_37->setGeometry(QRect(250, 380, 161, 16));
        LE_116 = new QLineEdit(GB_JointSelection_3);
        LE_116->setObjectName(QStringLiteral("LE_116"));
        LE_116->setGeometry(QRect(410, 380, 91, 16));
        LE_116->setFont(font4);
        label_38 = new QLabel(GB_JointSelection_3);
        label_38->setObjectName(QStringLiteral("label_38"));
        label_38->setGeometry(QRect(250, 320, 160, 16));
        LE_113_B = new QLineEdit(GB_JointSelection_3);
        LE_113_B->setObjectName(QStringLiteral("LE_113_B"));
        LE_113_B->setGeometry(QRect(410, 320, 90, 16));
        LE_113_B->setFont(font4);
        LE_014 = new QLineEdit(GB_JointSelection_3);
        LE_014->setObjectName(QStringLiteral("LE_014"));
        LE_014->setGeometry(QRect(150, 400, 91, 16));
        LE_014->setFont(font4);
        LE_114 = new QLineEdit(GB_JointSelection_3);
        LE_114->setObjectName(QStringLiteral("LE_114"));
        LE_114->setGeometry(QRect(410, 340, 91, 16));
        LE_114->setFont(font4);
        LE_501 = new QLineEdit(GB_JointSelection_3);
        LE_501->setObjectName(QStringLiteral("LE_501"));
        LE_501->setGeometry(QRect(590, 331, 91, 16));
        LE_501->setFont(font4);
        LE_500 = new QLineEdit(GB_JointSelection_3);
        LE_500->setObjectName(QStringLiteral("LE_500"));
        LE_500->setGeometry(QRect(590, 310, 91, 16));
        LE_500->setFont(font4);
        label_39 = new QLabel(GB_JointSelection_3);
        label_39->setObjectName(QStringLiteral("label_39"));
        label_39->setGeometry(QRect(590, 290, 141, 16));
        label_40 = new QLabel(GB_JointSelection_3);
        label_40->setObjectName(QStringLiteral("label_40"));
        label_40->setGeometry(QRect(570, 50, 16, 16));
        label_41 = new QLabel(GB_JointSelection_3);
        label_41->setObjectName(QStringLiteral("label_41"));
        label_41->setGeometry(QRect(570, 70, 16, 16));
        label_42 = new QLabel(GB_JointSelection_3);
        label_42->setObjectName(QStringLiteral("label_42"));
        label_42->setGeometry(QRect(570, 90, 16, 16));
        label_43 = new QLabel(GB_JointSelection_3);
        label_43->setObjectName(QStringLiteral("label_43"));
        label_43->setGeometry(QRect(570, 175, 16, 16));
        label_44 = new QLabel(GB_JointSelection_3);
        label_44->setObjectName(QStringLiteral("label_44"));
        label_44->setGeometry(QRect(570, 155, 16, 16));
        label_45 = new QLabel(GB_JointSelection_3);
        label_45->setObjectName(QStringLiteral("label_45"));
        label_45->setGeometry(QRect(570, 135, 16, 16));
        label_46 = new QLabel(GB_JointSelection_3);
        label_46->setObjectName(QStringLiteral("label_46"));
        label_46->setGeometry(QRect(570, 260, 16, 16));
        label_47 = new QLabel(GB_JointSelection_3);
        label_47->setObjectName(QStringLiteral("label_47"));
        label_47->setGeometry(QRect(570, 240, 16, 16));
        label_48 = new QLabel(GB_JointSelection_3);
        label_48->setObjectName(QStringLiteral("label_48"));
        label_48->setGeometry(QRect(570, 220, 16, 16));
        label_49 = new QLabel(GB_JointSelection_3);
        label_49->setObjectName(QStringLiteral("label_49"));
        label_49->setGeometry(QRect(520, 308, 61, 20));
        label_50 = new QLabel(GB_JointSelection_3);
        label_50->setObjectName(QStringLiteral("label_50"));
        label_50->setGeometry(QRect(520, 330, 71, 20));
        LE_0002 = new QLineEdit(GB_JointSelection_3);
        LE_0002->setObjectName(QStringLiteral("LE_0002"));
        LE_0002->setGeometry(QRect(150, 100, 91, 16));
        LE_0002->setFont(font4);
        label_19 = new QLabel(GB_JointSelection_3);
        label_19->setObjectName(QStringLiteral("label_19"));
        label_19->setGeometry(QRect(10, 80, 111, 16));
        label_51 = new QLabel(GB_JointSelection_3);
        label_51->setObjectName(QStringLiteral("label_51"));
        label_51->setGeometry(QRect(10, 100, 131, 16));
        label_52 = new QLabel(GB_JointSelection_3);
        label_52->setObjectName(QStringLiteral("label_52"));
        label_52->setGeometry(QRect(10, 260, 141, 16));
        LE_117 = new QLineEdit(GB_JointSelection_3);
        LE_117->setObjectName(QStringLiteral("LE_117"));
        LE_117->setGeometry(QRect(150, 260, 91, 16));
        LE_117->setFont(font4);
        GB_JointSelection_6 = new QGroupBox(HCBSettingDialog);
        GB_JointSelection_6->setObjectName(QStringLiteral("GB_JointSelection_6"));
        GB_JointSelection_6->setGeometry(QRect(10, 264, 511, 61));
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
        BTN_VALVEPOS_ENABLE = new QPushButton(GB_JointSelection_6);
        BTN_VALVEPOS_ENABLE->setObjectName(QStringLiteral("BTN_VALVEPOS_ENABLE"));
        BTN_VALVEPOS_ENABLE->setGeometry(QRect(140, 20, 111, 31));
        BTN_VALVEPOS_ENABLE->setFont(font4);
        BTN_VALVEPOS_ENABLE->setStyleSheet(QStringLiteral(""));
        BTN_ENC_ENABLE = new QPushButton(GB_JointSelection_6);
        BTN_ENC_ENABLE->setObjectName(QStringLiteral("BTN_ENC_ENABLE"));
        BTN_ENC_ENABLE->setGeometry(QRect(18, 20, 111, 31));
        BTN_ENC_ENABLE->setFont(font4);
        BTN_ENC_ENABLE->setStyleSheet(QStringLiteral(""));
        BTN_DISABLE_ALL = new QPushButton(GB_JointSelection_6);
        BTN_DISABLE_ALL->setObjectName(QStringLiteral("BTN_DISABLE_ALL"));
        BTN_DISABLE_ALL->setGeometry(QRect(402, 20, 91, 31));
        BTN_DISABLE_ALL->setFont(font2);
        BTN_DISABLE_ALL->setStyleSheet(QStringLiteral(""));
        line = new QFrame(GB_JointSelection_6);
        line->setObjectName(QStringLiteral("line"));
        line->setGeometry(QRect(378, 15, 16, 41));
        line->setFrameShape(QFrame::VLine);
        line->setFrameShadow(QFrame::Sunken);
        BTN_SOMETHING_ENABLE = new QPushButton(GB_JointSelection_6);
        BTN_SOMETHING_ENABLE->setObjectName(QStringLiteral("BTN_SOMETHING_ENABLE"));
        BTN_SOMETHING_ENABLE->setGeometry(QRect(261, 20, 111, 31));
        BTN_SOMETHING_ENABLE->setFont(font4);
        BTN_SOMETHING_ENABLE->setStyleSheet(QStringLiteral(""));
        BTN_BOARD_TEST_ERRORRESET = new QPushButton(HCBSettingDialog);
        BTN_BOARD_TEST_ERRORRESET->setObjectName(QStringLiteral("BTN_BOARD_TEST_ERRORRESET"));
        BTN_BOARD_TEST_ERRORRESET->setGeometry(QRect(540, 70, 211, 71));
        BTN_SET_BNO = new QPushButton(HCBSettingDialog);
        BTN_SET_BNO->setObjectName(QStringLiteral("BTN_SET_BNO"));
        BTN_SET_BNO->setGeometry(QRect(650, 40, 81, 21));
        BTN_SET_BNO->setFont(font1);
        BTN_SET_BNO->setStyleSheet(QStringLiteral(""));
        LE_TARGET_BNO = new QLineEdit(HCBSettingDialog);
        LE_TARGET_BNO->setObjectName(QStringLiteral("LE_TARGET_BNO"));
        LE_TARGET_BNO->setGeometry(QRect(550, 40, 91, 16));
        LE_TARGET_BNO->setFont(font4);
        BTN_PRESSURE_NULLING = new QPushButton(HCBSettingDialog);
        BTN_PRESSURE_NULLING->setObjectName(QStringLiteral("BTN_PRESSURE_NULLING"));
        BTN_PRESSURE_NULLING->setEnabled(false);
        BTN_PRESSURE_NULLING->setGeometry(QRect(650, 200, 101, 41));
        BTN_TORQUEFORCE_NULLING = new QPushButton(HCBSettingDialog);
        BTN_TORQUEFORCE_NULLING->setObjectName(QStringLiteral("BTN_TORQUEFORCE_NULLING"));
        BTN_TORQUEFORCE_NULLING->setGeometry(QRect(650, 152, 101, 41));
        BTN_TORQUEFORCE_NULLING_ONE = new QPushButton(HCBSettingDialog);
        BTN_TORQUEFORCE_NULLING_ONE->setObjectName(QStringLiteral("BTN_TORQUEFORCE_NULLING_ONE"));
        BTN_TORQUEFORCE_NULLING_ONE->setGeometry(QRect(540, 152, 101, 41));
        BTN_PRESSURE_NULLING_ONE = new QPushButton(HCBSettingDialog);
        BTN_PRESSURE_NULLING_ONE->setObjectName(QStringLiteral("BTN_PRESSURE_NULLING_ONE"));
        BTN_PRESSURE_NULLING_ONE->setEnabled(false);
        BTN_PRESSURE_NULLING_ONE->setGeometry(QRect(540, 200, 101, 41));

        retranslateUi(HCBSettingDialog);

        QMetaObject::connectSlotsByName(HCBSettingDialog);
    } // setupUi

    void retranslateUi(QWidget *HCBSettingDialog)
    {
        HCBSettingDialog->setWindowTitle(QApplication::translate("HCBSettingDialog", "Form", Q_NULLPTR));
        GB_JointSelection->setTitle(QApplication::translate("HCBSettingDialog", "Initial Setting", Q_NULLPTR));
        BTN_CAN_CHECK->setText(QApplication::translate("HCBSettingDialog", "CAN Check", Q_NULLPTR));
        BTN_FET_ONOFF->setText(QApplication::translate("HCBSettingDialog", "All FET Enable", Q_NULLPTR));
        BTN_FET_ONOFF_2->setText(QApplication::translate("HCBSettingDialog", "CAN RESET", Q_NULLPTR));
        BTN_CAN_CHANNEL_ARRANGE->setText(QApplication::translate("HCBSettingDialog", "CAN CH Arrange", Q_NULLPTR));
        GB_JointSelection_2->setTitle(QApplication::translate("HCBSettingDialog", "Joint Selection", Q_NULLPTR));
        BTN_ASK_EVERYTHING->setText(QApplication::translate("HCBSettingDialog", "ASK \n"
"EVERYTHING", Q_NULLPTR));
        BTN_ENC_ZERO->setText(QApplication::translate("HCBSettingDialog", "ENCODER ZERO", Q_NULLPTR));
        BTN_FINDHOME_ALL->setText(QApplication::translate("HCBSettingDialog", "FIND HOME ALL", Q_NULLPTR));
        BTN_FINDHOME_ONEJOINT->setText(QApplication::translate("HCBSettingDialog", "FIND HOME\n"
" One Joint ", Q_NULLPTR));
        RB_RHR->setText(QApplication::translate("HCBSettingDialog", "RHR", Q_NULLPTR));
        RB_LHR->setText(QApplication::translate("HCBSettingDialog", "LHR", Q_NULLPTR));
        RB_RHY->setText(QApplication::translate("HCBSettingDialog", "RHY", Q_NULLPTR));
        RB_LHY->setText(QApplication::translate("HCBSettingDialog", "LHY", Q_NULLPTR));
        RB_RHP->setText(QApplication::translate("HCBSettingDialog", "RHP", Q_NULLPTR));
        RB_LHP->setText(QApplication::translate("HCBSettingDialog", "LHP", Q_NULLPTR));
        RB_RKN->setText(QApplication::translate("HCBSettingDialog", "RKN", Q_NULLPTR));
        RB_LKN->setText(QApplication::translate("HCBSettingDialog", "LKN", Q_NULLPTR));
        RB_RAP->setText(QApplication::translate("HCBSettingDialog", "RAP", Q_NULLPTR));
        RB_LAP->setText(QApplication::translate("HCBSettingDialog", "LAP", Q_NULLPTR));
        RB_RAR->setText(QApplication::translate("HCBSettingDialog", "RAR", Q_NULLPTR));
        RB_LAR->setText(QApplication::translate("HCBSettingDialog", "LAR", Q_NULLPTR));
        RB_WST->setText(QApplication::translate("HCBSettingDialog", "WST", Q_NULLPTR));
        BTN_FINDHOME_SECOND_STAGE->setText(QApplication::translate("HCBSettingDialog", "FIND HOME\n"
"(2nd Stage)", Q_NULLPTR));
        BTN_FINDHOME_FIRST_STAGE->setText(QApplication::translate("HCBSettingDialog", "FIND HOME\n"
"(1st Stage)", Q_NULLPTR));
        GB_JointSelection_3->setTitle(QApplication::translate("HCBSettingDialog", "Parameter Setting", Q_NULLPTR));
        BTN_PARAMETER_SET_ALL->setText(QApplication::translate("HCBSettingDialog", "All Parameter Set!", Q_NULLPTR));
        BTN_SHOW_CURRENT_PARA->setText(QApplication::translate("HCBSettingDialog", "SHOW CURRENT", Q_NULLPTR));
        label_2->setText(QApplication::translate("HCBSettingDialog", "OperationMode", Q_NULLPTR));
        label_3->setText(QApplication::translate("HCBSettingDialog", "CanFrequency", Q_NULLPTR));
        label_4->setText(QApplication::translate("HCBSettingDialog", "ControlMode", Q_NULLPTR));
        label_5->setText(QApplication::translate("HCBSettingDialog", "JointEncDir", Q_NULLPTR));
        label_6->setText(QApplication::translate("HCBSettingDialog", "ValveInputDir", Q_NULLPTR));
        label_7->setText(QApplication::translate("HCBSettingDialog", "ValveEncDir", Q_NULLPTR));
        label_8->setText(QApplication::translate("HCBSettingDialog", "BoardInputVoltage", Q_NULLPTR));
        label_9->setText(QApplication::translate("HCBSettingDialog", "ValveOperationVoltage", Q_NULLPTR));
        label_10->setText(QApplication::translate("HCBSettingDialog", "PID VALVE", Q_NULLPTR));
        label_11->setText(QApplication::translate("HCBSettingDialog", "PID JOINT", Q_NULLPTR));
        label_12->setText(QApplication::translate("HCBSettingDialog", "PID JOINT_F", Q_NULLPTR));
        label_13->setText(QApplication::translate("HCBSettingDialog", "ValveDeadZone +", Q_NULLPTR));
        label_14->setText(QApplication::translate("HCBSettingDialog", "ValveCenterPos", Q_NULLPTR));
        label_15->setText(QApplication::translate("HCBSettingDialog", "VelCompensationGain", Q_NULLPTR));
        label_16->setText(QApplication::translate("HCBSettingDialog", "ComplianceGain", Q_NULLPTR));
        label_17->setText(QApplication::translate("HCBSettingDialog", "ValveFeedforward", Q_NULLPTR));
        label_18->setText(QApplication::translate("HCBSettingDialog", "BulkModulus", Q_NULLPTR));
        label_20->setText(QApplication::translate("HCBSettingDialog", "PistionArea A", Q_NULLPTR));
        label_21->setText(QApplication::translate("HCBSettingDialog", "SupplyPressure", Q_NULLPTR));
        label_22->setText(QApplication::translate("HCBSettingDialog", "JointEncLimit +", Q_NULLPTR));
        label_23->setText(QApplication::translate("HCBSettingDialog", "PistonStroke", Q_NULLPTR));
        label_24->setText(QApplication::translate("HCBSettingDialog", "ValvePositionLimit +", Q_NULLPTR));
        label_25->setText(QApplication::translate("HCBSettingDialog", "EncoderPPPosition", Q_NULLPTR));
        label_26->setText(QApplication::translate("HCBSettingDialog", "SensorPulsePerForce", Q_NULLPTR));
        label_27->setText(QApplication::translate("HCBSettingDialog", "PulsePerPressure_A", Q_NULLPTR));
        label_28->setText(QApplication::translate("HCBSettingDialog", "ConstantFriction", Q_NULLPTR));
        label_29->setText(QApplication::translate("HCBSettingDialog", "ReturnPressure", Q_NULLPTR));
        label_30->setText(QApplication::translate("HCBSettingDialog", "ValveDeadZone --", Q_NULLPTR));
        label_31->setText(QApplication::translate("HCBSettingDialog", "ChamberVolume A", Q_NULLPTR));
        label_32->setText(QApplication::translate("HCBSettingDialog", "ChamberVolume B", Q_NULLPTR));
        label_33->setText(QApplication::translate("HCBSettingDialog", "PistionArea B", Q_NULLPTR));
        label_34->setText(QApplication::translate("HCBSettingDialog", "JointEncLimit --", Q_NULLPTR));
        label_35->setText(QApplication::translate("HCBSettingDialog", "ValvePositionLimit --", Q_NULLPTR));
        label_36->setText(QApplication::translate("HCBSettingDialog", "HomePos Offset", Q_NULLPTR));
        label_37->setText(QApplication::translate("HCBSettingDialog", "HomePos Valve Opening", Q_NULLPTR));
        label_38->setText(QApplication::translate("HCBSettingDialog", "PulsePerPressure_B", Q_NULLPTR));
        label_39->setText(QApplication::translate("HCBSettingDialog", "Joint Spring & Damper", Q_NULLPTR));
        label_40->setText(QApplication::translate("HCBSettingDialog", "P", Q_NULLPTR));
        label_41->setText(QApplication::translate("HCBSettingDialog", "I", Q_NULLPTR));
        label_42->setText(QApplication::translate("HCBSettingDialog", "D", Q_NULLPTR));
        label_43->setText(QApplication::translate("HCBSettingDialog", "D", Q_NULLPTR));
        label_44->setText(QApplication::translate("HCBSettingDialog", "I", Q_NULLPTR));
        label_45->setText(QApplication::translate("HCBSettingDialog", "P", Q_NULLPTR));
        label_46->setText(QApplication::translate("HCBSettingDialog", "D", Q_NULLPTR));
        label_47->setText(QApplication::translate("HCBSettingDialog", "I", Q_NULLPTR));
        label_48->setText(QApplication::translate("HCBSettingDialog", "P", Q_NULLPTR));
        label_49->setText(QApplication::translate("HCBSettingDialog", "Spring(K)", Q_NULLPTR));
        label_50->setText(QApplication::translate("HCBSettingDialog", "Damper(D)", Q_NULLPTR));
        label_19->setText(QApplication::translate("HCBSettingDialog", "Sensing Mode", Q_NULLPTR));
        label_51->setText(QApplication::translate("HCBSettingDialog", "CurrentControlMode", Q_NULLPTR));
        label_52->setText(QApplication::translate("HCBSettingDialog", "VariableSupPresMode ", Q_NULLPTR));
        GB_JointSelection_6->setTitle(QApplication::translate("HCBSettingDialog", "Request Datas Enable/Disable", Q_NULLPTR));
        BTN_VALVEPOS_ENABLE->setText(QApplication::translate("HCBSettingDialog", "VALVEPOS ENABLE", Q_NULLPTR));
        BTN_ENC_ENABLE->setText(QApplication::translate("HCBSettingDialog", "ENCODER ENABLE", Q_NULLPTR));
        BTN_DISABLE_ALL->setText(QApplication::translate("HCBSettingDialog", "DISABLE ALL", Q_NULLPTR));
        BTN_SOMETHING_ENABLE->setText(QApplication::translate("HCBSettingDialog", "Other Info. ENABLE", Q_NULLPTR));
        BTN_BOARD_TEST_ERRORRESET->setText(QApplication::translate("HCBSettingDialog", "HCB Error Reset TEST", Q_NULLPTR));
        BTN_SET_BNO->setText(QApplication::translate("HCBSettingDialog", "BNO SET", Q_NULLPTR));
        LE_TARGET_BNO->setText(QApplication::translate("HCBSettingDialog", "0", Q_NULLPTR));
        BTN_PRESSURE_NULLING->setText(QApplication::translate("HCBSettingDialog", "Pressure\n"
"Nulling(ALL)", Q_NULLPTR));
        BTN_TORQUEFORCE_NULLING->setText(QApplication::translate("HCBSettingDialog", "Torque/Force\n"
"Nulling(ALL)", Q_NULLPTR));
        BTN_TORQUEFORCE_NULLING_ONE->setText(QApplication::translate("HCBSettingDialog", "Torque/Force\n"
"Nulling(One)", Q_NULLPTR));
        BTN_PRESSURE_NULLING_ONE->setText(QApplication::translate("HCBSettingDialog", "Pressure\n"
"Nulling(One)", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class HCBSettingDialog: public Ui_HCBSettingDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_HCBSETTINGDIALOG_H
