/********************************************************************************
** Form generated from reading UI file 'GUIMainWindow.ui'
**
** Created by: Qt User Interface Compiler version 5.9.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_GUIMAINWINDOW_H
#define UI_GUIMAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_GUIMainWindow
{
public:
    QAction *actionLAN;
    QAction *actionPODOAL;
    QAction *actionJOINT;
    QAction *actionSENSOR;
    QAction *actionDUMMY;
    QAction *actionJOYSTICK;
    QWidget *centralWidget;
    QTabWidget *MAIN_TAB;
    QGroupBox *groupBox_2;
    QLabel *LB_VOLTAGE;
    QFrame *line;
    QLabel *LB_CURRENT;
    QPushButton *BTN_REF_ENABLE;
    QPushButton *BTN_REF_DISABLE;
    QPushButton *BTN_CAN_CHECK;
    QPushButton *BTN_ENC_ENABLE;
    QPushButton *BTN_ENC_DISABLE;
    QPushButton *BTN_FINDHOME_ALL;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *GUIMainWindow)
    {
        if (GUIMainWindow->objectName().isEmpty())
            GUIMainWindow->setObjectName(QStringLiteral("GUIMainWindow"));
        GUIMainWindow->resize(910, 880);
        actionLAN = new QAction(GUIMainWindow);
        actionLAN->setObjectName(QStringLiteral("actionLAN"));
        actionLAN->setCheckable(true);
        QIcon icon;
        icon.addFile(QStringLiteral("../../share/GUI/icon/LAN_OFF_BLACK.png"), QSize(), QIcon::Normal, QIcon::Off);
        icon.addFile(QStringLiteral("../../share/GUI/icon/LAN_OFF_BLUE.png"), QSize(), QIcon::Normal, QIcon::On);
        actionLAN->setIcon(icon);
        actionPODOAL = new QAction(GUIMainWindow);
        actionPODOAL->setObjectName(QStringLiteral("actionPODOAL"));
        actionPODOAL->setCheckable(true);
        QIcon icon1;
        icon1.addFile(QStringLiteral("../../share/GUI/icon/MODULE_BLACK.png"), QSize(), QIcon::Normal, QIcon::Off);
        icon1.addFile(QStringLiteral("../../share/GUI/icon/MODULE_BLUE.png"), QSize(), QIcon::Normal, QIcon::On);
        actionPODOAL->setIcon(icon1);
        actionJOINT = new QAction(GUIMainWindow);
        actionJOINT->setObjectName(QStringLiteral("actionJOINT"));
        actionJOINT->setCheckable(true);
        QIcon icon2;
        icon2.addFile(QStringLiteral("../../share/GUI/icon/JOINT_BLACK.png"), QSize(), QIcon::Normal, QIcon::Off);
        icon2.addFile(QStringLiteral("../../share/GUI/icon/JOINT_BLUE.png"), QSize(), QIcon::Normal, QIcon::On);
        actionJOINT->setIcon(icon2);
        actionSENSOR = new QAction(GUIMainWindow);
        actionSENSOR->setObjectName(QStringLiteral("actionSENSOR"));
        actionSENSOR->setCheckable(true);
        QIcon icon3;
        icon3.addFile(QStringLiteral("../../share/GUI/icon/SENSOR_BLACK.png"), QSize(), QIcon::Normal, QIcon::Off);
        icon3.addFile(QStringLiteral("../../share/GUI/icon/SENSOR_BLUE.png"), QSize(), QIcon::Normal, QIcon::On);
        actionSENSOR->setIcon(icon3);
        actionDUMMY = new QAction(GUIMainWindow);
        actionDUMMY->setObjectName(QStringLiteral("actionDUMMY"));
        actionDUMMY->setEnabled(false);
        actionJOYSTICK = new QAction(GUIMainWindow);
        actionJOYSTICK->setObjectName(QStringLiteral("actionJOYSTICK"));
        QIcon icon4;
        icon4.addFile(QStringLiteral("../../share/GUI/icon/SIMULATOR_BLACK.png"), QSize(), QIcon::Normal, QIcon::Off);
        icon4.addFile(QStringLiteral("../../share/GUI/icon/SIMULATOR_BLUE.png"), QSize(), QIcon::Normal, QIcon::On);
        actionJOYSTICK->setIcon(icon4);
        centralWidget = new QWidget(GUIMainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        MAIN_TAB = new QTabWidget(centralWidget);
        MAIN_TAB->setObjectName(QStringLiteral("MAIN_TAB"));
        MAIN_TAB->setGeometry(QRect(10, 70, 780, 780));
        groupBox_2 = new QGroupBox(centralWidget);
        groupBox_2->setObjectName(QStringLiteral("groupBox_2"));
        groupBox_2->setGeometry(QRect(10, 10, 181, 51));
        groupBox_2->setStyleSheet(QLatin1String("QGroupBox {\n"
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
"\n"
""));
        LB_VOLTAGE = new QLabel(groupBox_2);
        LB_VOLTAGE->setObjectName(QStringLiteral("LB_VOLTAGE"));
        LB_VOLTAGE->setGeometry(QRect(10, 20, 71, 21));
        QFont font;
        font.setPointSize(13);
        LB_VOLTAGE->setFont(font);
        LB_VOLTAGE->setAlignment(Qt::AlignCenter);
        line = new QFrame(groupBox_2);
        line->setObjectName(QStringLiteral("line"));
        line->setGeometry(QRect(80, 20, 20, 21));
        line->setFrameShape(QFrame::VLine);
        line->setFrameShadow(QFrame::Sunken);
        LB_CURRENT = new QLabel(groupBox_2);
        LB_CURRENT->setObjectName(QStringLiteral("LB_CURRENT"));
        LB_CURRENT->setGeometry(QRect(100, 20, 71, 21));
        LB_CURRENT->setFont(font);
        LB_CURRENT->setAlignment(Qt::AlignCenter);
        BTN_REF_ENABLE = new QPushButton(centralWidget);
        BTN_REF_ENABLE->setObjectName(QStringLiteral("BTN_REF_ENABLE"));
        BTN_REF_ENABLE->setGeometry(QRect(620, 10, 81, 51));
        QFont font1;
        font1.setPointSize(8);
        BTN_REF_ENABLE->setFont(font1);
        BTN_REF_DISABLE = new QPushButton(centralWidget);
        BTN_REF_DISABLE->setObjectName(QStringLiteral("BTN_REF_DISABLE"));
        BTN_REF_DISABLE->setGeometry(QRect(710, 10, 81, 51));
        BTN_REF_DISABLE->setFont(font1);
        BTN_CAN_CHECK = new QPushButton(centralWidget);
        BTN_CAN_CHECK->setObjectName(QStringLiteral("BTN_CAN_CHECK"));
        BTN_CAN_CHECK->setGeometry(QRect(260, 10, 81, 51));
        BTN_CAN_CHECK->setFont(font1);
        BTN_ENC_ENABLE = new QPushButton(centralWidget);
        BTN_ENC_ENABLE->setObjectName(QStringLiteral("BTN_ENC_ENABLE"));
        BTN_ENC_ENABLE->setGeometry(QRect(350, 10, 81, 51));
        BTN_ENC_ENABLE->setFont(font1);
        BTN_ENC_DISABLE = new QPushButton(centralWidget);
        BTN_ENC_DISABLE->setObjectName(QStringLiteral("BTN_ENC_DISABLE"));
        BTN_ENC_DISABLE->setGeometry(QRect(440, 10, 81, 51));
        BTN_ENC_DISABLE->setFont(font1);
        BTN_FINDHOME_ALL = new QPushButton(centralWidget);
        BTN_FINDHOME_ALL->setObjectName(QStringLiteral("BTN_FINDHOME_ALL"));
        BTN_FINDHOME_ALL->setGeometry(QRect(530, 10, 81, 51));
        BTN_FINDHOME_ALL->setFont(font1);
        GUIMainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(GUIMainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 910, 19));
        GUIMainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(GUIMainWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(mainToolBar->sizePolicy().hasHeightForWidth());
        mainToolBar->setSizePolicy(sizePolicy);
        mainToolBar->setStyleSheet(QStringLiteral("background-color: rgb(202, 203, 210);"));
        mainToolBar->setMovable(false);
        mainToolBar->setIconSize(QSize(80, 80));
        GUIMainWindow->addToolBar(Qt::LeftToolBarArea, mainToolBar);
        statusBar = new QStatusBar(GUIMainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        GUIMainWindow->setStatusBar(statusBar);

        mainToolBar->addAction(actionLAN);
        mainToolBar->addAction(actionPODOAL);
        mainToolBar->addAction(actionDUMMY);
        mainToolBar->addAction(actionJOINT);
        mainToolBar->addAction(actionSENSOR);
        mainToolBar->addAction(actionJOYSTICK);
        mainToolBar->addSeparator();

        retranslateUi(GUIMainWindow);

        MAIN_TAB->setCurrentIndex(-1);


        QMetaObject::connectSlotsByName(GUIMainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *GUIMainWindow)
    {
        GUIMainWindow->setWindowTitle(QApplication::translate("GUIMainWindow", "PODO GUI -- Rainbow Robotics", Q_NULLPTR));
        actionLAN->setText(QApplication::translate("GUIMainWindow", "LAN Connection", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        actionLAN->setToolTip(QApplication::translate("GUIMainWindow", "LAN Connection", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        actionPODOAL->setText(QApplication::translate("GUIMainWindow", "PODOAL Control", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        actionPODOAL->setToolTip(QApplication::translate("GUIMainWindow", "PODOAL Control", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        actionJOINT->setText(QApplication::translate("GUIMainWindow", "JOINT Pannel", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        actionJOINT->setToolTip(QApplication::translate("GUIMainWindow", "JOINT Pannel", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        actionSENSOR->setText(QApplication::translate("GUIMainWindow", "SENSOR Pannel", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        actionSENSOR->setToolTip(QApplication::translate("GUIMainWindow", "SENSOR Pannel", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        actionDUMMY->setText(QString());
        actionJOYSTICK->setText(QApplication::translate("GUIMainWindow", "MODEL Pannel", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        actionJOYSTICK->setToolTip(QApplication::translate("GUIMainWindow", "MODEL Pannel", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        groupBox_2->setTitle(QApplication::translate("GUIMainWindow", "Power", Q_NULLPTR));
        LB_VOLTAGE->setText(QApplication::translate("GUIMainWindow", "NC", Q_NULLPTR));
        LB_CURRENT->setText(QApplication::translate("GUIMainWindow", "NC", Q_NULLPTR));
        BTN_REF_ENABLE->setText(QApplication::translate("GUIMainWindow", "Ref. Enable", Q_NULLPTR));
        BTN_REF_DISABLE->setText(QApplication::translate("GUIMainWindow", "Ref. Disable", Q_NULLPTR));
        BTN_CAN_CHECK->setText(QApplication::translate("GUIMainWindow", "CAN Check", Q_NULLPTR));
        BTN_ENC_ENABLE->setText(QApplication::translate("GUIMainWindow", "Enc. Enable", Q_NULLPTR));
        BTN_ENC_DISABLE->setText(QApplication::translate("GUIMainWindow", "Enc. Disable", Q_NULLPTR));
        BTN_FINDHOME_ALL->setText(QApplication::translate("GUIMainWindow", "All Joint\n"
"Find Home", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class GUIMainWindow: public Ui_GUIMainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_GUIMAINWINDOW_H
