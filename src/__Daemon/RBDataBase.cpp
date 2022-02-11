#include "RBDataBase.h"

RBDataBase::RBDataBase()
{
    dbCore = QSqlDatabase::addDatabase("QSQLITE","ROBOT_DB");
}

void RBDataBase::SetFilename(QString name){
    filename = name;
}

bool RBDataBase::OpenDB(){
    dbCore.setDatabaseName(filename);
    if(!dbCore.open()){
        FILE_LOG(logERROR) << "Can't open database file [" << filename.toStdString().data() << "]";
        return false;
    }

    QString strQuery;
    QSqlQuery query(dbCore);
    query.setForwardOnly(true);

    // General ----
    query.exec("SELECT * from General");
    if(query.next()){
        _DB_GENERAL.VERSION         = query.value(0).toInt();
        _DB_GENERAL.NO_OF_AL        = query.value(1).toInt();
        _DB_GENERAL.NO_OF_COMM_CH   = query.value(2).toInt();
        _DB_GENERAL.NO_OF_MC        = query.value(3).toInt();
        _DB_GENERAL.NO_OF_PC        = query.value(4).toInt();
        _DB_GENERAL.NO_OF_FT        = query.value(5).toInt();
        _DB_GENERAL.NO_OF_IMU       = query.value(6).toInt();
        _DB_GENERAL.NO_OF_SP        = query.value(7).toInt();
        _DB_GENERAL.NO_OF_OF        = query.value(8).toInt();
    }

    // PODO AL ----
    strQuery.sprintf("SELECT * from AL");
    query.exec(strQuery);
    for(int i=0; i<_DB_GENERAL.NO_OF_AL; i++){
        if(query.next()){
            _DB_AL[i].ALName          = query.value(0).toString();
            _DB_AL[i].FileName        = query.value(1).toString();
            _DB_AL[i].PathName        = query.value(2).toString();
        }
    }

    // Motor Controller ----
    strQuery.sprintf("SELECT * from MotionController");
    query.exec(strQuery);
    for(int i=0; i<_DB_GENERAL.NO_OF_MC; i++){
        if(query.next()){
            _DB_MC[i].BOARD_ID          = query.value(0).toInt();
            _DB_MC[i].BOARD_NAME        = query.value(1).toString();
            _DB_MC[i].CAN_CHANNEL       = query.value(2).toInt();
            _DB_MC[i].ACTUATOR_TYPE     = query.value(3).toString();

            _DB_MC[i].PULSE_PER_POSITION    = query.value(4).toDouble();
            _DB_MC[i].PULSE_PER_FORCETORQUE = query.value(5).toDouble();
            _DB_MC[i].PULSE_PER_PRESSURE    = query.value(6).toDouble();

            _DB_MC[i].ID_SEND_GENERAL   = query.value(7).toInt();
            _DB_MC[i].ID_SEND_POSVEL    = query.value(8).toInt();
            _DB_MC[i].ID_SEND_VALVEPOS  = query.value(9).toInt();

            _DB_MC[i].ID_RCV_GENERAL    = query.value(10).toInt();
            _DB_MC[i].ID_RCV_POSVEL     = query.value(11).toInt();
            _DB_MC[i].ID_RCV_VALVEPOS   = query.value(12).toInt();
            _DB_MC[i].ID_RCV_OTHERINFO  = query.value(13).toInt();
        }
    }

    // Pump Controller ---- (Buyoun, 20190710)
    strQuery.sprintf("SELECT * from PumpController");
    query.exec(strQuery);
    for(int i=0; i<_DB_GENERAL.NO_OF_PC; i++){
        if(query.next()){
            _DB_PC[i].BOARD_ID          = query.value(0).toInt();
            _DB_PC[i].CAN_CHANNEL       = query.value(1).toInt();

            _DB_PC[i].ID_SEND_GENERAL   = query.value(2).toInt();
            _DB_PC[i].ID_SEND_VELOCITY  = query.value(3).toInt();
            _DB_PC[i].ID_SEND_PRESSURE  = query.value(4).toInt();

            _DB_PC[i].ID_RCV_GENERAL    = query.value(5).toInt();
            _DB_PC[i].ID_RCV_VELOCITY   = query.value(6).toInt();
            _DB_PC[i].ID_RCV_PRESSURE   = query.value(7).toInt();

        }
    }

    // FT Sensor ----
    strQuery.sprintf("SELECT * from FTSensor");
    query.exec(strQuery);
    for(int i=0; i<_DB_GENERAL.NO_OF_FT; i++){
        if(query.next()){
            _DB_FT[i].BOARD_ID        = query.value(0).toInt();
            _DB_FT[i].BOARD_NAME      = query.value(1).toString();
            _DB_FT[i].CAN_CHANNEL     = query.value(2).toInt();
            _DB_FT[i].ID_SEND_CMD     = query.value(3).toInt();
            _DB_FT[i].ID_RCV_GENERAL  = query.value(4).toInt();
            _DB_FT[i].ID_RCV_MXMYFZ   = query.value(5).toInt();
            _DB_FT[i].ID_RCV_FXFYMZ   = query.value(6).toInt();

        }
    }

    // IMU Sensor ----
    strQuery.sprintf("SELECT * from IMUSensor");
    query.exec(strQuery);
    for(int i=0; i<_DB_GENERAL.NO_OF_IMU; i++){
        if(query.next()){
            _DB_IMU[i].BOARD_ID            = query.value(0).toInt();
            _DB_IMU[i].BOARD_NAME          = query.value(1).toString();
            _DB_IMU[i].CAN_CHANNEL         = query.value(2).toInt();
            _DB_IMU[i].ID_SEND_DATA        = query.value(3).toInt();
            _DB_IMU[i].ID_RCV_GENERAL      = query.value(4).toInt();
            _DB_IMU[i].ID_RCV_DATA_QUAT    = query.value(5).toInt();
            _DB_IMU[i].ID_RCV_DATA_LOCAL_X = query.value(6).toInt();
            _DB_IMU[i].ID_RCV_DATA_LOCAL_Y = query.value(7).toInt();
            _DB_IMU[i].ID_RCV_DATA_LOCAL_Z = query.value(8).toInt();
        }
    }

    // Smart Power Controller ----
    strQuery.sprintf("SELECT * from SmartPower");
    query.exec(strQuery);
    for(int i=0; i<_DB_GENERAL.NO_OF_SP; i++){
        if(query.next()){
            _DB_SP[i].BOARD_ID      = query.value(0).toInt();
            _DB_SP[i].BOARD_NAME    = query.value(1).toString();
            _DB_SP[i].CAN_CHANNEL   = query.value(2).toInt();
            _DB_SP[i].ID_RCV_DATA   = query.value(3).toInt();
            _DB_SP[i].ID_RCV_INFO   = query.value(5).toInt();
            _DB_SP[i].ID_SEND_GENERAL = query.value(6).toInt();
        }
    }

    // Optic Flow Sensor ----
    strQuery.sprintf("SELECT * from OpticFlowSensor");
    query.exec(strQuery);
    for(int i=0; i<_DB_GENERAL.NO_OF_OF; i++){
        if(query.next()){
            _DB_OF[i].BOARD_ID      = query.value(0).toInt();
            _DB_OF[i].BOARD_NAME    = query.value(1).toString();
            _DB_OF[i].SENSOR_ID     = query.value(2).toInt();
            _DB_OF[i].CAN_CHANNEL   = query.value(3).toInt();
            _DB_OF[i].ID_RCV_DATA   = query.value(4).toInt();
            _DB_OF[i].ID_RCV_INFO   = query.value(5).toInt();
        }
    }

    dbCore.close();
    return true;
}

bool RBDataBase::UpdateDB_CAN_Channel_MC(int BOARD_ID, int CAN_CH){
    dbCore.setDatabaseName(filename);
    if(!dbCore.open()){
        FILE_LOG(logERROR) << "Can't open database file [" << filename.toStdString().data() << "]";
        return false;
    }

    QSqlQuery query(dbCore);

    query.prepare("UPDATE MotionController SET CAN_CH=:CAN_CH WHERE Board_ID=:BOARD_ID;");
    query.bindValue(":CAN_CH", CAN_CH);
    query.bindValue(":BOARD_ID", BOARD_ID);
    query.exec();

    dbCore.close();
    return true;
}

bool RBDataBase::UpdateDB_CAN_Channel_PC(int BOARD_ID, int CAN_CH){
    dbCore.setDatabaseName(filename);
    if(!dbCore.open()){
        FILE_LOG(logERROR) << "Can't open database file [" << filename.toStdString().data() << "]";
        return false;
    }

    QSqlQuery query(dbCore);

    query.prepare("UPDATE PumpController SET CAN_CH=:CAN_CH WHERE Board_ID=:BOARD_ID;");
    query.bindValue(":CAN_CH", CAN_CH);
    query.bindValue(":BOARD_ID", BOARD_ID);
    query.exec();

    dbCore.close();
    return true;
}

bool RBDataBase::UpdateDB_CAN_Channel_IMU(int BOARD_ID, int CAN_CH){
    dbCore.setDatabaseName(filename);
    if(!dbCore.open()){
        FILE_LOG(logERROR) << "Can't open database file [" << filename.toStdString().data() << "]";
        return false;
    }

    QSqlQuery query(dbCore);

    query.prepare("UPDATE IMUSensor SET CAN_CH=:CAN_CH WHERE Board_ID=:BOARD_ID;");
    query.bindValue(":CAN_CH", CAN_CH);
    query.bindValue(":BOARD_ID", BOARD_ID);
    query.exec();

    dbCore.close();
    return true;
}

bool RBDataBase::UpdateDB_CAN_Channel_FT(int BOARD_ID, int CAN_CH){
    dbCore.setDatabaseName(filename);
    if(!dbCore.open()){
        FILE_LOG(logERROR) << "Can't open database file [" << filename.toStdString().data() << "]";
        return false;
    }

    QSqlQuery query(dbCore);

    query.prepare("UPDATE FTSensor SET CAN_CH=:CAN_CH WHERE Board_ID=:BOARD_ID;");
    query.bindValue(":CAN_CH", CAN_CH);
    query.bindValue(":BOARD_ID", BOARD_ID);
    query.exec();

    dbCore.close();
    return true;
}
