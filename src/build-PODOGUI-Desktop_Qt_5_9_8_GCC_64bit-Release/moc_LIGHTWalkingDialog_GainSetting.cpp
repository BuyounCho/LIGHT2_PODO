/****************************************************************************
** Meta object code from reading C++ file 'LIGHTWalkingDialog_GainSetting.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.8)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../PODOGUI/LIGHTWalkingDialog_GainSetting.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'LIGHTWalkingDialog_GainSetting.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.8. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_LIGHTWalkingDialog_GainSetting_t {
    QByteArrayData data[10];
    char stringdata0[281];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_LIGHTWalkingDialog_GainSetting_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_LIGHTWalkingDialog_GainSetting_t qt_meta_stringdata_LIGHTWalkingDialog_GainSetting = {
    {
QT_MOC_LITERAL(0, 0, 30), // "LIGHTWalkingDialog_GainSetting"
QT_MOC_LITERAL(1, 31, 27), // "on_BTN_DSP_GAIN_SET_clicked"
QT_MOC_LITERAL(2, 59, 0), // ""
QT_MOC_LITERAL(3, 60, 28), // "on_BTN_RSSP_GAIN_SET_clicked"
QT_MOC_LITERAL(4, 89, 28), // "on_BTN_LSSP_GAIN_SET_clicked"
QT_MOC_LITERAL(5, 118, 29), // "on_BTN_FLOAT_GAIN_SET_clicked"
QT_MOC_LITERAL(6, 148, 33), // "on_BTN_JOINT_IMPEDANCE_RF_cli..."
QT_MOC_LITERAL(7, 182, 33), // "on_BTN_JOINT_IMPEDANCE_LF_cli..."
QT_MOC_LITERAL(8, 216, 32), // "on_BTN_COM_LEAD_GAIN_SET_clicked"
QT_MOC_LITERAL(9, 249, 31) // "on_BTN_CHECK_PARAMETERS_clicked"

    },
    "LIGHTWalkingDialog_GainSetting\0"
    "on_BTN_DSP_GAIN_SET_clicked\0\0"
    "on_BTN_RSSP_GAIN_SET_clicked\0"
    "on_BTN_LSSP_GAIN_SET_clicked\0"
    "on_BTN_FLOAT_GAIN_SET_clicked\0"
    "on_BTN_JOINT_IMPEDANCE_RF_clicked\0"
    "on_BTN_JOINT_IMPEDANCE_LF_clicked\0"
    "on_BTN_COM_LEAD_GAIN_SET_clicked\0"
    "on_BTN_CHECK_PARAMETERS_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_LIGHTWalkingDialog_GainSetting[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   54,    2, 0x08 /* Private */,
       3,    0,   55,    2, 0x08 /* Private */,
       4,    0,   56,    2, 0x08 /* Private */,
       5,    0,   57,    2, 0x08 /* Private */,
       6,    0,   58,    2, 0x08 /* Private */,
       7,    0,   59,    2, 0x08 /* Private */,
       8,    0,   60,    2, 0x08 /* Private */,
       9,    0,   61,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void LIGHTWalkingDialog_GainSetting::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        LIGHTWalkingDialog_GainSetting *_t = static_cast<LIGHTWalkingDialog_GainSetting *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->on_BTN_DSP_GAIN_SET_clicked(); break;
        case 1: _t->on_BTN_RSSP_GAIN_SET_clicked(); break;
        case 2: _t->on_BTN_LSSP_GAIN_SET_clicked(); break;
        case 3: _t->on_BTN_FLOAT_GAIN_SET_clicked(); break;
        case 4: _t->on_BTN_JOINT_IMPEDANCE_RF_clicked(); break;
        case 5: _t->on_BTN_JOINT_IMPEDANCE_LF_clicked(); break;
        case 6: _t->on_BTN_COM_LEAD_GAIN_SET_clicked(); break;
        case 7: _t->on_BTN_CHECK_PARAMETERS_clicked(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObject LIGHTWalkingDialog_GainSetting::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_LIGHTWalkingDialog_GainSetting.data,
      qt_meta_data_LIGHTWalkingDialog_GainSetting,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *LIGHTWalkingDialog_GainSetting::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *LIGHTWalkingDialog_GainSetting::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_LIGHTWalkingDialog_GainSetting.stringdata0))
        return static_cast<void*>(this);
    return QDialog::qt_metacast(_clname);
}

int LIGHTWalkingDialog_GainSetting::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 8)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 8;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 8)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 8;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
