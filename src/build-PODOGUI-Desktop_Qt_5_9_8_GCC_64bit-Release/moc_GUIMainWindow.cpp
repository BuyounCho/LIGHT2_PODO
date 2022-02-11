/****************************************************************************
** Meta object code from reading C++ file 'GUIMainWindow.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.8)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../PODOGUI/GUIMainWindow.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'GUIMainWindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.8. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_GUIMainWindow_t {
    QByteArrayData data[17];
    char stringdata0[296];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_GUIMainWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_GUIMainWindow_t qt_meta_stringdata_GUIMainWindow = {
    {
QT_MOC_LITERAL(0, 0, 13), // "GUIMainWindow"
QT_MOC_LITERAL(1, 14, 17), // "ActionLAN_Toggled"
QT_MOC_LITERAL(2, 32, 0), // ""
QT_MOC_LITERAL(3, 33, 7), // "checked"
QT_MOC_LITERAL(4, 41, 20), // "ActionPODOAL_Toggled"
QT_MOC_LITERAL(5, 62, 19), // "ActionJOINT_Toggled"
QT_MOC_LITERAL(6, 82, 20), // "ActionSENSOR_Toggled"
QT_MOC_LITERAL(7, 103, 13), // "SLOT_LAN_HIDE"
QT_MOC_LITERAL(8, 117, 12), // "SLOT_AL_HIDE"
QT_MOC_LITERAL(9, 130, 14), // "SLOT_LAN_ONOFF"
QT_MOC_LITERAL(10, 145, 5), // "onoff"
QT_MOC_LITERAL(11, 151, 13), // "SLOT_NEW_DATA"
QT_MOC_LITERAL(12, 165, 25), // "on_BTN_REF_ENABLE_clicked"
QT_MOC_LITERAL(13, 191, 26), // "on_BTN_REF_DISABLE_clicked"
QT_MOC_LITERAL(14, 218, 24), // "on_BTN_CAN_CHECK_clicked"
QT_MOC_LITERAL(15, 243, 25), // "on_BTN_ENC_ENABLE_clicked"
QT_MOC_LITERAL(16, 269, 26) // "on_BTN_ENC_DISABLE_clicked"

    },
    "GUIMainWindow\0ActionLAN_Toggled\0\0"
    "checked\0ActionPODOAL_Toggled\0"
    "ActionJOINT_Toggled\0ActionSENSOR_Toggled\0"
    "SLOT_LAN_HIDE\0SLOT_AL_HIDE\0SLOT_LAN_ONOFF\0"
    "onoff\0SLOT_NEW_DATA\0on_BTN_REF_ENABLE_clicked\0"
    "on_BTN_REF_DISABLE_clicked\0"
    "on_BTN_CAN_CHECK_clicked\0"
    "on_BTN_ENC_ENABLE_clicked\0"
    "on_BTN_ENC_DISABLE_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_GUIMainWindow[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      13,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   79,    2, 0x08 /* Private */,
       4,    1,   82,    2, 0x08 /* Private */,
       5,    0,   85,    2, 0x08 /* Private */,
       6,    0,   86,    2, 0x08 /* Private */,
       7,    0,   87,    2, 0x08 /* Private */,
       8,    0,   88,    2, 0x08 /* Private */,
       9,    1,   89,    2, 0x08 /* Private */,
      11,    0,   92,    2, 0x08 /* Private */,
      12,    0,   93,    2, 0x08 /* Private */,
      13,    0,   94,    2, 0x08 /* Private */,
      14,    0,   95,    2, 0x08 /* Private */,
      15,    0,   96,    2, 0x08 /* Private */,
      16,    0,   97,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,   10,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void GUIMainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        GUIMainWindow *_t = static_cast<GUIMainWindow *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->ActionLAN_Toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 1: _t->ActionPODOAL_Toggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 2: _t->ActionJOINT_Toggled(); break;
        case 3: _t->ActionSENSOR_Toggled(); break;
        case 4: _t->SLOT_LAN_HIDE(); break;
        case 5: _t->SLOT_AL_HIDE(); break;
        case 6: _t->SLOT_LAN_ONOFF((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 7: _t->SLOT_NEW_DATA(); break;
        case 8: _t->on_BTN_REF_ENABLE_clicked(); break;
        case 9: _t->on_BTN_REF_DISABLE_clicked(); break;
        case 10: _t->on_BTN_CAN_CHECK_clicked(); break;
        case 11: _t->on_BTN_ENC_ENABLE_clicked(); break;
        case 12: _t->on_BTN_ENC_DISABLE_clicked(); break;
        default: ;
        }
    }
}

const QMetaObject GUIMainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_GUIMainWindow.data,
      qt_meta_data_GUIMainWindow,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *GUIMainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *GUIMainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_GUIMainWindow.stringdata0))
        return static_cast<void*>(this);
    return QMainWindow::qt_metacast(_clname);
}

int GUIMainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 13)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 13;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 13)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 13;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
