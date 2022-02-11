#ifndef SW_TASK_DIALOG_H
#define SW_TASK_DIALOG_H

#include <QWidget>

namespace Ui {
class SW_TASK_Dialog;
}

class SW_TASK_Dialog : public QWidget
{
    Q_OBJECT

public:
    explicit SW_TASK_Dialog(QWidget *parent = 0);
    ~SW_TASK_Dialog();

private slots:
//    void UpdateSettings();

    void on_BTN_FT_NULL_clicked();

    void on_BTN_MOVE_clicked();

    void on_BTN_PWM_ON_clicked();

    void on_BTN_SHOW_clicked();

private:
    Ui::SW_TASK_Dialog *ui;

    int ALNum;
};

#endif // SW_TASK_DIALOG_H
