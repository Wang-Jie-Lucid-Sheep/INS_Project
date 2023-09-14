#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFileDialog>
#include <QString>
#include <QMessageBox>
#include <QDebug>
#include <QSound>

#include "draw_jiasudu.h"
#include "draw_tuoluoyi.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_Calibration_Button_clicked();

    void on_Coarse_Button_clicked();

    void on_Caculate_Button_clicked();

    void guanbi();

    void baocun();

    void Daoru_ASSIC();

    void Daoru_BIN();

    void Xinxi();

    void on_Compare_Button_clicked();

    void Draw_jiasudu();

    void Draw_tuoluoyi();

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
