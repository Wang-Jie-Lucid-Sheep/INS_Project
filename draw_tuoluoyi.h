#ifndef DRAW_TUOLUOYI_H
#define DRAW_TUOLUOYI_H

#include <QMainWindow>
#include <Eigen/Dense>
#include <iostream>
#include <QDebug>

namespace Ui {
class Draw_Tuoluoyi;
}

class Draw_Tuoluoyi : public QMainWindow
{
    Q_OBJECT

public:
    explicit Draw_Tuoluoyi(Eigen::MatrixXd *jieguo, int geshu, int caiyang, QWidget *parent = 0);
    ~Draw_Tuoluoyi();

private:
    Ui::Draw_Tuoluoyi *ui;
};

#endif // DRAW_TUOLUOYI_H
