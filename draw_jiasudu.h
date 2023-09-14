#ifndef DRAW_JIASUDU_H
#define DRAW_JIASUDU_H

#include <QMainWindow>
#include <Eigen/Dense>
#include <iostream>
#include <QDebug>

namespace Ui {
class Draw_Jiasudu;
}

class Draw_Jiasudu : public QMainWindow
{
    Q_OBJECT

public:
    explicit Draw_Jiasudu(Eigen::MatrixXd *jieguo, int geshu, int caiyang, QWidget *parent = 0);
    ~Draw_Jiasudu();

private:
    Ui::Draw_Jiasudu *ui;
};

#endif // DRAW_JIASUDU_H
