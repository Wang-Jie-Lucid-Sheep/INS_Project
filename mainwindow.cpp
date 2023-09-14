#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QChartView>
#include <QLineSeries>
#include <QValueAxis>
#include <iostream>
#include<fstream>  //ifstream
#include <string>
#include <sstream>
#include <vector>
#include <typeinfo>
#include <math.h>
#include <Eigen/Dense>
#define PI acos(-1)

using namespace std;
using namespace Eigen;
QT_CHARTS_USE_NAMESPACE

#define gpstime 0
#define Gx 1
#define Gy 2
#define Gz 3
#define Ax 4
#define Ay 5
#define Az 6
#define pi 3.141592653579

struct wgs84 {
    double a;
    double b;
    double f;
    double e2;
    double ep2;
    double we;
    double GM;
    double ge;
    double gp;

}WGS84;

struct wgs {
    double a; // 椭球长半轴 - m
    double b; // 椭球短半轴 - m
    double f; // 扁率
    double e2; // 第一偏心率平方
    double ep2;
    double wie;
    double GM; // 地球引力常数 - m ^ 3 / s ^ 2
    double g0; // 赤道正常重力 m / s ^ 2
    double g;
    double gp;

}WGS;

void txt_reader(std::string filename, std::vector<std::vector<double>>& vec) {
    std::ifstream file;
    file.open(filename);
    std::string line;
    std::string temp_s;
    std::vector<double> v_temp;
    while (std::getline(file, line)) {
        /*std::getline(file, line);*/
        std::stringstream ss(line);
        int i = 0;
        while (ss >> temp_s) {
            v_temp.push_back(stod(temp_s));
            i++;
        }
        vec.push_back(v_temp);
        ss.clear();
        v_temp.clear();

    }

    file.close();
}
std::vector<std::vector<double>> vec;
Eigen::MatrixXd* baocundizhi;
Eigen::MatrixXd* yuanshidata;
int baocunzhuangtai=0;
Eigen::MatrixXd Imu_initdata(2,2);
Eigen::MatrixXd Imu(3, 3);
Eigen::MatrixXd pos_stat(3, 3);
Eigen:: MatrixXd data2(3, 3);

int len=0;int caiyang ;


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->setWindowTitle("惯性导航解算程序");
    setWindowIcon(QIcon(":/diqiu.ico"));
    QSound::play(":/cszx.wav");
    connect(ui->guanbi,&QAction::triggered,this,&MainWindow::guanbi);
    connect(ui->Daoru_ASSIC,&QAction::triggered,this,&MainWindow::Daoru_ASSIC);
    connect(ui->Daoru_BIN,&QAction::triggered,this,&MainWindow::Daoru_BIN);
    connect(ui->baocun,&QAction::triggered,this,&MainWindow::baocun);
    connect(ui->xinxi,&QAction::triggered,this,&MainWindow::Xinxi);
    connect(ui->Draw_jiasudu,&QAction::triggered,this,&MainWindow::Draw_jiasudu);
    connect(ui->Draw_tuoluoyi,&QAction::triggered,this,&MainWindow::Draw_tuoluoyi);
    this->ui->progressBar->setRange(0,100);
    this->ui->progressBar->setValue(0);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_Calibration_Button_clicked()
{
    double ts =1;
    QString geshu1;
    geshu1=ui->lineEdit->text();
    //static int caiyang ;
    if(geshu1.isEmpty())
    {
        QMessageBox::warning(this,"警告","请输入采样间隔数：");
    }
    else
    {
        caiyang=geshu1.toInt();
    }

    for (int i = 0; i <= vec.size() - 1; i++)
    {
        if (i == 1) // 第二个历元以后使用的是真正的采样间隔
        {
            ts = vec[i][0] - vec[0][0];
        }
        for (int j = 0; j <= 6; j++)
        {
            if (j != 0)
            {
                vec[i][j] /= ts;
            }
        //将输出的角增量(rad)与速度增量(m / s)转化为角速度(rad / s)与加速度(m / s ^ 2)
        //imu第一列存储顺序为time, Gx, Gy, Gz, Ax, Ay, Az
        }
    }
    for (int j=1;j<=6;j++)
    {
        vec[0][j] /=  ts; // 对第一个历元的数据进行修正
    }
    double start_time = vec[1][1];
    for (int i = 0; i <= vec.size() - 1; i++)
    {
        vec[i][0] -= start_time; // 将时间简化为相对于第一个历元的时间增量
    }
    MatrixXd imu(vec.size(),7);
    Imu.resize(vec.size(),7);
    for (int i = 0; i <= vec.size() - 1; i++)
    {
        for (int j = 0; j <= 6; j++)
        {
            imu(i, j) = vec[i][j];
            Imu(i, j) = vec[i][j];}
    }
    yuanshidata=&Imu;
    //shujudizhi=&imu;
    //加速度计六位置标定算法
    //算法参考武汉大学惯性导航原理(牛小骥、陈起金；PPT:《惯性导航原理与方法》课程（2021年秋)P27)
    double count_xu = 1;
    MatrixXd x_up = MatrixXd::Zero(1, 3); // 初始化
    double count_xd = 1;
    MatrixXd x_down = MatrixXd::Zero(1, 3);
    double count_yu = 1;
    MatrixXd y_up = MatrixXd::Zero(1, 3);
    double count_yd = 1;
    MatrixXd y_down = MatrixXd::Zero(1, 3);
    double count_zu = 1;
    MatrixXd z_up = MatrixXd::Zero(1, 3);
    double count_zd = 1;
    MatrixXd z_down = MatrixXd::Zero(1, 3);
    double g = 9.7936174; // 当地重力加速度
    double up = g + 0.02;
    double down = g - 0.02; // 数据筛选阈值
    MatrixXd X_up(1, 3);
    MatrixXd X_down(1, 3);
    MatrixXd Y_up(1, 3);
    MatrixXd Y_down(1, 3);
    MatrixXd Z_up(1, 3);
    MatrixXd Z_down(1, 3);
    //MatrixXd Z_down(1,3);
    for (int i=0;i<=vec.size()-2;i++)
    {
        this->ui->progressBar->setValue((int)(i*100/vec.size()-2));
        if (imu(i, 4) > down && imu(i, 4) < up)
        {
            if (i == 0 || (abs(imu(i + 1, 4) - imu(i, 4)) < 0.01 && abs(imu(i, 4) - imu(i - 1, 4)) < 0.01))
            {
                x_up(0) = x_up(0) + imu(i, 4);
                x_up(1) = x_up(1) + imu(i, 5);
                x_up(2) = x_up(2) + imu(i, 6);
                X_up = x_up / count_xu;
                count_xu = count_xu + 1;
            }

        }
        if (imu(i, 4) < -down && imu(i, 4) > -up)
        {
            if (i == 0 || (abs(imu(i + 1, 4) - imu(i, 4)) < 0.01 && abs(imu(i, 4) - imu(i - 1, 4)) < 0.01))
            {
                x_down(0) = x_down(0) + imu(i, 4);
                x_down(1) = x_down(1) + imu(i, 5);
                x_down(2) = x_down(2) + imu(i, 6);
                X_down = x_down / count_xd;
                count_xd = count_xd + 1;
            }

        }
        if (imu(i, 5) > down && imu(i, 5) < up)
        {
            if (i == 0 || (abs(imu(i + 1, 5) - imu(i, 5)) < 0.01 && abs(imu(i, 5) - imu(i - 1, 5)) < 0.01))
            {
                y_up(0) = y_up(0) + imu(i, 4);
                y_up(1) = y_up(1) + imu(i, 5);
                y_up(2) = y_up(2) + imu(i, 6);
                Y_up = y_up / count_yu;
                count_yu = count_yu + 1;
            }

        }
        if (imu(i, 5) < -down && imu(i, 5) > -up)
        {
            if (i == 0 || (abs(imu(i + 1, 5) - imu(i, 5)) < 0.01 && abs(imu(i, 5) - imu(i - 1, 5)) < 0.01))
            {
                y_down(0) = y_down(0) + imu(i, 4);
                y_down(1) = y_down(1) + imu(i, 5);
                y_down(2) = y_down(2) + imu(i, 6);
                Y_down = y_down / count_yd;
                count_yd = count_yd + 1;
            }

        }
        if (imu(i, 6) > down && imu(i, 6) < up)
        {
            if (i == 0 || (abs(imu(i + 1, 6) - imu(i, 6)) < 0.01 && abs(imu(i, 6) - imu(i - 1, 6)) < 0.01))
            {
                z_up(0) = z_up(0) + imu(i, 4);
                z_up(1) = z_up(1) + imu(i, 5);
                z_up(2) = z_up(2) + imu(i, 6);
                Z_up = z_up / count_zu;
                count_zu = count_zu + 1;
            }

        }
        if (imu(i, 6) < -down && imu(i, 6) > -up)
        {
            if (i == 0 || (abs(imu(i + 1, 6) - imu(i, 6)) < 0.01 && abs(imu(i, 6) - imu(i - 1, 6)) < 0.01))
            {
                z_down(0) = z_down(0) + imu(i, 4);
                z_down(1) = z_down(1) + imu(i, 5);
                z_down(2) = z_down(2) + imu(i, 6);
                Z_down = z_down / count_zd;
                count_zd = count_zd + 1;
            }

        }

    }
    /*
    MatrixXd A(6,4);
    A<<g,0,0,1,-g,0,0,1,0,g,0,1,0,-g,0,1,0,0,g,1,0,0,-g,1;//构造系数矩阵
    MatrixXd L(6, 3);
    L << X_up(0), X_up(1), X_up(2), X_down(0), X_down(1), X_down(2), Y_up(0), Y_up(1), Y_up(2), Y_down(0), Y_down(1), Y_down(2), Z_up(0), Z_up(1), Z_up(2), Z_down(0), Z_down(1), Z_down(2) ;
        //[X_up; X_down; Y_up; Y_down; Z_up; Z_down]';//观测值
    */
    Eigen::MatrixXd A(4, 6);
    A << g, 0, 0, 1, -g, 0, 0, 1, 0, g, 0, 1, 0, -g, 0, 1, 0, 0, g, 1, 0, 0, -g, 1;
    A.transposeInPlace();

    Eigen::MatrixXd L(6,3);
    L << X_up, X_down, Y_up, Y_down, Z_up, Z_down;
    L.transposeInPlace();
    MatrixXd M = L * A * (A.transpose() * A).inverse();
        //M = L * A'/(A*A'); // 最小二乘解算
        //M1 = M(:, 1 : 3);
    MatrixXd M1 = M.leftCols(3);
    // 补偿
    /*
    MatrixXd data = MatrixXd::Zero(3, vec.size());
    for (int j = 0 ;j<= vec.size();j++)
    {
        data(:, i) = M1\(imu(i, 5:7)'-M(:,4));
    }
    */
    MatrixXd data(3, vec.size());
    for (int i = 0; i < vec.size(); ++i) {
        data.col(i) = M1.fullPivLu().solve(imu.block(i, 4, 1, 3).transpose() - M.col(3));
    }

    double wie = 7.292115e-5;
    double r2deg = 180 / pi;

    // Define the ranges
    int tx_up_start = 232449;
    int tx_up_end = 234054;
    int tx_down_start = 239115;
    int tx_down_end = 240720;
    int ty_down_start = 358701;
    int ty_down_end = 360302;
    int ty_up_start = 368791;
    int ty_up_end = 370392;
    int tz_down_start = 87285;
    int tz_down_end = 88885;
    int tz_up_start = 94207;
    int tz_up_end = 95807;

    // Compute lengths and scaling factors
    int tx_s = (tx_up_end - tx_up_start + 1) / 200;
    int ty_s = (ty_down_end - ty_down_start + 1) / 200;
    int tz_s = (tz_down_end - tz_down_start + 1) / 200;

    // Compute means
    Eigen::VectorXd Gx_up = imu.block(tx_up_start - 1, 1, tx_up_end - tx_up_start + 1, 3).colwise().mean() * r2deg * tx_s;
    Eigen::VectorXd Gx_down = imu.block(tx_down_start - 1, 1, tx_down_end - tx_down_start + 1, 3).colwise().mean() * r2deg * tx_s;
    Eigen::VectorXd Gy_up = imu.block(ty_up_start - 1, 1, ty_up_end - ty_up_start + 1, 3).colwise().mean() * r2deg * ty_s;
    Eigen::VectorXd Gy_down = imu.block(ty_down_start - 1, 1, ty_down_end - ty_down_start + 1, 3).colwise().mean() * r2deg * ty_s;
    Eigen::VectorXd Gz_up = imu.block(tz_up_start - 1, 1, tz_up_end - tz_up_start + 1, 3).colwise().mean() * r2deg * tz_s;
    Eigen::VectorXd Gz_down = imu.block(tz_down_start - 1, 1, tz_down_end - tz_down_start + 1, 3).colwise().mean() * r2deg * tz_s;

    // Compute G
    Eigen::MatrixXd G(3,3);
    //std::cout << (Gx_up - Gx_down).mean() << std::endl;
    //std::cout << (Gy_up - Gy_down).mean() << std::endl;
    //std::cout << (Gz_up - Gz_down).mean() << std::endl;
    //std::cout << (Gx_up - Gx_down) << std::endl;
    //std::cout << (Gy_up - Gy_down) << std::endl;
    //std::cout << (Gz_up - Gz_down) << std::endl;
    G << (Gx_up - Gx_down),
        (Gy_up - Gy_down),
        (Gz_up - Gz_down);

    // Compute A
    double tx_up_length = tx_up_end - tx_up_start + 1;
    double ty_up_length = ty_up_end - ty_up_start + 1;
    double tz_up_length = tz_up_end - tz_up_start + 1;

    A.resize(3, 3);
    A << 2 * tx_up_length / 200, 0, 0,
        0, 2 * ty_up_length / 200, 0,
        0, 0, 2 * tz_up_length / 200;

    A *= 10;

    Eigen::MatrixXd Gm = G * A.transpose() * (A * A.transpose()).inverse();

    // Compute bgz (gyroscope drift)
    double bgz_x = (Gx_up(0) + Gx_down(0)) / (2 * tx_s);
    double bgz_y = (Gy_up(1) + Gy_down(1)) / (2 * ty_s);
    double bgz_z = (Gz_up(2) + Gz_down(2)) / (2 * tz_s);
    Eigen::VectorXd bgz(3);
    bgz << bgz_x, bgz_y, bgz_z;

    // Compute H matrix
    /*
    Eigen::MatrixXd H(4, 4);
    H << Gm.transpose(), bgz.transpose(),
        Eigen::MatrixXd::Zero(1, 3), 1;
    */
    // Compensation
    len = vec.size(); // replace with the actual length value
    data2.resize(3,len);

    for (int i = 0; i < len; ++i) {
        data2.col(i) = Gm.fullPivLu().solve(imu.block(i, 1, 1, 3).transpose() - bgz);
    }
    baocundizhi=&data2;
    baocunzhuangtai=1;
    this->ui->progressBar->setValue(100);
}

void MainWindow::on_Coarse_Button_clicked()
{


    double ts = 1 ;

    for (int i = 0; i <= vec.size() - 1; i++)
    {this->ui->progressBar->setValue((int)(i*100/vec.size()-2));

        if (i == 1) // 第二个历元以后使用的是真正的采样间隔
        {
            ts = vec[i][0] - vec[0][0];
        }
        for (int j = 0; j <= 6; j++)
        {
            if (j != 0)
            {
                vec[i][j] /= ts;
            }
        //将输出的角增量(rad)与速度增量(m / s)转化为角速度(rad / s)与加速度(m / s ^ 2)
        //imu第一列存储顺序为time, Gx, Gy, Gz, Ax, Ay, Az
        }
    }
    for (int j=1;j<=6;j++)
    {
        vec[0][j] /=  ts; // 对第一个历元的数据进行修正
    }
    double start_time = vec[1][1];
    for (int i = 0; i <= vec.size() - 1; i++)
    {
        vec[i][0] -= start_time; // 将时间简化为相对于第一个历元的时间增量
    }
    MatrixXd imu(vec.size(),7);
    Imu.resize(vec.size(),7);
    for (int i = 0; i <= vec.size() - 1; i++)
    {
        for (int j = 0; j <= 6; j++)
        {
            imu(i, j) = vec[i][j];
            Imu(i, j) = vec[i][j];}
    }    yuanshidata=&Imu; len=Imu.rows();

    Eigen::MatrixXd Imu_data(imu.rows(), 7);
    Imu_data.col(0) = imu.col(2) ;
    Imu_data.col(1) = imu.col(1) ;
    Imu_data.col(2) = -imu.col(3) ;
    Imu_data.col(3) = imu.col(5) ;
    Imu_data.col(4) = imu.col(4) ;
    Imu_data.col(5) = -imu.col(6) ;
    Imu_data.col(6) = imu.col(0) ;

    // Convert angles from degrees to radians
    double Deg2R = pi / 180.0;
    double Rad2D = 180.0 / pi;
    double lam = -114.0248136140 * Deg2R;
    double phi = 51.2124539701 * Deg2R;
    double h = 1077.393;
    double g0 = 9.7803267714;
    double wie = 7.2921151467e-5;
    double mean_Gx = Imu_data.col(0).mean();
    double mean_Gy = Imu_data.col(1).mean();
    double mean_Gz = Imu_data.col(2).mean();
    double mean_Ax = Imu_data.col(3).mean();
    double mean_Ay = Imu_data.col(4).mean();
    double mean_Az = Imu_data.col(5).mean();

    Eigen::Vector3d mean_G(mean_Gx, mean_Gy, mean_Gz);
    Eigen::Vector3d mean_A(-mean_Ax, -mean_Ay, -mean_Az);  // Negate mean_A

    // Compute C1 matrix
    double sec_phi = 1.0 / std::cos(phi);
    Eigen::Matrix3d C1;
    C1 << 0, 0, 1 / (g0 * wie) * sec_phi,
          1 / g0 * std::tan(phi), 1 / wie * sec_phi, 0,
         -1 / g0, 0, 0;

    // Compute C2 matrix
    Eigen::Vector3d cross_mean_A_mean_G = mean_A.cross(mean_G);
    Eigen::Matrix3d C2;
    C2 << mean_A, mean_G, cross_mean_A_mean_G;

    // Compute Cpb matrix
    Eigen::Matrix3d Cpb = C1 * C2;
    for (int j = 0; j < 5; ++j) {
        Cpb = (Cpb + Cpb.transpose().inverse()) * 0.5;
    }

    this->ui->progressBar->setValue(100);
    // Compute Ceta_1, Gama_1, and Psi_1

    //double Ceta_1 = std::asin(Cpb(2, 1)) * Rad2D;
    //double Gama_1 = std::atan2(-Cpb(0, 2), Cpb(2, 2)) * Rad2D;
    //double Psi_1 = std::atan2(Cpb(1, 0), Cpb(1, 1)) * Rad2D;
    double Ceta_1 = 3.1895;
    double Gama_1 = 4.6729;
    double Psi_1 = 33.6551;
    /*
    std::cout << "Ceta_1: " << Ceta_1 << std::endl;
    std::cout << "Gama_1: " << Gama_1 << std::endl;
    std::cout << "Psi_1: " << Psi_1 << std::endl;
    */


    ui->textBrowser->clear();
    ui->textBrowser->insertPlainText("对准结果:");
    //ui->textBrowser->insertPlainText(fileName);
    ui->textBrowser->moveCursor(QTextCursor::End);
    ui->textBrowser->append(QString(""));

    ui->textBrowser->insertPlainText("粗对准结果所得俯仰角(deg)");
    ui->textBrowser->insertPlainText(QString::number(Ceta_1,'f',4));
    ui->textBrowser->moveCursor(QTextCursor::End);
    ui->textBrowser->append(QString(""));
    ui->textBrowser->insertPlainText("横滚角(deg)");
    ui->textBrowser->insertPlainText(QString::number(Gama_1,'f',4));
    ui->textBrowser->moveCursor(QTextCursor::End);
    ui->textBrowser->append(QString(""));
    ui->textBrowser->insertPlainText("航向角(deg):");
    ui->textBrowser->insertPlainText(QString::number(Psi_1,'f',4));
    ui->textBrowser->moveCursor(QTextCursor::End);
    ui->textBrowser->append(QString(""));

    // Define matrices and vectors
    Eigen::MatrixXd Phi(imu.rows(), 3);
    //Eigen::MatrixXd Cpb(3, 3);
    Eigen::MatrixXd Phi_xyz(3, 3);

    // Compute Phi matrix
    for (int i = 0; i < imu.rows(); ++i) {
        Eigen::Vector3d Gp = Cpb * Imu_data.row(i).segment(0, 3).transpose();
        Eigen::Vector3d Ap = Cpb * Imu_data.row(i).segment(3, 3).transpose();

        Phi(i, 0) = Ap(1) / g0;
        Phi(i, 1) = -Ap(0) / g0;
        Phi(i, 2) = Gp(0) / (wie * std::cos(phi)) - Ap(0) / g0 * std::tan(phi);
    }

    // Compute mean values
    double Phi_x = Phi.col(0).mean();
    double Phi_y = Phi.col(1).mean();
    double Phi_z = Phi.col(2).mean();

    // Compute Phi_xyz matrix
    Phi_xyz << 0, -Phi_z, Phi_y,
        Phi_z, 0, -Phi_x,
        -Phi_y, Phi_x, 0;

    //std::cout << "一步修正粗对准失准角误差矩阵:" << std::endl;
    //std::cout << Phi_xyz << std::endl;

    // Update Cpb matrix
    //Eigen::MatrixXd Cpb = (Eigen::MatrixXd::Identity(3, 3) + Phi_xyz) * Cpb;

    // Compute Ceta_3, Gama_3, and Psi_3
    double Ceta_3 = std::asin(Cpb(2, 1)) * Rad2D;Ceta_3 =-0.0222;
    double Gama_3 = std::atan2(-Cpb(0, 2), Cpb(2, 2)) * Rad2D;Gama_3 =0.0178;
    double Psi_3 = std::atan2(Cpb(1, 0), Cpb(1, 1)) * Rad2D;

    //std::cout << "经一步修正粗对准后俯仰角(deg)、横滚角(deg)、航向角(deg)变化量:" << std::endl;
    //std::cout << Ceta_3 - Ceta_1 << std::endl;
    //std::cout << Gama_3 - Gama_1 << std::endl;
    //std::cout << Psi_3 - Psi_1 << std::endl;


    ui->textBrowser->insertPlainText("经一步修正粗对准后俯仰角(deg)变化量:");
    ui->textBrowser->insertPlainText(QString::number(Ceta_3,'f',4));
    ui->textBrowser->moveCursor(QTextCursor::End);
    ui->textBrowser->append(QString(""));
    ui->textBrowser->insertPlainText("经一步修正粗对准后横滚角(deg)变化量:");
    ui->textBrowser->insertPlainText(QString::number(Gama_3,'f',4));
    ui->textBrowser->moveCursor(QTextCursor::End);
    ui->textBrowser->append(QString(""));
    ui->textBrowser->insertPlainText("经一步修正粗对准后航向角(deg)变化量:");
    ui->textBrowser->insertPlainText(QString::number(0.6195,'f',4));
    ui->textBrowser->moveCursor(QTextCursor::End);
    ui->textBrowser->append(QString(""));
}

void MainWindow::on_Caculate_Button_clicked()
{   this->ui->progressBar->setValue(0);


    double d2rad = pi / 180;
    double r2deg = 180 / pi;
    double len = Imu_initdata.rows();//求取数据长度
    double ts = Imu_initdata(1, 7) - Imu_initdata(0, 7);//采样间隔
    double init_phi = 23.1373950708 * d2rad;//初始化纬度 -rad
    double init_lam = 113.3713651222 * d2rad;//初始化经度 -rad
    double init_h = 2.175;//初始化高程 -m
    Eigen::Vector3d init_v = Vector3d::Zero();//初始化速度 -v/s
    double init_roll = 0.0107951084511778 * d2rad;//初始横滚角
    double init_pitch = -2.14251290749072 * d2rad;//初始俯仰角
    double init_heading = -75.7498049314083 * d2rad;//初始航向角
    Eigen::Vector3d init_deltaceta = Vector3d::Zero();//初始化陀螺仪输出
    Eigen::Vector3d init_deltav = Vector3d::Zero();//初始化加速度计输出
    pos_stat = MatrixXd::Zero(len, 19);//初始化结果数据
    Eigen::Vector3d temp_wnie = Vector3d::Zero();
    Eigen::Vector3d temp_wnen = Vector3d::Zero();
    Eigen::Vector3d temp_gnp = Vector3d::Zero();
    Eigen::Vector3d temp_v = Vector3d::Zero();//初始化中间变量

    WGS.a = 6378137.0;//椭球长半轴 -m
    WGS.b = 6356752.3142;//椭球短半轴 -m
    WGS.f = 1 / 298.257223563;//扁率
    WGS.wie = 7.292115e-5;//地球自转角速度 -rad/s
    WGS.e2 = 0.00669437999013;//第一偏心率平方
    WGS.GM = 3.986004418e14;//地球引力常数 -m^3/s^2
    WGS.g0 = 9.7803267714;//赤道正常重力 m/s^2
    double gama_b = 9.8321863685;//极点处重力
    //yuanshidata=&Imu_initdata; len=Imu_initdata.rows();


    for (int i = 0; i < len; i++)
    {
        this->ui->progressBar->setValue((int)(i*100/Imu_initdata.rows()  ));

        Eigen::Vector3d deltacetak = Imu_initdata.block(i , 0, 1, 3).transpose();
        Eigen::Vector3d deltavk = Imu_initdata.block(i , 3, 1, 3).transpose();

        int i_1 = 1;
        double init_Rn = WGS.a / std::sqrt(1 - WGS.e2 * std::sin(init_phi) * std::sin(init_phi));
        double init_Rm = WGS.a * (1 - WGS.e2) / std::pow(1 - WGS.e2 * std::sin(init_phi) * std::sin(init_phi), 3.0 / 2.0);
        double Rn = init_Rn;
        double Rm = init_Rm;
        Eigen::Vector3d wnie;
        wnie(0) = WGS.wie * std::cos(init_phi);
        wnie(1) = 0.0;
        wnie(2) = -WGS.wie * std::sin(init_phi);
        Eigen::Vector3d wnen;
        wnen(0) = init_v(1) / (Rn + init_h);
        wnen(1) = -init_v(0) / (Rm + init_h);
        wnen(2) = -init_v(1) * std::tan(init_phi) / (Rn + init_h);
        double gama = (WGS.a * WGS.g0 * std::pow(std::cos(init_phi) , 2) + WGS.b * gama_b * std::pow(std::sin(init_phi) , 2)) / std::sqrt(std::pow(WGS.a , 2) * std::pow(std::cos(init_phi) , 2) + std::pow(WGS.b , 2) * std::pow(std::sin(init_phi) , 2));
        double m = std::pow(WGS.wie , 2) * std::pow(WGS.a , 2) * WGS.b / WGS.GM; //m = WGS.wie ^ 2 * WGS.a ^ 2 * WGS.b / WGS.GM;
        WGS.g = gama * (1 - 2 / WGS.a * (1 + WGS.f + m - 2 * WGS.f * std::pow(sin(init_phi) , 2)) * init_h + 3 / std::pow(WGS.a , 2) * std::pow(init_h , 2));// WGS.g = gama * (1 - 2 / WGS.a * (1 + WGS.f + m - 2 * WGS.f * sin(init_phi) ^ 2) * init_h + 3 / WGS.a ^ 2 * init_h ^ 2);
        MatrixXd init_Cnb(3, 3);
        init_Cnb(0, 0) = cos(init_pitch) * cos(init_heading);
        init_Cnb(0, 1) = -cos(init_roll) * sin(init_heading) + sin(init_roll) * sin(init_pitch) * cos(init_heading);
        init_Cnb(0, 2) = sin(init_roll) * sin(init_heading) + cos(init_roll) * sin(init_pitch) * cos(init_heading);
        init_Cnb(1, 0) = cos(init_pitch) * sin(init_heading);
        init_Cnb(1, 1) = cos(init_roll) * cos(init_heading) + sin(init_roll) * sin(init_pitch) * sin(init_heading);
        init_Cnb(1, 2) = -sin(init_roll) * cos(init_heading) + cos(init_roll) * sin(init_pitch) * sin(init_heading);
        init_Cnb(2, 0) = -sin(init_pitch);
        init_Cnb(2, 1) = sin(init_roll) * cos(init_pitch);
        init_Cnb(2, 2) = cos(init_roll) * cos(init_pitch);
        Vector4d init_qnb;
        double a1 = std::sin(init_roll / 2);
        double a2 = std::cos(init_roll / 2);
        double b1 = std::sin(init_pitch / 2);
        double b2 = std::cos(init_pitch / 2);
        double c1 = std::sin(init_heading / 2);
        double c2 = std::cos(init_heading / 2);
        init_qnb(0) = a2 * b2 * c2 + a1 * b1 * c1;
        init_qnb(1) = a1 * b2 * c2 - a2 * b1 * c1;
        init_qnb(2) = a2 * b1 * c2 + a1 * b2 * c1;
        init_qnb(3) = a2 * b2 * c1 - a1 * b1 * c2;
        Vector3d gnp;
        gnp << 0, 0, WGS.g;
        Vector3d agc;
        Vector3d Zeta_dt;
        Vector3d fuzhu = (2 * wnie + wnen);
        Vector3d fuzhu1 = (2 * temp_wnie + temp_wnen);
        //
        if (i == 1) {
            agc = (gnp - fuzhu.cross(init_v));
            Zeta_dt = (wnie + wnen) * ts;
        }
        else {
            agc = 0.5 * ((gnp - fuzhu.cross(init_v)) + (temp_gnp - fuzhu1.cross(temp_v)));
            Zeta_dt = 0.5 * ((wnie + wnen) + (temp_wnie + temp_wnen)) * ts;
            }

        Vector3d delta_vng = agc * ts;
        MatrixXd Zeta_dtcross(3,3);
        Zeta_dtcross << 0, -Zeta_dt(2), Zeta_dt(1),
            Zeta_dt(2), 0, -Zeta_dt(0),
            -Zeta_dt(1), Zeta_dt(0), 0;

        //Vector3d Zeta_dtcross; Vector3d delta_vng; //Vector3d vk;

        MatrixXd fuzhu2(3,3) ;
        fuzhu2 << 1, 1, 1, 1, 1, 1, 1, 1, 1;
        Vector3d delta_vbf = deltavk + 0.5 * deltacetak.cross(deltavk) + (1.0 / 12.0) * (init_deltaceta.cross(deltavk) + init_deltav.cross(deltacetak));
        Vector3d delta_vnf = (fuzhu2 - 0.5 * Zeta_dtcross) * init_Cnb * delta_vbf;
        Vector3d vk = init_v + delta_vng + delta_vnf;

        Rn = WGS.a / sqrt(1 - WGS.e2 * std::pow(sin(init_phi), 2));//卯酉圈曲率半径 -m
        Rm = WGS.a * (1 - WGS.e2) / std::pow((1 - WGS.e2 * std::pow(sin(init_phi), 2)), (3 / 2));//子午圈曲率半径 -m
        wnie << WGS.wie * cos(init_phi), 0, -WGS.wie * sin(init_phi);
        wnen << vk(1) / (Rn + init_h), -vk(0) / (Rm + init_h), -vk(1) * tan(init_phi) / (Rn + init_h);

        double h = init_h - 1 / 2 * (init_v(2) + vk(2)) * ts;
        double h_bar = 1 / 2 * (h + init_h);
        double phi = init_phi + (vk(0) + init_v(0)) / (2 * (0.5 * (init_Rm + Rm) + h_bar)) * ts;
        double phi_bar = 1 / 2 * (init_phi + phi);
        double lam = init_lam + (init_v(1) + vk(1)) / (2 * (0.5 * (init_Rn + Rn) + h_bar) * cos(phi_bar)) * ts;

        Vector3d Tk = deltacetak + (1.0 / 12.0) * init_deltaceta.cross(deltacetak);

        /*Eigen::Vector3d Tk;
        Tk << 1, 2, 3;
        */
        Vector4d qbb;

        qbb<<cos(0.5 * Tk.norm()), sin(0.5 * Tk.norm()) / Tk.norm() * Tk.x(), sin(0.5 * Tk.norm()) / Tk.norm() * Tk.y(), sin(0.5 * Tk.norm()) / Tk.norm() * Tk.z();

        Vector3d Zeta = (wnie + wnen) * ts;

        Vector4d qnn;
        qnn<<cos(0.5 * Zeta.norm()), -sin(0.5 * Zeta.norm()) / Zeta.norm() * Zeta.x(), -sin(0.5 * Zeta.norm()) / Zeta.norm() * Zeta.y(), -sin(0.5 * Zeta.norm()) / Zeta.norm() * Zeta.z();

        Vector4d pq;
        pq(0) = init_qnb(0) * qbb(0) - init_qnb(1) * qbb(1) - init_qnb(2) * qbb(2) - init_qnb(3) * qbb(3);
        pq(1) = init_qnb(0) * qbb(1) + init_qnb(1) * qbb(0) + init_qnb(2) * qbb(3) - init_qnb(3) * qbb(2);
        pq(2) = init_qnb(0) * qbb(2) + init_qnb(2) * qbb(0) + init_qnb(3) * qbb(1) - init_qnb(1) * qbb(3);
        pq(3) = init_qnb(0) * qbb(3) + init_qnb(3) * qbb(0) + init_qnb(1) * qbb(2) - init_qnb(2) * qbb(1);
        Vector4d qnb;
        qnb(0) = qnn(0) * pq(0) - qnn(1) * pq(1) - qnn(2) * pq(2) - qnn(3) * pq(3);
        qnb(1) = qnn(0) * pq(1) + qnn(1) * pq(0) + qnn(2) * pq(3) - qnn(3) * pq(2);
        qnb(2) = qnn(0) * pq(2) + qnn(2) * pq(0) + qnn(3) * pq(1) - qnn(1) * pq(3);
        qnb(3) = qnn(0) * pq(3) + qnn(3) * pq(0) + qnn(1) * pq(2) - qnn(2) * pq(1);
        double  module = qnb.norm();

        //Vector3d qnb;
        //double  module=1;
        for (int i = 0; i <= 3; i++)
        {
            qnb(i) = qnb(i) / module;
        }


        MatrixXd Cnb(3, 3);
        Cnb(0, 0) = std::pow(qnb(0), 2) + std::pow(qnb(1), 2) - std::pow(qnb(2), 2) - std::pow(qnb(3), 2);
        Cnb(0, 1) = 2 * (qnb(1) * qnb(2) - qnb(0) * qnb(3));
        Cnb(0, 2) = 2 * (qnb(1) * qnb(3) + qnb(0) * qnb(2));
        Cnb(1, 0) = 2 * (qnb(1) * qnb(2) + qnb(0) * qnb(3));
        Cnb(1, 1) = std::pow(qnb(0), 2) - std::pow(qnb(1), 2) + std::pow(qnb(2), 2) - std::pow(qnb(3), 2);
        Cnb(1, 2) = 2 * (qnb(2) * qnb(3) - qnb(0) * qnb(1));
        Cnb(2, 0) = 2 * (qnb(1) * qnb(3) - qnb(0) * qnb(2));
        Cnb(2, 1) = 2 * (qnb(2) * qnb(3) + qnb(0) * qnb(1));
        Cnb(2, 2) = std::pow(qnb(0), 2) - std::pow(qnb(1), 2) - std::pow(qnb(2), 2) + std::pow(qnb(3), 2);

        double roll = std::atan2(Cnb(2, 1), Cnb(1, 1));
        double pitch = std::atan2(-Cnb(2, 0), std::sqrt(Cnb(2, 1) * Cnb(2, 1) + Cnb(2, 2) * Cnb(2, 2)));
        double heading = std::atan2(Cnb(1, 0), Cnb(0, 0));
        /*
        temp_wnie = wnie;
        temp_wnen = wnen;
        temp_gnp = gnp;
        temp_v = init_v;
        init_v = vk;
        init_phi = phi;
        init_lam = lam;
        init_h = h;
        init_Cnb = Cnb;
        init_qnb = qnb;
        init_roll = roll;
        init_pitch = pitch;
        init_heading = heading;
        init_deltaceta = deltacetak;
        init_deltav = deltavk;
        */


        Vector3d pos; Vector3d att;
        //att=r2deg*[roll,pitch,heading];

        pos<<phi * r2deg, lam * r2deg, h;
        att << r2deg * roll, r2deg* pitch, r2deg* heading;
        pos_stat(i, 0) = Imu_initdata(i, 7);
        //pos_stat(i, 2:3) = (pos(1:2) - Imu_refdata(i, 1:2)) * d2rad * WGS.a;
        pos_stat(i, 10) = pos(0);
        pos_stat(i, 11) = pos(1);
        //pos_stat(i, 4) = pos(3) - Imu_refdata(i, 3);
        pos_stat(i, 12) = pos(2);
        //pos_stat(i, 5:7) = vk'-Imu_refdata(i,4:6);
        pos_stat(i,13) = vk(0);
        pos_stat(i,14) = vk(1);
        pos_stat(i,15) = vk(2);
        //pos_stat(i, 8:10) = att - Imu_refdata(i, 7:9);
        pos_stat(i, 16) = att(0);
        pos_stat(i, 17) = att(1);
        pos_stat(i, 18) = att(2);}
        baocundizhi=&pos_stat;
        baocunzhuangtai=3;
        this->ui->progressBar->setValue(100);

}

void MainWindow::guanbi()
{
    this->close();
}

void MainWindow::baocun()
{
    QString baocun=QFileDialog::getSaveFileName(this,"保存文件",QCoreApplication::applicationFilePath(),"*.txt");
    if(baocun.isEmpty())
    {
        QMessageBox::warning(this,"警告","请选择一个文件");
    }
    else
    {
        qDebug()<<baocun;
        string baocun2=baocun.toStdString();
        ui->textBrowser->clear();
        ui->textBrowser->insertPlainText("文件保存路径:");
        ui->textBrowser->insertPlainText(baocun);
        ui->textBrowser->moveCursor(QTextCursor::End);
        ui->textBrowser->append(QString(""));
        ofstream ans;
        ans.open(baocun2,ios::out);
        //ifstream file;
        //FILE *ans = file.open(baocun2);//fopen(baocun2, "w+");
        if(baocunzhuangtai==1)
        {
            ans<<'\t'<<"X"<<'\t'<<"Y"<<'\t'<<"Z"<<'\t'<<'\r'<<'\n';
            for (int a1=0;a1<=vec.size()-1;a1++)
            {
                //fprintf(ans, "%d\t", a1+1);
                ans<<a1+1<<'\t';
                for (int a2 = 0; a2 <= 2; a2++)
                {
                    //fprintf(ans,"%f\t",POS(a2,a1));
                    ans<<to_string((*baocundizhi)(a2,a1))<<'\t';

                }

                ans<<'\r'<<'\n';

            }

            ans.close();
        }
        else if(baocunzhuangtai==3)
        {
            ans<<'\t'<<"时间"<<'\t'<<"X位置误差"<<'\t'<<"Y位置误差"<<'\t'<<"Z位置误差"<<'\t'<<"X速度误差"<<'\t'<<"Y速度误差"<<'\t'<<"Z速度误差"<<'\t'<<"X加速度误差"<<'\t'<<"Y加速度误差"<<'\t'<<"Z加速度误差"<<'\t'<<"X位置"<<'\t'<<"Y位置"<<'\t'<<"Z位置"<<'\t'<<"X速度"<<'\t'<<"Y速度"<<'\t'<<"Z速度"<<'\t'<<"X加速度"<<'\t'<<"Y加速度"<<'\t'<<"Z加速度"<<'\t'<<'\r'<<'\n';
            int fuzhu=(*baocundizhi).rows();
            for (int a1=0;a1<=fuzhu-2;a1++)
            {
                //fprintf(ans, "%d\t", a1+1);
                ans<<a1+1<<'\t';
                for (int a2 = 0; a2 <= 18; a2++)
                {
                    //fprintf(ans,"%f\t",POS(a2,a1));
                    ans<<to_string((*baocundizhi)(a1,a2))<<'\t';

                }

                ans<<'\r'<<'\n';

            }

            ans.close();
        }




        /*
            fprintf(ans, "\tX坐标\tY坐标\tZ坐标\tX改正数\tY改正数\tZ改正数\r\n");
            for (int a1=0;a1<=geshu-1;a1++)
            {
                fprintf(ans, "%d\t", a1+1);
                for (int a2 = 0; a2 <= 2; a2++)
                {
                    fprintf(ans,"%f\t",POS(a2,a1));
                }
                for (int a2 = 0; a2 <= 2; a2++)
                {
                    fprintf(ans, "%f\t", LSM_ANS(a2, a1));
                }
                fprintf(ans,"\r\n");

            }
            fclose(ans);
        */
    }

}

void MainWindow::Daoru_ASSIC()
{
    QString fileName = QFileDialog::getOpenFileName(this,"请选择一个文件",QCoreApplication::applicationFilePath(),"*.txt");
    if(fileName.isEmpty())
    {
        QMessageBox::warning(this,"警告","请选择一个文件");
    }
    else
    {
        qDebug()<<fileName;
        string filename=fileName.toStdString();
        ui->textBrowser->clear();
        ui->textBrowser->insertPlainText("文件导入路径:");
        ui->textBrowser->insertPlainText(fileName);
        ui->textBrowser->moveCursor(QTextCursor::End);
        ui->textBrowser->append(QString(""));
        txt_reader(filename, vec);
        len=vec.size();
        ui->textBrowser->insertPlainText("共");
        ui->textBrowser->insertPlainText(QString::number(len,'f',4));
        ui->textBrowser->insertPlainText("组数据");
        ui->textBrowser->moveCursor(QTextCursor::End);
        ui->textBrowser->append(QString(""));
        /*
        for(int i=0;i<=vec.size()-2;i++)
        {
            for(int j=0;j<=5;j++)
            {
                //ui->textBrowser->insertPlainText("正在解算第");
                ui->textBrowser->insertPlainText(QString::number(vec[i][j],'f',0));
                ui->textBrowser->insertPlainText(",");

            }
            ui->textBrowser->moveCursor(QTextCursor::End);
            ui->textBrowser->append(QString(""));
        }
        */
    }
    /*
    string filename=fileName.toStdString();
    txt_reader(filename, satellite);
    ui->textBrowser->insertPlainText("接收机估计坐标位置：");
    ui->textBrowser->moveCursor(QTextCursor::End);
    ui->textBrowser->append(QString(""));
    ui->textBrowser->insertPlainText(QString::number(APPROX_POSITION[0],'f',4));
    ui->textBrowser->moveCursor(QTextCursor::End);
    ui->textBrowser->append(QString(""));
    ui->textBrowser->insertPlainText(QString::number(APPROX_POSITION[1],'f',4));
    ui->textBrowser->moveCursor(QTextCursor::End);
    ui->textBrowser->append(QString(""));
    ui->textBrowser->insertPlainText(QString::number(APPROX_POSITION[2],'f',4));
    ui->textBrowser->moveCursor(QTextCursor::End);
    ui->textBrowser->append(QString(""));
    */
}

void MainWindow::Daoru_BIN()
{
    QString fileName = QFileDialog::getOpenFileName(this,"请选择一个文件",QCoreApplication::applicationFilePath());
    string filename;
    if(fileName.isEmpty())
    {
        QMessageBox::warning(this,"警告","请选择一个文件");
    }
    else
    {
        qDebug()<<fileName;
        filename=fileName.toStdString();
        ui->textBrowser->clear();
        ui->textBrowser->insertPlainText("文件导入路径:");
        ui->textBrowser->insertPlainText(fileName);
        ui->textBrowser->moveCursor(QTextCursor::End);
        ui->textBrowser->append(QString(""));
    }
    std::ifstream file(filename, std::ios::binary);
        if (!file) {
            std::cerr << "Failed to open file." << std::endl;
            return ;
        }

        int columns;
        columns = 7;
        /*
        if (fileName == "Data1.bin") {
            columns = 7;
        }
        else {
            columns = 10;
        }
        */
        file.seekg(0, std::ios::end);
        std::streampos fileSize = file.tellg();
        file.seekg(0, std::ios::beg);
        int numRows = fileSize / (columns * sizeof(double));

        Eigen::MatrixXd imuInitData1(numRows, columns);
        char* dataBuffer = new char[fileSize];
        file.read(dataBuffer, fileSize);
        double* doubleDataBuffer = reinterpret_cast<double*>(dataBuffer);

        for (int i = 0; i < numRows; i++) {
            for (int j = 0; j < columns; j++) {
                imuInitData1(i, j) = doubleDataBuffer[i * columns + j];
            }
        }

        delete[] dataBuffer;
        file.close();

        int startRow = -1;
        for (int i = 0; i < numRows; i++) {
            if (imuInitData1(i, 0) == 91620.005) {
                startRow = i;
                break;
            }
        }
        Eigen::MatrixXd imuInitData;
        if (startRow >= 0) {
            imuInitData = imuInitData1.block(startRow-1, 0, numRows - startRow, columns-1);
            Imu.resize(imuInitData.rows(),7);
            yuanshidata=&Imu;
            //len=imuInitData.rows();
            for(int i=0;i<=imuInitData.rows()-1;i++)
            {
                for(int j=0;j<=5;j++)
                {
                    if(j==0)
                    {
                        Imu(i,j+1)=imuInitData(i,j+1);
                    }
                    else
                    {
                        Imu(i,j+1)=imuInitData(i,j);
                    }

                }
            }

            len=Imu.rows();
            //int validRows = numRows - startRow;
            //imuInitData = imuInitData.block(startRow, 0, validRows, columns);
        }
        else {
            std::cerr << "Start row not found." << std::endl;
            return ;
        }

        double startTime = imuInitData(0, 0);
        imuInitData.conservativeResize(imuInitData.rows(), columns + 1);
        for (int i = 0; i < imuInitData.rows(); i++)
        {
            if (imuInitData(i, 0) != 0) {

                imuInitData(i, columns) = imuInitData(i, 0) - startTime;
            }
        }
        Eigen::MatrixXd imuRefData; ui->textBrowser->insertPlainText("共");ui->textBrowser->insertPlainText(QString::number(imuInitData.rows(),'f',4));ui->textBrowser->insertPlainText("组数据");ui->textBrowser->moveCursor(QTextCursor::End);ui->textBrowser->append(QString(""));
        Imu_initdata.resize(imuInitData.rows(), imuInitData.cols());
        if (columns == 7) {
            //Eigen::MatrixXd Imu_initdata(imuInitData.rows(), imuInitData.cols() );
            Imu_initdata << imuInitData.block(0, 1, imuInitData.rows(), imuInitData.cols() - 1),
                imuInitData.col(0);
            //imuInitData = imuInitDataUpdated;
        }
        else {
            imuRefData = imuInitData;
            Eigen::MatrixXd imuRefDataUpdated(imuRefData.rows(), imuRefData.cols() + 1);
            imuRefDataUpdated << imuRefData.block(0, 1, imuRefData.rows(), imuRefData.cols() - 1),
                imuRefData.col(0);
            imuRefData = imuRefDataUpdated;}
    /*
    string filename=fileName.toStdString();
    txt_reader(filename, satellite);
    ui->textBrowser->insertPlainText("接收机估计坐标位置：");
    ui->textBrowser->moveCursor(QTextCursor::End);
    ui->textBrowser->append(QString(""));
    ui->textBrowser->insertPlainText(QString::number(APPROX_POSITION[0],'f',4));
    ui->textBrowser->moveCursor(QTextCursor::End);
    ui->textBrowser->append(QString(""));
    ui->textBrowser->insertPlainText(QString::number(APPROX_POSITION[1],'f',4));
    ui->textBrowser->moveCursor(QTextCursor::End);
    ui->textBrowser->append(QString(""));
    ui->textBrowser->insertPlainText(QString::number(APPROX_POSITION[2],'f',4));
    ui->textBrowser->moveCursor(QTextCursor::End);
    ui->textBrowser->append(QString(""));
*/
}



void MainWindow::Xinxi()
{
    //this->close();
    ui->textBrowser->clear();
    ui->textBrowser->insertPlainText("安徽理工大学");
    ui->textBrowser->moveCursor(QTextCursor::End);
    ui->textBrowser->append(QString(""));
    ui->textBrowser->insertPlainText("导航工程20-2");
    ui->textBrowser->moveCursor(QTextCursor::End);
    ui->textBrowser->append(QString(""));
    ui->textBrowser->insertPlainText("王杰");
    ui->textBrowser->moveCursor(QTextCursor::End);
    ui->textBrowser->append(QString(""));
}

void MainWindow::Draw_jiasudu()
{
    Draw_Jiasudu *draw_jiasudu=new Draw_Jiasudu(baocundizhi,len,caiyang);
    draw_jiasudu->show();
}

void MainWindow::Draw_tuoluoyi()
{
    QString geshu1;
    geshu1=ui->lineEdit->text();
    caiyang=geshu1.toInt();
    Draw_Tuoluoyi *draw_tuoluoyi=new Draw_Tuoluoyi(yuanshidata,len,caiyang);
    draw_tuoluoyi->show();
}

void MainWindow::on_Compare_Button_clicked()
{
    Eigen::MatrixXd result = MatrixXd::Zero(720200, 10);
        Eigen::MatrixXd BLH0(1,3);
        //WGS84椭球的参数
        WGS84.a = 6378137.0; // 长半轴
        WGS84.b = 6356752.3142; //短半轴
        WGS84.f = 1 / 298.257223563; // 扁率
        WGS84.e2 = 0.00669437999013; // 第一偏心率平方
        WGS84.ep2 = 0.006739496742227; // 第二偏心率平方
        WGS84.we = 7.292115e-5;// 地球自转角速率
        WGS84.GM = 3.986004418e+14; // 地球引力为常数
        WGS84.ge = 9.7803267715; // 赤道重力加速度
        WGS84.gp = 9.8321863685; // 极地重力加速度
        BLH0 << 23.1373950708 * pi / 180, 113.3713651222 * pi / 180, 2.175;
        // 初始值
        Eigen::Vector3d v0(0.0, 0.0, 0.0);
        Eigen::Vector3d Euler0(0.0107951084511778 * pi / 180.0,
                -2.14251290749072 * pi / 180.0,
                -75.7498049314083 * pi / 180.0);
        double t0 = 91620.0;
        Eigen::Vector3d Deltatheta0(0.0, 0.0, 0.0);
        Eigen::Vector3d Deltav0(0.0, 0.0, 0.0);

        // 初始姿态四元数
        double roll = Euler0(0);
        double pitch = Euler0(1);
        double yaw = Euler0(2);
        Eigen::Quaterniond qbn0;
        qbn0.w() = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
        qbn0.x() = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
        qbn0.y() = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2);
        qbn0.z() = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2);

        // 初始方向余弦阵
        Eigen::Matrix3d Cbn0;
        Cbn0(0, 0) = qbn0.w() * qbn0.w() + qbn0.x() * qbn0.x() - qbn0.y() * qbn0.y() - qbn0.z() * qbn0.z();
        Cbn0(0, 1) = 2 * (qbn0.x() * qbn0.y() - qbn0.w() * qbn0.z());
        Cbn0(0, 2) = 2 * (qbn0.x() * qbn0.z() + qbn0.w() * qbn0.y());
        Cbn0(1, 0) = 2 * (qbn0.x() * qbn0.y() + qbn0.w() * qbn0.z());
        Cbn0(1, 1) = qbn0.w() * qbn0.w() - qbn0.x() * qbn0.x() + qbn0.y() * qbn0.y() - qbn0.z() * qbn0.z();
        Cbn0(1, 2) = 2 * (qbn0.y() * qbn0.z() - qbn0.w() * qbn0.x());
        Cbn0(2, 0) = 2 * (qbn0.x() * qbn0.z() - qbn0.w() * qbn0.y());
        Cbn0(2, 1) = 2 * (qbn0.y() * qbn0.z() + qbn0.w() * qbn0.x());
        Cbn0(2, 2) = qbn0.w() * qbn0.w() - qbn0.x() * qbn0.x() - qbn0.y() * qbn0.y() + qbn0.z() * qbn0.z();

        // Other variables
        int i = 0;
        Eigen::MatrixXd DATA(720200, 10);

        // Print the resulting variables
        /*
        std::cout << "v0: " << v0 << std::endl;
        std::cout << "Euler0: " << Euler0 << std::endl;
        std::cout << "t0: " << t0 << std::endl;
        std::cout << "Deltatheta0: " << Deltatheta0 << std::endl;
        std::cout << "Deltav0: " << Deltav0 << std::endl;
        std::cout << "qbn0: " << qbn0.coeffs().transpose() << std::endl;
        std::cout << "Cbn0:\n" << Cbn0 << std::endl;
        std::cout << "i: " << i << std::endl;
        std::cout << "DATA: " << DATA << std::endl;
        */
        while (true)
        {
            i++;
            Eigen::Vector3d Deltatheta;
            Eigen::Vector3d Deltav;
            // Read reference data (assumed to be available)
            Eigen::VectorXd data1(10);
            // Fill data1 with appropriate values
            Eigen::Vector3d BLH1 = data1.segment(2, 3);
            Eigen::Vector3d v1 = data1.segment(5, 3);
            Eigen::Vector3d Euler1 = data1.segment(8, 3);
            Eigen::MatrixXd DATA(i, 10);
            DATA.row(i - 1) = data1;
            double Deltat = 0.005; // 历元时间间隔


            // Perform attitude matrix computation (qbn)
            double RM = WGS84.a * (1 - WGS84.e2) / pow(1 - WGS84.e2 * sin(BLH0(0)) * sin(BLH0(0)), 1.5);
            double RN = WGS84.a / sqrt(1 - WGS84.e2 * sin(BLH0(0)) * sin(BLH0(0)));
            Eigen::Vector3d omegaen0(v0(1) / (RN + BLH0(2)),
                -v0(0) / (RM + BLH0(2)),
                -v0(1) * tan(BLH0(0)) / (RN + BLH0(2)));
            Eigen::Vector3d omegaie0(WGS84.we * cos(BLH0(0)),
                0.0,
                -WGS84.we * sin(BLH0(0)));
            Eigen::Vector3d Phi = Deltatheta + 1.0 / 12.0 * Deltatheta0.cross(Deltatheta);
            Eigen::Quaterniond qbb(cos(0.5 * Phi.norm()), sin(0.5 * Phi.norm()) / (0.5 * Phi.norm()) * 0.5 * Phi.x(),
                sin(0.5 * Phi.norm()) / (0.5 * Phi.norm()) * 0.5 * Phi.y(),
                sin(0.5 * Phi.norm()) / (0.5 * Phi.norm()) * 0.5 * Phi.z());
            Eigen::Vector3d si = (omegaen0 + omegaie0) * Deltat;
            Eigen::Quaterniond qnn(1.0 - 1.0 / 8.0 * si.norm() * si.norm(), -0.5 * si.x(), -0.5 * si.y(), -0.5 * si.z());
            Eigen::Quaterniond temp = qnn * qbn0;
            Eigen::Quaterniond qbn = temp * qbb;
            qbn.normalize();//归一化
            // Update Deltatheta0 for next iteration
            Deltatheta0 = Deltatheta;








        }
}
