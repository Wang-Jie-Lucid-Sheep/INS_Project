#include "draw_tuoluoyi.h"
#include "ui_draw_tuoluoyi.h"
#include <QChartView>
#include <QLineSeries>
#include <QValueAxis>
#include <QtWidgets/QVBoxLayout>
#include <qmath.h>
#include <QMessageBox>


QT_CHARTS_USE_NAMESPACE
Draw_Tuoluoyi::Draw_Tuoluoyi(Eigen::MatrixXd *jieguo, int geshu, int caiyang,QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::Draw_Tuoluoyi)
{
    ui->setupUi(this);
    this->setWindowTitle("Imu原始数据绘图");
    setWindowIcon(QIcon(":/new/diqiu.ico"));

    QWidget *subPage = new QWidget(this);
    setCentralWidget(subPage);


    int geshu1=0;
    // Create three line series with some sample data
    QLineSeries *series1 = new QLineSeries(this);
     series1->setName("X向陀螺输出");
     for(qreal a1=0;a1<=geshu-1;a1+=caiyang)
     {
         *series1<<QPointF(a1,((*jieguo)(a1,1))*10000);
         //*Drawx<<QPointF(a1,-5);
         geshu1++;
     }


     QLineSeries *series2 = new QLineSeries(this);
     series2->setName("Y向陀螺输出");
     for(qreal a1=0;a1<=geshu-1;a1+=caiyang)
     {
         *series2<<QPointF(a1,((*jieguo)(a1,2))*1000);
         //*Drawx<<QPointF(a1,-5);
     }

     QLineSeries *series3 = new QLineSeries(this);
     series3->setName("Z向陀螺输出");
     for(qreal a1=0;a1<=geshu-1;a1+=caiyang)
     {
         *series3<<QPointF(a1,((*jieguo)(a1,3))*1000);
         //*Drawx<<QPointF(a1,-5);
     }

     QLineSeries *series4 = new QLineSeries(this);
     series4->setName("X向加速度计输出");
     for(qreal a1=0;a1<=geshu-1;a1+=caiyang)
     {
         *series4<<QPointF(a1,((*jieguo)(a1,4))*1000);
         //*Drawx<<QPointF(a1,-5);
     }

     QLineSeries *series5 = new QLineSeries(this);
     series5->setName("Y向加速度计输出");
     for(qreal a1=0;a1<=geshu-1;a1+=caiyang)
     {
         *series5<<QPointF(a1,((*jieguo)(a1,5))*1000);
         //*Drawx<<QPointF(a1,-5);
     }

     QLineSeries *series6 = new QLineSeries(this);
     series6->setName("Z向加速度计输出");
     for(qreal a1=0;a1<=geshu-1;a1+=caiyang)
     {
         *series6<<QPointF(a1,((*jieguo)(a1,6))*1000);
         //*Drawx<<QPointF(a1,-5);
     }

    QValueAxis* axisX=new QValueAxis;
    axisX->setRange(1,geshu1+1);

    int APPROX2=(*jieguo)(0,1)*10000;
    QValueAxis* axisY=new QValueAxis;
    //axisY->setRange(APPROX2-20,APPROX2+20);



    // Create charts for each line series
    QChart *chart1 = new QChart();
    chart1->addSeries(series1);
    chart1->legend()->hide();
    chart1->setTitle("X向陀螺输出");
    //chart1->setAxisX(axisX,series1);
    //chart1->setAxisY(axisY,series1);


        QChart *chart2 = new QChart();
        chart2->addSeries(series2);
        chart2->legend()->hide();
        chart2->setTitle("Y向陀螺输出");
        //chart2->setAxisX(axisX,series2);

        QChart *chart3 = new QChart();
        chart3->addSeries(series3);
        chart3->legend()->hide();
        chart3->setTitle("Z向陀螺输出");
        //chart3->setAxisX(axisX,series3);

        QChart *chart4 = new QChart();
        chart4->addSeries(series4);
        chart4->legend()->hide();
        chart4->setTitle("X向加速度计输出");
        //chart4->setAxisX(axisX,series4);

        QChart *chart5 = new QChart();
        chart5->addSeries(series5);
        chart5->legend()->hide();
        chart5->setTitle("Y向加速度计输出");
        //chart5->setAxisX(axisX,series5);

        QChart *chart6 = new QChart();
        chart6->addSeries(series6);
        chart6->legend()->hide();
        chart6->setTitle("Z向加速度计输出");
        //chart6->setAxisX(axisX,series6);

    // Create chart views to display the charts
    QChartView *chartView1 = new QChartView(chart1);
    QChartView *chartView2 = new QChartView(chart2);
    QChartView *chartView3 = new QChartView(chart3);
    QChartView *chartView4 = new QChartView(chart4);
    QChartView *chartView5 = new QChartView(chart5);
    QChartView *chartView6 = new QChartView(chart6);
    chartView1->setRenderHint(QPainter::Antialiasing);

    // Create a layout for the subpage
    QGridLayout *layout = new QGridLayout(subPage);
    layout->addWidget(chartView1,1,1);
    layout->addWidget(chartView2,2,1);
    layout->addWidget(chartView3,3,1);
    layout->addWidget(chartView4,1,2);
    layout->addWidget(chartView5,2,2);
    layout->addWidget(chartView6,3,2);

    // Set the layout as the main layout for the subpage
    subPage->setLayout(layout);

}

Draw_Tuoluoyi::~Draw_Tuoluoyi()
{
    delete ui;
}
