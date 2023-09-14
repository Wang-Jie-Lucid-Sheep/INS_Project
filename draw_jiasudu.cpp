#include "draw_jiasudu.h"
#include "ui_draw_jiasudu.h"
#include <QChartView>
#include <QLineSeries>
#include <QValueAxis>
#include <qmath.h>
#include <QMessageBox>
#include <QtWidgets/QGridLayout>
QT_CHARTS_USE_NAMESPACE

Draw_Jiasudu::Draw_Jiasudu(Eigen::MatrixXd *jieguo,int geshu,int caiyang,QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::Draw_Jiasudu)
{
    ui->setupUi(this);
    this->setWindowTitle("六方向绘图");
    setWindowIcon(QIcon(":/new/diqiu.ico"));
    //QGridLayout *baseLayout = new QGridLayout();//布局管理器
    QWidget *subPage = new QWidget(this);
    setCentralWidget(subPage);
    QVBoxLayout *layout = new QVBoxLayout(subPage);

    QChartView* xdraw=new QChartView(this);
    QChart* chart=new QChart();
    xdraw->setChart(chart);
    //setCentralWidget(xdraw);


    QLineSeries* Drawx=new QLineSeries;
    Drawx->setName("X补偿");
    chart->addSeries(Drawx);
    int geshu1=0;
    for(qreal a1=0;a1<=geshu-1;a1+=caiyang)
    {
        *Drawx<<QPointF(a1,((*jieguo)(0,a1))*1000);
        //*Drawx<<QPointF(a1,-5);
        geshu1++;
    }
    QValueAxis* axisX=new QValueAxis;
    axisX->setRange(1,geshu1+1);
    chart->setAxisX(axisX,Drawx);

    int APPROX2=(*jieguo)(0,0)*1000;

    QValueAxis* axisY=new QValueAxis;
    axisY->setRange(APPROX2-1,APPROX2+1);//(APPROX1[0]+15)/1000000,(APPROX1[0]-15)/1000000
    chart->setAxisY(axisY,Drawx);

    xdraw->setRenderHint(QPainter::Antialiasing);

    layout->addWidget(xdraw);

    //baseLayout->addWidget(xdraw,0,0);

    QChartView* ydraw=new QChartView(this);
    QChart* charty=new QChart();
    ydraw->setChart(charty);
    //setCentralWidget(ydraw);


    QLineSeries* Drawyx=new QLineSeries;
    Drawyx->setName("Y补偿");
    charty->addSeries(Drawyx);
    int geshuy1=0;
    for(qreal a1=0;a1<=geshu-1;a1+=caiyang)
    {
        *Drawyx<<QPointF(a1,((*jieguo)(1,a1))*1000);

        //*Drawyx<<QPointF(a1,-5);
        //Drawyx->append(a1,5);
        geshuy1++;
    }
    QValueAxis* axisyX=new QValueAxis;
    axisyX->setRange(1,geshuy1+1);
    charty->setAxisX(axisyX,Drawyx);

    int APPROXy2=(*jieguo)(1,1)*1000;

    QValueAxis* axisyY=new QValueAxis;
    axisyY->setRange(APPROXy2-1,APPROXy2+1);//(APPROX1[0]+15)/1000000,(APPROX1[0]-15)/1000000
    charty->setAxisY(axisyY,Drawyx);

    ydraw->setRenderHint(QPainter::Antialiasing);
   // baseLayout->addWidget(ydraw,1,0);
        //QVBoxLayout *layout = new QVBoxLayout(subPage);
        layout->addWidget(ydraw);

        QChartView* zdraw=new QChartView(this);
        QChart* chartz=new QChart();
        zdraw->setChart(chartz);
        //setCentralWidget(ydraw);


        QLineSeries* Drawzx=new QLineSeries;
        Drawzx->setName("Z补偿");
        chartz->addSeries(Drawzx);
        int geshuz1=0;
        for(qreal a1=0;a1<=geshu-1;a1+=caiyang)
        {
            *Drawzx<<QPointF(a1,((*jieguo)(2,a1))*100);

            //*Drawyx<<QPointF(a1,-5);
            //Drawyx->append(a1,5);
            geshuz1++;
        }
        QValueAxis* axiszX=new QValueAxis;
        axiszX->setRange(1,geshuz1+1);
        chartz->setAxisX(axiszX,Drawzx);

        int APPROXz2=(*jieguo)(2,2)*100;

        QValueAxis* axiszY=new QValueAxis;
        axiszY->setRange(APPROXz2-1,APPROXz2+1);//(APPROX1[0]+15)/1000000,(APPROX1[0]-15)/1000000
        chartz->setAxisY(axiszY,Drawzx);

        zdraw->setRenderHint(QPainter::Antialiasing);
       // baseLayout->addWidget(ydraw,1,0);
            //QVBoxLayout *layout = new QVBoxLayout(subPage);
            layout->addWidget(zdraw);

        subPage->setLayout(layout);
}

Draw_Jiasudu::~Draw_Jiasudu()
{
    delete ui;
}
