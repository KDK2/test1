#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "qcustomplot.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , m_running(false)
    , bReady(false)
{
    ui->setupUi(this);
    this->resize(1280,1024);
    ui->centralwidget->resize(1280,1024);
    ui->widget->resize(950,950);

    connect(&timer, &QTimer::timeout, this, &MainWindow::slotTimeout);
    timer.start(10);
}

MainWindow::~MainWindow()
{
    stopWorker();
    delete ui;
}

void MainWindow::startWorker(Robot& r)
{
    if(!m_running)
    {
        m_running = true;
        m_worker=std::thread(&MainWindow::worker,this, std::ref(r));
    }
}

void MainWindow::start()
{
    r.start();
    startWorker(r);
}

void MainWindow::stopWorker()
{
    m_running=false;
    if(m_worker.joinable())
    {
        m_worker.join();
    }
}

void MainWindow::slotTimeout()
{
    updateUi();
}

void MainWindow::worker(Robot& r)
{
    while (m_running)
    {
        std::vector<int> idata;
        std::vector<double> ddata;
        {
            std::unique_lock<std::mutex> lock(m_mutex);
            r.m_condition.wait(lock, [&](){return r.isDataUpdated();});

            idata = r.getiData();
            ddata = r.getdData();
        }
        updateRobotData(idata, ddata);
        bReady=true;
        r.setDataUpdated(false);
    }
}

void MainWindow::updateUi()
{
    if(!bReady) return;
    bReady=false;

    int num_sensor=m_sensor.num_sensors;
    int iCObs=m_obs.num_cobs;
    int iLObs=m_obs.num_lobs;
    int iQuark=m_obs.num_quark;
    int iPath=m_obs.num_path;
    double max_dist=m_sensor.max_dist;
    QVector<QCPCurveData> robot_sensor(num_sensor+1);
    QVector<QVector<QCPCurveData>> obs(iCObs);
    QVector<QVector<QCPCurveData>> quark(iQuark);
    QVector<QVector<QCPCurveData>> pth(iPath);
    QVector<QCPCurveData> goal;
    QVector<QCPCurve*> fermatObs;
    QVector<QCPCurve*> fermatQuark;
    QVector<QCPCurve*> fermatPath;
    QCPCurve* fermatGoal;
    QVector<QCPItemLine*> sensor_line;
    QCustomPlot *customPlot = ui->widget;
    QCPCurve *fermatCircle = new QCPCurve(customPlot->xAxis, customPlot->yAxis);

    for(int i=0;i<num_sensor;i++)
    {
        double d=m_sensor_data.dist[i];
        double x=m_sensor_data.x[i];
        double y=m_sensor_data.y[i];
        double q=m_sensor_data.q[i];
        if(d<0) d=max_dist;
        QCPItemLine* line = new QCPItemLine(customPlot);
        line->start->setCoords(x,y);
        line->end->setCoords(x+d*cos(q),y+d*sin(q));
        sensor_line.append(line);
        robot_sensor[i]=QCPCurveData(i, x, y);
    }
    for(int i=0;i<iLObs;i++)
    {
        double x1=m_lobs_data.x1[i];
        double y1=m_lobs_data.y1[i];
        double x2=m_lobs_data.x2[i];
        double y2=m_lobs_data.y2[i];
        QCPItemLine* line = new QCPItemLine(customPlot);
        line->start->setCoords(x1,y1);
        line->end->setCoords(x2,y2);
        sensor_line.append(line);
    }
    robot_sensor[num_sensor]=QCPCurveData(num_sensor, m_sensor_data.x[0], m_sensor_data.y[0]);
    sensor_line[0]->setPen(QPen(Qt::red));
    fermatCircle->data()->clear();
    fermatCircle->data()->add(robot_sensor,true);
    fermatCircle->setPen(QPen(Qt::blue));

    for(int i=0;i<iCObs;i++)
    {
        QCPCurve* obs_curve = new QCPCurve(customPlot->xAxis, customPlot->yAxis);
        fermatObs.append(obs_curve);
        double ox=m_cobs_data.x[i];
        double oy=m_cobs_data.y[i];
        double orad=m_cobs_data.radius[i];

        for(int j=0;j<num_sensor;j++)
        {
            obs[0].append(QCPCurveData(j, ox+orad*cos(2.0*M_PI*j/(num_sensor-1)), oy+orad*sin(2*M_PI*j/(num_sensor-1))));
        }
        QVector<QCPCurveData> obs_data=obs.takeFirst();
        fermatObs.at(i)->data()->add(obs_data,true);
        fermatObs.at(i)->setPen(QPen(Qt::green));
    }
    for(int i=0;i<iQuark;i++)
    {
        QCPCurve* quark_curve = new QCPCurve(customPlot->xAxis, customPlot->yAxis);
        fermatQuark.append(quark_curve);
        double qx=m_quark_data.x[i];
        double qy=m_quark_data.y[i];
        double qr=m_robot_data.radius-0.03;
        for(int j=0;j<num_sensor;j++)
        {
            quark[0].append(QCPCurveData(j, qx+qr*cos(2.0*M_PI*j/(num_sensor-1)), qy+qr*sin(2*M_PI*j/(num_sensor-1))));
        }
        QVector<QCPCurveData> quark_data=quark.takeFirst();
        fermatQuark.at(i)->data()->add(quark_data,true);
        fermatQuark.at(i)->setPen(QPen(Qt::blue));
    }
    for(int i=0; i<iPath;i++)
    {
        QCPCurve* path_curve = new QCPCurve(customPlot->xAxis, customPlot->yAxis);
        fermatPath.append(path_curve);
        double px=m_path_data.px[i];
        double py=m_path_data.py[i];
        double r =m_robot_data.radius;
        for(int j=0;j<num_sensor;j++)
        {
            pth[0].append(QCPCurveData(j, px+r*cos(2.0*M_PI*j/(num_sensor-1)), py+r*sin(2*M_PI*j/(num_sensor-1))));
        }
        QVector<QCPCurveData> path_data=pth.takeFirst();
        fermatPath.at(i)->data()->add(path_data,true);
        fermatPath.at(i)->setPen(QPen(Qt::yellow));
    }
    fermatGoal = new QCPCurve(customPlot->xAxis, customPlot->yAxis);
    double gx=m_robot_data.goal_x;
    double gy=m_robot_data.goal_y;
    double gr=m_robot_data.radius;
    for(int j=0;j<num_sensor;j++)
    {
        goal.append(QCPCurveData(j, gx+gr*cos(2.0*M_PI*j/(num_sensor-1)), gy+gr*sin(2*M_PI*j/(num_sensor-1))));
    }
    fermatGoal->data()->add(goal,true);
    fermatGoal->setPen(QPen(Qt::red));

    customPlot->addGraph();
    customPlot->xAxis->setLabel("x");
    customPlot->yAxis->setLabel("y");
    // customPlot->xAxis->setRange(m_robot_data.x-3.0, m_robot_data.x+3.0);
    // customPlot->yAxis->setRange(m_robot_data.y-3.0, m_robot_data.y+3.0);
    customPlot->xAxis->setRange(0.0, 3.0);
    customPlot->yAxis->setRange(0.0, 3.0);

    customPlot->replot();
    customPlot->clearPlottables();
    customPlot->clearItems();
}

void MainWindow::updateRobotData(std::vector<int> iData, std::vector<double> dData)
{
    m_obs.num_cobs=iData[0];
    m_obs.num_lobs=iData[1];
    m_obs.num_quark=iData[2];
    m_obs.num_path=iData[3];
    m_sensor.num_sensors=iData[4];

    int index=0;
    m_sensor_data.x.clear();
    m_sensor_data.y.clear();
    m_sensor_data.q.clear();
    m_sensor_data.dist.clear();
    m_sensor_data.vx.clear();
    m_sensor_data.vy.clear();
    for(int i=0;i<m_sensor.num_sensors;i++)
    {
        m_sensor_data.x.push_back(dData[index++]);
        m_sensor_data.y.push_back(dData[index++]);
        m_sensor_data.q.push_back(dData[index++]);
        m_sensor_data.dist.push_back(dData[index++]);
        m_sensor_data.vx.push_back(dData[index++]);
        m_sensor_data.vy.push_back(dData[index++]);
    }
    m_cobs_data.x.clear();
    m_cobs_data.y.clear();
    m_cobs_data.radius.clear();
    for(int i=0;i<m_obs.num_cobs;i++)
    {
        m_cobs_data.x.push_back(dData[index++]);
        m_cobs_data.y.push_back(dData[index++]);
        m_cobs_data.radius.push_back(dData[index++]);
    }
    m_lobs_data.x1.clear();
    m_lobs_data.y1.clear();
    m_lobs_data.x2.clear();
    m_lobs_data.y2.clear();
    for(int i=0;i<m_obs.num_lobs;i++)
    {
        m_lobs_data.x1.push_back(dData[index++]);
        m_lobs_data.y1.push_back(dData[index++]);
        m_lobs_data.x2.push_back(dData[index++]);
        m_lobs_data.y2.push_back(dData[index++]);
    }
    m_quark_data.x.clear();
    m_quark_data.y.clear();
    for(int i=0;i<m_obs.num_quark;i++)
    {
        m_quark_data.x.push_back(dData[index++]);
        m_quark_data.y.push_back(dData[index++]);
    }
    m_path_data.px.clear();
    m_path_data.py.clear();
    for(int i=0;i<m_obs.num_path;i++)
    {
        m_path_data.px.push_back(dData[index++]);
        m_path_data.py.push_back(dData[index++]);
    }
    m_sensor.max_dist=dData[index++];
    m_robot_data.x=dData[index++];
    m_robot_data.y=dData[index++];
    m_robot_data.q=dData[index++];
    m_robot_data.goal_x=dData[index++];
    m_robot_data.goal_y=dData[index++];
    m_robot_data.radius=dData[index++];
    m_robot_data.dt=dData[index++];
}


void MainWindow::on_pushButton_clicked()
{
    start();
}

