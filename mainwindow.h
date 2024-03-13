#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include "robot.h"
#include <thread>
#include <condition_variable>
#include <mutex>
QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    struct info_obs
    {
        int num_cobs;
        int num_lobs;
        int num_quark;
        int num_path;
    };
    struct info_sensor
    {
        int    num_sensors;
        double max_dist;
    };
    struct sensor_data
    {
        std::vector<double> x;
        std::vector<double> y;
        std::vector<double> q;
        std::vector<double> dist;
        std::vector<double> vx;
        std::vector<double> vy;
    };
    struct cobs_data
    {
        std::vector<double> x;
        std::vector<double> y;
        std::vector<double> radius;
    };
    struct lobs_data
    {
        std::vector<double> x1;
        std::vector<double> y1;
        std::vector<double> x2;
        std::vector<double> y2;
    };
    struct quark_data
    {
        std::vector<double> x;
        std::vector<double> y;
    };
    struct path_data
    {
        std::vector<double> px;
        std::vector<double> py;
    };
    struct robot_data
    {
        double x;
        double y;
        double q;
        double goal_x;
        double goal_y;
        double radius;
        double dt;
    };

    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void startWorker(Robot& r);
    void start();

private slots:
    void slotTimeout();

    void on_pushButton_clicked();

private:
    Ui::MainWindow *ui;
    QTimer timer;
    Robot r;
    std::thread m_worker;
    std::mutex m_mutex;
    // std::condition_variable condition;
    bool m_running;
    bool bReady;

    void worker(Robot& r);
    void stopWorker();
    void updateRobotData(std::vector<int> iData, std::vector<double> dData);
    void updateUi();

    info_obs m_obs;
    info_sensor m_sensor;
    sensor_data m_sensor_data;
    cobs_data m_cobs_data;
    lobs_data m_lobs_data;
    quark_data m_quark_data;
    path_data m_path_data;
    robot_data m_robot_data;
};
#endif // MAINWINDOW_H
