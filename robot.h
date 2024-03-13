#ifndef ROBOT_H
#define ROBOT_H
#include <iostream>
#include <functional>
#include <thread>
#include <mutex>
#include <vector>
#include <condition_variable>
#include "controller.h"

class Robot
{
public:
    Robot();
    ~Robot();

    void start();
    void run();
    void stop();
    std::vector<int>    getiData();
    std::vector<double> getdData();
    //get robot state
    bool isDataUpdated();
    void setDataUpdated(bool updated);

    Sensor* sen;
    Actuator* act;
    Generator* gen;
    Controller con;
    std::condition_variable m_condition;
private:
    void updateLoop();

    void updateData();

    std::thread m_worker;
    std::mutex m_mutex;
    bool m_running;
    bool m_update;

    std::vector<int> m_iData;
    std::vector<double> m_dData;
};

#endif // ROBOT_H
