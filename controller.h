#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "actuator.h"
#include "sensor.h"
#include "generator.h"
class Controller
{
public:
    Controller();
    ~Controller();
    struct goal
    {
        double x;
        double y;
        double theta;
        double d;
        bool   arrived;
    };
    enum con_state
    {
        idle,
        localminimum
    };
    void setSensor(Sensor* sensor);//original pointer
    void setActuator(Actuator* actuator);//original pointer
    void setGenerator(Generator* generator);//original pointer
    void setTemporaryGoal(double x, double y, double theta, double d);
    void addGoal(double x, double y, double theta);
    void checkMaxVelocity(double vel, double vel_max, double& dst);
    void checkGoal();
    bool isArrived();
    bool checkGoal(std::vector<Generator::path> path,bool bGlobal);

    void velocity(double* src, double& v,double& w);
    void control();

    void getPos(double* dst);
    void getGoal(double* dst,bool bGlobal);
    void getConState(con_state& dst);
    Sensor* s;
    Actuator* a;
    Generator* g;
private:
    void updateGenerator();
    goal temporary;
    std::vector<goal> goals;

    double rPos[3];//robot state
    double esum;

    double kp,ki;
    con_state state;
};

#endif // CONTROLLER_H
