#include "robot.h"
#include "math.h"
#include <iostream>
Robot::Robot():
    m_running(false)
{
    Actuator::info_rparam ia;
    Sensor::info_param is;
    Generator::info ig;

    ia.d=0.08;
    ia.m=10.0;
    ia.dt=0.01;
    ia.radius=0.1;

    is.qparam.max_quark=50;
    is.sparam.max_dist=3.0;
    is.sparam.num_sensors=15;
    is.sparam.radius=ia.radius;

    ig.f_param.aparam.k_vg=1.0;
    ig.f_param.aparam.q_g=5.0;
    ig.f_param.mparam.q_v=-1.0;
    ig.f_param.rparam.d_o=0.2;
    ig.f_param.rparam.k_vo=1.0/(static_cast<double>(is.sparam.num_sensors)+80);
    ig.f_param.rparam.q_o=-1.0;
    ig.f_param.qparam.k_vq=5.0*ig.f_param.rparam.k_vo;
    ig.f_param.qparam.q_q=-1.0;

    ig.p_param.lparam.delta=0.05;
    ig.p_param.lparam.lam=3.0;
    ig.p_param.lparam.lam_stagnation=0.25*ig.p_param.lparam.lam;
    ig.p_param.lparam.radius=0.25*ig.p_param.lparam.lam_stagnation;

    ig.m_param.eparam.theta_max=30.0*M_PI/180.0;
    ig.m_param.eparam.tolorance=0.2;
    ig.m_param.vparam.v_max=0.2;
    ig.m_param.vparam.w_max=8.0;

    double pos[SIZE_STATE];
    double cgoal[SIZE_STATE];

    con.getPos(pos);
    con.addGoal(2.5,2.0,0.0);//add global goal
    con.getGoal(cgoal,true);

    sen=new Sensor(is);
    // sen->addLObs(1.8,2.52,1.79,2.72);
    // sen->addLObs(1.8,2.52,2.5,2.5);
    // sen->addLObs(2.5,2.5,2.48,1.8);
    // sen->addLObs(2.48,1.8,2.68,1.81);

    // sen->addCObs(2.0,6.0,1.0);
    // sen->addCObs(3.0,1.5,1.0);
    // sen->addCObs(4.0,4.8,0.8);
    // sen->addCObs(3.0,6.0,1.0);
    // sen->addCObs(4.0,6.0,1.0);
    // sen->addCObs(4.0,1.5,1.0);
    // sen->addCObs(5.0,6.0,1.0);
    // sen->addCObs(6.0,6.0,1.0);

    sen->addLObs(1.7,1.7,1.55,1.85);
    sen->addLObs(1.55,1.85,1.85,2.15);
    sen->addLObs(1.7,1.7,2.0,2.0);
    sen->addLObs(2.0,2.0,2.4,1.6);
    sen->addLObs(2.4,1.6,2.1,1.3);
    sen->addLObs(2.1,1.3,2.25,1.15);
    sen->addLObs(2.25,1.15,2.55,1.45);

    act=new Actuator(ia);
    gen=new Generator(ig,*sen,pos,cgoal);


    con.setSensor(sen);
    con.setActuator(act);
    con.setGenerator(gen);
}

Robot::~Robot()
{
    stop();
    delete gen;
    delete act;
    delete sen;
}

void Robot::start()
{
    if(!m_running)
    {
        m_running = true;
        m_worker = std::thread(&Robot::updateLoop, this);
    }
}

void Robot::run()
{
    m_running = true;
}

void Robot::stop()
{
    m_running=false;
    if(m_worker.joinable())
    {
        m_worker.join();
    }
}

std::vector<int> Robot::getiData()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_iData;
}

std::vector<double> Robot::getdData()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_dData;
}

void Robot::updateLoop()
{
    while(m_running)
    {
        con.control();
        updateData();
        {
            setDataUpdated(true);
        }
        m_condition.notify_all();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

bool Robot::isDataUpdated()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_update;
}

void Robot::setDataUpdated(bool updated)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_update = updated;
}

void Robot::updateData()
{
    std::vector<int> iData;
    std::vector<double> dData;

    iData.push_back(con.s->vc.size());//cobstacle size
    iData.push_back(con.s->vl.size());//lobstacle size
    iData.push_back(con.s->vq.size());//quark size
    iData.push_back(con.g->rPath.size());//future path size
    iData.push_back(con.s->ip.sparam.num_sensors);//sensor size

    for(int i=0;i<con.s->ip.sparam.num_sensors;i++)//sensor pos, data
    {
        dData.push_back(con.s->is[i].pos.x);
        dData.push_back(con.s->is[i].pos.y);
        dData.push_back(con.s->is[i].pos.q);
        dData.push_back(con.s->is[i].sense.dist);
        dData.push_back(con.s->is[i].sense.vx);
        dData.push_back(con.s->is[i].sense.vy);
    }
    for(int i=0;i<con.s->vc.size();i++)//cobs pos, radius
    {
        dData.push_back(con.s->vc[i].pos.x);
        dData.push_back(con.s->vc[i].pos.y);
        dData.push_back(con.s->vc[i].param.radius);
    }
    for(int i=0;i<con.s->vl.size();i++)//lobs pos
    {
        dData.push_back(con.s->vl[i].pos.x1);
        dData.push_back(con.s->vl[i].pos.y1);
        dData.push_back(con.s->vl[i].pos.x2);
        dData.push_back(con.s->vl[i].pos.y2);
    }
    for(int i=0;i<con.s->vq.size();i++)//quark pos
    {
        dData.push_back(con.s->vq[i].pos.x);
        dData.push_back(con.s->vq[i].pos.y);
    }
    for(int i=0;i<con.g->rPath.size();i++)//path pos
    {
        dData.push_back(con.g->rPath[i].px);
        dData.push_back(con.g->rPath[i].py);
    }

    dData.push_back(con.s->ip.sparam.max_dist);

    double rpos[SIZE_STATE];//robot pos
    con.getPos(rpos);
    dData.push_back(rpos[0]);
    dData.push_back(rpos[1]);
    dData.push_back(rpos[2]);

    double cgoal[SIZE_STATE];//robot goal
    con.getGoal(cgoal,false);
    dData.push_back(cgoal[0]);
    dData.push_back(cgoal[1]);

    dData.push_back(con.a->ip.radius);//robot radius
    dData.push_back(con.a->ip.dt);//robot dt

    m_iData = iData;
    m_dData = dData;
}










