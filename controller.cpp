#include "controller.h"
#include "math.h"
#define INDEX_REF_V 0
#define INDEX_REF_Q 1

#define INDEX_X 0
#define INDEX_Y 1
#define INDEX_Q 2
Controller::Controller():
    s(nullptr),
    a(nullptr),
    g(nullptr),
    temporary({0,0,0,0,true}),
    rPos({0,0,0}),
    esum(0.0),
    kp(6.0),
    ki(0.006),
    state(idle)
{

}

Controller::~Controller()
{
    delete g;
    delete a;
    delete s;
}

void Controller::setSensor(Sensor *sensor)
{
    if(s!=nullptr)
    {
        delete s;
    }
    s=sensor;
}

void Controller::setActuator(Actuator *actuator)
{
    if(a!=nullptr)
    {
        delete a;
    }
    a=actuator;
}

void Controller::setGenerator(Generator *generator)
{
    if(g!=nullptr)
    {
        delete g;
    }
    g=generator;
}

void Controller::setTemporaryGoal(double x, double y, double theta, double d)
{
    if(!(d>temporary.d)) return;
    temporary.x=x;
    temporary.y=y;
    temporary.theta=theta;
    temporary.arrived=false;
    temporary.d=d;
    //s->vq.clear();
    //esum=0.0;
}

void Controller::addGoal(double x, double y, double theta)
{
    goal g;
    g.x=x;
    g.y=y;
    g.theta=theta;
    g.arrived=false;
    goals.push_back(g);
}

void Controller::checkMaxVelocity(double vel, double vel_max, double &dst)
{
    if(abs(vel)<=vel_max)
    {
        dst=vel;
    }
    else
    {
        dst=vel_max*vel/abs(vel);
    }
}

void Controller::checkGoal()
{
    double goal[3];
    getGoal(goal,false);

    double dx=goal[INDEX_X]-rPos[INDEX_X];
    double dy=goal[INDEX_Y]-rPos[INDEX_Y];
    double d=sqrt(dx*dx+dy*dy);
    double tolorance=g->ip.m_param.eparam.tolorance;
    if(!temporary.arrived)
    {
        if(d<tolorance)
        {
            temporary.arrived=true;
            temporary.d=0.0;
        }
        return;
    }
    else
    {
        for(int i=0;i<goals.size();i++)
        {
            if(goals[i].arrived) continue;
            if(d<tolorance) goals[i].arrived=true;
        }
    }
}

bool Controller::isArrived()
{
    bool ret=true;
    for(int i=0;i<goals.size();i++)
    {
        ret&=goals[i].arrived;
    }
    return ret;
}

bool Controller::checkGoal(std::vector<Generator::path> path,bool bGlobal)
{
    double goal[3];
    getGoal(goal,bGlobal);
    for(int i=0;i<path.size();i++)
    {
        double dx=path[i].px-goal[INDEX_X];
        double dy=path[i].py-goal[INDEX_Y];
        double d=sqrt(dx*dx+dy*dy);
        if(d<g->ip.m_param.eparam.tolorance) return true;
    }
    return false;
}

void Controller::velocity(double *src, double &v, double &w)
{
    double v_max=g->ip.m_param.vparam.v_max;
    double w_max=g->ip.m_param.vparam.w_max;
    double v_ref=src[INDEX_REF_V];
    double q_ref=src[INDEX_REF_Q];
    double e;
    g->normalizeAngle(q_ref-rPos[INDEX_Q],e);

    esum+=e;
    checkMaxVelocity(v_ref,v_max,v);
    checkMaxVelocity(kp*e+ki*esum,w_max,w);
}

void Controller::control()
{
    if(isArrived())
    {
        return;
    }
    double lam=g->ip.p_param.lparam.lam;
    double lam_stagnation=g->ip.p_param.lparam.lam_stagnation;
    double delta=g->ip.p_param.lparam.delta;
    int    iter_max=(lam-lam_stagnation)/delta;

    std::vector<Generator*> pGen(iter_max-1);//future genarator
    double goal[3];
    getGoal(goal,false);
    g->setPos(rPos);
    g->setGoal(goal);
    g->gen(Generator::prediction);
    //Generator *test=nullptr;
    for(int i=0;i<iter_max-1;i++)
    {
        double pos[3]={g->rPath[i+1].px,g->rPath[i+1].py,g->rPath[i+1].pq};
        pGen[i]=new Generator(*g,pos);
        pGen[i]->gen(Generator::prediction);
    }
    int iLocalmin=-1;
    // if(checkGoal(g->rPath,false))
    // {
    //     if(checkGoal(g->rPath,true))
    //     {
    //         return;
    //     }
    //     double pg_goal[3];
    //     double g_goal[3];
    //     getGoal(pg_goal,false);
    //     getGoal(g_goal,true);
    //     test=new Generator(*g,pg_goal);
    //     test->setPos(pg_goal);
    //     test->setGoal(g_goal);
    //     test->gen(Generator::prediction);
    //     for(int i=0;i<test->rPath.size();i++)
    //     {
    //         g->rPath.push_back(test->rPath[i]);
    //     }
    //     if(test->isLocalmin())
    //     {
    //         iLocalmin=0;
    //     }
    // }
    for(int i=0;i<iter_max-1;i++)
    {
        std::vector<Generator::path> tempPath=pGen[i]->getPath();
        if(checkGoal(tempPath,false))
        {
            if(checkGoal(tempPath,true))
            {
                break;
            }
            double pg_goal[3];
            double g_goal[3];
            double pos[3];
            getGoal(pg_goal,false);
            getGoal(g_goal,true);
            pos[INDEX_X]=g->getPath()[i].px;
            pos[INDEX_Y]=g->getPath()[i].py;
            pos[INDEX_Q]=g->getPath()[i].pq;
            pGen[i]->setPos(pos);
            pGen[i]->setGoal(g_goal);
            pGen[i]->gen(Generator::prediction);
            if(pGen[i]->isLocalmin())
            {
                iLocalmin=i;
                break;
            }
        }
        if(pGen[i]->isLocalmin())
        {
            iLocalmin=i;
            break;
        }
    }  
    //i번째에서 checkgoal이 true인 경우 거기에서부터 다시 남은 generator만큼 predict한다.
    //위의 코드 참고해서 뒤에서 predict하는 친구는 이전의 path를 일부 참조해서 하는 것으로 해야할 것이다.
    if(iLocalmin==-1)
    {
        double d=0.0;
        double dmax=0.0;
        int idx=-1;
        for(int i=1;i<pGen.size();i++)
        {
            d=pGen[i]->calcTemporaryGoal();
            if(d>dmax)
            {
                dmax=d;
                idx=i;
            }
        }
        // Generator* q;
        // if(test==nullptr)
        // {
        //     q=g;
        // }
        // else
        // {
        //     q=test;
        // }
        if(!(d<0.01))
        {
            double tem[3];
            pGen[idx]->getTemporaryGoal(tem);
            setTemporaryGoal(tem[INDEX_X],tem[INDEX_Y],tem[INDEX_Q],d);//temporary goal의 생성 기준이 필요하다...
            state=idle;
            for(int i=0;i<pGen[idx]->rPath.size();i++)
            {
                g->rPath.push_back(pGen[idx]->rPath[i]);
            }
            updateGenerator();
        }
        getGoal(goal,false);
        g->setGoal(goal);
        double v_ref,q_ref,v,w;
        g->gen(Generator::reference);
        g->getRef(v_ref,q_ref);

        double ref[2]={v_ref,q_ref};
        velocity(ref,v,w);
        a->update(rPos,rPos,v,w);
    }
    else
    {
        double qpos[2];
        // Generator* q;
        // if(test==nullptr)
        // {
        //     return;
        // }
        // else
        // {
        //     q=test;
        // }
        // q->getStagPos(qpos);
        pGen[iLocalmin]->getStagPos(qpos);
        s->addQuark(qpos[INDEX_X],qpos[INDEX_Y]);
        state=localminimum;
        //localminimum 판단 함수의 추가 기능
        //1. temporary goal 도착을 예측한다.
        //2. temporary goal에 도착한다는 예측이 나오면 해당하는 g는 temporary goal을 시작 위치로 하는
        //   future path를 계산한다.
        //3. 계산 후 localminimum을 계산한다.
        //4. true일 경우 local을 벗어나게하는 진정한 temporary goal이 아닌 것이다. 따라서 localminimum결과값은 true다.
        //5. false일 경우 local을 벗어나게 하는 temporary goal이고 localminimum 결과값은 false다.
        //temporary골의 선택
        //temporary T 점에서 robot - goal 과의 수직하는 거리가 새로운 temporary goal T_n이 계산하는 robot - goal거리 d_n
        //보다 작으면 새로운 temporary 골이라고 정한다.
    }
    updateGenerator();
}

void Controller::getPos(double *dst)
{
    dst[INDEX_X]=rPos[INDEX_X];
    dst[INDEX_Y]=rPos[INDEX_Y];
    dst[INDEX_Q]=rPos[INDEX_Q];
}

void Controller::getGoal(double *dst,bool bGlobal)
{
    if(bGlobal)
    {
        for(int i=0;i<goals.size();i++)
        {
            if(!goals[i].arrived)
            {
                dst[INDEX_X]=goals[i].x;
                dst[INDEX_Y]=goals[i].y;
                dst[INDEX_Q]=goals[i].theta;
                return;
            }
        }
    }
    if(!temporary.arrived)
    {
        dst[INDEX_X]=temporary.x;
        dst[INDEX_Y]=temporary.y;
        dst[INDEX_Q]=temporary.theta;
        return;
    }
    else
    {
        for(int i=0;i<goals.size();i++)
        {
            if(!goals[i].arrived)
            {
                dst[INDEX_X]=goals[i].x;
                dst[INDEX_Y]=goals[i].y;
                dst[INDEX_Q]=goals[i].theta;
                return;
            }
        }
    }
}

void Controller::updateGenerator()
{
    g->updateSensor(*s);
    checkGoal();
}
