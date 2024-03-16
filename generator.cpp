#include "generator.h"
#include "math.h"
#define INDEX_X 0
#define INDEX_Y 1
#define INDEX_Q 2

#define RAD(X) ((X)*(M_PI/180.0))

Generator::Generator(const info in, const Sensor& sen, const double* pos, const double* gpos)
{
    ip=in;
    s=new Sensor(sen);

    rPos[INDEX_X]=pos[INDEX_X];
    rPos[INDEX_Y]=pos[INDEX_Y];
    rPos[INDEX_Q]=pos[INDEX_Q];

    gPos[INDEX_X]=gpos[INDEX_X];
    gPos[INDEX_Y]=gpos[INDEX_Y];
    gPos[INDEX_Q]=gpos[INDEX_Q];

    rPath.push_back({rPos[INDEX_X],rPos[INDEX_Y],rPos[INDEX_Q]});
    m_localmin=false;
}

Generator::Generator(const Generator &gen)
{
    ip=gen.ip;
    s=new Sensor(*gen.s);

    rPos[INDEX_X]=gen.rPos[INDEX_X];
    rPos[INDEX_Y]=gen.rPos[INDEX_Y];
    rPos[INDEX_Q]=gen.rPos[INDEX_Q];

    gPos[INDEX_X]=gen.gPos[INDEX_X];
    gPos[INDEX_Y]=gen.gPos[INDEX_Y];
    gPos[INDEX_Q]=gen.gPos[INDEX_Q];

    rPath.push_back({rPos[INDEX_X],rPos[INDEX_Y],rPos[INDEX_Q]});
    m_localmin=false;
}

Generator::Generator(const Generator &gen, const double *pos)
{
    ip=gen.ip;
    s=new Sensor(*gen.s);

    rPos[INDEX_X]=pos[INDEX_X];
    rPos[INDEX_Y]=pos[INDEX_Y];
    rPos[INDEX_Q]=pos[INDEX_Q];

    gPos[INDEX_X]=gen.gPos[INDEX_X];
    gPos[INDEX_Y]=gen.gPos[INDEX_Y];
    gPos[INDEX_Q]=gen.gPos[INDEX_Q];

    rPath.push_back({rPos[INDEX_X],rPos[INDEX_Y],rPos[INDEX_Q]});
    m_localmin=false;
}

Generator::~Generator()
{
    delete s;
}

void Generator::setSensor(Sensor &sen)
{
    if(s!=nullptr)
    {
        delete s;
    }
    s=new Sensor(sen);
}

void Generator::updateSensor(Sensor &sen)
{
    s=&sen;
}

void Generator::setGoal(double *gpos)
{
    gPos[INDEX_X]=gpos[INDEX_X];
    gPos[INDEX_Y]=gpos[INDEX_Y];
    gPos[INDEX_Q]=gpos[INDEX_Q];
}

void Generator::setPos(double *pos)
{
    rPos[INDEX_X]=pos[INDEX_X];
    rPos[INDEX_Y]=pos[INDEX_Y];
    normalizeAngle(pos[INDEX_Q], rPos[INDEX_Q]);

    rPath.clear();
    rPath.push_back({rPos[INDEX_X],rPos[INDEX_Y],rPos[INDEX_Q]});
}

void Generator::attForce(double* pos, double* f)
{
    double x=pos[INDEX_X];
    double y=pos[INDEX_Y];
    double gx=gPos[INDEX_X];
    double gy=gPos[INDEX_Y];

    double dx=gx-x;
    double dy=gy-y;
    double dist=sqrt(pow(dx,2)+pow(dy,2));
    double cutoff=1.0;

    double k_vg=ip.f_param.aparam.k_vg;
    double q_v=ip.f_param.mparam.q_v;
    double q_g=ip.f_param.aparam.q_g;

    if(dist>cutoff)
    {
        f[INDEX_X]=-((k_vg*q_v*q_g)/(dist*dist))*(dx/dist);
        f[INDEX_Y]=-((k_vg*q_v*q_g)/(dist*dist))*(dy/dist);
    }
    else
    {
        f[INDEX_X]=-((k_vg*q_v*q_g))*(dx);
        f[INDEX_Y]=-((k_vg*q_v*q_g))*(dy);
    }
}

void Generator::repForce(int i, double* f)
{
    double d_o=ip.f_param.rparam.d_o;
    double k_vo=ip.f_param.rparam.k_vo;
    double q_v=ip.f_param.mparam.q_v;
    double q_o=ip.f_param.rparam.q_o;
    double dist=s->is[i].sense.dist;
    double v_sp[CORD_SIZE]={s->is[i].sense.vx,s->is[i].sense.vy};
    if(dist<d_o)
    {
        f[INDEX_X]=((k_vo*q_v*q_o)/(dist*dist))*(v_sp[INDEX_X]);
        f[INDEX_Y]=((k_vo*q_v*q_o)/(dist*dist))*(v_sp[INDEX_Y]);
    }
    else
    {
        f[INDEX_X]=0.0;
        f[INDEX_Y]=0.0;
    }
    if(dist<0)
    {
        f[INDEX_X]=0.0;
        f[INDEX_Y]=0.0;
    }
}

void Generator::quarkForce(int i, double* f)
{
    double d_o=ip.f_param.rparam.d_o;
    double k_vq=ip.f_param.qparam.k_vq;
    double q_v=ip.f_param.mparam.q_v;
    double q_q=ip.f_param.qparam.q_q;
    double dist=s->vq[i].sense.dist;
    double v_rq[CORD_SIZE]={s->vq[i].sense.vx,s->vq[i].sense.vy};
    if(dist<=d_o)
    {
        f[INDEX_X]=((k_vq*q_v*q_q)/(dist*dist))*(v_rq[INDEX_X]);
        f[INDEX_Y]=((k_vq*q_v*q_q)/(dist*dist))*(v_rq[INDEX_Y]);
    }
    else
    {
        f[INDEX_X]=0.0;
        f[INDEX_Y]=0.0;
    }
    if(dist<0)
    {
        f[INDEX_X]=0.0;
        f[INDEX_Y]=0.0;
    }
}

void Generator::force(double* pos, double* f,bool bQuark)
{
    double aForce[CORD_SIZE]={0.0,0.0};
    double rForce[CORD_SIZE]={0.0,0.0};
    double qForce[CORD_SIZE]={0.0,0.0};

    int s_num=s->ip.sparam.num_sensors;
    int q_num=s->vq.size();

    attForce(pos, aForce);
    for(int i=0;i<s_num;i++)
    {
        double rForce_temp[CORD_SIZE]={0.0,0.0};
        repForce(i, rForce_temp);
        rForce[INDEX_X]+=rForce_temp[INDEX_X];
        rForce[INDEX_Y]+=rForce_temp[INDEX_Y];
    }
    if(bQuark)
    for(int i=0;i<q_num;i++)
    {
        double qForce_temp[CORD_SIZE]={0.0,0.0};
        quarkForce(i, qForce_temp);
        qForce[INDEX_X]+=qForce_temp[INDEX_X];
        qForce[INDEX_Y]+=qForce_temp[INDEX_Y];
    }
    rForce[INDEX_X]=-rForce[INDEX_X];
    rForce[INDEX_Y]=-rForce[INDEX_Y];
    qForce[INDEX_X]=-qForce[INDEX_X];
    qForce[INDEX_Y]=-qForce[INDEX_Y];

    f[INDEX_X]=aForce[INDEX_X]+rForce[INDEX_X]+qForce[INDEX_X];
    f[INDEX_Y]=aForce[INDEX_Y]+rForce[INDEX_Y]+qForce[INDEX_Y];
}

void Generator::ref()
{
    double norm,ref,alpha;
    s->sense(rPos);
    double tForce[CORD_SIZE]={0.0,0.0};
    force(rPos,tForce,false);
    norm=sqrt(pow(tForce[INDEX_X],2)+pow(tForce[INDEX_Y],2));
    ref=atan2(tForce[INDEX_Y],tForce[INDEX_X]);
    checkMaxRef(ref,alpha);
    v_ref=norm*alpha;
    q_ref=ref;
}

void Generator::checkMaxRef(double ref, double &dst)
{
    double ret,error,max_error;
    double q=rPos[INDEX_Q];

    max_error=ip.m_param.eparam.theta_max;
    normalizeAngle(abs(ref-q),error);
    if(error<=max_error)
    {
        ret=(max_error-error)/max_error;
    }
    else
    {
        ret=0.0;
    }
    dst=ret;
}

void Generator::predict()
{
    double lam=ip.p_param.lparam.lam;
    double lam_stagnation=ip.p_param.lparam.lam_stagnation;
    double delta=ip.p_param.lparam.delta;
    double iter_max=(lam-lam_stagnation)/delta;

    for(int i=1;i<iter_max;i++)
    {
        double px=rPath[i-1].px;
        double py=rPath[i-1].py;
        double pq=rPath[i-1].pq;
        double pos[SIZE_STATE]={px,py,pq};
        double tForce[CORD_SIZE]={0.0,0.0};
        s->sense(pos);
        force(pos,tForce,true);

        double ref=atan2(tForce[INDEX_Y],tForce[INDEX_X]);
        double x=px+delta*cos(ref);
        x=addNoise(x,0.03);
        double y=py+delta*sin(ref);
        y=addNoise(y,0.03);
        double q;
        ref=addNoise(ref,RAD(3.0));
        normalizeAngle(ref,q);
        rPath.push_back({x,y,q});
    }
}

void Generator::detLocalmin()
{
    double mean_x,mean_y;
    mean_x=std::accumulate(rPath.begin(),rPath.end(),0.0,[](double sum, path p){return sum+p.px;});
    mean_x/=rPath.size();
    mean_y=std::accumulate(rPath.begin(),rPath.end(),0.0,[](double sum, path p){return sum+p.py;});
    mean_y/=rPath.size();

    if(rPath.empty()) return;

    bool ret=true;
    for(int i=0;i<rPath.size();i++)
    {
        double diff_x = rPath.at(i).px-mean_x;
        double diff_y = rPath.at(i).py-mean_y;
        double norm   = sqrt(pow(diff_x,2)+pow(diff_y,2));
        ret&=norm<ip.p_param.lparam.radius;
    }
    m_localmin=ret;

    double dInit[3]={rPath.at(0).px,rPath.at(0).py,rPath.at(0).pq};
    s->sense(dInit);
}

void Generator::gen(genmode mode)
{
    genmode m=mode;
    if(genmode::prediction==m)
    {
        predict();
        detLocalmin();
    }
    if(genmode::reference==m)
    {
        ref();
    }
}

void Generator::getRef(double &v, double &q)
{
    v=v_ref;
    q=q_ref;
}

void Generator::getStagPos(double *pos)
{
    if(m_localmin)
    {
        double mean_x, mean_y;
        mean_x=std::accumulate(rPath.begin(),rPath.end(),0.0,[](double sum, path p){return sum+p.px;});
        mean_x/=rPath.size();
        mean_y=std::accumulate(rPath.begin(),rPath.end(),0.0,[](double sum, path p){return sum+p.py;});
        mean_y/=rPath.size();
        pos[INDEX_X]=mean_x;
        pos[INDEX_Y]=mean_y;
    }
}
#include <random>
void Generator::getTemporaryGoal(double *pos)
{
    pos[INDEX_X]=temporaryGoal[INDEX_X];
    pos[INDEX_Y]=temporaryGoal[INDEX_Y];
    pos[INDEX_Q]=temporaryGoal[INDEX_Q];
}

void Generator::normalizeAngle(double angle, double &dst)
{
    // Normalize angle to be within the range [0, 2π)
    angle = fmod(angle, 2.0 * M_PI);
    // Shift angle to be within the range [-π, π)
    if (angle > M_PI) {
        angle -= 2.0 * M_PI;
    } else if (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    dst=angle;
}

double Generator::addNoise(double src, double noiseLevel)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> d(-noiseLevel,noiseLevel);
    return src+d(gen);
}

double Generator::calcTemporaryGoal()
{
    double rg_x,rg_y;
    double a,b,c;
    double dMax=0.0;
    int    index=0;
    int    iSize=rPath.size();

    rg_x=gPos[INDEX_X]-rPos[INDEX_X];
    rg_y=gPos[INDEX_Y]-rPos[INDEX_Y];

    a=-(rg_y/rg_x);
    b=1.0;
    c=-(a*rPos[INDEX_X]+rPos[INDEX_Y]);
    for(int i=0;i<iSize;i++)
    {
        double path[2]={rPath.at(i).px,rPath.at(i).py};
        double d=abs(a*path[INDEX_X]+b*path[INDEX_Y]+c)/sqrt(a*a+b*b);
        if(d>dMax)
        {
            dMax=d;
            index=i;
        }
    }
    temporaryGoal[INDEX_X]=rPath.at(index).px;
    temporaryGoal[INDEX_Y]=rPath.at(index).py;
    temporaryGoal[INDEX_Q]=rPath.at(index).pq;
    return dMax;
}

double Generator::calcTemporaryGoal(double *pos)
{
    double rg_x,rg_y;
    double a,b,c;
    double dMax=0.0;
    int    index=0;
    int    iSize=rPath.size();

    rg_x=gPos[INDEX_X]-pos[INDEX_X];
    rg_y=gPos[INDEX_Y]-pos[INDEX_Y];

    a=-(rg_y/rg_x);
    b=1.0;
    c=-(a*pos[INDEX_X]+pos[INDEX_Y]);
    for(int i=0;i<iSize;i++)
    {
        double path[2]={rPath.at(i).px,rPath.at(i).py};
        double d=abs(a*path[INDEX_X]+b*path[INDEX_Y]+c)/sqrt(a*a+b*b);
        if(d>dMax)
        {
            dMax=d;
            index=i;
        }
    }
    temporaryGoal[INDEX_X]=rPath.at(index).px;
    temporaryGoal[INDEX_Y]=rPath.at(index).py;
    temporaryGoal[INDEX_Q]=rPath.at(index).pq;
    return dMax;
}

bool Generator::isLocalmin()
{
    return m_localmin;
}

std::vector<Generator::path> Generator::getPath()
{
    return rPath;
}
