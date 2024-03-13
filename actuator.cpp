#include "actuator.h"
#include "math.h"
#define INDEX_X 0
#define INDEX_Y 1
#define INDEX_Q 2
Actuator::Actuator(double d,double r, double dt, double m)
{
    ip.d=d;
    ip.radius=r;
    ip.dt=dt;
    ip.m=m;
}

Actuator::Actuator(const info_rparam ir)
{
    ip=ir;
}

Actuator::~Actuator()
{

}

void Actuator::update(double* srcPos, double (&dstPos)[3], double v, double w)
{
    double dx,dy,dq;

    double x=srcPos[INDEX_X];
    double y=srcPos[INDEX_Y];
    double q=srcPos[INDEX_Q];

    double c=cos(q);
    double s=sin(q);

    double r=ip.radius;
    double dt=ip.dt;

    if(w==0)
    {
        dx=0.5*r*v*c*dt;
        dy=0.5*r*v*s*dt;
    }
    else
    {
        dx=-v/w*s+v/w*sin(q+w*dt);
        dy=v/w*c-v/w*cos(q+w*dt);
    }
    dq=0.5*r*w*dt;

    dstPos[INDEX_X]=x+dx;
    dstPos[INDEX_Y]=y+dy;
    dstPos[INDEX_Q]=q+dq;
}
