#ifndef ACTUATOR_H
#define ACTUATOR_H


class Actuator
{
public:
    struct info_rparam
    {
        double d;
        double radius;
        double dt;
        double m;
    };
    Actuator(double d,double r, double dt, double m);
    Actuator(const info_rparam ir);
    ~Actuator();

    void update(double* srcPos, double (&dstPos)[3], double v, double w);

    info_rparam ip;
};

#endif // ACTUATOR_H
