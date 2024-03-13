#ifndef SENSOR_H
#define SENSOR_H

#include <vector>
#include <thread>
#include <mutex>
#include <chrono>
#define SIZE_STATE 3
class Sensor
{
public:
    struct info_param
    {
        struct sensor_param
        {
            int num_sensors;
            double max_dist;
            double radius;//robot radius
        };
        struct quark_param
        {
            int max_quark;
        };
        sensor_param sparam;
        quark_param qparam;
    };
    struct info_cobs
    {
        struct cobs_pos
        {
            double x,y;
        };
        struct cobs_param
        {
            double radius;
        };
        cobs_pos pos;
        cobs_param param;
    };
    struct info_lobs
    {
        struct lobs_pos
        {
            double x1,y1,x2,y2;
        };
        struct lobs_unknown
        {
            double m,c;//y=mx+c
        };
        lobs_pos pos;
        lobs_unknown unknown;
    };
    struct info_quark
    {
        struct quark_pos
        {
            double x,y;
        };
        struct quark_sense
        {
            double dist;
            double vx,vy;
        };
        quark_pos pos;
        quark_sense sense;
    };
    struct info_sensor
    {
        struct sensor_pos
        {
            double x,y,q;
        };
        struct sensor_sense
        {
            double dist;
            double vx,vy;
        };
        sensor_pos pos;
        sensor_sense sense;
    };

    Sensor(int num_sensors, double max_dist, double radius, int max_quark);
    Sensor(const info_param in);
    Sensor(const Sensor& sen);
    ~Sensor();
    void updateSensorPos();
    void senseQuark();
    void senseCObs(int sensor);
    void senseLObs(int sensor);
    void selectDist(int sensor);

    virtual void sense(double *pos);

    void addQuark(double x,  double y);
    void addCObs (double x,  double y,  double radius);
    void addLObs (double x1, double y1, double x2, double y2);

    std::vector<info_cobs>  vc;//circular obstacles
    std::vector<info_lobs>  vl;//linear obstacles
    std::vector<info_quark> vq;//quark, sense data

    info_sensor *is;
    info_param  ip;

    double rpos[SIZE_STATE];

    info_sensor::sensor_sense *s;
};

#endif // SENSOR_H
