#ifndef GENERATOR_H
#define GENERATOR_H

#include <thread>
#include <mutex>
#include <vector>
#include <numeric>
#include "sensor.h"
#define CORD_SIZE 2
#define SIZE_STATE 3
class Generator
{
public:
    struct info
    {
        struct force_param
        {
            struct mobile_param
            {
                double q_v;
            };
            struct att_param
            {
                double k_vg, q_g;
            };
            struct rep_param
            {
                double k_vo, q_o, d_o;
            };
            struct quark_param
            {
                double k_vq, q_q;
            };
            mobile_param mparam;
            att_param aparam;
            rep_param rparam;
            quark_param qparam;
        };
        struct predict_param
        {
            struct length_param
            {
                double lam,delta,lam_stagnation,radius;
            };
            length_param lparam;
        };
        struct move_param
        {
            struct velocity_param
            {
                double v_max, w_max;
            };
            struct error_param
            {
                double tolorance;
                double theta_max;
            };
            velocity_param vparam;
            error_param eparam;
        };
        force_param f_param;//force
        predict_param p_param;//predict
        move_param m_param;//move
    };
    struct path
    {
        double px,py,pq;
    };

    enum genmode{reference,prediction};//artificial potential field, predict artificial potential field

    Generator(const info in, const Sensor& sen, const double* pos, const double* gpos);
    Generator(const Generator& gen);
    Generator(const Generator& gen, const double *pos);
    ~Generator();
    void setSensor(Sensor& sen);
    void updateSensor(Sensor& sen);
    void setGoal(double* gpos);
    void setPos(double* pos);
    void gen(genmode mode);
    void getRef(double &v, double &q);
    void getStagPos(double* pos);
    void getTemporaryGoal(double* pos);
    double addNoise(double src,double noiseLevel=0.01);
    double calcTemporaryGoal();
    double calcTemporaryGoal(double *pos);
    bool isLocalmin();
    std::vector<path> getPath();

    info ip;
    std::vector<path> rPath;
protected:
    void attForce(double* pos, double* f);
    void repForce(int i, double* f);
    void quarkForce(int i, double* f);

    void force(double* pos, double* f,bool bQuark);
    void ref();
    void checkMaxRef(double ref, double& dst);
    void predict();
    void detLocalmin();
private:
    Sensor* s;
    double rPos[SIZE_STATE];
    double gPos[SIZE_STATE];
    double temporaryGoal[SIZE_STATE];
    double v_ref,q_ref;
    bool m_localmin;
};

#endif // GENERATOR_H
