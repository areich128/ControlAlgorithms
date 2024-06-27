#ifndef PD_H
#define PD_H

#include <iostream>
#include <string>
#include <cmath>
#include <vector>
#include <fstream>

using namespace std;

class PD {
    private:
        double _Kp;
        double _Kd;
        double _w_n;
        double _damp;
        double _phi;
        double _theta;
        double _ref;
        double _dt = 0.1;
        double _phidot;
        double _phidotdot;
        double _force = 1;
        double _distance = 1;
        double _inertia = 1;
        double _t_range = 200;

    public:
        PD();

        double getError(double, double);
        double applyPD(double, double);
        double applyDynamics(double, double);
        void tune();
        void run(int version);

        double getKp();
        void setKp(double);

        double getKd();
        void setKd(double);

        double getWn();
        void setWn(double);

        double getDamp();
        void setDamp(double);

        double getPhi();
        void setPhi(double);

        double getTheta();
        void setTheta(double);

        double getRef();
        void setRef(double);
};

#endif