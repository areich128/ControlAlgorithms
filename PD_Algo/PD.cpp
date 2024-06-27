#include "PD.h"

//CONSTRUCTOR
PD::PD(){
    _Kp = 0;
    _Kd = 0;
    _w_n = 0;
    _damp = 0;
    _phi = 0;
    _theta = 0;
    _ref = 0;
    _phidot = 0;
}

//SETTERS AND GETTERS
double PD::getKp(){return _Kp;}
double PD::getKd(){return _Kd;}
double PD::getWn(){return _w_n;}
double PD::getDamp(){return _damp;}
double PD::getPhi(){return _phi;}
double PD::getTheta(){return _theta;}
double PD::getRef(){return _ref;}

void PD::setKp(double Kp){_Kp = Kp;}
void PD::setKd(double Kd){_Kd = Kd;}
void PD::setWn(double Wn){_w_n = Wn;}
void PD::setDamp(double damp){_damp = damp;}
void PD::setPhi(double phi){_phi = phi;}
void PD::setTheta(double theta){_theta = theta;}
void PD::setRef(double ref){_ref = ref;}

//CONTROL FUNCTIONS
double PD::getError(double ref, double output){
    return (ref - output);
}

double PD::applyPD(double prev_error, double error){
    double derivative = (error - prev_error) / _dt;
    double theta = -(_Kp * error) - (_Kd * derivative);
    if (theta > 15) {
        return 15;
    } else if (theta < -15) {
        return -15;
    }
    // double theta = 10;
    return theta;
}

double PD::applyDynamics(double theta, double prev_phi){
    double new_phi, force = 1, distance_to_axis = 1, inertia = 20;
    _phidotdot = distance_to_axis * force * theta / inertia;
    _phidot += (abs(_phi - prev_phi) / _dt) + (_phidotdot * _dt);

    //cout << "prev_phi = " << prev_phi << endl;
    // cout << "phi = " << _phi << endl;
    _phi = _phi - (_phidot * _dt) - (0.5 * (_phidotdot) * _dt * _dt);

    return _phi;
}