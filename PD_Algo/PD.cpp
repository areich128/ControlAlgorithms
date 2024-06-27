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
    _phidotdot = _distance * _force * theta / _inertia;
    _phidot += (abs(_phi - prev_phi) / _dt) + (_phidotdot * _dt);

    //cout << "prev_phi = " << prev_phi << endl;
    // cout << "phi = " << _phi << endl;
    _phi = _phi - (_phidot * _dt) - (0.5 * (_phidotdot) * _dt * _dt);

    return _phi;
}

void PD::tune(){
    double x = (_force * _distance) / _inertia;
    _Kd = (2 * _damp * _w_n) / x;
    _Kp = (_w_n * _w_n) / x;
}

void PD::run(int version){
    cout << "Kp = " << getKp() << ", Kd = " << getKd() << endl;
    setPhi(10);
    setRef(0);

    vector<double> theta_series;
    vector<double> phi_series;
    vector<double> error_series;

    double initial_phi = getPhi();
    double initial_theta = getTheta();
    double initial_error = getError(getRef(), initial_phi);

    phi_series.push_back(initial_phi);
    theta_series.push_back(initial_theta);
    error_series.push_back(initial_error);

    ofstream file_out("output" + to_string(version) + ".txt");
    file_out << phi_series.at(0) << "," << theta_series.at(0) << endl;

    for (int t = 1; t < _t_range; ++t) {
        // if (t == 175){
        //     pd.setRef(4);
        // }
        // if (t == 315){
        //     pd.setRef(0);
        // }

        double error = getError(getRef(), phi_series.back());
        error_series.push_back(error);

        double theta = applyPD(error_series[t-1], error_series[t]);
        theta_series.push_back(theta);

        double new_phi = applyDynamics(theta_series[t], phi_series[t-1]);
        phi_series.push_back(new_phi);

        setTheta(theta);
        setPhi(new_phi);

        file_out << phi_series.at(t) << "," << theta_series.at(t) << endl;
    }

    file_out.close();
}