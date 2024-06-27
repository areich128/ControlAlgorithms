#include "PD_Algo/PD.h"

using namespace std;

int main(){
    PD pd;
    pd.setKd(3);
    pd.setKp(1.5);
    pd.setPhi(10);
    pd.setRef(0);

    vector<double> theta_series;
    vector<double> phi_series;
    vector<double> error_series;

    int t_range = 1000;

    double initial_phi = pd.getPhi();
    double initial_theta = pd.getTheta();
    double initial_error = pd.getError(pd.getRef(), initial_phi);

    phi_series.push_back(initial_phi);
    theta_series.push_back(initial_theta);
    error_series.push_back(initial_error);

    ofstream file_out("output.txt");
    file_out << phi_series.at(0) << "," << theta_series.at(0) << endl;

    for (int t = 1; t < t_range; ++t) {
        // if (t == 175){
        //     pd.setRef(4);
        // }
        // if (t == 315){
        //     pd.setRef(0);
        // }

        double error = pd.getError(pd.getRef(), phi_series.back());
        error_series.push_back(error);

        double theta = pd.applyPD(error_series[t-1], error_series[t]);
        theta_series.push_back(theta);

        double new_phi = pd.applyDynamics(theta_series[t], phi_series[t-1]);
        phi_series.push_back(new_phi);

        pd.setTheta(theta);
        pd.setPhi(new_phi);

        file_out << phi_series.at(t) << "," << theta_series.at(t) << endl;
    }

    file_out.close();
    return 0;
}