#include "PD_Algo/PD.h"

using namespace std;

int main(){
    PD pd;
    pd.setDamp(1);
    pd.setWn(0.5);
    pd.tune();
    pd.run(1);

    pd.setWn(1);
    pd.tune();
    pd.run(2);

    pd.setWn(1.5);
    pd.tune();
    pd.run(3);

    return 0;
}