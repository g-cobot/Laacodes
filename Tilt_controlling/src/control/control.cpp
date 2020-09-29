#include "control.h"


control::control(int numStates, int numCtrl) {
    
    ifstream controlGain;
    controlGain.open("GainMatrix.txt");
    if (!controlGain)
	{
        std::cerr<< "File does not exist"<<std::endl;
    }
    
    for (int i{0}; i<numCtrl; ++i){
        for (int j{0}; j<numStates; ++j){
            controlGain>>K(i,j);
        }
    }
}


arma::mat control::computeControl(arma::mat SV)
{
    return K*SV;  
}

