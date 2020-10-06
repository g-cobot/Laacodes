#include "CONTROL_THETA4_LQR.hpp"


Control::Control(int numStates, int numInput, int numRef, int numEstadosArtificiais, arma::mat refVal, double timeSample, double forces_trim[2]){
       	u0 << forces_trim[0] << endr
       	   << forces_trim[1] << endr;

       	ref.set_size(size(refVal));
        ref = refVal*deg2rad;

        K.set_size(numInput,numStates);

        ifstream in_file;
        in_file.open("Ganho_Estados.txt");
        if (!in_file)
        {
        	std::cerr<< "File is not open"<<std::endl;
        }
        for (int i{0}; i<numInput; ++i){
        	for (int j{0}; j<numStates; ++j){
        		in_file>>K(i,j);
        	}
        };
        in_file.close();

        //cout<< K(0,0) << K(0,1) << K(1,0) << K(1,1)<<endl;

        Nx.set_size(numStates,1);
    	Nx.zeros();

    	for(int j{0}; j<numStates; ++j){
    		if( j < numRef ){
    			Nx(j,0)=1;
    		}
    	};

    	//cout<< Nx(0,0)<< " " << Nx(1,0) <<endl;
}

arma::mat Control::computeU(arma::mat States){
    //mat Thetas(3,1);
    //mat Estados = trans(States);
    
    //Thetas(0,0) = States(0);  //Theta1
    //Thetas(1,0) = States(1);  //Theta2
   	//Thetas(2,0) = States(2);  //Theta4

	mat Estados(2,1);
	Estados << States(0,2) << endr
			<< States(0,5) << endr;

    return ((u0)+(K*Nx*ref-K*Estados));
}





