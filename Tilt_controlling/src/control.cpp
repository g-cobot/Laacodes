/*
 * control.cpp
 *
 *  Created on: Jul 8, 2020
 *      Author: -F.Machini
 */

#include "control.hpp"


Control::Control(int numStates, int numInput, int numTrack, arma::mat refVal, double timeSample, int trim){
        Fz = PWM2force(trim);
		ref.set_size(size(refVal));
        err.set_size(size(refVal)); err.zeros();
        ref = refVal*deg2rad;
        std::cerr<< "Reference set to: Theta_1 "<< refVal(0,0) << " and Theta_2 "<< refVal(1,0) << " and Theta_4 "<< refVal(2,0) << std::endl;
        dt = timeSample;
	K.set_size(numInput,numStates);
        Kp.set_size(numInput,numTrack);
	ifstream in_file;
	in_file.open("Kx.txt");
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
        ifstream in_file2;
	in_file2.open("Kp.txt");
	if (!in_file2)
	{
		std::cerr<< "File is not open"<<std::endl;
        }
	for (int i{0}; i<numInput; ++i){
	        for (int j{0}; j<numTrack; ++j){
	        in_file2>>Kp(i,j);
	    }
	};
        in_file2.close();
        Kp.print();
        K.print();
}

arma::mat Control::computeOutput(arma::mat States, int cont){
    mat Aux2(3,1);
    mat Aux1 = trans(States(cont,span::all));
    
    Aux2(0,0)= States(cont,0); //Theta1
    Aux2(1,0) = States(cont,1);//Theta2
    Aux2(2,0)= States(cont,2);//Theta4

   
    err += (ref - Aux2)*dt;

    return (Kp*err-K*Aux1);
}

arma::mat Control::computeForce(arma::mat cmd){
	//cmd fz fy tauz
    mat Aux(3,1); // f1 f2 alpha
    float Fzi= 2*Fz+(cmd(0,0));
	if(cmd(1,0)>0.0001){
		Aux(1,0) = (l1*Fzi-cmd(2,0))/(l1+l2);      // F2
		Aux(2,0) = atan2(cmd(1,0),(Fzi-Aux(1,0))); // Alpha
		Aux(0,0) = cmd(1,0)/sin(Aux(2,0));        // F1
	}
	else if(cmd(1,0)<-0.0001){
		Aux(0,0) = (l2*Fzi+cmd(2,0))/(l1+l2);      //F1
		Aux(2,0) = atan2(cmd(1,0),(Fzi-Aux(0,0))); //Alpha
		Aux(1,0) = cmd(1,0)/sin(Aux(2,0));        //F2
		}
	else if(cmd(1,0)>-0.0001){
		Aux(0,0) = (cmd(0,0)+ (cmd(2,0)/l2))/( 1+ (l1/l2)); // F1
		Aux(1,0) = (cmd(1,0)- Aux(0,0));					// F2
		Aux(2,0) = 0.0;										// Alpha
		}

	return Aux;
}


arma::mat Control::force2duty(arma::mat forceVec){

    mat duty(2,1);


    duty(0,0) = 1000000 + ((forceVec(0,0) + 1.39 )/0.0917)*10000;
    duty(1,0) = 1000000 + ((forceVec(1,0) + 1.39 )/0.0917)*10000;

    return duty;
}

double Control::angle2duty(float angleVec){

    double duty;
    
    duty = (-18.27632*(angleVec*rad2deg) +  2.2799e+03)*1000; // Tilt 1

    return duty;
}


double Control::PWM2force(int PWM){
    
    return 0.0917*PWM - 1.39;
}




