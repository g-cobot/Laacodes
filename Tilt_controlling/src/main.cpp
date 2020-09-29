#include <iostream>
#include <iomanip>
#include <fstream>
#include <stdio.h>
#include <armadillo>
#include "getStates.hpp"
#include "motor.hpp"
#include "Servo.hpp"
#include "control.hpp"
#include <unistd.h>


const int field1 {10};
const int field2 {10};
const int field3 {10};
const int field4 {10};
const int field5 {10};
//#include <bits/stdc++.h>

#define path_2_pwmchip "pwmchip0/"
#define export1 0 // Motor 1 export value (creates pwm0)
#define path_1_pwmchip "pwmchip1/"// motor address
#define export2 0 // Motor 1 export value (creates pwm1)

#define path_4_pwmchip "pwmchip6/"
#define export4 0 // Motor 1 export value (creates pwm0) Tilt 2 Motor 2
#define path_3_pwmchip "pwmchip6/"// motor address
#define export3 1 // Motor 1 export value (creates pwm1) Tilt 1 Motor 1

using namespace std;
using namespace arma;

int numit, numIT, trim, trimPWM,disbalance;
double dt;
int in{1};
std::string file2save;
std::vector<double> data;
std::vector<double> angle;
std::vector<double> enc0;
std::vector<double> enc;
std::vector<double> rateNoFilter; // output from low pass filter vector

//double eptrV, eptrH,eV0,eH0;
clock_t t0, tend; 
int numStates{6},numInput{4},numTrack{3}; // number of states, inputs and tracking variables
mat Ctrout(3,1);
mat Falpha(4,1);

mat Duty(2,1);
mat Ref(3,1);
double dutyAngle;

int main()
{
    // Getting system parameters from txt file
    ifstream sysParam;
    sysParam.open("System_parameters.txt");
    if (!sysParam)
	{
        std::cerr<< "File does not exist"<<std::endl;
    }
    sysParam>>dt; //time sample
    sysParam>>numit; //number of iterations
    sysParam>>numIT; // number of initialization
    sysParam>>trim; // PWM value at hoover condition
    sysParam>> file2save; //file to save data name
    sysParam>> Ref(0,0); // lateral reference value
    sysParam>> Ref(1,0); // altitude reference value
    sysParam>> Ref(2,0); // altitude reference value
    sysParam.close();
    mat armaSV(numit,6,fill::zeros); //state matrix

    getData system(dt);
    Control Ctr(numStates, numInput, numTrack, Ref, dt, trim);
    system.imuInitialization();
    Motor motor1(path_1_pwmchip, export1); // creating motor 1 object
    Motor motor2(path_2_pwmchip, export2); // creating motor 2 object
    Servo servo1(path_3_pwmchip, export3); // creating tilt 1 object
    Servo servo2(path_4_pwmchip, export4); // creating tilt 2 object
    
    std::cout << "Hit 0 to set initial state"<< std::endl;
    
    while (in!=0)
    {
        cin>>in;
    }
    // initializing filter and motor
    int cont_while = 0;
     while (cont_while < 100)
    {
        t0 = clock();
        data = system.imuRawData(); // return accelerations, angular velocities an non-filtered angles
        angle = system.imuFilteredData(data); 
        cont_while++;    
        motor1.setDutyCycle(1150000);
        motor2.setDutyCycle(1150000);
        system.waitSample(t0);
    }
   
    // set trim duty-cycle
    trimPWM = 1000000 + trim*10000;
    motor1.setDutyCycle(trimPWM);
    motor2.setDutyCycle(trimPWM);

    system.encoderInitialPosition(); // getting encoder initial position
    
    //eptrV = eV0; eptrH = eH0;  
    
    std::cout << "Bias Corrected"<< std::endl;
    
    ofstream  file;//file2;
    file.open(file2save);
    //file2.open("Comando.txt");

       
    // Begin experiment
    
    std::cout << "Hit 1 to begin experiment"<< std::endl;
     while (in!=1)
    {
        cin>>in;
    }
    
    std::cout << "Collecting Data"<< std::endl;
    
    int cont_while2 = 0;
    
    while (cont_while2 < numit)
    {    

    	t0 = clock();
    	data = system.imuRawData(); // return accelerations, angular velocities an non-filtered angles
        
    	angle = system.imuFilteredData(data); // return filtered roll and pitch angles and rates  x = [roll,roll_rate, pitch, pitch_rate];

    	enc = system.encoderPosition();  // return encoder angular position and rates x = [theta_1, theta_1_rate, theta_2, theta_2_rate];
        
        system.getStateVector(angle,enc,armaSV,cont_while2);
        
        // apply move average filter to velocity vector
        system.moveMean(armaSV,5,cont_while2);
        
        // system.printData(armaSV,cont_while2);

        //Ctrout = Ctr.computeOutput(armaSV,cont_while2);
        //Falpha = Ctr.computeForce(Ctrout);

        Falpha = Ctr.computeOutput(armaSV,cont_while2);
        //Falpha = Ctr.computeForce(Ctrout);

        //if(Falpha(2,0)>0.001){
        	//dutyAngle = Ctr.angle2duty(Falpha(2,0));

        	//servo1.setDutyCycle(dutyAngle);

        	//dutyAngle = Ctr.angle2duty(0);
        	//servo2.setDutyCycle(dutyAngle);


        	//dutyAngle = Ctr.angle2duty(Falpha(2,0));
        	//servo1.setDutyCycle(dutyAngle);

		    //dutyAngle = Ctr.angle2duty(0.0);
        	//servo2.setDutyCycle(dutyAngle);
        	//servo1.setAngle(Falpha(2,0));
        	//servo2.setAngle(0.0);
        	//servo1.setAngle(Falpha(2,0));
        	servo1.setAngle(Falpha(2,0));
            servo2.setAngle(Falpha(3,0));

       // }

       // else if(Falpha(2,0)<-0.001){
             //tiltAngle = Ctr.angle2duty(abs(Falpha(2,0)));
             //servo2.setDutyCycle(tiltAngle);
             //tiltAngle = Ctr.angle2duty(0);
             //servo1.setDutyCycle(tiltAngle);
        	//servo2.setAngle(abs(Falpha(2,0)));
        	//servo1.setAngle(0.0);
        	//dutyAngle = Ctr.angle2duty(abs(Falpha(2,0)));
        	//servo2.setDutyCycle(dutyAngle);

        	//dutyAngle = Ctr.angle2duty(0.0);
        	//servo1.setDutyCycle(dutyAngle);

       // }
       // else{

        	//servo2.setAngle(0.0);
        	//servo1.setAngle(0.0);

       // }


        //Alfa  Fz Fy
        cout<< setprecision(3)<<fixed;
        cout<< setw(field1)<<left<< Falpha(2,0) <<setw(field2)<<left<< armaSV(cont_while2,0) <<setw(field3)<<left<< armaSV(cont_while2,1) << setw(field4)<<left<< armaSV(cont_while2,2) << endl;
        //cout<< setw(field1)<<left<< Falpha(2,0) <<setw(field2)<<left<< Ctrout(0,0) <<setw(field3)<<left<< Ctrout(1,0) << setw(field4)<<left<< Ctrout(2,0) << endl;
        //cout <<setw(field2)<<left<< Ctrout(0,0) <<setw(field3)<<left<< Ctrout(1,0) << setw(field4)<<left<< Ctrout(2,0) << endl;

        cout<< setw(field1)<<left<< Falpha(0,0) <<setw(field2)<<left<< Falpha(1,0) <<setw(field3)<<left<< Falpha(2,0) << setw(field4)<<left<< Falpha(3,0) << endl;



        Duty = Ctr.force2duty(Falpha(span(0,1),0));


        motor1.setDutyCycle(Duty(0,0));
        motor2.setDutyCycle(Duty(1,0));  

        enc.push_back(Ctrout(0,0));
        enc.push_back(Ctrout(1,0));
        enc.push_back(Ctrout(2,0));
        enc.push_back(Falpha(0,0));
        enc.push_back(Falpha(1,0));
        enc.push_back(Falpha(2,0));

        // Saving states @ file
        system.saveData(file,enc);
        //system.saveData(file,angle);

        file << "\n";

        system.waitSample(t0);

	    cont_while2++;
	   
    }
    cout<< " Size State Matrix "<<size(armaSV)<< endl;
	file.close();	 
	std::cout << "Finished Data Acquisition"<< std::endl;
        
    // Deactivating ESC
    motor1.deactivateESC();
    motor2.deactivateESC();


	std::cout << "Putting 0"<< std::endl;
	servo1.setAngled(0);
	servo2.setAngled(0);
	std::cout << "Putting 10"<< std::endl;
	servo1.setAngled(10);
	servo2.setAngled(10);
	std::cout << "Deactivating"<< std::endl;
	servo1.deactivate();
    servo2.deactivate();


    return 0; 
}
