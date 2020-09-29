//============================================================================
// Name        : Theta4_Control.cpp
// Author      : Gabrieis
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <armadillo>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <unistd.h>
#include "Sensor/SENSORS.hpp"
#include "Atuador/MOTOR.hpp"
#include "Atuador/SERVO.hpp"
#include "Controlador/CONTROL_THETA4_LQR.hpp"

using namespace std;
using namespace arma;

#define P9_42 "P9.42"  // pwmchip1 pwm0 pin P9.42 para o motor1
#define P9_22 "P9.22"  // pwmchip0 pwm0 pin P9.22 para o motor2

//#define coef_angular_motor1 0.0887
//#define coef_linear_motor1 -2.2374

//#define coef_angular_motor2 0.0822
//#define coef_linear_motor2 -2.0675


#define coef_a_motor1 0.0008
#define coef_b_motor1 0.0090
#define coef_c_motor1 -0.2854

#define coef_a_motor2 0.000459
#define coef_b_motor2 0.036316
#define coef_c_motor2 -0.944062


// Inicialização de variaveis

int numit, numInitIMU, trim;
double dt;
int in{1};
string file2save;
vector<double> imu_theta4;
vector<double> enc_theta12;
vector<double> rateNoFilter;
clock_t t0, tend;
int numStates{2},numInput{2},numRef{1},numEstadosArtificiais{2};
mat Forces(2,1);
double forces_trim[2];
mat Ref(1,1);
mat Estados(numit,6,fill::zeros);
mat data(numit,2,fill::zeros);
mat data_total;


int main() {

	// Lendo parametros do arquivo txt

	ifstream parametros_sistema;
	parametros_sistema.open("parametros_sistema.txt",ifstream::in);

	if (parametros_sistema.is_open())
	{
		parametros_sistema>> dt;         //time sample
		parametros_sistema>> numit;      //number of iterations
		parametros_sistema>> numInitIMU; // number of initalization
		parametros_sistema>> forces_trim[0];       // PWM value at hoover condition
		parametros_sistema>> forces_trim[1];       // PWM value at hoover condition
		parametros_sistema>> file2save; //file to save data name
		parametros_sistema>> Ref(0,0);  // Theta4 reference value

	}
	else{
		cerr << "Arquivo não existente ou não encontrado! "<< endl;
		return 1;
	}


	parametros_sistema.close();

	cout << "Parametros Iniciais"<< endl;
	cout << "Periodo de amostragem "<< dt <<  endl;
	cout << "Referencia Theta 4 "<< Ref(0,0)<<  endl;
	cout << "Forcas de trimagem "<< forces_trim[0] << "\t" << forces_trim[1]<<  endl;

	Sensors sensores(dt);
	//Control controle(numStates, numInput, numRef, numEstadosArtificiais, Ref, dt, forces_trim);

	//sensores.imuInitialization();

	//Motor motor1(P9_42,coef_a_motor1,coef_b_motor1,coef_c_motor1);
	//Motor motor2(P9_22,coef_a_motor2,coef_b_motor2,coef_c_motor2);



	//motor1.deactivate();
	//motor2.deactivate();



	return 0;
}
