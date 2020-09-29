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
	Control controle(numStates,numInput, numRef, numEstadosArtificiais, Ref,dt,forces_trim);

	sensores.imuInitialization();

	Motor motor1(P9_42,coef_a_motor1,coef_b_motor1,coef_c_motor1);
	Motor motor2(P9_22,coef_a_motor2,coef_b_motor2,coef_c_motor2);

	std::cout << "Pressione 0 para inicializar os motores! "<< std::endl;

	while (in!=0)
	{
	        cin>>in;
	}

	// initializing filter and motor
	int count_while = 0;
	motor1.setInitialMovement();
	motor2.setInitialMovement();

	while (count_while < 100)
	{
		t0 = clock(); imu_theta4 = sensores.imuRawData(); // return accelerations, angular velocities an non-filtered angles
		imu_theta4 = sensores.imuFilteredData(imu_theta4);
		count_while++;
		sensores.waitSample(t0);
	}

	motor1.setForceinNewton(forces_trim[0]);
	motor2.setForceinNewton(forces_trim[1]);

	sensores.encoderInitialPosition();

	std::cout << "Pressione 1 para inicializar o experimento!"<< std::endl;
	while (in!=1)
	{
		cin>>in;
	}

	count_while = 0;

	while (count_while < numit)
	{

		t0 = clock();

		imu_theta4 =sensores.imuRawData(); // return accelerations, angular velocities an non-filtered angles

		imu_theta4 =sensores.imuFilteredData(imu_theta4); // return filtered roll and pitch angles and rates  x = [roll,roll_rate, pitch, pitch_rate];

		enc_theta12 =sensores.encoderPosition();  // return encoder angular position and rates x = [theta_1, theta_1_rate, theta_2, theta_2_rate];

		sensores.getStateVector(imu_theta4,enc_theta12,Estados,count_while);

		// apply move average filter to velocity vector
		sensores.moveMean(Estados,5,count_while);

		sensores.printData(Estados.row(count_while));

		Forces = controle.computeU(Estados.row(count_while));

		motor1.setForceinNewton(Forces(0,0));
		motor2.setForceinNewton(Forces(1,0));

		sensores.waitSample(t0);

		data(count_while,0) = Forces(0,0);
		data(count_while,1) = Forces(1,0);

		count_while++;

	}

	//mat k = linspace<mat>(0,count_while-1,count_while);
	//data_total = join_cols(data,Estados);
	//data_total = join_cols(data_total,k);
	//data_total.save(file2save,raw_ascii);

	return 0;
}
