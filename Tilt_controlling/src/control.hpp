/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   control.h
 * Author: -F.Machini
 *
 * Created on July 20, 2020, 2:47 PM
 */

#ifndef CONTROL_H
#define CONTROL_H

#include <iostream>
#include <string>
#include <armadillo>
#include <fstream>
#define force2dutyRelation 1000000
#define rad2deg (180/3.1415)
#define deg2rad (3.1415/180)


using namespace std;
using namespace arma;

class Control{
	private:
	mat K, Kp, ref;
        mat err; 
        double ekm1, ek, wkm1, wk, dt;
        double l1{0.215},l2{0.225};
        double Fz;
	public:
	Control(int numStates, int numInput, int numTrack, arma::mat refVal, double timeSample, int trim);
	arma::mat computeOutput(arma::mat States,int cont);
	arma::mat force2duty(arma::mat forceVec);
        double PWM2force(int PWM);
        void motorSaturation(mat &duty);
        arma::mat computeForce(arma::mat cmd);
        double angle2duty(float angleVec);
};

#endif /* CONTROL_H */
