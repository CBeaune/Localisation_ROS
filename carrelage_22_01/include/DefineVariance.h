#ifndef DEFINEVARIANCE_H
#define DEFINEVARIANCE_H

#include <eigen3/Eigen/Dense>
#include <math.h>
#include "RobotAndSensorDefinition.h"

using Eigen::MatrixXd;

// Set the parameters which have the "Determined by student" comment to tune
//the Kalman filter. Do not modify anything else in this file.

//Uncertainty on initial position of the robot.
float sigmaX     = 0.1;         // Determined by student
float sigmaY     = 0.1;         // Determined by student
float sigmaTheta = 0.1*M_PI/180 ;   // Determined by student



//Measurement noise.

//Constantes nÃ©cessaires au calcul
float T = samplingPeriod ; //Periode d echantillonage des capteurs
float Vmax = topRobotSpeed ; //Vitesse maximale admissible par le robot. A verifier. Pour l'instant on l'estime a 2m/s

float sigmaXmeasurement = 1/sqrt(3)*(width/2+T*Vmax) ; //determine par nous ; Vmax*T ; % demandé par le prof
float sigmaYmeasurement = 1/sqrt(3)*(width/2+T*Vmax) ; //determine par nous ; Vmax*T ; % demandé par le prof
float QgammaX = sigmaXmeasurement*sigmaXmeasurement;
float QgammaY = sigmaYmeasurement*sigmaYmeasurement;


// Input noise

float sigmaTuning = 0.001 ; //0.4 a la base, 0.1 mieux, 0.01 encore mieux, %0.0001 bon %0.003 d'apres charlotte

//Qbeta   = jointToCartesian * Qwheels * jointToCartesian.' ;

// State noise

//Qalpha = zeros(3) ;

// Mahalanobis distance threshold

//float mahaThreshold = sqrt(chi2inv(0.95,1)); // Determined by student
//TODO : Define the mahaThreshold 'DefineVariance.m'
//chi2(1 degre de liberte, intervalle de confiance a 0.95) = 3.84
float mahaThreshold = 3.84;

#endif
