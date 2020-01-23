#ifndef ROBOTANDSENSORDEFINITION_H
#define ROBOTANDSENSORDEFINITION_H


// Constants defining the robot and sensor setup
// All lengths are in mm.


#include<math.h>

//Robot characteristics

float rwheel = 21.5 ;      // Wheel radius
float trackGauge       = 112  ;      // Distance between the fixed wheels
float realencoderRes = 180  ;      // In dots per wheel rotation

//Fausser l'odometrie
//Ajouté par Charlotte le 15/01
float dumbfactor = 10 ;
float encoderRes = realencoderRes/dumbfactor;


// To dumb down the robot's odometry, we artificially lower the
// encoder resolution by a certain factor.
// Also, we use a lower sampling frequency

float samplingFrequency    = 100 ;
float samplingPeriod       = 1/samplingFrequency ;

// The sensor is supposed to be orthogonal to axis Xm of the robot.

int noLineDetected =  0   ;      // Bit value when no magnet is detected
int lineDetected   =  1   ;      // Bit value when a magnet is detected

float topRobotSpeed = 1000 ;


// Homogeneous coordinates of the line detector sensors in robot frame Rm.

// One column per sensor.

////s1   s2  ...

//mSensors = [  0   0  ;
//             50 -50  ;
//              1   1  ] ;

//nbSensors = size(mSensors,2) ;


// Line spacing


float xSpacing = 300 ;

float ySpacing = 300 ;

float width = 5 ;


// ---------------------------------------------------------------

// The following are calculated from previous data. Do not modify.

// ---------------------------------------------------------------

float hwidth = width/2;

float dots2rad = (2*M_PI)/encoderRes ;

float rad2dots = 1/dots2rad        ;


//jointToCartesian = [ rwheel/2 rwheel/2 ; rwheel/trackGauge -rwheel/trackGauge ] ;
//jointToCartesianfaux = [ rwheel/2 rwheel/2 ; rwheel/trackGauge -rwheel/trackGauge ] ;

//cartesianToJoint = inv(jointToCartesian) ;

#endif
