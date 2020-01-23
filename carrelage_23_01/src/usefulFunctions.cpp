#include <eigen3/Eigen/Dense>
#include <math.h>
#include <iostream>
#include "usefulFunctions.h"


using Eigen::MatrixXd;

MatrixXd EvolutionModel(MatrixXd X, MatrixXd U ){

    if(X.rows()==3 && U.rows()==2){

        MatrixXd Y(3,1);
        Y(0,0)=X(0,0)+U(0,0)*cos(X(2,0));
        Y(1,0)=X(1,0)+U(0,0)*sin(X(2,0));
        Y(2,0)=X(2,0)+U(1,0);
       return(Y) ;

    }
    else{

        std::cout<<"ERROR IN THE MATRIX DIMENSION (EvolutionModel)"<<std::endl;
    return X;
    }


}

MatrixXd InverseEvolutionModel(MatrixXd X1, MatrixXd X2) {
    MatrixXd U(2,1);
    if(X1.rows()==3 && X2.rows()==3){

        U(0,0)=sqrt( (X1(0,0)-X2(0,0))*(X1(0,0)-X2(0,0)) + (X1(1,0)-X2(1,0))*(X1(1,0)-X2(1,0)) );
        U(1,0)=X1(2,0)-X2(2,0);
       return(U) ;

    }
    else{

        std::cout<<"ERROR IN THE MATRIX DIMENSION (InverseEvolutionModel)"<<std::endl;
    return X1;
    }
}

void MajX(int sensor,MatrixXd &X,MatrixXd oTm, MatrixXd mSensors, MatrixXd &P) {
    MatrixXd oRealSensor(3,1);
    MatrixXd oMeasSensor(3,1);
    MatrixXd mRealSensor(3,1);
    MatrixXd C(1,3);
    MatrixXd K(3,1);
    float Y;
    float Yhat;
    float innov;
    float dMaha;

    if(sensor==1){
    oMeasSensor(0,0) = mSensors(0,0) ;
    oMeasSensor(1,0) = mSensors(1,0) ;
    oMeasSensor(2,0) = mSensors(2,0) ;
    oMeasSensor = oTm * oMeasSensor ;
    }
    if(sensor==2){
    oMeasSensor(0,0) = mSensors(0,1) ;
    oMeasSensor(1,0) = mSensors(1,1) ;
    oMeasSensor(2,0) = mSensors(2,1) ;
    oMeasSensor = oTm * oMeasSensor ;
    }
    float x = (oMeasSensor(0,0)/xSpacing)-floor((oMeasSensor(0,0)/xSpacing))-0.5;
    float y= (oMeasSensor(1,0)/ySpacing)-floor((oMeasSensor(1,0)/ySpacing))-0.5;

    if (abs(x)>abs(y)) { //proche d'une verticale
          oRealSensor=oMeasSensor;
          oRealSensor(0,0)=round(oMeasSensor(0,0)/xSpacing)*xSpacing ;
          mRealSensor = oTm.inverse() * oRealSensor ;
          Y = mSensors(0,sensor-1);
          Yhat = mRealSensor(0,0);
          C(0,0)=1;
          C(0,1)=0;
          C(0,2)=-X(0,0)*sin(X(2,0))-X(1,0)*cos(X(2,0));
          innov=Y-Yhat;
          MatrixXd QGX(1,1);
          QGX = C*P*C.transpose();
          dMaha = innov *sqrt( 1/ (QGX(0,0) + QgammaX));
            if (dMaha<=mahaThreshold) {
                 K = 1/(QGX(0,0) +QgammaX) *P*C;
                  X+= innov*K;
                  P = (MatrixXd::Identity(3,3)-K*C)*P;
            }}
    else {
              oRealSensor=oMeasSensor;
              oRealSensor(0,0)=round(oMeasSensor(1,0)/ySpacing)*ySpacing ;
              mRealSensor = oTm.inverse() * oRealSensor ;
              Y = mSensors(1,sensor-1);
              Yhat = mRealSensor(1,0);
              C(0,0)=0;
              C(0,1)=1;
              C(0,2)=X(0,0)*cos(X(2,0))-X(1,0)*sin(X(2,0));
              innov=Y-Yhat;
              MatrixXd QGX(1,1);
              QGX = C*P*C.transpose();
              dMaha = innov *sqrt( 1/ (QGX(0,0) + QgammaY));
                if (dMaha<=mahaThreshold) {
                     K = 1/(QGX(0,0)+QgammaY) *P*C;
                      X+= innov*K;
                      P = (MatrixXd::Identity(3,3)-K*C)*P;
                }




    }

}
