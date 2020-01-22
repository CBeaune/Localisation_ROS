//-------------------------------------------------------------
//Conversion de Matlab a cpp faite :
// DefineVariance, RobotAndSensorDefinition, EvolutionModel, Localisation

//Il manque :
//LogData, PlotResults
//-------------------------------------------------------------

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU>
#include <math.h>
#include<iostream>
#include "usefulFunctions.h"

using Eigen::MatrixXd;
using namespace std;



int main(){

    MatrixXd Pinit(3,3) ;
           Pinit.topLeftCorner(3,3) = MatrixXd::Zero(3, 3);
           Pinit(0,0) = sigmaX*sigmaX ;
           Pinit(1,1) = sigmaY*sigmaY ;
           Pinit(2,2) = sigmaTheta*sigmaTheta ;
           std::cout<<Pinit<<std::endl;

    MatrixXd Qgamma(2,2);
    Qgamma.topLeftCorner(2,2) = MatrixXd::Zero(2, 2);
    Qgamma(0,0) = QgammaX ;
    Qgamma(1,1) = QgammaY ;

    MatrixXd Qwheels(2,2);
    Qwheels.topLeftCorner(2,2) = MatrixXd::Zero(2, 2);
    Qwheels(0,0) = sigmaTuning*sigmaTuning ;
    Qwheels(1,1) = sigmaTuning*sigmaTuning;



     MatrixXd jointToCartesian(2,2);
     jointToCartesian(0,0) = rwheel/2 ;
     jointToCartesian(0,1) = rwheel/2 ;
     jointToCartesian(1,0) = rwheel/trackGauge ;
     jointToCartesian(1,1) = -rwheel/trackGauge ;
     //cout<<jointToCartesian<<endl;

//     MatrixXd jointToCartesianFaux(2,2);
//     jointToCartesian(0,0) = rwheel/2 ;
//     jointToCartesian(0,1) = rwheel/2 ;
//     jointToCartesian(1,0) = rwheel/trackGauge ;
//     jointToCartesian(1,1) = -rwheel/trackGauge ;



    MatrixXd cartesianToJoint = jointToCartesian.inverse() ;
    //cout<<cartesianToJoint<<endl;

    MatrixXd Qbeta(2,2);
    Qbeta=jointToCartesian*Qwheels*jointToCartesian.transpose();

    MatrixXd Qalpha(2,2);
    Qalpha.topLeftCorner(2,2)=MatrixXd::Zero(2,2);

    MatrixXd mSensors(3,2);
    mSensors(0,0)=0;
    mSensors(0,1)=0;
    mSensors(1,0)=50;
    mSensors(1,1)=-50;
    mSensors(2,0)=1;
    mSensors(2,1)=1;



    //Position initiale
    MatrixXd Xinit(3,1) ;
    Xinit(0,0) = 0 ; //Defined by student
    Xinit(1,0) = 0 ; //Defined by student
    Xinit(2,0) = 0 ; //Defined by student

    MatrixXd X(3,1) ;
    X = Xinit ;

    MatrixXd U(2,1);

    MatrixXd P(3,3) ;
    P = Pinit ;

    MatrixXd A(3,3) ;
    MatrixXd B(3,2) ;

    MatrixXd oTm(3,3);

    //TODO : LogData ??

    //TODO : Entree boucle while sur les echantillons

        //TODO : Recuperer les donnees Odometrie du TurtleBot + capteurs Arduino
        //Pose + 2 capteurs


        ///TODO : odometrie
        //Deception du evolutionModel :(
        //on suppose qu'il nous donne X par rapport à la position initiale
        //tbot = le turtlebot
       // deltaX = getOdometry(tbot);
        //X = deltaX + Xinit ;

        //TODO : calcul de U
        // U = InverseEvolutionModel(X,X_precedent);
        //on calcule A et B (matrices du filtre de kalman) en fonction de X
        A(0,0) = 1;
        A(1,0) = 0;
        A(2,0) = 0;
        A(0,1) = 0;
        A(1,1) = 1;
        A(2,1) = 0;
        A(0,2) = -U(0,0)*sin(X(2,0));
        A(1,2) = U(0,0)*cos(X(2,0));
        A(2,2) = 1 ;

        B(0,0) = cos(X(2,0));
        B(1,0) = sin(X(2,0));
        B(2,0) = 0;
        B(0,1) = 0;
        B(1,1) = 0;
        B(2,1) = 1;
        //on calcule P la propagation d'erreur
        P = A*P*A.transpose() + B*Qbeta*B.transpose() + Qalpha ;

        float mesures=1; // pour test a supprimer
        if (mesures) {//si au moins un des deux capteurs detectent une ligne

            ///TODO : sensors
            //simple traduction du code matlab, mais loooong...
            oTm(0,0) = cos(X(2,0));
            oTm(1,0)=sin(X(2,0));
            oTm(2,0)=0;
            oTm(0,1) = -sin(X(2,0));
            oTm(1,1)=cos(X(2,0));
            oTm(2,1)=0;
            oTm(0,2) = X(0,0);
            oTm(1,2)=X(1,0);
            oTm(2,2)=1;

            if(mesures==1) {
            MajX(1,X,oTm,mSensors,P);
            //calculs
            }

            if(mesures==2) {
            MajX(2,X,oTm,mSensors,P);
            //calculs
            }

            if(mesures==3){
                MajX(1,X,oTm,mSensors,P);
                //calcul
                oTm(0,0) = cos(X(2,0));
                oTm(1,0)=sin(X(2,0));
                oTm(2,0)=0;
                oTm(0,1) = -sin(X(2,0));
                oTm(1,1)=cos(X(2,0));
                oTm(2,1)=0;
                oTm(0,2) = X(0,0);
                oTm(1,2)=X(1,0);
                oTm(2,2)=1;
                MajX(2,X,oTm,mSensors,P);
                //calcul
            }
    }


    //TODO : save all tunings and robot parameters

    //TODO : break ?

    //TODO : PlotResults






    return 0;
}
