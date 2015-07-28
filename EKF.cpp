//
//  EKF.cpp
//  INS_EKF
//
//  Created by Louis Faury on 13/07/2015.
//  Copyright (c) 2015 Louis Faury. All rights reserved.
//

#include "EKF.h"

#define PI 3,141592
#define TODEG 57.295791



typedef Eigen::Matrix<float,10,1> Vector10f;
typedef Eigen::Matrix<float, 10, 10> Matrix10f;


EKF::EKF(){}



void EKF::init_state_vector(){
    X << 0,0,0,0,0,0,1,0,0,0;
   // std::cout << X.transpose() << std::endl;
}



Vector10f EKF::get_state_vector(){
    return X;
}



void EKF::build_jacobian_matrix(Eigen::Vector3f* acc, Eigen::Vector3f* gyro){
    
    // Position
    
    J.setZero();
    for (int i=0; i<3; i++){
        J(i,i+3) = dt; // Position = dt*speed
    }
    
    // Vitesse
    J(3,6)  =   2*dt*(X(6)*(*acc)(0)-X(9)*(*acc)(1)+X(8)*(*acc)(2));
    J(3,7)  =   2*dt*(X(7)*(*acc)(0)+X(8)*(*acc)(1)+X(9)*(*acc)(2));
    J(3,8)  =  2*dt*(-X(8)*(*acc)(0)+X(7)*(*acc)(1)+X(6)*(*acc)(2));
    J(3,9)  =  2*dt*(-X(9)*(*acc)(0)-X(6)*(*acc)(1)+X(7)*(*acc)(2));
    J(4,6)  =   2*dt*(X(9)*(*acc)(0)+X(6)*(*acc)(1)-X(7)*(*acc)(2));
    J(4,7)  =   2*dt*(X(8)*(*acc)(0)-X(6)*(*acc)(1)-X(6)*(*acc)(2));
    J(4,8)  =   2*dt*(X(7)*(*acc)(0)+X(8)*(*acc)(1)+X(9)*(*acc)(2));
    J(4,9)  =   2*dt*(X(6)*(*acc)(0)-X(9)*(*acc)(1)+X(8)*(*acc)(2));
    J(5,6)  =  2*dt*(-X(8)*(*acc)(0)+X(7)*(*acc)(1)+X(6)*(*acc)(2));
    J(5,7)  =   2*dt*(X(9)*(*acc)(0)+X(6)*(*acc)(1)-X(7)*(*acc)(2));
    J(5,8)  =  2*dt*(-X(6)*(*acc)(0)+X(9)*(*acc)(1)-X(8)*(*acc)(2));
    J(5,9)  =   2*dt*(X(7)*(*acc)(0)+X(8)*(*acc)(1)+X(9)*(*acc)(2));
    
    
    // Quaternion
    J(6,6)   =                   0;
    J(6,7)   =    -dt*(*gyro)(0)/2;
    J(6,8)   =    -dt*(*gyro)(1)/2;
    J(6,9)   =    -dt*(*gyro)(2)/2;
    J(7,6)   =     dt*(*gyro)(0)/2;
    J(7,7)   =                   0;
    J(7,8)   =     dt*(*gyro)(2)/2;
    J(7,9)   =    -dt*(*gyro)(1)/2;
    J(8,6)   =     dt*(*gyro)(1)/2;
    J(8,7)   =    -dt*(*gyro)(2)/2;
    J(8,8)   =                   0;
    J(8,9)   =     dt*(*gyro)(0)/2;
    J(9,6)   =     dt*(*gyro)(2)/2;
    J(9,7)   =     dt*(*gyro)(1)/2;
    J(9,8)   =    -dt*(*gyro)(0)/2;
    J(9,9)   =                   0;
    
    for (int i=0; i<10; i++){
        J(i,i) += 1;
    }
        
}



Eigen::Vector3f EKF::toRPY(Vector10f vector){
    float roll  = atan2(2*(vector(6)*vector(7)+vector(8)*vector(9)),1-2*(vector(7)*vector(7)+vector(8)*vector(8)));
    float pitch =                                                asin(2*(vector(6)*vector(8)-vector(9)*vector(7)));
    float yaw   = atan2(2*(vector(6)*vector(9)+vector(7)*vector(8)),1-2*(vector(8)*vector(8)+vector(9)*vector(9)));
    
    Eigen::Vector3f RPY(roll,pitch,yaw);
    return RPY;
}


Eigen::Vector3f EKF::getRPY(){
    float roll  = atan2(2*(X(6)*X(7)+X(8)*X(9)),1-2*(X(7)*X(7)+X(8)*X(8)));
    float pitch =                            asin(2*(X(6)*X(8)-X(9)*X(7)));
    float yaw   = atan2(2*(X(6)*X(9)+X(7)*X(8)),1-2*(X(8)*X(8)+X(9)*X(9)));
    
    Eigen::Vector3f RPY(roll,pitch,yaw);
    return RPY;
}



void EKF::predict(){
    X = J*X;
}



void EKF::init_state_vector(double heading){
    // We first calculate the quaternion linked with the heading
    float q0 = cos(heading/2);
    float q1 = 0;
    float q2 = 0;
    float q3 = sin(heading/2);
    
    X << 0,0,0,0,0,0,q0,q1,q2,q3;
}





    
    EKF::~EKF(){
    }
    

