//
//  EKF.cpp
//  INS_EKF
//
//  Created by Louis Faury on 13/07/2015.
//  Copyright (c) 2015 Louis Faury. All rights reserved.
//

#include "EKF.h"

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
    J(4,7)  =   2*dt*(X(8)*(*acc)(0)-X(1)*(*acc)(1)-X(6)*(*acc)(2));
    J(4,8)  =   2*dt*(X(7)*(*acc)(0)+X(8)*(*acc)(1)+X(9)*(*acc)(2));
    J(4,9)  =   2*dt*(X(6)*(*acc)(0)-X(9)*(*acc)(1)+X(8)*(*acc)(2));
    J(5,6)  =  2*dt*(-X(8)*(*acc)(0)+X(6)*(*acc)(1)+X(6)*(*acc)(2));
    J(5,7)  =   2*dt*(X(9)*(*acc)(0)+X(6)*(*acc)(1)-X(7)*(*acc)(2));
    J(5,8)  =  2*dt*(-X(6)*(*acc)(0)+X(9)*(*acc)(1)-X(8)*(*acc)(2));
    J(5,9)  =   2*dt*(X(7)*(*acc)(0)+X(8)*(*acc)(1)+X(9)*(*acc)(2));
    
    
    // Quaternion
    J(6,6)   =                   0;
    J(6,7)   =    -dt*(*gyro)(0)/2;
    J(6,8)   =    -dt*(*gyro)(1)/2;
    J(6,9)   =    -dt*(*gyro)(2)/2;
    J(7,6)   =    -dt*(*gyro)(0)/2;
    J(7,7)   =                   0;
    J(7,8)   =     dt*(*gyro)(2)/2;
    J(7,9)   =     dt*(*gyro)(0)/2;
    J(8,6)   =     dt*(*gyro)(1)/2;
    J(8,7)   =    -dt*(*gyro)(2)/2;
    J(8,8)   =                   0;
    J(8,9)   =     dt*(*gyro)(0)/2;
    J(9,6)   =     dt*(*gyro)(2)/2;
    J(9,7)   =     dt*(*gyro)(1)/2;
    J(9,8)   =     dt*(*gyro)(0)/2;
    J(9,9)   =                   0;
    
    for (int i=0; i<10; i++){
        J(i,i) += 1;
    }
        
}





void EKF::predict(){
    X = J*X;
}
                   
        



    
    EKF::~EKF(){
    }
    

