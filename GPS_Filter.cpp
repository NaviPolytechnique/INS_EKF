//
//  GPS_Filter.cpp
//  INS_EKF
//
//  Created by Louis Faury on 25/07/2015.
//  Copyright (c) 2015 Louis Faury. All rights reserved.
//

#include "GPS_Filter.h"



Vector6f getColumn(Matrix6_10d input, int i){
    Vector6f result;
    for (int j=0; j<input.RowsAtCompileTime; j++){
        result(j) = input (j,i);
    }
    return result;
}


GPS_Filter::GPS_Filter(){
}


GPS_Filter::GPS_Filter(GPS* gps) : gps(gps){
    // Initalisation of F
    F.setZero();
    for (int i=0; i<6; i++){
        F(i,i) = 1;
    }
    // Initalisation of H
    H.setZero();
    for (int i=0; i<6; i++){
        H(i,i) = 1;
    }
    // Initalisation of P_
    P_.setZero();
    for (int i=0; i<6; i++){
        P_(i,i) = 1;
    }
    // Initalisation of Q
    Q.setZero();
    Q(0,0) = 0.1;
    Q(1,1) = 0.2;
    Q(2,2) = 0.21;
    Q(3,3) = 0.23;
    Q(4,4) = 0.26;
    Q(5,5) = 0.3;
    
    // Initalisation of R
    R.setZero();
    R(0,0) = 1.63;
    R(1,1) = 1.14;
    R(2,2) = 2.15;
    R(3,3) = 3;
    R(4,4) = 2.85;
    R(5,5) = 5.3;
    // Initialisation of X_
    X_ << 0,0,0,0,0,0;
    // Initialisation of identity matrix
    I6.setZero();
    for (int i=0; i<6; i++){
        I6(i,i) = 1;
    }
    // Initialisation of K
    K.setZero();
    // Initialisation of mu
    mu.setZero();

}


void GPS_Filter::calculateTransitionMatrix(){
    dt = gps->time_since_last_update;
    F(3,0) = dt;
    F(4,1) = dt;
    F(5,2) = dt;
}


void GPS_Filter::predict(){
    calculateTransitionMatrix();
    _X = F*X_;
    _P = F*P_*(F.transpose()) + Q;
}


void GPS_Filter::updateMesure(){
    for (int i=0; i<3; ++i){
        Z(i) = (gps->ground_speed)(i);
    }
    for (int i=3; i<6; ++i){
        Z(i) = (gps->actual_position)(i-3);
    }
}


void GPS_Filter::updateFilter(){
    updateMesure();
    adaptRMatrix();
    K  = _P * (H.transpose())*((H*_P*(H.transpose())+R).inverse());
    X_ = _X + K*(Z-H*_X);
    P_ = (I6 - K*H)*_P;
    actual_position << X_(3),X_(4),X_(5);
    
    
}


Eigen::Vector3f GPS_Filter::getActualPosition() const{
    return actual_position;
}


void GPS_Filter::adaptRMatrix(){
    
    for (int j = 9; j > 0; j--) {
        for (int i=0; i<6; ++i){
            mu(i,j) = mu(i,j-1);
        }
    }
    
    for (int i=0; i<6; i++){
        mu(i,0) = (Z-H*_X)(i);
    }
    C.setZero();
    for (int i=0; i<10; ++i){
        C = C + (getColumn(mu,i)*(getColumn(mu,i).transpose()))/10;
    }
    R = C + H*_P*(H.transpose());
}


Vector6f GPS_Filter::getState() const{
    return X_;
}


GPS_Filter::~GPS_Filter(){
    
}














