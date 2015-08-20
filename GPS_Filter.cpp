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
        P_(i,i) = 0.3;
    }
    // Initalisation of Q
    Q.setZero();
    Q(0,0) = 0.03;
    Q(1,1) = 0.043;
    Q(2,2) = 0.11;
    Q(3,3) = 0.023;
    Q(4,4) = 0.026;
    Q(5,5) = 0.1;
    
    // Initalisation of R
    R.setZero();
    R(0,0) = 1.63;
    R(1,1) = 1.14;
    R(2,2) = 2.15;
    R(3,3) = 3;
    R(4,4) = 2.85;
    R(5,5) = 5.3;
    // Initialisation of X_
    X_.setZero();
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


void GPS_Filter::predict(Eigen::Vector3f ekf_speed){
    calculateTransitionMatrix();
    for (int i=0; i<3; ++i){
        X_(i) = ekf_speed(i);
    }
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
    actual_speed << X_(0),X_(1),X_(2);
}


Eigen::Vector3f GPS_Filter::getCurrentPosition() const{
    return actual_position;
}


Eigen::Vector3f GPS_Filter::getCurrentSpeed() const{
    return actual_speed;
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



void GPS_Filter::updateSpeedEKF(Eigen::Vector3f ekf_speed){
    for (int i=0; i<3; ++i){
        X_(i) = (X_(i) +0.1*ekf_speed(i))/1.1;  // We're doing it the way of a low pass filter since captor is really noisy
                                                // Strong change in speed will then be detected while only speed drift will be ignored
    }
}


Vector6f GPS_Filter::getState() const{
    return X_;
}



Eigen::Matrix3f GPS_Filter::getSpeedCovMatrix() const{
    Eigen::Matrix3f result = P_.block<3,3>(0,0);
    return result;
}


Eigen::Matrix3f GPS_Filter::getPositionCovMatrix() const{
    Eigen::Matrix3f result = P_.block<3,3>(3,3);
    return result;
}


GPS_Filter::~GPS_Filter(){
    
}














