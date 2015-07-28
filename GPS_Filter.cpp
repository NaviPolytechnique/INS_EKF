//
//  GPS_Filter.cpp
//  INS_EKF
//
//  Created by Louis Faury on 25/07/2015.
//  Copyright (c) 2015 Louis Faury. All rights reserved.
//

#include "GPS_Filter.h"



GPS_Filter::GPS_Filter(GPS* gps,EKF* ekf) : gps(gps), ekf(ekf) {
    // Initalisation of F
    for (int i=0; i<6; i++){
        F(i,i) = 1;
    }
    // Initalisation of H
    for (int i=0; i<6; i++){
        H(i,i) = 1;
    }
    // Initalisation of _P
    for (int i=0; i<6; i++){
        _P(i,i) = 0.05;
    }
    // Initalisation of Q
    for (int i=0; i<6; i++){
        Q(i,i) = 0.2;
    }
    // Initalisation of R
    for (int i=0; i<6; i++){
        R(i,i) = 0.1;
    }
    // Initialisation of X_
    X_ << 0,0,0,0,0,0;
    
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
        Z(i) = (gps->actual_position)(i);
    }
    for (int i=3; i<5; ++i){
        Z(i) = (gps->ground_speed)(i-3);
    }
}







