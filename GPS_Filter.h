//
//  GPS_Filter.h
//  INS_EKF
//
//  Created by Louis Faury on 25/07/2015.
//  Copyright (c) 2015 Louis Faury. All rights reserved.
//

#ifndef __INS_EKF__GPS_Filter__
#define __INS_EKF__GPS_Filter__

#include <stdio.h>
#include <string>
#include "/usr/include/Eigen/Dense"
#include <iostream>
#include "GPS.h"
#include "EKF.h"

typedef Eigen::Matrix<float,6,6> Matrix6f;
typedef Eigen::Matrix<float, 6, 1> Vector6f;

class GPS_Filter{
    
public:
    
    GPS_Filter();
    
    
    GPS_Filter(GPS*,EKF*);
    
    
    void calculateTransitionMatrix();
    
    
    void predict();
    
    
    void updateMesure();
    
    
    ~GPS_Filter();
    
    
private:
    
    Matrix6f P_;  // P(k)+
    Matrix6f _P;  // P(k)-
    Matrix6f R;
    Matrix6f Q;
    Matrix6f H;
    Matrix6f F;
    Matrix6f K;
    Vector6f _X;
    Vector6f X_;
    Vector6f Z;
    float dt;
    EKF* ekf;
    GPS* gps;

};






#endif /* defined(__INS_EKF__GPS_Filter__) */
