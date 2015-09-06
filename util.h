//
//  util.h
//  INS_EKF
//
//  Created by Louis Faury on 05/08/2015.
//  Copyright (c) 2015 Louis Faury. All rights reserved.
//

#ifndef INS_EKF_util_h
#define INS_EKF_util_h

#include <stdio.h>
#include </usr/include/Eigen/Dense>


#define PI 3.14159265
#define SPQ_DECLINATION_RAD 0.0052
#define HMC5843L_GAIN 660;
#define TODEG 57.295791
#define RT 6371000
#define GRAVITY_INT_PARIS 9.80665

#define ADCALIBRATION   // To compile all the code used for acc advanced calibration
                        // which take some non-negligable space (this is why we use pre-processors here)


typedef Eigen::Matrix<float,6,6> Matrix6f;
typedef Eigen::Matrix<float, 6, 1> Vector6f;
typedef Eigen::Matrix<double,6,10> Matrix6_10d;
typedef Eigen::Matrix<float,16,1> Vector16f;
typedef Eigen::Matrix<float,9,1> Vector9f;
typedef Eigen::Matrix<float, 16, 16> Matrix16f;



#endif
