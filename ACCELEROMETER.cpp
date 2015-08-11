//
//  ACCELEROMETER.cpp
//  INS_EKF
//
//  Created by Louis Faury on 12/07/2015.
//  Copyright (c) 2015 Louis Faury. All rights reserved.
//

#include <stdio.h>
#include "ACCELEROMETER.h"

ACCELEROMETER::ACCELEROMETER(){
}


ACCELEROMETER::ACCELEROMETER(const std::string acc_file_name, STYLE style) : Captor(acc_file_name){
    std::cout << "Acceleration file open" << std::endl;
    captor_id = "ACC";
    LINE_MARK = "$A";
    OFFSET_MARK = "$AO";
    
    if (style != COLD_START){
        // TO DO
    }
}


int ACCELEROMETER::initOffsets(){
    int ret = Captor::initOffsets();
    if (ret == 1){
        captor_offsets[2] += GRAVITY_INT_QRO; // We assume here that calibration was performed on an even surface
    }
    return ret;
}


void ACCELEROMETER::correctOutput(Eigen::Vector3f* acc_buffer, Eigen::Matrix3f _C){
    Captor::correctOutput(acc_buffer);
    Eigen::Vector3f _gravity;
    _gravity << 0,0,-GRAVITY_INT_QRO;
    Eigen::Vector3f _rotate_gravity = (_C.transpose())*_gravity;
    *acc_buffer -= _rotate_gravity;
}



