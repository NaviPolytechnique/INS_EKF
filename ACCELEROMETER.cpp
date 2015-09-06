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
    ADVANCED_OFFSET_MARK = "$AAC";
    
    if (style != COLD_START){
        // TO DO
    }
}


int ACCELEROMETER::initOffsets(){
    int ret = Captor::initOffsets();
    if (ret == 1){
        captor_offsets[2] += GRAVITY_INT_PARIS; // We assume here that calibration was performed on an even surface
    }
    return ret;
}



void ACCELEROMETER::correctOutput(Eigen::Vector3f* acc_buffer, Eigen::Matrix3f _C){
    Captor::correctOutput(acc_buffer);
    Eigen::Vector3f _gravity;
    _gravity << 0,0,-GRAVITY_INT_PARIS;
    Eigen::Vector3f _rotate_gravity = (_C.transpose())*_gravity;
    *acc_buffer -= _rotate_gravity;
}



// What follows is the source code for the function used for the Gauss-Newton method calibration (see header). 
#ifdef ADCALIBRATION

int ACCELEROMETER::captureAdvancedCalibrationRoutine(){
    int row(0),column(0),num_pos(6);    // num_pos is the number of position we will put the IMU in,
                                        // thus is the number of rows of the storage matrix
    _ad_acc_data.setZero();
    std::string::size_type sz;
    while (row < num_pos){
        if (std::getline(captor_file,line)){
            if (line[2] == 'A'){
                std::stringstream ss(line);
                while (getline(ss,line,',') && column <3){
                    if (line == ADVANCED_OFFSET_MARK);
                    else {
                        _ad_acc_data(row,column) = std::stof(line,&sz);
                        column++;
                    }
                }
            }
            else return 0;
        }
        else return -1;
        column = 0; // Re - initializing column to zero
        row++;
    }
    //std::cout << _ad_acc_data << std::endl;
    return 1;
}


void ACCELEROMETER::initGNMethod(){
    // We set the first beta to be the one captured on length surface, should not be far from the final result
    for (int i=0; i<3; ++i){
        _beta(i) = captor_offsets(i);
        _beta(i+3) = captor_gain(i)/GRAVITY_INT_PARIS;
    }
}


float ACCELEROMETER::calculateSumSquareDiff(){
    float result(0);
    for (int i=0; i<6; ++i){
        result += _r_vector(i)*_r_vector(i); // S = sum(r_{i}^{2}) : the sum we're trying to minimize
    }
    return result;
}


void ACCELEROMETER::updateGNAlgorithm(){
    // We start by updating the residual vector
    _r_vector.setZero();
    for (int i=0; i<6; ++i){
        for (int j=0; j<3; j++){
            _r_vector(i) -= ((_ad_acc_data(i,j)-_beta(j))*_beta(j+3))*(_ad_acc_data(i,j)-_beta(j))*_beta(j+3);
        }
        _r_vector(i) += 1; // Each line of _r_vector is the difference between unity and the measured gravity vector norm normalized.
    }
    //std::cout << _r_vector.transpose() << std::endl;
    
    // We then update the jacobian matrix
    _jacobian.setZero();
    for (int i=0; i<6; ++i){
        for (int j=0; j<3; ++j){
            _jacobian(i,j) = -2*(_ad_acc_data(i,j)-_beta(j))*_beta(j+3)*_beta(j+3);
            _jacobian(i,j+3) = 2*(_ad_acc_data(i,j)-_beta(j))*(_ad_acc_data(i,j)-_beta(j))*_beta(j+3);
        }
    }
    
}


Vector6f ACCELEROMETER::findIncrement(){
    // Gauss-Newton method enable to calculate directly the miniisation direction 
    Vector6f delta = (_jacobian.transpose()*_jacobian).inverse()*(_jacobian.transpose())*_r_vector;
    return delta;
}


void ACCELEROMETER::correct_beta(){
    _beta += findIncrement();
}


int ACCELEROMETER::performAdvancedAccCalibration(){
    // Capturing data for the algorithm to work
    if (captureAdvancedCalibrationRoutine() != 1) return -1;
    else {
        int counter(0); // Counter so that it won't calculate for ever if fails
        initGNMethod();
        while (calculateSumSquareDiff() > 0.0000001 && counter <10){
            updateGNAlgorithm();
            correct_beta();
            counter++;
        }
        if (counter < 10) {
            std::cout << _beta.transpose() << std::endl;
            return 1;
        }
    }
    return 0;
}



void ACCELEROMETER::correct_GN(){
    for (int i=0; i<3; ++i){
        captor_offsets(i) = _beta(i); // Correcting the offsets
        captor_gain(i) = _beta(i+3)*GRAVITY_INT_PARIS; // Correcting the gain in m.s^-2 (they are in g's in the GN algorithm)
    }
}


#endif







