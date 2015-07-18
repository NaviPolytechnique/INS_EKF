//
//  main.cpp
//  INS_EKF
//
//  Created by Louis Faury on 12/07/2015.
//  Copyright (c) 2015 Louis Faury. All rights reserved.
//

#include <iostream>
#include "ACCELEROMETER.h"
#include "GYRO.h"
#include "GPS.h"
#include "MAGNETOMETER.h"
#include <unistd.h>
#include </usr/include/Eigen/Dense>
#include "EKF.h"


#define PI = 3,141592
#define g = 9.81


// Path to files
const std::string acc_file_path    =   "/Users/louisfaury/Documents/C++/APM_LOG/test/acc.txt";
const std::string gyro_file_path   =  "/Users/louisfaury/Documents/C++/APM_LOG/test/gyro.txt";
const std::string gps_file_path    =   "/Users/louisfaury/Documents/C++/APM_LOG/test/gps.txt";
const std::string mag_file_path    =   "/Users/louisfaury/Documents/C++/APM_LOG/test/mag.txt";
const std::string to_log_file_path =   "/Users/louisfaury/Documents/C++/APM_LOG/test/log_out.txt";




int main(int argc, const char * argv[]) {
    
    std::ofstream to_log(to_log_file_path.c_str());
    
    try {
        
    // Declaring objects used for inertial navigation
        ACCELEROMETER acc(acc_file_path, Captor::COLD_START);
        GYRO gyro(gyro_file_path, Captor::COLD_START);
        MAGNETOMETER mag(mag_file_path, Captor::COLD_START);
        GPS gps(gps_file_path);
        EKF ekf;
        
    
    
    
    // Getting offset
        int ret = acc.initOffsets();
        if (ret != 1) std::cout << "Couldn't get acc offsets !" << std::endl;
        else std::cout << "Acc offsets updated" << std::endl;
        
        ret = gyro.initOffsets();
        if (ret != 1) std::cout << "Couldn't get gyro offsets !" << std::endl;
        else std::cout << "Gyro offsets updated" << std::endl;
        
        ret = mag.initOffsets();
        if (ret != 1) std::cout << "Couldn't get magnetometer offsets !" << std::endl;
        else std::cout << "Mag offsets updated" << std::endl;
        
        ret = gps.setHome();
        if (ret != 1) std::cout << "Couldn't get HOME !" << std::endl;
        else std::cout << "HOME updated" << std::endl;
        
        
        
        
    // Reading line
        Eigen::Vector3f acc_vector_buffer;
        Eigen::Vector3f gyro_vector_buffer;
        ekf.init_state_vector();

        while (!acc.line_end && !gyro.line_end){
            acc.getOutput(&acc_vector_buffer);
            acc.correctOutput(&acc_vector_buffer);
            //std::cout << acc_vector_buffer.transpose() << std::endl;
            gyro.getOutput(&gyro_vector_buffer);
            gyro.correctOutput(&gyro_vector_buffer);
            //std::cout << gyro_vector_buffer(0) << std::endl;
            usleep(10000);

        
        
            //Predicting state vector
            ekf.build_jacobian_matrix(&acc_vector_buffer, &gyro_vector_buffer);
            ekf.predict();
            to_log << ekf.get_state_vector().transpose() << std::endl;
        }
    }
    catch (std::exception const& e)
    { std::cout << e.what() <<  " file " << std::endl; }
    
    

    
    return 0;
}
