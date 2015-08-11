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
#include "GPS_Filter.h"
#include "util.h"





// Path to reading files
const std::string acc_file_path        =        "/Users/louisfaury/Documents/C++/APM_LOG/test/acc.txt";
const std::string gyro_file_path       =       "/Users/louisfaury/Documents/C++/APM_LOG/test/gyro.txt";
const std::string gps_file_path        =        "/Users/louisfaury/Documents/C++/APM_LOG/test/gps.txt";
const std::string mag_file_path        =        "/Users/louisfaury/Documents/C++/APM_LOG/test/mag.txt";

// Path to output file
const std::string to_log_file_path     =    "/Users/louisfaury/Documents/C++/APM_LOG/test/log_out.txt";
const std::string rpy_file_path        =        "/Users/louisfaury/Documents/C++/APM_LOG/test/rpy.txt";
const std::string gps_out_path         =    "/Users/louisfaury/Documents/C++/APM_LOG/test/gps_out.txt";
const std::string gps_filter_out_path  = "/Users/louisfaury/Documents/C++/APM_LOG/test/gps_filter.txt";
const std::string mag_out_path         =    "/Users/louisfaury/Documents/C++/APM_LOG/test/mag_out.txt";

/*
// Path to reading files
const std::string acc_file_path        =        "/Users/louisfaury/Documents/C++/APM_LOG/immobile2/acc.txt";
const std::string gyro_file_path       =       "/Users/louisfaury/Documents/C++/APM_LOG/immobile2/gyro.txt";
const std::string gps_file_path        =        "/Users/louisfaury/Documents/C++/APM_LOG/immobile2/gps.txt";
const std::string mag_file_path        =        "/Users/louisfaury/Documents/C++/APM_LOG/immobile2/mag.txt";

// Path to output file
const std::string to_log_file_path     =    "/Users/louisfaury/Documents/C++/APM_LOG/test/log_out.txt";
const std::string rpy_file_path        =        "/Users/louisfaury/Documents/C++/APM_LOG/test/rpy.txt";
const std::string gps_out_path         =    "/Users/louisfaury/Documents/C++/APM_LOG/test/gps_out.txt";
const std::string gps_filter_out_path  = "/Users/louisfaury/Documents/C++/APM_LOG/test/gps_filter.txt";
const std::string mag_out_path         =    "/Users/louisfaury/Documents/C++/APM_LOG/test/mag_out.txt";
*/




int main(int argc, const char * argv[]) {
    
    std::ofstream to_log(to_log_file_path.c_str());
    std::ofstream to_rpy(rpy_file_path.c_str());
    std::ofstream gps_out(gps_out_path.c_str());
    std::ofstream gps_filter_out(gps_filter_out_path.c_str());
    std::ofstream mag_out(mag_out_path.c_str());
    
    try {
        
    // Declaring objects used for inertial navigation
        ACCELEROMETER acc(acc_file_path, Captor::COLD_START);
        GYRO gyro(gyro_file_path, Captor::COLD_START);
        MAGNETOMETER mag(mag_file_path, Captor::COLD_START);
        GPS gps(gps_file_path,GPS::UBLOX);
        GPS_Filter gps_filter(&gps);
        EKF ekf(&gps_filter,&acc,&gyro,&mag);
        
    
    
    
    // Getting offset and gain for accelerometer, gyroscope and magnetometer. Getting GPS HOME field.
    // This routine will warn you if one of the offset couldn't be updated
        int ret = acc.initOffsets();
        if (ret != 1) std::cout << "Couldn't get acc offsets !" << std::endl;
        else std::cout << "Acc offsets updated" << std::endl;
        ret = acc.initGain();
        if (ret != 1) std::cout << "Couldn't get acc gain !" << std::endl;
        else std::cout << "Acc gain updated" << std::endl;

        ret = gyro.initOffsets();
        if (ret != 1) std::cout << "Couldn't get gyro offsets !" << std::endl;
        else std::cout << "Gyro offsets updated" << std::endl;
        ret = gyro.initGain();
        if (ret != 1) std::cout << "Couldn't get gyro gain !" << std::endl;
        else std::cout << "Gyro gain updated" << std::endl;
        
        ret = mag.initOffsets();
        if (ret != 1) std::cout << "Couldn't get magnetometer offsets !" << std::endl;
        else std::cout << "Mag offsets updated" << std::endl;
        
        ret = gps.setHome();
        if (ret != 1) std::cout << "Couldn't get HOME !" << std::endl;
        else std::cout << "HOME updated" << std::endl;
        
        
        
    // Reading lines
        // We create the buffer for the sensor's output values
        Eigen::Vector3f acc_vector_buffer;
        Eigen::Vector3f gyro_vector_buffer;
        Eigen::Vector3d gps_buffer_vector;
        Eigen::Vector3d gps_position_vector;
        
       // We update and correct the magnetometer
        mag._init_earth_even_magnetic_field();
        mag.update_correct();
        // We initialize the state vector giving the initial heading
        ekf.init_state_vector(mag.getHeading());
        //ekf.init_state_vector();
        to_rpy << TODEG*(ekf.toRPY(ekf.get_state_vector())).transpose() << std::endl;           // Storing operation

        
        
        while (!acc.line_end && !gyro.line_end){
            
            // Reading from inertial captors and correcting the outputs
            acc.getOutput(&acc_vector_buffer);
            acc.correctOutput(&acc_vector_buffer, ekf.getDCM());
            gyro.getOutput(&gyro_vector_buffer);
            gyro.correctOutput(&gyro_vector_buffer);
            // Updating and correcting magnetometer
            mag.update_correct();
            // Updating GPS and storing into the &gps_buffer_vector
            gps.update(&gps_buffer_vector);
            
            
        
            // Predicting the state vector
            // Extended Kalman Filter according step is the prediction step. We first build the Jacobian matrix linearized around the current state and we then multiply the currrent state vector by such a matrix
            ekf.build_jacobian_matrix(&acc_vector_buffer, &gyro_vector_buffer);
            ekf.predict();
            
            to_rpy << TODEG*(ekf.getRPY()).transpose() << std::endl;                            // Storing operation for data exploitation
            to_log << ekf.get_state_vector().transpose() << std::endl;                          // Storing operation for data exploitation
            //mag_out << TODEG*mag.getHeading() << std::endl;                                   // Storing operation for data exploitation
            mag_out << TODEG*mag.getHeading((ekf.getRPY())(1),(ekf.getRPY())(0)) << std::endl;  // Storing operation for data exploitation

            
    

            
            // Checking inf a new gps data is availaible
            if (gps.isAvailable()){
                // If yes, we first update the GPS filter
                gps.calculatePositionFromHome(&gps_buffer_vector);
                gps_out << gps.getActualPosition().transpose() << std::endl;
                gps.actualizeInternDatas(&gps_buffer_vector);
                gps_filter.predict();
                gps_filter.updateFilter();
                gps_filter_out << gps_filter.getState().transpose() << std::endl;               // Storing operation for data exploitation
                // And we then correct the EKF filter by correcting in order position, speed and quaternion attitude vector
                ekf.correct();
            }
            
        }

        

        
        

    }
    catch (std::exception const& e)
    { std::cout << e.what() <<  " file " << std::endl; }
    
    

    
    return 0;
}
