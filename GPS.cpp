//
//  GPS.cpp
//  INS_EKF
//
//  Created by Louis Faury on 12/07/2015.
//  Copyright (c) 2015 Louis Faury. All rights reserved.
//

#include "GPS.h"


#define PI 3,141592
#define TODEG 57.295791
#define RT 6371000




GPS::GPS(){
}



GPS::GPS(std::string gps_file_path, style style) throw(std::exception) : gps_file(gps_file_path.c_str()) {
    if (style == NMEA) time_scale = 100;
    if (style == UBLOX) time_scale = 1000;
    if (!gps_file) throw std::domain_error("Couldn't open GPS file !");
    std::cout << "GPS file open" << std::endl;
}



int GPS::setHome(){
    int count = 0;
    std::string::size_type sz;
    if (std::getline(gps_file,line)){
        std::stringstream ss(line);
        while (getline(ss,line,',')){
                if (line == HOME_MARK);
                else {
                    if (count < 3) {
                        HOME[count] = (std::stod(line,&sz));
                        count++;
                    }
                    else time_init = std::stod(line,&sz)/time_scale;
                }
            }
            //std::cout << HOME << std::endl;
            last_update << 0,0,0;
            time_of_last_update = time_init;
            return 1;
        }
    return -1;
}



int GPS::update(Eigen::Vector3d* gps_buffer){
    int count = 0;
    std::string::size_type sz;
    if (std::getline(gps_file,line)){
        std::stringstream ss(line);
        while (getline(ss,line,',') && count<4){
            if (line == LINE_MARK);
            else {
                if (line == "NA"){
                    available = false;
                    return -1;
                }
                else {
                    if (count < 3){
                        (*gps_buffer)[count] = std::stod(line,&sz);
                        count++;
                    }
                    else current_time = std::stod(line,&sz)/time_scale;
                    available = true;
                }
            }
        }
        std::cout << "GPS : line read" << std::endl;
        return 1;
    }
    std::cout << "Couldn't read from GPS" << std::endl;
    return 0;
}



void GPS::toCartesian1(Eigen::Vector3d &source){
    source(0) = RT*source(1)*cos(source(0));
    source(1) = RT*source(0)*cos(source(1));
}



Eigen::Vector3d GPS::toCartesian2(Eigen::Vector3d source){
    Eigen::Vector3d result;
    double x = RT*(source(1)/TODEG)*cos(source(0)/TODEG);
    double y = RT*(source(0)/TODEG)*cos(source(1)/TODEG);
    result << x,y,source(2);
    return result;
}



Eigen::Vector3d GPS::getPositionFromHome(Eigen::Vector3d* source){
    Eigen::Vector3d HOME_C = toCartesian2(HOME);
    Eigen::Vector3d ACTUAL = toCartesian2(*source);
    return ACTUAL-HOME_C;
}



void GPS::calculatePositionFromHome(Eigen::Vector3d* source){
    Eigen::Vector3d HOME_C = toCartesian2(HOME);
    Eigen::Vector3d ACTUAL = toCartesian2(*source);
    actual_position = ACTUAL-HOME_C;
}



bool GPS::isAvailable(){
    return available;
}



void GPS::actualizeInternDatas(Eigen::Vector3d* gps_buffer_vector){
    time_since_last_update = current_time - time_of_last_update;
    if (time_since_last_update != 0){
    time_of_last_update = current_time;
        //std::cout << time_since_last_update << std::endl;
        for (int i=0; i<3; i++){
        ground_speed(i) = (actual_position(i)-last_update(i))/time_since_last_update;
        }
        //std::cout << ground_speed.transpose() << std::endl;
        last_update = actual_position;
    }
    else {
        actual_position = last_update;
        ground_speed *= 0;
    }
    
}




















