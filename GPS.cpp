//
//  GPS.cpp
//  INS_EKF
//
//  Created by Louis Faury on 12/07/2015.
//  Copyright (c) 2015 Louis Faury. All rights reserved.
//

#include "GPS.h"




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



void GPS::calculatePositionFromHome(Eigen::Vector3d *source){
    // We first calculate the true distance from the earth's center
    // in meter
    double _distance_from_earth_center = RT + (*source)(2);
    // We then calculate the distance we are from HOME on the EAST axis
    // x = (RT+h)*(actual_long - HOME_long)*cos(actual_lat)
    // in meter
    double x = _distance_from_earth_center*(((*source)(1)-HOME(1))/TODEG)*cos((*source)(0)/TODEG);
    // We then calculate the distance we are from HOME on the NORTH axis
    // y = (RT+h)*(actual_lat - HOME_lat)
    // in meter
    double y = _distance_from_earth_center*((*source)(0)-HOME(0))/TODEG;
    // We then calculate the distance we are from HOME regarding altitude in meter
    // z = actual_altitude - HOME_altitude
    // in meter
    double z = (*source)(2)-HOME(2);
    // We eventually update the actual_position as NORTH / EAST / ALTITUDE to match with IMU 
    actual_position << y,x,z;
}



Eigen::Vector3d GPS::getActualPosition(){
    return actual_position;
}



bool GPS::isAvailable(){
    return available;
}



void GPS::actualizeInternDatas(Eigen::Vector3d* gps_buffer_vector){
    // We update the time since the last update
    time_since_last_update = current_time - time_of_last_update;
    
    // Prevent the anomaly that happens when functionning in NMEA : multiple array with same GPS time are given out by GPS
    // Thus we test if the time since the last update in non-zero
    // either way the speed will not be valid.
    if (time_since_last_update != 0){
        time_of_last_update = current_time;
        for (int i=0; i<3; i++){
        ground_speed(i) = (actual_position(i)-last_update(i))/time_since_last_update;   // Actualisation of the ground_speed
                                                                                        // v = (actual_pos - last_pos) / dt
        }
        last_update = actual_position;                                                  // Update last_update
                                                                                        // now that we've actualized the datas
    }
    else {
        actual_position = last_update;  // This is in case of NMEA mode as it appears it prints multiple data for same timing
        ground_speed *= 0;              // We assume speed is then 0 and position stays the same
    }
    
}




















