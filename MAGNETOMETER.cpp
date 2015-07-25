//
//  MAGNETOMETER.cpp
//  INS_EKF
//
//  Created by Louis Faury on 12/07/2015.
//  Copyright (c) 2015 Louis Faury. All rights reserved.
//

#include "MAGNETOMETER.h"


#define TODEG 57.295791
#define TORAD 0.017453



MAGNETOMETER::MAGNETOMETER(){
}


MAGNETOMETER::MAGNETOMETER(const std::string mag_file_name, STYLE style) : Captor(mag_file_name){
    std::cout << "Magnetometer file open" << std::endl;
    captor_id = "MAG";
    LINE_MARK = "$M";
    OFFSET_MARK = "$MO";
    
    if (style != COLD_START){
        // TODO
    }
}





double MAGNETOMETER::getHeading(const Eigen::Matrix3f &dcm_matrix){
    float headX;
    float headY;
    float cos_pitch = sqrt(1-(dcm_matrix(0)*dcm_matrix(0)));
    float heading;
    
    // sinf(pitch) = - dcm_matrix(3,1)
    // cosf(pitch)*sinf(roll) = - dcm_matrix(3,2)
    // cosf(pitch)*cosf(roll) = - dcm_matrix(3,3)
    
    if (cos_pitch == 0.0f) {
        // we are pointing straight up or down so don't update our
        // heading using the compass. Wait for the next iteration when
        // we hopefully will have valid values again.
        return 0;
    }
    
    // Tilt compensated magnetic field X component:
    headX = mag_x*cos_pitch - mag_y*dcm_matrix(1)*dcm_matrix(0)/cos_pitch - mag_z*dcm_matrix(2)*dcm_matrix(1)/cos_pitch;
    // Tilt compensated magnetic field Y component:
    headY = mag_y*dcm_matrix(2)/cos_pitch - mag_z*dcm_matrix(1)/cos_pitch;
    // magnetic heading
    // 6/4/11 - added constrain to keep bad values from ruining DCM Yaw - Jason S.
    heading = atan2(-headY,headX);
    
    return (double)heading;
}




double MAGNETOMETER::getHeading(float pitch, float roll) const{
    int heading_x = mag_x*cos(pitch)-mag_y*sin(roll)*sin(pitch)-mag_z*cos(roll)*sin(pitch);
    //int heading_x = mag_x*cos(PRY(1))-mag_y*sin(PRY(0))*sin(PRY(1))-mag_z*cos(PRY(0))*sin(PRY(1));
    int heading_y = -mag_y*cos(roll)+mag_z*sin(roll);
    //int heading_y = mag_y*cos(PRY(0))-mag_z*sin(PRY(0));
    
    return (double)(atan2(-heading_y, heading_x));
}


double MAGNETOMETER::getHeading() const{
    return atan2(mag_y,mag_x);
}



void MAGNETOMETER::update(){
    int count = 0;
    std::string::size_type sz;
    if (std::getline(captor_file,line)){
        line_end = false;
        std::stringstream ss(line);
        while (getline(ss,line,',')){
            if (line == LINE_MARK);
            else {
                switch (count) {
                    case 0:
                        mag_x = std::stof(line,&sz);
                        break;
                    case 1:
                        mag_y = std::stof(line,&sz);
                        break;
                    case 2:
                        mag_z = std::stof(line,&sz);
                        break;
                }
                count++;
            }
        }
        std::cout << captor_id << " : line read" << std::endl;
    }
    else{
        std::cout << "Couldn't read from captor : " << captor_id << std::endl;
        line_end = true;
    }

}


void MAGNETOMETER::correct(){
    mag_x += (int)captor_offsets(0);
    mag_y += (int)captor_offsets(1);
    mag_z += (int)captor_offsets(2);
}



void MAGNETOMETER::update_correct(){
    this->update();
    this->correct();
}


void MAGNETOMETER::printState() const{
    std::cout << mag_x << "\t" << mag_y << "\t" << mag_z << std::endl;
}


Eigen::Vector3i MAGNETOMETER::getState() const{
    Eigen::Vector3i result;
    result << mag_x, mag_y, mag_z;
    return result;
}




