//
//  MAGNETOMETER.cpp
//  INS_EKF
//
//  Created by Louis Faury on 12/07/2015.
//  Copyright (c) 2015 Louis Faury. All rights reserved.
//

#include "MAGNETOMETER.h"


#define TODEG 57.295791



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



int MAGNETOMETER::getHeading(Eigen::Vector3f PRY) const{
    // heading_x = magx*cos(pitch)-magy*sin(roll)cos(pitch)-magz*cos(roll)sin(pitch)
    int heading_x = mag_x*cos(PRY(1))-mag_y*sin(PRY(0))*sin(PRY(1))-mag_z*cos(PRY(0))*sin(PRY(1));
    // heading_y = magy*cos(roll)-magz*sin(roll)
    int heading_y = mag_y*cos(PRY(0))-mag_z*sin(PRY(0));
    
    return (int)(TODEG*atan2(heading_y, heading_x));
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


void MAGNETOMETER::printState(){
    std::cout << mag_x << "\t" << mag_y << "\t" << mag_z << std::endl;
}




