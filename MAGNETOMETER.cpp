//
//  MAGNETOMETER.cpp
//  INS_EKF
//
//  Created by Louis Faury on 12/07/2015.
//  Copyright (c) 2015 Louis Faury. All rights reserved.
//

#include "MAGNETOMETER.h"




MAGNETOMETER::MAGNETOMETER(){
}


MAGNETOMETER::MAGNETOMETER(const std::string mag_file_name, STYLE style) : Captor(mag_file_name){
    std::cout << "Magnetometer file open" << std::endl;
    captor_id = "MAG";
    LINE_MARK = "$M";
    OFFSET_MARK = "$MO";
    _declination = SPQ_DECLINATION_RAD; // Setting the declination angle for the tests in Qro,Mexico
    R.setZero();
    R(0,0) = 0.0009;
    R(1,1) = 0.0004;
    R(2,2) = 0.02;
    if (style != COLD_START){
        // TODO
    }
}


double MAGNETOMETER::getHeading(float pitch, float roll) const{
    double heading_x = mag_x*cos(pitch)-mag_y*sin(roll)*sin(pitch)-mag_z*cos(roll)*sin(pitch);
    //int heading_x = mag_x*cos(PRY(1))-mag_y*sin(PRY(0))*sin(PRY(1))-mag_z*cos(PRY(0))*sin(PRY(1));
    double heading_y = -mag_y*cos(roll)+mag_z*sin(roll);
    //int heading_y = mag_y*cos(PRY(0))-mag_z*sin(PRY(0));
    
    double heading = (double) atan2(-heading_y, heading_x);
     if( _declination > 0.0f )
     {
     heading = heading + _declination;
     if (heading > PI)    // We normalize the angle, so that it is between -PI and +PI
     heading -= (2.0f * PI);
     else if (heading < -PI)
     heading += (2.0f * PI);
     }
     
     return heading;
    
}



double MAGNETOMETER::getHeading() const{
    double heading = (double) atan2(mag_y, mag_x);
    if( _declination > 0.0f )
    {
        heading = heading + _declination;
        if (heading > PI)    // We normalize the angle, so that it is between -PI and +PI
            heading -= (2.0f * PI);
        else if (heading < -PI)
            heading += (2.0f * PI);
    }
    
    return heading;
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
                        mag_x = std::stod(line,&sz);
                        break;
                    case 1:
                        mag_y = std::stod(line,&sz);
                        break;
                    case 2:
                        mag_z = std::stod(line,&sz);
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



void MAGNETOMETER::_init_earth_even_magnetic_field(){
    this->update_correct();
    _earth_even_magnetic_field << mag_x,mag_y,mag_z;
    //std::cout << _earth_even_magnetic_field.transpose() << std::endl;
}


Eigen::Vector3d MAGNETOMETER::getEEMF(){
    return _earth_even_magnetic_field;
}


void MAGNETOMETER::correct(){
    mag_x = (mag_x+captor_offsets(0))/HMC5843L_GAIN;
    mag_y = (mag_y+captor_offsets(1))/HMC5843L_GAIN;
    mag_z = (mag_z+captor_offsets(2))/HMC5843L_GAIN;
}



void MAGNETOMETER::update_correct(){
    this->update();
    this->correct();
}



void MAGNETOMETER::printState() const{
    std::cout << mag_x << "\t" << mag_y << "\t" << mag_z << std::endl;
}


Eigen::Vector3d MAGNETOMETER::getState() const{
    Eigen::Vector3d result;
    result << mag_x, mag_y, mag_z;
    return result;
}



Eigen::Matrix3f MAGNETOMETER::getMagCovarianceMatrix(){
    return R;
}




