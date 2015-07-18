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



GPS::GPS(std::string gps_file_path) throw(std::exception) : gps_file(gps_file_path.c_str()) {
    
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
                    HOME[count] = (std::stof(line,&sz));
                    count++;
                }
            }
            //std::cout << HOME << std::endl;
            return 1;
        }
    return -1;
}



int GPS::update(Eigen::Vector3f* gps_buffer){
    int count = 0;
    std::string::size_type sz;
    if (std::getline(gps_file,line)){
        std::stringstream ss(line);
        while (getline(ss,line,',')){
            if (line == LINE_MARK);
            else {
                (*gps_buffer)[count] = std::stof(line,&sz);
                count++;
            }
        }
        return 1;
    }
    std::cout << "Couldn't read from GPS" << std::endl;
    return 0;
}




