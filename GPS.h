//
//  GPS.h
//  INS_EKF
//
//  Created by Louis Faury on 12/07/2015.
//  Copyright (c) 2015 Louis Faury. All rights reserved.
//

#ifndef __INS_EKF__GPS__
#define __INS_EKF__GPS__

#include <stdio.h>
#include <string>
#include <iostream>
#include <fstream>
#include "/usr/include/Eigen/Dense"
#include <exception>
#include <sstream>


class GPS{
    
public:
    
    GPS();
    
    
   /* \brief Constructor of GPS object
    * \brief throw exception if file couldn't be opened
    * \param string of the gps file path
    */
    GPS(std::string) throw(std::exception);
    
    
    
   /* \brief Set the HOME of the INU by giving the first GPS value
    */
    int setHome();
    
    
    
   /* \brief Update the GPS position
    */
    int update(Eigen::Vector3f*);
    
    
    
    
private:
    
    const std::string HOME_MARK = "$GPSI";
    const std::string LINE_MARK = "$GPS";
    std::ifstream gps_file;
    Eigen::Vector3f HOME;
    std::string line;
    
    
    
    
};

#endif /* defined(__INS_EKF__GPS__) */
