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
#include "Captor.h"
#include "util.h"




class GPS{
    
public:
    
    // Enum for the mode the GPS would have automatically chosen given the available satelites. TO DO : Automatic detection of the style 
    enum style{
        NMEA,
        UBLOX
    };
    
    
    
    GPS();

    
   /* \brief Constructor of GPS object
    * \brief throw exception if file couldn't be opened
    * \param string of the gps file path
    */
    GPS(std::string,style) throw(std::exception);
    
    
    
   /* \brief Set the HOME of the INU by giving the first GPS value
    */
    int setHome();
    
    
    
   /* \brief Update the GPS position
    */
    int update(Eigen::Vector3d*);

    
    
    /* \brief Transform LLA data to local frame coordinates and stores it into actual_position
     * \brief To match with the IMU when we'll perform Kalman filtering, this function calculates distance on EAST NORTH and ALTITUDE axis from the HOME position
     * \brief We consider the following formulas for calculating such distances
     * \brief See source code for formulas
     * \param The gps output you want to transform - the pointer to buffer of the gps datas
     */
    void calculatePositionFromHome(Eigen::Vector3d*);
    
    
    
   /* \brief Checks if GPS received available datas
    */
    bool isAvailable();
    
    
   /* \brief Store in a Vector3d the last gps update and the current_time 
    * \param Pointer to the buffer vector for GPS position
    */
    void actualizeInternDatas(Eigen::Vector3d*);
    
    
    
    
   /* \brief Returns actual GPS Position from HOME as calculated in calculatePositionFromHome
    */
    Eigen::Vector3d getActualPosition();
    
    
    friend class GPS_Filter;
    
private:
    
    const std::string HOME_MARK = "$GPSI";
    const std::string LINE_MARK = "$GPS";
    std::ifstream gps_file;
    Eigen::Vector3d HOME;
    Eigen::Vector3d actual_position;
    Eigen::Vector3d last_update;
    double current_time;
    double time_since_last_update;
    double time_of_last_update;
    double time_init;
    Eigen::Vector3d ground_speed;
    std::string line;
    bool available;
    int time_scale; //Since the GPS can have to style of fonctionnement with two different time_scale
    
    
    
    
};

#endif /* defined(__INS_EKF__GPS__) */
