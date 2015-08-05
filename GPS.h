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
    
    
   /* \brief Transform a lattitude longitude attitude data into local frame coordinates
    * \param The LLA to transform
    */
    void toCartesian1(Eigen::Vector3d &source);
    
    
    /* \brief Returns from a lattitude longitude attitude data earth frame coordinates vector
     * \param The LLA to transform
     */
    Eigen::Vector3d toCartesian2(Eigen::Vector3d source);

    
    
    /* \brief Returns from a lattitude longitude attitude data local frame coordinates vector
     * \param The gps output you want to transform
     */
    Eigen::Vector3d getPositionFromHome(Eigen::Vector3d*);
    
    
    
    /* \brief Calculate from a lattitude longitude attitude data  to local frame coordinates a vector  abd stores it into position_vector
     * \param The gps output you want to transform
     */
    void calculatePositionFromHome(Eigen::Vector3d*);
    
    
    
   /* \brief Checks if GPS received available datas
    */
    bool isAvailable();
    
    
   /* \brief Store in a Vector3d the last gps update and the current_time 
    * \param The buffer vector for GPS position
    */
    void actualizeInternDatas(Eigen::Vector3d*);
    
 
    
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
