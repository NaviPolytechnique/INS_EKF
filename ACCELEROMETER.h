//
//  ACCELEROMETER.h
//  INS_EKF
//
//  Created by Louis Faury on 12/07/2015.
//  Copyright (c) 2015 Louis Faury. All rights reserved.
//

#ifndef __INS_EKF__ACCELEROMETER__
#define __INS_EKF__ACCELEROMETER__

#include <stdio.h>
#include "Captor.h"
#include "util.h"




class ACCELEROMETER : public Captor{
 
    
public:
    
    ACCELEROMETER();
    
    
    ACCELEROMETER(const std::string, STYLE);
    
    
   /*\brief Init accelerometer offsets
    * /!\ SUPPOSING THE IMU IS PLACED ON EVEN SURFACE
    * \brief Calls Captors::initOffsets() method and add g vector for the Z-axis offset
    */
    int initOffsets();
    

   /* \brief Corrects acc output 
    * \brief Calls the Captors::correctOutput() method and add to the buffer vector the rotated g acceleration of gravity vector
    * \param The buffer of the acc_output and the rotation matrix of the EKF
    */
    void correctOutput(Eigen::Vector3f*,Eigen::Matrix3f);
    

};

#endif /* defined(__INS_EKF__ACCELEROMETER__) */
