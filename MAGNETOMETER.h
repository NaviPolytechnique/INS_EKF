//
//  MAGNETOMETER.h
//  INS_EKF
//
//  Created by Louis Faury on 12/07/2015.
//  Copyright (c) 2015 Louis Faury. All rights reserved.
//

#ifndef __INS_EKF__MAGNETOMETER__
#define __INS_EKF__MAGNETOMETER__

#include <stdio.h>
#include "Captor.h"

class MAGNETOMETER : public Captor{
   
public:
    
    MAGNETOMETER();
    
    MAGNETOMETER(const std::string, STYLE);
    
    
   /* \brief Returns the inertial unit heading in RAD
    * \brief Deprecated, we will eventually be using DCM Matrix method. 
    * \param Ptich and roll
    */
    double getHeading(float,float) const;
    
    
    
    
    /* \brief Returns the inertial unit heading assuming that the IMU in on even surface
     * \brief In RAD
     */
    double getHeading() const;
    
    
    
    
    /* TODO
     */
    double getHeading(const Eigen::Matrix3f &dcm_matrix);
    
    

   /* \brief Update magnetometer
    */
    void update();
    
    
    /* \brief Correct magnetometer (offsets)
     */
    void correct();
    
    
    /* \brief Update and correct magnetometer (offsets)
     */
    void update_correct();
    
    
    
    void printState() const;
    
    
    Eigen::Vector3i getState() const;
    
    
private:
    
    int mag_x, mag_y, mag_z;

};

#endif /* defined(__INS_EKF__MAGNETOMETER__) */
