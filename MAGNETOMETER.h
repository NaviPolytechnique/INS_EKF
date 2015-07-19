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
    
    
    
    
    
    
   /* \brief Returns the inertial unit heading
    * \brief Deprecated, we will eventually be using DCM Matrix method. 
    * \param A 3 columns vector (actually we will be using only roll & pitch but to make it easier we take the whole PRY vector as argument)
    */
    int getHeading(Eigen::Vector3f PRY) const;
    
    
    
    
    
   /* TODO
    */
   // int getHeading(Eigen::Matrix3f DCM);
    
    
    
    void update();
    
    
    
    void correct();
    
    
    
    void update_correct();
    
    
    void printState();
    
    
private:
    
    int mag_x, mag_y, mag_z;

};

#endif /* defined(__INS_EKF__MAGNETOMETER__) */
