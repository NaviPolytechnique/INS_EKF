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
#include "util.h"

class MAGNETOMETER : public Captor{
       

public:
    
    MAGNETOMETER();
    
    
    MAGNETOMETER(const std::string, STYLE);
    
    
    
   /* \brief Returns the inertial unit heading in RAD
    * \param Roll and Pitch angle in RAD
    */
    double getHeading(float,float) const;
    
    
    
    /* \brief Returns the inertial unit heading assuming that the IMU in on even surface
     * \brief In RAD
     */
    double getHeading() const;
    
    

   /* \brief Update magnetometer
    */
    void update();
    
    
    /* \brief Initialize _earth_magnetic_field_even
     */
    void _init_earth_even_magnetic_field();
    
    
    /* \brief Returns _earth_even_magnetic_field as a Vector3d
     */
    Eigen::Vector3d getEEMF();
    
    
    /* \brief Correct magnetometer (offsets)
     */
    void correct();
    
    
    /* \brief Update and correct magnetometer (offsets)
     */
    void update_correct();
    
    
   /* \brief Print the magnetometer state (magx,magy,magz)
    */
    void printState() const;
    

   /* \brief Returns the state (mag_x,mag_y,mag_z)
    */
    Eigen::Vector3d getState() const;
    
    
    
   /* \brief Returns the magnetometer noises' covariance matrix 
    * which is initalised when constructing MAGNETOMETER mag.
    */
    Eigen::Matrix3f getMagCovarianceMatrix();
    
    
private:
    
    double mag_x, mag_y, mag_z;
    double _declination;    // Declination angle with the North in RAD.
                            // Need to be specified in preprocessor and modified in the initialiser
    Eigen::Vector3d _earth_even_magnetic_field;     // The magnetic field measured by the magnetometer when on even surface
                                                    // Is initialized when the first line is read

    Eigen::Matrix3f R; // Noise covariance matrix
};

#endif /* defined(__INS_EKF__MAGNETOMETER__) */
