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
   
    
#define PI 3.14159265
#define QRO_DECLINATION_RAD 0.094537
#define HMC5843L_GAIN 660;
    

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
    
    
private:
    
    double mag_x, mag_y, mag_z;
    double _declination;    // Declination angle with the North in RAD.
                            // Need to be specified in preprocessor and modified in the initialiser
    Eigen::Vector3d _earth_even_magnetic_field;     // The magnetic field measured by the magnetometer when on even surface
                                                    // Is initialized when the first line is read

};

#endif /* defined(__INS_EKF__MAGNETOMETER__) */
