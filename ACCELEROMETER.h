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
    * WARNING : SUPPOSING THE IMU IS PLACED ON EVEN SURFACE
    * \brief Calls Captors::initOffsets() method and add g vector for the Z-axis offset
    */
    int initOffsets();
    

   /* \brief Corrects acc output 
    * \brief Calls the Captors::correctOutput() method and add to the buffer vector the rotated g acceleration of gravity vector
    * \param The buffer of the acc_output and the rotation matrix of the EKF
    */
    void correctOutput(Eigen::Vector3f*,Eigen::Matrix3f);
    
    
    
    /* \brief General method for the Gauss-Newton calibration of the accelerometer
     * \brief This method (and the followings) is inspired from Rolfe Schmidt's website and work :
     * \brief http://chionophilous.wordpress.com/2011/10/24/accelerometer-calibration-iv-1-implementing-gauss-newton-on-an-atmega/
     * \brief The original script was found here :
     * \brief http://rolfeschmidt.com/mathtools/skimetrics/adxl_gn_calibration.pde
     * \brief Returns 1 if succeeded.
     * \brief Returns -1 if couldn't capture the calibration routines acceleration datas
     * \brief Returns 0 if the method couln't converge in less than 10 iterations 
     */
    int performAdvancedAccCalibration();
    
    
    
   /* \brief Is called in order to capture the acc values when performing the GN algorithm for acc advanced calibration
    * \brief Part of acc advanced calibration routine
    * \brief Returns 1 if succeeded
    * \brief Returns 0 if line wasn't part of the good routine and -1 if there is no line to read in the acc data file. 
    */
    int captureAdvancedCalibrationRoutine();
    
    
    
    /* \brief Initialize the Gauss-Newton algorithm
     * \brief Part of acc advanced calibration routine
     */
    void initGNMethod();
    
    
    
    /* \brief Calculate and returns the sum square of the residual
     * Part of acc advanced calibration routine
     */
    float calculateSumSquareDiff();
    
    
    
    
    /* \brief Update the matrix of the GN algorithm
     * Part of acc advanced calibration routine
     */
    void updateGNAlgorithm();
    
    
    
    
    /* \brief Find and returns the direction in which the state vector should be moved to minimize the square distance sum.
     * \brief Part of acc advanced calibration routine
     */
    Vector6f findIncrement();
    
    
    
    
    /* \brief Correct the _beta vector
     * \brief Part of acc advanced calibration routine
     */
    void correct_beta();
    
    

    
    

    
    
private:
    
    std::string ADVANCED_OFFSET_MARK;
    
#ifdef ADCALIBRATION
    Eigen::Matrix<float,6,3> _ad_acc_data;  // Matrix used to store the advanced acc calibration method readings
    Vector6f _r_vector;                     // Vector of the residuals for the Gauss-Newton method
    Vector6f _beta;                         // Vector of offsets and gains
                                            // In order : offset_x,offset_y,offset_z,gain_x,gain_y,gain_Z
    Matrix6f _jacobian;                     // The jacobian matrix for the Gauss-Newton method
#endif

};

#endif /* defined(__INS_EKF__ACCELEROMETER__) */
