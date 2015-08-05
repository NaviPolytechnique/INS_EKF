//
//  GPS_Filter.h
//  INS_EKF
//
//  Created by Louis Faury on 25/07/2015.
//  Copyright (c) 2015 Louis Faury. All rights reserved.
//

#ifndef __INS_EKF__GPS_Filter__
#define __INS_EKF__GPS_Filter__

#include <stdio.h>
#include <string>
#include "/usr/include/Eigen/Dense"
#include <iostream>
#include "GPS.h"

typedef Eigen::Matrix<float,6,6> Matrix6f;
typedef Eigen::Matrix<float, 6, 1> Vector6f;
typedef Eigen::Matrix<double,6,10> Matrix6_10d;

class GPS_Filter{
    
public:
    
    GPS_Filter();
    
    
   /* \brief Constructor of the class
    * \param Pointer to the current GPS object
    */
    GPS_Filter(GPS*);
    
    
    
   /* \brief Used for the predict step of the Kalman filter
    * \brief Calculate the state matrix at incrementation time k
    */
    void calculateTransitionMatrix();
    
    
    
   /* \brief Used for the predict step of the Kalman filter
    * \brief Main method for the predict step. 
    */
    void predict();
    
    
    
   /* \brief Used for the update step of the Kalman filter
    * \brief Calculate Z vector from the gps datas
    */
    void updateMesure();
    
    
   /* \brief Used for the update step of the Kalman filter
    * \brief Main method for the update step
    */
    void updateFilter();
    
    
   /* \brief Used for the update step of the Kalman filter
    * \brief Implement R matrix adaptative evolution 
    * \brief : Taken from Ali Almagbile, Jinling Wang, and Weidong Ding paper : Evaluating the Performances of Adaptive Kalman Filter Methods in GPS/INS Integration
    */
    void adaptRMatrix();
    
    
   /* \brief  Returns the actual position field in the forme of a Vector3f
    */
    Eigen::Vector3f getActualPosition() const;
    
    
    /* \brief  Returns the actual state vector in the form of a Vector6f
     */
    Vector6f getState() const;
    
    
   /* \brief Returns the current speed in a Vector3f
    */
    Eigen::Vector3f getActualSpeed();
    
    
    /* \brief Returns the current position in a Vector3f
     */
    Eigen::Vector3f getActualPosition();
    
    
   /* \brief Return the speed part of P_ covariance matrix
    */
    Eigen::Matrix3f getSpeedCovMatrix();
    
    
    /* \brief Return the pos part of P_ covariance matrix
     */
    Eigen::Matrix3f getPositionCovMatrix();
    
    
    ~GPS_Filter();
    
    
    
private:
    
    Matrix6f P_;  // P(k)+
    Matrix6f _P;  // P(k)-
    Matrix6f R;
    Matrix6f Q;
    Matrix6f H;
    Matrix6f F;
    Matrix6f K;
    Vector6f _X;
    Vector6f X_; // WARNING : the GPS state vector is state_vector=(ground_speed,position)
    Vector6f Z;
    Matrix6f I6;
    float dt;
    GPS* gps;
    Eigen::Vector3f actual_position;
    Matrix6f C; // For R adaptative method
    Matrix6_10d mu;  // For R adpatative method
    

};






#endif /* defined(__INS_EKF__GPS_Filter__) */
