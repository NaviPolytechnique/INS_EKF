//
//  EKF.h
//  INS_EKF
//
//  Created by Louis Faury on 13/07/2015.
//  Copyright (c) 2015 Louis Faury. All rights reserved.
//

#ifndef __INS_EKF__EKF__
#define __INS_EKF__EKF__

#include <stdio.h>
#include "/usr/include/Eigen/Dense"
#include <iostream>
#include <string>
#include <cmath>


class EKF{
    
    typedef Eigen::Matrix<float,10,1> Vector10f;
    typedef Eigen::Matrix<float, 10, 10> Matrix10f;
    
public:
    
    
    EKF();
    
   /* \brief Initialize the state vector
    * \brief WARNING : For now, initialize in the inertial frame
    * \brief Position, speed and quaternion
    */
    void init_state_vector();
    
    
    
   /* \brief Initalize the state_vector given an initial heading
    * \brief Assume the IMU is on an even surface, with only a non zero heading
    * \brief Allow the EKF to evolve in N/E frame
    * \param heading (int)
    */
    void init_state_vector(double heading);
    
    
    
   /* \brief Return the state vector
    */
    Vector10f get_state_vector();
    
    
   /* \brief Construct the jacobian matrix for the prediction step of the Kalman filter
    * \param Pointer to a Vector3f (buffer for the acc)
    * \param Pointer to a Vector3f (buffer for the gyro)
    */
    void build_jacobian_matrix(Eigen::Vector3f*, Eigen::Vector3f*);
    
    
   /* \brief Prediction step for the EKF
    */
    void predict(); //TODO
    
    
    
   /* TODO
    */
    void correct();
    
    
    
   /* \brief Converts the quaternion vector inside the state_vector into a Vector3f of PRY (Pitch, Roll, and Yaw) in RAD.
    * \param The state vector
    */
    Eigen::Vector3f toRPY(Vector10f vector);
    
    
    Eigen::Vector3f getRPY();
    
    
    
    
    
    
    
    
    ~EKF(); //TODO
    
    
    

    
    
    
    
    
private:
    
    Vector10f X; // The state vector
    Matrix10f J; // The jacobian Matrix for integration of inertial datas
    const double dt = 0.02; // Sampling time in seconds
    
    
    
};

#endif /* defined(__INS_EKF__EKF__) */
