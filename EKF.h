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
#include "/usr/local/include/Eigen/Dense"
#include <iostream>
#include <string>
#include <cmath>
#include "GPS_Filter.h"
#include "ACCELEROMETER.h"
#include "GYRO.h"
#include "MAGNETOMETER.h"


class EKF{

    //The state vector :  {pos_vector      NORTH,EAST,ALTITUDE
    //                     speed_vector    NORTH,EAST,ATLITUDE
    //                     quaternion_vector,
    //                     acc_bias_random_walk,
    //                     gyro_bias_random_walk,
    //                     acc_white_gaussian_noise,
    //                     gyro_white_gaussian_noise}
#include "Drone.h"
#include "Runnable.h"
#include "Listener.h"
#include "BlockingQueue.h"


extern char* IMUport;



class EKF : public Runnable{
>>>>>>> adaptation-au-drone
    
    
public:
    
    
    EKF(Drone* drone_);
    
    void start();
    
    void* run();
    
    
   /* Constructor of the class. 
    * \param A pointer to a GPS_Filter Object - used for actualizing the general filter
    */
    EKF(GPS_Filter*,ACCELEROMETER*,GYRO*,MAGNETOMETER*);
    
    
    
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
    Vector16f get_state_vector() const;
    
    
   /* \brief Construct the jacobian matrix for the prediction step of the Kalman filter
    * \param Pointer to a Vector3f (buffer for the acc)
    * \param Pointer to a Vector3f (buffer for the gyro)
    */
    void build_jacobian_matrix(Eigen::Vector3f*, Eigen::Vector3f*);
    
    
    
<<<<<<< HEAD
=======
    void interpret(char* msg);
    
    Eigen::Vector3f toPRY(Vector10f vector);
>>>>>>> adaptation-au-drone
    
   /* \brief Prediction step for the EKF
    */
    void predict(); 
    
    
    
   /* \brief Correcting step for the EKF
    */
    void correct();
    
    
    
   /* DEPRECATED
    * \brief Converts the quaternion vector inside the state_vector into a Vector3f of PRY (Pitch, Roll, and Yaw) in RAD.
    * \param The state vector
    */
    Eigen::Vector3f toRPY(Vector16f vector);
    
    
    
   /* \brief Converts the quaternion vector inside the state_vector into a Vector3f of PRY (Pitch, Roll, and Yaw) in RAD and returns it in form of a Vector3f.
    * \param none
    */
    Eigen::Vector3f getRPY() const;
    
    
    
   /* \brief Return current position from the Extended Kalman Filter
    */
    Eigen::Vector3f getCurrentPos() const;
    
    
    
   /* \brief Return current speed from the Extended Kalman Filter
    */
    Eigen::Vector3f getCurrentSpeed() const;
    
    
   /* \brief Returns as a Vector6f the Offsets calculated by the Kalman filter
    */
    Vector6f getActualInertialOffsets() const;
    
    
   /* Update the measure vector Z from the gps_filter datas 
    */
    void updateMeasure();
    
    
    
   /* Computes the DCM matrix from the quaternion vector
    */
    void calculateDCM();
    
    
   /* \brief Returns actual rotation matrix from inertial body to earth body frame
    */
    Eigen::Matrix3f getDCM();

    
    ~EKF();
    
    

    
    
    

    Drone* drone;
    Vector10f X; // The state vector
    Matrix10f J; // The jacobian Matrix for integration of inertial datas
    const double dt = 0.02; // Samplimp time in seconds
    
    //Pour la communication avec l'APM
    Listener* IMUlistener;
    BlockingQueue<char *>* received;
    int working;

    
    
private:
    
    GPS_Filter* gps_filter;
    ACCELEROMETER* acc;
    GYRO* gyro;
    MAGNETOMETER* mag;
    Vector16f _X; // For X-
    Vector16f X_; // For X+
    Vector9f Z; // Measurement vector : Position, Speed and Magnetic Field
    Matrix16f J; // The jacobian Matrix for integration of inertial datas // In Extended Kalman Filter, would be state matrix.
    double dt; // Sampling time in seconds
    Matrix16f P_;  // P(k)+
    Matrix16f _P;  // P(k)-
    Matrix16f R;   // Covariance matrix for the measure noises
    Matrix16f Q;   // Covariance matrix for the model noises
    Eigen::Matrix<float,16,3> K;
    Eigen::Matrix<float,3,16> H_GPS_POS;
    Eigen::Matrix<float,3,16> H_GPS_SPEED;
    Eigen::Matrix<float,3,16> H_MAG;
    Eigen::Matrix3f _C; // DCM Matrix from inertial body to earth body
    
    
};

#endif /* defined(__INS_EKF__EKF__) */
