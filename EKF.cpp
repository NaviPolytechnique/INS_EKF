//
//  EKF.cpp
//  INS_EKF
//
//  Created by Louis Faury on 13/07/2015.
//  Copyright (c) 2015 Louis Faury. All rights reserved.
//

#include "EKF.h"





EKF::EKF(){
}



EKF::EKF(GPS_Filter* gps_filter, ACCELEROMETER* acc, GYRO* gyro, MAGNETOMETER* mag) : gps_filter(gps_filter), acc(acc), gyro(gyro), mag(mag) {

    // Initialisation of _P
    _P.setZero();
    // Initialisation of P_
    P_.setZero();
    for (int i=0; i<6; ++i){
        P_(i,i) = 0.5;
    }
    // Initialisation of Q // TO DO
    Q.setZero();
    Q(10,10) = 0.0023;
    Q(11,11) = 0.0017;
    Q(12,12) = 0.0027;
    Q(13,13) = 0.0001;
    Q(14,14) = 0.0001;
    Q(15,15) = 0.0001;
    // Initialisation of H_GPS_POS
    H_GPS_POS.setZero();
    for (int i=0; i<3; ++i){
        H_GPS_POS(i,i) = 1;
    }
    // Initialisation of H_GPS_SPEED
    H_GPS_SPEED.setZero();
    for (int i=3; i<6; ++i){
        H_GPS_SPEED(i-3,i) = 1;
    }
    // Initialisation of H_MAG
    H_MAG.setZero();
    // Initialisation of K
    K.setZero();
    // Initialisation of Z
    Z.setZero();
}



void EKF::init_state_vector(){
    X_.setZero();
    X_(6) = 1;
   // std::cout << X.transpose() << std::endl;
    for (int i=0; i<3; ++i){
        X_(10+i) = (acc->getOffsets())(i);
    }
    for (int i=0; i<3; ++i){
        X_(12+i) = (gyro->getOffsets())(i);
    }
}



void EKF::init_state_vector(double heading){
    // We first calculate the quaternion linked with the heading
    float q0 = cos(heading/2);
    float q1 = 0;
    float q2 = 0;
    float q3 = sin(heading/2);
    X_ << 0,0,0,0,0,0,q0,q1,q2,q3,0,0,0,0,0,0;
    // Then we replace the 0 values by the actual offsets values
    for (int i=0; i<3; ++i){
        X_(10+i) = (acc->getOffsets())(i);
    }
    for (int i=0; i<3; ++i){
        X_(13+i) = (gyro->getOffsets())(i);
    }
}



Vector16f EKF::get_state_vector(){
    return X_;
}



void EKF::build_jacobian_matrix(Eigen::Vector3f* acc, Eigen::Vector3f* gyro){
    
    // Position
    J.setZero();
    
    for (int i=0; i<3; i++){
        J(i,i+3) = dt; // Position = dt*speed
    }
    
    // Vitesse
    J(3,6)  =   2*dt*(X_(6)*(*acc)(0)-X_(9)*(*acc)(1)+X_(8)*(*acc)(2));
    J(3,7)  =   2*dt*(X_(7)*(*acc)(0)+X_(8)*(*acc)(1)+X_(9)*(*acc)(2));
    J(3,8)  =  2*dt*(-X_(8)*(*acc)(0)+X_(7)*(*acc)(1)+X_(6)*(*acc)(2));
    J(3,9)  =  2*dt*(-X_(9)*(*acc)(0)-X_(6)*(*acc)(1)+X_(7)*(*acc)(2));
    J(4,6)  =   2*dt*(X_(9)*(*acc)(0)+X_(6)*(*acc)(1)-X_(7)*(*acc)(2));
    J(4,7)  =   2*dt*(X_(8)*(*acc)(0)-X_(6)*(*acc)(1)-X_(6)*(*acc)(2));
    J(4,8)  =   2*dt*(X_(7)*(*acc)(0)+X_(8)*(*acc)(1)+X_(9)*(*acc)(2));
    J(4,9)  =   2*dt*(X_(6)*(*acc)(0)-X_(9)*(*acc)(1)+X_(8)*(*acc)(2));
    J(5,6)  =  2*dt*(-X_(8)*(*acc)(0)+X_(7)*(*acc)(1)+X_(6)*(*acc)(2));
    J(5,7)  =   2*dt*(X_(9)*(*acc)(0)+X_(6)*(*acc)(1)-X_(7)*(*acc)(2));
    J(5,8)  =  2*dt*(-X_(6)*(*acc)(0)+X_(9)*(*acc)(1)-X_(8)*(*acc)(2));
    J(5,9)  =   2*dt*(X_(7)*(*acc)(0)+X_(8)*(*acc)(1)+X_(9)*(*acc)(2));
    
    
    // Quaternion
    J(6,6)   =                   0;
    J(6,7)   =    -dt*(*gyro)(0)/2;
    J(6,8)   =    -dt*(*gyro)(1)/2;
    J(6,9)   =    -dt*(*gyro)(2)/2;
    J(7,6)   =     dt*(*gyro)(0)/2;
    J(7,7)   =                   0;
    J(7,8)   =     dt*(*gyro)(2)/2;
    J(7,9)   =    -dt*(*gyro)(1)/2;
    J(8,6)   =     dt*(*gyro)(1)/2;
    J(8,7)   =    -dt*(*gyro)(2)/2;
    J(8,8)   =                   0;
    J(8,9)   =     dt*(*gyro)(0)/2;
    J(9,6)   =     dt*(*gyro)(2)/2;
    J(9,7)   =     dt*(*gyro)(1)/2;
    J(9,8)   =    -dt*(*gyro)(0)/2;
    J(9,9)   =                   0;
    
    

    for (int i=0; i<16; i++){
        J(i,i) += 1;
    }
    
}



Eigen::Vector3f EKF::toRPY(Vector16f vector){
    float roll  = atan2(2*(vector(6)*vector(7)+vector(8)*vector(9)),1-2*(vector(7)*vector(7)+vector(8)*vector(8)));
    float pitch =                                                asin(2*(vector(6)*vector(8)-vector(9)*vector(7)));
    float yaw   = atan2(2*(vector(6)*vector(9)+vector(7)*vector(8)),1-2*(vector(8)*vector(8)+vector(9)*vector(9)));
    
    Eigen::Vector3f RPY(roll,pitch,yaw);
    return RPY;
}



Eigen::Vector3f EKF::getRPY() const{
    float roll  = atan2(2*(X_(6)*X_(7)+X_(8)*X_(9)),1-2*(X_(7)*X_(7)+X_(8)*X_(8)));
    float pitch =                            asin(2*(X_(6)*X_(8)-X_(9)*X_(7)));
    float yaw   = atan2(2*(X_(6)*X_(9)+X_(7)*X_(8)),1-2*(X_(8)*X_(8)+X_(9)*X_(9)));
    
    Eigen::Vector3f RPY(roll,pitch,yaw);
    return RPY;
}



Eigen::Vector3f EKF::getActualPos() const{
    Eigen::Vector3f result;
    result << X_(0),X_(1),X_(2);
    return result;
}



Eigen::Vector3f EKF::getActualSpeed() const{
    Eigen::Vector3f result;
    result << X_(3),X_(4),X_(5);
    return result;
}



Vector6f EKF::getActualInertialOffsets() const{
    Vector6f result;
    result.setZero();
    result << X_(10),X_(11),X_(12),X_(13),X_(14),X_(15);
    return result;
}


void EKF::predict(){
    _X = J*X_;
    _P = J*P_*(J.transpose())+Q;
    X_=_X;
    P_ = _P;
}


void EKF::updateMeasure(){
    for (int i=0; i<3;++i){
        Z(i) = (gps_filter->getState())(i+3)-_X(i);  // Position
    } //std::cout << Z.transpose() <<std::endl;
    for (int i=3; i<6;++i){
        Z(i) = (gps_filter->getState())(i-3)-_X(i);    // Speed
    }
    for (int i=6; i<9;++i){
        Z(i) = (mag->getState())(i-6);    // Magnetic field
    }
    //std::cout << Z.transpose() << std::endl;
}


void EKF::correct(){

    //We actualize the measurement vector with the gps_filter datas
    updateMeasure();
    //We create dimensionnaly acceptable alias
    Eigen::Vector3f Z_alias;
    Z_alias.setZero();
    
    // We first correct the position
    for (int i=0; i<3; ++i){
        Z_alias(i) = Z(i);
    }
    if ((H_GPS_POS*_P*(H_GPS_POS.transpose())+gps_filter->getPositionCovMatrix()).inverse().determinant() != 0){
        K = _P*(H_GPS_POS.transpose())*((H_GPS_POS*_P*(H_GPS_POS.transpose())+gps_filter->getPositionCovMatrix()).inverse()); //Computes K
    }
    P_ = _P - K*H_GPS_POS*_P;                                                              //Computes P
    //Correct prediction
    X_ = _X + K*Z_alias;
    
    // We then correct the speed :
    for (int i=0; i<3; ++i){
        Z_alias(i) = Z(i+3);
    }
    if ((H_GPS_SPEED*_P*(H_GPS_SPEED.transpose())+gps_filter->getSpeedCovMatrix()).inverse().determinant() != 0){
        K = _P*(H_GPS_SPEED.transpose())*((H_GPS_SPEED*_P*(H_GPS_SPEED.transpose())+gps_filter->getSpeedCovMatrix()).inverse());     //Computes K
    }
    P_ = _P - K*H_GPS_SPEED*_P;                                                              //Computes P
    //Correct prediction
    X_ = _X + K*Z_alias;
    
    /* TODO : Test
    // Eventually we correct attitude via the magnetometer
    for (int i=0; i<3; ++i){
        Z_alias(i) = Z(i+6);
    }
    // Computing H_MAG
    H_MAG(0,6)  =   2*(X_(6)*(mag->getEEMF())(0)-X_(9)*(mag->getEEMF())(1)+X_(8)*(mag->getEEMF())(2));
    H_MAG(0,7)  =   2*(X_(7)*(mag->getEEMF())(0)+X_(8)*(mag->getEEMF())(1)+X_(9)*(mag->getEEMF())(2));
    H_MAG(0,8)  =  2*(-X_(8)*(mag->getEEMF())(0)+X_(7)*(mag->getEEMF())(1)+X_(6)*(mag->getEEMF())(2));
    H_MAG(0,9)  =  2*(-X_(9)*(mag->getEEMF())(0)-X_(6)*(mag->getEEMF())(1)+X_(7)*(mag->getEEMF())(2));
    H_MAG(1,6)  =   2*(X_(9)*(mag->getEEMF())(0)+X_(6)*(mag->getEEMF())(1)-X_(7)*(mag->getEEMF())(2));
    H_MAG(1,7)  =   2*(X_(8)*(mag->getEEMF())(0)-X_(6)*(mag->getEEMF())(1)-X_(6)*(mag->getEEMF())(2));
    H_MAG(1,8)  =   2*(X_(7)*(mag->getEEMF())(0)+X_(8)*(mag->getEEMF())(1)+X_(9)*(mag->getEEMF())(2));
    H_MAG(1,9)  =   2*(X_(6)*(mag->getEEMF())(0)-X_(9)*(mag->getEEMF())(1)+X_(8)*(mag->getEEMF())(2));
    H_MAG(2,6)  =  2*(-X_(8)*(mag->getEEMF())(0)+X_(7)*(mag->getEEMF())(1)+X_(6)*(mag->getEEMF())(2));
    H_MAG(2,7)  =   2*(X_(9)*(mag->getEEMF())(0)+X_(6)*(mag->getEEMF())(1)-X_(7)*(mag->getEEMF())(2));
    H_MAG(2,8)  =  2*(-X_(6)*(mag->getEEMF())(0)+X_(9)*(mag->getEEMF())(1)-X_(8)*(mag->getEEMF())(2));
    H_MAG(2,9)  =   2*(X_(7)*(mag->getEEMF())(0)+X_(8)*(mag->getEEMF())(1)+X_(9)*(mag->getEEMF())(2));

    
    K = _P*(H_MAG.transpose())*((H_MAG*_P*(H_MAG.transpose())+mag->getNoiseCovarianceMatrix).inverse());     //Computes K // OTDO
    P_ = _P - K*H_MAG*_P;                                                              //Computes P
    //Correct prediction
    X_ = _X + K*(Z_alias-H_MAG*_X);*/
    
}




    
    EKF::~EKF(){
    }
    

