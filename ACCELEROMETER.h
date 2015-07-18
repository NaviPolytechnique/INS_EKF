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




class ACCELEROMETER : public Captor{
 
    
public:
    
    ACCELEROMETER();
    
    ACCELEROMETER(const std::string, STYLE);
    

};

#endif /* defined(__INS_EKF__ACCELEROMETER__) */
