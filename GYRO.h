//
//  GYRO.h
//  INS_EKF
//
//  Created by Louis Faury on 12/07/2015.
//  Copyright (c) 2015 Louis Faury. All rights reserved.
//

#ifndef __INS_EKF__GYRO__
#define __INS_EKF__GYRO__

#include <stdio.h>
#include "Captor.h"

class GYRO : public Captor {
public:
    
    GYRO();
    
    GYRO(const std::string, STYLE);
    
    
};

#endif /* defined(__INS_EKF__GYRO__) */
