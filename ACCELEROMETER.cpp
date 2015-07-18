//
//  ACCELEROMETER.cpp
//  INS_EKF
//
//  Created by Louis Faury on 12/07/2015.
//  Copyright (c) 2015 Louis Faury. All rights reserved.
//

#include <stdio.h>
#include "ACCELEROMETER.h"

ACCELEROMETER::ACCELEROMETER(){
}


ACCELEROMETER::ACCELEROMETER(const std::string acc_file_name, STYLE style) : Captor(acc_file_name){
    std::cout << "Acceleration file open" << std::endl;
    captor_id = "ACC";
    LINE_MARK = "$A";
    OFFSET_MARK = "$AO";
    
    if (style != COLD_START){
        // TO DO
    }
}

