//
//  GYRO.cpp
//  INS_EKF
//
//  Created by Louis Faury on 12/07/2015.
//  Copyright (c) 2015 Louis Faury. All rights reserved.
//

#include <stdio.h>
#include "GYRO.h"


GYRO::GYRO(){
}

GYRO::GYRO(const std::string gyro_file_name, STYLE style) : Captor(gyro_file_name){
    std::cout << "Gyro file open" << std::endl;
    captor_id = "GYRO";
    LINE_MARK = "$G";
    OFFSET_MARK = "$GO";
    
    if (style != COLD_START){
        // TODO
    }
}

