//
//  MAGNETOMETER.cpp
//  INS_EKF
//
//  Created by Louis Faury on 12/07/2015.
//  Copyright (c) 2015 Louis Faury. All rights reserved.
//

#include "MAGNETOMETER.h"



MAGNETOMETER::MAGNETOMETER(){
}


MAGNETOMETER::MAGNETOMETER(const std::string mag_file_name, STYLE style) : Captor(mag_file_name){
    std::cout << "Magnetometer file open" << std::endl;
    captor_id = "MAG";
    LINE_MARK = "$M";
    OFFSET_MARK = "$MO";
    
    if (style != COLD_START){
        // TODO
    }
}
