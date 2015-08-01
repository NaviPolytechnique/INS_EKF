//
//  Captors.cpp
//  INS_EKF
//
//  Created by Louis Faury on 12/07/2015.
//  Copyright (c) 2015 Louis Faury. All rights reserved.
//

#include "Captor.h"


Captor::Captor(){};


Captor::Captor(const std::string cap_file_name) throw(std::exception) : captor_file(cap_file_name)
{
    if (!captor_file) throw std::domain_error("Couldn't open file");
}



int Captor::initOffsets(){
    int count = 0;
    std::string::size_type sz;
    if (std::getline(captor_file,line)){
        if (line[2] == 'O'){
            std::stringstream ss(line);
            while (getline(ss,line,',')){
                if (line == OFFSET_MARK);
                else {
                    captor_offsets[count] = std::stof(line,&sz);
                    count++;
                }
            }
           //std::cout << captor_offsets.transpose() << std::endl;
            return 1;
        }
        else return 0;
    }
    return -1;
}



Eigen::Vector3f Captor::getOffsets() const{
    return captor_offsets;
}


int Captor::initGain(){
    captor_gain << 1,1,1;
    return 1;
}



void Captor::getOutput(Eigen::Vector3f* captor_vector_buffer){
    int count = 0;
    std::string::size_type sz;
    if (std::getline(captor_file,line)){
        line_end = false;
            std::stringstream ss(line);
            while (getline(ss,line,',')){
                if (line == LINE_MARK);
                else {
                    (*captor_vector_buffer)[count] = std::stof(line,&sz);
                    count++;
                }
            }
        std::cout << captor_id << " : line read" << std::endl;
    }
    else{
        std::cout << "Couldn't read from captor : " << captor_id << std::endl;
        line_end = true;
    }
}



void Captor::correctOutput(Eigen::Vector3f* captor_buffer){
    (*captor_buffer) -= captor_offsets;
    for (int i=0; i<3; ++i){
        (*captor_buffer)(i) *= captor_gain(i);
    }
}




Captor::~Captor(){
}






