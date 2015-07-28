//
//  Captors.h
//  INS_EKF
//
//  Created by Louis Faury on 12/07/2015.
//  Copyright (c) 2015 Louis Faury. All rights reserved.
//

#ifndef __INS_EKF__Captor__
#define __INS_EKF__Captor__

#include <stdio.h>
#include <fstream>
#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include </usr/include/Eigen/Dense>
#include "GPS.h"



class Captor{

public:

    // Basic constructor of the class
    Captor();

    
    // Name for mark syle, see the captors constructors
    enum STYLE{
        COLD_START,
        CALIBRATED
    };

    
    /* Basic constructor of the class
    * \brief Instanciate a stream to a file given the file path name
    * \param The file path name
    */
    Captor(const std::string) throw(std::exception);



   /* \brief Store offset read in the reading file into a vector of float.
    * \brief Return 1 if succeeded in opening file and 0 if failed to detect the initial offset line
    * \brief Return -1 if couldn't get any line
    * TODO : Implement Gauss-Newton method for better initialisation
    */
    int initOffsets();
    
    
    
   /* \brief Initialize the captor's gain vector. 
    * \brief For now, we'll assume the gain vector starts at {1,1,1}
    * \brief Return 1 (success) for now 
    * TODO : Implement Gauss-Newton method for better initialisation
    */
    int initGain();


    
    
   /* \brief Returns the captor's offset
    */
    Eigen::Vector3f getOffsets() const;
    
    
    
   /* \brief Reads line from captor file and return it as an output vector
    */
    void getOutput(Eigen::Vector3f*);
    
    
    
   /* \brief Correct the captor's output with its offset
    * \param The outbut buffer you want to correct
    */
    void correctOutput(Eigen::Vector3f*);
    
    
    
    
    // TODO
    void setOffsets(Eigen::Vector3f);
    
    
    
    ~Captor();
    
    bool line_end = false;
    
    

    
protected:

    std::ifstream captor_file;
    Eigen::Vector3f captor_offsets;
    Eigen::Vector3f captor_gain;
    std::string line;
    std::string OFFSET_MARK; // Offset mark for the setOffsets method
    std::string captor_id;
    std::string LINE_MARK; // Line mark for the getOutput method

};


#endif /* defined(__INS_EKF__Captor__) */
