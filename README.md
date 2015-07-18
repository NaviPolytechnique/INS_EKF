# INS_EKF


Program reading from log files implementing an extended Kalman filter for the inertial navigation.

To compile, you'll need to install the Eigen library. Simply go on their website to download it and then run : 

        sudo -s
        mv Eigen/ /usr/include


You'll then simply need to run : 

        make && make clean 

The 'make clean' command being optional. 

You'll need to change the path to the log you want to treat, log that would have been stored thanks to the APM_READER programm. 

