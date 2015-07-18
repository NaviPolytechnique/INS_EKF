# Makefile for INS_EKF


.PHONY: clean
.SUFFIXES:

# Définitions des variables
CC = gcc
CFLAGS = -v
CXX = g++
CXXFLAGS =  


# Compilation
#
main: EKF.o Captor.o ACCELEROMETER.o GYRO.o GPS.o MAGNETOMETER.o main.o
	$(CXX) $^ -o main $(CXXFLAGS)

%.o: %.cpp 
	$(CXX) -c $< -o $@ $(CXXFLAGS)

clean :
	rm -rf  *.o


  
