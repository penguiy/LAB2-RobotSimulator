// Main.cpp
// Runs the simulation with the simulation parameters specified in Simulation.h.
// Compile program with:
/*
g++ -std=c++11 \
-Wall -Wextra -Wshadow -Wnon-virtual-dtor -pedantic \
main.cpp \
Wheel.cpp \
Sensor.cpp \
RangeSensor.cpp \
Robot.cpp \
WallFollower.cpp \
Simulation.cpp \
CLoopReader.cpp \
CRender.cpp \
-o wall_follower \
$(pkg-config --cflags --libs raylib)
*/
// ./Wall_test

#include "Simulation.h"

int main()
{
    Simulation simulation;
    simulation.Run();

    return 0;
}