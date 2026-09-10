#include "Sensor.h"
#include <iostream>

Sensor::Sensor(std::string aId, double aInitVal)
    : value(aInitVal), id(std::move(aId))
{
}

double Sensor::getSensorValue() const
{
    return value;
}

void Sensor::Report() const
{
    std::cout << "Sensor ID: " << id << ", Value: " << value << '\n';
}