#include "Sensor.h"
#include <iostream>

Sensor::Sensor(std::string id, double initVal)
    : value(initVal), id(std::move(id))
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