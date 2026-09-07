#pragma once
#include <iostream>
#include <string>

class Sensor
{
protected:
    double value;
    double threshold;
    std::string id;

public:
    Sensor(std::string id, double threshold, double initVal = 0);
    virtual ~Sensor() = default;
    virtual double getSensorValue() const;
    virtual void Report() const;
};

Sensor::Sensor(std::string id, double threshold, double initVal)
    : value(initVal), threshold(threshold), id(id)
{
}

void Sensor::Report() const
{
    std::cout << "Sensor ID: " << id << ", Value: " << value << '\n';
}

void sense(double x, double y) 
{

}

double Sensor::getSensorValue() const
{
    return value;
}

