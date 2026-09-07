#pragma once
#include "Sensor.h"

class RangeSensor : public Sensor
{

public:
    RangeSensor(std::string id, int threshold, int initVal = 0);
    ~RangeSensor() override = default;

    void Report() const override;
};

RangeSensor::RangeSensor(std::string id, int threshold, int initVal)
    : Sensor(id, threshold, initVal)
{
}

void RangeSensor::Report() const
{
    std::cout << "Sensor ID: " << id 
              << ", Distance: " << value 
              << ", Threshold: " << threshold << '\n';
}
