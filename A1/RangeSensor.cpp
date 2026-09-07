
#include <iostream>
#include <string>
#include <Sensor.h>

class RangeSensor : public Sensor
{
private:
    int value;
    int threshold;
    std::string id;
public:
    RangeSensor(std::string id, int value = 0, int threshold);

    ~RangeSensor();
    void Report() const;
};

RangeSensor::RangeSensor(std::string id, int initVal, int threshold)
    : id(std::string(id)), value(initVal), threshold(threshold)
{
}

RangeSensor::~RangeSensor()
{
}

void RangeSensor::Report() const
{
    std::cout << "Sensor ID: " << id << ", Distance: " << value << '\n';
}