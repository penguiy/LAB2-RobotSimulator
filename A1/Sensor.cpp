#include <iostream>
#include <string>

class Sensor
{
private:
    double value;
    double threshold;
    std::string id;
public:
    Sensor(std::string id, double value = 0, double threshold = 0);
    ~Sensor();

    virtual void Report() const;
    virtual double getSensorValue() const;
};

Sensor::Sensor(std::string id, double initVal, double initThreshold)
    : id(std::string(id)), value(initVal), threshold(initThreshold)
{
}

Sensor::~Sensor()
{
}

void Sensor::Report() const
{
    std::cout << "Sensor ID: " << id << ", Value: " << value << '\n';
}

double Sensor::getSensorValue() const
{
    return value;
}