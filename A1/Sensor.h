#pragma once
#include <string>
#include <vector>

struct Vec2D;

class Sensor
{
protected:
    double value;
    std::string id;

public:
    Sensor(std::string id, double initVal = 0.0);
    virtual ~Sensor() = default;

    virtual void sense(double x, double y, double heading, const std::vector<Vec2D>& mapVerts) = 0;

    virtual double getSensorValue() const;
    virtual void Report() const;
};