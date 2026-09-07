#pragma once
#include "Sensor.h"
#include <vector>
#include <string>

struct Vec2D {
    float x;
    float y;
};

class RangeSensor : public Sensor
{
private:
    double mAngleOffset;
    double mMaxRange;

    static double Cross(Vec2D v, Vec2D w);

public:
    RangeSensor(std::string id, double angleOffset, double maxRange = 200.0, double initVal = 0.0);
    ~RangeSensor() override = default;

    void sense(double x, double y, double heading, const std::vector<Vec2D>& mapVerts) override;
    void Report() const override;
};