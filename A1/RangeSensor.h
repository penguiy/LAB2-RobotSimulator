#pragma once
#include "Sensor.h"
#include <vector>
#include <string>

class RangeSensor : public Sensor
{
private:
    double mAngleOffset;
    double mMaxRange;

    static double Cross(Vec2D v, Vec2D w);

public:
    RangeSensor(std::string aId, double angleOffset, double maxRange = 200.0, double aInitVal = 0.0);
    ~RangeSensor() override = default;

    void sense(double x, double y, double heading, const std::vector<Vec2D>& mapVerts) override;
    void Report() const override;
};