#pragma once
#include "Sensor.h"
#include <vector>
#include <string>


struct Vec2D {
    float x;
    float y;
};

class LineSensor : public Sensor
{
private:
    double mAngleOffset;   // Mounting angle relative to robot heading (radians)
    double mProbeLength;   // Distance from robot center to ground sensing point

    // 2D cross product helper: v x w = v.x * w.y - v.y * w.x
    static double Cross(Vec2D v, Vec2D w);

public:
    LineSensor(std::string id, double angleOffset, double probeLength = 20.0, double initVal = 0.0);
    ~LineSensor() override = default;

    // Checks intersection with map line segments and sets value to 1.0 (on line) or 0.0 (off line)
    void sense(double x, double y, double heading, const std::vector<Vec2D>& mapVerts) override;
    void Report() const override;

    bool IsOnLine() const { return value > 0.5; }
};