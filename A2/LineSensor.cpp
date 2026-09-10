#include "LineSensor.h"
#include "CRender.h"

#include <iostream>
#include <cmath>
#include <algorithm>

LineSensor::LineSensor(std::string id, double angleOffset, double probeLength, double initVal)
    : Sensor(std::move(id), initVal), mAngleOffset(angleOffset), mProbeLength(probeLength)
{
}

double LineSensor::Cross(Vec2D v, Vec2D w)
{
    return v.x * w.y - v.y * w.x;
}

void LineSensor::sense(double x, double y, double heading, const std::vector<Vec2D>& mapVerts)
{
    // 1. Calculate the ground coordinates directly beneath the sensor probe tip
    double sensorAngle = heading + mAngleOffset;
    double sx = x + mProbeLength * std::cos(sensorAngle);
    double sy = y + mProbeLength * std::sin(sensorAngle);

    bool hitDetected = false;
    
    // Half of the 5-unit line width (2.5) plus a small tolerance margin (0.5)
    const double halfLineWidth = 3.0;
    const double thresholdSq = halfLineWidth * halfLineWidth;
    const size_t count = mapVerts.size();

    // 2. Measure distance from the sensor point (sx, sy) to each line segment AB
    for (size_t i = 0; i < count; ++i)
    {
        Vec2D A = mapVerts[i];
        Vec2D B = mapVerts[(i + 1) % count];

        double dx = B.x - A.x;
        double dy = B.y - A.y;
        double segLenSq = dx * dx + dy * dy;

        if (segLenSq < 1e-6) continue;

        // Project sensor point (sx, sy) onto segment AB, clamped to [0, 1]
        double t = ((sx - A.x) * dx + (sy - A.y) * dy) / segLenSq;
        t = std::max(0.0, std::min(1.0, t));

        // Coordinates of the closest point on segment AB
        double closestX = A.x + t * dx;
        double closestY = A.y + t * dy;

        // Squared distance between sensor head and the segment
        double distSq = (sx - closestX) * (sx - closestX) + (sy - closestY) * (sy - closestY);

        if (distSq <= thresholdSq)
        {
            hitDetected = true;
            break;
        }
    }

    // 3. Binary output: 1.0 if inside the line's width, 0.0 otherwise
    value = hitDetected ? 1.0 : 0.0;
}

void LineSensor::Report() const
{
    std::cout << "LineSensor [" << id << "] "
              << "Status: " << (IsOnLine() ? "ON LINE" : "OFF LINE") << '\n';
}