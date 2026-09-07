#include "LineSensor.h"
#include <iostream>
#include <cmath>

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
    // 1. Calculate probe orientation and segment vector (P -> P + r)
    double probeAngle = heading + mAngleOffset;
    Vec2D P = { static_cast<float>(x), static_cast<float>(y) };
    Vec2D r = { 
        static_cast<float>(mProbeLength * std::cos(probeAngle)), 
        static_cast<float>(mProbeLength * std::sin(probeAngle)) 
    };

    bool hitDetected = false;
    size_t count = mapVerts.size();

    // 2. Test intersection against every boundary segment using cross-product logic
    for (size_t i = 0; i < count; ++i)
    {
        Vec2D A = mapVerts[i];
        Vec2D B = mapVerts[(i + 1) % count];
        Vec2D s = { B.x - A.x, B.y - A.y };

        double denom = Cross(r, s);

        // Segments are not parallel
        if (std::abs(denom) > 1e-6)
        {
            Vec2D A_minus_P = { A.x - P.x, A.y - P.y };
            double t = Cross(A_minus_P, s) / denom;
            double u = Cross(A_minus_P, r) / denom;

            // Intersection occurs within the probe span and along the wall segment
            if (t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0)
            {
                hitDetected = true;
                break; // One intersection confirms the sensor is over a boundary line
            }
        }
    }

    // 3. Binary state: 1.0 = on line / crossing boundary, 0.0 = clear
    value = hitDetected ? 1.0 : 0.0;
}

void LineSensor::Report() const
{
    std::cout << "LineSensor [" << id << "] "
              << "Status: " << (IsOnLine() ? "ON LINE" : "OFF LINE") << '\n';
}