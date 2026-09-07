#include "RangeSensor.h"
#include <iostream>
#include <cmath>
#include <algorithm>

RangeSensor::RangeSensor(std::string id, double angleOffset, double maxRange, double initVal)
    : Sensor(std::move(id), initVal), mAngleOffset(angleOffset), mMaxRange(maxRange)
{
}

double RangeSensor::Cross(Vec2D v, Vec2D w)
{
    return v.x * w.y - v.y * w.x;
}

void RangeSensor::sense(double x, double y, double heading, const std::vector<Vec2D>& mapVerts)
{
    double beamAngle = heading + mAngleOffset;
    Vec2D P = { static_cast<float>(x), static_cast<float>(y) };
    Vec2D r = { 
        static_cast<float>(mMaxRange * std::cos(beamAngle)), 
        static_cast<float>(mMaxRange * std::sin(beamAngle)) 
    };

    double closest_t = 1.0;

    size_t count = mapVerts.size();
    for (size_t i = 0; i < count; ++i)
    {
        Vec2D A = mapVerts[i];
        Vec2D B = mapVerts[(i + 1) % count];
        Vec2D s = { B.x - A.x, B.y - A.y };

        double denom = Cross(r, s);

        if (std::abs(denom) > 1e-6)
        {
            Vec2D A_minus_P = { A.x - P.x, A.y - P.y };
            double t = Cross(A_minus_P, s) / denom;
            double u = Cross(A_minus_P, r) / denom;

            if (t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0)
            {
                if (t < closest_t)
                {
                    closest_t = t;
                }
            }
        }
    }

    value = closest_t * mMaxRange;
}

void RangeSensor::Report() const
{
    std::cout << "Sensor ID: " << id 
              << ", Distance: " << value 
              << " / " << mMaxRange << '\n';
}