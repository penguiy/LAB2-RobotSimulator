#ifndef LINEFOLLOWER_H
#define LINEFOLLOWER_H

#include <vector>
#include "LineSensor.h"
#include "Robot.h"

class LineFollower : public Robot
{
  public:
    LineFollower(
        double aStartX,
        double aStartY,
        double aStartHeading );

    virtual ~LineFollower();

  private:
    enum EControlState
    {
        FOLLOWING_LINE,
        TURNING_LEFT,
        TURNING_RIGHT
    };

    virtual void UpdateSensors(
        const std::vector<Vec2D>& mapVerts ) override;

    virtual void UpdateControl() override;

    // Wheel speeds
    const double cruiseSpeed_;
    const double turnSpeed_;

    LineSensor onLineSensor_;
    LineSensor sideSensor_;

    EControlState controlState_;
};

#endif