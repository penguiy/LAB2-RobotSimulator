// LineFollower.cpp
#include "LineFollower.h"
#include <iostream>

LineFollower::LineFollower(double aStartX, double aStartY, double aStartHeading)
    : Robot(aStartX, aStartY, aStartHeading, false),
      cruiseSpeed_(30.0),
      turnSpeed_(22.0),
      // Center sensor straight ahead; side sensor offset to the LEFT (-0.35 rad)
      onLineSensor_("onLineSensor", 0.0, 18.0, 0.0),
      sideSensor_("sideSensor", -0.35, 18.0, 0.0),
      controlState_(FOLLOWING_LINE)
{
}

LineFollower::~LineFollower()
{
}

void LineFollower::UpdateSensors(const std::vector<Vec2D>& mapVerts)
{
    onLineSensor_.sense(GetX(), GetY(), GetHeading(), mapVerts);
    sideSensor_.sense(GetX(), GetY(), GetHeading(), mapVerts);
}

void LineFollower::UpdateControl()
{
    const bool onLine = onLineSensor_.getSensorValue() > 0.5;
    const bool onSide = sideSensor_.getSensorValue() > 0.5;

    // With side sensor on LEFT:
    // 1. onLine && !onSide -> robot is nicely aligned along right edge of line
    // 2. onSide            -> line swung left under the side sensor; steer left
    // 3. !onLine && !onSide -> robot swung too far right off the line; steer right
    if (onLine && !onSide)
    {
        controlState_ = FOLLOWING_LINE;
    }
    else if (onSide)
    {
        controlState_ = TURNING_LEFT;
    }
    else
    {
        controlState_ = TURNING_RIGHT;
    }

    const double innerSpeed = cruiseSpeed_ * 0.1; // Keeps robot moving forward through turns

    switch (controlState_)
    {
        case FOLLOWING_LINE:
            SetWheelSpeeds(cruiseSpeed_, cruiseSpeed_);
            break;

        case TURNING_LEFT:
            SetWheelSpeeds(innerSpeed, turnSpeed_);
            break;

        case TURNING_RIGHT:
            SetWheelSpeeds(turnSpeed_, innerSpeed);
            break;

        default:
            SetWheelSpeeds(cruiseSpeed_, cruiseSpeed_);
            break;
    }
}