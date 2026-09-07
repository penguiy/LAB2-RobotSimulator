#ifndef WALLFOLLOWER_H
#define WALLFOLLOWER_H

#include <vector>

#include "RangeSensor.h"
#include "Robot.h"

// A WallFollower is a Robot that uses two right-facing range sensors to
// choose wheel speeds and follow the room wall.
class WallFollower : public Robot
{
  public:
    WallFollower(
        double aStartX,
        double aStartY,
        double aStartHeading );

    virtual ~WallFollower();

  private:
    enum EControlState
    {
        FOLLOWING_WALL,
        TURNING_LEFT,
        FINDING_WALL
    };

    virtual void UpdateSensors(
        const std::vector<Vec2D>& mapVerts );

    virtual void UpdateControl();

    double LimitCorrection(
        double aCorrection ) const;

    // Sensor distances
    const double pi_;
    const double sensorMaximumDistance_;
    const double targetWallDistance_;
    const double frontCornerDistance_;
    const double wallLostDistance_;

    // Wheel speeds
    const double cruiseSpeed_;
    const double turnSpeed_;
    const double searchOuterSpeed_;
    const double searchInnerSpeed_;

    // Wall-following correction
    const double distanceGain_;
    const double headingGain_;
    const double sinFortyFiveDegrees_;
    const double maximumCorrection_;

    const int collisionRecoveryUpdates_;

    RangeSensor rightSensor_;
    RangeSensor frontRightSensor_;

    EControlState controlState_;
    int recoveryUpdatesRemaining_;
};

#endif
