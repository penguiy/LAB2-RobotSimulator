#ifndef ROBOT_H
#define ROBOT_H

#include <vector>

#include "Wheel.h"

struct Vec2D;
class CRender;

// A Robot stores the common state of a differential-drive robot.
// Derived classes provide their own sensing and control behaviour.
class Robot
{
  public:
    Robot( double aStartX, double aStartY, double aStartHeading, bool enableCollisions = true);
    virtual ~Robot();

    void SetPose( double aX, double aY, double aHeading );

    void Update(
        double aTimeStep,
        const std::vector<Vec2D>& mapVerts );

    void Draw( CRender& aRender ) const;

    double GetX() const;
    double GetY() const;
    double GetHeading() const;
    int GetCollisionCount() const;

  protected:
    virtual void UpdateSensors(
        const std::vector<Vec2D>& mapVerts ) = 0;

    virtual void UpdateControl() = 0;

    void SetWheelSpeeds( double aLeftSpeed, double aRightSpeed );
    bool CollidedLastUpdate() const;

  private:
    struct TrailPoint
    {
        double x;
        double y;
    };

    void Move(
        double aTimeStep,
        const std::vector<Vec2D>& mapVerts );

    bool CollidesWithWall(
        double aCentreX,
        double aCentreY,
        const std::vector<Vec2D>& mapVerts ) const;

    double PointToWallDistanceSquared(
        double aPointX,
        double aPointY,
        const Vec2D& aWallStart,
        const Vec2D& aWallEnd ) const;

    void RecordTrailPoint();
    void NormaliseHeading();

    const double radius_;
    const double wheelSeparation_;
    const double fullTurnRadians_;

    Wheel leftWheel_;
    Wheel rightWheel_;

    double x_;
    double y_;
    double heading_;

    std::vector<TrailPoint> trail_;

    int collisionCount_;
    bool collisionActive_;
    bool collidedLastUpdate_;
    bool enableCollisions_;

};

#endif
