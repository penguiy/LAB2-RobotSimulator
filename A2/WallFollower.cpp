// WallFollower.cpp
// Implementation of the WallFollower class. A WallFollower is a Robot that follows walls.
// Right wall follwing.

#include "WallFollower.h"

// WallFollower constructor with starting parameters.
WallFollower::WallFollower(double aStartX, double aStartY, double aStartHeading ):
        // Initialize the base Robot class with starting position and heading.
        Robot( aStartX, aStartY, aStartHeading, true ),
        pi_( 3.141592653589793 ),
        sensorMaximumDistance_( 250.0 ),
        targetWallDistance_( 27.0 ),
        frontCornerDistance_( 32.0 ),
        wallLostDistance_( 75.0 ),
        cruiseSpeed_( 35.0 ),
        turnSpeed_( 22.0 ),
        searchOuterSpeed_( 35.0 ),
        searchInnerSpeed_( 10.0 ),
        distanceGain_( 1.2 ),
        headingGain_( 0.8 ),
        sinFortyFiveDegrees_( 0.707106781186548 ),
        maximumCorrection_( 18.0 ),
        collisionRecoveryUpdates_( 24 ),
        
        // Map headings increase clockwise, so +90 degrees is right and
        // +45 degrees is forward-right.
        rightSensor_(
            "right",
            pi_ / 2.0,
            sensorMaximumDistance_,
            sensorMaximumDistance_ ),

        frontRightSensor_(
            "front-right",
            pi_ / 4.0,
            sensorMaximumDistance_,
            sensorMaximumDistance_ ),

        controlState_( FOLLOWING_WALL ),
        recoveryUpdatesRemaining_( 0 )
{
}


WallFollower::~WallFollower()
{
}

// Get the distance reading from the sensors.
void WallFollower::UpdateSensors(
    const std::vector<Vec2D>& mapVerts )
{
    rightSensor_.sense(
        GetX(),
        GetY(),
        GetHeading(),
        mapVerts );

    frontRightSensor_.sense(
        GetX(),
        GetY(),
        GetHeading(),
        mapVerts );
}

// Control states using the sensor readings to control the robot's movement.
void WallFollower::UpdateControl()
{
    const double RightDistance =
        rightSensor_.getSensorValue();

    const double FrontRightDistance =
        frontRightSensor_.getSensorValue();

    // A sensor returns its maximum range when no wall is detected.
    const bool RightWallFound =
        RightDistance < sensorMaximumDistance_;

    const bool FrontRightWallFound =
        FrontRightDistance < sensorMaximumDistance_;

    if( CollidedLastUpdate() )
    {
        recoveryUpdatesRemaining_ =
            collisionRecoveryUpdates_;
    }

    // Choose one of three simple behaviours.
    if( recoveryUpdatesRemaining_ > 0 )
    {
        controlState_ = TURNING_LEFT;
        --recoveryUpdatesRemaining_;
    }
    else if( FrontRightWallFound &&
             FrontRightDistance < frontCornerDistance_ )
    {
        controlState_ = TURNING_LEFT;
    }
    else if( !RightWallFound ||
             RightDistance > wallLostDistance_ )
    {
        controlState_ = FINDING_WALL;
    }
    else
    {
        controlState_ = FOLLOWING_WALL;
    }

    if( controlState_ == TURNING_LEFT )
    {
        // Rotate on the spot to clear an inside corner or collision.
        SetWheelSpeeds(
            -turnSpeed_,
            turnSpeed_ );
    }
    else if( controlState_ == FINDING_WALL )
    {
        // Follow a right-hand arc until the side sensor sees the wall again.
        SetWheelSpeeds(
            searchOuterSpeed_,
            searchInnerSpeed_ );
    }
    else
    {
        double Correction =
            distanceGain_ *
            ( targetWallDistance_ - RightDistance );

        if( FrontRightWallFound )
        {
            // For a wall parallel to the robot, a 45-degree sensor reading
            // projected sideways should approximately match RightDistance.
            const double ProjectedFrontDistance =
                FrontRightDistance *
                sinFortyFiveDegrees_;

            Correction +=
                headingGain_ *
                ( RightDistance -
                  ProjectedFrontDistance );
        }

        Correction =
            LimitCorrection( Correction );

        SetWheelSpeeds(
            cruiseSpeed_ - Correction,
            cruiseSpeed_ + Correction );
    }
}

// Limit the correction to avoid wheel speeds going negative or too high.
double WallFollower::LimitCorrection(
    double aCorrection ) const
{
    double Result = aCorrection;

    if( Result > maximumCorrection_ )
    {
        Result = maximumCorrection_;
    }
    else if( Result < -maximumCorrection_ )
    {
        Result = -maximumCorrection_;
    }

    return Result;
}
