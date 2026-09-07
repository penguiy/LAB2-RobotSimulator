#include "Robot.h"

#include "CRender.h"

#include <cmath>
#include <iostream>

// default constructor  
Robot::Robot( double aStartX, double aStartY, double aStartHeading )
    :
        radius_( 15.0 ),
        wheelSeparation_( 30.0 ),
        fullTurnRadians_( 6.283185307179586 ),
        leftWheel_(),
        rightWheel_(),
        x_( aStartX ),
        y_( aStartY ),
        heading_( aStartHeading ),
        trail_(),
        collisionCount_( 0 ),
        collisionActive_( false ),
        collidedLastUpdate_( false )
{
    NormaliseHeading();
    RecordTrailPoint();
}


Robot::~Robot()
{
}


void Robot::SetPose( double aX, double aY, double aHeading )
{
    x_ = aX;
    y_ = aY;
    heading_ = aHeading;

    collisionCount_ = 0;
    collisionActive_ = false;
    collidedLastUpdate_ = false;

    NormaliseHeading();

    trail_.clear();
    RecordTrailPoint();
}

// update the robot's position and heading based on the current wheel speeds, the elapsed time step, and the map walls.
void Robot::Update(
    double aTimeStep,
    const std::vector<Vec2D>& mapVerts )
{
    if( aTimeStep <= 0.0 )
    {
        std::cout << "ERROR: Robot received an invalid timestep."
                  << std::endl;
    }
    else
    {
        UpdateSensors( mapVerts );
        UpdateControl();
        Move( aTimeStep, mapVerts );
        RecordTrailPoint();
    }
}

// Draw consecutive recorded positions to leave a persistent trail.
void Robot::Draw( CRender& aRender ) const
{
    // Draw the trail first so the robot is drawn on top.
    for( std::vector<TrailPoint>::size_type i = 1;
         i < trail_.size();
         ++i )
    {
        Vec2D Start =
        {
            static_cast<float>( trail_[i - 1].x ),
            static_cast<float>( trail_[i - 1].y )
        };

        Vec2D End =
        {
            static_cast<float>( trail_[i].x ),
            static_cast<float>( trail_[i].y )
        };

        // rendering the trail of the robot
        aRender.DrawLine(
            Start,
            End,
            2.0f,
            BLUE );
    }

    // Draw robot with circle and line for heading.
    Vec2D Centre =
    {
        static_cast<float>( x_ ),
        static_cast<float>( y_ )
    };

    // The short line from the centre shows the robot's heading.
    Vec2D Front =
    {
        static_cast<float>(
            x_ + radius_ * std::cos( heading_ ) ),
        static_cast<float>(
            y_ + radius_ * std::sin( heading_ ) )
    };

    // rendering robot 
    aRender.DrawCircle(
        Centre,
        static_cast<int>( radius_ ),
        SKYBLUE );

    aRender.DrawLine(
        Centre,
        Front,
        2.0f,
        WHITE );
}

// returning position and heading of the robot
double Robot::GetX() const
{
    return x_;
}


double Robot::GetY() const
{
    return y_;
}


double Robot::GetHeading() const
{
    return heading_;
}

// Get collision count of the robot
int Robot::GetCollisionCount() const
{
    return collisionCount_;
}

bool Robot::CollidedLastUpdate() const
{
    return collidedLastUpdate_;
}

// Get wheel speed of the robot
void Robot::SetWheelSpeeds( double aLeftSpeed, double aRightSpeed )
{
    leftWheel_.SetSpeed( aLeftSpeed );
    rightWheel_.SetSpeed( aRightSpeed );
}

// Move robot on wheelspeed, timestep and map walls.
void Robot::Move(
    double aTimeStep,
    const std::vector<Vec2D>& mapVerts )
{
    const double LeftSpeed = leftWheel_.GetSpeed();
    const double RightSpeed = rightWheel_.GetSpeed();

    const double LinearSpeed =
        ( LeftSpeed + RightSpeed ) / 2.0;

    // Screen y increases downward and map headings increase clockwise.
    // Left wheel > Right wheel, rotates clockwise, so angular speed is positive.
    const double AngularSpeed =
        ( LeftSpeed - RightSpeed ) / wheelSeparation_;

    // Use the midpoint method to estimate the robot's new position and heading.
    const double MiddleHeading =
        heading_ + AngularSpeed * aTimeStep / 2.0;

    const double ProposedX =
        x_ + LinearSpeed * std::cos( MiddleHeading ) * aTimeStep;

    const double ProposedY =
        y_ + LinearSpeed * std::sin( MiddleHeading ) * aTimeStep;

    const double ProposedHeading =
        heading_ + AngularSpeed * aTimeStep;

    // Check for collision with the map walls at the proposed new position and heading.
    const bool Collision =
        CollidesWithWall(
            ProposedX,
            ProposedY,
            mapVerts );
    
    // Update collision state
    collidedLastUpdate_ = Collision;
    
    // record colision at position and heading for collision state active
    if( Collision )
    {
        // Count a continuous contact as one collision event rather than
        // counting every update while the robot remains against the wall.
        if( !collisionActive_ )
        {
            ++collisionCount_;

            std::cout << "Collision " << collisionCount_
                      << " at (" << ProposedX
                      << ", " << ProposedY << ")"
                      << std::endl;
        }

        collisionActive_ = true;
    }
    else // no collision, so update position and heading
    {
        x_ = ProposedX;
        y_ = ProposedY;
        heading_ = ProposedHeading;
        collisionActive_ = false;

        NormaliseHeading();
    }
}

// Check if the robot's circular body collides with any wall segment.
bool Robot::CollidesWithWall(
    double aCentreX,
    double aCentreY,
    const std::vector<Vec2D>& mapVerts ) const
{
    bool Collision = false;
    const double RadiusSquared = radius_ * radius_;

    if( mapVerts.size() >= 2 )
    {
        for( std::vector<Vec2D>::size_type i = 0;
             i < mapVerts.size() && !Collision;
             ++i )
        {
            const Vec2D& WallStart = mapVerts[i];
            const Vec2D& WallEnd =
                mapVerts[( i + 1 ) % mapVerts.size()];

            Collision =
                PointToWallDistanceSquared(
                    aCentreX,
                    aCentreY,
                    WallStart,
                    WallEnd ) <= RadiusSquared;
        }
    }

    return Collision;
}

// Compute the squared distance from a point to wall segment.
double Robot::PointToWallDistanceSquared(
    double aPointX,
    double aPointY,
    const Vec2D& aWallStart,
    const Vec2D& aWallEnd ) const
{
    const double WallX = aWallEnd.x - aWallStart.x;
    const double WallY = aWallEnd.y - aWallStart.y;

    const double WallLengthSquared =
        WallX * WallX + WallY * WallY;

    double WallFraction = 0.0;

    if( WallLengthSquared > 0.0 )
    {
        // Project the point onto the wall segment, computing the fraction along the wall.
        WallFraction =
            ( ( aPointX - aWallStart.x ) * WallX +
              ( aPointY - aWallStart.y ) * WallY ) /
            WallLengthSquared;

        if( WallFraction < 0.0 )
        {
            WallFraction = 0.0;
        }
        else if( WallFraction > 1.0 )
        {
            WallFraction = 1.0;
        }
    }

    const double NearestX =
        aWallStart.x + WallFraction * WallX;

    const double NearestY =
        aWallStart.y + WallFraction * WallY;

    const double DifferenceX =
        aPointX - NearestX;

    const double DifferenceY =
        aPointY - NearestY;

    // Return the squared distance.
    return
        DifferenceX * DifferenceX +
        DifferenceY * DifferenceY;
}

// Record the robot's current position as a trail point.
void Robot::RecordTrailPoint()
{
    TrailPoint NewPoint;

    NewPoint.x = x_;
    NewPoint.y = y_;

    trail_.push_back( NewPoint );
}

// Normalize the robot's heading to be in the range [0, 2*PI).
void Robot::NormaliseHeading()
{
    while( heading_ >= fullTurnRadians_ )
    {
        heading_ -= fullTurnRadians_;
    }

    while( heading_ < 0.0 )
    {
        heading_ += fullTurnRadians_;
    }
}
