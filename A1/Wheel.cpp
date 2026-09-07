// Wheel.cpp
// Implementation of the Wheel class. A Wheel stores its commanded linear ground speed.

#include "Wheel.h"

Wheel::Wheel()
    : speed_( 0.0 )
{
}

void Wheel::SetSpeed( double aSpeed )
{
    speed_ = aSpeed;
}

double Wheel::GetSpeed() const
{
    return speed_;
}
