#ifndef WHEEL_H
#define WHEEL_H

// A Wheel stores its commanded linear ground speed.
class Wheel
{
  public:
    Wheel();

    void SetSpeed( double aSpeed );
    double GetSpeed() const;

  private:
    double speed_;
};

#endif
