#include <cmath>
#include "OmniDriveOdometry.hpp"
#include "gz/math/RollingMean.hh"

using namespace gz;
using namespace gz::math;

// The implementation was borrowed from: https://github.com/ros-controls/ros_controllers/blob/melodic-devel/diff_drive_controller/src/odometry.cpp

//////////////////////////////////////////////////
OmniDriveOdometry::OmniDriveOdometry(size_t _windowSize)
{
  this->SetVelocityRollingWindowSize(_windowSize);
}

//////////////////////////////////////////////////
void OmniDriveOdometry::Init(const clock::time_point &_time)
{
  // Reset accumulators and timestamp.
  this->linearMean.Clear();
  this->angularMean.Clear();
  this->x = 0.0;
  this->y = 0.0;
  this->heading = 0.0;
  this->linearVel = 0.0;
  this->angularVel = 0.0;
  this->leftWheelOldPosFront = 0.0;
  this->leftWheelOldPosBack = 0.0;
  this->rightWheelOldPosFront = 0.0;
  this->rightWheelOldPosBack = 0.0;

  this->lastUpdateTime = _time;
  this->initialized = true;
}

//////////////////////////////////////////////////
bool OmniDriveOdometry::Initialized() const
{
  return this->initialized;
}

//////////////////////////////////////////////////
bool OmniDriveOdometry::Update(const Angle &_leftPosFront, const Angle &_leftPosBack, 
                      const Angle &_rightPosFront, const Angle &_rightPosBack,
                      const clock::time_point &_time)
{
  // Compute x, y and heading using velocity
  const std::chrono::duration<double> dt =
    _time - this->lastUpdateTime;

  // Get current wheel joint positions:
  const double leftWheelCurPosFront= *_leftPosFront * this->wheelRadius;
  const double leftWheelCurPosBack = *_leftPosBack * this->wheelRadius;
  const double rightWheelCurPosFront = *_rightPosFront * this->wheelRadius;
  const double rightWheelCurPosBack = *_rightPosBack * this->wheelRadius;

  // Estimate velocity of wheels using old and current position:
  const double leftWheelEstVelFront = leftWheelCurPosFront - this->leftWheelOldPosFront;
  const double leftWheelEstVelBack = leftWheelCurPosBack - this->leftWheelOldPosBack;
  const double rightWheelEstVelFront = rightWheelCurPosFront - this->rightWheelOldPosFront;
  const double rightWheelEstVelBack = rightWheelCurPosBack - this->rightWheelOldPosBack;

  // Update old position with current
  this->leftWheelOldPosFront = leftWheelCurPosFront;
  this->leftWheelOldPosBack = leftWheelCurPosBack;
  this->rightWheelOldPosFront = rightWheelCurPosFront;
  this->rightWheelOldPosBack = rightWheelCurPosBack;

  // Compute linear and angular diff
  const double linear = (rightWheelEstVel + leftWheelEstVel) * 0.5;
  const double angular = (rightWheelEstVel - leftWheelEstVel) /
    this->wheelSeparation;

  this->IntegrateExact(linear, angular);

  // We cannot estimate the speed if the time interval is zero (or near
  // zero).
  if (equal(0.0, dt.count()))
    return false;

  this->lastUpdateTime = _time;

  // Estimate speeds using a rolling mean to filter them out:
  this->linearMean.Push(linear / dt.count());
  this->angularMean.Push(angular / dt.count());

  this->linearVel = this->linearMean.Mean();
  this->angularVel = this->angularMean.Mean();

  return true;
}

//////////////////////////////////////////////////
void OmniDriveOdometry::SetWheelParams(double _wheelSeparation,
    double _leftWheelRadius, double _rightWheelRadius)
{
  this->wheelSeparation = _wheelSeparation;
  this->leftWheelRadius = _leftWheelRadius;
  this->rightWheelRadius = _rightWheelRadius;
}

//////////////////////////////////////////////////
void OmniDriveOdometry::SetVelocityRollingWindowSize(size_t _size)
{
  this->linearMean.SetWindowSize(_size);
  this->angularMean.SetWindowSize(_size);
}

//////////////////////////////////////////////////
const Angle &OmniDriveOdometry::Heading() const
{
  return this->heading;
}

//////////////////////////////////////////////////
double OmniDriveOdometry::X() const
{
  return this->x;
}

//////////////////////////////////////////////////
double OmniDriveOdometry::Y() const
{
  return this->y;
}

//////////////////////////////////////////////////
double OmniDriveOdometry::LinearVelocity() const
{
  return this->linearVel;
}

//////////////////////////////////////////////////
const Angle &OmniDriveOdometry::AngularVelocity() const
{
  return this->angularVel;
}

//////////////////////////////////////////////////
void OmniDriveOdometry::Implementation::IntegrateRungeKutta2(
    double _linear, double _angular)
{
  const double direction = *this->heading + _angular * 0.5;

  // Runge-Kutta 2nd order integration:
  this->x += _linear * std::cos(direction);
  this->y += _linear * std::sin(direction);
  this->heading += _angular;
}

//////////////////////////////////////////////////
void OmniDriveOdometry::Implementation::IntegrateExact(double _linear,
                                double _angular)
{
  if (std::fabs(_angular) < 1e-6)
  {
    this->IntegrateRungeKutta2(_linear, _angular);
  }
  else
  {
    // Exact integration (should solve problems when angular is zero):
    const double headingOld = *this->heading;
    const double ratio = _linear / _angular;
    this->heading += _angular;
    this->x += ratio * (std::sin(*this->heading) - std::sin(headingOld));
    this->y += -ratio * (std::cos(*this->heading) - std::cos(headingOld));
  }
}