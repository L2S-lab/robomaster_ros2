// This module is inspired by DiffDriveOdometry plugin from gz-math from
// https://github.com/gazebosim/gz-math/

// Copyright (c) 2024, Aarsh Thakker
// All rights reserved.
//
// Software License Agreement (BSD License 3.0)
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
//
//  * Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the company nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef OMNIDRIVEODOMETRY_HH_
#define OMNIDRIVEODOMETRY_HH_

#include <chrono>
#include <gz/math/Angle.hh>
#include <gz/math/Export.hh>
#include <gz/math/config.hh>
#include <gz/utils/ImplPtr.hh>
#include <Angle.hh>

using namespace gz::math;
// Use a steady clock
using clock = std::chrono::steady_clock;
  //inline namespace GZ_MATH_VERSION_NAMESPACE {

  /** \class OmniDriveOdometry OmniDriveOdometry.hpp \
   **/
  /// \brief Computes odometry values based on a set of kinematic
  /// properties and wheel speeds for a omni-drive vehicle.
  ///
  /// A vehicle with a heading of zero degrees has a local
  /// reference frame according to the diagram below.
  ///
  ///       Y
  ///       ^
  ///       |
  ///       |
  ///       O--->X(forward)
  ///
  /// Rotating the right wheel while keeping the left wheel fixed will cause
  /// the vehicle to rotate counter-clockwise. For example (excuse the
  /// lack of precision with ASCII art):
  ///
  ///     Y     X(forward)
  ///     ^     ^
  ///      \   /
  ///       \ /
  ///        O
  ///
  /// **Example Usage**
  ///
  /// \code{.cpp}
  /// gz::math::OmniDriveOdometry odom;
  /// odom.SetWheelParams(2.0, 0.5, 0.5);
  /// odom.Init(std::chrono::steady_clock::now());
  ///
  /// // ... Some time later
  ///
  /// // Both wheels have rotated the same amount
  /// odom.Update(GZ_DTOR(2), GZ_DTOR(2), std::chrono::steady_clock::now());
  ///
  /// // ... Some time later
  ///
  /// // The left wheel has rotated, the right wheel did not rotate
  /// odom.Update(GZ_DTOR(4), GZ_DTOR(2), std::chrono::steady_clock::now());
  /// \endcode

class OmniDriveOdometry
{
  /// \brief Constructor.
  /// \param[in] _windowSize Rolling window size used to compute the
  /// velocity mean
  public: explicit OmniDriveOdometry(size_t _windowSize = 10);

  /// \brief Integrates the velocities (linear and angular) using 2nd order
  /// Runge-Kutta.
  /// \param[in] _linear Linear velocity.
  /// \param[in] _angular Angular velocity.
  public: void IntegrateRungeKutta2(double _linear,
                                    double _angular);

  /// \brief Integrates the velocities (linear and angular) using exact
  /// method.
  /// \param[in] _linear Linear velocity.
  /// \param[in] _angular Angular velocity.
  public: void IntegrateExact(double _linear, double _angular);

  /// \brief Current timestamp.
  public: clock::time_point lastUpdateTime;

  /// \brief Current x position in meters.
  public: double x{0.0};

  /// \brief Current y position in meters.
  public: double y{0.0};

  /// \brief Current heading in radians.
  public: Angle heading;

  /// \brief Current velocity in meter/second.
  public: double linearVel{0.0};

  /// \brief Current angular velocity in radians/second.
  public: Angle angularVel;

  /// \brief wheel radius in meters.
  public: double wheelRadius{0.0};

  /// \brief Wheel separation in meters.
  public: double wheelSeparation{1.0};

  /// \brief Wheel base in meters.
  public: double wheelBase{1.0};

  /// \brief Previous front left wheel position/state in radians.
  public: double leftWheelOldPosFront{0.0};

  /// \brief Previous back left wheel position/state in radians.
  public: double leftWheelOldPosBack{0.0};

  /// \brief Previous front right wheel position/state in radians.
  public: double rightWheelOldPosFront{0.0};

  /// \brief Previous back right wheel position/state in radians.
  public: double rightWheelOldPosBack{0.0};

  /// \brief Rolling mean accumulators for the linear velocity
  public: RollingMean linearMean {0};

  /// \brief Rolling mean accumulators for the angular velocity
  public: RollingMean angularMean {0};

  /// \brief Initialized flag.
  public: bool initialized{false};

  /// \brief Initialize the odometry
  /// \param[in] _time Current time.
  public: void Init(const clock::time_point &_time);

  /// \brief Get whether Init has been called.
  /// \return True if Init has been called, false otherwise.
  public: bool Initialized() const;

  /// \brief Updates the odometry class with latest wheels and
  /// steerings position
  /// \param[in] _leftPosFront Front left wheel position in radians.
  /// \param[in] _leftPosBack Back left wheel position in radians.
  /// \param[in] _rightPosFront Front right wheel postion in radians.
  /// \param[in] _rightPosBack Back right wheel postion in radians.
  /// \param[in] _time Current time point.
  /// \return True if the odometry is actually updated.
  public: bool Update(const Angle &_leftPosFront, const Angle &_leftPosBack, 
                      const Angle &_rightPosFront, const Angle &_rightPosBack,
                      const clock::time_point &_time);

  /// \brief Get the heading.
  /// \return The heading in radians.
  public: const Angle &Heading() const;

  /// \brief Get the X position.
  ///  \return The X position in meters
  public: double X() const;

  /// \brief Get the Y position.
  /// \return The Y position in meters.
  public: double Y() const;

  /// \brief Get the linear velocity.
  /// \return The linear velocity in meter/second.
  public: double LinearVelocity() const;

  /// \brief Get the angular velocity.
  /// \return The angular velocity in radian/second.
  public: const Angle &AngularVelocity() const;

  /// \brief Set the wheel parameters including the radius and separation.
  /// \param[in] _wheelSeparation Distance between left and right wheels.
  /// \param[in] _wheelBase Distance between front and back wheels.
  /// \param[in] _wheelRadius Radius of the wheel.
  public: void SetWheelParams(double _wheelSeparation,
                  double _wheelBase,
                  double _wheelRadius);

  /// \brief Set the velocity rolling window size.
  /// \param[in] _size The Velocity rolling window size.
  public: void SetVelocityRollingWindowSize(size_t _size);

  //private: std::unique_ptr<OmniDrivePrivate> dataPtr;
  /// \brief Private data pointer.
  //GZ_UTILS_IMPL_PTR(dataPtr)
};
// } // inline namespace GZ_MATH_VERSION_NAMESPACE

#endif