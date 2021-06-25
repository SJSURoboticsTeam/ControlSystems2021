#pragma once
#include "utility/math/units.hpp"
#include "devices/actuators/servo/rmd_x.hpp"
#include "devices/sensors/movement/accelerometer/mpu6050.hpp"

namespace sjsu::arm
{
// the Joint class is used for the rotunda, elbow, and shoulder motors.
class Joint
{
 private:
  float gearRatio = 6;
  // The minimum allowable angle the joint is able to turn to in normal
  // operation.
  units::angle::degree_t minimum_angle = 0_deg;
  // The maximum allowable angle the joint is able to turn to in normal
  // operation.
  units::angle::degree_t maximum_angle = 180_deg;
  // The angle the joint will move to when is not operational
  units::angle::degree_t rest_angle = 0_deg;
  // The angle between the motor's zero position and the actual homed zero
  // positon.
  units::angle::degree_t zero_offset_angle = 0_deg;
  // Motor object that controls the joint
  sjsu::RmdX & motor;
  // accelerometer attached to the joint that is used to home the arm
  sjsu::Mpu6050 & mpu;

 public:
  Joint(sjsu::RmdX & joint_motor, sjsu::Mpu6050 & accelerometer)
      : motor(joint_motor), mpu(accelerometer)
  {
  }

  Joint(sjsu::RmdX & joint_motor,
        sjsu::Mpu6050 & accelerometer,
        units::angle::degree_t min_angle,
        units::angle::degree_t max_angle,
        units::angle::degree_t standby_angle)
      : minimum_angle(min_angle),
        maximum_angle(max_angle),
        rest_angle(standby_angle),
        motor(joint_motor),
        mpu(accelerometer)
  {
  }

  /// Initialize the joint object, This must be called before any other
  /// function.
  void Initialize()
  {
    motor.Initialize();
    mpu.Initialize();
  }

  // Get the position of the motor from its encoder.
  units::angle::degree_t encoderPosition()
  {
    return static_cast<units::angle::degree_t>(motor.GetFeedback().encoder_position / 360.0f / 6);
  }

  /// Move the motor to the (calibrated) angle desired.
  void SetPosition(units::angle::degree_t angle)
  {
    angle =
        units::math::min(units::math::max(angle, minimum_angle), maximum_angle);
    units::angle::degree_t calibrated_angle = angle + zero_offset_angle;
    motor.SetAngle(calibrated_angle);
  }

  // Moves the joint to its park angle.
  void park()
  {
    SetPosition(rest_angle);
  }

  /// Sets the zero_offset_angle value that the motor uses to know its true '0'
  /// position. Called by RoverArmSystem::Home
  /// It is required to pass the pitch of the lower arm-segment to properly
  /// callibrate. (Rotunda->Elbow->Shoulder)
  units::angle::degree_t home(units::angle::degree_t previous_pitch)
  {
    // TODO: Rewrite these to take several samples of accelerometer data.
    units::angle::degree_t jointPitch = pitch();
    zero_offset_angle = jointPitch - encoderPosition() - previous_pitch;
    return jointPitch;
  }

  void SetZeroOffset(units::angle::degree_t offset)
  {
    zero_offset_angle = offset;
  }

  /// Return the acceleration values for the MPU6050 on the joint.
  sjsu::Accelerometer::Acceleration_t GetAccelerometerData()
  {
    return mpu.Read();
  }

  units::angle::degree_t pitch()
  {
    sjsu::Accelerometer::Acceleration_t acceleration = mpu.Read();
    return units::math::atan2(
        -acceleration.x / 1_SG,
        units::math::hypot(acceleration.y / 1_SG, acceleration.z / 1_SG));
  }
};
}  // namespace sjsu::arm
