#pragma once
#include "utility/math/units.hpp"
#include "joint.hpp"
#include "wrist_joint.hpp"

namespace sjsu::arm
{
class RoverArmSystem
{
 private:
  // isOperational defines if the arm should be allowed to move or if it should
  // be in its resting 'off' position.
  bool isOperational;

  sjsu::arm::Joint & Rotunda;
  sjsu::arm::Joint & Shoulder;
  sjsu::arm::Joint & Elbow;
  sjsu::arm::WristJoint & Wrist;

  // The target angle for each of the joints
  units::angle::degree_t shoulder_pos;
  units::angle::degree_t elbow_pos;
  units::angle::degree_t rotunda_pos;
  units::angle::degree_t wrist_roll_pos;
  units::angle::degree_t wrist_pitch_pos;

 public:
  // TODO: Remove constructor and keep Joints within the class.
  RoverArmSystem(sjsu::arm::Joint & rotunda,
                 sjsu::arm::Joint & shoulder,
                 sjsu::arm::Joint & elbow,
                 sjsu::arm::WristJoint wrist)
      : Rotunda(rotunda), Shoulder(shoulder), Elbow(elbow), Wrist(wrist)
  {
  }

  /// Homes all of the joints on the arm, so that the motors know their actual
  /// position. Returns true if successful.
  bool Home(bool park = false)
  {
    // Home Shoulder and Elbow
    Elbow.home(Shoulder.home(Rotunda.pitch()));
    // home wrist

    if (park)
    {  // It may be benifical to put the arm in its compact park postion, to
      // avoid crashing into thing while homing the rotunda.
      Shoulder.park();
      Elbow.park();
      Wrist.park();
    }

    // home rotunda
    while (true /*rotunda magno-switch not triggered*/)
    {
      // TODO improve search algorithm.
      Rotunda.SetPosition(Rotunda.encoderPosition() + 1_deg);
    }
    Rotunda.SetZeroOffset(Rotunda.encoderPosition());
    return true;
  }

  /// Initialize all of the arms joint objects, This must be called before any
  /// other function.
  void Initialize()
  {
    Rotunda.Initialize();
    Shoulder.Initialize();
    Elbow.Initialize();
    Wrist.Initialize();
  }

  /// Retrives all of information for arm movement from the Mission Control
  /// server. Returns True if successful.
  bool GetData()
  {
    // TODO: Acctually get the data from the server; Blocked by wifi code not
    // yet written.
    rotunda_pos     = 0_deg;
    shoulder_pos    = 0_deg;
    elbow_pos       = 0_deg;
    wrist_pitch_pos = 0_deg;
    wrist_roll_pos  = 0_deg;
    return true;
  }

  /// Moves each of the arm joints to the aproppriate angle
  /// Returns True if successful.
  bool MoveArm()
  {
    Rotunda.SetPosition(rotunda_pos);
    Shoulder.SetPosition(shoulder_pos);
    Elbow.SetPosition(elbow_pos);
    Wrist.SetPosition(wrist_pitch_pos, wrist_roll_pos);
    return true;
  }
};
}  // namespace sjsu::arm
