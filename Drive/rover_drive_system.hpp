#pragma once

#include "utility/log.hpp"
#include "utility/time.hpp"
#include "utility/units.hpp"
#include "utility/map.hpp"
#include "wheel.hpp"

namespace sjsu::drive
{
class RoverDriveSystem
{
 public:
  /// Rover drive modes
  enum class Mode : char
  {
    kDrive       = 'D',
    kSpin        = 'S',
    kTranslation = 'T'
  };

  // MAKE CONST STRUCT INSTEAD OF DELCARING WITHIN FUNCTION

  /// Mission controls possible input values
  struct MissionControlData
  {
    bool is_operational;
    char drive_mode;
    float rotation_angle;
    float speed;
    char * GET_request;
  };

  RoverDriveSystem(sjsu::drive::Wheel & left_wheel,
                   sjsu::drive::Wheel & right_wheel,
                   sjsu::drive::Wheel & back_wheel)
      : left_wheel_(left_wheel),
        right_wheel_(right_wheel),
        back_wheel_(back_wheel){};

  void Initialize()
  {
    left_wheel_.Initialize();
    right_wheel_.Initialize();
    back_wheel_.Initialize();
  };

  void Enable(bool enable = true)
  {
    mission_control_data_.is_operational = true;
    left_wheel_.Enable(enable);
    right_wheel_.Enable(enable);
    back_wheel_.Enable(enable);
    SetMode();
  }

  // Handles GET /drive?parameters for rover drive system to mission control
  /// @return returns true if connection is established from mission control
  bool ExchangeMissionControlData()
  {
    // TODO: GET /drive?key=value&key=value... JSON value.
    if (SendGETRequest())
    {
      char response[] = {};
      ParseMissionControlData(response);
      return true;
    }
    else
    {
      SetWheelSpeed(kZeroSpeed);
      sjsu::LogError("Unable to reach mission control server");
      return false;
    }
  }

  /// Main function for handling all the rover drive system functionality.
  /// drive_mode updates drive mode if new mode is different.
  /// rotation_angle adjusts wheel position depending on mode.
  /// speed adjusts the movement speed of the rover.
  void Move()
  {
    if (mission_control_data_.is_operational)
    {
      if (mission_control_data_.drive_mode != static_cast<char>(current_mode_))
      {
        SetMode(mission_control_data_.drive_mode);
      }
      else
      {
        HandleRoverMovement(mission_control_data_.rotation_angle,
                            mission_control_data_.speed);
      }
    }
  };

  /// Resets all the wheels so the motors know their actual position.
  /// @return true if successfully resets wheels into start position
  bool Reset()
  {
    SetMode();
    return true;
  };

 private:
  /// Sets all wheels to the speed provided. Wheel class handles max/min speeds
  /// @param speed the new movement speed of the rover
  void SetWheelSpeed(units::angular_velocity::revolutions_per_minute_t speed)
  {
    // TODO: Implement linear interploation (exponentional moving average) to
    // smooth out changes in speed.
    left_wheel_.SetHubSpeed(speed);
    right_wheel_.SetHubSpeed(speed);
    back_wheel_.SetHubSpeed(speed);
  };

  /// Updates Mission Control /drive/status endpoint with rover's current status
  bool SendGETRequest()
  {
    if (GetRoverData())
    {
      // TODO: Add state of charge parameter
      mission_control_data_.GET_request =
          "192.168.1.153:5000/"
          "?is_operational=%d&drive_mode=%c&left_wheel_speed=%f&right_wheel_"
          "speed=%f&back_wheel_speed=%f",
      mission_control_data_.is_operational, static_cast<char>(current_mode_),
      left_wheel_.GetSpeed(), right_wheel_.GetSpeed(), back_wheel_.GetSpeed();
      sjsu::LogInfo("GET request: %s", mission_control_data_.GET_request);
      return true;
    }
    else
    {
      sjsu::LogError("Unable to retrieve wheel data!");
      return false;
    }
  };

  /// Gets the speed and position/angle of each wheel on the rover
  /// @return true if rover is able to retrieve data from all the wheels
  bool GetRoverData()
  {
    try
    {
      sjsu::LogInfo("is_operational: %d", mission_control_data_.is_operational);
      sjsu::LogInfo("drive_mode: %c", static_cast<char>(current_mode_));
      sjsu::LogInfo("left wheel speed: %f", left_wheel_.GetSpeed());
      sjsu::LogInfo("left wheel position: %f", left_wheel_.GetPosition());
      sjsu::LogInfo("right wheel speed: %f", right_wheel_.GetSpeed());
      sjsu::LogInfo("right wheel position: %f", right_wheel_.GetPosition());
      sjsu::LogInfo("back wheel speed: %f", back_wheel_.GetSpeed());
      sjsu::LogInfo("back wheel position: %f", back_wheel_.GetPosition());
      return true;
    }
    catch (const std::exception & e)
    {
      return false;
    }
  };

  /// Parses incoming data from mission control to command rover
  /// @param response incoming JSON / string data from mission control
  void ParseMissionControlData(char * response)
  {
    // TODO: parse data using sscanf. Refer to GitHub Issue #152 for solution
    sjsu::LogInfo("MISSION CONTROL RESPONSE:\n%s", response);
    // hard coded for now
    mission_control_data_.is_operational = true;
    mission_control_data_.drive_mode     = 'S';
    mission_control_data_.rotation_angle = 10.0f;
    mission_control_data_.speed          = 30.0f;
  };

  /// Sets the new driving mode for the rover. Rover will stop before switching
  /// @param mode Three Modes: D (drive), S (spin), T (translation)
  void SetMode(char mode = 'S')
  {
    switch (mode)
    {
      case 'D':
        current_mode_ = Mode::kDrive;
        SetWheelSpeed(kZeroSpeed);
        SetDriveMode();
        sjsu::LogInfo("Drive mode set");
        break;
      case 'S':
        current_mode_ = Mode::kSpin;
        SetWheelSpeed(kZeroSpeed);
        SetSpinMode();
        sjsu::LogInfo("Spin mode set");
        break;
      case 'T':
        current_mode_ = Mode::kTranslation;
        SetWheelSpeed(kZeroSpeed);
        SetTranslationMode();
        sjsu::LogInfo("Translation mode set");
        break;
      default:
        SetWheelSpeed(kZeroSpeed);
        sjsu::LogError("Unable to assign drive mode!");
    };
  };

  /// Handles the rover movement depending on the mode
  /// @param rotation_angle adjusts wheel position depending on mode.
  /// @param wheel_speed adjusts the movement speed of the rover.
  void HandleRoverMovement(float roatation_angle, float wheel_speed)
  {
    units::angle::degree_t angle(roatation_angle);
    units::angular_velocity::revolutions_per_minute_t speed(wheel_speed);

    switch (current_mode_)
    {
      case Mode::kDrive:
        sjsu::LogInfo("Driving...");
        HandleDriveMode(speed, angle);
        break;
      case Mode::kSpin:
        sjsu::LogInfo("Spining...");
        HandleSpinMode(speed);
        break;
      case Mode::kTranslation:
        sjsu::LogInfo("Translating...");
        HandleTranslationMode(speed, angle);
        break;
      default:
        SetWheelSpeed(kZeroSpeed);
        sjsu::LogError("Unable to assign drive mode handler!");
        break;
    }
    current_speed_ = speed;
  };

  // ======================
  // = DRIVE MODE SETTERS =
  // ======================

  /// Aligns rover wheels all in the same direction, facing foward
  void SetDriveMode()
  {
    const units::angle::degree_t left_wheel_angle  = -45_deg;
    const units::angle::degree_t right_wheel_angle = -135_deg;
    const units::angle::degree_t back_wheel_angle  = 90_deg;
    SetSpinMode();
    left_wheel_.SetSteeringAngle(left_wheel_angle);
    right_wheel_.SetSteeringAngle(right_wheel_angle);
    back_wheel_.SetSteeringAngle(back_wheel_angle);
  };

  /// Aligns rover wheels perpendicular to their legs using their homing mark
  void SetSpinMode()
  {
    left_wheel_.HomeWheel();
    right_wheel_.HomeWheel();
    back_wheel_.HomeWheel();
  };

  /// Aligns rover wheel all in the same direction, facing towards the right
  void SetTranslationMode()
  {
    const units::angle::degree_t left_wheel_angle  = 45_deg;
    const units::angle::degree_t right_wheel_angle = -45_deg;
    const units::angle::degree_t back_wheel_angle  = -180_deg;
    SetSpinMode();
    left_wheel_.SetSteeringAngle(left_wheel_angle);
    right_wheel_.SetSteeringAngle(right_wheel_angle);
    back_wheel_.SetSteeringAngle(back_wheel_angle);
  };

  // =======================
  // = DRIVE MODE HANDLERS =
  // =======================

  /// Handles drive mode. Adjusts only the rear wheel of the rover
  void HandleDriveMode(units::angular_velocity::revolutions_per_minute_t speed,
                       units::angle::degree_t angle)
  {
    back_wheel_.SetSteeringAngle(angle);
    SetWheelSpeed(speed);
  };

  /// Handles spin mode. Adjusts only the speed (aka the spin direction)
  void HandleSpinMode(units::angular_velocity::revolutions_per_minute_t speed)
  {
    SetWheelSpeed(speed);
  };

  /// Handles translation mode. Adjusts all the wheels, keeping them parallel
  void HandleTranslationMode(
      units::angular_velocity::revolutions_per_minute_t speed,
      units::angle::degree_t angle)
  {
    left_wheel_.SetSteeringAngle(angle);
    right_wheel_.SetSteeringAngle(angle);
    back_wheel_.SetSteeringAngle(angle);
    SetWheelSpeed(speed);
  };

  sjsu::drive::Wheel & left_wheel_;
  sjsu::drive::Wheel & right_wheel_;
  sjsu::drive::Wheel & back_wheel_;
  MissionControlData mission_control_data_;
  sjsu::drive::RoverDriveSystem::Mode current_mode_ = Mode::kSpin;
  units::angular_velocity::revolutions_per_minute_t current_speed_   = 0_rpm;
  const units::angular_velocity::revolutions_per_minute_t kZeroSpeed = 0_rpm;
};
}  // namespace sjsu::drive
