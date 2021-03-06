#pragma once

#include <stdio.h>

#include "utility/log.hpp"
#include "utility/time/time.hpp"
#include "utility/math/units.hpp"
#include "utility/math/map.hpp"

#include "../Common/esp.hpp"
#include "wheel.hpp"

namespace sjsu::drive
{
class RoverDriveSystem
{
 public:
  struct MissionControlData
  {
    int is_operational;
    char drive_mode;
    float rotation_angle;
    float speed;
  };

  RoverDriveSystem(Wheel & left_wheel, Wheel & right_wheel, Wheel & back_wheel)
      : left_wheel_(left_wheel),
        right_wheel_(right_wheel),
        back_wheel_(back_wheel){};

  char GetCurrentMode()
  {
    return current_mode_;
  }

  /// Initializes wheels and sets rover to operational starting mode (spin)
  void Initialize()
  {
    try
    {
      mc_data.is_operational = true;
      left_wheel_.Initialize();
      right_wheel_.Initialize();
      back_wheel_.Initialize();
      HomeWheels();
    }
    catch (const std::exception & e)
    {
      sjsu::LogError("Error initializing!");
      throw e;
    }
  };

  /// Constructs GET request parameter
  /// @return requestParameters endpoint & parameters i.e. /drive?ex=param
  std::string CreateRequestParameters()
  {
    try
    {
      // TODO - make these floats go to hundredths place (i.e. 0.00)?
      // Breaks unit test often since it never knows correct decimal value
      char reqParam[250];
      snprintf(reqParam, 300,
               "Vishnu-Adda/json-robo-test/"
               "drive?is_operational=%d&drive_mode=%c&battery=%d&left_wheel_"
               "speed=%4g&left_wheel_angle=%4g&right_wheel_speed=%4g&right_"
               "wheel_angle=%4g&back_wheel_speed=%4g&back_wheel_angle=%4g",
               mc_data.is_operational, current_mode_, state_of_charge_,
               left_wheel_.GetSpeed(), left_wheel_.GetPosition(),
               right_wheel_.GetSpeed(), right_wheel_.GetPosition(),
               back_wheel_.GetSpeed(), back_wheel_.GetPosition());
      std::string requestParameter = reqParam;
      return requestParameter;
    }
    catch (const std::exception & e)
    {
      sjsu::LogError("Error constructing GET request parameter!");
      throw e;
    }
  };

  /// Parses GET response body and assigns it to rover variables
  /// @param response JSON response body
  void ParseJSONResponse(std::string_view response)
  {
    try
    {
      sscanf(
          reinterpret_cast<const char *>(response.data()),
          R"({ "is_operational": %d, "drive_mode": "%c", "speed": %f, "angle": %f }\n)",
          &mc_data.is_operational, &mc_data.drive_mode, &mc_data.speed,
          &mc_data.rotation_angle);

      sjsu::LogInfo("is_operational: %d", mc_data.is_operational);
      sjsu::LogInfo("drive_mode: %c", mc_data.drive_mode);
      sjsu::LogInfo("speed: %f", mc_data.speed);
      sjsu::LogInfo("rotation_angle: %f", mc_data.rotation_angle);
    }
    catch (const std::exception & e)
    {
      sjsu::LogError("Error parsing GET response!");
      throw e;
    }
  };

  /// Handles the rover movement depending on the mode.
  /// D = Drive, S = Spin, T = Translation
  void HandleRoverMovement()
  {
    try
    {
      // sjsu::LogInfo("is_operational: %d", mc_data.is_operational);
      // sjsu::LogInfo("drive_mode: %c", mc_data.drive_mode);
      // sjsu::LogInfo("speed: %f", mc_data.speed);
      // sjsu::LogInfo("angle: %f", mc_data.rotation_angle);

      units::angle::degree_t angle(mc_data.rotation_angle);
      units::angular_velocity::revolutions_per_minute_t speed(mc_data.speed);
      // If current mode is same as mc mode value and rover is operational
      if (mc_data.is_operational && (current_mode_ == mc_data.drive_mode))
      {
        sjsu::LogInfo("Handling %c movement...", current_mode_);
        switch (current_mode_)
        {
          case 'D': HandleDriveMode(speed, angle); break;
          case 'S': HandleSpinMode(speed); break;
          case 'T': HandleTranslationMode(speed, angle); break;
          default:
            SetWheelSpeed(kZeroSpeed);
            sjsu::LogError("Unable to assign drive mode handler!");
            break;
        }
      }
      else
      {
        // If current mode is not same as mc mode value
        sjsu::LogInfo("Switching rover into %c mode...", mc_data.drive_mode);
        SetMode();
      }
    }
    catch (const std::exception & e)
    {
      sjsu::LogError("Error handling movement!");
      throw e;
    }
  };

  /// HomeWheels all the wheels so the motors know their actual position.
  /// @return true if successfully moves wheels into home position
  void HomeWheels()
  {
    try
    {
      SetWheelSpeed(kZeroSpeed);
      left_wheel_.HomeWheel();
      right_wheel_.HomeWheel();
      back_wheel_.HomeWheel();
    }
    catch (const std::exception & e)
    {
      sjsu::LogError("Error homing the wheels!");
      throw e;
    }
  };

  /// Sets all wheels to the speed provided. Wheel class handles max/min speeds
  /// @param speed the new movement speed of the rover
  void SetWheelSpeed(units::angular_velocity::revolutions_per_minute_t speed)
  {
    // TODO: Implement linear interpolation (exponential moving average) to
    // smooth out changes in speed.
    try
    {
      left_wheel_.SetHubSpeed(speed);
      right_wheel_.SetHubSpeed(speed);
      back_wheel_.SetHubSpeed(speed);
    }
    catch (const std::exception & e)
    {
      sjsu::LogError("Error setting wheels speed!");
      throw e;
    }
  };

  /// Prints the speed and position/angle of each wheel on the rover
  void PrintRoverData()
  {
    sjsu::LogInfo("is_operational: %d", mc_data.is_operational);
    sjsu::LogInfo("drive_mode: %c", current_mode_);
    sjsu::LogInfo("state of charge: %d", state_of_charge_);
    sjsu::LogInfo("left wheel speed: %g", left_wheel_.GetSpeed());
    sjsu::LogInfo("left wheel position: %g", left_wheel_.GetPosition());
    sjsu::LogInfo("right wheel speed: %g", right_wheel_.GetSpeed());
    sjsu::LogInfo("right wheel position: %g", right_wheel_.GetPosition());
    sjsu::LogInfo("back wheel speed: %g", back_wheel_.GetSpeed());
    sjsu::LogInfo("back wheel position: %g", back_wheel_.GetPosition());
  };

 private:
  /// Stops the rover and sets a new mode.
  void SetMode()
  {
    try
    {
      SetWheelSpeed(kZeroSpeed);  // Stops rover
      // TODO - Add a 1 second delay?
      switch (mc_data.drive_mode)
      {
        case 'D': SetDriveMode(); break;
        case 'S': SetSpinMode(); break;
        case 'T': SetTranslationMode(); break;
        default: sjsu::LogError("Unable to set drive mode!");
      };
    }
    catch (const std::exception & e)
    {
      sjsu::LogError("Error setting drive mode!");
      throw e;
    }
  };

  // ======================
  // = DRIVE MODE SETTERS =
  // ======================

  /// Aligns rover wheels all in the same direction, facing forward
  void SetDriveMode()
  {
    try
    {
      HomeWheels();
      const units::angle::degree_t left_wheel_angle  = -45_deg;
      const units::angle::degree_t right_wheel_angle = -135_deg;
      const units::angle::degree_t back_wheel_angle  = 90_deg;
      left_wheel_.SetSteeringAngle(left_wheel_angle);
      right_wheel_.SetSteeringAngle(right_wheel_angle);
      back_wheel_.SetSteeringAngle(back_wheel_angle);
      current_mode_ = 'D';
    }
    catch (const std::exception & e)
    {
      sjsu::LogError("Error setting drive mode!");
      throw e;
    }
  };

  /// Aligns rover wheels perpendicular to their legs using homing slip ring
  void SetSpinMode()
  {
    try
    {
      HomeWheels();
      const units::angle::degree_t left_wheel_angle  = 0_deg;
      const units::angle::degree_t right_wheel_angle = 0_deg;
      const units::angle::degree_t back_wheel_angle  = 0_deg;
      left_wheel_.SetSteeringAngle(left_wheel_angle);
      right_wheel_.SetSteeringAngle(right_wheel_angle);
      back_wheel_.SetSteeringAngle(back_wheel_angle);
      current_mode_ = 'S';
    }
    catch (const std::exception & e)
    {
      sjsu::LogError("Error setting spin mode!");
      throw e;
    }
  };

  /// Aligns rover wheel all in the same direction, facing towards the right
  void SetTranslationMode()
  {
    try
    {
      HomeWheels();
      const units::angle::degree_t left_wheel_angle  = 45_deg;
      const units::angle::degree_t right_wheel_angle = -45_deg;
      const units::angle::degree_t back_wheel_angle  = -180_deg;
      left_wheel_.SetSteeringAngle(left_wheel_angle);
      right_wheel_.SetSteeringAngle(right_wheel_angle);
      back_wheel_.SetSteeringAngle(back_wheel_angle);
      current_mode_ = 'T';
    }
    catch (const std::exception & e)
    {
      sjsu::LogError("Error setting translation mode!");
      throw e;
    }
  };

  // =======================
  // = DRIVE MODE HANDLERS =
  // =======================

  /// Handles drive mode. Adjusts only the rear wheel of the rover
  void HandleDriveMode(units::angular_velocity::revolutions_per_minute_t speed,
                       units::angle::degree_t angle)
  {
    try
    {
      back_wheel_.SetSteeringAngle(angle);
      SetWheelSpeed(speed);
    }
    catch (const std::exception & e)
    {
      sjsu::LogError("Error handling drive mode!");
      throw e;
    }
  };

  /// Handles spin mode. Adjusts only the speed (aka the spin direction)
  void HandleSpinMode(units::angular_velocity::revolutions_per_minute_t speed)
  {
    try
    {
      SetWheelSpeed(speed);
    }
    catch (const std::exception & e)
    {
      sjsu::LogError("Error handling spin mode!");
      throw e;
    }
  };

  /// Handles translation mode. Adjusts all the wheels, keeping them parallel
  void HandleTranslationMode(
      units::angular_velocity::revolutions_per_minute_t speed,
      units::angle::degree_t angle)
  {
    try
    {
      left_wheel_.SetSteeringAngle(angle);
      right_wheel_.SetSteeringAngle(angle);
      back_wheel_.SetSteeringAngle(angle);
      SetWheelSpeed(speed);
    }
    catch (const std::exception & e)
    {
      sjsu::LogError("Error handling translation mode!");
      throw e;
    }
  };

  char current_mode_   = 'S';
  int state_of_charge_ = 90;  // TODO - hardcoded for now

  const units::angular_velocity::revolutions_per_minute_t kZeroSpeed = 0_rpm;

 public:
  MissionControlData mc_data;
  Wheel & left_wheel_;
  Wheel & right_wheel_;
  Wheel & back_wheel_;
};
}  // namespace sjsu::drive