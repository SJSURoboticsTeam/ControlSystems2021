#include "utility/log.hpp"
#include "RoverArmSystem.hpp"
#include "peripherals/lpc40xx/i2c.hpp"
#include "peripherals/lpc40xx/can.hpp"
#include "devices/actuators/servo/rmd_x.hpp"
#include "devices/sensors/movement/accelerometer/mpu6050.hpp"

int main()
{
  // Create a can bus for the RMD_x7 motors to use and all of the RMD_x7
  // motor objects for the arm.
  sjsu::lpc40xx::Can & can = sjsu::lpc40xx::GetCan<2>();
  sjsu::StaticMemoryResource<1024> memory_resource;
  sjsu::CanNetwork can_network(can, &memory_resource);
  sjsu::RmdX rmd_rotunda(can_network, 0x148);
  sjsu::RmdX rmd_shoulder(can_network, 0x149);
  sjsu::RmdX rmd_elbow(can_network, 0x14A);
  sjsu::RmdX rmd_left_wrist(can_network, 0x14B);
  sjsu::RmdX rmd_right_wrist(can_network, 0x14C);

  // Create an I2C object for the IMU's as well as all of the MPU objects for
  // the arm.
  //
  // Note that this only works if an address translator is used for the
  // MPU6050s; as they can only be set to use two addresses. If the chip
  // select
  // method is used then they would all be set to the same address and
  // pulling
  // the address pin HIGH/LOW would select the correct MPU to access.
  sjsu::lpc40xx::I2c & i2c = sjsu::lpc40xx::GetI2c<2>();
  sjsu::Mpu6050 mpu_rotunda(i2c, 0x68);
  sjsu::Mpu6050 mpu_shoulder(i2c, 0x69);
  sjsu::Mpu6050 mpu_elbow(i2c, 0x6A);
  sjsu::Mpu6050 mpu_wrist(i2c, 0x6B);

  // Attach the RMD_x7 motor objects and the MPUs to the appropriate arm
  // joint.
  sjsu::arm::Joint rotunda(rmd_rotunda, mpu_rotunda, 0_deg, 3600_deg, 1800_deg);
  sjsu::arm::Joint shoulder(rmd_shoulder, mpu_shoulder);
  sjsu::arm::Joint elbow(rmd_elbow, mpu_elbow);
  sjsu::arm::WristJoint wrist(rmd_left_wrist, rmd_right_wrist, mpu_wrist);

  // Attach the Joins to the arm controller object.
  sjsu::arm::RoverArmSystem armControl(rotunda, shoulder, elbow, wrist);
  armControl.Initialize();
  armControl.Home();

  while (true)
  {
    armControl.GetData();
    armControl.MoveArm();
    sjsu::Delay(100ms);
  }
}
