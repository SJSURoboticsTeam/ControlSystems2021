#include "../Arm/pca9685.hpp"
#include "peripherals/lpc40xx/i2c.hpp"

int main(void)
{
  sjsu::lpc40xx::I2c & i2c = sjsu::lpc40xx::GetI2c<2>();
  sjsu::Pca9685 pca(i2c);
  pca.ModuleInitialize();
  sjsu::LogInfo("Starting program");


  while (true)
  {
    pca.setPulseWidth(0,500_us);
    sjsu::Delay(1.5_s);
    pca.setPulseWidth(0,2500_us);

    sjsu::Delay(1.5_s);
    sjsu::LogInfo("ran program");
  }
}
