#include "L1_Peripheral/lpc40xx/pwm.hpp"
#include "L3_Application/task_scheduler.hpp"
#include "L2_HAL/boards/sjtwo.hpp"
#include "utility/log.hpp"
#include "utility/rtos.hpp"

template <class Target>
Target GetValue()
{
  return Targets{};
}

template <typename T>
class Targets
{
public:
  Targets();
  ~Targets();
  getTargetValues(){return Targets{}};
  getFirstTarget(){return firstTarget};
  getSecondTarget(){return secondTarget};

private:
  T firstTarget;
  T secondTarget;
  bool hasTarget;
}

main()
{
  return 0;
}
