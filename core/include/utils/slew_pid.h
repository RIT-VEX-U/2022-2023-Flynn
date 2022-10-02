#pragma once
#include "../core/include/utils/pid.h"

#include <cmath>
#include "../core/include/utils/math_util.h"
#include "vex.h"

using namespace vex;

class SlewPID: public PID
{
public:
  /**
   * Create the PID object
   */
  SlewPID(pid_config_t &config, double maximum_change);
  /**
     * Gets the current PID out value (what we want to send) and constrains it's change
     * can think of it as calculating the derivative between this 'frame' and last 'frame'
     * if the derivative is too great, slow down the acceleration by limitting the new value sent
  */
  double get();

private:
  double maximum_rate_of_change;
  double last_value_sent;
  
  double last_time;
  vex::timer slew_timer;
};