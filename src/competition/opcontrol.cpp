#include "competition/opcontrol.h"
#include "C:/Users/richi/VEX/2022-2023-Flynn/include/robot-config.h"
#include <iostream>
/**
 * Contains the main loop of the robot code while running in the driver-control period.
 */


void opcontrol()
{
  // Initialization
  main_controller.ButtonA.pressed([](){printf("a pressed");});
  main_controller.ButtonA.released([](){printf("a released");});
  

  // Periodic
  while(true)
  {
    
    // ========== DRIVING CONTROLS ==========

    // ========== MANIPULATING CONTROLS ==========

    // ========== SECONDARY REMOTE ==========

    // ========== AUTOMATION ==========    

    // Wait 20 milliseconds for control loops to calculate time correctly
    vexDelay(20);
  }
}