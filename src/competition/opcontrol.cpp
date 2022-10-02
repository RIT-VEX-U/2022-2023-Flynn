#include "competition/opcontrol.h"
#include "robot-config.h"

/**
 * Contains the main loop of the robot code while running in the driver-control period.
 */
void opcontrol()
{
  // Initialization


  // Periodic
  while(true)
  {
    
    // ========== DRIVING CONTROLS ==========
    double left_analog = main_controller.Axis3.position() / 100.0;
    double right_analog = main_controller.Axis2.position() / 100.0;

    drive_sys.drive_tank(left_analog, right_analog, 1, true);

    // ========== MANIPULATING CONTROLS ==========

    // ========== SECONDARY REMOTE ==========

    // ========== AUTOMATION ==========    


    // Wait 20 milliseconds for control loops to calculate time correctly
    vexDelay(20);
  }
}