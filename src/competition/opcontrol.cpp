#include "competition/opcontrol.h"
#include "robot-config.h"

/**
 * Contains the main loop of the robot code while running in the driver-control period.
 */
void opcontrol()
{
  // Initialization
  main_controller.ButtonR1.pressed([](){shooter.Fire();});
  main_controller.ButtonR2.pressed([](){shooter.StartIntaking();});
  main_controller.ButtonR2.released([](){shooter.StopIntaking();});
  
  main_controller.ButtonL1.pressed([](){shooter.SpinAt(2600);});
  main_controller.ButtonL1.released([](){shooter.StopSpinning();});
  
  main_controller.ButtonUp.pressed([](){shooter.SpinAt(shooter.targetRPM()+100);});
  main_controller.ButtonDown.pressed([](){shooter.SpinAt(shooter.targetRPM()-100);});
  
  main_controller.ButtonB.pressed([](){shooter.stop_firing();});

  // Periodic
  while(true)
  {
    main_controller.Screen.setCursor(0, 0);
    main_controller.Screen.print("SP: %f", shooter.targetRPM());
    // ========== DRIVING CONTROLS ==========

    // ========== MANIPULATING CONTROLS ==========

    // ========== SECONDARY REMOTE ==========

    // ========== AUTOMATION ==========    


    // Wait 20 milliseconds for control loops to calculate time correctly
    vexDelay(20);
  }
}