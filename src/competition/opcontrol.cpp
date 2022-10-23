#include "competition/opcontrol.h"
#include "robot-config.h"

/**
 * Contains the main loop of the robot code while running in the driver-control period.
 */
void opcontrol()
{
  // Initialization
  /*
  main_controller.ButtonR1.pressed([](){shooter.Fire();});

  main_controller.ButtonR2.pressed([](){shooter.StartIntaking();});
  main_controller.ButtonR2.released([](){shooter.StopIntaking();});
  
  main_controller.ButtonL1.pressed([](){shooter.SpinAt(1000);});
  main_controller.ButtonUp.pressed([](){shooter.SpinAt(shooter.targetRPM()+100);});
  main_controller.ButtonDown.pressed([](){shooter.SpinAt(shooter.targetRPM()-100);});  
  main_controller.ButtonL1.released([](){shooter.StopSpinning();});

  
  main_controller.ButtonL2.pressed([](){shooter.applyPressure();});
  main_controller.ButtonL2.released([](){shooter.releasePressure();});
  

  main_controller.Screen.clearScreen();
  */
  // Periodic
  while(!shooter.indexer_switch_state()){
    indexer.spin(directionType::fwd, 3, voltageUnits::volt);
    vexDelay(10);
  }
  
  while(true)
  {
    // ========== DRIVING CONTROLS ==========
    if(main_controller.ButtonR2.pressing()){
      shooter.shootAllRPM(2500);
    }

    if(main_controller.ButtonL2.pressing()){
      shooter.stopSpinning();
    }

    
    // ========== MANIPULATING CONTROLS ==========

    // ========== SECONDARY REMOTE ==========

    // ========== AUTOMATION ==========

    // Wait 20 milliseconds for control loops to calculate time correctly
    vexDelay(20);
  }
}