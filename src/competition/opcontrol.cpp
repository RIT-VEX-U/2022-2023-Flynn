#include "competition/opcontrol.h"
#include "robot-config.h"

/**
 * Contains the main loop of the robot code while running in the driver-control period.
 */
void opcontrol()
{
  // Initialization
  printf("starting\n");
  fflush(stdout);

  flywheel_sys.spin_manual(4000);//Change speed to what is needed
  main_controller.ButtonL2.pressed([](){intake.spinFor(1,timeUnits::sec);});//Change 1 second to whatever is needed
  // Periodic
  while(true)
  {
    // ========== DRIVING CONTROLS ==========
    drive_sys.drive_tank(main_controller.Axis3.position()/100.0,main_controller.Axis2.position() / 100.0);
    // ========== MANIPULATING CONTROLS ==========


    if(main_controller.ButtonR1.pressing()){
      intake.spin(reverse);
    }
    else if(main_controller.ButtonR2.pressing()){
      intake.spin(fwd);
    }
    else if(main_controller.ButtonL1.pressing()){
      roller.spin(fwd);
    }
    else if(main_controller.ButtonY.pressing() && main_controller.ButtonRight.pressing()){
      //TODO eject string
    }else{
      roller.stop();
      intake.stop();
    }

    // ========== SECONDARY REMOTE ==========

    // ========== AUTOMATION ==========    

    vexDelay(20);
  }
}