#include "competition/opcontrol.h"
#include "automation.h"
#include "robot-config.h"
#include "tuning.h"

/**
 * Contains the main loop of the robot code while running in the driver-control period.
 */
void opcontrol()
{

  // Initialization
  fflush(stdout);
  double oneshot_time = .05;//Change 1 second to whatever is needed
  bool oneshotting = false;
  flywheel_sys.stop();
  flywheel.spin(fwd, 12, volt);//TODO measure speed that is needed
  
  main_controller.ButtonR1.pressed([](){intake.spin(reverse, 12, volt);}); // Intake
  main_controller.ButtonR2.pressed([](){intake.spin(fwd, 12, volt);}); // Shoot
  main_controller.ButtonL2.pressed([](){intake.spin(fwd, 12, volt);oneshot_tmr.reset();}); //Single Shoot
  main_controller.ButtonL1.pressed([](){roller.spin(vex::reverse, 12, vex::volt);}); //Roller
  main_controller.ButtonL1.released([](){roller.stop();}); //Roller
  
  main_controller.ButtonB.pressed([](){odometry_sys.set_position();});

  odometry_sys.end_async();
  // Periodic
  while(true)
  {
    

    
    // ========== DRIVING CONTROLS ==========
    drive_sys.drive_tank(main_controller.Axis3.position()/100.0,main_controller.Axis2.position() / 100.0);
    
    // ========== MANIPULATING CONTROLS ==========


    if(main_controller.ButtonY.pressing() && main_controller.ButtonRight.pressing())
    {
      endgame_solenoid.set(true);
    }

    oneshotting = oneshot_tmr.time(vex::sec) < oneshot_time;
    if (!main_controller.ButtonR1.pressing() && !main_controller.ButtonR2.pressing() && !oneshotting)
    {
      intake.stop();
    }

    // ========== SECONDARY REMOTE ==========

    // ========== AUTOMATION ==========    

    vexDelay(20);
  }
}