#include "competition/opcontrol.h"
#include "robot-config.h"
#include "tuning.h"

/**
 * Contains the main loop of the robot code while running in the driver-control period.
 */
void opcontrol()
{

  // Initialization
  printf("starting\n");
  fflush(stdout);
  double oneshot_time = .05;//Change 1 second to whatever is needed
  bool oneshotting = false;
  
  //flywheel_sys.spin_manual(3000);//TODO measure speed that is needed
  main_controller.ButtonR1.pressed([](){intake.spin(reverse, 12, volt);}); // Intake
  main_controller.ButtonR2.pressed([](){intake.spin(fwd, 12, volt);}); // Shoot
  main_controller.ButtonL2.pressed([](){intake.spin(fwd, 12, volt);oneshot_tmr.reset();}); //Single Shoot
  main_controller.ButtonL1.pressed([](){roller.spin(fwd);}); //Roller
  main_controller.ButtonUp.pressed([](){odometry_sys.set_position();});
  // Periodic
  while(true)
  {

    //tune_odometry_wheel_diam();
    //tune_odometry_wheelbase();
    tune_flywheel_ff();
    
    auto pos = odometry_sys.get_position();
    main_controller.Screen.clearScreen();
    main_controller.Screen.setCursor(0, 0);
    main_controller.Screen.print("(%.3f, %.3f) : %.3f", pos.x, pos.y, pos.rot);
    
    // ========== DRIVING CONTROLS ==========
    // drive_sys.drive_tank(main_controller.Axis3.position()/100.0,main_controller.Axis2.position() / 100.0);
    drive_sys.drive_arcade(main_controller.Axis3.position()/100.0, main_controller.Axis1.position()/100.0);
    
    // ========== MANIPULATING CONTROLS ==========


    if(main_controller.ButtonY.pressing() && main_controller.ButtonRight.pressing()){
      //TODO eject string
    }

    oneshotting = oneshot_tmr.time(vex::sec) < oneshot_time;
    if (!main_controller.ButtonR1.pressing() && !main_controller.ButtonR2.pressing() && !oneshotting){
      intake.stop();
    }

    // ========== SECONDARY REMOTE ==========

    // ========== AUTOMATION ==========    

    vexDelay(20);
  }
  
}