#include "competition/opcontrol.h"
#include "automation.h"
#include "robot-config.h"
#include "tuning.h"


int print_odom(){
    while(true){
        position_t pos = odometry_sys.get_position();
        printf("%.2f, %.2f, %.2f\n", pos.x, pos.y, pos.rot);
        vexDelay(20);
    }
    return 0;
}


void test_stuff(){


  //vex::task printodom(print_odom);

  // while(true){
  //   tune_flywheel_distcalc();
  //   vexDelay(50);
  // }

  while(imu.isCalibrating()){
    vexDelay(20);
  }

  auto DriveToPointFast = [](double x, double y){return new DriveToPointCommand(drive_sys, drive_fast_mprofile, x, y, fwd, 1.0);};
  auto TurnToHeading = [](double heading_deg){return new TurnToHeadingCommand(drive_sys, *config.turn_feedback, heading_deg, .6);};

  CommandController mine;
  mine.add(new PrintOdomCommand(odometry_sys));
  mine.add(DriveToPointFast(0, 10));
  mine.add(TurnToHeading(0));
  
  mine.add(new PrintOdomCommand(odometry_sys));  mine.run();
  printf("timedout %d\n", mine.last_command_timed_out());
  printf("finshed\n");
}

/**
 * Contains the main loop of the robot code while running in the driver-control period.
 */
void opcontrol()
{
  test_stuff();
  //return;
  printf("opcontrollin");
  // Initialization
  fflush(stdout);
  double oneshot_time = .05;//Change 1 second to whatever is needed
  bool oneshotting = false;
  //flywheel_sys.spinRPM(3000);
  //flywheel_sys.stop();
  //flywheel.spin(fwd, 12, volt);//TODO measure speed that is needed
  main_controller.ButtonUp.pressed([](){flywheel_sys.spinRPM(3000);});
  main_controller.ButtonDown.pressed([](){flywheel_sys.stop();});
  main_controller.ButtonR1.pressed([](){intake.spin(reverse, 12, volt);}); // Intake
  main_controller.ButtonR2.pressed([](){intake.spin(fwd, 12, volt);}); // Shoot
  main_controller.ButtonL2.pressed([](){intake.spin(fwd, 12, volt);oneshot_tmr.reset();}); //Single Shoot
  main_controller.ButtonL1.pressed([](){roller.spin(vex::reverse, 12, vex::volt);}); //Roller
  main_controller.ButtonL1.released([](){roller.stop();}); //Roller
  
  main_controller.ButtonB.pressed([](){odometry_sys.set_position();});

  odometry_sys.end_async();
  int i = 0;
  // Periodic
  while(true)
  {
    i++;
    if (i%5==0){
    main_controller.Screen.setCursor(0, 0);
    main_controller.Screen.clearScreen();
    main_controller.Screen.print("fw rpm: %f", flywheel_sys.getRPM());
    main_controller.Screen.setCursor(2, 0);
    main_controller.Screen.print("fw temp: %.1ff", flywheel.temperature(vex::fahrenheit));
    main_controller.Screen.setCursor(4, 0);
    main_controller.Screen.print("bat volt: %.2fv", Brain.Battery.voltage(vex::volt));
    
  }
    
    // ========== DRIVING CONTROLS ==========
    //drive_sys.drive_tank(main_controller.Axis3.position()/100.0,main_controller.Axis2.position() / 100.0);
    drive_sys.drive_arcade(main_controller.Axis2.position(pct)/200.0, main_controller.Axis4.position(pct)/100.0);    
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