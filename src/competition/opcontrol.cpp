#include "competition/opcontrol.h"
#include "robot-config.h"
#include "tuning.h"

void tuning()
{
  // tune_odometry_wheel_diam();
  // tune_odometry_wheelbase();
  // tune_flywheel_ff();
  // tune_drive_ff_ks(TURN);
  // tune_drive_ff_kv(TURN, 0.12);
  // tune_drive_motion_maxv(TURN);
  // tune_drive_motion_accel(TURN, 700);
  // tune_drive_pid(TURN);
  
  // auto pos = odometry_sys.get_position();
  // main_controller.Screen.clearScreen();
  // main_controller.Screen.setCursor(0, 0);
  // main_controller.Screen.print("(%.3f, %.3f) : %.3f", pos.x, pos.y, pos.rot);
  // printf("X: %f, Y: %f, R: %f\n", pos.x, pos.y, pos.rot);
  // motion_t cur_motion = turn_fast_mprofile.get_motion();
  // double s = (cur_motion.pos != 0? 24 + cur_motion.pos : 0);
  // double s = cur_motion.pos + 180; //(cur_motion.pos != 0? 90 + cur_motion.pos : 0) + 90;
  // printf("%.2f %.2f %.2f %.2f %.2f %.2f %.2f\n", tmr.time(sec), s, pos.rot, fabs(cur_motion.vel), fabs(odometry_sys.get_angular_speed_deg()), cur_motion.accel, odometry_sys.get_angular_accel_deg());

  // if(main_controller.ButtonX.pressing())
  //   drive_sys.drive_tank(.11, -.11);
}

/**
 * Contains the main loop of the robot code while running in the driver-control period.
 */
void opcontrol()
{
  while(imu.isCalibrating()){}

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
  timer tmr;
  // Periodic
  while(true)
  {
    

    // ========== DRIVING CONTROLS ==========
    drive_sys.drive_tank(main_controller.Axis3.position()/100.0,main_controller.Axis2.position() / 100.0);
    // drive_sys.drive_arcade(main_controller.Axis3.position()/100.0, main_controller.Axis1.position()/100.0);
    
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