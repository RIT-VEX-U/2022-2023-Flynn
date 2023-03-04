#include "competition/opcontrol.h"
#include "competition/autonomous_clu.h"
#include "automation.h"
#include "robot-config.h"
#include "tuning.h"
#include "vision.h"
#include <stdio.h>

int print_odom()
{
  while (true)
  {
    position_t pos = odometry_sys.get_position();
    printf("%.2f, %.2f, %.2f\n", pos.x, pos.y, pos.rot);
    vexDelay(100);
  }
  return 0;
}


/**
 * Contains the main loop of the robot code while running in the driver-control period.
 */
void opcontrol()
{
  vex::task odom_print(print_odom);

  // draw_image();
  // Brain.Screen.drawImageFromBuffer(&intense_milk[0], 0, 0, intense_milk_width , intense_milk_height);
  endgame_solenoid.set(false);
  flapup_solenoid.set(false);

  // Initialization
  double oneshot_time = .05; // Change 1 second to whatever is needed
  bool oneshotting = false;
  flywheel_sys.spinRPM(3200);

  main_controller.ButtonUp.pressed([]()
                                   { flywheel_sys.spinRPM(3400); });
  main_controller.ButtonLeft.pressed([]()
                                     { flywheel_sys.spinRPM(3100); });
  main_controller.ButtonRight.pressed([]()
                                      { flywheel_sys.spinRPM(3200); });

  main_controller.ButtonDown.pressed([]()
                                     { flywheel_sys.stop(); });

  main_controller.ButtonR1.pressed([]()
                                   { intake.spin(reverse, 12, volt); }); // Intake
  main_controller.ButtonR2.pressed([]()
                                   { intake.spin(fwd, 9.5, volt); }); // Shoot

  main_controller.ButtonL1.pressed([]()
                                   { flapup_solenoid.set(false); }); // Flapup
  main_controller.ButtonL2.pressed([]()
                                   { flapup_solenoid.set(true); }); // Flaodown

  main_controller.ButtonB.pressed([]()
                                  { odometry_sys.set_position(); });

  // odometry_sys.end_async();

  // Periodic
  while (true)
  {
    // ========== DRIVING CONTROLS ==========
    drive_sys.drive_tank(main_controller.Axis3.position() / 100.0, main_controller.Axis2.position() / 100.0);
    // ========== MANIPULATING CONTROLS ==========

    if (main_controller.ButtonY.pressing() && main_controller.ButtonRight.pressing())
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

    vexDelay(0);
  }
}