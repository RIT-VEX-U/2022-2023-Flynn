#include "competition/opcontrol.h"
#include "competition/autonomous_clu.h"
#include "automation.h"
#include "robot-config.h"
#include "tuning.h"
#include "vision.h"
#include <stdio.h>
// #include "../core/include/intense_milk.h"

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

enum KnobSelection
{
  P,  // Programming
  C,  // opControl
  T1, // testing
  T2  // testing2
};

const double CTop = 193; // degrees
const double T1Top = 66; // degrees
const double T2Top = 27; // degrees

KnobSelection ProgramType;

KnobSelection GetSelection(vex::pot selector)
{
  double ang = selector.angle(deg);
  printf("ang; %f", ang);
  if (ang < T2Top)
  {
    return T2;
  }
  else if (ang < T1Top)
  {
    return T1;
  }
  else if (ang < CTop)
  {
    return C;
  }
  else
  {
    return P;
  }
}

/**
 * Contains the main loop of the robot code while running in the driver-control period.
 */
void opcontrol()
{
  vex::task odom_print(print_odom);
  // Stats
  StartScreen(Brain.Screen, {page_one, page_two, page_three});
  //// Knob
  // KnobSelection ProgramType = GetSelection(selector_pot);
  // printf("Program Type: %d: %f \n", ProgramType, selector_pot.angle(deg));
  // if (ProgramType == P)
  //{
  //   test_stuff();
  // }
  
 // test_stuff();

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