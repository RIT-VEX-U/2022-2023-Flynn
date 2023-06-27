#include "competition/opcontrol.h"
#include "competition/autonomous_flynn.h"
#include "robot-config.h"
#include "vex_global.h"
#include "vision.h"
#include <algorithm>
#include <experimental/optional>
#include <stdio.h>

template <typename Iter, typename Predicate>
std::experimental::optional<typename Iter::value_type>
find_if_option(Iter iter, Predicate predicate)
{
  auto res = std::find_if(iter.begin(), iter.end(), predicate);
  if (res == iter.end()) {
    return {};
  }
  return std::experimental::optional<typename Iter::value_type>(*res);
}

int controller_screen()
{
  static std::string happy = ":)";
  while (1)
  {
    uint32_t bat = (uint32_t)Brain.Battery.capacity();
    double f = flywheel_motors.temperature(celsius);
    //    double i = intake.temperature(celsius);
    //    double d = (left_motors.temperature(celsius) +
    //    right_motors.temperature(celsius)) / 2;

    main_controller.Screen.setCursor(1, 1);
    main_controller.Screen.print("F: %.0f        B: %d%%", f, bat);

    main_controller.Screen.setCursor(2, 1);
    main_controller.Screen.print("I: %.0f        V: %.1fv",
                                 intake.temperature(celsius),
                                 Brain.Battery.voltage());
    main_controller.Screen.setCursor(3, 1);

    std::experimental::optional problem_motor
        = find_if_option(motor_names, [](auto name_and_motor) {
            return !name_and_motor.second.installed();
          });

    std::string &problem_str = happy;
    if (problem_motor) {
      problem_str = problem_motor->first;
    }
    main_controller.Screen.print("D: %.0f      %s", intake.temperature(celsius),
                                 problem_motor->first.c_str());

    vexDelay(500);
  }
}
/**
 * Contains the main loop of the robot code while running in the driver-control period.
 */
void opcontrol()
{
  // test_stuff();
  vex::thread controller_screen_thread(controller_screen);

  endgame_solenoid.set(false);
  flapup_solenoid.set(false);
  intake_solenoid.set(false);
  // Initialization
  double oneshot_time = .05; // Change 1 second to whatever is needed
  const static double RPM1 = 2300;
  const static double RPM2 = 2700;
  const static double RPM3= 3400;

  flywheel_sys.spinRPM(RPM2);

  main_controller.ButtonUp.pressed([]()
                                   { flywheel_sys.spinRPM(RPM3); });
  main_controller.ButtonLeft.pressed([]()
                                     { flywheel_sys.spinRPM(RPM2); });
  main_controller.ButtonRight.pressed([]()
                                      { flywheel_sys.spinRPM(RPM1); });

  main_controller.ButtonDown.pressed([]()
                                     { flywheel_sys.stop(); });

  main_controller.ButtonR1.pressed(
      []() { intake.spin(vex::directionType::rev, 12, volt); }); // Intake
  main_controller.ButtonR2.pressed([]()
                                   { intake.spin(fwd, 9.5, volt); }); // Shoot

  main_controller.ButtonL1.pressed([]()
                                   { flapup_solenoid.set(true); }); // Flapup
  main_controller.ButtonL1.released([]()
                                   { flapup_solenoid.set(false); }); // Flapdown

  main_controller.ButtonL2.pressed([]()
                                   { intake_solenoid.set(true); }); // Raisined
  main_controller.ButtonL2.released([]()
                                   { intake_solenoid.set(false); }); // Lowened

  main_controller.ButtonB.pressed([]()
                                  { odometry_sys.set_position(); });

  odometry_sys.set_position();
  // PeriodicA
  while (true)
  {
    // printf("%f : %f\n", left_enc.position(rotationUnits::rev), right_enc.position(rotationUnits::rev));
    // ========== DRIVING CONTROLS ==========
    drive_sys.drive_tank(main_controller.Axis3.position() / 100.0, main_controller.Axis2.position() / 100.0);
    // ========== MANIPULATING CONTROLS ==========


    if (main_controller.ButtonY.pressing() && main_controller.ButtonRight.pressing())
    {
      endgame_solenoid.set(true);
    }

    bool oneshotting = oneshot_tmr.time(vex::sec) < oneshot_time;
    if (!main_controller.ButtonR1.pressing() && !main_controller.ButtonR2.pressing() && !oneshotting)
    {
      intake.stop();
    }

    // ========== SECONDARY REMOTE ==========

    // ========== AUTOMATION ==========

    vexDelay(10);
  }
}
