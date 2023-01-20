#include "competition/opcontrol.h"
#include "robot-config.h"

/**
 * Contains the main loop of the robot code while running in the driver-control period.
 */
void opcontrol()
{
  // Initialization
  //odometry3wheel.tune(main_controller, drive_sys);
  printf("starting\n");fflush(stdout);
  main_controller.Screen.clearScreen();
  int i = 0;
  odometry3wheel.set_position();
  main_controller.ButtonR1.pressed([](){odometry3wheel.set_position({.x = 0, .y = 0, .rot = 90});});
  // Periodic
  while(true)
  {
    // drive_sys.drive_arcade(main_controller.Axis2.position()*.015, main_controller.Axis1.position()*.015);
    auto l = lside.position(rotationUnits::rev);
    auto r = rside.position(rotationUnits::rev);
    auto oa = offaxis.position(rotationUnits::rev);
    main_controller.Screen.setCursor(0, 0);
    // ========== DRIVING CONTROLS ==========

    // ========== MANIPULATING CONTROLS ==========

    // ========== SECONDARY REMOTE ==========

    // ========== AUTOMATION ==========    

    main_controller.Screen.setCursor(2, 0);
    auto pos = odometry3wheel.get_position();
    // printf("(%f, %f) r: %f\n", pos.x, pos.y, pos.rot);
    main_controller.Screen.print("(%f, %f)\n", pos.x, pos.y);
    main_controller.Screen.setCursor(4, 0);
    main_controller.Screen.print("r: %f\n", pos.rot);

    printf("Offax Dist: %f\n", offaxis.rotation(rev) * PI * 2.75);
    
    drive_sys.drive_arcade(main_controller.Axis3.position() / 100.0, main_controller.Axis1.position() / 100.0, 3);

    // Wait 20 milliseconds for control loops to calculate time correctly
    vexDelay(20);
  }
}