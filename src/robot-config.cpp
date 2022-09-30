#include "robot-config.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;

// ======== OUTPUTS ========

motor indexer(PORT9);
motor intake(PORT10);


// ======== INPUTS ========

// ======== SUBSYSTEMS ========

motor rf(PORT7);
motor rm(PORT6);
motor rr(PORT8);
motor_group drive_right(rf, rm, rr);

motor lf(PORT3);
motor lm(PORT5);
motor lr(PORT4);
motor_group drive_left(lf, lm, lr);

// WARNING: DUMMY VALUES, TO BE REPLACED
robot_specs_t specs = {
  .robot_radius = 7.5,
  .odom_wheel_diam = 2.75,
  .odom_gear_ratio = 1,
  .dist_between_wheels = 8.5,
  .drive_correction_cutoff = 7.0,

  // Driving PID
  .drive_pid={
    .p = 0.1,
    .i = 0.01,
    .d = 0.01,
    .deadband = 1.5,
    .on_target_time = .1
  },
  // Turning PID
  .turn_pid={
    .p = 0.03, 
    .i = 0.01,
    .d = 0.002,
    .deadband = 5.0,
    .on_target_time = 0.1
  },
  .correction_pid={
    .p=0.05,
    .d=0.01
  }
};

OdometryTank odom(drive_left, drive_right, specs);
TankDrive drive_sys(drive_left, drive_right, specs);

// TODO: add Flywheel class
motor fw_top(PORT1);
motor fw_bot(PORT2);


// ======== UTILS ========

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {

}