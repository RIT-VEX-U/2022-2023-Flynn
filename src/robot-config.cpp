#include "robot-config.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;
controller main_controller;

// ======== OUTPUTS ========

motor indexer(PORT9);
motor intake(PORT10);

// ======== INPUTS ========
limit shoot_limit(Brain.ThreeWirePort.B);

//CustomEncoder lside(Brain.ThreeWirePort.E, 2048);
//CustomEncoder rside(Brain.ThreeWirePort.C, 2048);
//CustomEncoder offaxis(Brain.ThreeWirePort.A, 2048);

CustomEncoder rside(Brain.ThreeWirePort.A, 90);
CustomEncoder offaxis(Brain.ThreeWirePort.C, 90);
CustomEncoder lside(Brain.ThreeWirePort.E, 90);

// ======== SUBSYSTEMS ========
PID::pid_config_t flywheel_pid={
  .p = 0.00005,
  .i = 0.0000,
  .d = 0.0,
};

FeedForward::ff_config_t flywheel_ff={
  .kS = .01, // measured
  .kV = 0.00025, // tested
  .kA = 0.0, 
  .kG = 0.0, // no gravity - hopefully
};


motor lf(PORT1);
motor lm(PORT2); 
motor_group drive_left(lf, lm);

motor rf(PORT3);
motor rm(PORT4);

motor_group drive_right(rf, rm);



// WARNING: DUMMY VALUES, TO BE REPLACED
robot_specs_t specs = {
  .robot_radius = 7.5,
  .odom_wheel_diam = 2.75,
  .odom_gear_ratio = 1,
  .dist_between_wheels = 10,
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

Odometry3Wheel::odometry3wheel_cfg_t odometry3wheel_cfg = {
  .wheel_diam = 2.75,
  .wheelbase_dist = 10.0,
  .off_axis_center_dist = 4.67
};

Odometry3Wheel odometry3wheel(lside, rside, offaxis, odometry3wheel_cfg);

// OdometryTank odom(drive_left, drive_right, specs);
TankDrive drive_sys(drive_left, drive_right, specs);

// TODO: add Flywheel class
motor fw_top(PORT1);
motor fw_bot(PORT2, true);
motor_group fw_group(fw_top, fw_bot);
Flywheel flywheel(fw_group, flywheel_pid, flywheel_ff, 18);


// ======== UTILS ========

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {

}