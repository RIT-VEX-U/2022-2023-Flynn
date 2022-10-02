#include "robot-config.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;
controller main_controller;

// ======== OUTPUTS ========

motor indexer(PORT9);
motor intake(PORT10);


// ======== INPUTS ========
inertial imu(PORT1);

// ======== SUBSYSTEMS ========

// motor rf(PORT7);
// motor rm(PORT6);
// motor rr(PORT8);
// motor_group drive_right(rf, rm, rr);

// motor lf(PORT3);
// motor lm(PORT5);
// motor lr(PORT4);
// motor_group drive_left(lf, lm, lr);

// WARNING: DUMMY VALUES, TO BE REPLACED
// robot_specs_t specs = {
//   .robot_radius = 7.5,
//   .odom_wheel_diam = 2.75,
//   .odom_gear_ratio = 1,
//   .dist_between_wheels = 8.5,
//   .drive_correction_cutoff = 7.0,

//   // Driving PID
//   .drive_pid={
//     .p = 0.1,
//     .i = 0.01,
//     .d = 0.01,
//     .deadband = 1.5,
//     .on_target_time = .1
//   },
//   // Turning PID
//   .turn_pid={
//     .p = 0.03, 
//     .i = 0.01,
//     .d = 0.002,
//     .deadband = 5.0,
//     .on_target_time = 0.1
//   },
//   .correction_pid={
//     .p=0.05,
//     .d=0.01
//   }
// };

// NEMO SPECS FOR DRIVE TESTING
motor lf_drive(PORT11, true), lr_drive(PORT12, true), rf_drive(PORT20), rr_drive(PORT19);

motor_group left_motors = {lf_drive, lr_drive};
motor_group right_motors = {rf_drive, rr_drive};

PID::pid_config_t pid_c = {
  .p = .1,
  .i = .001,
  .d = .008,
  .deadband = 0.3,
  .on_target_time = 0
};

PID pid(pid_c);

robot_specs_t robot_cfg = {
  .robot_radius = 12, // inches
  .odom_wheel_diam = 2.84, // inches
  .odom_gear_ratio = 1.03, // inches
  .dist_between_wheels = 9.18, // inches
  .drive_correction_cutoff = 12, //inches
  .drive_feedback = &pid
  /*(PID::pid_config_t) 
  {
    .p = .1,
    .i = .001,
    .d = .008,
    .deadband = 0.3,
    .on_target_time = 0
  },
  .turn_pid = (PID::pid_config_t)
  {
    .p = 0.025,
    .i = 0.01,
    .d = 0.0015,
    .deadband = 5,
    .on_target_time = 0.1
  },
  .correction_pid = (PID::pid_config_t)
  {
    .p = .01,
  }*/
};

// OdometryTank odom(drive_left, drive_right, robot_cfg);
// TankDrive drive_sys(drive_left, drive_right, robot_cfg);

OdometryTank odom(left_motors, right_motors, robot_cfg, &imu);
TankDrive drive_sys(left_motors, right_motors, robot_cfg, &odom);

// NEMO DRIVE TESTING


// TODO: add Flywheel class
motor fw_top(PORT1);
motor fw_bot(PORT2);


// ======== UTILS ========

// -------- EXAMPLE ONLY, DO NOT USE ------------

// -------- Default Drive Motion Profile --------
MotionController::m_profile_cfg_t mprof_drive_normal_cfg = 
{
    .max_v = 5,
    .accel = 2,
    .pid_cfg = (PID::pid_config_t) {
        .p = .1,
        .i = 0,
        .d = 0.01
    },
    .ff_cfg = (FeedForward::ff_config_t)
    {
        .kS = .05,
        .kV = .1,
        .kA = 0
    }
};

// -------- Default Turn Motion Profile --------
MotionController::m_profile_cfg_t mprof_turn_normal_cfg = 
{
    .max_v = 5,
    .accel = 2,
    .pid_cfg = (PID::pid_config_t) {
        .p = .1,
        .i = 0,
        .d = 0.01
    },
    .ff_cfg = (FeedForward::ff_config_t)
    {
        .kS = .05,
        .kV = .1,
        .kA = 0
    }
};

robot_specs_t config = {
    .drive_feedback = new MotionController(mprof_drive_normal_cfg),
    .turn_feedback = new MotionController(mprof_turn_normal_cfg)
    // .drive_feedback = new PID(),
    // .turn_feedback = new PID()

};

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {

}