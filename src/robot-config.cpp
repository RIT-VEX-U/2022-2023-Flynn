#include "robot-config.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;
controller main_controller;

// ======== OUTPUTS ========

// ======== INPUTS ========

// ======== SUBSYSTEMS ========

// ======== UTILS ========

// -------- EXAMPLE ONLY, DO NOT USE ------------

// -------- Default Drive Motion Profile --------
MotionController::m_profile_cfg_t mprof_drive_normal_cfg = 
{
    .max_v =  55,
    .accel = 140,
    .pid_cfg = (PID::pid_config_t) {
        .p = .35,
        .i = 0,
        .d = 0.0001,
        .deadband = .1,
    },
    .ff_cfg = (FeedForward::ff_config_t)
    {
        .kS = 0.061000,//1000,
        .kV = 0.012098, //.028098,
        .kA = 0.003293, //.004293,
        .kG = 0,
    }
};

// -------- Default Turn Motion Profile --------
MotionController::m_profile_cfg_t mprof_turn_normal_cfg = 
{
    .max_v = 5,
    .accel = 2,
    .pid_cfg = (PID::pid_config_t) {
        .p = .03,
        .i = 0,
        .d = 0,
    },
    .ff_cfg = (FeedForward::ff_config_t)
    {
        .kS = .0,
        .kV = .0,
        .kA = 0
    }
};


motor lf(PORT3, true);
motor lm(PORT5, true); 
motor lr(PORT4); // yes, cable is sketchy
motor_group drive_right(lf, lm, lr);

motor rf(PORT7);
motor rm(PORT6);
motor rr(PORT8, true);
motor_group drive_left(rf, rm, rr);

PID::pid_config_t correction_pid = (PID::pid_config_t) {
        .p = .03,
        .i = 0,
        .d = 0,
    };

// WARNING: DUMMY VALUES, TO BE REPLACED
robot_specs_t specs = {
  .robot_radius = 7.5,
  .odom_wheel_diam = 2.75,
  .odom_gear_ratio = .447,
  .dist_between_wheels = 8.5,
  .drive_correction_cutoff = 1.0,
  .drive_feedback = new MotionController(mprof_drive_normal_cfg),
  .turn_feedback = new MotionController(mprof_turn_normal_cfg),
  .correction_pid = correction_pid,

};

OdometryTank odom(drive_left, drive_right, specs);
TankDrive drive_sys(drive_left, drive_right, specs, &odom);


/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {

}