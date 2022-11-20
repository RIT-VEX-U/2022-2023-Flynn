#include "robot-config.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;
controller main_controller;
inertial imu(PORT20);

// ======== OUTPUTS ========

// ======== INPUTS ========

// ======== SUBSYSTEMS ========

// ======== UTILS ========

// -------- EXAMPLE ONLY, DO NOT USE ------------

// -------- Default Drive Motion Profile --------
MotionController::m_profile_cfg_t mprof_drive_normal_cfg = 
{
    .max_v =  60,
    .accel = 200,
    .pid_cfg = (PID::pid_config_t) {
        .p = .31,
        .i = 0.012,
        .d = 0.0001,
        .deadband = .15,
    },
    .ff_cfg = (FeedForward::ff_config_t)
    {
        .kS = 0.061000,
        .kV = 0.028098,
        .kA = 0.004293, 
        .kG = 0,
    }
};

// -------- Default Turn Motion Profile --------
MotionController::m_profile_cfg_t mprof_turn_normal_cfg = 
{
    .max_v = 600,
    .accel = 1200,
    .pid_cfg = (PID::pid_config_t) {
        .p = 0.03,//6,
        .i = 0.01,
        .d = 0.0025,
        .deadband = 1,//degree
    },
    .ff_cfg = (FeedForward::ff_config_t)
    {
        .kS = 0,//0,//0.1,//8,
        .kV = 0,//0.0015,//-0.0025,//1,//2,
        .kA = 0,//0.00065,//004,//03
    }
};


motor lf(PORT3, true);
motor lm(PORT5, true); 
motor lr(PORT4); // yes, cable is sketchy
motor_group drive_left(lf, lm, lr);

motor rf(PORT7);
motor rm(PORT6);
motor rr(PORT8, true);
motor_group drive_right(rf, rm, rr);

PID::pid_config_t correction_pid = (PID::pid_config_t) {
        .p = .03,
        .i = 0,
        .d = 0,
    };


double calculate_circular_error (double target, double sensor_val){
  return OdometryBase::smallest_angle(target, sensor_val);
}
// WARNING: DUMMY VALUES, TO BE REPLACED
robot_specs_t specs = {
  .robot_radius = 7.5,
  .odom_wheel_diam = 3.25,
  .odom_gear_ratio = 36.0/60.0,
  .dist_between_wheels = 10.75,
  .drive_correction_cutoff = 2.0,
  .drive_feedback = new MotionController(mprof_drive_normal_cfg),
  .turn_feedback = new MotionController(mprof_turn_normal_cfg, calculate_circular_error),
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