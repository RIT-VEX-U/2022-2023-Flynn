#include "robot-config.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;
controller main_controller;

// ======== OUTPUTS ========

// Drive
motor left_front(PORT20), left_mid(PORT8), left_rear(PORT9);
motor right_front(PORT11, true), right_mid(PORT1, true), right_rear(PORT2, true);

motor_group left_motors(left_front, left_mid, left_rear);
motor_group right_motors(right_front, right_mid, right_rear);

// Manipulation 
motor flywheel(PORT12);
motor intake(PORT19);
motor roller(PORT18);

motor_group flywheel_motors(flywheel);

// ======== INPUTS ========
CustomEncoder left_enc(Brain.ThreeWirePort.A, 2048);
CustomEncoder mid_enc(Brain.ThreeWirePort.C, 2048);
CustomEncoder right_enc(Brain.ThreeWirePort.E, 2048);

// ======== UTILS ========
// Odometry Tuning
Odometry3Wheel::odometry3wheel_cfg_t odometry_cfg = {

};

// Drive Tuning
PID::pid_config_t drive_pid_cfg = {
    .p = 0,
    .i = 0, 
    .d = 0
};

FeedForward::ff_config_t drive_ff_cfg = {
    .kS = 0,
    .kV = 0,
    .kA = 0
};

MotionController::m_profile_cfg_t drive_fast_mprofile_cfg = {
    .pid_cfg = drive_pid_cfg,
    .ff_cfg = drive_ff_cfg,
    .max_v = 0,
    .accel = 0
};

MotionController::m_profile_cfg_t drive_slow_mprofile_cfg = {
    .pid_cfg = drive_pid_cfg,
    .ff_cfg = drive_ff_cfg,
    .max_v = 0,
    .accel = 0
};

// Turn Tuning
PID::pid_config_t turn_pid_cfg = {
    .p = 0,
    .i = 0, 
    .d = 0
};

FeedForward::ff_config_t turn_ff_cfg = {
    .kS = 0,
    .kV = 0,
    .kA = 0
};

MotionController::m_profile_cfg_t turn_fast_mprofile_cfg = {
    .pid_cfg = turn_pid_cfg,
    .ff_cfg = turn_ff_cfg,
    .max_v = 0,
    .accel = 0
};

MotionController::m_profile_cfg_t turn_slow_mprofile_cfg = {
    .pid_cfg = turn_pid_cfg,
    .ff_cfg = turn_ff_cfg,
    .max_v = 0,
    .accel = 0
};

MotionController drive_fast_mprofile(drive_fast_mprofile_cfg), drive_slow_mprofile(drive_slow_mprofile_cfg);
MotionController turn_fast_mprofile(turn_fast_mprofile_cfg), turn_slow_mprofile(turn_slow_mprofile_cfg);

robot_specs_t config = {
    .robot_radius = 0,
    .drive_correction_cutoff = 0,
    .drive_feedback = &drive_fast_mprofile,
    .turn_feedback = &turn_fast_mprofile,
    .correction_pid = {
        .p = 0,
        .i = 0,
        .d = 0
    }
};

// Flywheel Tuning
FeedForward::ff_config_t flywheel_ff_cfg = {
  .kV =  0.000293
};

PID::pid_config_t flywheel_pid_cfg = {
    .p = .00015
};

// ======== SUBSYSTEMS ========

Odometry3Wheel odometry_sys(left_enc, right_enc, mid_enc, odometry_cfg);

TankDrive drive_sys(left_motors, right_motors, config, &odometry_sys);

Flywheel flywheel_sys(flywheel_motors, flywheel_pid_cfg, flywheel_ff_cfg, 18);

AutoChooser autochooser(Brain);

std::string AutoLoaderSideDisplayName = "Auto Loader Side";
std::string AutoNonLoaderSideDisplayName = "Auto Non Loader Side";
std::string SkillsLoaderSideDisplayName = "Skills Loader Side";
std::string SkillsNonLoaderSideDisplayName = "Skills Non Loader Side";

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {

}