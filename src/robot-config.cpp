#include "robot-config.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;
controller main_controller;

// ======== OUTPUTS ========

// Drive
motor left_front(PORT20, vex::gearSetting::ratio18_1, true), left_mid(PORT8, vex::gearSetting::ratio18_1, true), left_rear(PORT9, vex::gearSetting::ratio18_1, true);
motor right_front(PORT11, vex::gearSetting::ratio18_1), right_mid(PORT1, vex::gearSetting::ratio18_1), right_rear(PORT2, vex::gearSetting::ratio18_1);

motor_group left_motors(left_front, left_mid, left_rear);
motor_group right_motors(right_front, right_mid, right_rear);

// Manipulation 
motor flywheel(PORT12);
motor intake(PORT19);
motor roller(PORT5);

motor_group flywheel_motors(flywheel);

// ======== INPUTS ========
CustomEncoder left_enc(Brain.ThreeWirePort.A, 2048);
CustomEncoder right_enc(Brain.ThreeWirePort.C, 2048);

inertial imu(PORT4);

// ======== UTILS ========

// Drive Tuning
PID::pid_config_t drive_pid_cfg = {
    .p = .025,
    .i = 0, 
    .d = 0,
    .deadband = 2,
    .on_target_time = 0.2
};

FeedForward::ff_config_t drive_ff_cfg = {
    .kS = 0.07,
    .kV =.011, // 0.014205,
    .kA = 0.0015
};

MotionController::m_profile_cfg_t drive_fast_mprofile_cfg = {
    .pid_cfg = drive_pid_cfg,
    .ff_cfg = drive_ff_cfg,
    .max_v = 40,// MAX = 48,
    .accel = 150 // MAX = 200
};

MotionController::m_profile_cfg_t drive_slow_mprofile_cfg = {
    .pid_cfg = drive_pid_cfg,
    .ff_cfg = drive_ff_cfg,
    .max_v = 20,
    .accel = 100
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



MotionController drive_fast_mprofile(drive_fast_mprofile_cfg), drive_slow_mprofile(drive_slow_mprofile_cfg);
//MotionController turn_fast_mprofile(turn_fast_mprofile_cfg), turn_slow_mprofile(turn_slow_mprofile_cfg);



robot_specs_t config = {
    .robot_radius = 10,
    .odom_wheel_diam = 6.374,
    .odom_gear_ratio = 1, // .44    16:12
    .dist_between_wheels = 10.163,

    .drive_correction_cutoff = 4,

    .drive_feedback = &drive_fast_mprofile,
    .turn_feedback = new PID(turn_pid_cfg),
    .correction_pid = {
        .p = .012,
        .i = 0,
        .d = 0.0012
    }
};

// Flywheel Tuning
FeedForward::ff_config_t flywheel_ff_cfg = {
  .kV =  0.0003 
};

PID::pid_config_t flywheel_pid_cfg = {
    .p = .003,
    .d = 0.000015,
};

// ======== SUBSYSTEMS ========


// OdometryTank odometry_sys(left_enc, right_enc, config);
OdometryTank odometry_sys(left_motors, right_motors, config, &imu);

TankDrive drive_sys(left_motors, right_motors, config, &odometry_sys);

Flywheel flywheel_sys(flywheel_motors, flywheel_pid_cfg, flywheel_ff_cfg, 18);
vex::timer oneshot_tmr;

AutoChooser autochooser(Brain);

vex::digital_out endgame_solenoid(Brain.ThreeWirePort.H); //TODO make this an actual port

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
    endgame_solenoid.set(false); //TODO figure out if false or true shoots 
    imu.calibrate();
}