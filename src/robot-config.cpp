#include <map>
#include "../include/robot-config.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;
controller main_controller;

// ======== OUTPUTS ========

// Drive
motor left_front(PORT13, vex::gearSetting::ratio18_1, true), left_mid(PORT12, vex::gearSetting::ratio18_1, true), left_rear(PORT11, vex::gearSetting::ratio18_1, true);
motor right_front(PORT18, vex::gearSetting::ratio18_1), right_mid(PORT19, vex::gearSetting::ratio18_1), right_rear(PORT20, vex::gearSetting::ratio18_1);

motor_group left_motors(left_front, left_mid, left_rear);
motor_group right_motors(right_front, right_mid, right_rear);

inertial imu(PORT3);

// Manipulation
motor flywheel(PORT10);
motor intake(PORT1);

motor_group flywheel_motors(flywheel);

std::map<std::string, motor &> motor_names{
    {"left front", left_front},
    {"left mid", left_mid},
    {"left rear", left_rear},

    {"right front", right_front},   
    {"right mid", right_mid},
    {"right rear", right_rear},

    {"flywheel", flywheel},
    {"intake", intake},

};

std::map<std::string, device&> device_names{
    {"imu", imu},
};

// Other Outputs
vex::digital_out endgame_solenoid(Brain.ThreeWirePort.H); // TODO make this an actual port

vex::digital_out flapup_solenoid(Brain.ThreeWirePort.G);

// ======== INPUTS ========
CustomEncoder left_enc(Brain.ThreeWirePort.G, -2048);
CustomEncoder right_enc(Brain.ThreeWirePort.E, 2048);


// ======== UTILS ========
// Drive Tuning
PID::pid_config_t drive_pid_cfg = {
    .p = .035,
    .i = 0,
    .d = 0,
    .deadband = 2,
    .on_target_time = 0.2};

FeedForward::ff_config_t drive_ff_cfg = {
    .kS = 0.07,
    .kV = .011, // 0.014205,
    .kA = 0.0015};

MotionController::m_profile_cfg_t drive_fast_mprofile_cfg = {
    .max_v = 40,  // MAX = 48,
    .accel = 150, // MAX = 200
    .pid_cfg = drive_pid_cfg,
    .ff_cfg = drive_ff_cfg};

MotionController::m_profile_cfg_t drive_slow_mprofile_cfg = {
    .max_v = 15,
    .accel = 100,
    .pid_cfg = drive_pid_cfg,
    .ff_cfg = drive_ff_cfg};


MotionController::m_profile_cfg_t drive_super_fast_mprofile_cfg = {
    .max_v = 40,  // MAX = 48,
    .accel = 150, // MAX = 200
    .pid_cfg = drive_pid_cfg,
    .ff_cfg = drive_ff_cfg};

// Turn Tuning
PID::pid_config_t turn_pid_cfg = {
    .p = .013,
    .i = 0.00001,
    .d = .00085,
    .deadband = 2.0,
    .on_target_time = .2};

FeedForward::ff_config_t turn_ff_cfg =
    {
        .kS = 0.08};

// FeedForward::ff_config_t turn_ff_cfg = {
//     .kS = 0.11,
//     .kV = 0.001,
//     .kA = .00017
// };

// MotionController::m_profile_cfg_t turn_fast_mprofile_cfg = {
//     .max_v = 600, //700,
//     .accel = 1000, //1400,
//     .pid_cfg = turn_pid_cfg,
//     .ff_cfg = turn_ff_cfg
// };

// MotionController::m_profile_cfg_t turn_slow_mprofile_cfg = {
//     .max_v = 0,
//     .accel = 0,
//     .pid_cfg = turn_pid_cfg,
//     .ff_cfg = turn_ff_cfg
// };

MotionController drive_fast_mprofile(drive_fast_mprofile_cfg), drive_slow_mprofile(drive_slow_mprofile_cfg), drive_super_fast_mprofile(drive_fast_mprofile_cfg);
// MotionController turn_fast_mprofile(turn_fast_mprofile_cfg), turn_slow_mprofile(turn_slow_mprofile_cfg);

robot_specs_t config = {
    .robot_radius = 10,
    .odom_wheel_diam = 6.424194,
    .odom_gear_ratio = 1, // .44    16:12
    .dist_between_wheels = 10.163,

    .drive_correction_cutoff = 4,

    .drive_feedback = &drive_fast_mprofile,
    .turn_feedback = new PIDFF(turn_pid_cfg, turn_ff_cfg),
    .correction_pid = {
        .p = .012,
        .i = 0,
        .d = 0.0012}};

// Flywheel Tuning
FeedForward::ff_config_t flywheel_ff_cfg = {
    .kV = 0.00028};

PID::pid_config_t flywheel_pid_cfg = {
    .p = .0000, // 5,
    //    .d = 0.000015,
};

// ======== SUBSYSTEMS ========

// OdometryTank odometry_sys(left_enc, right_enc, config);
OdometryTank odometry_sys(left_motors, right_motors, config, &imu); // PUT THIS BACK YUO HOOLIGAN

TankDrive drive_sys(left_motors, right_motors, config, &odometry_sys);

Flywheel flywheel_sys(flywheel_motors, 0.000009, 18);
// Flywheel flywheel_sys(flywheel_motors, flywheel_pid_cfg, flywheel_ff_cfg, 18);
vex::timer oneshot_tmr;

AutoChooser autochooser(Brain);

bool target_red = false;
bool vision_enabled = true;
int num_roller_fallback = 2;
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void)
{
    StartScreen(Brain.Screen, {page_one, page_two, page_three, page_four, page_five, page_six}, 4);

    endgame_solenoid.set(false); // TODO figure out if false or true shoots
    imu.calibrate();
}