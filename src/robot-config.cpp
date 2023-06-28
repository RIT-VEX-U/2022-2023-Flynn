#include "robot-config.h"
#include "../core/include/subsystems/screen.h"
#include <map>

using namespace vex;
using namespace unit_literals;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;
controller main_controller;

// ======== OUTPUTS ========

// Drive
motor left_front(PORT13, vex::gearSetting::ratio18_1, true), left_mid(PORT12, vex::gearSetting::ratio18_1, true), left_rear(PORT11, vex::gearSetting::ratio18_1, true);
motor right_front(PORT18, vex::gearSetting::ratio18_1), right_mid(PORT19, vex::gearSetting::ratio18_1), right_rear(PORT20, vex::gearSetting::ratio18_1);

motor_group left_motors(left_front, left_mid, left_rear);
motor_group right_motors(right_front, right_mid, right_rear);

inertial imu(PORT2);

// Manipulation
motor flywheel(PORT10);
motor intake(PORT1);

motor_group flywheel_motors(flywheel);

// Motor Mappings
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

std::map<std::string, device &> device_names{
    {"imu", imu},
};

// Other Outputs
vex::digital_out endgame_solenoid(Brain.ThreeWirePort.A); // TODO make this an actual port
vex::digital_out flapup_solenoid(Brain.ThreeWirePort.C);
vex::digital_out intake_solenoid(Brain.ThreeWirePort.B);

optical roller_sensor(PORT17);

// ======== INPUTS ========
CustomEncoder left_enc(Brain.ThreeWirePort.G, -2048);
CustomEncoder right_enc(Brain.ThreeWirePort.E, 2048);

// ======== UTILS ========
// Drive Tuning
DrivePid::pid_config_t drive_pid_cfg = {.p = .035_v / 1_in,
                                        .i = 0_v / (1_in * 1_s),
                                        .d = 0_v / (1_in / 1_s),
                                        .deadband = 2_in,
                                        .on_target_time = 0.2_s};

DriveFF::ff_config_t drive_ff_cfg
    = {.kS = 0.07_v, .kV = .011_v / (1_inps), .kA = 0.0015_v / (1_inps / 1_s)};

DriveMC::m_profile_cfg_t drive_fast_mprofile_cfg
    = {.max_v = 60_inps,        // MAX = 48,
       .accel = 140_inps / 1_s, // MAX = 200
       .pid_cfg = drive_pid_cfg,
       .ff_cfg = drive_ff_cfg};

DriveMC::m_profile_cfg_t drive_slow_mprofile_cfg = {.max_v = 15_inps,
                                                    .accel = 100_inps / 1_s,
                                                    .pid_cfg = drive_pid_cfg,
                                                    .ff_cfg = drive_ff_cfg};

// Turn Tuning
TurnPid::pid_config_t turn_pid_cfg
    = {.p = .013_v / units::degree,
       .i = 0.00001_v / (units::degree * units::second),
       .d = .00085_v / (1_deg / 1_s),
       .deadband = 2.0_deg,
       .on_target_time = .2_s};

TurnFF::ff_config_t turn_ff_cfg = {.kS = 0.08_v};

DriveMC drive_fast_mprofile(drive_fast_mprofile_cfg),
    drive_slow_mprofile(drive_slow_mprofile_cfg),
    drive_super_fast_mprofile(drive_fast_mprofile_cfg);

robot_specs_t config = {
    .robot_radius = 10,
    .odom_wheel_diam = 6.424194_in,
    .odom_gear_ratio = 1, // .44    16:12
    .dist_between_wheels = 10.99_in,

    .drive_correction_cutoff = 6_in,

    .drive_feedback = &drive_fast_mprofile,
    .turn_feedback = new PIDFF<units::Angle::Dims, units::Voltage::Dims>(turn_pid_cfg, turn_ff_cfg),
    .correction_pid = {.p = .012_v / units::degree,
                       .i = 0_v / (units::degree * units::second),
                       .d = 0.0012_v / (units::degree_per_sec)}};

// Flywheel Tuning
FeedForwardFW::ff_config_t flywheel_ff_cfg = {.kV = 0.0003_v / 1_rpm};

PidFW::pid_config_t flywheel_pid_cfg = {
    .p = .0000_v / 1_rpm, // 5,
};

// ======== SUBSYSTEMS ========

// OdometryTank odometry_sys(left_enc, right_enc, config);
OdometryTank odometry_sys(left_motors, right_motors, config, &imu); // PUT THIS BACK YUO HOOLIGAN

TankDrive drive_sys(left_motors, right_motors, config, &odometry_sys);

Flywheel flywheel_sys(flywheel_motors, flywheel_ff_cfg, 18);
// Flywheel flywheel_sys(flywheel_motors, flywheel_pid_cfg, flywheel_ff_cfg, 18);
vex::timer oneshot_tmr;
vex::timer auto_tmr;

AutoChooser autochooser(Brain);

bool target_red = true;
bool vision_enabled = true;
int num_roller_fallback = 3;
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
std::vector<screen::Page *> pages = {};
void vexcodeInit(void)
{
    // StartScreen(Brain.Screen, {page_one, page_two, page_three, page_four, page_five, page_six}, 4);
    screen::start_screen(Brain.Screen, pages);

    endgame_solenoid.set(false); // TODO figure out if false or true shoots
    imu.calibrate();
}
