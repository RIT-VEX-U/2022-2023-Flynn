#pragma once
#include "core.h"
#include "vex.h"
#include <map>

using namespace vex;

extern brain Brain;
extern controller main_controller;

// ======== OUTPUTS ========
extern motor left_front, left_mid, left_rear;
extern motor right_front, right_mid, right_rear;
extern motor intake, roller, flywheel;
extern motor_group left_motors, right_motors;
extern motor_group flywheel_motors;

extern std::map<std::string, motor &> motor_names;
extern std::map<std::string, device&> device_names;

extern vex::pot selector_pot;


extern vex::digital_out endgame_solenoid;

extern vex::digital_out flapdown_solenoid;
extern vex::digital_out flapup_solenoid;
extern vex::digital_out intake_solenoid;
// ======== INPUTS ========
extern CustomEncoder left_enc, right_enc, mid_enc;
extern inertial imu;

extern vex::optical roller_sensor;

// ======== UTILS ========
// Drive Tuning

using DrivePid = PID<units::Length::Dims, units::Voltage::Dims>;
using TurnPid = PID<units::Angle::Dims, units::Voltage::Dims>;

using DriveFF = FeedForward<units::Length::Dims, units::Voltage::Dims>;
using TurnFF = FeedForward<units::Angle::Dims, units::Voltage::Dims>;

using DriveMC = MotionController<units::Length::Dims, units::Voltage::Dims>;
using TurnMC = MotionController<units::Angle::Dims, units::Voltage::Dims>;

extern DrivePid::pid_config_t drive_pid_cfg;
extern TurnPid::pid_config_t turn_pid_cfg;

extern DriveFF::ff_config_t drive_ff_cfg;
extern TurnFF::ff_config_t turn_ff_cfg;

extern DriveMC::m_profile_cfg_t drive_fast_mprofile_cfg,
    drive_slow_mprofile_cfg;

extern DriveMC drive_fast_mprofile, drive_slow_mprofile,
    drive_super_fast_mprofile;

extern robot_specs_t config;

// Flywheel Tuning

using FeedForwardFW = FeedForward<units::Angle::Dims, units::Voltage::Dims>;
using PidFW = PID<units::AngularSpeed::Dims, units::Voltage::Dims>;

extern FeedForwardFW::ff_config_t flywheel_ff_cfg;
extern PidFW::pid_config_t flywheel_pid_cfg;

// ======== SUBSYSTEMS ========
extern OdometryTank odometry_sys;
extern TankDrive drive_sys;
extern Flywheel flywheel_sys;
extern vex::timer oneshot_tmr;
extern vex::timer auto_tmr;



extern AutoChooser autochooser;

extern std::string AutoLoaderSideDisplayName;
extern std::string AutoNonLoaderSideDisplayName;
extern std::string SkillsLoaderSideDisplayName;
extern std::string SkillsNonLoaderSideDisplayName;

extern bool target_red;
extern bool vision_enabled;
extern int num_roller_fallback;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);

