#pragma once
#include "vex.h"
#include "core.h"

using namespace vex;

extern brain Brain;
extern controller main_controller;

// ======== OUTPUTS ========
extern motor left_front, left_mid, left_rear;
extern motor right_front, right_mid, right_rear;
extern motor intake, roller, flywheel;
extern motor_group left_motors, right_motors;
extern motor_group flywheel_motors;

// ======== INPUTS ========
extern CustomEncoder left_enc, right_enc, mid_enc;

// ======== UTILS ========
// Drive Tuning
extern Odometry3Wheel::odometry3wheel_cfg_t odometry_cfg;
extern robot_specs_t config;

// Flywheel Tuning
extern FeedForward::ff_config_t flywheel_ff_cfg;
extern PID::pid_config_t flywheel_pid_cfg;

// ======== SUBSYSTEMS ========
extern Odometry3Wheel odometry_sys;
extern TankDrive drive_sys;
extern Flywheel flywheel_sys;



/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);

