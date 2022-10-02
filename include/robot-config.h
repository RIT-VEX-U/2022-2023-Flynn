#pragma once
#include "vex.h"
#include "core.h"

using namespace vex;

extern brain Brain;
extern controller main_controller;

// ======== OUTPUTS ========

extern motor indexer, intake;


// ======== INPUTS ========
extern inertial imu;

// ======== SUBSYSTEMS ========

// extern motor_group drive_right;
// extern motor rf, rm, rr;

// extern motor_group drive_left;
// extern motor lf, lm, lr;

// NEMO DRIVE TESTING
extern motor lf_drive, lr_drive, rf_drive, rr_drive;

extern motor_group left_motors, right_motors;

extern PID pid;

extern OdometryTank odom;
extern TankDrive drive_sys;

// TODO: add Flywheel class
extern motor fw_top, fw_bot;


// ======== UTILS ========
extern robot_specs_t config;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);

