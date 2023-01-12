#pragma once
#include "vex.h"
#include "core.h"

using namespace vex;

extern brain Brain;
extern controller main_controller;

// ======== OUTPUTS ========

extern motor indexer, intake;


// ======== INPUTS ========
extern limit shoot_limit;

// ======== SUBSYSTEMS ========
extern PID::pid_config_t flywheel_pid;
extern motor_group drive_right;
extern motor rf, rm, rr;

extern motor_group drive_left;
extern motor lf, lm, lr;

extern vex::encoder lside, rside, offaxis;

extern OdometryTank odom;
extern Odometry3Wheel odometry3wheel;
extern TankDrive drive_sys;

// TODO: add Flywheel class
extern motor fw_top, fw_bot;
extern motor_group fw_group;
extern Flywheel flywheel;


// ======== UTILS ========

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);