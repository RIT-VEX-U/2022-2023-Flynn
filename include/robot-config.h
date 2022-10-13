#pragma once
#include "vex.h"
#include "core.h"

using namespace vex;

extern brain Brain;
extern controller main_controller;

// ======== OUTPUTS ========

extern motor indexer, intake;


// ======== INPUTS ========

// ======== SUBSYSTEMS ========

extern motor_group drive_right;
extern motor rf, rm, rr;

extern motor_group drive_left;
extern motor lf, lm, lr;

extern OdometryTank odom;
extern TankDrive drive_sys;

// TODO: add Flywheel class
extern motor fw_top, fw_bot;


// ======== UTILS ========

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);

