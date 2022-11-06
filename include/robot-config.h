#pragma once
#include "vex.h"
#include "core.h"

using namespace vex;

extern brain Brain;
extern controller main_controller;
extern inertial imu;

// ======== OUTPUTS ========

// ======== INPUTS ========

// ======== SUBSYSTEMS ========

// ======== UTILS ========
extern robot_specs_t config;
extern OdometryTank odom;
extern TankDrive drive_sys;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);

