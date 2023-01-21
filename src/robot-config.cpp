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
robot_specs_t config = {

};

// Flywheel Tuning
FeedForward::ff_config_t flywheel_ff_cfg = {

};

PID::pid_config_t flywheel_pid_cfg = {
    
};

// ======== SUBSYSTEMS ========

Odometry3Wheel odometry_sys(left_enc, right_enc, mid_enc, odometry_cfg);

TankDrive drive_sys(left_motors, right_motors, config, &odometry_sys);

Flywheel flywheel_sys(flywheel_motors, flywheel_pid_cfg, flywheel_ff_cfg, 1.0);



/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {

}