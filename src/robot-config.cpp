#include "robot-config.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;

// ======== OUTPUTS ========

// ======== INPUTS ========

// ======== SUBSYSTEMS ========

// ======== UTILS ========

// -------- EXAMPLE ONLY, DO NOT USE ------------

// -------- Default Drive Motion Profile --------
MotionController::m_profile_cfg_t mprof_drive_normal_cfg = 
{
    .max_v = 5,
    .accel = 2,
    .pid_cfg = (PID::pid_config_t) {
        .p = .1,
        .i = 0,
        .d = 0.01
    },
    .ff_cfg = (FeedForward::ff_config_t)
    {
        .kS = .05,
        .kV = .1,
        .kA = 0
    }
};

// -------- Default Turn Motion Profile --------
MotionController::m_profile_cfg_t mprof_turn_normal_cfg = 
{
    .max_v = 5,
    .accel = 2,
    .pid_cfg = (PID::pid_config_t) {
        .p = .1,
        .i = 0,
        .d = 0.01
    },
    .ff_cfg = (FeedForward::ff_config_t)
    {
        .kS = .05,
        .kV = .1,
        .kA = 0
    }
};

robot_specs_t config = {
    .drive_feedback = new MotionController(mprof_drive_normal_cfg),
    .turn_feedback = new MotionController(mprof_turn_normal_cfg)
    // .drive_feedback = new PID(),
    // .turn_feedback = new PID()

};

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {

}