#include "../core/include/utils/motion_controller.h"

/**
 * @brief Construct a new Motion Controller object
 * 
 * @param max_v Maximum velocity the movement is capable of
 * @param accel Acceleration / deceleration of the movement
 * @param pid_cfg Definitions of kP, kI, and kD
 * @param ff_cfg Definitions of kS, kV, and kA
 */
MotionController::MotionController(double max_v, double accel, PID::pid_config_t &pid_cfg, FeedForward::ff_config_t &ff_cfg)
: max_v(max_v), accel(accel), pid_cfg(pid_cfg), pid(pid_cfg), ff(ff_cfg), profile(max_v, accel)
{}

/**
 * @brief Initialize the motion profile for a new movement
 * This will also reset the PID and profile timers.
 * @param start_pt Movement starting position
 * @param end_pt Movement ending posiiton 
 */
void MotionController::init(double start_pt, double end_pt)
{
    profile.set_endpts(start_pt, end_pt);
    pid.reset();
    tmr.reset();
}

/**
 * @brief Update the motion profile with a new sensor value
 * 
 * @param sensor_val Value from the sensor
 * @return The motor input generated from the motion profile 
 */
double MotionController::update(double sensor_val)
{
    motion_t motion = profile.calculate(tmr.time(timeUnits::sec));
    pid.set_target(motion.pos);
    pid.update(sensor_val);

    return pid.get() +  ff.calculate(motion.vel, motion.accel);
}

/** 
 * @return Whether or not the movement has finished, and the PID
 * confirms it is on target
 */
bool MotionController::is_on_target()
{
    return (tmr.time(timeUnits::sec) > profile.get_movement_time()) && pid.is_on_target();
}
