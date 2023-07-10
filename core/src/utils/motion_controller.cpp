#include "../core/include/utils/motion_controller.h"
#include "../core/include/utils/math_util.h"
#include "../core/include/utils/units.h"
#include <vector>

/**
* @brief Construct a new Motion Controller object
* 
* @param config The definition of how the robot is able to move
*    max_v Maximum velocity the movement is capable of
*    accel Acceleration / deceleration of the movement
*    pid_cfg Definitions of kP, kI, and kD
*    ff_cfg Definitions of kS, kV, and kA
*/
template <units::Dimensions input_dims, units::Dimensions output_dims>
MotionController<input_dims, output_dims>::MotionController(
    m_profile_cfg_t &config)
    : config(config), pid(config.pid_cfg), ff(config.ff_cfg),
      profile(config.max_v, config.accel)
{}

/**
 * @brief Initialize the motion profile for a new movement
 * This will also reset the PID and profile timers.
 * @param start_pt Movement starting position
 * @param end_pt Movement ending posiiton 
 */
template <units::Dimensions input_dims, units::Dimensions output_dims>
void MotionController<input_dims, output_dims>::init(InputType start_pt,
                                                     InputType end_pt)
{
    profile.set_endpts(start_pt, end_pt);
    pid.reset();
    tmr.reset();
}

/**
* @brief Update the motion profile with a new sensor value
* 
* @param sensor_val Value from the sensor
* @return the motor input generated from the motion profile 
*/
template <units::Dimensions input_dims, units::Dimensions output_dims>
units::Quantity<output_dims>
MotionController<input_dims, output_dims>::update(InputType sensor_val)
{
    cur_motion = profile.calculate(1_s * tmr.time(timeUnits::sec));
    pid.set_target(cur_motion.pos);
    pid.update(sensor_val);

    out = pid.get() +  ff.calculate(cur_motion.vel, cur_motion.accel, pid.get());

    if(lower_limit != upper_limit)
        out = clamp(out, lower_limit, upper_limit);
    
    return out;
}

/**
 * @return the last saved result from the feedback controller
 */
template <units::Dimensions input_dims, units::Dimensions output_dims>
units::Quantity<output_dims> MotionController<input_dims, output_dims>::get()
{
    return out;
}

/**
 * Clamp the upper and lower limits of the output. If both are 0, no limits should be applied.
 * 
 * @param lower Upper limit
 * @param upper Lower limit
 */
template <units::Dimensions input_dims, units::Dimensions output_dims>
void MotionController<input_dims, output_dims>::set_limits(OutputType lower,
                                                           OutputType upper)
{
    lower_limit = lower;
    upper_limit = upper;
}

/** 
 * @return Whether or not the movement has finished, and the PID
 * confirms it is on target
 */
template <units::Dimensions input_dims, units::Dimensions output_dims>
bool MotionController<input_dims, output_dims>::is_on_target()
{
    return (1_s * tmr.time(timeUnits::sec) > profile.get_movement_time())
           && pid.is_on_target();
}

/**
 * @return The current postion, velocity and acceleration setpoints
*/
template <units::Dimensions input_dims, units::Dimensions output_dims>
motion_t<MotionController<input_dims, output_dims>::InputType::Dims>
MotionController<input_dims, output_dims>::get_motion()
{
    return cur_motion;
}

using DriveFF = FeedForward<units::Length::Dims, units::Voltage::Dims>;

/**
 * This method attempts to characterize the robot's drivetrain and automatically tune the
 * feedforward. It does this by first calculating the kS (voltage to overcome static friction) by
 * slowly increasing the voltage until it moves.
 *
 * Next is kV (voltage to sustain a certain velocity), where the robot will record it's steady-state
 * velocity at 'pct' speed.
 *
 * Finally, kA (voltage needed to accelerate by a certain rate), where the robot will record the
 * entire movement's velocity and acceleration, record a plot of [X=(pct-kV*V-kS), Y=(Acceleration)]
 * along the movement, and since kA*Accel = pct-kV*V-kS, the reciprocal of the linear regression is
 * the kA value.
 *
 * @param drive The tankdrive to operate on
 * @param odometry The robot's odometry subsystem
 * @param volts Maximum voltage applied
 * @param duration Amount of time the robot should be moving for the test
 * @return A tuned feedforward object
 */
DriveFF::ff_config_t tune_feedforward(TankDrive &drive, OdometryTank &odometry,
                                      units::Voltage volts, units::Time duration)
{
    typename DriveFF::ff_config_t out = {};

    units::pose_t start_pos = odometry.get_position();

    // ========== kS Tuning =========
    // Start at 0 and slowly increase the power until the robot starts moving
    units::Voltage power = 0_v;
    while (odometry.pos_diff(start_pos, odometry.get_position()) < 0.05_in) {
        drive.drive_tank(power, power);
        power += 0.001_v;
        vexDelay(100);
    }
    out.kS = power;
    drive.stop();


    // ========== kV / kA Tuning =========

    std::vector<std::pair<units::Time, units::Speed>> vel_data_points;
    std::vector<std::pair<units::Time, units::Acceleration>> accel_data_points;

    units::Speed max_speed = 0_inps;
    timer tmr;
    units::Time time;

    MovingAverage<DriveFF::VelocityType::Dims> vel_ma(3);
    MovingAverage<DriveFF::AccelType::Dims> accel_ma(3);

    // Move the robot forward at a fixed percentage for X seconds while taking velocity and accel measurements
    do
    {
        time = 1_s * tmr.time(sec);

        vel_ma.add_entry(odometry.get_speed());
        accel_ma.add_entry(odometry.get_accel());

        units::Speed speed = vel_ma.get_average();
        units::Acceleration accel = accel_ma.get_average();

        // For kV:
        if(speed > max_speed)
            max_speed = speed;

        // For kA:
        // Filter out the acceleration dampening due to motor inductance
        if (time > 0.25_s) {
            vel_data_points.push_back({time, speed});
            accel_data_points.push_back({time, accel});
        }

        // Theoretical polling rate = 100hz (it won't be that much, cause, y'know, vex.)
        vexDelay(10); 
    } while(time < duration);

    drive.stop();

    // Calculate kV (volts/12 per unit per second)
    out.kV = (volts - out.kS) / max_speed;

    // Calculate kA (volts/12 per unit per second^2)
    // nasty un unitted thing
    std::vector<std::pair<double, double>> accel_per_pct;
    for (int i = 0; i < vel_data_points.size(); i++)
    {
        accel_per_pct.push_back({
            (volts - out.kS - (vel_data_points[i].second * out.kV))
                .getValue(), // Acceleration-causing percent (X variable)
            accel_data_points[i]
                .second.getValue() // Measured acceleration (Y variable)
        });
    }
    
    // kA is the reciprocal of the slope of the linear regression
    double regres_slope = calculate_linear_regression(accel_per_pct).first;
    // get back to unit land
    out.kA = 1.0_v / (regres_slope * 1_inps / 1_s);

    return out;
}

template class MotionController<units::Length::Dims, units::Voltage::Dims>;
template class MotionController<units::Angle::Dims, units::Voltage::Dims>;
template class MotionController<units::AngularSpeed::Dims,
                                units::Voltage::Dims>;
