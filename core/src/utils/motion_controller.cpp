#include "../core/include/utils/motion_controller.h"
#include "../core/include/utils/math_util.h"
#include "../core/include/utils/moving_average.h"
#include "C:/Users/richi/VEX/2022-2023-Flynn/include/robot-config.h"
#include <vector>

/**
 * @brief Construct a new Motion Controller object
 * 
 * @param max_v Maximum velocity the movement is capable of
 * @param accel Acceleration / deceleration of the movement
 * @param pid_cfg Definitions of kP, kI, and kD
 * @param ff_cfg Definitions of kS, kV, and kA
 */
MotionController::MotionController(m_profile_cfg_t &config)
: config(config), pid(config.pid_cfg), ff(config.ff_cfg), profile(config.max_v, config.accel)
{}

MotionController::MotionController(m_profile_cfg_t &config, double (*calculate_error)(double target, double sensor_val))
: config(config), pid(config.pid_cfg), ff(config.ff_cfg), profile(config.max_v, config.accel)
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
    printf("targetP: %f\ttargetV: %f\t", motion.pos, motion.vel);
    pid.set_target(motion.pos);
    
    
    /*


    */

    double naive_error = motion.pos - sensor_val;
    double actual_error = OdometryBase::smallest_angle(motion.pos, sensor_val);
    if (!(fabs(naive_error - actual_error)<.001)){
      //pid error and our error dont agree
      if (sensor_val<0){
        sensor_val +=360;
      } else {
        sensor_val -= 360;
      }
      pid.update(sensor_val);
    }

    out = pid.get() +  ff.calculate(motion.vel, motion.accel);

    if(lower_limit != upper_limit){
        out = clamp(out, lower_limit, upper_limit);
    }
    return out;
}

/**
 * @return the last saved result from the feedback controller
 */
double MotionController::get()
{
      
    return out;
}

/**
 * Clamp the upper and lower limits of the output. If both are 0, no limits should be applied.
 * 
 * @param lower Upper limit
 * @param upper Lower limit
 * @return double 
 */
void MotionController::set_limits(double lower, double upper)
{
    lower_limit = lower;
    upper_limit = upper;
}

/** 
 * @return Whether or not the movement has finished, and the PID
 * confirms it is on target
 */
bool MotionController::is_on_target()
{
    return (tmr.time(timeUnits::sec) > profile.get_movement_time()) && pid.is_on_target();
}

/**
 * This method attempts to characterize the robot's drivetrain and automatically tune the feedforward.
 * It does this by first calculating the kS (voltage to overcome static friction) by slowly increasing
 * the voltage until it moves.
 * 
 * Next is kV (voltage to sustain a certain velocity), where the robot will record it's steady-state velocity
 * at 'pct' speed.
 * 
 * Finally, kA (voltage needed to accelerate by a certain rate), where the robot will record the entire movement's
 * velocity and acceleration, record a plot of [X=(pct-kV*V-kS), Y=(Acceleration)] along the movement,
 * and since kA*Accel = pct-kV*V-kS, the reciprocal of the linear regression is the kA value.
 * 
 * @param drive The tankdrive to operate on
 * @param odometry The robot's odometry subsystem
 * @param pct Maximum velocity in percent (0->1.0)
 * @param duration Amount of time the robot should be moving for the test
 * @return A tuned feedforward object
 */
FeedForward::ff_config_t MotionController::tune_feedforward(TankDrive &drive, OdometryTank &odometry, double pct, double duration)
{
    FeedForward::ff_config_t out = {};
    
    position_t start_pos = odometry.get_position();

    // ========== kS Tuning =========
    printf("kS\n");fflush(stdout);

    // Start at 0 and slowly increase the power until the robot starts moving
    double power = 0;
    while(odometry.pos_diff(start_pos, odometry.get_position()) < 0.05)
    {
        drive.drive_tank(power, power, 1);
        power += 0.001;
        vexDelay(100);
    }
    out.kS = power;
    drive.stop();


    // ========== kV / kA Tuning =========
    printf("kV, kA\n");fflush(stdout);
    std::vector<std::pair<double, double>> vel_data_points; // time, velocity
    std::vector<std::pair<double, double>> accel_data_points; // time, accel

    double max_speed = 0;
    timer tmr;
    double time;

    MovingAverage vel_ma(3);
    MovingAverage accel_ma(3);

    // Move the robot forward at a fixed percentage for X seconds while taking velocity and accel measurements
    drive.drive_tank(pct, pct, 1);
    do
    {
        time = tmr.time(sec);

        vel_ma.add_entry(odometry.get_speed());
        accel_ma.add_entry(odometry.get_accel());

        double speed = vel_ma.get_average();
        double accel = accel_ma.get_average();

        // For kV:
        if(speed > max_speed)
            max_speed = speed;

        // For kA:
        // Filter out the acceleration dampening due to motor inductance
        if(time > 0.25)
        {
            vel_data_points.push_back(std::pair<double, double>(time, speed));
            accel_data_points.push_back(std::pair<double, double>(time, accel));
        }

        // Theoretical polling rate = 100hz (it won't be that much, cause, y'know, vex.)
        vexDelay(10); 
    } while(time < duration);

    drive.stop();

    // Calculate kV (volts/12 per unit per second)
    out.kV = (pct - out.kS) / max_speed;

    // Calculate kA (volts/12 per unit per second^2)
    std::vector<std::pair<double, double>> accel_per_pct;
    for (int i = 0; i < vel_data_points.size(); i++)
    {
        accel_per_pct.push_back(std::pair<double, double>(
            pct - out.kS - (vel_data_points[i].second * out.kV),   // Acceleration-causing percent (X variable)
            accel_data_points[i].second                            // Measured acceleration (Y variable)
        ));
    }
    
    // kA is the reciprocal of the slope of the linear regression
    double regres_slope = calculate_linear_regression(accel_per_pct).first;
    out.kA = 1.0 / regres_slope; 

    return out;
}

/**
 * This method attempts to characterize the robot's drivetrain and automatically tune the feedforward.
 * It does this by first calculating the kS (voltage to overcome static friction) by slowly increasing
 * the voltage until it moves.
 * 
 * Next is kV (voltage to sustain a certain velocity), where the robot will record it's steady-state velocity
 * at 'pct' speed.
 * 
 * Finally, kA (voltage needed to accelerate by a certain rate), where the robot will record the entire movement's
 * velocity and acceleration, record a plot of [X=(pct-kV*V-kS), Y=(Acceleration)] along the movement,
 * and since kA*Accel = pct-kV*V-kS, the reciprocal of the linear regression is the kA value.
 * 
 * @param drive The tankdrive to operate on
 * @param odometry The robot's odometry subsystem
 * @param pct Maximum velocity in percent (0->1.0)
 * @param duration Amount of time the robot should be moving for the test
 * @return A tuned feedforward object
 */
FeedForward::ff_config_t MotionController::tune_feedforward_turning(TankDrive &drive, OdometryTank &odometry, double pct, double duration)
{
    FeedForward::ff_config_t out = {};
    

    // ========== kS Tuning =========
    printf("kS\n");fflush(stdout);

    // Start at 0 and slowly increase the power until the robot starts moving
    double power = 0;

    double initial_heading = imu.heading();
    /*
    while(    OdometryBase::smallest_angle(imu.heading(), initial_heading ) < 0.05)
    {
        printf("diff %f : %f : %f\t\n", imu.heading(), initial_heading, OdometryBase::smallest_angle(imu.heading(), initial_heading));fflush(stdout);
        drive.drive_tank(-power, power, 1);
        power += 0.001;

        vexDelay(100);
    }
    out.kS = power;
    drive.stop();
    */

    // ========== kV / kA Tuning =========
    printf("kV, kA\n");fflush(stdout);
    std::vector<std::pair<double, double>> vel_data_points; // time, velocity
    std::vector<std::pair<double, double>> accel_data_points; // time, accel

    double max_speed = 0;
    timer tmr;
    double time;

    MovingAverage rot_ma(3);
    MovingAverage vel_ma(6);
    MovingAverage accel_ma(10);

    double last_rot = 0;
    double last_vel = 0;

    double full_rotations = 0;
    double last_raw_rotation = 0;

    // Move the robot forward at a fixed percentage for X seconds while taking velocity and accel measurements
    drive.drive_tank(pct, -pct, 1);
    do
    {
        time = tmr.time(sec);

        double dt = 0.01;

        if (imu.heading() < last_raw_rotation){
          full_rotations+=1;
        }

        double rot = imu.heading();
        rot=360*full_rotations + rot;
        rot_ma.add_entry(rot);
        rot = rot_ma.get_average();
        
        double speed = vel_ma.get_average();
        // For kV:
        if(speed > max_speed)
            max_speed = speed;

        // For kA:
        // Filter out the acceleration dampening due to motor inductance
        if(time > 0.15)
        {
          double vel = (rot-last_rot)/dt;
          vel_ma.add_entry(vel);
          vel = vel_ma.get_average();
          
          accel_ma.add_entry((vel-last_vel)/dt);

          double speed = vel_ma.get_average();
          double accel = accel_ma.get_average();
          last_rot = rot;
          last_vel = vel;

          vel_data_points.push_back(std::pair<double, double>(time, speed));
          accel_data_points.push_back(std::pair<double, double>(time, accel));
          printf("time: %f\t%f\t%f\t%f\n", time, rot, vel, accel);

        }
        // Theoretical polling rate = 100hz (it won't be that much, cause, y'know, vex.)
        vexDelay(10); 
    } while(time < duration);

    drive.stop();

    // Calculate kV (volts/12 per unit per second)
    out.kV = (pct - out.kS) / max_speed;

    // Calculate kA (volts/12 per unit per second^2)
    std::vector<std::pair<double, double>> accel_per_pct;
    for (int i = 0; i < vel_data_points.size(); i++)
    {
        accel_per_pct.push_back(std::pair<double, double>(
            pct - out.kS - (vel_data_points[i].second * out.kV),   // Acceleration-causing percent (X variable)
            accel_data_points[i].second                            // Measured acceleration (Y variable)
        ));
    }
    
    // kA is the reciprocal of the slope of the linear regression
    double regres_slope = calculate_linear_regression(accel_per_pct).first;
    out.kA = 1.0 / regres_slope; 

    return out;
}