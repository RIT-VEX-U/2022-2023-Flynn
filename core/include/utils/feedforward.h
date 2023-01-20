#pragma once

/**
 * FeedForward
 * 
 * Stores the feedfoward constants, and allows for quick computation.
 * Feedfoward should be used in systems that require smooth precise movements
 * and have high inertia, such as drivetrains and lifts.
 * 
 * This is best used alongside a PID loop, with the form:
 * output = pid.get() + feedforward.calculate(v, a);
 * 
 * In this case, the feedforward does the majority of the heavy lifting, and the 
 * pid loop only corrects for inconsistencies
 * 
 * For information about tuning feedforward, I reccommend looking at this post:
 * https://www.chiefdelphi.com/t/paper-frc-drivetrain-characterization/160915
 * (yes I know it's for FRC but trust me, it's useful)
 * 
 * @author Ryan McGee
 * @date 6/13/2022
 */
#include <math.h>
#include <vector>
#include "../core/include/utils/math_util.h"
#include "../core/include/utils/moving_average.h"
#include "vex.h"

class FeedForward
{
    public:

    /**
     * kS - Coefficient to overcome static friction: the point at which the motor *starts* to move.
     * kV - Veclocity coefficient: the power required to keep the mechanism in motion. Multiplied by the requested velocity.
     * kA - Acceleration coefficient: the power required to change the mechanism's speed. Multiplied by the requested acceleration.
     * kG - Gravity coefficient: only needed for lifts. The power required to overcome gravity and stay at steady state.
     *      Should relate to acceleration due to gravity and mass of the lift.
     */
    typedef struct
    {
        double kS, kV, kA, kG;
    } ff_config_t;

    
    FeedForward(ff_config_t &cfg) : cfg(cfg) {}

    /**
     * @brief Perform the feedforward calculation
     * 
     * This calculation is the equation:
     * F = kG + kS*sgn(v) + kV*v + kA*a
     * 
     * @param v Requested velocity of system
     * @param a Requested acceleration of system
     * @return A feedforward that should closely represent the system if tuned correctly
     */
    double calculate(double v, double a)
    {
        return (cfg.kS * (v > 0 ? 1 : v < 0 ? -1 : 0)) + (cfg.kV * v) + (cfg.kA * a) + cfg.kG;
    }

    private:

    ff_config_t &cfg;

};


FeedForward::ff_config_t tune_feedforward(vex::motor_group &motor, double pct, double duration)
{
    FeedForward::ff_config_t out = {};
    
    double start_pos = motor.rotation(vex::rotationUnits::rev);

    // ========== kS Tuning =========
    // Start at 0 and slowly increase the power until the robot starts moving
    double power = 0;
    while(fabs(motor.rotation(vex::rotationUnits::rev) - start_pos) < 0.05)
    {
        motor.spin(vex::directionType::fwd, power, vex::voltageUnits::volt);
        power += 0.001;
        vexDelay(100);
    }
    out.kS = power;
    motor.stop();


    // ========== kV / kA Tuning =========

    std::vector<std::pair<double, double>> vel_data_points; // time, velocity
    std::vector<std::pair<double, double>> accel_data_points; // time, accel

    double max_speed = 0;
    vex::timer tmr;
    double time;

    MovingAverage vel_ma(3);
    MovingAverage accel_ma(3);

    // Move the robot forward at a fixed percentage for X seconds while taking velocity and accel measurements
    do
    {
        double last_time = time;
        time = tmr.time(vex::sec);
        double dt = time - last_time;

        vel_ma.add_entry(motor.velocity(vex::velocityUnits::rpm));
        accel_ma.add_entry(motor.velocity(vex::velocityUnits::rpm)/dt);

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

    motor.stop();

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