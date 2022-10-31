#include "../core/include/subsystems/odometry/odometry_3wheel.h"
#include "../core/include/utils/vector2d.h"

Odometry3Wheel::Odometry3Wheel(CustomEncoder &lside_fwd, CustomEncoder &rside_fwd, CustomEncoder &off_axis, odometry3wheel_cfg_t &cfg, bool is_async) 
: OdometryBase(is_async), lside_fwd(lside_fwd), rside_fwd(rside_fwd), off_axis(off_axis), cfg(cfg)
{}

/**
 * Update the current position of the robot once, using the current state of
 * the encoders and the previous known location
 * 
 * @return the robot's updated position
 */
position_t Odometry3Wheel::update()
{
    static double lside_old=0, rside_old=0, offax_old=0;

    double lside = lside_fwd.position(deg);
    double rside = rside_fwd.position(deg);
    double offax = off_axis.position(deg);

    double lside_delta = lside - lside_old;
    double rside_delta = rside - rside_old;
    double offax_delta = offax - offax_old;

    lside_old = lside;
    rside_old = rside;
    offax_old = offax;

    mut.lock();
    current_pos = calculate_new_pos(lside_delta, rside_delta, offax_delta, current_pos, cfg);
    mut.unlock();

    return current_pos;
}

/**
 * Calculation method for the robot's new position using the change in encoders, the old position, and the robot's configuration.
 * This uses a series of arclength formulae for finding distance driven and change in angle.
 * Then vector math is used to combine it with the robot's old position data
 * 
 * @param lside_delta_deg Left encoder change in rotation, in degrees
 * @param rside_delta_deg Right encoder change in rotation, in degrees
 * @param offax_delta_deg Off-axis (perpendicular) encoder change in rotation, in degrees
 * @param old_pos Robot's old position, for integration
 * @param cfg Data on robot's configuration (wheel diameter, wheelbase, off-axis distance from center)
 * @return The robot's new position (x, y, rot) 
 */
position_t Odometry3Wheel::calculate_new_pos(double lside_delta_deg, double rside_delta_deg, double offax_delta_deg, position_t old_pos, odometry3wheel_cfg_t cfg)
{
    position_t retval = {};

    // Arclength formula for encoder degrees -> single wheel distance driven
    double lside_dist = (cfg.wheel_diam / 2.0) * deg2rad(lside_delta_deg);
    double rside_dist = (cfg.wheel_diam / 2.0) * deg2rad(rside_delta_deg);
    double offax_dist = (cfg.wheel_diam / 2.0) * deg2rad(offax_delta_deg);
    
    // Inverse arclength formula for arc distance driven -> robot angle
    double delta_angle_rad = 2 * (rside_dist - lside_dist) / cfg.wheelbase_dist;
    double delta_angle_deg = rad2deg(delta_angle_rad);

    // Distance along the robot's local Y axis (forward/backward)
    double dist_local_y = (lside_dist + rside_dist) / 2.0;

    // Distance along the robot's local X axis (right/left)
    double dist_local_x = offax_dist - (delta_angle_rad * cfg.off_axis_center_dist);

    // Change in displacement as a vector, on the local coordinate system (+y = robot fwd)
    Vector2D local_displacement({.x=dist_local_x, .y=dist_local_y});

    // Rotate the local displacement to match the old robot's rotation
    double global_dir_rad = fmod(local_displacement.get_dir() + deg2rad(old_pos.rot), 2*PI);
    Vector2D global_displacement(global_dir_rad, local_displacement.get_mag());

    // Tack on the position change to the old position
    Vector2D old_pos_vec({.x=old_pos.x, .y=old_pos.y});
    Vector2D new_pos_vec = old_pos_vec + global_displacement;

    retval.x = new_pos_vec.get_x();
    retval.y = new_pos_vec.get_y();
    retval.rot = fmod(old_pos.rot + delta_angle_deg, 360);
    
    return retval;
}
