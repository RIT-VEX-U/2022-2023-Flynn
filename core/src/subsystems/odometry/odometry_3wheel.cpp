#include "../core/include/subsystems/odometry/odometry_3wheel.h"
#include "../core/include/utils/vector2d.h"

Odometry3Wheel::Odometry3Wheel(vex::encoder &lside_fwd, vex::encoder &rside_fwd, vex::encoder &off_axis, odometry3wheel_cfg_t &cfg, bool is_async) 
: OdometryBase(is_async), lside_fwd(lside_fwd), rside_fwd(rside_fwd), off_axis(off_axis), cfg(cfg)
{

  current_pos = position_t{.x = 0, .y = 0, .rot = 90};
}



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
    //printf("opos(%.2f, %.2f) r: %.2f\t", old_pos.x, old_pos.y, old_pos.rot);      
    
    position_t retval = {};

    // Arclength formula for encoder degrees -> single wheel distance driven
    double lside_dist = (cfg.wheel_diam / 2.0) * deg2rad(lside_delta_deg);
    double rside_dist = (cfg.wheel_diam / 2.0) * deg2rad(rside_delta_deg);
    double offax_dist = (cfg.wheel_diam / 2.0) * deg2rad(offax_delta_deg);

    // Inverse arclength formula for arc distance driven -> robot angle
    double delta_angle_rad = (rside_dist - lside_dist) / cfg.wheelbase_dist;
    double delta_angle_deg = rad2deg(delta_angle_rad);

    // Distance along the robot's local X axis (drive wheels forward/backward)
    double dist_local_x = (lside_dist + rside_dist) / 2.0;

    // Distance along the robot's local Y axis (drive wheels right/left)
    double dist_local_y = offax_dist - (delta_angle_rad * cfg.off_axis_center_dist);

    // Change in displacement as a vector, on the local coordinate system (+x = robot fwd)
    Vector2D local_displacement({.x=dist_local_x, .y=dist_local_y});

    // Rotate the local displacement to match the old robot's rotation
    double global_dir_rad = fmod(local_displacement.get_dir() + deg2rad(old_pos.rot), 2*PI);
    Vector2D global_displacement(global_dir_rad, local_displacement.get_mag());


    // Tack on the position change to the old position
    Vector2D old_pos_vec({.x=old_pos.x, .y=old_pos.y});
    Vector2D new_pos_vec = old_pos_vec + global_displacement;

    //printf("oposV(%.2f, %.2f)\t", old_pos_vec.get_x(), old_pos_vec.get_y());      


    retval.x = new_pos_vec.get_x();
    retval.y = new_pos_vec.get_y();
    retval.rot = fmod(old_pos.rot + delta_angle_deg, 360);
    
    //printf("retval(%.2f, %.2f) r: %.2f\n", retval.x, retval.y, retval.rot);      
    //fflush(stdout);

    return retval;
}

/**
 * A guided tuning process to automatically find tuning parameters.
 * This method is blocking, and returns when tuning has finished. Follow
 * the instructions on the controller to complete the tuning process
 * 
 * It is assumed the gear ratio and encoder PPR have been set correctly
 * 
 * @param con Controller reference, for screen and button control
 * @param drive Drivetrain reference for robot control
 */
void Odometry3Wheel::tune(vex::controller &con, TankDrive &drive)
{

    // TODO check if all the messages fit on the controller screen
    // STEP 1: Align robot and reset odometry
    con.Screen.clearScreen();
    con.Screen.setCursor(1,1);
    con.Screen.print("Wheel Diameter Test");
    con.Screen.newLine();
    con.Screen.print("Align robot with ref");
    con.Screen.newLine();
    con.Screen.newLine();
    con.Screen.print("Press A to continue");
    while(!con.ButtonA.pressing()) { vexDelay(20); }

    double old_lval = lside_fwd.position(deg);
    double old_rval = rside_fwd.position(deg);

    // Step 2: Drive robot a known distance
    con.Screen.clearLine(2);
    con.Screen.setCursor(2,1);
    con.Screen.print("Drive or Push robot");
    con.Screen.newLine();
    con.Screen.print("10 feet (5 tiles)");
    con.Screen.newLine();
    con.Screen.print("Press A to continue");
    while(!con.ButtonA.pressing())
    {
        drive.drive_arcade(con.Axis3.position() / 100.0, con.Axis1.position() / 100.0);
        vexDelay(20);
    }

    // Wheel diameter is ratio of expected distance / measured distance
    double avg_deg = ((lside_fwd.position(deg) - old_lval) + (rside_fwd.position(deg) - old_rval)) / 2.0;
    double measured_dist = 0.5 * deg2rad(avg_deg); // Simulate diam=1", radius=1/2"
    double found_diam = 120.0 / measured_dist; 

    // Step 3: Reset alignment for turning test
    con.Screen.clearScreen();
    con.Screen.setCursor(1,1);
    con.Screen.print("Wheelbase Test");
    con.Screen.newLine();
    con.Screen.print("Align robot with ref");
    con.Screen.newLine();
    con.Screen.newLine();
    con.Screen.print("Press A to continue");
    while(!con.ButtonA.pressing()) { vexDelay(20); }
    
    old_lval = lside_fwd.position(deg);
    old_rval = rside_fwd.position(deg);
    double old_offax = off_axis.position(deg);

    con.Screen.setCursor(2,1);
    con.Screen.clearLine();
    con.Screen.print("Turn robot 10");
    con.Screen.newLine();
    con.Screen.print("times in place");
    con.Screen.newLine();
    con.Screen.print("Press A to continue");
    while(!con.ButtonA.pressing())
    {
        drive.drive_arcade(0, con.Axis1.position() / 100.0);
        vexDelay(20);
    }

    double lside_dist = deg2rad(lside_fwd.position(deg) - old_lval) * (found_diam / 2.0);
    double rside_dist = deg2rad(rside_fwd.position(deg) - old_rval) * (found_diam / 2.0);
    double offax_dist = deg2rad(off_axis.position(deg) - old_offax) * (found_diam / 2.0);

    double expected_angle = 10 * (2*PI);
    double found_wheelbase = fabs(rside_dist - lside_dist) / expected_angle;
    double found_offax_center_dist = offax_dist / expected_angle;

    con.Screen.clearScreen();
    con.Screen.setCursor(1,1);
    con.Screen.print("Diam: %f", found_diam);
    con.Screen.newLine();
    con.Screen.print("whlbase: %f", found_wheelbase);
    con.Screen.newLine();
    con.Screen.print("offax: %f", found_offax_center_dist);

    printf("Tuning completed.\n  Wheel Diameter: %f\n  Wheelbase: %f\n  Off Axis Distance: %f\n", found_diam, found_wheelbase, found_offax_center_dist);
}
