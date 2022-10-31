#pragma once
#include "../core/include/subsystems/odometry/odometry_base.h"
#include "../core/include/subsystems/custom_encoder.h"

/**
 * Odometry3Wheel
 * 
 * This class handles the code for a standard 3-pod odometry setup, where there are 3 "pods" made up of undriven
 * (dead) wheels connected to encoders in the following configuration:
 * 
 *  ---------------
 *  |             |
 *  |             |
 *  | ||   X   || |
 *  |             |
 *  |     ===     |
 *  ---------------
 * 
 * Where X is the center of rotation. The robot will monitor the changes in rotation of these wheels and calculate
 * the robot's X, Y and rotation on the field.
 * 
 * This is a "set and forget" class, meaning once the object is created, the robot will immediately begin
 * tracking it's movement in the background.
 * 
 * @author Ryan McGee
 * @date Oct 31 2022
 * 
 */
class Odometry3Wheel : public OdometryBase
{
    public:

    typedef struct
    {
        double wheelbase_dist;
        double off_axis_center_dist;
        double wheel_diam;
        
    } odometry3wheel_cfg_t;
    
    /**
     * Construct a new Odometry 3 Wheel object
     * 
     * @param lside_fwd left-side encoder reference
     * @param rside_fwd right-side encoder reference
     * @param off_axis off-axis (perpendicular) encoder reference
     * @param cfg robot odometry configuration
     * @param is_async true to constantly run in the background
     */
    Odometry3Wheel(CustomEncoder &lside_fwd, CustomEncoder &rside_fwd, CustomEncoder &off_axis, odometry3wheel_cfg_t &cfg, bool is_async=true);

    /**
     * Update the current position of the robot once, using the current state of
     * the encoders and the previous known location
     * 
     * @return the robot's updated position
     */
    position_t update() override;

    private:

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
    static position_t calculate_new_pos(double lside_delta_deg, double rside_delta_deg, double offax_delta_deg, position_t old_pos, odometry3wheel_cfg_t cfg);

    CustomEncoder &lside_fwd, &rside_fwd, &off_axis;
    odometry3wheel_cfg_t &cfg;


};