#pragma once

#include "../core/include/robot_specs.h"
#include "../core/include/subsystems/custom_encoder.h"
#include "../core/include/subsystems/odometry/odometry_base.h"
#include "../core/include/utils/geometry.h"
#include "../core/include/utils/unit_types.h"
#include "../core/include/utils/units.h"

static int background_task(void *odom_obj);

/**
 * OdometryTank defines an odometry system for a tank drivetrain
 * This requires encoders in the same orientation as the drive wheels
 * Odometry is a "start and forget" subsystem, which means once it's created and
 * configured, it will constantly run in the background and track the robot's X,
 * Y and rotation coordinates.
 */
class OdometryTank : public OdometryBase
{
public:
    /**
    * Initialize the Odometry module, calculating position from the drive motors.
    * @param left_side The left motors 
    * @param right_side The right motors
    * @param config the specifications that supply the odometry with descriptions of the robot. See robot_specs_t for what is contained
    * @param imu The robot's inertial sensor. If not included, rotation is calculated from the encoders.
    * @param is_async If true, position will be updated in the background continuously. If false, the programmer will have to manually call update().
    */
    OdometryTank(vex::motor_group &left_side, vex::motor_group &right_side, robot_specs_t &config, vex::inertial *imu=NULL, bool is_async=true);

    /**
    * Initialize the Odometry module, calculating position from the drive motors.
    * @param left_enc The left motors 
    * @param right_enc The right motors
    * @param config the specifications that supply the odometry with descriptions of the robot. See robot_specs_t for what is contained
    * @param imu The robot's inertial sensor. If not included, rotation is calculated from the encoders.
    * @param is_async If true, position will be updated in the background continuously. If false, the programmer will have to manually call update().
    */

    OdometryTank(CustomEncoder &left_enc, CustomEncoder &right_enc, robot_specs_t &config, vex::inertial *imu=NULL, bool is_async=true);

    /**
     * Update the current position on the field based on the sensors
     * @return the position that odometry has calculated itself to be at
     */
    units::pose_t update() override;

    /**
     * set_position tells the odometry to place itself at a position
     * @param newpos the position the odometry will take
    */
    void set_position(const units::pose_t &newpos = zero_pos) override;

  private:
    /**
     * Get information from the input hardware and an existing position, and calculate a new current position
     */
    static units::pose_t calculate_new_pos(robot_specs_t &config,
                                           units::pose_t &stored_info,
                                           units::Angle lside_diff,
                                           units::Angle rside_diff,
                                           units::Angle angle_deg);

    vex::motor_group *left_side, *right_side;
    CustomEncoder *left_enc, *right_enc;
    vex::inertial *imu;
    robot_specs_t &config;

    units::Angle rotation_offset = 0_deg;
};
