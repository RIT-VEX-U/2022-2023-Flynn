#include "../core/include/subsystems/odometry/odometry_tank.h"
#include "../core/include/utils/vector2d.h"

/**
* Initialize the Odometry module, calculating position from the drive motors.
* @param left_side The left motors 
* @param right_side The right motors
* @param config the specifications that supply the odometry with descriptions of the robot. See robot_specs_t for what is contained
* @param imu The robot's inertial sensor. If not included, rotation is calculated from the encoders.
* @param is_async If true, position will be updated in the background continuously. If false, the programmer will have to manually call update().
*/
OdometryTank::OdometryTank(vex::motor_group &left_side, vex::motor_group &right_side, robot_specs_t &config, vex::inertial *imu, bool is_async)
: OdometryBase(is_async), left_side(&left_side), right_side(&right_side), left_enc(NULL), right_enc(NULL), imu(imu), config(config)
{
}

/**
* Initialize the Odometry module, calculating position from the drive motors.
* @param left_enc The left motors 
* @param right_enc The right motors
* @param config the specifications that supply the odometry with descriptions of the robot. See robot_specs_t for what is contained
* @param imu The robot's inertial sensor. If not included, rotation is calculated from the encoders.
* @param is_async If true, position will be updated in the background continuously. If false, the programmer will have to manually call update().
*/
OdometryTank::OdometryTank(CustomEncoder &left_enc, CustomEncoder &right_enc, robot_specs_t &config, vex::inertial *imu, bool is_async)
: OdometryBase(is_async), left_side(NULL), right_side(NULL), left_enc(&left_enc), right_enc(&right_enc), imu(imu), config(config)
{
}

/**
 * Resets the position and rotational data to the input.
 * 
 */
void OdometryTank::set_position(const units::pose_t &newpos)
{
  mut.lock();
  rotation_offset = newpos.rot - (current_pos.rot - rotation_offset);
  mut.unlock();

  OdometryBase::set_position(newpos);
}

/**
 * Update, store and return the current position of the robot. Only use if not initializing
 * with a separate thread.
 */
units::pose_t OdometryTank::update()
{
  units::Angle lside_revs = 0_rev, rside_revs = 0_rev;

  if (left_side != NULL && right_side != NULL) {
    lside_revs = 1_rev * left_side->position(vex::rotationUnits::rev) / config.odom_gear_ratio;
    rside_revs = 1_rev * right_side->position(vex::rotationUnits::rev) / config.odom_gear_ratio;
  } else if (left_enc != NULL && right_enc != NULL) {
    lside_revs = 1_rev * left_enc->position(vex::rotationUnits::rev) / config.odom_gear_ratio;
    rside_revs = 1_rev * right_enc->position(vex::rotationUnits::rev) / config.odom_gear_ratio;
  }

  units::Angle angle = 0_deg;

  // If the IMU data was passed in, use it for rotational data
  if (imu == NULL || imu->installed() == false) {
    // Get the difference in distance driven between the two sides
    // Uses the absolute position of the encoders, so resetting them will result
    // in a bad angle. Get the arclength of the turning circle of the robot
    units::Length distance_diff
        = units::arc_length((rside_revs - lside_revs), config.odom_wheel_diam / 2.0);

    // Use the arclength formula to calculate the angle. Add 90 to make "0
    // degrees" to starboard
    angle = (1_rad * distance_diff / config.dist_between_wheels) + 90_deg;

    // printf("angle: %f, ", (180.0 / PI) * (distance_diff /
    // config.dist_between_wheels));

  } else {
    // Translate "0 forward and clockwise positive" to "90 forward and CCW
    // negative"
    angle = (-imu->rotation(vex::rotationUnits::deg) + 90) * 1_deg;
  }

  // Offset the angle, if we've done a set_position
  angle += rotation_offset;

  // Limit the angle betwen 0 and 360.
  // fmod (floating-point modulo) gets it between -359 and +359, so tack on
  // another 360 if it's negative.
  angle = mod(angle, 360.0_deg);
  if (angle < 0_deg)
    angle += 360_deg;

  current_pos = calculate_new_pos(config, current_pos, lside_revs, rside_revs, angle);

  static units::pose_t last_pos = current_pos;
  static timer tmr;
  bool update_vel_accel = tmr.time(sec) > 0.1;

  // This loop runs too fast. Only check at LEAST every 1/10th sec
  if (update_vel_accel) {
    static units::Speed last_speed = 0_inps;
    static units::AngularSpeed last_ang_speed = 0_rpm;

    // Calculate robot velocity
    speed = pos_diff(current_pos, last_pos) / (tmr.time(sec) * 1_s);

    // Calculate robot acceleration
    accel = (speed - last_speed) / (tmr.time(sec) * 1_s);

    // Calculate robot angular velocity (deg/sec)
    ang_speed_deg = units::smallest_angle(current_pos.rot, last_pos.rot) / (tmr.time(sec) * 1_s);

    // Calculate robot angular acceleration (deg/sec^2)
    ang_accel_deg = (ang_speed_deg - last_ang_speed) / (tmr.time(sec) * 1_s);

    tmr.reset();
    last_pos = current_pos;
    last_speed = speed;
    last_ang_speed = ang_speed_deg;
  }

  return current_pos;
}

/**
 * Using information about the robot's mechanical structure and sensors, calculate a new position
 * of the robot, relative to when this method was previously ran.
 */
units::pose_t OdometryTank::calculate_new_pos(robot_specs_t &config, units::pose_t &curr_pos,
                                              units::Angle lside_revs, units::Angle rside_revs,
                                              units::Angle angle)
{
  units::pose_t new_pos;

  static units::Angle stored_lside_revs = lside_revs;
  static units::Angle stored_rside_revs = rside_revs;

  // Convert the revolutions into "change in distance", and average the values
  // for a "distance driven"
  units::Length rside_diff
      = units::arc_length((rside_revs - stored_rside_revs), config.odom_wheel_diam / 2.0);
  units::Length lside_diff
      = units::arc_length((lside_revs - stored_lside_revs), config.odom_wheel_diam / 2.0);
  units::Length dist_driven = (lside_diff + rside_diff) / 2.0;

  // Create a vector from the change in distance in the current direction of
  // the robot
  // deg2rad((smallest_angle(curr_pos.rot, angle_deg)/2 + curr_pos.rot,
  // dist_driven)
  Vector2D chg_vec(angle.Convert(units::radian), dist_driven.Convert(units::inch));

  // Create a vector from the current position in reference to X,Y=0,0
  point_t curr_point = {.x = curr_pos.x.Convert(units::inch), .y = curr_pos.y.Convert(units::inch)};
  Vector2D curr_vec(curr_point);

  // Tack on the "difference" vector to the current vector
  Vector2D new_vec = curr_vec + chg_vec;

  new_pos.x = new_vec.get_x() * 1_in;
  new_pos.y = new_vec.get_y() * 1_in;
  new_pos.rot = angle;

  // Store the left and right encoder values to find the difference in the next
  // iteration
  stored_lside_revs = lside_revs;
  stored_rside_revs = rside_revs;

  return new_pos;
}
