#include "../core/include/subsystems/tank_drive.h"
#include "../core/include/utils/geometry.h"
#include "../core/include/utils/math_util.h"

TankDrive::TankDrive(motor_group &left_motors, motor_group &right_motors, robot_specs_t &config, OdometryBase *odom)
    : left_motors(left_motors), right_motors(right_motors), correction_pid(config.correction_pid), odometry(odom), config(config)
{
  drive_default_feedback = config.drive_feedback;
  turn_default_feedback = config.turn_feedback;
}

/**
 * Reset the initialization for autonomous drive functions
 */
void TankDrive::reset_auto()
{
  func_initialized = false;
}

/**
 * Stops rotation of all the motors using their "brake mode"
 */
void TankDrive::stop()
{
  left_motors.stop();
  right_motors.stop();
}

/**
 * Drive the robot using differential style controls. left_motors controls the left motors,
 * right_motors controls the right motors.
 * 
 * left_motors and right_motors are in "percent": -1.0 -> 1.0
 */
void TankDrive::drive_tank(units::Voltage left, units::Voltage right)
{
  left_motors.spin(directionType::fwd, left.Convert(units::volt),
                   voltageUnits::volt);
  right_motors.spin(directionType::fwd, right.Convert(units::volt),
                    voltageUnits::volt);
}

/**
 * Drive the robot using arcade style controls. forward_back controls the linear motion,
 * left_right controls the turning.
 * 
 * left_motors and right_motors are in "percent": -1.0 -> 1.0
 */
void TankDrive::drive_arcade(double forward_back, double left_right, int power)
{
  forward_back = modify_inputs(forward_back, power);
  left_right = modify_inputs(left_right, power);

  double left = forward_back + left_right;
  double right = forward_back - left_right;

  left_motors.spin(directionType::fwd, left * 12, voltageUnits::volt);
  right_motors.spin(directionType::fwd, right * 12, voltageUnits::volt);
}

/**
 * Use odometry to drive forward a certain distance using a custom feedback controller
 *
 * Returns whether or not the robot has reached it's destination.
 * @param inches     the distance to drive forward
 * @param dir        the direction we want to travel forward and backward
 * @param feedback   the custom feedback controller we will use to travel. controls the rate at which we accelerate and drive.
 * @param max_speed  the maximum percentage of robot speed at which the robot will travel. 1 = full power
 */
bool TankDrive::drive_forward(double inches, directionType dir,
                              Drive_Feedback &feedback, double max_speed)
{
  static pose_t pos_setpt;

  // We can't run the auto drive function without odometry
  if(odometry == NULL)
  {
    fprintf(stderr, "Odometry is NULL. Unable to run drive_forward()\n");
    fflush(stderr);
    return true;
  }

  // Generate a point X inches forward of the current position, on first startup
  if (!func_initialized)
  {
    units::pose_t cur_pos = odometry->get_position();

    // forwards is positive Y axis, backwards is negative
    if (dir == directionType::rev)
      inches = -fabs(inches);
    else
      inches = fabs(inches);

    // Use vector math to get an X and Y
    Vector2D cur_pos_vec(
        {.x = cur_pos.x.Convert(units::inch), .y = cur_pos.y.Convert(units::inch)});
    Vector2D delta_pos_vec(deg2rad(cur_pos.rot.Convert(units::radian)), inches);
    Vector2D setpt_vec = cur_pos_vec + delta_pos_vec;

    // Save the new X and Y values
    pos_setpt = {.x=setpt_vec.get_x(), .y=setpt_vec.get_y()};

  }

  // Call the drive_to_point with updated point values
  return drive_to_point(pos_setpt.x * 1_in, pos_setpt.y * 1_in, dir, feedback, max_speed);
}
/**
 * Autonomously drive the robot forward a certain distance
 * 
 * 
 * @param inches      degrees by which we will turn relative to the robot (+) turns ccw, (-) turns cw
 * @param dir        the direction we want to travel forward and backward
 * @param max_speed   the maximum percentage of robot speed at which the robot will travel. 1 = full power
 * @return true if we have finished driving to our point
 */
bool TankDrive::drive_forward(double inches, directionType dir, double max_speed)
{
  if(drive_default_feedback != NULL)
    return drive_forward(inches, dir, *drive_default_feedback, max_speed);

  printf("tank_drive.cpp: Cannot run drive_forward without a feedback controller!\n");
  fflush(stdout);
  return true;
}

/**
 * Autonomously turn the robot X degrees to counterclockwise (negative for
 * clockwise), with a maximum motor speed of percent_speed (-1.0 -> 1.0)
 *
 * Uses the specified feedback for it's control.
 *
 * @param amt     angle by which we will turn relative to the robot (+) turns
 * ccw, (-) turns cw
 * @param feedback    the feedback controller we will use to travel. controls
 * the rate at which we accelerate and drive.
 * @param limit  the maximum voltage that the robot will be commanded 12_v =
 * full power
 * @return true if we have turned our target number of degrees
 */
bool TankDrive::turn_relative(units::Angle amt, Turn_Feedback &feedback,
                              units::Voltage limit)
{
  // We can't run the auto drive function without odometry
  if(odometry == NULL)
  {
    fprintf(stderr, "Odometry is NULL. Unable to run drive_forward()\n");
    fflush(stderr);
    return true;
  }

  static units::Angle target_heading = 0_deg;

  // On the first run of the funciton, reset the gyro position and PID
  if (!func_initialized)
  {
    units::Angle start_heading = odometry->get_position().rot;
    target_heading = start_heading + amt;
  }

  return turn_to_heading(target_heading);
}

/**
 * Autonomously turn the robot X degrees to counterclockwise (negative for
 * clockwise), with a maximum motor speed of percent_speed (-1.0 -> 1.0)
 *
 * Uses the defualt turning feedback of the drive system.
 *
 * @param amt angle by which we will turn relative to the robot (+) turns ccw,
 * (-) turns cw
 * @param limit  the maximum voltage that the robot will be commanded 12_v =
 * full power
 * @return true if we turned te target number of degrees
 */
bool TankDrive::turn_relative(units::Angle amt, units::Voltage limit)
{
  if(turn_default_feedback != NULL)
    return turn_relative(amt, *turn_default_feedback, limit);

  printf("tank_drive.cpp: Cannot run turn_degrees without a feedback "
         "controller!\n");
  fflush(stdout);
  return true;
}

/**
  * Use odometry to automatically drive the robot to a point on the field.
  * X and Y is the final point we want the robot.
  *
  * Returns whether or not the robot has reached it's destination.
  * @param x          the x position of the target
  * @param y          the y position of the target
  * @param dir        the direction we want to travel forward and backward
  * @param feedback   the feedback controller we will use to travel. controls the rate at which we accelerate and drive.
  * @param max_speed  the maximum percentage of robot speed at which the robot will travel. 1 = full power
  * @return true if we have reached our target point
  */
bool TankDrive::drive_to_point(units::Length x, units::Length y, vex::directionType dir,
                               Drive_Feedback &feedback, double max_speed)
{
  // We can't run the auto drive function without odometry
  if(odometry == NULL)
  {
    fprintf(stderr, "Odometry is NULL. Unable to run drive_forward()\n");
    fflush(stderr);
    return true;
  }

  if (!func_initialized) {

    units::Length initial_dist = OdometryBase::pos_diff(odometry->get_position(), {.x = x, .y = y});

    // Reset the control loops
    correction_pid.init(0_deg, 0_deg);
    feedback.init(-initial_dist, 0_in);

    correction_pid.set_limits(-12_v, 12_v);
    feedback.set_limits(-12_v, 12_v);

    func_initialized = true;
  }

  // Store the initial position of the robot
  units::pose_t current_pos = odometry->get_position();
  units::pose_t end_pos = {.x = x, .y = y, .rot = current_pos.rot};

  // Create a point (and vector) to get the direction
  point_t pos_diff_pt = {.x = (x - current_pos.x).Convert(units::inch),
                         .y = (y - current_pos.y).Convert(units::inch)};

  Vector2D point_vec(pos_diff_pt);

  // Get the distance between 2 points
  units::Length dist_left = OdometryBase::pos_diff(current_pos, end_pos);

  double sign = 1;

  // Make an imaginary perpendicualar line to that between the bot and the point. If the point is
  // behind that line, and the point is within the robot's radius, use negatives for feedback
  // control.

  units::Angle angle_to_point
      = 1_rad * atan2((y - current_pos.y).Convert(1_in), (x - current_pos.x).Convert(1_in));
  units::Angle angle = mod(current_pos.rot - angle_to_point, 360.0_deg);

  // Normalize the angle between 0 and 360
  angle = units::wrap_angle(angle);

  // If the angle is behind the robot, report negative.
  if (dir == directionType::fwd && angle > 90_deg && angle < 270_deg)
    sign = -1;
  else if (dir == directionType::rev && (angle < 90_deg || angle > 270_deg))
    sign = -1;

  if (abs(dist_left) < config.drive_correction_cutoff) {
    // When inside the robot's cutoff radius, report the distance to the point along the robot's forward axis,
    // so we always "reach" the point without having to do a lateral translation
    dist_left = dist_left * fabs(cos(angle * PI / 180.0));
  }

  // Get the heading difference between where we are and where we want to be
  // Optimize that heading so we don't turn clockwise all the time
  units::Angle heading = 1_deg * rad2deg(point_vec.get_dir());
  units::Angle delta_heading = 0_deg;

  // Going backwards "flips" the robot's current heading
  if (dir == directionType::fwd)
    delta_heading = units::smallest_angle(current_pos.rot, heading);
  else
    delta_heading = units::smallest_angle(current_pos.rot - 180_deg, heading);

  // Update the PID controllers with new information
  correction_pid.update(delta_heading);
  feedback.update(dist_left * -sign);

  // Disable correction when we're close enough to the point
  units::Voltage correction = 0_v;
  if (is_pure_pursuit || abs(dist_left) > config.drive_correction_cutoff)
    correction = correction_pid.get();

  // Reverse the drive_pid output if we're going backwards
  units::Voltage drive_pid_rval;
  if(dir == directionType::rev)
    drive_pid_rval = -feedback.get();
  else
    drive_pid_rval = feedback.get();

  // Combine the two pid outputs
  units::Voltage lside = drive_pid_rval + correction;
  units::Voltage rside = drive_pid_rval - correction;

  using namespace unit_literals;

  // limit the outputs between -1 and +1
  lside = clamp(lside, -12_v, 12_v);
  rside = clamp(rside, -12_v, 12_v);

  drive_tank(lside, rside);

  // Check if the robot has reached it's destination
  if(feedback.is_on_target())
  {
    stop();
    func_initialized = false;
    stop();
    return true;
  }

  return false;
}

/**
  * Use odometry to automatically drive the robot to a point on the field.
  * X and Y is the final point we want the robot.
  * Here we use the default feedback controller from the drive_sys
  *
  * Returns whether or not the robot has reached it's destination.
  * @param x          the x position of the target
  * @param y          the y position of the target
  * @param dir        the direction we want to travel forward and backward
  * @param max_speed  the maximum percentage of robot speed at which the robot will travel. 1 = full power
  * @return true if we have reached our target point
  */
bool TankDrive::drive_to_point(units::Length x, units::Length y, vex::directionType dir,
                               double max_speed)
{
  if(drive_default_feedback != NULL)
    return this->drive_to_point(x, y, dir, *drive_default_feedback, max_speed);

  printf("tank_drive.cpp: Cannot run drive_to_point without a feedback controller!\n");
  fflush(stdout);
  return true;
}

/**
 * Turn the robot in place to an exact heading relative to the field.
 * 0 is forward.
 *
 * @param heading the heading to which we will turn
 * @param feedback    the feedback controller we will use to travel. controls
 * the rate at which we accelerate and drive.
 * @param limit  the maximum voltage that the robot will be commanded 12_v =
 * full power
 * @return true if we have reached our target heading
 */
bool TankDrive::turn_to_heading(units::Angle heading, Turn_Feedback &feedback,
                                units::Voltage limit)
{
  // We can't run the auto drive function without odometry
  if(odometry == NULL)
  {
    fprintf(stderr, "Odometry is NULL. Unable to run drive_forward()\n");
    fflush(stderr);
    return true;
  }

  if(!func_initialized)
  {
    units::Angle initial_delta = units::smallest_angle(odometry->get_position().rot, heading);
    feedback.init(-initial_delta, 0_deg);
    feedback.set_limits(-abs(limit), abs(limit));

    func_initialized = true;
  }

  // Get the difference between the new heading and the current, and decide whether to turn left or right.
  units::Angle delta_heading = units::smallest_angle(odometry->get_position().rot, heading);
  feedback.update(-delta_heading);

  fflush(stdout);

  drive_tank(-feedback.get(), feedback.get());

  // When the robot has reached it's angle, return true.
  if(feedback.is_on_target())
  {
    func_initialized = false;
    stop();
    return true;
  }

  return false;
}
/**
 * Turn the robot in place to an exact heading relative to the field.
 * 0 is forward. Uses the defualt turn feedback of the drive system
 *
 * @param heading the heading to which we will turn
 * @param limit  the maximum voltage the robot will be commanded to travel at.
 * @return true if we have reached our target heading
 */
bool TankDrive::turn_to_heading(units::Angle heading, units::Voltage limit)
{
  if(turn_default_feedback != NULL)
    return turn_to_heading(heading, *turn_default_feedback, limit);

  printf("tank_drive.cpp: Cannot run turn_to_heading without a feedback controller!\n");
  fflush(stdout);
  return true;
}

/**
 * Modify the inputs from the controller by squaring / cubing, etc
 * Allows for better control of the robot at slower speeds
 * @param input the input signal -1 -> 1
 * @param power the power to raise the signal to
 * @return input^power accounting for any sign issues that would arise with this naive solution
 */
double TankDrive::modify_inputs(double input, int power)
{
  return sign(input)* pow(std::abs(input), power);
}

bool TankDrive::pure_pursuit(std::vector<PurePursuit::hermite_point> path,
                             directionType dir, double radius, double res,
                             Drive_Feedback &feedback, double max_speed)
{
  is_pure_pursuit = true;
  std::vector<point_t> smoothed_path = PurePursuit::smooth_path_hermite(path, res);

  point_t lookahead = PurePursuit::get_lookahead(
      smoothed_path,
      {odometry->get_position().x.Convert(1_in), odometry->get_position().y.Convert(1_in)}, radius);
  //printf("%f\t%f\n", odometry->get_position().x, odometry->get_position().y); 
  //printf("%f\t%f\n", lookahead.x, lookahead.y);
  bool is_last_point = (path.back().x == lookahead.x) && (path.back().y == lookahead.y);

  if(is_last_point)
    is_pure_pursuit = false;

  bool retval = drive_to_point(lookahead.x * 1_in, lookahead.y * 1_in, dir, feedback, max_speed);

  if(is_last_point)
    return retval;

  return false;
}
