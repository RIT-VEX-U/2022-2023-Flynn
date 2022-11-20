#include "../core/include/subsystems/tank_drive.h"
#include "../core/include/utils/math_util.h"

TankDrive::TankDrive(motor_group &left_motors, motor_group &right_motors, robot_specs_t &config, OdometryTank *odom)
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
void TankDrive::drive_tank(double left, double right, int power, bool isdriver)
{
  left = modify_inputs(left, power);
  right = modify_inputs(right, power);

  if(isdriver == false)
  {
    left_motors.spin(directionType::fwd, left * 12, voltageUnits::volt);
    right_motors.spin(directionType::fwd, right * 12, voltageUnits::volt);
  }else
  {
    left_motors.spin(directionType::fwd, left * 100.0, percentUnits::pct);
    right_motors.spin(directionType::fwd, right * 100.0, percentUnits::pct);
  }
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
 * Autonomously drive forward or backwards, X inches infront or behind the robot's current position.
 * This driving method is relative, so excessive use may cause the robot to get off course!
 *
 * @param inches Distance to drive in a straight line
 * @param speed How fast the robot should travel, 0 -> 1.0
 * @param correction How much the robot should correct for being off angle
 * @param dir Whether the robot is travelling forwards or backwards
 */
bool TankDrive::drive_forward(double inches, directionType dir, Feedback &feedback, double max_speed)
{
  static position_t pos_setpt;

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
    position_t cur_pos = odometry->get_position();

    // forwards is positive Y axis, backwards is negative
    if (dir == directionType::rev)
      inches = -fabs(inches);
    else
      inches = fabs(inches);

    // Use vector math to get an X and Y
    Vector2D cur_pos_vec({cur_pos.x , cur_pos.y});
    Vector2D delta_pos_vec(deg2rad(cur_pos.rot), inches);
    Vector2D setpt_vec = cur_pos_vec + delta_pos_vec;

    // Save the new X and Y values
    pos_setpt = {.x=setpt_vec.get_x(), .y=setpt_vec.get_y()};

  }

  // Call the drive_to_point with updated point values
  return drive_to_point(pos_setpt.x, pos_setpt.y, dir, feedback, max_speed);
}

bool TankDrive::drive_forward(double inches, directionType dir, double max_speed)
{
  if(drive_default_feedback != NULL)
    return drive_forward(inches, dir, *drive_default_feedback, max_speed);

  printf("tank_drive.cpp: Cannot run drive_forward without a feedback controller!\n");
  fflush(stdout);
  return true;
}

/**
 * Autonomously turn the robot X degrees to the right (negative for left), with a maximum motor speed
 * of percent_speed (-1.0 -> 1.0)
 * 
 * Uses a PID loop for it's control.
 */
bool TankDrive::turn_degrees(double degrees, Feedback &feedback, double max_speed)
{
  // We can't run the auto drive function without odometry
  if(odometry == NULL)
  {
    fprintf(stderr, "Odometry is NULL. Unable to run drive_forward()\n");
    fflush(stderr);
    return true;
  }

  static position_t saved_pos;

  // On the first run of the funciton, reset the gyro position and PID
  if (!func_initialized)
  {
    saved_pos = odometry->get_position();
    feedback.init(-degrees, 0);
    feedback.set_limits(-fabs(max_speed), fabs(max_speed));

    func_initialized = true;
  }
  double heading = odometry->get_position().rot - saved_pos.rot;
  double delta_heading = OdometryBase::smallest_angle(heading, degrees);
  feedback.update(-delta_heading);

  drive_tank(feedback.get(), -feedback.get());

  // If the robot is at it's target, return true
  if (feedback.is_on_target())
  {
    drive_tank(0, 0);
    func_initialized = false;
    return true;
  }

  return false;
}

bool TankDrive::turn_degrees(double degrees, double max_speed)
{
  if(turn_default_feedback != NULL)
    return turn_degrees(degrees, *turn_default_feedback, max_speed);
  
  printf("tank_drive.cpp: Cannot run turn_degrees without a feedback controller!\n");
  fflush(stdout);
  return true;
}


/**
  * Use odometry to automatically drive the robot to a point on the field.
  * X and Y is the final point we want the robot.
  *
  * Returns whether or not the robot has reached it's destination.
  */
bool TankDrive::drive_to_point(double x, double y, vex::directionType dir, Feedback &feedback, double max_speed)
{
  static double time = 0;
  // We can't run the auto drive function without odometry
  if(odometry == NULL)
  {
    fprintf(stderr, "Odometry is NULL. Unable to run drive_forward()\n");
    fflush(stderr);
    return true;
  }
  
  if(!func_initialized)
  {
    time = 0;
    double initial_dist = OdometryBase::pos_diff(odometry->get_position(), {.x=x, .y=y});

    // Reset the control loops
    correction_pid.init(0, 0);
    feedback.init(-initial_dist, 0);

    correction_pid.set_limits(-1, 1);
    feedback.set_limits(-1, 1);

    func_initialized = true;
  }
  printf("%.3f\t", time);

  // Store the initial position of the robot
  position_t current_pos = odometry->get_position();
  position_t end_pos = {.x=x, .y=y};

  // Create a point (and vector) to get the direction
  Vector2D::point_t pos_diff_pt = 
  {
    .x = x - current_pos.x,
    .y = y - current_pos.y
  };

  Vector2D point_vec(pos_diff_pt);

  // Get the distance between 2 points
  double dist_left = OdometryBase::pos_diff(current_pos, end_pos);
  //dist_left = end_pos.y - current_pos.y;  
  int sign = 1;

  // Make an imaginary perpendicualar line to that between the bot and the point. If the point is behind that line,
  // and the point is within the robot's radius, use negatives for feedback control.

  double angle_to_point = atan2(y - current_pos.y, x - current_pos.x) * 180.0 / PI;
  double angle = fmod(current_pos.rot - angle_to_point, 360.0);

  // Normalize the angle between 0 and 360
  if (angle > 360) angle -= 360;
  if (angle < 0) angle += 360; 

  // If the angle is behind the robot, report negative.
  if (dir == directionType::fwd && angle > 90 && angle < 270){
    sign = -1;

  }else if(dir == directionType::rev && (angle < 90 || angle > 270)){
    sign = -1;
  }

  if (fabs(dist_left) < config.drive_correction_cutoff) 
  {
    // When inside the robot's cutoff radius, report the distance to the point along the robot's forward axis,
    // so we always "reach" the point without having to do a lateral translation
    dist_left *= fabs(cos(angle * PI / 180.0));
  }

  // Get the heading difference between where we are and where we want to be
  // Optimize that heading so we don't turn clockwise all the time
  double heading = rad2deg(point_vec.get_dir());
  double delta_heading = 0;

  // Going backwards "flips" the robot's current heading
  if (dir == directionType::fwd)
    delta_heading = OdometryBase::smallest_angle(current_pos.rot, heading);
  else
    delta_heading = OdometryBase::smallest_angle(current_pos.rot - 180, heading);

  // Update the PID controllers with new information
  printf("Heading:\t");
  correction_pid.update(delta_heading);
  feedback.update(sign * -1 * dist_left);

  // Disable correction when we're close enough to the point
  double correction = 0;
  if(is_pure_pursuit || fabs(dist_left) > config.drive_correction_cutoff)
    correction = correction_pid.get();

  // Reverse the drive_pid output if we're going backwards
  double drive_pid_rval;
  if(dir == directionType::rev){
    drive_pid_rval = feedback.get() * -1;
  }else{
    drive_pid_rval = feedback.get();
  }

  // Combine the two pid outputs
  double lside = drive_pid_rval - correction;
  double rside = drive_pid_rval + correction;

  // limit the outputs between -1 and +1
  lside = clamp(lside, -1, 1);
  rside = clamp(rside, -1, 1);

  drive_tank(lside, rside);
  time += .02;



  //printf("x: %.3f\t", current_pos.x);
  //printf("y: %.3f\t", current_pos.y);
  //printf("rot: %.3f\t", current_pos.rot);
  //printf("targetX: %.3f\t", x);
  //printf("targetY: %.3f\t", y);
  //printf("Correctoin:%.3f\t", correction);
  


  //printf("%.3f\t", end_pos.y);
  //printf("%.3f\t", angle/10);


  // Check if the robot has reached it's destination
  if(feedback.is_on_target())
  {
    printf("\n======\nfinished dist_left: %f\n", dist_left);fflush(stdout);
    stop();
    func_initialized = false;
    return true;
  }

  return false;
}

bool TankDrive::drive_to_point(double x, double y, vex::directionType dir, double max_speed)
{
  if(drive_default_feedback != NULL)
    return this->drive_to_point(x, y, dir, *drive_default_feedback, max_speed);

  printf("tank_drive.cpp: Cannot run drive_to_point without a feedback controller!\n");
  fflush(stdout);
  return true;
}

/**
 * Turn the robot in place to an exact heading relative to the field.
 * 0 is forward, and 0->360 is clockwise.
 */
bool TankDrive::turn_to_heading(double heading_deg, Feedback &feedback, double max_speed)
{
  // We can't run the auto drive function without odometry
  if(odometry == NULL)
  {
    fprintf(stderr, "Odometry is NULL. Unable to run drive_forward()\n");
    fflush(stderr);
    return true;
  }
  static position_t last_pos = odometry->get_position(); 
  if(!func_initialized)
  {
    double initial_delta = OdometryBase::smallest_angle(odometry->get_position().rot, heading_deg);
    feedback.init(-initial_delta, 0);
    feedback.set_limits(-fabs(max_speed), fabs(max_speed));

    func_initialized = true;
  }

  // Get the difference between the new heading and the current, and decide whether to turn left or right.
  double delta_heading = OdometryBase::smallest_angle(odometry->get_position().rot, heading_deg);
  feedback.update(-delta_heading);
  


  last_pos = odometry->get_position();


  fflush(stdout);

  printf("out: %f\t", feedback.get());

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

bool TankDrive::turn_to_heading(double heading_deg, double max_speed)
{
  if(turn_default_feedback != NULL)
    return turn_to_heading(heading_deg, *turn_default_feedback, max_speed);

  printf("tank_drive.cpp: Cannot run turn_to_heading without a feedback controller!\n");
  fflush(stdout);
  return true;
}

/**
 * Modify the inputs from the controller by squaring / cubing, etc
 * Allows for better control of the robot at slower speeds
 */
double TankDrive::modify_inputs(double input, int power)
{
  return (power % 2 == 0 ? (input < 0 ? -1 : 1) : 1) * pow(input, power);
}

bool TankDrive::pure_pursuit(std::vector<PurePursuit::hermite_point> path, directionType dir, double radius, double res, Feedback &feedback, double max_speed) 
{
  is_pure_pursuit = true;
  std::vector<Vector2D::point_t> smoothed_path = PurePursuit::smooth_path_hermite(path, res);

  Vector2D::point_t lookahead = PurePursuit::get_lookahead(smoothed_path, {odometry->get_position().x, odometry->get_position().y}, radius);
  bool is_last_point = (path.back().x == lookahead.x) && (path.back().y == lookahead.y);

  if(is_last_point)
    is_pure_pursuit = false;

  bool retval = drive_to_point(lookahead.x, lookahead.y, dir, feedback, max_speed);

  if(is_last_point)
    return retval;

  return false;
}
