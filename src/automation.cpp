#include <cstdio>
#include "automation.h"
#include "robot-config.h"
#include "vision.h"

// Pushes the firing flap to the up position for close in shots
void flap_up()
{
  flapup_solenoid.set(false);
}

// Pushes the firing flap to the down position for far away shots
void flap_down()
{
  flapup_solenoid.set(true);
}
/**
 * Construct a FlapUpCommand
 * when run it flaps the flap up
 */
FlapUpCommand::FlapUpCommand() {}
bool FlapUpCommand::run()
{
  flap_up();
  return true;
}
/**
 * Construct a FlapDownCommand
 * when run it flaps the flap down
 */
FlapDownCommand::FlapDownCommand() {}
bool FlapDownCommand::run()
{
  flap_down();
  return true;
}

/**
 * Construct a SpinRollerCommand
 * @param drive_sys the drive train that will allow us to apply pressure on the rollers
 * @param roller_motor The motor that will spin the roller
 */
SpinRollerCommandAUTO::SpinRollerCommandAUTO(TankDrive &drive_sys, vex::motor &roller_motor) : drive_sys(drive_sys), roller_motor(roller_motor) {}

/**
 * Run roller controller to spin the roller to our color
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise
 */
bool SpinRollerCommandAUTO::run()
{
  const double roller_cutoff_threshold = .05;      // revolutions // [measure]
  const double num_revolutions_to_spin_motor = -2; // revolutions // [measure]
  const double drive_power = .2;                   // [measure]

  // Initialize start and end position if not already
  if (!func_initialized)
  {
    start_pos = roller_motor.position(vex::rev);
    target_pos = start_pos + num_revolutions_to_spin_motor;
    func_initialized = true;
  }

  // Calculate error
  double current_pos = roller_motor.position(vex::rev);
  double error = target_pos - current_pos;

  // If we're close enough, call it here.
  if (fabs(error) > roller_cutoff_threshold < roller_cutoff_threshold)
  {
    func_initialized = false;
    roller_motor.stop();
    return true;
  }

  vex::directionType dir = fwd;
  if (error < 0)
  {
    dir = reverse;
  }
  // otherwise, do a P controller
  roller_motor.spin(dir, 8, vex::volt);
  drive_sys.drive_tank(drive_power, drive_power);
  return false;
}

/**
 * Construct a ShootCommand
 * @param firing_motor The motor that will spin the disk into the flywheel
 */
ShootCommand::ShootCommand(vex::motor &firing_motor, double seconds_to_shoot, double volt) : firing_motor(firing_motor), seconds_to_shoot(seconds_to_shoot), volt(volt) {}

/**
 * Run the intake motor backward to move the disk into the flywheel
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise
 */
bool ShootCommand::run()
{
  if (!func_initialized)
  {
    tmr.reset();
    func_initialized = true;
  }

  if (tmr.time(vex::seconds) > seconds_to_shoot)
  {
    func_initialized = false;
    firing_motor.stop();
    return true;
  }
  printf("Shooting at %f RPM\n", flywheel_sys.getRPM());
  firing_motor.spin(vex::fwd, volt, vex::volt); // TODO figure out if this needs to be negated to slap it into the flywheel
  return false;
}

/**
 * Construct a StartIntakeCommand
 * @param intaking_motor The motor that will pull the disk into the robot
 * @param intaking_voltage The voltage at which to run the intake motor
 */
StartIntakeCommand::StartIntakeCommand(vex::motor &intaking_motor, double intaking_voltage) : intaking_motor(intaking_motor), intaking_voltage(intaking_voltage) {}

/**
 * Run the StartIntakeCommand
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise

*/
bool StartIntakeCommand::run()
{
  intaking_motor.spin(vex::reverse, intaking_voltage, vex::volt);
  return true;
}

SpinRawCommand::SpinRawCommand(vex::motor &flywheel_motor, double voltage) : flywheel_motor(flywheel_motor), voltage(voltage) {}

/**
 * Run the StartIntakeCommand
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise

*/
bool SpinRawCommand::run()
{
  flywheel_motor.spin(vex::fwd, voltage, vex::volt);
  return true;
}

/**
 * Construct a StartIntakeCommand
 * @param intaking_motor The motor that will be stopped
 */
StopIntakeCommand::StopIntakeCommand(vex::motor &intaking_motor) : intaking_motor(intaking_motor) {}

/**
 * Run the StopIntakeCommand
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise
 */
bool StopIntakeCommand::run()
{
  intaking_motor.stop();
  return true;
}

EndgameCommand::EndgameCommand(vex::digital_out &solenoid) : solenoid(solenoid) {}
bool EndgameCommand::run()
{
  solenoid.set(true);
  return true;
}
PrintOdomCommand::PrintOdomCommand(OdometryTank &odom) : odom(odom) {}

bool PrintOdomCommand::run()
{
  position_t pos = odom.get_position();
  printf("%.2f, %.2f, %.2f\n", pos.x, pos.y, pos.rot);
  return true;
}

PrintOdomContinousCommand::PrintOdomContinousCommand(OdometryTank &odom) : odom(odom) {}
bool PrintOdomContinousCommand::run()
{
  position_t pos = odom.get_position();
  printf("(%.2f, %.2f), %.2f\n", pos.x, pos.y, pos.rot);
  return false;
}
/**
 * Construct a StartIntakeCommand
 * @param colorSensor The color sensor being used
 * @param color The hue value of the color being detected
 * @param rollerMotor The rollor motor to spin the roller
 * @param error Error for color detection color+-error
 */
SpinToColorCommand::SpinToColorCommand(vex::optical &colorSensor, double color, vex::motor &rollerMotor, double error) : colorSensor(colorSensor), color(color), rollerMotor(rollerMotor), error(error) {}

/**
 * Run the StopIntakeCommand
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise
 */
bool SpinToColorCommand::run()
{
  // To deal with wrap around
  if (color <= error)
  {
    if (colorSensor.hue() > color + 15 && colorSensor.hue() < wrap_angle_deg(color - error))
    {
      return false;
    }
  }
  // no wrap around
  else if (colorSensor.hue() < wrap_angle_deg(color - error) || colorSensor.hue() > wrap_angle_deg(color + error))
  {
    rollerMotor.spin(vex::directionType::fwd);
    return false;
  }

  rollerMotor.stop();

  return true;
}

PID::pid_config_t vis_pid_cfg = {
    .p = .003,
    // .d = .0001,
    .deadband = 5,
    .on_target_time = .2};

FeedForward::ff_config_t vis_ff_cfg = {
    .kS = 0.07};

#define VISION_CENTER 145
#define MIN_AREA 500
#define MAX_SPEED 0.5
#define FALLBACK_MAX_DEGREES 10

VisionAimCommand::VisionAimCommand(bool odometry_fallback)
    : pidff(vis_pid_cfg, vis_ff_cfg), odometry_fallback(odometry_fallback), first_run(true), fallback_triggered(false)
{
}

/**
 * Run the VisionAimCommand
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise
 */
bool VisionAimCommand::run()
{

 if (first_run)
  {
    stored_pos = odometry_sys.get_position();
    drive_sys.reset_auto();
    first_run = false;
  }

  if (odometry_fallback &&
      (fallback_triggered || fabs(OdometryBase::smallest_angle(stored_pos.rot, odometry_sys.get_position().rot)) > FALLBACK_MAX_DEGREES))
  {
    fallback_triggered = true;
    if (drive_sys.turn_to_heading(stored_pos.rot, 0.6))
      return true;
    else
      return false;
  }

  // If the camera isn't installed, move on to the next command
  if (!cam.installed())
    return true;
  // If we have disabled vision on the screen, move on to the next command
  if (!vision_enabled)
    return true;

  // Take a snapshot with each color selected,
  // and store the largest found object for each in a vector
  vision::object red_obj, blue_obj;

  // Get largest red blob
  cam.takeSnapshot(RED_GOAL);
  int red_count = cam.objectCount;
  if (red_count > 0)
    red_obj = cam.largestObject;

  // Get largest blue blob
  cam.takeSnapshot(BLUE_GOAL);
  int blue_count = cam.objectCount;
  if (blue_count > 0)
    blue_obj = cam.largestObject;

  // Compare the areas of the largest
  double red_area = red_obj.width * red_obj.height;
  double blue_area = blue_obj.width * blue_obj.height;
  int x_val = 0;

  if (red_area > blue_area && red_area > MIN_AREA && target_red)
  {
    x_val = red_obj.centerX;
  }
  else if (blue_area > red_area && blue_area > MIN_AREA && !target_red)
  {
    x_val = blue_obj.centerX;
  }

  printf("CenterX: %d\n", x_val);

  if (x_val != 0)
  {

    // Update the PID loop & drive the robot
    pidff.set_target(VISION_CENTER);
    pidff.set_limits(-MAX_SPEED, MAX_SPEED);
    double out = pidff.update(x_val);

    // Currently set up for upside-down camera. Flip signs if going backwards.
    drive_sys.drive_tank(out, -out);

    if (pidff.is_on_target())
    {
      drive_sys.stop();
      return true;
    }
  }
  else
  {
    drive_sys.stop();
    printf("Nothing Found\n");
  }

  return false;
}

/**
 * Constuct a TurnToPointCommand
 * @param odom Refrence to the OdometryTank object
 * @param drive_sys Reference to the TankDrive system
 * @param point The point we want to turn towards
 */
TurnToPointCommand::TurnToPointCommand(TankDrive &drive_sys, OdometryTank &odom, Feedback &turn_feedback, Vector2D::point_t point) : drive_sys(drive_sys), odom(odom), feedback(turn_feedback), point(point) {}

/**
 * Run the TurnToPointCommand
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise
 */
bool TurnToPointCommand::run()
{
  // get differences in x and y to calculate angle relative to x axis
  double delta_x = point.x - odom.get_position().x;
  double delta_y = point.y - odom.get_position().y;

  // get the angle
  double heading_deg = rad2deg(atan2(delta_y, delta_x));

  return drive_sys.turn_to_heading(heading_deg, feedback);
}

/**
 * Constuct a FunctionCommand
 * @param func the function to run
 */
FunctionCommand::FunctionCommand(std::function<bool(void)> func) : func(func) {}

/**
 * Run the TurnToPointCommand
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise
 */
bool FunctionCommand::run()
{
  return func();
}
/**
 * Align with a wall at a certain x, y and heading.
 * If a value is not known at compile time, set it to NO_CHANGE and it will be filled in when the command is run
 * @param drive_sys how to drive into the wall
 * @param odom how to know where we hit a wall
 * @param x where we hit the wall in the x dimension (NO_CHANGE if this is the unknown quantity)
 * @param y where we hit the wall in the y dimension (NO_CHANGE if this is the unknown quantity)
 * @param heading the angle we want to hit the wall at (should never be no change)
 * @param drive_power how fast we want it hit the wall
 */
WallAlignCommand::WallAlignCommand(TankDrive &drive_sys, OdometryTank &odom, double x, double y, double heading, double drive_power, double time) : drive_sys(drive_sys), odom(odom), x(x), y(y), heading(heading), time(time), func_initialized(false) {}

/**
 * reset the position to that which is specified
 */
bool WallAlignCommand::run()
{
  // Start cutoff timer
  if (!func_initialized)
  {
    tmr.reset();
    func_initialized = true;
  }

  // If we're not cutoff, drive
  if (tmr.time(seconds) < time)
  {
    drive_sys.drive_tank(drive_power, drive_power);
    return false;
  }
  // otherwise stop and reset the position
  drive_sys.stop();

  position_t old_pos = odom.get_position();
  position_t newpos = {.x = x, .y = y, .rot = heading};
  if (x == NO_CHANGE)
  {
    newpos.x = old_pos.x;
  }
  if (x == NO_CHANGE)
  {
    newpos.y = old_pos.y;
  }
  odom.set_position(newpos);
  return true;
}