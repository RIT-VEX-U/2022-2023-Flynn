#include <cstdio>
#include "automation.h"
#include "robot-config.h"
#include "vision.h"

// Pushes the firing flap to the up position for close in shots
void intake_up()
{
  intake_solenoid.set(true);
}

// Pushes the firing flap to the down position for far away shots
void intake_down()
{
  intake_solenoid.set(false);
}


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


#define BLUE_HUE 27
#define RED_HUE 44
#define NEUTRAL_BAND 4

Pepsi get_roller_scored()
{
  static const double HUE_THRESH = (BLUE_HUE + RED_HUE) / 2.0;
  static const bool IS_RED_GREATER = RED_HUE > BLUE_HUE;

  double hue = roller_sensor.hue();

  if(hue > HUE_THRESH - NEUTRAL_BAND && hue < HUE_THRESH + NEUTRAL_BAND)
    return NEUTRAL;

  if((hue > HUE_THRESH && IS_RED_GREATER) || (hue < HUE_THRESH && !IS_RED_GREATER))
    return BLUE;
  
  return RED;
}

/**
 * Construct a SpinRollerCommand
 * @param align_pos The motor that will spin the roller
 */
SpinRollerCommand::SpinRollerCommand(pose_t align_pos) {}

/**
 * Run roller controller to spin the roller to our color
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise
 */

bool SpinRollerCommand::run()
{
  CommandController cmd;
  cmd.add(new DriveForwardCommand(drive_sys, drive_fast_mprofile, 12, directionType::fwd), 0.5);
  cmd.add(new FunctionCommand([](){drive_sys.drive_tank(0.2,0.2); return true;}));
  cmd.add_delay(800);
  cmd.run();

  Pepsi cur_roller = get_roller_scored();

  CommandController cmd1;
  cmd1.add(new DriveForwardCommand(drive_sys, drive_fast_mprofile, 6, directionType::rev), 1);
  cmd1.add(new DriveStopCommand(drive_sys)),
  cmd1.run();

  printf("RED? = %d, CUR = %s\n", target_red, cur_roller==RED?"red":cur_roller==BLUE?"blue":"neutral");
  if((cur_roller == RED && target_red) || (cur_roller == BLUE && !target_red))
  {
    drive_sys.stop();
    return true;
  }

  return false;
}

void SpinRollerCommand::on_timeout()
{
  drive_sys.stop();
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
  pose_t pos = odom.get_position();
  printf("%.2f, %.2f, %.2f\n", pos.x, pos.y, pos.rot);
  return true;
}

PrintOdomContinousCommand::PrintOdomContinousCommand(OdometryTank &odom) : odom(odom) {}
bool PrintOdomContinousCommand::run()
{
  pose_t pos = odom.get_position();
  printf("CONTINUAL(%.2f, %.2f), %.2f\n", pos.x, pos.y, pos.rot);
  if (main_controller.ButtonA.pressing()){
    return true;
  }
  return false;
}




PID::pid_config_t vis_pid_cfg = {
    .p = .004,
    .d = .0004,
    .deadband = 10,
    .on_target_time = .2};

FeedForward::ff_config_t vis_ff_cfg = {
    .kS = 0.03};

#define MIN_AREA 500
#define MIN_VISION_AREA 150
#define MAX_VISION_AREA 4000
#define MAX_SPEED 0.5

VisionAimCommand::VisionAimCommand(bool odometry_fallback, int vision_center, int fallback_degrees)
    : pidff(vis_pid_cfg, vis_ff_cfg), odometry_fallback(odometry_fallback), first_run(true), fallback_triggered(false), vision_center(vision_center), fallback_degrees(fallback_degrees)
{}

int VisionAimCommand::get_x(){

  int x_val = 0;

  if(!cam.installed())
  {
    return 0;
  }

  if(target_red){
    cam.takeSnapshot(RED_GOAL);
  } else {
    cam.takeSnapshot(BLUE_GOAL);
  }

  for(int i = 0; i < cam.objectCount; i++){
    double blob_1_area = cam.objects[i].width * cam.objects[i].height;
    if(blob_1_area < MIN_VISION_AREA || blob_1_area > MAX_VISION_AREA){
      continue;
    }
      for(int j = i+1; j < cam.objectCount; j++){
        double blob_2_area = cam.objects[j].width * cam.objects[j].height;
        if(fabs(cam.objects[i].centerX - cam.objects[j].centerX) < 5 && blob_2_area >= MIN_VISION_AREA && blob_2_area <= MAX_VISION_AREA){
          x_val = cam.objects[i].centerX;
        }
      }
    }

    return x_val;
}

/**
 * Run the VisionAimCommand
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise
 */
bool VisionAimCommand::run()
{
  // If the camera isn't installed, move on to the next command
  if (!cam.installed() || !vision_enabled)
  {
    drive_sys.stop();
    return true;
  }
  
  if (first_run)
  {
    stored_pos = odometry_sys.get_position();
    drive_sys.reset_auto();
    first_run = false;
  }

  if (odometry_fallback &&
      (fallback_triggered || fabs(OdometryBase::smallest_angle(stored_pos.rot, odometry_sys.get_position().rot)) > fallback_degrees))
  {
    fallback_triggered = true;
    if (drive_sys.turn_to_heading(stored_pos.rot, 0.6))
      return true;
    else
      return false;
  }

  // // If the camera isn't installed, move on to the next command
  // if (!cam.installed())
  //   return true;
  // // If we have disabled vision on the screen, move on to the next command
  // if (!vision_enabled)
  //   return true;

  // if(target_red){
  //   cam.takeSnapshot(RED_GOAL);
  // } else {
  //   cam.takeSnapshot(BLUE_GOAL);
  // }

  int x_val = get_x();

  printf("Object Count %ld\n", cam.objectCount);

  printf("CenterX: %d\n", x_val);

  if (x_val != 0)
  {

    // Update the PID loop & drive the robot
    pidff.set_target(vision_center);
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

void VisionAimCommand::on_timeout()
{
  drive_sys.stop();
}

/**
 * Constuct a TurnToPointCommand
 * @param odom Refrence to the OdometryTank object
 * @param drive_sys Reference to the TankDrive system
 * @param point The point we want to turn towards
 */
TurnToPointCommand::TurnToPointCommand(TankDrive &drive_sys, OdometryTank &odom, Feedback &turn_feedback, point_t point) : drive_sys(drive_sys), odom(odom), feedback(turn_feedback), point(point) {}

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

  pose_t old_pos = odom.get_position();
  pose_t newpos = {.x = x, .y = y, .rot = heading};
  if (x == NO_CHANGE)
  {
    newpos.x = old_pos.x;
  }
  if (y == NO_CHANGE)
  {
    newpos.y = old_pos.y;
  }
  odom.set_position(newpos);
  return true;
}

#define ROLLER_AREA_CUTOFF 1500

Pepsi scan_roller()
{
  if (!cam.installed())
  {
    printf("Cam Disconnected!\n");
    return NEUTRAL;
  }

  // SCAN FOR RED
  cam.takeSnapshot(RED_GOAL);
  vex::vision::object red_obj = cam.largestObject;
  int red_area = red_obj.width * red_obj.height;
  int red_y = red_obj.centerY;

  if (cam.objectCount < 1 || red_area < ROLLER_AREA_CUTOFF)
    return NEUTRAL;

  // SCAN FOR BLUE
  cam.takeSnapshot(BLUE_GOAL);
  vex::vision::object blue_obj = cam.largestObject;
  int blue_area = blue_obj.width * blue_obj.height;
  int blue_y = blue_obj.centerY;

  if (cam.objectCount < 1 || blue_area < ROLLER_AREA_CUTOFF)
    return NEUTRAL;

  if (red_y > blue_y)
    return RED;
  else
    return BLUE;
}
