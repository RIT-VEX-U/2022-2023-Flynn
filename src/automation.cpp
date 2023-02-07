#include "../include/automation.h"
#include "../include/robot-config.h"
#include "../core/include/utils/math_util.h"
#include "../core/include/subsystems/tank_drive.h"
#include "../core/include/subsystems/odometry/odometry_base.h"

// Pushes the firing flap to the up position for close in shots
void FlapUp(){
  flapup_solenoid.set(true);
  flapdown_solenoid.set(false);
}

// Pushes the firing flap to the down position for far away shots
void FlapDown(){
  flapup_solenoid.set(false);
  flapdown_solenoid.set(true);
}
/**
 * Construct a FlapUpCommand
 * when run it flaps the flap up
*/
FlapUpCommand::FlapUpCommand(){}
bool FlapUpCommand::run(){
  FlapUp();
  return true;
}
/**
 * Construct a FlapDownCommand
 * when run it flaps the flap down
*/
FlapDownCommand::FlapDownCommand(){}
bool FlapDownCommand::run(){
  FlapDown();
  return true;
}

/**
* Construct a SpinRollerCommand
* @param drive_sys the drive train that will allow us to apply pressure on the rollers
* @param roller_motor The motor that will spin the roller
*/
SpinRollerCommandAUTO::SpinRollerCommandAUTO(TankDrive &drive_sys, vex::motor &roller_motor): drive_sys(drive_sys), roller_motor(roller_motor){};

/**
 * Run roller controller to spin the roller to our color
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise
 */
bool SpinRollerCommandAUTO::run() {
    const double roller_cutoff_threshold = .05; //revolutions // [measure]
    const double num_revolutions_to_spin_motor = -2; //revolutions // [measure]
    const double drive_power = .2; // [measure]

    // Initialize start and end position if not already
    if (!func_initialized){
        start_pos = roller_motor.position(vex::rev);
        target_pos = start_pos + num_revolutions_to_spin_motor;
        func_initialized = true;
    }    

    // Calculate error
    double current_pos = roller_motor.position(vex::rev);
    double error = target_pos-current_pos;

    // If we're close enough, call it here.
    if (fabs(error)>roller_cutoff_threshold < roller_cutoff_threshold){
        func_initialized = false;
        roller_motor.stop();
        return true;
    }

    vex::directionType dir = fwd;
    if (error<0){
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
ShootCommand::ShootCommand(vex::motor &firing_motor, double seconds_to_shoot, double volt): firing_motor(firing_motor), seconds_to_shoot(seconds_to_shoot), volt(volt){}

/**
 * Run the intake motor backward to move the disk into the flywheel
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise
 */
bool ShootCommand::run(){
  if (!func_initialized){
    tmr.reset();
    func_initialized = true;
  }

  if (tmr.time(vex::seconds) > seconds_to_shoot){
    func_initialized = false;
    firing_motor.stop();
    return true;
  }
  printf("Shooting at %f RPM\n", flywheel_sys.getRPM());
  firing_motor.spin(vex::fwd, volt, vex::volt); //TODO figure out if this needs to be negated to slap it into the flywheel
  return false;
}


/**
* Construct a StartIntakeCommand
* @param intaking_motor The motor that will pull the disk into the robot
* @param intaking_voltage The voltage at which to run the intake motor
*/
StartIntakeCommand::StartIntakeCommand(vex::motor &intaking_motor, double intaking_voltage):intaking_motor(intaking_motor), intaking_voltage(intaking_voltage){}

/**
 * Run the StartIntakeCommand
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise
 
*/
bool StartIntakeCommand::run(){
  intaking_motor.spin(vex::reverse, intaking_voltage, vex::volt); 
  return true;
}

SpinRawCommand::SpinRawCommand(vex::motor &flywheel_motor, double voltage):flywheel_motor(flywheel_motor), voltage(voltage){}

/**
 * Run the StartIntakeCommand
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise
 
*/
bool SpinRawCommand::run(){
  flywheel_motor.spin(vex::fwd, voltage, vex::volt); 
  return true;
}

/**
* Construct a StartIntakeCommand
* @param intaking_motor The motor that will be stopped
*/
StopIntakeCommand::StopIntakeCommand(vex::motor &intaking_motor):intaking_motor(intaking_motor){}

/**
 * Run the StopIntakeCommand
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise
 */
bool StopIntakeCommand::run(){
  intaking_motor.stop(); 
  return true;
}

EndgameCommand::EndgameCommand(vex::digital_out &solenoid): solenoid(solenoid){}
bool EndgameCommand::run(){
  solenoid.set(true);
  return true;
}
PrintOdomCommand::PrintOdomCommand(OdometryTank &odom): odom(odom){}

bool PrintOdomCommand::run(){
  position_t pos = odom.get_position();
  printf("%.2f, %.2f, %.2f\n", pos.x, pos.y, pos.rot);
  return true;
}

PrintOdomContinousCommand::PrintOdomContinousCommand(OdometryTank &odom): odom(odom){}
bool PrintOdomContinousCommand::run(){
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
SpinToColorCommand::SpinToColorCommand(vex::optical &colorSensor, double color, vex::motor &rollerMotor, double error):colorSensor(colorSensor), color(color), rollerMotor(rollerMotor), error(error){}
  
  /**
 * Run the StopIntakeCommand
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise
 */
  bool SpinToColorCommand::run(){
    //To deal with wrap around
    if(color<=error){
      if(colorSensor.hue()>color+15 && colorSensor.hue() < wrap_angle_deg(color-error)){
        return false;
      }
    }
    //no wrap around
    else if(colorSensor.hue() < wrap_angle_deg(color-error) || colorSensor.hue() > wrap_angle_deg(color+error)){
      rollerMotor.spin(vex::directionType::fwd);
      return false;
    }

    rollerMotor.stop();

    return true;
  }



PID::pid_config_t vis_pid_cfg = {
  .p = 0,
  .d = 0,
  .deadband = 0,
  .on_target_time = 0
};

#define VISION_CENTER 0
#define NOT_DETECTED_TIME 2
#define MAX_SPEED 0.5

VisionAimCommand::VisionAimCommand(vision &cam, initializer_list<vision::signature> sigs, TankDrive &drive_sys): cam(cam), sig_vec(sigs), drive_sys(drive_sys), pid(vis_pid_cfg) 
{}
VisionAimCommand::VisionAimCommand(vision &cam, vision::signature sig, TankDrive &drive_sys): cam(cam), sig_vec({sig}), drive_sys(drive_sys), pid(vis_pid_cfg)
{}

/**
 * Run the VisionAimCommand
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise
*/
bool VisionAimCommand::run()
{
  // If the camera isn't installed, move on to the next command
  if(!cam.installed())
    return true;
  
  // cam.takeSnapshot(sig, 1);
  if(cam.objectCount > 0)
  {
    // Take a snapshot with each color selected, 
    // and store the largest found object for each in a vector
    vector<vision::object> found;
    for(vision::signature s : sig_vec)
    {
      cam.takeSnapshot(s, 1);
      for(int i=0; i<cam.objects.getLength(); i++)
        found.push_back(cam.objects[i]);
    }

    // Make sure we have something
    if(found.size() < 1)
      return false;

    // Find the largest object in the "found" list
    vision::object &largest = found[0];
    for(int i=1; i<found.size(); i++)
    {
      if((found[i].width * found[i].height) > (largest.width * largest.height))
        largest = found[i];
    }

    // Update the PID loop & drive the robot
    pid.set_target(VISION_CENTER);
    pid.set_limits(-MAX_SPEED, MAX_SPEED);
    double out = pid.update(largest.centerX);

    drive_sys.drive_tank(out, -out);

    if(pid.is_on_target())
      return true;

  } else
  {
    drive_sys.stop();
  }

  return false;
}

/**
 * Constuct a TurnToPointCommand
 * @param odom Refrence to the OdometryTank object
 * @param drive_sys Reference to the TankDrive system
 * @param point The point we want to turn towards
*/
TurnToPointCommand::TurnToPointCommand(TankDrive &drive_sys, OdometryTank &odom, Feedback &turn_feedback, Vector2D::point_t point): drive_sys(drive_sys), odom(odom), feedback(turn_feedback), point(point){}

/**
 * Run the TurnToPointCommand
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise
*/
bool TurnToPointCommand::run(){
  //get differences in x and y to calculate angle relative to x axis
  double delta_x=point.x-odom.get_position().x;
  double delta_y=point.y-odom.get_position().y;

  //get the angle
  double heading_deg = rad2deg(atan2(delta_y,delta_x));

  return drive_sys.turn_to_heading(heading_deg, feedback);
}