#include "../include/automation.h"
#include "robot-config.h"


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
SpinRollerCommandAUTO::SpinRollerCommandAUTO(TankDrive &drive_sys, vex::motor roller_motor): roller_motor(roller_motor), drive_sys(drive_sys){};

/**
 * Run roller controller to spin the roller to our color
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise
 */
bool SpinRollerCommandAUTO::run() {
    const double roller_cutoff_threshold = .05  ; //revolutions // [measure]
    const double num_revolutions_to_spin_motor = -2.5; //revolutions // [measure]
    const double drive_power = .2; // [measure]

    // Initialize start and end position if not already
    if (!func_initialized){
        start_pos = roller_motor.position(vex::rev);
        target_pos = start_pos + num_revolutions_to_spin_motor;
    }

    // Calculate error
    double current_pos = roller_motor.position(vex::rev);
    double error = target_pos - current_pos;
    printf("error: %f\n", error);
    // If we're close enough, call it here.
    if (fabs(error) < roller_cutoff_threshold){
        func_initialized = false;
        roller_motor.stop();
        drive_sys.stop() ;       
        return true;
    }

    // otherwise, do a P controller
    vex::directionType spin_dir = fwd;
    if (sign(error) < 0){
      spin_dir = reverse;
    }
    roller_motor.spin(spin_dir, 12, volt);
    drive_sys.drive_tank(drive_power, drive_power);
    func_initialized = true;
    return false;
}


/**
* Construct a SpinRollerCommand
* @param drive_sys the drive train that will allow us to apply pressure on the rollers
* @param roller_motor The motor that will spin the roller
*/
SpinRollerCommandSKILLS::SpinRollerCommandSKILLS(TankDrive &drive_sys, vex::motor roller_motor): roller_motor(roller_motor), drive_sys(drive_sys){};

/**
 * Run roller controller to spin the roller to our color
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise
 */
bool SpinRollerCommandSKILLS::run() {
    const double roller_cutoff_threshold = .01; //revolutions // [measure]
    const double num_revolutions_to_spin_motor = -4.5; //revolutions // [measure]
    const double drive_power = .2; // [measure]

    // Initialize start and end position if not already
    if (!func_initialized){
        start_pos = roller_motor.position(vex::rev);
        target_pos = start_pos + num_revolutions_to_spin_motor;
    }

    // Calculate error
    double current_pos = roller_motor.position(vex::rev);
    double error = target_pos - current_pos;
    printf("error: %f\n", error);
    // If we're close enough, call it here.
    if (fabs(error) < roller_cutoff_threshold){
        func_initialized = false;
        roller_motor.stop();
        drive_sys.stop() ;       
        return true;
    }

    // otherwise, do a P controller
    vex::directionType spin_dir = fwd;
    if (sign(error) < 0){
      spin_dir = reverse;
    }
    roller_motor.spin(spin_dir, 12, volt);
    drive_sys.drive_tank(drive_power, drive_power);
    func_initialized = true;
    return false;
}
/**
* Construct a ShootCommand
* @param firing_motor The motor that will spin the disk into the flywheel
*/
ShootCommand::ShootCommand(vex::motor firing_motor, double seconds_to_shoot, double volt): firing_motor(firing_motor), seconds_to_shoot(seconds_to_shoot), volt(volt){}

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
StartIntakeCommand::StartIntakeCommand(vex::motor intaking_motor, double intaking_voltage):intaking_motor(intaking_motor), intaking_voltage(intaking_voltage){}

/**
 * Run the StartIntakeCommand
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise
 
*/
bool StartIntakeCommand::run(){
  intaking_motor.spin(vex::reverse, intaking_voltage, vex::volt); 
  return true;
}

/**
* Construct a StartIntakeCommand
* @param intaking_motor The motor that will be stopped
*/
StopIntakeCommand::StopIntakeCommand(vex::motor intaking_motor):intaking_motor(intaking_motor){}

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