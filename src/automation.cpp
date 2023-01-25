#include "../include/automation.h"


/**
* Construct a SpinRollerCommand
* @param roller_motor The motor that will spin the roller
*/
SpinRollerCommand::SpinRollerCommand(vex::motor roller_motor): roller_motor(roller_motor){};

/**
 * Run roller controller to spin the roller to our color
 * Overrides run from AutoCommand
 * @returns true when execution is complete, false otherwise
 */
bool SpinRollerCommand::run() {
    const double roller_cutoff_threshold = .01; //revolutions //TODO measure once against roller
    const double num_revolutions_to_spin_motor = 1; //revolutions //TODO measure once against roller
    const double kP = .01; // Proportional constant for spinning the roller half a revolution// TODO measure based on field
 
    // Initialize start and end position if not already
    if (!func_initialized){
        start_pos = roller_motor.position(vex::rev);
        target_pos = start_pos + num_revolutions_to_spin_motor;
    }    

    // Calculate error
    double current_pos = roller_motor.position(vex::rev);
    double error = current_pos - start_pos;

    // If we're close enough, call it here.
    if (fabs(error)>roller_cutoff_threshold < roller_cutoff_threshold){
        func_initialized = false;
        return true;
    }

    // otherwise, do a P controller
    roller_motor.spin(vex::fwd, error * kP, vex::volt);
    return false;
}

/**
* Construct a ShootCommand
* @param firing_motor The motor that will spin the disk into the flywheel
*/
ShootCommand::ShootCommand(vex::motor firing_motor, double seconds_to_shoot): firing_motor(firing_motor), seconds_to_shoot(seconds_to_shoot){}

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
    return true;
  }

  firing_motor.spin(vex::reverse); //TODO figure out if this needs to be negated to slap it into the flywheel
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
  intaking_motor.spin(vex::fwd, intaking_voltage, vex::volt); 
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