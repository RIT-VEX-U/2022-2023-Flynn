#include "../include/roller_controller.h"


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


