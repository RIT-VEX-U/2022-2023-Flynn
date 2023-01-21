#include "competition/autonomous.h"



//functions to choose which auto we want. construct a GenericAuto when chosen.
GenericAuto auto_loader_side(){}
GenericAuto auto_non_loader_side(){};

GenericAuto prog_skills_loader_side(){};
GenericAuto priog_skills_non_loader_size(){};


static bool func_initialized;
static double start_pos;
static double target_pos;


bool auto_spin_spinner(){
    const double roller_cutoff_threshold = .01; //revolutions
    const double num_revolutions_to_spin_motor = 1; //revolutions
    const double kP = .01; // Proportional constant for spinning the roller half a revolution// TODO measure based on field

    // Initialize start and end position if not already
    if (!func_initialized){
        double start_pos = roller.position(rev);
        double target_pos = start_pos + num_revolutions_to_spin_motor;
    }    
    
    // Calculate error
    double current_pos = roller.position(rev);
    double error = current_pos - start_pos;

    // If we're close enough, call it here.
    if (fabs(error)>roller_cutoff_threshold < roller_cutoff_threshold){
        func_initialized = false;
        return true;
    }

    // otherwise, do a P controller
    roller.spin(fwd, error * kP, volt);
    return false;
}

/**
 * Contains all the code run during autonomous.
 */ 
void autonomous()
{
    GenericAuto current_auto = auto_loader_side();    
    //current_auto.run(true);

}


/*
Auto loader side

Map from page 40 of the game manual

(R) = Red Hoop, Blue Zone
(B) = Blue Hoop, Red Zone
 *  = Starting position for this auto
+-------------------+
|        |____| (R) |
|___           |    |
| * |          |    |
|   |          |    |
|   |          |_*__|
|___|  ____         |
|(B)  |____|        |
+-------------------+

 Human Instructions:
 Align robot to specified place and angle using LOADER SIDE AUTO jig
*/
GenericAuto auto_loader_side(){
    const double loader_side_full_court_shot_rpm = 3000;  // TODO measure this RPM based on testing
    GenericAuto loader_side_auto;
    //TODO - when shooter is here, use it
    //loader_side_auto.add([](){return shooter.spinToRPM(loader_side_full_court_shot_rpm)})
    //loader_side_auto.add([](){return shooter.shoot_all();})
    loader_side_auto.add( [](){ return drive_sys.turn_degrees(60); }); // Angle to point directly upwards. Towards far field edge. // TODO measure this angle once initial shooting angle is determined
    loader_side_auto.add([](){return drive_sys.drive_forward(2, vex::directionType::fwd);}); // Drive to align vertically with the spinners. // TODO measure this distance on the field with the actual robot
    loader_side_auto.add( [](){ return drive_sys.turn_degrees(90); }); // Turn from facing directly upwards to facing the spinner.
    loader_side_auto.add([](){return drive_sys.drive_forward(2, vex::directionType::fwd);}); // Drive until touching the spinner. // TODO measure this distance on the field with the actual robot
    loader_side_auto.add([](){return auto_spin_spinner();}); // TODO implement auto_spin_spinner_to_red based on testing on our field
}

/*
Auto Non-loader side

Map from page 40 of the game manual

(R) = Red Hoop, Blue Zone
(B) = Blue Hoop, Red Zone
 *  = Starting position for this auto
+-------------------+
|        |*___| (R) |
|___           |    |
|   |          |    |
|   |          |    |
|   |          |____|
|___|  ____         |
|(B)  |___*|        |
+-------------------+

 Human Instructions:
 Align robot to specified place and angle using NON LOADER SIDE AUTO jig
*/
GenericAuto auto_non_loader_side(){
    const double non_loader_side_full_court_shot_rpm = 3000;     // TODO measure this RPM based on testing
    GenericAuto non_loader_side_auto;
    //TODO - when shooter is here, use it
    //loader_side_auto.add([](){return shooter.spinToRPM(loader_side_full_court_shot_rpm)})
    //loader_side_auto.add([](){return shooter.shoot_all();})
    non_loader_side_auto.add( [](){ return drive_sys.turn_degrees(-60); }); // Angle to point directly upwards. Towards far field edge. // TODO measure this angle once initial shooting angle is determined
    non_loader_side_auto.add([](){return drive_sys.drive_forward(20 , vex::directionType::fwd);}); // Drive to align horizontally with the spinners. // TODO measure this distance on the field with the actual robot
    non_loader_side_auto.add( [](){ return drive_sys.turn_degrees(90); }); // Turn from facing directly to the side to facing the spinner.
    non_loader_side_auto.add([](){return drive_sys.drive_forward(2, vex::directionType::fwd);}); // Drive until touching the spinner. // TODO measure this distance on the field with the actual robot
    non_loader_side_auto.add([](){return auto_spin_spinner();}); // TODO implement auto_spin_spinner_to_red based on testing on our field
}

