#include "competition/autonomous.h"


//functions to choose which auto we want. construct a GenericAuto when chosen.
GenericAuto auto_loader_side(){}
GenericAuto auto_non_loader_side(){};

GenericAuto prog_skills_loader_side(){};
GenericAuto priog_skills_non_loader_size(){};

bool auto_spin_spinner_to_red();
bool auto_spin_spinner_to_blue();

/**
 * Contains all the code run during autonomous.
 */ 
void autonomous()
{
    GenericAuto current_auto = auto_loader_side();    

    //my_auto.add( [](){ return drive_system.drive_to_point(12, 24, 0.8, 1, directionType::fwd); } );

}

/**
 * Human Instructions:
 * Align robot to specified place and angle using LOADER SIDE AUTO jig
*/

GenericAuto auto_loader_side(){
    const double loader_side_full_court_shot_rpm = 3000;    
    GenericAuto loader_side_auto;
    //TODO - when shooter is here, use it
    //loader_side_auto.add([](){return shooter.spinToRPM(loader_side_full_court_shot_rpm)})
    //loader_side_auto.add([](){return shooter.shoot_all();})
    loader_side_auto.add( [](){ return drive_sys.turn_degrees(60); }); // Angle to point directly upwards. Towards far field edge. // TODO measure this angle once initial shooting angle is determined
    loader_side_auto.add([](){return drive_sys.drive_forward(2, vex::directionType::fwd);}); // Drive to align vertically with the spinners. // TODO measure this distance on the field with the actual robot
    loader_side_auto.add( [](){ return drive_sys.turn_degrees(90); }); // Turn from facing directly upwards to facing the spinner.
    loader_side_auto.add([](){return drive_sys.drive_forward(2, vex::directionType::fwd);}); // Drive until touching the spinner. // TODO measure this distance on the field with the actual robot
    loader_side_auto.add([](){return auto_spin_spinner_to_red();}); // TODO implement auto_spin_spinner_to_red based on testing on our field
}

