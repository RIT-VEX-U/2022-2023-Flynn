#include "competition/opcontrol.h"
#include "C:/Users/richi/VEX/2022-2023-Flynn/core/include/subsystems/odometry/odometry_base.h"
#include "robot-config.h"

void test_speed(){
    double accel_time = 1.25;   //Drive forward at 100% for this many seconds.
    double decell_time = 0.25;  //Drive at 0% for this many seconds to measure how we decelerate. 
    double time = 0;
    double dt = 0.02;

    MovingAverage *pos_avg = new MovingAverage(2);
    double displacement= 0;
    double last_displacement= 0;

    MovingAverage *vel_avg = new MovingAverage(5);
    double velocity = 0;
    double last_velocity = 0;

    MovingAverage *acc_avg = new MovingAverage(5);
    double acceleration = 0;

    position_t start_position = odom.get_position();
    position_t position = odom.get_position();
    
    while (time<accel_time + decell_time && main_controller.ButtonA.pressing()){

        if (time<accel_time){
            drive_sys.drive_tank(1, 1);
        } else {
            drive_sys.drive_tank(-0.1, -0.1);
        }
        position = odom.get_position();
        
        
        displacement = OdometryBase::pos_diff(start_position, position);
        pos_avg->add_entry(displacement);
        displacement = pos_avg->get_average();
        
        velocity = (displacement - last_displacement)/dt;
        vel_avg->add_entry(velocity);

        velocity = vel_avg->get_average();

        acceleration = (velocity - last_velocity)/dt;
        acc_avg->add_entry(acceleration);
        acceleration = acc_avg->get_average();


        printf("%f\t%f\t%f\t%f\n", time, displacement, velocity, acceleration);
        fflush(stdout);

        time += dt;
        vexDelay((int)(dt*1000));
        last_displacement = displacement;
        last_velocity = velocity;
    }
}

/**
 * Contains the main loop of the robot code while running in the driver-control period.
 */
void opcontrol()
{
  // Initialization
  double time = 0;
  imu.calibrate();
  while(imu.isCalibrating()){
    vexDelay(20);
  }

  if (1){
    //test_speed();
    
    while (!drive_sys.turn_to_heading(0) && main_controller.ButtonA.pressing()){
      position_t pos = odom.get_position();
      printf("%f\t%f\n", time, pos.rot);
      time+=.02;
      vexDelay(20);
    }
    /*
    while (!drive_sys.drive_to_point(-20, 20, fwd) && main_controller.ButtonA.pressing()){
      position_t pos = odom.get_position();
      printf("%f\t%f\t%f\n", time, pos.x, pos.y);fflush(stdout);
      time+=.02;  
      vexDelay(20);
    }
    */
  }
  
  // Periodic
  while(true)
  {
    drive_sys.drive_tank(main_controller.Axis2.value()*.015,main_controller.Axis3.value()*.015);
    auto pos = odom.get_position();
    printf("(%f)\n", pos.rot);fflush(stdout);
     // ========== DRIVING CONTROLS ==========

    // ========== MANIPULATING CONTROLS ==========

    // ========== SECONDARY REMOTE ==========

    // ========== AUTOMATION ==========    
    time+=.02;

    // Wait 20 milliseconds for control loops to calculate time correctly
    vexDelay(20);
  }
}