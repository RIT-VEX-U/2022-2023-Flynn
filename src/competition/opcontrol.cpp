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
    printf("P");fflush(stdout);
    while (time<accel_time + decell_time && main_controller.ButtonA.pressing()){

        if (time<accel_time){
            drive_sys.drive_tank(1, 1);
                printf("P");fflush(stdout);

        } else {
            drive_sys.drive_tank(-0.1, -0.1);
            printf("NP");fflush(stdout);

        }
        position = odom.get_position();
        
        
        displacement = OdometryBase::pos_diff(start_position, position);
        pos_avg->add_entry(displacement);
        displacement = pos_avg->get_average();
        
        velocity = odom.get_speed();
        vel_avg->add_entry(velocity);

        velocity = vel_avg->get_average();

        acceleration = odom.get_accel();
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

void test_rot_speed(){
    double accel_time = 3.25;   //Drive forward at 100% for this many seconds.
    double decell_time = 0.25;  //Drive at 0% for this many seconds to measure how we decelerate. 
    double time = 0;
    double dt = 0.02;

    MovingAverage *pos_avg = new MovingAverage(5);
    double displacement= 0;
    double last_displacement= 0;

    MovingAverage *vel_avg = new MovingAverage(10);
    double velocity = 0;
    double last_velocity = 0;

    MovingAverage *acc_avg = new MovingAverage(10);
    double acceleration = 0;

    position_t position = odom.get_position();
    
    double full_rotation = 0;
    double last_raw_rotation = 0;
    while (time<.2){
      printf("⏸️");fflush(stdout);
      time+=.02;
      vexDelay(20);
    }
    printf("✅\n");
        
    main_controller.Screen.clearScreen();

    double last_vel_time = 0;
    while (time<accel_time + decell_time && main_controller.ButtonA.pressing()){

        if (time<accel_time){
            drive_sys.drive_tank(1, -1);
        } else {
            drive_sys.drive_tank(-0.1, 0.1);
        }
        position = odom.get_position();
        if (position.rot < last_raw_rotation){
          full_rotation+=1;
        }
        last_raw_rotation = position.rot;

        displacement = 360*full_rotation + position.rot;
        pos_avg->add_entry(displacement);
        displacement = pos_avg->get_average();
        
        if (time - last_vel_time>.2){
          double long_dt = time - last_vel_time;
          velocity = (displacement - last_displacement)/long_dt;
          acceleration = (velocity - last_velocity)/long_dt;
          last_velocity = velocity;

          vel_avg->add_entry(velocity);
          velocity = vel_avg->get_average();
          acc_avg->add_entry(acceleration);
          acceleration = acc_avg->get_average();
        }



        printf("%f\t%f\t%f\t%f\t✅\n", time, displacement, velocity, acceleration);
        fflush(stdout);

        time += dt;
        vexDelay((int)(dt*1000));
        last_displacement = displacement;
    }

}

//output: kS 0.072000, kV 0.000103, kA -0.000003
//output: kS 0.044000, kV 0.000106, kA -0.000003
//output: kS 0.005000, kV 0.000111, kA -0.000003
//output: kS 0.052000, kV 0.000105, kA -0.000003
//output: kS 0.002500, kV 0.000111, kA -0.000010
//output: kS 0.076500, kV 0.000051, kA -0.000005


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
  printf("🍞\n");fflush(stdout);
  
  

  


  if (1){
    //test_rot_speed();
    //FeedForward::ff_config_t f = MotionController::tune_feedforward_turning(drive_sys, odom, 1, 2);
    //printf("output: kS %f, kV %f, kA %f\n", f.kS, f.kV, f.kA);fflush(stdout);
  
    
    while (main_controller.ButtonA.pressing() && !drive_sys.turn_to_heading(270)){
      position_t pos = odom.get_position();
      printf("time: %f\tactual position: %f\n", time, pos.rot);fflush(stdout);
      time+=.02;

      vexDelay(20);
    }
    
    
    
  }
  
  // Periodic
  while(true)
  {
    drive_sys.drive_tank(main_controller.Axis2.value()*.015,main_controller.Axis3.value()*.015);
    auto pos = odom.get_position();
    if (main_controller.ButtonB.pressing()){
      printf("%f\n", pos.rot);fflush(stdout);
    }
    // ========== DRIVING CONTROLS ==========

    // ========== MANIPULATING CONTROLS ==========

    // ========== SECONDARY REMOTE ==========

    // ========== AUTOMATION ==========    
    time+=.02;

    // Wait 20 milliseconds for control loops to calculate time correctly
    vexDelay(20);
  }
}