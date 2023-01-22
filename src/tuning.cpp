#include "tuning.h"
#include "robot-config.h"
#include "math.h"
#include "core.h"

#define ENC_IN(enc) (enc.rotation(rev) * PI * odometry_cfg.wheel_diam)
#define ENC_DIFF_IN(left,right) (fabs(ENC_IN(left)-ENC_IN(right))/2.0)

double stored_avg = 0;
double stored_num = 0;
double continuous_avg(double updateval)
{
    stored_num++;
    return (stored_avg * (stored_num-1) / stored_num) + (updateval / stored_num);
}

void reset_avg_counter()
{
    stored_avg = 0;
    stored_num = 0;
}

// Odometry Tuning

void tune_odometry_wheel_diam()
{
    if(main_controller.ButtonA.pressing())
    {
        left_enc.resetRotation();
        right_enc.resetRotation();
    }
    double avg = (fabs(left_enc.rotation(rev)) + fabs(right_enc.rotation(rev))) / 2.0;
    double diam = 100.0 / (avg * PI);

    main_controller.Screen.clearScreen();
    main_controller.Screen.setCursor(1,1);
    main_controller.Screen.print("Push robot 100 inches");
    main_controller.Screen.setCursor(2,1);
    main_controller.Screen.print("Diameter: %f", diam);
    printf("Diameter: %f\n", diam);
}

void tune_odometry_wheelbase()
{
    if(main_controller.ButtonA.pressing())
    {
        left_enc.resetRotation();
        right_enc.resetRotation();
    }
    double radius =  ENC_DIFF_IN(left_enc, right_enc) / (5 * 2 * PI); // radius = arclength / theta
    double wheelbase = 2 * radius;

    main_controller.Screen.clearScreen();
    main_controller.Screen.setCursor(1,1);
    main_controller.Screen.print("Turn the robot in place 5 times");
    main_controller.Screen.setCursor(2,1);
    main_controller.Screen.print("Wheelbase: %f", wheelbase);
    printf("Wheelbase: %f\n", wheelbase);
}

void tune_odometry_offax_dist()
{
    // Other values must be tuned first!
    if(main_controller.ButtonA.pressing())
    {
        left_enc.resetRotation();
        right_enc.resetRotation();
        mid_enc.resetRotation();
    }
    
    double angle_rad = ENC_DIFF_IN(left_enc, right_enc) / odometry_cfg.wheelbase_dist;
    double offax_dist = ENC_IN(mid_enc) / angle_rad;

    main_controller.Screen.clearScreen();
    main_controller.Screen.setCursor(1,1);
    main_controller.Screen.print("Turn the robot in place 5 times");
    main_controller.Screen.setCursor(2,1);
    main_controller.Screen.print("Offaxis Dist: %f", offax_dist);
    printf("Angle: %f, Offaxis Dist: %f\n", angle_rad, offax_dist);
}

// Drive Tuning
void tune_drive_ff_ks(DriveType dt)
{
    static timer tmr;
    static double last_time = 0.0;
    static double test_pct = 0.0;
    static bool new_press = true;
    static bool done = false;

    if(main_controller.ButtonA.pressing())
    {  
        if(new_press)
        {
            // Initialize the function once
            tmr.reset();
            test_pct = 0.0;
            done = false;
            new_press = false;
        }

        if (done || odometry_sys.get_speed() > 0)
        {
            main_controller.Screen.clearScreen();
            main_controller.Screen.setCursor(1,1);
            main_controller.Screen.print("kS: %f", test_pct);
            printf("kS: %f\n", test_pct);
            done = true;
            return;
        }else
        {
            main_controller.Screen.clearScreen();
            main_controller.Screen.setCursor(1,1);
            main_controller.Screen.print("Running...");
        }

       if (tmr.time() - last_time > 500)
            test_pct += 0.01;

        if(dt == DRIVE)
            drive_sys.drive_tank(test_pct, test_pct);
        else if(dt == TURN)
            drive_sys.drive_tank(test_pct, -test_pct);

    }else
    {
        drive_sys.stop();
        new_press = true;
    }
}

void tune_drive_ff_kv(DriveType dt, double ks)
{
    static bool new_press = true;

    if(main_controller.ButtonA.pressing())
    {
        if(new_press)
        {
            reset_avg_counter();
            new_press = false;
        }

        double vel = 0;
        
        if(dt == DRIVE)
        {
            drive_sys.drive_tank(0.5, 0.5);
            vel = odometry_sys.get_speed();
        } else if(dt == TURN)
        {
            drive_sys.drive_tank(0.5, -0.5);
            vel = odometry_sys.get_angular_speed_deg();
        }
        
        double kv = (0.5-ks) / continuous_avg(vel);

        main_controller.Screen.clearScreen();
        main_controller.Screen.setCursor(1,1);
        main_controller.Screen.print("kV: %f", kv);
        printf("kV: %f\n", kv);
    }
    else
    {
        drive_sys.stop();
        new_press = true;
    }

}

void tune_drive_pid(DriveType dt)
{
    static bool done = false;

    if(main_controller.ButtonB.pressing())
        odometry_sys.set_position();

    if (main_controller.ButtonA.pressing())
    {
        if(dt == DRIVE && (done || drive_sys.drive_to_point(0,24,fwd, drive_fast_mprofile)))
            done = true;
        
        if(dt == TURN && (done || drive_sys.turn_to_heading(270, turn_fast_mprofile)))
            done = true;

    }else
    {
        drive_sys.drive_arcade(main_controller.Axis3.position() / 100.0, main_controller.Axis1.position() / 100.0);
        done = false;
    }
}

void tune_drive_motion_maxv(DriveType dt)
{
    static bool new_press = true;

    if(main_controller.ButtonA.pressing())
    {
        if(new_press)
        {
            reset_avg_counter();
            new_press = false;
        }

        double maxv = 0;

        if(dt == DRIVE)
        {
            drive_sys.drive_tank(1.0, 1.0);
            maxv = continuous_avg(odometry_sys.get_speed());
        }else if (dt == TURN)
        {
            drive_sys.drive_tank(1.0, -1.0);
            maxv = continuous_avg(odometry_sys.get_angular_speed_deg());
        }    

        main_controller.Screen.clearScreen();
        main_controller.Screen.setCursor(1,1);
        main_controller.Screen.print("maxV: %f", maxv);
        printf("maxV: %f\n", maxv);

    }else
    {
        drive_sys.stop();
        new_press = true;
    }
}

void tune_drive_motion_accel(DriveType dt, double maxv)
{
    static bool done = false;
    static bool new_press = true;

    if(main_controller.ButtonA.pressing())
    {
        if(new_press)
        {
            reset_avg_counter();
            done = false;
            new_press = false;
        }

        double accel = 0;
        double vel = 0;

        if(dt == DRIVE)
        {
            vel = odometry_sys.get_speed();
            accel = continuous_avg(odometry_sys.get_accel());
            drive_sys.drive_tank(1.0, 1.0);
        } else if (dt == TURN)
        {
            vel = odometry_sys.get_angular_speed_deg();
            accel = continuous_avg(odometry_sys.get_angular_accel_deg());
            drive_sys.drive_tank(1.0, -1.0);
        }

        if(done || vel >= maxv)
        {
            main_controller.Screen.clearScreen();
            main_controller.Screen.setCursor(1,1);
            main_controller.Screen.print("Accel: %f", accel);
            printf("Accel: %f\n", accel);
            done = true;
            return;
        }else
        {
            main_controller.Screen.clearScreen();
            main_controller.Screen.setCursor(1,1);
            main_controller.Screen.print("Running...");
        }
        
    }else
    {
        drive_sys.stop();
        new_press = true;
    }
}


// Flywheel Tuning
void tune_flywheel_ff()
{ 
    static bool new_press = true;
    if(main_controller.ButtonA.pressing())
    {
        if(new_press)
        {
            reset_avg_counter();
            new_press = false;
        }

        flywheel_sys.spin_raw(0.5);
        double rpm = flywheel_sys.getRPM();
        double kv = 0.5 / continuous_avg(rpm);

        main_controller.Screen.clearScreen();
        main_controller.Screen.setCursor(1, 1);
        main_controller.Screen.print("kv: %f", kv);
        printf("rpm: %f, kV: %f\n", rpm, kv);
    }else
    {
        flywheel_sys.stop();
        new_press = true;
    }
    
}

void tune_flywheel_pid()
{
    static bool first_run = true;
    static int setpt_rpm = 0;
    if(first_run)
    {
        main_controller.ButtonUp.pressed([](){setpt_rpm+=500;});
        main_controller.ButtonDown.pressed([](){setpt_rpm-=500;});
        main_controller.ButtonRight.pressed([](){setpt_rpm+=50;});
        main_controller.ButtonLeft.pressed([](){setpt_rpm-=50;});
    }

    static bool new_press = true;
    if(main_controller.ButtonA.pressing())
    {
        if(new_press)
        {
            new_press = false;
        }

        flywheel_sys.spinRPM(setpt_rpm);
        double rpm = flywheel_sys.getRPM();

        main_controller.Screen.clearScreen();
        main_controller.Screen.setCursor(1, 1);
        main_controller.Screen.print("rpm: %f", rpm);
        main_controller.Screen.setCursor(2,1);
        main_controller.Screen.print("setpt: %d", setpt_rpm);
        printf("setpt: %d, rpm: %f\n", setpt_rpm, rpm);
    }else
    {
        flywheel_sys.stop();
        new_press = true;
    }
}

void tune_flywheel_distcalc()
{
    static bool first_run = true;
    static int setpt_rpm = 0;
    if(first_run)
    {
        main_controller.ButtonUp.pressed([](){setpt_rpm+=500;});
        main_controller.ButtonDown.pressed([](){setpt_rpm-=500;});
        main_controller.ButtonRight.pressed([](){setpt_rpm+=50;});
        main_controller.ButtonLeft.pressed([](){setpt_rpm-=50;});
    }

    main_controller.Screen.clearScreen();
    main_controller.Screen.setCursor(1,1);
    main_controller.Screen.print("setpt: %d", setpt_rpm);
    printf("setpt: %d, rpm: %f\n", setpt_rpm, flywheel_sys.getRPM());
}