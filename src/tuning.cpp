#include "tuning.h"
#include "robot-config.h"
#include "math.h"

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
    printf("Offaxis Dist: %f\n", offax_dist);
}

// Drive Tuning
void tune_drive_ff_ks()
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

    }else
    {
        drive_sys.stop();
        new_press = true;
    }
}

void tune_drive_ff_kv()
{
    static bool new_press = true;

    if(main_controller.ButtonA.pressing())
    {
        if(new_press)
        {
            reset_avg_counter();
            new_press = false;
        }
        
        drive_sys.drive_arcade(0.5, 0.0);
        double vel = odometry_sys.get_speed();
        double kv = 0.5 / continuous_avg(vel);

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

void tune_drive_pid()
{
    static bool done = false;

    if (main_controller.ButtonA.pressing())
    {
        if(done || drive_sys.drive_to_point(0,24,fwd, drive_fast_mprofile))
            done = true;

    }else
    {
        drive_sys.drive_arcade(main_controller.Axis3.position() / 100.0, main_controller.Axis1.position() / 100.0);
        done = false;
    }
}

void tune_drive_motion_maxv()
{
    static bool new_press = true;

    if(main_controller.ButtonA.pressing())
    {
        if(new_press)
        {
            reset_avg_counter();
            new_press = false;
        }

        drive_sys.drive_arcade(1.0, 0);
        double maxv = continuous_avg(odometry_sys.get_speed());

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

void tune_drive_motion_accel(double maxv)
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

        double accel = continuous_avg(odometry_sys.get_accel());

        if(done || odometry_sys.get_speed() >= maxv)
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

}

void tune_flywheel_pid()
{

}

void tune_flywheel_distcalc()
{

}