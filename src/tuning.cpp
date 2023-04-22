#include "tuning.h"
#include "robot-config.h"
#include "math.h"
#include "core.h"

#define ENC_IN(enc) (enc.position(rev) * PI * config.odom_wheel_diam)
#define ENC_DIFF_IN(left, right) (fabs(ENC_IN(left) - ENC_IN(right)) / 2.0)

double stored_avg = 0;
double stored_num = 0;
double continuous_avg(double updateval)
{
    stored_num++;

    if (stored_num < 1)
    {
        stored_avg = 0.0;
    }
    else
    {
        stored_avg = (stored_avg * (stored_num - 1) / stored_num) + (updateval / stored_num);
    }
    return stored_avg;
}

void reset_avg_counter()
{
    stored_avg = 0;
    stored_num = 0;
}

// Odometry Tuning
void tune_odometry_gear_ratio_right_wheel()
{
    if (main_controller.ButtonA.pressing())
    {
        // SET THESE BACK TO LEFT ENC RIGHT ENC
        right_enc.resetRotation();
    }
    double ratio = right_enc.rotation(rev);

    main_controller.Screen.clearScreen();
    main_controller.Screen.setCursor(1, 1);
    main_controller.Screen.print("turn right odom wheel 1 rev");
    main_controller.Screen.setCursor(2, 1);
    main_controller.Screen.print("ratio: %f", ratio);
    printf("ratio: %f", ratio);
}

void tune_odometry_wheel_diam()
{
    if (main_controller.ButtonA.pressing())
    {
        // SET THESE BACK TO LEFT ENC RIGHT ENC
        left_motors.resetPosition();
        right_motors.resetPosition();
    }
    // double avg = (fabs(left_enc.rotation(rev)) + fabs(right_enc.rotation(rev))) / 2.0;
    double avg = (fabs(left_motors.position(rev)) + fabs(right_motors.position(rev))) / 2.0;
    if (fabs(avg) < .1)
    {
        printf("Diam: 0\n");
        return;
    }
    double diam = 96.0 / (avg * PI);

    main_controller.Screen.clearScreen();
    main_controller.Screen.setCursor(1, 1);
    main_controller.Screen.print("Push robot 96.0 inches (4 tiles)");
    main_controller.Screen.setCursor(2, 1);
    main_controller.Screen.print("Diam: %f", diam);
    printf("Diam: %f\n", diam);
}

void tune_odometry_wheelbase()
{
    int times_to_turn = 15;
    if (main_controller.ButtonA.pressing())
    {
        // left_enc.resetRotation();
        // right_enc.resetRotation();
        left_motors.resetPosition();
        right_motors.resetPosition();
    }
    // double radius =  ENC_DIFF_IN(left_enc, right_enc) / ((double)times_to_turn * 2 * PI); // radius = arclength / theta
    double radius = ENC_DIFF_IN(left_motors, right_motors) / ((double)times_to_turn * 2 * PI); // radius = arclength / theta

    double wheelbase = 2 * radius;

    main_controller.Screen.clearScreen();
    main_controller.Screen.setCursor(1, 1);
    main_controller.Screen.print("Turn the robot in place %d times", times_to_turn);
    main_controller.Screen.setCursor(2, 1);
    main_controller.Screen.print("Wheelbase: %f", wheelbase);
    printf("Wheelbase: %f\n", wheelbase);
}

void tune_odometry_offax_dist()
{
    // Other values must be tuned first!
    if (main_controller.ButtonA.pressing())
    {
        left_enc.resetRotation();
        right_enc.resetRotation();
        mid_enc.resetRotation();
    }

    double angle_rad = ENC_DIFF_IN(left_enc, right_enc) / config.dist_between_wheels;
    double offax_dist = ENC_IN(mid_enc) / angle_rad;

    main_controller.Screen.clearScreen();
    main_controller.Screen.setCursor(1, 1);
    main_controller.Screen.print("Turn the robot in place 5 times");
    main_controller.Screen.setCursor(2, 1);
    main_controller.Screen.print("Offaxis Dist: %f", offax_dist);
    printf("Angle: %f, Offaxis Dist: %f\n", angle_rad, offax_dist);
}

// Drive Tuning
void tune_drive_ff_ks(DriveType dt)
{
    static timer tmr;
    static double test_pct = 0.0;
    static bool new_press = true;
    static bool done = false;

    if (main_controller.ButtonA.pressing())
    {
        if (new_press)
        {
            // Initialize the function once
            tmr.reset();
            left_enc.resetRotation();
            right_enc.resetRotation();
            // left_motors.resetPosition();
            // right_motors.resetPosition();
            test_pct = 0.0;
            done = false;
            new_press = false;
        }

        if (done || (fabs(left_enc.position(rev)) + fabs(right_enc.position(rev))) > 0)
        {
            main_controller.Screen.clearScreen();
            main_controller.Screen.setCursor(1, 1);
            main_controller.Screen.print("kS: %f", test_pct);
            printf("kS: %f\n", test_pct);
            done = true;
            return;
        }
        else
        {
            main_controller.Screen.clearScreen();
            main_controller.Screen.setCursor(1, 1);
            main_controller.Screen.print("Running...");
        }

        if (tmr.time() > 500)
        {
            test_pct += 0.01;
            tmr.reset();
        }

        if (dt == DRIVE)
            drive_sys.drive_tank(test_pct, test_pct);
        else if (dt == TURN)
            drive_sys.drive_tank(test_pct, -test_pct);
    }
    else
    {
        drive_sys.stop();
        new_press = true;
    }
}

void tune_drive_ff_kv(DriveType dt, double ks)
{
    static bool new_press = true;

    if (main_controller.ButtonA.pressing())
    {
        if (new_press)
        {
            reset_avg_counter();
            new_press = false;
        }

        double vel = 0;

        if (dt == DRIVE)
        {
            drive_sys.drive_tank(0.5, 0.5);
            vel = odometry_sys.get_speed();
        }
        else if (dt == TURN)
        {
            drive_sys.drive_tank(0.5, -0.5);
            vel = odometry_sys.get_angular_speed_deg();
        }

        double kv = (0.5 - ks) / continuous_avg(vel);

        main_controller.Screen.clearScreen();
        main_controller.Screen.setCursor(1, 1);
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

    if (main_controller.ButtonB.pressing())
        odometry_sys.set_position();

    // auto pos = odometry_sys.get_position();
    // printf("%.2f, %.2f, %.2f\n", pos.x, pos.y, pos.rot);

    if (main_controller.ButtonA.pressing())
    {
        if (dt == DRIVE && (done || drive_sys.drive_to_point(24, 24, fwd, drive_fast_mprofile)))
        {
            auto pos = odometry_sys.get_position();
            printf("%.2f, %.2f, %.2f\n", pos.x, pos.y, pos.rot);

            printf("Finished\n");
            fflush(stdout);
            done = true;
        }

        if (dt == TURN && (done || drive_sys.turn_to_heading(180, *config.turn_feedback, 1.0)))
        {
            auto pos = odometry_sys.get_position();
            printf("%.2f, %.2f, %.2f\n", pos.x, pos.y, pos.rot);

            printf("Finished\n");
            fflush(stdout);

            done = true;
        }
    }
    else
    {
        drive_sys.drive_arcade(main_controller.Axis3.position() / 100.0, main_controller.Axis1.position() / 100.0);
        drive_sys.reset_auto();
        done = false;
    }
}

void tune_drive_motion_maxv(DriveType dt)
{
    static bool new_press = true;

    if (main_controller.ButtonA.pressing())
    {
        if (new_press)
        {
            reset_avg_counter();
            new_press = false;
        }

        double maxv = 0;

        if (dt == DRIVE)
        {
            drive_sys.drive_tank(1.0, 1.0);
            maxv = continuous_avg(odometry_sys.get_speed());
        }
        else if (dt == TURN)
        {
            drive_sys.drive_tank(1.0, -1.0);
            maxv = continuous_avg(odometry_sys.get_angular_speed_deg());
        }

        main_controller.Screen.clearScreen();
        main_controller.Screen.setCursor(1, 1);
        main_controller.Screen.print("maxV: %f", maxv);
        printf("maxV: %f\n", maxv);
    }
    else
    {
        drive_sys.stop();
        new_press = true;
    }
}

void tune_drive_motion_accel(DriveType dt, double maxv)
{
    static bool done = false;
    static bool new_press = true;
    static double accel = 0;

    if (main_controller.ButtonA.pressing())
    {
        if (new_press)
        {
            reset_avg_counter();
            done = false;
            new_press = false;
        }

        double vel = 0;

        if (!done && dt == DRIVE)
        {
            vel = odometry_sys.get_speed();
            accel = odometry_sys.get_accel();
        }
        else if (!done)
        {
            vel = odometry_sys.get_angular_speed_deg();
            accel = odometry_sys.get_angular_accel_deg();
        }

        if (done || vel >= maxv)
        {
            main_controller.Screen.clearScreen();
            main_controller.Screen.setCursor(1, 1);
            main_controller.Screen.print("Accel: %f", accel);
            printf("Accel: %f\n", accel);
            done = true;
            drive_sys.stop();
            return;
        }

        main_controller.Screen.clearScreen();
        main_controller.Screen.setCursor(1, 1);
        main_controller.Screen.print("Running...");

        if (dt == DRIVE)
            drive_sys.drive_tank(1.0, 1.0);
        else
            drive_sys.drive_tank(1.0, -1.0);
    }
    else
    {
        drive_sys.stop();
        new_press = true;
    }
}

// Flywheel Tuning
//.5 .000340
//.75 .000316
// 1.0 0.000299
void tune_flywheel_ff()
{
    double flywheel_target_pct = 0.7;
    static bool new_press = true;
    static int counter = 0;
    static MovingAverage avg(20, 0.0);
    static MovingAverage var(20, 0.0);
    if (main_controller.ButtonA.pressing())
    {
        counter++;
        if (new_press)
        {
            reset_avg_counter();
            new_press = false;
            printf("rpm kv");
        }

        flywheel_sys.spin_raw(flywheel_target_pct);

        if (counter < 30)
        {
            return;
        }

        double rawRPM = 18.0 * flywheel_motors.velocity(velocityUnits::rpm);

        double rpm = rawRPM; // flywheel_sys.getRPM();
        double kv = flywheel_target_pct / continuous_avg(rpm);

        avg.add_entry(rpm);
        double x_bar = avg.get_average();
        double squared = (rawRPM - x_bar) * (rawRPM - x_bar);
        var.add_entry(squared);
        double sd = sqrt(var.get_average());

        main_controller.Screen.clearScreen();
        main_controller.Screen.setCursor(1, 1);
        main_controller.Screen.print("kv: %f", kv);
        printf("%f %f %f\n", rpm, kv, sd);
    }
    else
    {
        flywheel_sys.stop();
        new_press = true;
    }
}

void tune_flywheel_pid()
{
    static bool first_run = true;
    static int setpt_rpm = 0;
    if (first_run)
    {
        main_controller.ButtonUp.pressed([]()
                                         { setpt_rpm += 500; });
        main_controller.ButtonDown.pressed([]()
                                           { setpt_rpm -= 500; });
        main_controller.ButtonRight.pressed([]()
                                            { setpt_rpm += 50; });
        main_controller.ButtonLeft.pressed([]()
                                           { setpt_rpm -= 50; });
    }

    static bool new_press = true;
    if (main_controller.ButtonA.pressing())
    {
        if (new_press)
        {
            new_press = false;
        }

        flywheel_sys.spinRPM(setpt_rpm);
        double rpm = flywheel_sys.getRPM();

        main_controller.Screen.clearScreen();
        main_controller.Screen.setCursor(1, 1);
        main_controller.Screen.print("rpm: %f", rpm);
        main_controller.Screen.setCursor(2, 1);
        main_controller.Screen.print("setpt: %d", setpt_rpm);
        printf("setpt %d rpm: %f\n", setpt_rpm, rpm);
    }
    else
    {
        flywheel_sys.stop();
        new_press = true;
    }
}

void tune_flywheel_distcalc()
{
    static bool first_run = true;
    static int setpt_rpm = 0;
    static double t = 0;
    static MovingAverage avg_err = MovingAverage(10, 0);
    if (first_run)
    {
        main_controller.ButtonUp.pressed([]()
                                         { setpt_rpm += 500; });
        main_controller.ButtonDown.pressed([]()
                                           { setpt_rpm -= 500; });
        main_controller.ButtonRight.pressed([]()
                                            { setpt_rpm += 25; });
        main_controller.ButtonLeft.pressed([]()
                                           { setpt_rpm -= 25; });
        main_controller.ButtonX.pressed([]()
                                        { setpt_rpm = 0; });
        main_controller.ButtonA.pressed([]()
                                        { intake.spin(fwd, 4, volt); });
        main_controller.ButtonA.released([]()
                                         { intake.spin(fwd, 0, volt); });
        main_controller.ButtonA.pressed([]()
                                        { intake.spin(fwd, 4, volt); });
        main_controller.ButtonA.released([]()
                                         { intake.spin(fwd, 0, volt); });
        first_run = false;
        flywheel.setMaxTorque(100.0, vex::pct);
        printf("time setpt rpm fb_out volts amps\n");
    }
    t += .02;
    flywheel_sys.spinRPM(setpt_rpm);
    main_controller.Screen.clearScreen();
    main_controller.Screen.setCursor(1, 1);
    main_controller.Screen.print("setpt: %d", setpt_rpm);
    avg_err.add_entry(fabs(flywheel_sys.getRPM() - flywheel_sys.getDesiredRPM()));
    printf("%f %d %f %f %f %f \n", t, setpt_rpm, flywheel_sys.getRPM(), flywheel_sys.getFeedforwardValue() + flywheel_sys.getPIDValue(), flywheel.voltage(volt), flywheel.current(amp));
}
void tune_generic_pid(Feedback &pid2tune, double error_lower_bound, double error_upper_bound)
{
    if (pid2tune.get_type() != Feedback::FeedbackType::PIDType)
    {
        printf("Cant tune a pid on not a pid? what are you trying to do. I'm not sad just dissapointed. %s:%d\n", __FILE__, __LINE__);
        return;
    }
    static PID &pid = static_cast<PID &>(pid2tune);

    static int selection = 0;
    static double sensitivity = 0.0001;
    static bool func_init = false;

    if (!func_init)
    {
        main_controller.ButtonRight.pressed([]()
                                            {selection++; if (selection > 2){selection = 2;}; });
        main_controller.ButtonLeft.pressed([]()
                                           {selection--; if (selection < 0){selection = 0;} });
        main_controller.ButtonL1.pressed([]()
                                         { sensitivity /= 10.0; });
        main_controller.ButtonR1.pressed([]()
                                         { sensitivity *= 10.0; });
        main_controller.ButtonUp.pressed([]()
                                         { if (selection == 0){pid.config.p += sensitivity;} else if (selection == 1){pid.config.i += sensitivity;} else {pid.config.d += sensitivity;} printf("{\n\tkP: %f\n\tkI: %f\n\tkD: %f\n\tontime: %f\n\tdeadband: %f\n}", pid.config.p, pid.config.i, pid.config.d, pid.config.on_target_time, pid.config.deadband); });
        main_controller.ButtonDown.pressed([]()
                                           { if (selection == 0){pid.config.p -= sensitivity;} else if (selection == 1){pid.config.i -= sensitivity;} else {pid.config.d -= sensitivity;}printf("{\n\tkP: %f\n\tkI: %f\n\tkD: %f\n\tontime: %f\n\tdeadband: %f\n}", pid.config.p, pid.config.i, pid.config.d, pid.config.on_target_time, pid.config.deadband); });
        main_controller.ButtonR2.pressed([]()
                                         { if (selection == 0){pid.config.p = 0.0 ;} else if (selection == 1){pid.config.i = 0.0;} else {pid.config.d = 0.0;} printf("{\n\tkP: %f\n\tkI: %f\n\tkD: %f\n\tontime: %f\n\tdeadband: %f\n}\n", pid.config.p, pid.config.i, pid.config.d, pid.config.on_target_time, pid.config.deadband); });
        main_controller.ButtonL2.pressed([]()
                                         {
            pid.config.p = 0.0;
            pid.config.i = 0.0;
            pid.config.d = 0.0; });

        func_init = true;
    }
    static GraphDrawer output_graph(Brain.Screen, 50, "time", "output", vex::red, true, -1.0, 1.0);
    static GraphDrawer error_graph(Brain.Screen, 50, "time", "error", vex::blue, true, error_lower_bound, error_upper_bound);
    static double t = 0.0;

    double err = pid.get_error();
    double out = pid.get();

    Vector2D::point_t p_out = {.x = t, .y = out};
    Vector2D::point_t p_err = {.x = t, .y = err};

    output_graph.add_sample(p_out);
    error_graph.add_sample(p_err);

    int width = 480;
    int height = 240;
    int margin = 20;

    Brain.Screen.clearScreen();
    // Draw Graph
    output_graph.draw(margin, margin, width / 2 - margin, height - 2 * margin);
    error_graph.draw(margin, margin, width / 2 - margin, height - 2 * margin);

    Brain.Screen.setPenColor(vex::red);
    Brain.Screen.printAt(width / 2 + margin, 80, "out: %.3f", out);

    Brain.Screen.setPenColor(vex::blue);
    Brain.Screen.printAt(width / 2 + margin, 100, "err: %.3f", err);

    Brain.Screen.setPenColor(vex::white);

    // Instructions
    Brain.Screen.printAt(width / 2 + margin, 120, "left, right to select");
    Brain.Screen.printAt(width / 2 + margin, 140, "P, I, or D.");
    Brain.Screen.printAt(width / 2 + margin, 160, "up down to change");
    Brain.Screen.printAt(width / 2 + margin, 180, "const. value");
    Brain.Screen.printAt(width / 2 + margin, 200, "L1, R1 to change");
    Brain.Screen.printAt(width / 2 + margin, 220, "sensitivity */ 10");

    char cons = '?';
    double display_const = 0.0;
    if (selection == 0)
    {
        cons = 'P';
        if (pid.config.p < 0)
        {
            pid.config.p = 0.0;
        }
        display_const = pid.config.p;
    }
    else if (selection == 1)
    {
        cons = 'I';
        if (pid.config.i < 0)
        {
            pid.config.i = 0.0;
        }
        display_const = pid.config.i;
    }
    else if (selection == 2)
    {
        cons = 'D';
        if (pid.config.d < 0)
        {
            pid.config.d = 0.0;
        }

        display_const = pid.config.d;
    }

    main_controller.Screen.clearScreen();
    main_controller.Screen.setCursor(0, 0);
    main_controller.Screen.print("k%c: %.6f", cons, display_const);
    main_controller.Screen.setCursor(2, 0);
    main_controller.Screen.print("sens: %.6f", sensitivity);
    main_controller.Screen.setCursor(4, 0);
    main_controller.Screen.print("err: %.4f%s", err, pid.is_on_target() ? " :)" : " ");

    t += 0.02;
}