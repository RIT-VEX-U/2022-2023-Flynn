#include "competition/autonomous_flynn.h"
#include "vision.h"
#include "tuning.h"
#include "../core/include/splash.h"

#define CALIBRATE_IMU()       \
  while (imu.isCalibrating()) \
  {                           \
  }

#define TURN_SPEED 0.6
#define INTAKE_VOLT 12
#define SHOOTING_RPM 3200
#define SINGLE_SHOT_TIME 0.2
#define SINGLE_SHOT_VOLT 6
#define SINGLE_SHOT_RECOVER_DELAY_MS 1000

#define DriveToPointSlow(x, y) (new DriveToPointCommand(drive_sys, drive_slow_mprofile, x, y, fwd, 1.0))
#define DriveToPointSlowPt(pt) (new DriveToPointCommand(drive_sys, drive_slow_mprofile, pt, fwd, 1.0))

#define DriveToPointFast(x, y) (new DriveToPointCommand(drive_sys, drive_fast_mprofile, x, y, fwd, 1.0))
#define DriveToPointFastPt(pt) (new DriveToPointCommand(drive_sys, drive_fast_mprofile, pt, fwd, 1.0))
#define DriveToPointFastPtRev(pt) (new DriveToPointCommand(drive_sys, drive_fast_mprofile, pt, reverse, 1.0))

#define DriveForwardFast(dist, dir) (new DriveForwardCommand(drive_sys, drive_fast_mprofile, dist, dir, 1.0))
#define TurnToHeading(heading_deg) (new TurnToHeadingCommand(drive_sys, *config.turn_feedback, heading_deg, TURN_SPEED))

#define TurnToPoint(point) (new TurnToPointCommand(drive_sys, odometry_sys, *config.turn_feedback, point))

#define StartIntake (new StartIntakeCommand(intake, INTAKE_VOLT))
#define StopIntake (new StopIntakeCommand(intake))

#define VisionAim (new VisionAimCommand(true))
#define WaitForFW (new WaitUntilUpToSpeedCommand(flywheel_sys, 10))
#define ShootDisk (new ShootCommand(intake, SINGLE_SHOT_TIME, SINGLE_SHOT_VOLT))
#define SpinFWAt(rpm) (new SpinRPMCommand(flywheel_sys, rpm))

#define PrintOdom (new PrintOdomCommand(odometry_sys))
#define PrintOdomContinous (new PrintOdomContinousCommand(odometry_sys))

#define TRI_SHOT_TIME 1.0
#define TRI_SHOT_VOLT 2
#define TRI_SHOT_RECOVER_DELAY_MS 200

#define AUTO_AIM (new VisionAimCommand())
#define WAIT_FOR_FLYWHEEL (new WaitUntilUpToSpeedCommand(flywheel_sys, 150))
#define TRI_SHOT_DISK (new ShootCommand(intake, TRI_SHOT_TIME, TRI_SHOT_VOLT))

#define SHOOT_DISK (new ShootCommand(intake, SINGLE_SHOT_TIME, SINGLE_SHOT_VOLT))
#define CLEAR_DISKS (new ShootCommand(intake, .75, SINGLE_SHOT_VOLT))

static void add_single_shot_cmd(CommandController &controller, double vis_timeout = 1.0)
{
  controller.add(WaitForFW, vis_timeout);
  if (vis_timeout == 0.0)
    controller.add(VisionAim);
  else
    controller.add(VisionAim, vis_timeout);
  controller.add(ShootDisk);
  controller.add_delay(600);
}

static void add_tri_shot_cmd(CommandController &controller, double timeout = 0.0)
{
  controller.add(WAIT_FOR_FLYWHEEL, timeout);
  controller.add(TRI_SHOT_DISK, 2.0);
  controller.add_delay(TRI_SHOT_RECOVER_DELAY_MS);
}

void pleasant_opcontrol();

void test_stuff()
{

  CALIBRATE_IMU();
pleasant_opcontrol();

  CommandController mine = auto_loader_side();
  mine.run();
  return;
  //
  pleasant_opcontrol();

  // CommandController mine = prog_skills_loader_side();
  // mine.run();
  // vex_printf("timedout %d\n", mine.last_command_timed_out());
  // vex_printf("finshed\n");
}

void pleasant_opcontrol()
{
  // Initialization
  double oneshot_time = .02; // Change 1 second to whatever is needed
  bool oneshotting = false;

  main_controller.ButtonUp.pressed([]()
                                   { flywheel_sys.spinRPM(flywheel_sys.getDesiredRPM() + 250); });
  main_controller.ButtonDown.pressed([]()
                                     { flywheel_sys.spinRPM(flywheel_sys.getDesiredRPM() - 250); });
  main_controller.ButtonR1.pressed([]()
                                   { intake.spin(reverse, 12, volt); }); // Intake
  main_controller.ButtonR2.pressed([]()
                                   { intake.spin(fwd, 12, volt); }); // Shoot

  main_controller.ButtonB.pressed([]()
                                  { odometry_sys.set_position(); });

  // intake.spin(fwd, 12, volt);
  main_controller.ButtonX.pressed([]()
                                  {  intake.spin(fwd, 12, volt); vexDelay(40); intake.spin(fwd, 0, volt); });

  main_controller.ButtonL2.pressed([]()
                                   { flapup_solenoid.set(false); }); // Single Shoot

  main_controller.ButtonL1.pressed([]()
                                   { flapup_solenoid.set(true); });

  int i = 0;

  VisionAimCommand visaim;

  timer loop_timer;
  loop_timer.reset();
  double delay_time = 0.0;
  double loop_time = 0.0;
  // Periodic
  while (true)
  {
    i++;
    if (i % 5 == 0)
    {
      main_controller.Screen.setCursor(0, 0);
      main_controller.Screen.clearScreen();
      main_controller.Screen.print("fw rpm: %f", flywheel_sys.getRPM());
      main_controller.Screen.setCursor(2, 0);
      main_controller.Screen.print("fw temp: %.1ff", flywheel.temperature(vex::fahrenheit));
      main_controller.Screen.setCursor(4, 0);
      main_controller.Screen.print("bat fw : %.2fv %.2fv", Brain.Battery.voltage(vex::volt), flywheel.voltage(volt));
    }

    // ========== DRIVING CONTROLS ==========
    if (main_controller.ButtonA.pressing())
      visaim.run();
    else
      drive_sys.drive_arcade(main_controller.Axis3.position(pct) / 100.0, main_controller.Axis1.position(pct) / 300.0);

    // ========== MANIPULATING CONTROLS ==========

    if (main_controller.ButtonY.pressing() && main_controller.ButtonRight.pressing())
    {
      endgame_solenoid.set(true);
    }

    oneshotting = oneshot_tmr.time(vex::sec) < oneshot_time;
    if (!main_controller.ButtonR1.pressing() && !main_controller.ButtonR2.pressing() && !oneshotting)
    {
      intake.stop();
    }

    // ========== SECONDARY REMOTE ==========

    // ========== AUTOMATION ==========

    // ======== Real timing ========
    loop_time = loop_timer.time(vex::timeUnits::sec);
    delay_time = max(0.02 - loop_time, 0.0);
    vexDelay((int)(delay_time * 1000));
    loop_timer.reset();
  }
}

/*
Auto loader side

Map from page 40 of the game manual

(R) = Red Hoop, Blue Zone
(B) = Blue Hoop, Red Zone
 *  = Starting position for this auto

           ^ 180 degrees
  +-------------------+
  |        |____| (R) |
  |___           |    |
  | * |          |    |
  |   |          |    |  --> 90 degrees
  |   |          |_*__|
  |___|  ____         |
  |(B)  |____|        |
  +-------------------+
           v 0 degrees

 Human Instructions:
 Align robot to specified place and angle using LOADER SIDE AUTO jig
*/
void add_auto_roller(CommandController &cc, double roller_power, position_t set_pos)
{
  cc.add(new FunctionCommand([=]()
                             {drive_sys.drive_tank(roller_power, roller_power);return false; }),
         0.65);
  cc.add(new DriveStopCommand(drive_sys));
  cc.add(new OdomSetPosition(odometry_sys, set_pos));
  // cc.add(new FunctionCommand([=](){drive_sys.drive_tank(-roller_power*.5, -roller_power*.5);return false; }), 0.15);
  cc.add(DriveForwardFast(7, reverse));
}




CommandController auto_loader_side()
{

  CommandController lsa;
  flapup_solenoid.set(true);
  position_t start_pos = position_t{.x = 36.0, .y = 12.2, .rot = -90};
  position_t roller_in_pos = {.x = 36.0, .y = 4.16, .rot = -90};

  static const double roller_power = .5;


  // Rollers =======================================================
  lsa.add(new OdomSetPosition(odometry_sys, start_pos));
  add_auto_roller(lsa, roller_power, roller_in_pos);
  add_auto_roller(lsa, roller_power, roller_in_pos);
  // add_auto_roller(lsa, roller_power, roller_in_pos);
  // add_auto_roller(lsa, roller_power, roller_in_pos);

  lsa.add(DriveForwardFast(2, reverse));

  Vector2D::point_t out_of_way_pos1 = {.x = 79.0, .y = 12.0};
  Vector2D::point_t shoot_point1 = {.x = 64, .y = 52};


  // First Shot =======================================================
  lsa.add(SpinFWAt(3150));
  lsa.add({
      // Wa
      TurnToPoint(out_of_way_pos1)->withTimeout(2.0),
      DriveToPointFastPt(out_of_way_pos1)->withTimeout(2.0),


      TurnToPoint(shoot_point1)->withTimeout(2.0),
      DriveToPointFastPt(shoot_point1)->withTimeout(2.0),

  });

  lsa.add(TurnToHeading(120.6), 2.0);
  lsa.add(PrintOdom);
  lsa.add(WaitForFW, 1.0);
  lsa.add(ShootDisk);
  lsa.add(WaitForFW, 1.0);
  lsa.add_delay(600);
  lsa.add(CLEAR_DISKS);
  lsa.add_delay(600);

  
  
  Vector2D::point_t pre_stack3_pos = {.x = 64.0, .y = 44.0};
  Vector2D::point_t mid_stack3_pos = {.x = 54.0, .y = 34.0};
  Vector2D::point_t after_stack3_pos = {.x = 42.0, .y = 22.0};

  // Vector2D::point_t shoot_point1 = {.x = 64, .y = 51};
  // double goal_pos1_deg = 120.0;

  // Third Shot =======================================================

  lsa.add(SpinFWAt(3500));
  lsa.add({
      // line up to stack
      TurnToPoint(pre_stack3_pos)->withTimeout(2.0),
      DriveToPointFastPt(pre_stack3_pos)->withTimeout(2.0),

      TurnToPoint(mid_stack3_pos)->withTimeout(2.0),
      DriveToPointFastPt(mid_stack3_pos)->withTimeout(2.0),
      new DelayCommand(200),
      StartIntake,

      // grab stack
      TurnToPoint(after_stack3_pos)->withTimeout(2.0),
      DriveToPointSlowPt(after_stack3_pos)->withTimeout(2.0),

      StopIntake,

      // to shoot point
      TurnToPoint(shoot_point1)->withTimeout(2.0),
      DriveToPointFastPt(shoot_point1)->withTimeout(2.0),

  });

  lsa.add(TurnToHeading(125), 2.0);
  lsa.add(WaitForFW, 1.0);
  lsa.add(ShootDisk);
  lsa.add_delay(2000);
  lsa.add(ShootDisk);
  lsa.add_delay(2200);
  lsa.add(CLEAR_DISKS);
  lsa.add_delay(600);


  Vector2D::point_t disk1_pos = {.x = 84.0, .y = 45.0};
  Vector2D::point_t disk2_pos = {.x = 83.0, .y = 35.0};
  Vector2D::point_t disk3_pos = {.x = 83.0, .y = 27.0};

  Vector2D::point_t lineup_disk2_pos = {.x = 73.0, .y = 41.0};
  Vector2D::point_t lineup_disk3_pos = {.x = 73.0, .y = 32.0};

  // Second Shot =======================================================
  lsa.add(SpinFWAt(3450));
  lsa.add({
      

      // First
      TurnToPoint(disk1_pos)->withTimeout(2.0),
      StartIntake,
      DriveToPointFastPt(disk1_pos)->withTimeout(2.0),

      // Second
      DriveToPointFastPtRev(lineup_disk2_pos)->withTimeout(2.0),
      TurnToPoint(disk2_pos)->withTimeout(2.0),
      DriveToPointFastPt(disk2_pos)->withTimeout(2.0),

      // Third
      DriveToPointFastPtRev(lineup_disk3_pos)->withTimeout(2.0),
      TurnToPoint(disk3_pos)->withTimeout(2.0),
      DriveToPointFastPt(disk3_pos)->withTimeout(2.0),
      StopIntake
  });

  // go shoot
  //  Vector2D::point_t disk3_pos = {.x = 86.0, .y = 27.0};
  Vector2D::point_t halfway = {.x = 75, .y = 39}; // turn here
  Vector2D::point_t shoot_point2 = {.x = 62, .y = 53};

  lsa.add({
      DriveToPointFastPtRev(halfway)->withTimeout(2.0),
      TurnToPoint(shoot_point2)->withTimeout(2.0),
      DriveToPointFastPt(shoot_point2)->withTimeout(2.0),
  });
  lsa.add(TurnToHeading(125), 2.0);
  lsa.add(PrintOdom);
  lsa.add(WaitForFW, 1.0);
  lsa.add(ShootDisk);
  lsa.add(WaitForFW, 1.0);
  lsa.add_delay(600);
  lsa.add(ShootDisk);
  lsa.add(WaitForFW, 1.0);
  lsa.add_delay(1000);
  lsa.add(CLEAR_DISKS);
  lsa.add_delay(600);


  // Line up to 3 stack
  Vector2D::point_t pre_op_lineup = {.x = 43.6, .y = 17.05};
  lsa.add({
      TurnToPoint(pre_op_lineup)->withTimeout(2.0),
      DriveToPointFastPt(pre_op_lineup)->withTimeout(2.0),
      TurnToHeading(130)->withTimeout(2.0),
  });

  // lsa.add(PrintOdomContinous, 60.0);
  return lsa;
}

/*
Skills loader side

Map from page 40 of the game manual

(R) = Red Hoop, Blue Zone
(B) = Blue Hoop, Red Zone
 *  = Starting position for this auto
    -x   ^ 180 degrees
  0-------------------+
  |        |____| (R) |
  |___           |    |
  | * |          |    |
  |   |          |    |  --> 90 degrees (+y)
  |   |          |_*__|
  |___|  ____         |
  |(B)  |____|        |
  +-------------------+ (140, 140)
           v 0 degrees
           (+x)

 Human Instructions:
 Align robot to specified place and angle using LOADER SIDE SKILLS jig
*/

CommandController prog_skills_loader_side()
{
  flapup_solenoid.set(false);
  position_t start_pos = position_t{.x = 32.0, .y = 12.2, .rot = -90};

  CommandController lss;
  lss.add(new OdomSetPosition(odometry_sys, start_pos)); // #1
  // lss.add(PrintOdomContinous);

  static const double roller_power = .5;
  // spin -90 degree roller
  Vector2D::point_t corner_disk_point = {.x = 8, .y = 12};
  lss.add({

      (new FunctionCommand([]()
                           {drive_sys.drive_tank(roller_power, roller_power);return false; }))
          ->withTimeout(0.65),
      (new FunctionCommand([]()
                           {drive_sys.drive_tank(0, 0);return true; })),
      (new OdomSetPosition(odometry_sys, {.x = 32.0, .y = 4.16, .rot = -90})),
      (new FunctionCommand([]()
                           {drive_sys.drive_tank(-roller_power, -roller_power);return false; }))
          ->withTimeout(0.25),

      (new FunctionCommand([]()
                           {drive_sys.drive_tank(roller_power, roller_power);return false; }))
          ->withTimeout(0.65),
      (new FunctionCommand([]()
                           {drive_sys.drive_tank(0, 0);return true; })),
      (new OdomSetPosition(odometry_sys, {.x = 32.0, .y = 4.16, .rot = -90})),
      (new FunctionCommand([]()
                           {drive_sys.drive_tank(-roller_power, -roller_power);return false; }))
          ->withTimeout(0.25),

      DriveForwardFast(4, reverse), // #4
  });

  // intake corner disk
  lss.add({
      TurnToPoint(corner_disk_point)->withTimeout(1.5), // #5
      StartIntake,                                      // #6
      DriveToPointSlowPt(corner_disk_point),            // #7
      DriveForwardFast(4, reverse),                     // #8
      new DelayCommand(1000),                           // #9
      StopIntake                                        // #10
  });
  lss.add(PrintOdom);

  Vector2D::point_t shoot_point = {.x = 10.5, .y = 93};

  // align to 180 degree roller
  lss.add(new SpinRPMCommand(flywheel_sys, 3300)); // #21

  lss.add({
      TurnToHeading(90)->withTimeout(1.5),  // #11
      DriveToPointFast(12, 35),             // #12
      TurnToHeading(180)->withTimeout(1.5), // #13

      // spin 180 degree roller
      DriveForwardFast(2, fwd), // #14
      (new FunctionCommand([]()
                           {drive_sys.drive_tank(+roller_power, +roller_power);return false; }))
          ->withTimeout(0.5),
      (new FunctionCommand([]()
                           {drive_sys.drive_tank(0, 0);return true; })),
      new OdomSetPosition(odometry_sys, {.x = 5.25, .y = 32.82, .rot = 180}),
      (new FunctionCommand([]()
                           {drive_sys.drive_tank(-roller_power, -roller_power);return false; }))
          ->withTimeout(0.25),

      (new FunctionCommand([]()
                           {drive_sys.drive_tank(+roller_power, +roller_power);return false; }))
          ->withTimeout(0.5),
      (new FunctionCommand([]()
                           {drive_sys.drive_tank(0, 0);return true; })),
      new OdomSetPosition(odometry_sys, {.x = 5.25, .y = 32.82, .rot = 180}),
      (new FunctionCommand([]()
                           {drive_sys.drive_tank(-roller_power, -roller_power);return false; }))
          ->withTimeout(0.25),
      DriveForwardFast(8, reverse), // #16

      // drive to shoot point
      TurnToPoint(shoot_point)->withTimeout(1.5),       // #17
      DriveToPointFastPt(shoot_point)->withTimeout(2.0) // #19
  });

  // Shoot
  lss.add(TurnToHeading(71), 0.5); // #20

  add_single_shot_cmd(lss); // 22
  add_single_shot_cmd(lss); // 23
  add_single_shot_cmd(lss); // 24

  lss.add_delay(500);
  // lss.add(PrintOdomContinous); /// the guy youre looking for =================================================================================> :)

  // DISKS AGAINST L PIECE
  Vector2D::point_t disk_pos1 = {.x = 24.0, .y = 84.0};
  Vector2D::point_t disk_pos2 = {.x = 33.0, .y = 84.0};
  Vector2D::point_t disk_pos3 = {.x = 42.0, .y = 83.0};

  Vector2D::point_t disk_prep_pos1 = {.x = 25, .y = 70};
  Vector2D::point_t disk_prep_pos2 = {.x = 28, .y = 70};
  Vector2D::point_t disk_prep_pos3 = {.x = 31, .y = 70};

  // disks against L piece
  lss.add({
      // farthest
      StartIntake,                                             // #26
      DriveToPointFastPtRev(disk_prep_pos1)->withTimeout(2.0), // #27
      TurnToPoint(disk_pos1)->withTimeout(1.5),                // #28
      DriveToPointSlowPt(disk_pos1)->withTimeout(2.0),         // #29

      // middle
      DriveToPointFastPtRev(disk_prep_pos2)->withTimeout(2.0), // #30
      TurnToPoint(disk_pos2)->withTimeout(1.5),                // #31
      DriveToPointSlowPt(disk_pos2)->withTimeout(2.0),         // #32

      // closest disk
      DriveToPointFastPtRev(disk_prep_pos3)->withTimeout(2.0), // #33
      TurnToPoint(disk_pos3)->withTimeout(1.5),                // #34
      DriveToPointSlowPt(disk_pos3)->withTimeout(2.0),         // #35
      DriveToPointFastPtRev(disk_prep_pos3)->withTimeout(2.0), // #36
  });
  lss.add(new SpinRPMCommand(flywheel_sys, 3100)); // #40

  Vector2D::point_t shoot_point_other = {.x = 10.5, .y = 93};

  lss.add(TurnToPoint(shoot_point), 1.5);              // #37
  lss.add(DriveToPointFastPt(shoot_point_other), 4.0); // #38

  lss.add(TurnToHeading(69), 0.5); // #39

  add_single_shot_cmd(lss); // #41
  add_single_shot_cmd(lss); // #42
  add_single_shot_cmd(lss); // #43

  // lss.add(DriveForwardFast(4, reverse));
  //  Wall align
  //  lss.add(new WallAlignCommand(drive_sys, odometry_sys, bumper_dist, NO_CHANGE, 0, -1, 2.0));

  // Arrow 3 -------------------------

  Vector2D::point_t start_of_line = {.x = 36, .y = 57};
  Vector2D::point_t end_of_line = {.x = 63, .y = 84};
  Vector2D::point_t out_of_way_point = {.x = 90, .y = 124};

  // go to line and collect line
  lss.add({
      DriveToPointFastPtRev(disk_prep_pos1)->withTimeout(2.0), // #27

      // Start of line
      TurnToPoint(start_of_line), // #39
      // StartIntake,                       // #40
      DriveToPointFastPt(start_of_line), // #41
      // DriveForwardFast(2, reverse),      // #42

      // Drive to End of line
      // TurnToPoint(end_of_line),        // #43
      // DriveToPointSlowPt(end_of_line), // $44

      // StopIntake, // #45

      TurnToPoint(out_of_way_point),       // #47
      DriveToPointFastPt(out_of_way_point) // #48
  });

  // drive to shooting point
  // Vector2D::point_t shoot_point2 = {.x = 56, .y = 128};
  // lss.add(TurnToPoint(shoot_point2));        // #49
  // lss.add(DriveToPointFastPt(shoot_point2)); // #50

  // face hoop and fire
  // lss.add(TurnToHeading(180));                     // #51
  // lss.add(new SpinRPMCommand(flywheel_sys, 3100)); // #52

  // add_single_shot_cmd(lss);
  // add_single_shot_cmd(lss);
  // add_single_shot_cmd(lss);
  // //lss.add(DriveForwardFast(6, reverse));

  // lss.add(TurnToHeading(0));
  //  lss.add(new WallAlignCommand(drive_sys, odometry_sys, NO_CHANGE, 140 - bumper_dist, -90, -1, 2.0));

  // Vector2D::point_t out_of_way_point2 = {.x = 95, .y = 122};

  // lss.add(TurnToPoint(out_of_way_point2));        // [measure]
  // lss.add(DriveToPointFastPt(out_of_way_point2)); //[measure]

  // Arrow 4 -------------------------
  Vector2D::point_t endgame_point = {.x = 122.36, .y = 124.23};

  lss.add(TurnToPoint(endgame_point), 1.0);        // [measure]
  lss.add(DriveToPointFastPt(endgame_point), 4.0); //[measure]
  lss.add(TurnToHeading(48), 3.0);
  //draw_image();

  //// lss.add(PrintOdomContinous);

  // Vector2D::point_t endgame_point = {.x = 122.36, 124.23};

  // Vector2D::point_t south_disk_pos1 = {.x = 50.0, .y = 112.0};
  // Vector2D::point_t south_disk_pos2 = {.x = 50.0, .y = 103.0};
  // Vector2D::point_t south_disk_pos3 = {.x = 50.0, .y = 95.0};

  // Vector2D::point_t south_disk_prep_pos1 = {.x = 66, .y = 112};
  // Vector2D::point_t south_disk_prep_pos2 = {.x = 66, .y = 106};
  // Vector2D::point_t south_disk_prep_pos3 = {.x = 66, .y = 100};

  // // disks against L piece
  // lss.add({
  //     // farthest
  //     StartIntake,
  //     DriveToPointFastPtRev(south_disk_prep_pos1)->withTimeout(2.0),
  //     TurnToPoint(south_disk_pos1)->withTimeout(1.5),
  //     DriveToPointSlowPt(south_disk_pos1)->withTimeout(2.0),

  //     // middle
  //     DriveToPointFastPtRev(south_disk_prep_pos2)->withTimeout(2.0),
  //     TurnToPoint(south_disk_pos2)->withTimeout(1.5),
  //     DriveToPointSlowPt(south_disk_pos2)->withTimeout(2.0),

  //     // closest disk
  //     DriveToPointFastPtRev(south_disk_prep_pos3)->withTimeout(2.0),
  //     TurnToPoint(south_disk_pos3)->withTimeout(1.5),
  //     DriveToPointSlowPt(south_disk_pos3)->withTimeout(2.0),
  //     DriveToPointFastPtRev(south_disk_prep_pos3)->withTimeout(2.0),
  // });

  // lss.add(TurnToPoint(out_of_way_point));        // [measure]
  // lss.add(DriveToPointFastPt(out_of_way_point)); //[measure]

  // // Move to endgame pos
  // Vector2D::point_t endgame_point = {.x = 122, .y = 122};
  // lss.add(TurnToPoint(endgame_point));
  // lss.add(DriveToPointFastPt(endgame_point)); //[measure]

  // Endgame
  lss.add(TurnToHeading(45)); //[measure]
  lss.add(new EndgameCommand(endgame_solenoid));
  lss.add(PrintOdom);

  return lss;
}
