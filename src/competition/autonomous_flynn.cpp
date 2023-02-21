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
#define SINGLE_SHOT_VOLT 8
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

#define TRI_SHOT_TIME 1
#define TRI_SHOT_VOLT 12
#define TRI_SHOT_RECOVER_DELAY_MS 200


#define AUTO_AIM (new VisionAimCommand())
#define WAIT_FOR_FLYWHEEL (new WaitUntilUpToSpeedCommand(flywheel_sys, 150))
#define TRI_SHOT_DISK (new ShootCommand(intake, TRI_SHOT_TIME, TRI_SHOT_VOLT))


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

static void add_tri_shot_cmd(CommandController &controller, double timeout=0.0)
{
    controller.add(WAIT_FOR_FLYWHEEL, timeout);
    controller.add(TRI_SHOT_DISK);
    controller.add_delay(TRI_SHOT_RECOVER_DELAY_MS);
}


void pleasant_opcontrol();

void draw_image()
{
  Brain.Screen.drawImageFromBuffer(&intense_milk[0], 0, 0, intense_milk_width, intense_milk_height);
}

int stats_on_brain()
{
  static bool wasPressing = false;
  static int current_page = 1;
  const int pages = 4;
  static const int width = 480;
  static const int height = 240;

  GraphDrawer rpm_graph(Brain.Screen, 50, "time", "rpm", vex::blue, true, 0.0, 4000.0);
  GraphDrawer setpt_graph(Brain.Screen, 50, "time", "setpt", vex::red, true, 0.0, 4000.0);
  GraphDrawer output_graph(Brain.Screen, 50, "time", "out", vex::yellow, true, 0.0, 1.0);

  auto draw_page_four = []()
  {
    Brain.Screen.setFillColor(vex::transparent);
    Brain.Screen.setPenColor(vex::white);
    Brain.Screen.setFont(prop20);
    int text_height = 20;

    Brain.Screen.clearScreen();
    Brain.Screen.printAt(2, 1 * text_height, "flywheel", Brain.Battery.voltage(volt));
    Brain.Screen.printAt(2, 2 * text_height, "set: %.2f real: %.2f", flywheel_sys.getDesiredRPM(), flywheel_sys.getRPM());
    Brain.Screen.printAt(2, 3 * text_height, "%.3f volts %.3f amps %.3fwatts", flywheel.voltage(volt), flywheel.current(amp), flywheel.voltage(volt) * flywheel.current(amp));
    Brain.Screen.printAt(2, 4 * text_height, "%.2f F %.2f%% effeciency", flywheel.temperature(fahrenheit), flywheel.efficiency());

    Brain.Screen.printAt(2, 6 * text_height, "battery", Brain.Battery.voltage(volt));
    Brain.Screen.printAt(2, 7 * text_height, "%.2f volts", Brain.Battery.voltage(volt));
    Brain.Screen.printAt(2, 8 * text_height, "%.2f amps", Brain.Battery.current(amp));
    Brain.Screen.printAt(2, 9 * text_height, "%d%%", Brain.Battery.capacity(pct));
    Brain.Screen.printAt(2, 10 * text_height, "%.f F", Brain.Battery.temperature(fahrenheit));
  };
  auto draw_page_one = []()
  {
    Brain.Screen.clearScreen();
    Brain.Screen.setFillColor(vex::transparent);
    Brain.Screen.setPenColor(vex::white);
    Brain.Screen.setFont(prop40);
    int text_height = 40;
    Brain.Screen.printAt(2, 2 * text_height, "odometry");
    auto pos = odometry_sys.get_position();
    Brain.Screen.printAt(2, 2 * text_height, "(%.2f, %.2f) : %.2f", pos.x, pos.y, pos.rot);
  };

  auto draw_page_three = [&rpm_graph, &setpt_graph, &output_graph]()
  {
    int x = 20;
    int y = 20;
    Brain.Screen.clearScreen();

    int graph_w = 200; // width / 2 - 2 * x;
    int graph_h = 200; // height / 2 - 2 * y;
    rpm_graph.draw(x, y, graph_w, graph_h);
    setpt_graph.draw(x, y, graph_w, graph_h);
    output_graph.draw(x, y, graph_w, graph_h);
  };

  auto draw_page_two = []()
  {
    Brain.Screen.drawImageFromBuffer(&intense_milk[0], 0, 0, intense_milk_width, intense_milk_height);
  };

  double t = 0.0;
  while (true)
  {

    if (Brain.Screen.pressing() && !wasPressing)
    {
      if (Brain.Screen.xPosition() > width / 2)
      {
        current_page++;
      }
      else
      {
        current_page--;
      }
    }
    if (Brain.Screen.pressing())
    {
      wasPressing = true;
    }
    else
    {
      wasPressing = false;
    }

    if (current_page > pages - 1)
    {
      current_page = pages - 1;
    }
    else if (current_page < 0)
    {
      current_page = 0;
    }

    Vector2D::point_t p = {.x = t, .y = flywheel_sys.getRPM()};
    Vector2D::point_t p2 = {.x = t, .y = flywheel_sys.getDesiredRPM()};
    Vector2D::point_t p3 = {.x = t, .y = flywheel_sys.getFeedforwardValue() + flywheel_sys.getPIDValue()};

    rpm_graph.add_sample(p);
    setpt_graph.add_sample(p2);
    output_graph.add_sample(p3);

    if (current_page == 0)
    {
      draw_page_one();
    }
    else if (current_page == 1)
    {
      draw_page_two();
    }
    else if (current_page == 2)
    {
      draw_page_three();
    }
    else
    {
      draw_page_four();
    }
    Brain.Screen.setFont(mono20);
    Brain.Screen.printAt(width - 5 * 10, height - 10, "(%d/%d)", current_page + 1, pages);

    vexDelay(100);
    t += .1;
  }
  return 0;
}

void test_stuff()
{

  CALIBRATE_IMU();

  vex::task screen_info_task(stats_on_brain);

  CommandController mine = prog_skills_loader_side();
  mine.run();
  return;

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
CommandController auto_loader_side()
{

  CommandController lsa;
  flapup_solenoid.set(false);
  position_t start_pos = position_t{.x = 32.0, .y = 12.2, .rot = -90};
  static const double roller_power = .5;

  CommandController lss;
  lsa.add(new OdomSetPosition(odometry_sys, start_pos));

  // Roller Shenanigans
  lsa.add({

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

  Vector2D::point_t pre_stack3_pos = {.x = 54.0, .y = 32.0};
  Vector2D::point_t after_stack3_pos = {.x = 70.0, .y = 47.0};

  Vector2D::point_t shoot_point1 = {.x = 64, .y = 51};
  double goal_pos1_deg = 120.0;

  lsa.add({
      // line up to stack
      TurnToPoint(pre_stack3_pos)->withTimeout(2.0),
      DriveToPointFastPt(pre_stack3_pos)->withTimeout(2.0),

      // grab stack
      DriveToPointSlowPt(after_stack3_pos)->withTimeout(2.0),

      // to shoot point
      TurnToPoint(shoot_point1)->withTimeout(2.0),
      DriveToPointFastPt(shoot_point1)->withTimeout(2.0),

      // point to goal
      TurnToHeading(goal_pos1_deg)->withTimeout(2.0),
  });
  
  add_tri_shot_cmd(lsa);

  Vector2D::point_t disk1_pos = {.x = 88.0, .y = 45.0};
  Vector2D::point_t disk2_pos = {.x = 88.0, .y = 36.0};
  Vector2D::point_t disk3_pos = {.x = 88.0, .y = 27.0};

  Vector2D::point_t lineup_disk2_pos = {.x = 71.0, .y = 41.0};
  Vector2D::point_t lineup_disk3_pos = {.x = 70.0, .y = 32.0};

  lsa.add({
      // First
      TurnToPoint(disk1_pos)->withTimeout(2.0),
      DriveToPointFastPt(disk1_pos)->withTimeout(2.0),

      // Second
      DriveToPointFastPtRev(lineup_disk2_pos)->withTimeout(2.0),
      TurnToPoint(disk2_pos)->withTimeout(2.0),
      DriveToPointFastPt(disk2_pos)->withTimeout(2.0),

      // Third
      DriveToPointFastPtRev(lineup_disk3_pos)->withTimeout(2.0),
      TurnToPoint(disk3_pos)->withTimeout(2.0),
      DriveToPointFastPt(disk3_pos)->withTimeout(2.0),
  });

  // go shoot
  Vector2D::point_t shoot_point2 = {.x = 64, .y = 51};
  double goal_pos2_deg = 120.0;

  lsa.add({
      DriveToPointFastPtRev(shoot_point2)->withTimeout(2.0),
      TurnToHeading(goal_pos2_deg)->withTimeout(2.0),
  });

  add_tri_shot_cmd(lsa);

  Vector2D::point_t line_disk_pos1 = {.x = 58.0, .y = 58.0};
  Vector2D::point_t line_disk_pos2 = {.x = 47.0, .y = 47.0};
  Vector2D::point_t pre_line_disk_pos2 = {.x = 47.0, .y = 47.0};
  Vector2D::point_t shoot_point3 = {.x = 61, .y = 54};
  double goal_pos3_deg = 120.0;
  
  // Yoink disks on line
  lsa.add({
    TurnToPoint(line_disk_pos1)->withTimeout(2.0),
    DriveToPointFastPt(line_disk_pos1)->withTimeout(2.0),
    
    DriveToPointFastPtRev(pre_line_disk_pos2)->withTimeout(2.0),
    
    TurnToPoint(line_disk_pos2)->withTimeout(2.0),
    DriveToPointFastPt(line_disk_pos2)->withTimeout(2.0),
    
    DriveToPointFastPtRev(shoot_point3)->withTimeout(2.0),
    TurnToHeading(goal_pos3_deg)->withTimeout(2.0),

  });

  // Shoot line disks
  add_tri_shot_cmd(lsa);

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
  draw_image();

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
