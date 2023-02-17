#include "competition/autonomous_flynn.h"
#include "vision.h"
#include "tuning.h"
#include "../core/include/intense_milk.h"

#define CALIBRATE_IMU()       \
  while (imu.isCalibrating()) \
  {                           \
  }

#define TURN_SPEED 0.6
#define INTAKE_VOLT 12
#define SHOOTING_RPM 3200
#define SINGLE_SHOT_TIME 0.02
#define SINGLE_SHOT_VOLT 2
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

static void add_single_shot_cmd(CommandController &controller, double vis_timeout = 1.0)
{
  controller.add(WaitForFW, vis_timeout);
  if (vis_timeout == 0.0)
    controller.add(VisionAim);
  else
    controller.add(VisionAim, vis_timeout);
  controller.add(ShootDisk);
  controller.add_delay(1000);
}

void pleasant_opcontrol();

int stats_on_brain()
{
  static bool wasPressing = false;
  static int current_page = 2;
  const int pages = 3;
  static const int width = 480;
  static const int height = 240;

  GraphDrawer rpm_graph(Brain.Screen, 50, "time", "rpm", vex::blue, true, 0.0, 4000.0);
  GraphDrawer setpt_graph(Brain.Screen, 50, "time", "setpt", vex::red, true, 0.0, 4000.0);
  GraphDrawer output_graph(Brain.Screen, 50, "time", "out", vex::yellow, true, 0.0, 1.0);

  // auto draw_page_two = []()
  // {
  //   Brain.Screen.setFillColor(vex::transparent);
  //   Brain.Screen.setPenColor(vex::white);
  //   Brain.Screen.setFont(prop20);
  //   int text_height = 20;

  //   Brain.Screen.clearScreen();
  //   Brain.Screen.printAt(2, 1 * text_height, "flywheel", Brain.Battery.voltage(volt));
  //   Brain.Screen.printAt(2, 2 * text_height, "set: %.2f real: %.2f", flywheel_sys.getDesiredRPM(), flywheel_sys.getRPM());
  //   Brain.Screen.printAt(2, 3 * text_height, "%.3f volts %.3f amps %.3fwatts", flywheel.voltage(volt), flywheel.current(amp), flywheel.voltage(volt) * flywheel.current(amp));
  //   Brain.Screen.printAt(2, 4 * text_height, "%.2f F %.2f%% effeciency", flywheel.temperature(fahrenheit), flywheel.efficiency());

  //   Brain.Screen.printAt(2, 6 * text_height, "battery", Brain.Battery.voltage(volt));
  //   Brain.Screen.printAt(2, 7 * text_height, "%.2f volts", Brain.Battery.voltage(volt));
  //   Brain.Screen.printAt(2, 8 * text_height, "%.2f amps", Brain.Battery.current(amp));
  //   Brain.Screen.printAt(2, 9 * text_height, "%d%%", Brain.Battery.capacity(pct));
  //   Brain.Screen.printAt(2, 10 * text_height, "%.f F", Brain.Battery.temperature(fahrenheit));
  // };
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
    else
    {
      draw_page_three();
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
  while (true)
  {
    PIDFF *turn_pidff = static_cast<PIDFF *>(config.turn_feedback);
    tune_generic_pid(((*turn_pidff).pid), -180, 180);
    tune_drive_pid(TURN);
    vexDelay(30);
  }
  return;
  vex::task screen_info_task(stats_on_brain);

  ////CommandController mine = auto_loader_side();
  // mine.run();

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
  main_controller.ButtonL2.pressed([]()
                                   {intake.spin(fwd, 12, volt);oneshot_tmr.reset(); }); // Single Shoot
  main_controller.ButtonL1.pressed([]()
                                   { roller.spin(vex::reverse, 12, vex::volt); }); // Roller
  main_controller.ButtonL1.released([]()
                                    { roller.stop(); }); // Roller

  main_controller.ButtonB.pressed([]()
                                  { odometry_sys.set_position(); });

  // intake.spin(fwd, 12, volt);
  main_controller.ButtonX.pressed([]()
                                  {  intake.spin(fwd, 12, volt); vexDelay(40); intake.spin(fwd, 0, volt); });

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

  position_t start_pos = position_t{.x = 30.5, .y = 10.2, .rot = -90};
  const double bumper_dist = 6.0;

  CommandController lss;
  lss.add(new OdomSetPosition(odometry_sys, start_pos)); // #1

  // spin -90 degree roller
  Vector2D::point_t corner_disk_point = {.x = 8, .y = 12};
  lss.add({
      DriveForwardFast(1, fwd),                     // #2
      new SpinRollerCommandAUTO(drive_sys, roller), // #3
      DriveForwardFast(4, reverse),                 // #4
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

  // align to 180 degree roller
  lss.add({
      TurnToHeading(90)->withTimeout(1.5), // #11
      DriveToPointFast(12, 31.5),          // #12
      TurnToHeading(180)->withTimeout(1.5) // #13
  });

  // spin 180 degree roller

  lss.add(DriveForwardFast(2, fwd));                     // #14
  lss.add(new SpinRollerCommandAUTO(drive_sys, roller)); // #15
  lss.add(DriveForwardFast(2, reverse));                 // #16

  // spin and shoot 3
  Vector2D::point_t shoot_point = {.x = 12, .y = 78};
  lss.add(TurnToPoint(shoot_point), 1.5);        // #17
  lss.add(DriveToPointFastPt(shoot_point), 4.0); // #19

  lss.add(TurnToHeading(85), 0.5); // #20

  lss.add(new SpinRPMCommand(flywheel_sys, 3100)); // #21

  add_single_shot_cmd(lss); // 22
  add_single_shot_cmd(lss); // 23
  add_single_shot_cmd(lss); // 24

  lss.add_delay(1000);
  // lss.add(PrintOdomContinous); /// the guy youre looking for =================================================================================> :)

  // DISKS AGAINST L PIECE
  Vector2D::point_t disk_pos1 = {.x = 27.0, .y = 88.0};
  Vector2D::point_t disk_pos2 = {.x = 37, .y = 88.0};
  Vector2D::point_t disk_pos3 = {.x = 44.0, .y = 88.0};

  Vector2D::point_t disk_prep_pos1 = {.x = 25, .y = 70};
  Vector2D::point_t disk_prep_pos2 = {.x = 28, .y = 70};
  Vector2D::point_t disk_prep_pos3 = {.x = 13, .y = 70};

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

  lss.add(TurnToPoint(shoot_point), 1.5);        // #37
  lss.add(DriveToPointFastPt(shoot_point), 4.0); // #38

  // Wall align
  lss.add(TurnToHeading(0));
  lss.add(new WallAlignCommand(drive_sys, odometry_sys, bumper_dist, NO_CHANGE, 0, -1, 2.0));

  // Arrow 3 -------------------------

  Vector2D::point_t start_of_line = {.x = 36, .y = 58};
  Vector2D::point_t end_of_line = {.x = 60, .y = 84};

  lss.add(TurnToPoint(start_of_line));        // #39
  lss.add(StartIntake);                       // #40
  lss.add(DriveToPointSlowPt(start_of_line)); // #41
  lss.add(DriveForwardFast(4, reverse));      // #42

  lss.add(TurnToPoint(end_of_line));        // #43
  lss.add(DriveToPointSlowPt(end_of_line)); // 44

  lss.add(StopIntake); // #45

  Vector2D::point_t out_of_way_point = {.x = 70, .y = 124};
  lss.add(TurnToPoint(out_of_way_point));        // [measure]
  lss.add(DriveToPointFastPt(out_of_way_point)); //[measure]

  // drive to shooting point
  Vector2D::point_t shoot_point2 = {.x = 46, .y = 124};
  lss.add(TurnToPoint(shoot_point2));        // [measure]
  lss.add(DriveToPointFastPt(shoot_point2)); //[measure]

  lss.add(TurnToHeading(-90));
  lss.add(new WallAlignCommand(drive_sys, odometry_sys, NO_CHANGE, 140 - bumper_dist, -90, -1, 2.0));

  // face hoop and fire
  lss.add(TurnToHeading(180));                     // [measure]
  lss.add(new SpinRPMCommand(flywheel_sys, 3100)); // [measure]

  add_single_shot_cmd(lss);
  add_single_shot_cmd(lss);
  add_single_shot_cmd(lss);

  // Arrow 4 -------------------------
  lss.add(TurnToPoint(out_of_way_point));        // [measure]
  lss.add(DriveToPointFastPt(out_of_way_point)); //[measure]

  Vector2D::point_t south_disk_pos1 = {.x = 50.0, .y = 112.0};
  Vector2D::point_t south_disk_pos2 = {.x = 50.0, .y = 103.0};
  Vector2D::point_t south_disk_pos3 = {.x = 50.0, .y = 95.0};

  Vector2D::point_t south_disk_prep_pos1 = {.x = 66, .y = 112};
  Vector2D::point_t south_disk_prep_pos2 = {.x = 66, .y = 106};
  Vector2D::point_t south_disk_prep_pos3 = {.x = 66, .y = 100};

  // disks against L piece
  lss.add({
      // farthest
      StartIntake,
      DriveToPointFastPtRev(south_disk_prep_pos1)->withTimeout(2.0),
      TurnToPoint(south_disk_pos1)->withTimeout(1.5),
      DriveToPointSlowPt(south_disk_pos1)->withTimeout(2.0),

      // middle
      DriveToPointFastPtRev(south_disk_prep_pos2)->withTimeout(2.0),
      TurnToPoint(south_disk_pos2)->withTimeout(1.5),
      DriveToPointSlowPt(south_disk_pos2)->withTimeout(2.0),

      // closest disk
      DriveToPointFastPtRev(south_disk_prep_pos3)->withTimeout(2.0),
      TurnToPoint(south_disk_pos3)->withTimeout(1.5),
      DriveToPointSlowPt(south_disk_pos3)->withTimeout(2.0),
      DriveToPointFastPtRev(south_disk_prep_pos3)->withTimeout(2.0),
  });

  lss.add(TurnToPoint(out_of_way_point));        // [measure]
  lss.add(DriveToPointFastPt(out_of_way_point)); //[measure]

  // Move to endgame pos
  Vector2D::point_t endgame_point = {.x = 122, .y = 122};
  lss.add(TurnToPoint(endgame_point));
  lss.add(DriveToPointFastPt(endgame_point)); //[measure]

  // Endgame
  lss.add(TurnToHeading(45)); //[measure]
  lss.add(new EndgameCommand(endgame_solenoid));
  lss.add(PrintOdom);

  return lss;
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

  position_t start_pos = position_t{.x = 30.5, .y = 10.2, .rot = -90};

  CommandController lss;
  lsa.add(new OdomSetPosition(odometry_sys, start_pos)); // #1
  lsa.add(new FunctionCommand([]()
                              {main_controller.Screen.print("Starting\n"); return true; }));
  lsa.add(SpinFWAt(3000));
  // spin -90 degree roller
  lsa.add(DriveForwardFast(1, fwd)); //[measure]
  lsa.add(new SpinRollerCommandAUTO(drive_sys, roller));
  lsa.add(DriveForwardFast(4, reverse)); // [measure]

  Vector2D::point_t first_shoot_point = {.x = 69, .y = 47};
  lsa.add(TurnToPoint(first_shoot_point));
  lsa.add(DriveToPointFastPt(first_shoot_point));

  lsa.add(TurnToHeading(120.0));
  add_single_shot_cmd(lsa);

  lsa.add(TurnToHeading(120.0));
  add_single_shot_cmd(lsa);

  lsa.add(TurnToHeading(120.0));
  add_single_shot_cmd(lsa);

  Vector2D::point_t disk_pos1 = {.x = 86.8, .y = 48.10};
  Vector2D::point_t disk_pos2 = {.x = 87, .y = 39};
  Vector2D::point_t disk_pos3 = {.x = 87, .y = 25};

  Vector2D::point_t disk_prep_pos2 = {.x = 70, .y = 39};
  Vector2D::point_t disk_prep_pos3 = {.x = 70, .y = 27};

  // disks against L piece
  lsa.add({
      // farthest
      TurnToPoint(disk_pos1),
      StartIntake,
      DriveToPointSlowPt(disk_pos1),

      // middle
      DriveToPointFastPtRev(disk_prep_pos2),
      TurnToPoint(disk_pos2),
      DriveToPointSlowPt(disk_pos2),

      // closest disk
      DriveToPointFastPtRev(disk_prep_pos3),
      TurnToPoint(disk_pos3),
      DriveToPointSlowPt(disk_pos3),
      DriveToPointFastPtRev(disk_prep_pos3),
  });

  Vector2D::point_t second_shoot_point = {.x = 66, .y = 50};

  lsa.add(TurnToPoint(second_shoot_point));

  lsa.add(StopIntake);

  lsa.add(DriveToPointFastPt(second_shoot_point));

  lsa.add(TurnToHeading(120.0));

  add_single_shot_cmd(lsa);
  add_single_shot_cmd(lsa);
  add_single_shot_cmd(lsa);

  return lsa;
}
