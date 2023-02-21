#include "competition/comp_screen.h"

void page_one(vex::brain::lcd &screen, int x, int y, int width, int height, bool first_run)
{
    screen.setFillColor(vex::black);
    screen.setPenColor(vex::black);
    screen.drawRectangle(x, y, width, height);
    // Left Half - motors

    // Column header
    draw_mot_header(screen, x, 0, width / 2);

    const int line_height = 20;
    int row_num = 1;
    // motors and their information
    for (auto const &name_and_motor : motor_names)
    {
        draw_mot_stats(screen, x, row_num * line_height, width / 2, name_and_motor.first.c_str(), name_and_motor.second);
        row_num++;
    }

    double bat_voltage = Brain.Battery.voltage();
    double bat_percentage = Brain.Battery.capacity();
    draw_battery_stats(screen, x, 180, bat_voltage, bat_percentage);

    // Right Top
    screen.drawImageFromBuffer(splash_little, 40 + width / 2, 0, width / 2, 100);
}

void page_two(vex::brain::lcd &screen, int x, int y, int width, int height, bool first_run)
{
    screen.setPenColor(vex::black);
    screen.setFillColor(vex::black);

    screen.drawRectangle(x, y, width, height);
    position_t pos = odometry_sys.get_position();
    screen.setFont(vex::mono40);
    screen.setPenColor(vex::white);
    screen.setFillColor(vex::white);
    screen.printAt(1000, 100, "Pos: (%.2f, %.2f) : %.2f", pos.x, pos.y, pos.rot);
}
