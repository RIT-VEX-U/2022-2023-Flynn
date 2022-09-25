#include "vex.h"
class drive_interface{
  public:
  virtual void turn_to_heading(double heading_deg, double speed);
  virtual bool drive_to_point(double x, double y, double speed, double correction_speed, vex::directionType direction=vex::directionType::fwd);
  virtual~drive_interface();
  
};