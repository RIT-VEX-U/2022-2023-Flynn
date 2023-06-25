#include "core/utils/geometry.h"

const std::string to_string(pose_t pose)
{
  return "(" + std::to_string(pose.x) + "," + to_string(pose.y) + ") " +
         to_string(pose.rot);
}
const std::string to_string(point_t pt)
{
  return "(" + to_string(pt.x) + "," + to_string(pt.y) + ")";
}
