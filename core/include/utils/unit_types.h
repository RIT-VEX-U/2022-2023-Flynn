#pragma once
#include "../core/include/utils/units.h"
namespace units
{
  /**
   * Data structure representing an X,Y coordinate
   */
  struct point_t {
    units::Length x; ///< the x position in space
    units::Length y; ///< the y position in space

    /**
     * dist calculates the euclidian distance between this point and another
     * point using the pythagorean theorem
     * @param other the point to measure the distance from
     * @return the euclidian distance between this and other
     */
    units::Length dist(const point_t other)
    {
      return sqrt(square(x - other.x) + square(y - other.y));
    }

    /**
     * Vector2D addition operation on points
     * @param other the point to add on to this
     * @return this + other (this.x + other.x, this.y + other.y)
     */
    point_t operator+(const point_t &other)
    {
      point_t p{.x = x + other.x, .y = y + other.y};
      return p;
    }

    /**
     * Vector2D subtraction operation on points
     * @param other the point_t to subtract from this
     * @return this - other (this.x - other.x, this.y - other.y)
     */
    point_t operator-(const point_t &other)
    {
      point_t p{.x = x - other.x, .y = y - other.y};
      return p;
    }
  };

  /**
   *  Describes a single position and rotation
   */
  typedef struct {
    units::Length x;  ///< x position in the world
    units::Length y;  ///< y position in the world
    units::Angle rot; ///< rotation in the world
  } pose_t;

  //  const std::string to_string(point_t pt);
  //  const std::string to_string(pose_t pose);
} // namespace units
