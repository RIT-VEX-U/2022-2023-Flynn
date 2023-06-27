#pragma once

#include <cmath>
#include <string>

#ifndef M_PI
#define M_PI 3.1415926535897932384626433832795
#endif

namespace units
{

  struct Dimensions {
    int length_dim;
    int mass_dim;
    int time_dim;
    int current_dim;
    int temp_dim;
    int angle_dim;

    constexpr bool operator==(const Dimensions &rhs) const
    {
      return length_dim == rhs.length_dim && mass_dim == rhs.mass_dim
             && time_dim == rhs.time_dim && current_dim == rhs.current_dim
             && temp_dim == rhs.temp_dim && angle_dim == rhs.angle_dim;
    }
    constexpr Dimensions operator+(const Dimensions &rhs) const
    {
      return {length_dim + rhs.length_dim, mass_dim + rhs.mass_dim,
              time_dim + rhs.time_dim,     current_dim + rhs.current_dim,
              temp_dim + rhs.temp_dim,     angle_dim + rhs.angle_dim};
    }
    constexpr Dimensions operator-(const Dimensions &rhs) const
    {
      return {length_dim - rhs.length_dim, mass_dim - rhs.mass_dim,
              time_dim - rhs.time_dim,     current_dim - rhs.current_dim,
              temp_dim - rhs.temp_dim,     angle_dim - rhs.angle_dim};
    }

    constexpr Dimensions operator*(int rhs) const
    {
      return {length_dim * rhs,  mass_dim * rhs, time_dim * rhs,
              current_dim * rhs, temp_dim * rhs, angle_dim * rhs};
    }

    constexpr bool isDivisbleBy(const int denom) const
    {
      return (length_dim % denom == 0) && (mass_dim % denom == 0)
             && (time_dim % denom == 0) && (current_dim % denom == 0)
             && (temp_dim % denom == 0) && (angle_dim % denom == 0);
    }

    constexpr Dimensions operator/(const int rhs) const
    {

      return {length_dim / rhs,  mass_dim / rhs, time_dim / rhs,
              current_dim / rhs, temp_dim / rhs, angle_dim / rhs};
    }
    constexpr bool isSquareRootable() const { return isDivisbleBy(2); }
  };
  inline std::string to_string(const Dimensions &d)
  {
    std::string s = "";
#define ADD_IF_NONZERO(which, letter)                                          \
  if (which == 1) {                                                            \
    s = s + letter;                                                            \
  } else if (which != 0) {                                                     \
    s = s + letter + "^" + std::to_string(which);                              \
  }
    ADD_IF_NONZERO(d.length_dim, "m")
    ADD_IF_NONZERO(d.mass_dim, "kg")
    ADD_IF_NONZERO(d.time_dim, "s")
    ADD_IF_NONZERO(d.current_dim, "A")
    ADD_IF_NONZERO(d.temp_dim, "C")
    ADD_IF_NONZERO(d.angle_dim, "rad")
#undef ADD_IF_NONZERO
    return s;
  }

  template <Dimensions dim>
  class Quantity
  {

  private:
    double value;

  public:
    static constexpr Dimensions Dims = dim;

    constexpr Quantity() : value(0.0) {}
    // Explicit if unitted
    // not explicit if Number
    explicit(dim != Dimensions{0, 0, 0, 0, 0, 0}) constexpr Quantity(double val)
        : value(val)
    {
    }

    /// convert to a double in the units specified
    constexpr double Convert(const Quantity &rhs) const
    {
      return value / rhs.value;
    }
    /// Get raw stored value
    constexpr double getValue() const { return value; }

    constexpr operator double()
    {
      static_assert(dim == Dimensions{0, 0, 0, 0, 0, 0},
                    "Implicit conversion to double only allowed for Number");
      return value;
    }

    constexpr Quantity operator-() { return Quantity(-1.0 * this->getValue()); }
  };

  template <Dimensions dim>
  inline std::string to_string(const Quantity<dim> q)
  {
    std::string s = std::to_string(q.getValue() + to_string(dim));
    return s;
  }

  constexpr Dimensions number_dimensions = {0, 0, 0, 0, 0, 0};

  constexpr Dimensions length_dimensions = {1, 0, 0, 0, 0, 0};
  constexpr Dimensions area_dimensions = {2, 0, 0, 0, 0, 0};
  constexpr Dimensions volume_dimensions = {3, 0, 0, 0, 0, 0};

  constexpr Dimensions mass_dimensions = {0, 1, 0, 0, 0, 0};

  constexpr Dimensions time_dimensions = {0, 0, 1, 0, 0, 0};
  constexpr Dimensions frequency_dimensions = {0, 0, -1, 0, 0, 0};

  constexpr Dimensions current_dimensions = {0, 0, 0, 1, 0, 0};
  constexpr Dimensions voltage_dimensions = {2, 1, -3, 1, 0, 0};

  constexpr Dimensions speed_dimensions = length_dimensions - time_dimensions;
  constexpr Dimensions accel_dimensions
      = length_dimensions - time_dimensions - time_dimensions;

  constexpr Dimensions angle_dimensions = {0, 0, 0, 0, 0, 1};
  constexpr Dimensions angular_speed_dimensions
      = angle_dimensions - time_dimensions;

  typedef Quantity<number_dimensions> Number;

  typedef Quantity<length_dimensions> Length;
  typedef Quantity<area_dimensions> Area;
  typedef Quantity<volume_dimensions> Volume;

  typedef Quantity<mass_dimensions> Mass;

  typedef Quantity<time_dimensions> Time;
  typedef Quantity<frequency_dimensions> Frequency;

  typedef Quantity<speed_dimensions> Speed;
  typedef Quantity<accel_dimensions> Acceleration;

  typedef Quantity<current_dimensions> Current;
  typedef Quantity<voltage_dimensions> Voltage;

  typedef Quantity<angle_dimensions> Angle;
  typedef Quantity<angular_speed_dimensions> AngularSpeed;

  template <Dimensions dim>
  constexpr Quantity<dim> operator+(const Quantity<dim> &lhs,
                                    const Quantity<dim> &rhs)
  {
    return Quantity<dim>(lhs.getValue() + rhs.getValue());
  }
  template <Dimensions dim>
  constexpr Quantity<dim> operator-(const Quantity<dim> &lhs,
                                    const Quantity<dim> &rhs)
  {
    return Quantity<dim>(lhs.getValue() - rhs.getValue());
  }
  template <Dimensions dim1, Dimensions dim2>
  constexpr Quantity<dim1 + dim2> operator*(const Quantity<dim1> &lhs,
                                            const Quantity<dim2> &rhs)
  {
    return Quantity<dim1 + dim2>(lhs.getValue() * rhs.getValue());
  }
  template <Dimensions dim1, Dimensions dim2>
  constexpr Quantity<dim1 - dim2> operator/(const Quantity<dim1> &lhs,
                                            const Quantity<dim2> &rhs)
  {
    return Quantity<dim1 - dim2>(lhs.getValue() / rhs.getValue());
  }

  template <Dimensions dim>
  constexpr Quantity<dim> operator+=(Quantity<dim> &lhs,
                                     const Quantity<dim> &rhs)
  {
    lhs = lhs + rhs;
    return lhs;
  }

  template <Dimensions dim>
  constexpr Quantity<dim> operator-=(Quantity<dim> &lhs,
                                     const Quantity<dim> &rhs)
  {
    lhs = lhs - rhs;
    return lhs;
  }

  template <Dimensions dim>
  constexpr Quantity<dim> operator*(const Quantity<dim> &lhs, const double rhs)
  {
    return Quantity<dim>(lhs.getValue() * rhs);
  }
  template <Dimensions dim>
  constexpr Quantity<dim> operator*(const double lhs, const Quantity<dim> &rhs)
  {
    return Quantity<dim>(rhs.getValue() * lhs);
  }
  template <Dimensions dim>
  constexpr Quantity<dim> operator/(const Quantity<dim> &lhs, const double rhs)
  {
    return Quantity<dim>(lhs.getValue() / rhs);
  }
  template <Dimensions dim>
  constexpr Quantity<dim> operator/(const double lhs, const Quantity<dim> &rhs)
  {
    return Quantity<dim>(lhs / rhs.getValue());
  }

  // Comparison
  template <Dimensions dim>
  constexpr bool operator==(const Quantity<dim> &lhs, const Quantity<dim> &rhs)
  {
    return (lhs.getValue() == rhs.getValue());
  }
  template <Dimensions dim>
  constexpr bool operator!=(const Quantity<dim> &lhs, const Quantity<dim> &rhs)
  {
    return (lhs.getValue() != rhs.getValue());
  }
  template <Dimensions dim>
  constexpr bool operator<=(const Quantity<dim> &lhs, const Quantity<dim> &rhs)
  {
    return (lhs.getValue() <= rhs.getValue());
  }
  template <Dimensions dim>
  constexpr bool operator>=(const Quantity<dim> &lhs, const Quantity<dim> &rhs)
  {
    return (lhs.getValue() >= rhs.getValue());
  }
  template <Dimensions dim>
  constexpr bool operator<(const Quantity<dim> &lhs, const Quantity<dim> &rhs)
  {
    return (lhs.getValue() < rhs.getValue());
  }
  template <Dimensions dim>
  constexpr bool operator>(const Quantity<dim> &lhs, const Quantity<dim> &rhs)
  {
    return (lhs.getValue() > rhs.getValue());
  }

  // Mass
  constexpr Mass kg(1.0);
  constexpr Mass gram = 0.001 * kg;
  constexpr Mass ounce = 0.028349523125 * kg;
  constexpr Mass pound = ounce * 16.0;

  // Length

  constexpr Length meter(1.0);
  constexpr Length centimeter = meter / 100.0;
  constexpr Length millimeter = meter / 1000.0;
  constexpr Length kilometer = meter * 1000.0;
  constexpr Length inch = 2.54 * centimeter;
  constexpr Length foot = 12 * inch;
  constexpr Length yard = 3 * foot;
  constexpr Length mile = 5280 * foot;

  // Area
  constexpr Area kilometer2 = kilometer * kilometer;
  constexpr Area meter2 = meter * meter;
  constexpr Area centimeter2 = centimeter * centimeter;
  constexpr Area millimeter2 = millimeter * millimeter;
  constexpr Area inch2 = inch * inch;
  constexpr Area foot2 = foot * foot;
  constexpr Area yard2 = yard * yard;

  // Time

  constexpr Time second(1.0);
  constexpr Time millisecond = second / 1000.0;
  constexpr Time minute = second * 60.0;
  constexpr Time hour = minute * 60.0;

  // Frequency
  constexpr Frequency hz(1.0);

  // Velocity
  constexpr Speed meter_per_second = (meter / second);
  constexpr Speed inch_per_second = inch / second;
  constexpr Speed foot_per_second = foot / second;

  // Acceleration
  constexpr Acceleration meter_per_second2 = meter / (second * second);
  constexpr Acceleration inch_per_second2 = inch / (second * second);

  // Current
  constexpr Current amp(1.0);

  // Voltage
  constexpr Voltage volt
      = kg * meter * meter / (second * second * second) * amp;
  constexpr Voltage millivolt = volt / 1000.0;

  // Angular
  constexpr Angle radian(1.0);
  constexpr Angle degree = (2.0 * M_PI / 360.0) * radian;
  constexpr Angle revolution = (2.0 * M_PI) * radian;

  // Angular Speed
  constexpr AngularSpeed radian_per_sec = radian / second;
  constexpr AngularSpeed degree_per_sec = degree / second;
  constexpr AngularSpeed revolution_per_sec = revolution / second;
  constexpr AngularSpeed revolution_per_min = revolution / minute;

} // namespace units

/// @brief Namespace that contains literals
/// if you want things like 1_in, 2_s, or 45_deg, use this namespace
namespace unit_literals
{
  // Length Literals
  constexpr units::Length operator"" _mm(long double x)
  {
    return static_cast<double>(x) * units::millimeter;
  }
  constexpr units::Length operator"" _mm(unsigned long long int x)
  {
    return static_cast<double>(x) * units::millimeter;
  }

  constexpr units::Length operator"" _cm(long double x)
  {
    return static_cast<double>(x) * units::centimeter;
  }
  constexpr units::Length operator"" _cm(unsigned long long int x)
  {
    return static_cast<double>(x) * units::centimeter;
  }

  constexpr units::Length operator"" _m(long double x)
  {
    return static_cast<double>(x) * units::meter;
  }
  constexpr units::Length operator"" _m(unsigned long long int x)
  {
    return static_cast<double>(x) * units::meter;
  }

  constexpr units::Length operator"" _km(long double x)
  {
    return static_cast<double>(x) * units::kilometer;
  }
  constexpr units::Length operator"" _km(unsigned long long int x)
  {
    return static_cast<double>(x) * units::meter;
  }

  constexpr units::Length operator"" _in(long double x)
  {
    return static_cast<double>(x) * units::inch;
  }
  constexpr units::Length operator"" _in(unsigned long long int x)
  {
    return static_cast<double>(x) * units::inch;
  }

  constexpr units::Length operator"" _ft(long double x)
  {
    return static_cast<double>(x) * units::foot;
  }
  constexpr units::Length operator"" _ft(unsigned long long int x)
  {
    return static_cast<double>(x) * units::foot;
  }

  // Speed Literals
  constexpr units::Speed operator"" _mps(long double x)
  {
    return static_cast<double>(x) * units::meter / units::second;
  }
  constexpr units::Speed operator"" _mps(unsigned long long int x)
  {
    return static_cast<double>(x) * units::meter / units::second;
  }

  constexpr units::Speed operator"" _inps(long double x)
  {
    return static_cast<double>(x) * units::inch / units::second;
  }
  constexpr units::Speed operator"" _inps(unsigned long long int x)
  {
    return static_cast<double>(x) * units::inch / units::second;
  }

  // Time Literal
  constexpr units::Time operator"" _s(long double x)
  {
    return static_cast<double>(x) * units::second;
  }
  constexpr units::Time operator"" _s(unsigned long long int x)
  {
    return static_cast<double>(x) * units::second;
  }

  constexpr units::Time operator"" _ms(long double x)
  {
    return static_cast<double>(x) * units::millisecond;
  }
  constexpr units::Time operator"" _ms(unsigned long long int x)
  {
    return static_cast<double>(x) * units::millisecond;
  }

  constexpr units::Time operator"" _min(long double x)
  {
    return static_cast<double>(x) * units::minute;
  }
  constexpr units::Time operator"" _min(unsigned long long int x)
  {
    return static_cast<double>(x) * units::minute;
  }

  // Angle Literal
  constexpr units::Angle operator"" _rad(long double x)
  {
    return static_cast<double>(x) * units::radian;
  }
  constexpr units::Angle operator"" _rad(unsigned long long int x)
  {
    return static_cast<double>(x) * units::radian;
  }

  constexpr units::Angle operator"" _deg(long double x)
  {
    return static_cast<double>(x) * units::degree;
  }
  constexpr units::Angle operator"" _deg(unsigned long long int x)
  {
    return static_cast<double>(x) * units::degree;
  }

  constexpr units::Angle operator"" _rev(long double x)
  {
    return static_cast<double>(x) * units::revolution;
  }
  constexpr units::Angle operator"" _rev(unsigned long long int x)
  {
    return static_cast<double>(x) * units::revolution;
  }

  constexpr units::AngularSpeed operator"" _rpm(long double x)
  {
    return static_cast<double>(x) * units::revolution_per_min;
  }
  constexpr units::AngularSpeed operator"" _rpm(unsigned long long int x)
  {
    return static_cast<double>(x) * units::revolution_per_min;
  }

  constexpr units::AngularSpeed operator"" _rps(long double x)
  {
    return static_cast<double>(x) * units::revolution_per_sec;
  }
  constexpr units::AngularSpeed operator"" _rps(unsigned long long int x)
  {
    return static_cast<double>(x) * units::revolution_per_sec;
  }

  // Voltage Literal
  constexpr units::Voltage operator"" _v(long double x)
  {
    return static_cast<double>(x) * units::volt;
  }
  constexpr units::Voltage operator"" _v(unsigned long long int x)
  {
    return static_cast<double>(x) * units::volt;
  }

  constexpr units::Voltage operator"" _mv(long double x)
  {
    return static_cast<double>(x) * units::millivolt;
  }
  constexpr units::Voltage operator"" _mv(unsigned long long int x)
  {
    return static_cast<double>(x) * units::millivolt;
  }

  constexpr long double operator"" _pi(long double x)
  {
    return static_cast<double>(x) * 3.1415926535897932384626433832795;
  }
  constexpr long double operator"" _pi(unsigned long long int x)
  {
    return static_cast<double>(x) * 3.1415926535897932384626433832795;
  }
}; // namespace unit_literals

// Global overloads
// Typesafe trigonometric operations
inline double sin(const units::Angle &num) { return std::sin(num.getValue()); }
inline double cos(const units::Angle &num) { return std::cos(num.getValue()); }
inline double tan(const units::Angle &num) { return std::tan(num.getValue()); }

// typesafe sign
template <units::Dimensions dim>
constexpr units::Number sign(units::Quantity<dim> op)
{
  if (op.getValue() >= 0) {
    return 1;
  } else {
    return -1;
  }
}

// typesafe abs
template <units::Dimensions dim>
constexpr units::Quantity<dim> abs(units::Quantity<dim> op)
{
  if (op.getValue() >= 0) {
    return op;
  } else {
    return -op;
  }
}

// typesafe mod
template <units::Dimensions dim>
constexpr units::Quantity<dim> mod(units::Quantity<dim> op,
                                   units::Quantity<dim> wrap_at)
{
  return units::Quantity<dim>(fmod(op.getValue(), wrap_at.getValue()));
}


template <units::Dimensions dims>
constexpr units::Quantity<dims / 2> sqrt(units::Quantity<dims> op)
{
  static_assert(dims.isDivisbleBy(2) == true,
                "Can't take square root of this. I would get fractional "
                "dimension which I am scared of");
  return (units::Quantity<dims / 2>)(std::sqrt(op.getValue()));
}

// typesafe clamp
template <units::Dimensions dims>
constexpr units::Quantity<dims> clamp(units::Quantity<dims> v,
                                      units::Quantity<dims> lo,
                                      units::Quantity<dims> hi)
{
  if (v < lo) {
    return lo;
  } else if (v > hi) {
    return hi;
  }
  return v;
}

// other global functions unique to units lib
namespace units
{
  inline Angle smallest_angle(Angle a1, Angle a2)
  {
    Angle retval;
    // get the difference between 0 and 2PI
    retval = mod(a1 - a2, Angle(2 * M_PI));
    if (retval < Angle(0))
      retval += Angle(2 * M_PI);

    // Get the closest angle, now between -180 (turn left) and +180 (turn right)
    if (retval > Angle(2 * M_PI))
      retval -= Angle(2 * M_PI);

    return retval;
  }

  template <units::Dimensions dims>
  inline constexpr units::Quantity<dims * 2> square(units::Quantity<dims> op)
  {
    return (units::Quantity<dims * 2>)(op * op);
  }

  inline constexpr units::Length arc_length(units::Angle ang,
                                            units::Length radius)
  {
    units::Length s = ang.Convert(units::radian) * radius;
    return s;
  }

} // namespace units

using namespace unit_literals;
