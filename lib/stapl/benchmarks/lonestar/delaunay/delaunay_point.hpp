/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARK_LONESTAR_DELAUNAY_POINT_HPP
#define STAPL_BENCHMARK_LONESTAR_DELAUNAY_POINT_HPP

#include <stapl/views/proxy_macros.hpp>
#include <stapl/runtime/serialization/typer_fwd.hpp>
#include <benchmarks/lonestar/delaunay/delaunay_point.hpp>

using std::cout;
using std::endl;
using std::string;


namespace stapl
{


namespace delaunay
{


///////////////////////////////////////////////////////////////////////////////
/// @brief Represents a two-dimensional point containing an x and y coordinate.
///////////////////////////////////////////////////////////////////////////////
struct point2d
{
  double m_x;
  double m_y;

  point2d(void)                      = default;
  point2d(point2d const&)            = default;
  point2d(point2d&&)                 = default;
  point2d& operator=(point2d const&) = default;
  point2d& operator=(point2d&&)      = default;

  point2d(double const& xy)
    : m_x(xy), m_y(xy)
  { }

  point2d(double const& x, double const& y)
    : m_x(x), m_y(y)
  { }

  point2d get_copy() const
  { return *this; }

  double x() const
  { return m_x; }

  double y() const
  { return m_y; }

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Returns a pseudo-angle between two points.
  ///
  /// Used for comparing angles and is less expensive to compute then the true
  /// angle. The calculated pseudo-angle is in the range [-2,2).
  /////////////////////////////////////////////////////////////////////////////
  double get_pseudo_angle(point2d const& p2) const
  {
    double dx = p2.m_x - m_x;
    double dy = p2.m_y - m_y;
    return copysign(1.0 - dx/(std::abs(dx)+std::abs(dy)), -dy);
  }

  static double get_pseudo_angle_range()
  { return 4.0; }

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Returns the angle between two points.
  ///
  /// The calculated angle is in the range [-M_PI,M_PI).
  /////////////////////////////////////////////////////////////////////////////
  double get_angle(point2d const& p2) const
  {
    double dx = p2.m_x - m_x;
    double dy = p2.m_y - m_y;
    double angle = -atan2(dy, dx);
    return angle;
  }

  static double get_angle_range()
  { return 2.0*M_PI; }

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Returns true if the difference between the x and y dimensions of
  /// both points are within 10^(-significance) of each other; false otherwise.
  /////////////////////////////////////////////////////////////////////////////
  bool approximately_equals(point2d const& b, int significance=6) const
  {
    if (*this == b) {
      return true;
    }
    double x = m_x - b.m_x;
    double y = m_y - b.m_y;

    double epsilon = 0.9 * pow(10.0, (double)(-significance));
    return ((x > -epsilon) && (x < epsilon)
         && (y > -epsilon) && (y < epsilon));
  }

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Reverses the angle
  /////////////////////////////////////////////////////////////////////////////
  friend double reverse_angle(double angle)
  {
    angle += M_PI;
    if (angle >= M_PI) {
      angle -= 2.0*M_PI;
    }
    return angle;
  }

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Reverses the pseudo angle
  /////////////////////////////////////////////////////////////////////////////
  friend double reverse_pseudo_angle(double angle)
  {
    return angle - copysign(2.0, angle);
  }

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Returns positive value if the points are counter-clockwise;
  /// returns negative value if the points are clockwise;
  /// returns zero if the points are collinear.
  /////////////////////////////////////////////////////////////////////////////
  friend inline double point_test(point2d const& a, point2d const& b,
                                  point2d const& c)
  {
    return (b.m_x - a.m_x)*(c.m_y - a.m_y) - (b.m_y - a.m_y)*(c.m_x - a.m_x);
  }

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Tests if the three points are oriented in a counter-clockwise
  /// direction.
  /////////////////////////////////////////////////////////////////////////////
  friend inline bool ccw(point2d const& a, point2d const& b, point2d const& c)
  {
    return (point_test(a, b, c) > 0.0);
  }

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Tests if the point d lies in circum-circle of points a, b, and c.
  /////////////////////////////////////////////////////////////////////////////
  friend inline bool in_circle(point2d const& a, point2d const& b,
                               point2d const& c, point2d const& d)
  {
    double a_dx = a.m_x - d.m_x, a_dy = a.m_y - d.m_y;
    double b_dx = b.m_x - d.m_x, b_dy = b.m_y - d.m_y;
    double c_dx = c.m_x - d.m_x, c_dy = c.m_y - d.m_y;

    return (a_dx * a_dx + a_dy * a_dy) * (b_dx * c_dy - c_dx * b_dy) +
           (b_dx * b_dx + b_dy * b_dy) * (c_dx * a_dy - a_dx * c_dy) +
           (c_dx * c_dx + c_dy * c_dy) * (a_dx * b_dy - b_dx * a_dy) > 0;
  }

  friend inline point2d midpoint(point2d const& pt1, point2d const& pt2)
  {
    double cx = (pt1.x() + pt2.x()) / 2.0;
    double cy = (pt1.y() + pt2.y()) / 2.0;
    return point2d(cx, cy);
  }

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Returns true if this point is less than the point b in x dimension;
  /// or they are same in x dimension and this point's y dimension is less than
  /// points b's y dimension.
  /////////////////////////////////////////////////////////////////////////////
  bool operator<(point2d const& b) const
  { return ((m_x < b.m_x) || (m_x == b.m_x && m_y < b.m_y)); }

  bool operator>(point2d const& b) const
  { return ((m_x > b.m_x) || (m_x == b.m_x && m_y > b.m_y)); }

  bool operator<=(point2d const& b) const
  { return !(this->operator>(b)); }

  bool operator>=(point2d const& b) const
  { return !(this->operator<(b)); }

  bool operator==(point2d const& b) const
  { return ((m_x == b.m_x) && (m_y == b.m_y)); }

  bool operator!=(point2d const& b) const
  { return !this->operator==(b); }

  point2d operator-(point2d const& b) const
  { return point2d(m_x - b.m_x, m_y - b.m_y); }

  double operator[](size_t i) const
  { return (&m_x)[i]; }

  double distance_squared(point2d const& pt)
  {
    double tx = m_x - pt.m_x;
    double ty = m_y - pt.m_y;

    return (tx * tx) + (ty * ty);
  }

  double distance(point2d const& pt)
  {
    return sqrt(distance_squared(pt));
  }


  void define_type(stapl::typer& t)
  {
    t.member(m_x);
    t.member(m_y);
  }
};


} // namespace delaunay


using namespace delaunay;


STAPL_PROXY_HEADER(point2d)
{
  STAPL_PROXY_DEFINES(point2d)
  STAPL_PROXY_METHOD_RETURN(x, double)
  STAPL_PROXY_METHOD_RETURN(y, double)
  STAPL_PROXY_METHOD_RETURN(get_copy, point2d)
  STAPL_PROXY_METHOD_RETURN(get_angle, double)
  STAPL_PROXY_METHOD_RETURN(get_pseudo_angle, double)
  STAPL_PROXY_METHOD_RETURN(approximately_equals, bool, point2d)
  STAPL_PROXY_METHOD_RETURN(operator==, bool, point2d)
  STAPL_PROXY_METHOD_RETURN(operator!=, bool, point2d)
  STAPL_PROXY_METHOD_RETURN(operator<, bool, point2d)
  STAPL_PROXY_METHOD_RETURN(operator>, bool, point2d)
  STAPL_PROXY_METHOD_RETURN(operator<=, bool, point2d)
  STAPL_PROXY_METHOD_RETURN(operator>=, bool, point2d)
  STAPL_PROXY_METHOD_RETURN(operator[], double, size_t)
};



} // namespace stapl




#endif /* STAPL_BENCHMARK_LONESTAR_DELAUNAY_POINT_HPP */
