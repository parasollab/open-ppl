/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef TEST_POINT_H
#define TEST_POINT_H

#include <string>
#include <ostream>
#include <limits>
#include <stapl/runtime.hpp>

namespace stapl {

// class point with x and y coordinates
class Point
{
 public:
  double x;
  double y;

  Point() : x(0.0), y(0.0) { }
  Point(const Point& p)  = default;
  Point(const double a, const double b) : x(a), y(b) { }
  Point(const double a) : x(a), y(a) { }
  Point(const int a) : x(a), y(a) { }

  void set_x(double a){ x=a; }
  void set_y(double a){ y=a; }

  void define_type(stapl::typer &t){
    t.member(x);
    t.member(y);
  }

  bool operator<(const Point& p) const {
    return x < p.x;
  }

  bool operator>(const Point& p) const {
    return x > p.x;
  }

  bool operator==(const Point& p) const {
    return (x - p.x < std::numeric_limits<double>::epsilon())
      && (y - p.y < std::numeric_limits<double>::epsilon());
  }

  bool operator!=(const Point& p) const {
    return !(operator==(p));
  }

  // pre-increment
  inline Point& operator++(){
    x += 1.0; y += 1.0;
    return *this;
  }

  // post increment
  inline Point operator++(int) {
    Point tmp = *this;
    x += 1.0; y += 1.0;
    return tmp;
  }

  // pre-decrement
  inline Point& operator--(){
    x -= 1.0; y -= 1.0;
    return *this;
  }

  // post decrement
  inline Point operator--(int) {
    Point tmp = *this;
    x -= 1.0; y -= 1.0;
    return tmp;
  }

  inline Point operator+(const Point& p) const {
    Point c;
    c.x = this->x + p.x;
    c.y = this->y + p.y;
    return c;
  }

  // unary negate
  inline Point operator-(const Point& p) const {
    return Point(-p.x, -p.y);
  }

  // unary negate
  inline void operator-(void) {
    this->x = -this->x;
    this->y = -this->y;
  }

  // unary negate
  inline Point operator-(void) const {
    return Point(-this->x, -this->y);
  }

  void operator+=(const Point& p) {
    this->x += p.x;
    this->y += p.y;
  }
};


inline std::ostream& operator<<(std::ostream& s, Point const& p)
{
  s<<"("<<p.x<<","<<p.y<<")";
  return s;
}


STAPL_DEFINE_IDENTITY_VALUE(plus<Point>,       Point, Point(0, 0))
STAPL_DEFINE_IDENTITY_VALUE(multiplies<Point>, Point, Point(1, 1))

} // namespace stapl

#endif
