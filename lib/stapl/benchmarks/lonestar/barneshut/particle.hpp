/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARK_LONESTAR_BH_PARTICLE
#define STAPL_BENCHMARK_LONESTAR_BH_PARTICLE

#include <stapl/runtime/runtime.hpp>
#include <stapl/views/proxy_macros.hpp>

#include "point.hpp"

//////////////////////////////////////////////////////////////////////
/// @brief A particle representing a body that has mass, a point in space,
///        velocity and acceleration.
//////////////////////////////////////////////////////////////////////
struct particle
{
  point m_coord, m_velocity, m_acceleration;
  double m_mass;

  particle(void)
   : m_coord(), m_velocity(), m_acceleration(), m_mass(0.0)
  { }

  particle(point const& c, point const& v, point const& a, double m)
   : m_coord(c), m_velocity(v), m_acceleration(a), m_mass(m)
  { }

  particle(point const& c, double m)
   : m_coord(c), m_velocity(), m_acceleration(), m_mass(m)
  { }

  double mass(void) const
  {
    return m_mass;
  }

  point acceleration(void) const
  {
    return m_acceleration;
  }

  point velocity(void) const
  {
    return m_velocity;
  }

  point coord(void) const
  {
    return m_coord;
  }

  void coord(point const& c)
  {
    m_coord = c;
  }

  void acceleration(point const& acc)
  {
    m_acceleration = acc;
  }

  void velocity(point const& vel)
  {
    m_velocity = vel;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_coord);
    t.member(m_velocity);
    t.member(m_acceleration);
    t.member(m_mass);
  }
};


bool operator==(particle const& a, particle const& b)
{
  return a.coord() == b.coord() && std::abs(a.mass() - b.mass()) < FLT_EPSILON;
}


namespace stapl {

STAPL_PROXY_HEADER(particle)
{
  STAPL_PROXY_DEFINES(particle)
  STAPL_PROXY_METHOD_RETURN(mass, double)
  STAPL_PROXY_METHOD_RETURN(acceleration, point)
  STAPL_PROXY_METHOD_RETURN(velocity, point)
  STAPL_PROXY_METHOD_RETURN(coord, point)
  STAPL_PROXY_METHOD(acceleration, point)
  STAPL_PROXY_METHOD(velocity, point)
  STAPL_PROXY_METHOD(coord, point)
};

} //namespace stapl

#endif
