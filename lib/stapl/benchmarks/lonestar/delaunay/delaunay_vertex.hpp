/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARK_LONESTAR_DELAUNAY_VERTEX_HPP
#define STAPL_BENCHMARK_LONESTAR_DELAUNAY_VERTEX_HPP

#include <stapl/containers/sequential/graph/graph_util.h>
#include <stapl/runtime/serialization/typer_fwd.hpp>
#include <stapl/views/proxy_macros.hpp>
#include <benchmarks/lonestar/delaunay/delaunay_point.hpp>
#include <benchmarks/lonestar/delaunay/delaunay_edge.hpp>


namespace stapl
{


namespace delaunay
{


///////////////////////////////////////////////////////////////////////////////
/// @brief Holds the 2D-point and used as the default property for vertices in
/// a graph.
///////////////////////////////////////////////////////////////////////////////
class delaunay_vertex
{
public:
  point2d m_point;

  delaunay_vertex(void)                              = default;
  delaunay_vertex(delaunay_vertex const&)            = default;
  delaunay_vertex(delaunay_vertex&&)                 = default;
  delaunay_vertex& operator=(delaunay_vertex const&) = default;
  delaunay_vertex& operator=(delaunay_vertex&&)      = default;

  delaunay_vertex get_copy() const
  { return *this; }

  point2d get_point() const
  { return m_point; }

  double get_angle(point2d point) const
  { return m_point.get_angle(point); }

  void set_point(double x, double y)
  { m_point = point2d(x, y); }

  void set_point(point2d const& point)
  { m_point = point; }

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Constructs a full @ref delaunay_edge from a Vertex, and a graph
  /// Edge containing the reduced_delaunay_edge property.
  /////////////////////////////////////////////////////////////////////////////
  template <typename Vertex, typename Edge>
  delaunay_edge make_edge(Vertex&& v, Edge&& e) const
  {
    return make_edge(v, e.target(), e.property());
  }

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Constructs a full @ref delaunay_edge from a Vertex, target,
  /// and reduced_delaunay_edge property.
  /////////////////////////////////////////////////////////////////////////////
  template <typename Vertex, typename Property>
  delaunay_edge make_edge(Vertex&& v, size_t const& target,
                          Property&& prop) const
  {
    return delaunay_edge(v.descriptor(), target, m_point, prop.get_Dest(),
                         prop.get_angle());
  }

  void define_type(stapl::typer& t)
  { t.member(m_point); }

};


} // namespace delaunay


using namespace delaunay;


STAPL_PROXY_HEADER(delaunay_vertex)
{
public:
  STAPL_PROXY_DEFINES(delaunay_vertex)
  STAPL_PROXY_METHOD_RETURN(get_point, point2d)
  STAPL_PROXY_METHOD_RETURN(get_copy, delaunay_vertex)

  void set_point(point2d const& point)
  {
    typedef void (target_t::*func_t) (point2d const&);
    func_t const& func = &target_t::set_point;
    Accessor::invoke(func, point);
  }

  double get_angle(point2d const& point) const
  {
    return Accessor::const_invoke(&target_t::get_angle, point);
  }

  template <typename Vertex, typename Edge>
  delaunay_edge make_edge(Vertex const& v, Edge const& e) const
  {
    typedef delaunay_edge (target_t::*func_t)
                          (Vertex const&, Edge const&) const;
    func_t const& func = &target_t::make_edge;
    return Accessor::const_invoke(func, v, e);
  }

  template <typename Vertex, typename Property>
  delaunay_edge make_edge(Vertex const& v, size_t const& target,
                          Property const& prop) const
  {
    typedef delaunay_edge (target_t::*func_t)
                          (Vertex const&, size_t const&, Property const&) const;
    func_t const& func = &target_t:: make_edge;
    return Accessor::const_invoke(func, v, target, prop);
  }

};


} // namespace stapl


#endif /* STAPL_BENCHMARK_LONESTAR_DELAUNAY_VERTEX_HPP */
