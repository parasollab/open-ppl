/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARK_LONESTAR_DELAUNAY_EDGE_HPP
#define STAPL_BENCHMARK_LONESTAR_DELAUNAY_EDGE_HPP

#include <stapl/containers/sequential/graph/graph_util.h>
#include <stapl/runtime/serialization/typer_fwd.hpp>
#include <stapl/views/proxy_macros.hpp>
#include <benchmarks/lonestar/delaunay/delaunay_point.hpp>
#include <benchmarks/lonestar/delaunay/delaunay_utilities.hpp>


namespace stapl
{


template <typename VD>
edge_descriptor_impl<VD> reverse(edge_descriptor_impl<VD> const& t);


namespace delaunay
{

double reverse_angle(double angle);

struct delaunay_edge;

///////////////////////////////////////////////////////////////////////////////
/// @brief Represents a edge and used as the default property for graph edges.
/// Only contains information needed to build the full edge. Because both the id
/// and point of the source vertex are already stored on the source vertex,
/// using this class as the property reduces memory usage.
///////////////////////////////////////////////////////////////////////////////
template <typename FullEdge>
struct delaunay_reduced_edge
{
  typedef edge_descriptor_impl<size_t> edge_descriptor;
  point2d m_target_point;
  double  m_angle;

  delaunay_reduced_edge(void)                                    = default;
  delaunay_reduced_edge(delaunay_reduced_edge const&)            = default;
  delaunay_reduced_edge(delaunay_reduced_edge&&)                 = default;
  delaunay_reduced_edge& operator=(delaunay_reduced_edge const&) = default;
  delaunay_reduced_edge& operator=(delaunay_reduced_edge&&)      = default;

  delaunay_reduced_edge(edge_descriptor const& ed, point2d const& spt,
                        point2d const& tpt)
    : m_target_point(tpt), m_angle(spt.get_angle(tpt))
  { }

  delaunay_reduced_edge(edge_descriptor const& ed, point2d const& spt,
                point2d const& tpt, double const& angle)
    : m_target_point(tpt), m_angle(spt.get_angle(tpt))
  { }

  delaunay_reduced_edge(size_t source, size_t target, point2d const& spt,
                        point2d const& tpt)
    : m_target_point(tpt), m_angle(spt.get_angle(tpt))
  { }

  delaunay_reduced_edge(size_t source, size_t target, point2d const& spt,
                point2d const& tpt, double const& angle)
    : m_target_point(tpt), m_angle(angle)
  { }

  delaunay_reduced_edge(point2d const& tpt, double const& angle)
    : m_target_point(tpt), m_angle(angle)
  { }

  delaunay_reduced_edge(point2d const& spt, point2d const& tpt)
    : m_target_point(tpt), m_angle(spt.get_angle(tpt))
  { }

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Makes a copy of this edge.
  /////////////////////////////////////////////////////////////////////////////
  delaunay_reduced_edge get_copy() const
  { return *this; }

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Returns the target vertex's point.
  /////////////////////////////////////////////////////////////////////////////
  point2d get_target_point() const
  { return m_target_point; }

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Returns the target (destination) vertex's point.
  /////////////////////////////////////////////////////////////////////////////
  point2d get_Dest() const
  { return m_target_point; }

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Returns the pesudo-angle between source and vertex.
  /////////////////////////////////////////////////////////////////////////////
  double get_angle() const
  { return m_angle; }

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Constructs a full delaunay edge.
  /////////////////////////////////////////////////////////////////////////////
  template <typename Vertex>
  FullEdge make_edge(Vertex&& v, size_t target) const
  {
    return FullEdge(v.descriptor(), target,
                    v.property().get_point(), m_target_point, m_angle);
  }


  /////////////////////////////////////////////////////////////////////////////
  /// @brief Constructs a full delaunay edge.
  /////////////////////////////////////////////////////////////////////////////
  template <typename Vertex, typename Edge>
  FullEdge make_edge(Vertex&& v, Edge&& e) const
  {
    return FullEdge(v.descriptor(), e.target(),
                    v.property().get_point(), m_target_point, m_angle);
  }


  /////////////////////////////////////////////////////////////////////////////
  /// @brief Constructs a full delaunay edge.
  /////////////////////////////////////////////////////////////////////////////
  FullEdge make_edge(size_t source, size_t target, point2d const& spt) const
  {
    return FullEdge(source, target, spt, m_target_point, m_angle);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_target_point);
    t.member(m_angle);
  }
};


///////////////////////////////////////////////////////////////////////////////
/// @brief Represents a full edge containing all the information of an edge.
/// This structure contains both the source and target id's, both the source and
/// target points, and the angle between those points. It also provides the
/// functionality needed to find ccw and cw edges.
///////////////////////////////////////////////////////////////////////////////
struct delaunay_edge
  : public delaunay_reduced_edge<delaunay_edge>
{
  typedef delaunay_reduced_edge<delaunay_edge>    base_type;
  typedef delaunay_edge                           this_type;
  typedef edge_descriptor_impl<size_t>            edge_descriptor;
  typedef std::pair<delaunay_edge, delaunay_edge> edge_pair;
  typedef ccw_edge_wf<this_type>                  ccw_edge_wf_t;
  typedef cw_edge_wf<this_type>                   cw_edge_wf_t;

  using base_type::get_Dest;
  using base_type::get_angle;
  using base_type::m_target_point;
  using base_type::m_angle;

  size_t m_source;
  size_t m_target;
  point2d m_source_point;

  delaunay_edge(void)                            = default;
  delaunay_edge(delaunay_edge const&)            = default;
  delaunay_edge(delaunay_edge&&)                 = default;
  delaunay_edge& operator=(delaunay_edge const&) = default;
  delaunay_edge& operator=(delaunay_edge&&)      = default;

  delaunay_edge(edge_descriptor const& ed, point2d const& spt,
                point2d const& tpt)
    : base_type(spt, tpt), m_source(ed.source()), m_target(ed.target()),
      m_source_point(spt)
  { }

  delaunay_edge(edge_descriptor const& ed, point2d const& spt,
                point2d const& tpt, double const& angle)
    : base_type(tpt, angle), m_source(ed.source()), m_target(ed.target()),
      m_source_point(spt)
  { }

  delaunay_edge(size_t source, size_t target, point2d const& spt,
                point2d const& tpt)
    : base_type(spt, tpt), m_source(source), m_target(target),
      m_source_point(spt)
  { }

  delaunay_edge(size_t source, size_t target, point2d const& spt,
                point2d const& tpt, double const& angle)
    : base_type(tpt, angle), m_source(source), m_target(target),
      m_source_point(spt)
  { }

  template <typename Vertex, typename Edge>
  delaunay_edge(Vertex&& v, Edge&& edge)
    : base_type(edge.property().get_Dest(), edge.property().get_angle()),
      m_source(v.descriptor()), m_target(edge.target()),
      m_source_point(v.property().get_point())
  { }

  template <typename Vertex, typename Property>
  delaunay_edge(Vertex&& v, size_t const& target, Property&& prop)
    : base_type(prop.get_Dest(), prop.get_angle()),
      m_source(v.descriptor()), m_target(target),
      m_source_point(v.property().get_point())
  { }

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Makes a copy of this edge.
  /////////////////////////////////////////////////////////////////////////////
  delaunay_edge get_copy() const
  { return *this; }

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Returns true if the source and target match (or if the reversed
  /// source and target) match that of the other edge.
  /////////////////////////////////////////////////////////////////////////////
  bool operator==(delaunay_edge const& edge) const
  {
    return (m_source == edge.m_source && m_target == edge.m_target) ||
           (m_source == edge.m_target && m_target == edge.m_source);
  }

  bool operator!=(delaunay_edge const& edge) const
  { return !operator==(edge); }

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Returns the source vertex's point.
  /////////////////////////////////////////////////////////////////////////////
  point2d get_source_point() const
  { return m_source_point; }

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Returns the source (origin) vertex's point.
  /////////////////////////////////////////////////////////////////////////////
  point2d get_Org() const
  { return m_source_point; }

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Calculates angle between source and target points.
  /////////////////////////////////////////////////////////////////////////////
  double calc_angle() const
  { return m_source_point.get_angle(m_target_point); }

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Constructs a edge_descriptor from source and target id's.
  /////////////////////////////////////////////////////////////////////////////
  edge_descriptor get_ed() const
  { return edge_descriptor(m_source, m_target); }

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Constructs a reversed edge_descriptor from source and target id's.
  /////////////////////////////////////////////////////////////////////////////
  edge_descriptor get_reverse_ed() const
  { return edge_descriptor(m_target, m_source); }

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Returns the adjacent edge that lays counter-clockwise of the
  ///        current edge of the source vertex.
  /////////////////////////////////////////////////////////////////////////////
  template <typename GraphView>
  delaunay_edge get_Onext(GraphView& gv) const
  { return gv.apply_get(m_source, ccw_edge_wf_t(*this)); }

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Returns the adjacent edge that lays clockwise of the current edge
  ///        of the source vertex.
  /////////////////////////////////////////////////////////////////////////////
  template <typename GraphView>
  delaunay_edge get_Oprev(GraphView const& gv) const
  { return gv.apply_get(m_source, cw_edge_wf_t(*this)); }

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Returns the adjacent edge that lays counter-clockwise of the
  ///        current edge going into the target vertex.
  /////////////////////////////////////////////////////////////////////////////
  template <typename GraphView>
  delaunay_edge get_Dnext(GraphView const& gv) const
  { return gv.apply_get(m_target, cw_edge_wf_t(get_Sym())).reverse(); }

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Returns the adjacent edge that lays clockwise of the current edge
  ///        going into the target vertex.
  /////////////////////////////////////////////////////////////////////////////
  template <typename GraphView>
  delaunay_edge get_Dprev(GraphView const& gv) const
  { return gv.apply_get(m_target, ccw_edge_wf_t(get_Sym())).reverse(); }

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Return the ccw edge around the left face following the current
  ///        edge.
  /////////////////////////////////////////////////////////////////////////////
  template <typename GraphView>
  delaunay_edge get_Lnext(GraphView const& gv) const
  { return get_Sym().get_Oprev(gv); }

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Return the ccw edge around the left face before the current
  ///        edge.
  /////////////////////////////////////////////////////////////////////////////
  template <typename GraphView>
  delaunay_edge get_Lprev(GraphView const& gv) const
  { return get_Onext(gv).reverse(); }

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Return the edge around the right face ccw following the current
  ///        edge.
  /////////////////////////////////////////////////////////////////////////////
  template <typename GraphView>
  delaunay_edge get_Rnext(GraphView const& gv) const
  { return get_Sym().get_Oprev(gv); }

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Return the edge around the right face ccw before the current edge.
  /////////////////////////////////////////////////////////////////////////////
  template <typename GraphView>
  delaunay_edge get_Rprev(GraphView const& gv) const
  { return get_Sym().get_Onext(gv); }

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Returns the symmetrical (reversed) edge.
  /////////////////////////////////////////////////////////////////////////////
  template <typename GraphView>
  delaunay_edge get_Sym(GraphView const& gv) const
  { return get_Sym(); }

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Returns the symmetrical (reversed) edge.
  /////////////////////////////////////////////////////////////////////////////
  delaunay_edge get_Sym() const
  {
    auto reversed_angle = reverse_angle(m_angle);
    return delaunay_edge(m_target, m_source, m_target_point, m_source_point,
                         reversed_angle);
  }

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Reverses this edge in-place.
  /////////////////////////////////////////////////////////////////////////////
  delaunay_edge& reverse()
  {
    m_angle = reverse_angle(m_angle);
    std::swap(m_source, m_target);
    std::swap(m_source_point, m_target_point);
    return *this;
  }

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Returns true if the point x is to the left of this edge.
  /////////////////////////////////////////////////////////////////////////////
  bool left_of(point2d const& x) const
  { return ccw(x, m_source_point, m_target_point); }

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Returns true if the point x is to the right of this edge.
  /////////////////////////////////////////////////////////////////////////////
  bool right_of(point2d const& x) const
  { return ccw(x, m_target_point, m_source_point); }

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Returns true if the edge e is valid in reference to this edge.
  /////////////////////////////////////////////////////////////////////////////
  bool valid(delaunay_edge const& e) const
  { return ccw(e.m_target_point, m_target_point, m_source_point); }

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Returns true if this edge is vertical.
  /////////////////////////////////////////////////////////////////////////////
  bool is_vertical() const
  { return m_source_point.m_x == m_source_point.m_x; }

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Returns true if this edge and edge e are collinear.
  /////////////////////////////////////////////////////////////////////////////
  bool is_collinear(delaunay_edge const& e) const
  {
    if (*this == e) {
      return true;
    }
    return point_test(m_source_point, m_target_point, e.m_source_point) == 0 &&
           point_test(m_target_point, e.m_source_point, e.m_target_point) == 0;
  }

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Constructs a reversed edge from the passed in edge.
  /////////////////////////////////////////////////////////////////////////////
  friend delaunay_edge reverse(delaunay_edge const& edge)
  { return edge.get_Sym(); }

  bool operator<(delaunay_edge const& b) const
  { return get_angle() < b.get_angle(); }

  bool operator>(delaunay_edge const& b) const
  { return get_angle() > b.get_angle(); }

  bool operator<=(delaunay_edge const& b) const
  { return get_angle() <= b.get_angle(); }

  bool operator>=(delaunay_edge const& b) const
  { return get_angle() >= b.get_angle(); }

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Returns the id of the source vertex.
  /////////////////////////////////////////////////////////////////////////////
  size_t source() const
  { return m_source; }

  /////////////////////////////////////////////////////////////////////////////
  /// @brief Returns the id of the target vertex.
  /////////////////////////////////////////////////////////////////////////////
  size_t target() const
  { return m_target; }

  void define_type(stapl::typer& t)
  {
    base_type::define_type(t);
    t.member(m_source);
    t.member(m_target);
    t.member(m_source_point);
  }
};

} // namespace delaunay


using namespace delaunay;

STAPL_PROXY_HEADER_TEMPLATE(delaunay_reduced_edge, FullEdge)
{
  typedef edge_descriptor_impl<size_t> edge_descriptor;
  STAPL_PROXY_DEFINES(delaunay_reduced_edge<FullEdge>)
  STAPL_PROXY_METHOD_RETURN(get_Dest, point2d)
  STAPL_PROXY_METHOD_RETURN(get_angle, double)
  STAPL_PROXY_METHOD_RETURN(get_copy, delaunay_reduced_edge<FullEdge>)

  template <typename Vertex>
  FullEdge make_edge(Vertex&& v) const
  {
    typedef FullEdge (target_t::*func_t) (Vertex const&) const;
    func_t const& func = &target_t::template make_edge<Vertex const&>;
    return Accessor::const_invoke(func, v);
  }

  template <typename Vertex, typename Edge>
  FullEdge make_edge(Vertex&& v, Edge&& e) const
  {
    typedef FullEdge (target_t::*func_t) (Vertex, Edge) const;
    func_t const& func = &target_t::template make_edge<Vertex, Edge>;
    return Accessor::const_invoke(func, v, e);
  }

  FullEdge make_edge(size_t source, size_t target, point2d const& spt) const
  {
    typedef FullEdge (target_t::*func_t) (size_t, size_t, point2d const&) const;
    func_t const& func = &target_t::make_edge;
    return Accessor::const_invoke(func, source, target, spt);
  }
};


STAPL_PROXY_HEADER(delaunay_edge),
  public proxy<delaunay_reduced_edge<delaunay_edge>, Accessor>
{
  typedef edge_descriptor_impl<size_t> edge_descriptor;
  typedef proxy<delaunay_reduced_edge<delaunay_edge>, Accessor> base_type;

  STAPL_PROXY_DEFINES(delaunay_edge)
  STAPL_PROXY_METHOD_RETURN(get_source_point, point2d)
  STAPL_PROXY_METHOD_RETURN(get_Sym,          delaunay_edge)
  STAPL_PROXY_METHOD_RETURN(left_of,          bool, point2d)
  STAPL_PROXY_METHOD_RETURN(right_of,         bool, point2d)
  STAPL_PROXY_METHOD_RETURN(valid,            bool, delaunay_edge)
  STAPL_PROXY_METHOD_RETURN(is_vertical,      bool)
  STAPL_PROXY_METHOD_RETURN(is_collinear,     bool, delaunay_edge)
  STAPL_PROXY_METHOD_RETURN(source,           size_t)
  STAPL_PROXY_METHOD_RETURN(target,           size_t)

  STAPL_PROXY_METHOD_RETURN(get_Org, point2d)
  STAPL_PROXY_METHOD_RETURN(calc_angle, double)
  STAPL_PROXY_METHOD_RETURN(get_ed, edge_descriptor)
  STAPL_PROXY_METHOD_RETURN(get_reverse_ed, edge_descriptor)

  STAPL_PROXY_METHOD_RETURN(operator==, bool, delaunay_edge)
  STAPL_PROXY_METHOD_RETURN(operator!=, bool, delaunay_edge)
  STAPL_PROXY_METHOD_RETURN(operator<, bool, delaunay_edge)
  STAPL_PROXY_METHOD_RETURN(operator>, bool, delaunay_edge)
  STAPL_PROXY_METHOD_RETURN(operator<=, bool, delaunay_edge)
  STAPL_PROXY_METHOD_RETURN(operator>=, bool, delaunay_edge)

  template <typename GView>
  delaunay_edge get_Onext(GView const& a0) const
  { return Accessor::const_invoke(&target_t::get_Onext<GView>, a0); }

  template <typename GView>
  delaunay_edge get_Oprev(GView const& a0) const
  { return Accessor::const_invoke(&target_t::get_Oprev<GView>, a0); }

  template <typename GView>
  delaunay_edge get_Dnext(GView const& a0) const
  { return Accessor::const_invoke(&target_t::get_Dnext<GView>, a0); }

  template <typename GView>
  delaunay_edge get_Dprev(GView const& a0) const
  { return Accessor::const_invoke(&target_t::get_Dprev<GView>, a0); }

  template <typename GView>
  delaunay_edge get_Lnext(GView const& a0) const
  { return Accessor::const_invoke(&target_t::get_Lnext<GView>, a0); }

  template <typename GView>
  delaunay_edge get_Lprev(GView const& a0) const
  { return Accessor::const_invoke(&target_t::get_Lprev<GView>, a0); }

  template <typename GView>
  delaunay_edge get_Rprev(GView const& a0) const
  { return Accessor::const_invoke(&target_t::get_Rprev<GView>, a0); }

  template <typename GView>
  delaunay_edge get_Sym(GView const& a0) const
  { return Accessor::const_invoke(&target_t::get_Sym<GView>, a0); }

  STAPL_PROXY_METHOD_RETURN(get_copy, delaunay_edge)
};


} // namespace stapl


#endif /* STAPL_BENCHMARK_LONESTAR_DELAUNAY_EDGE_HPP */
