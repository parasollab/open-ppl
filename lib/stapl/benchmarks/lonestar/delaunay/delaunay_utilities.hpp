/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARK_LONESTAR_DELAUNAY_UTILITES_HPP
#define STAPL_BENCHMARK_LONESTAR_DELAUNAY_UTILITES_HPP

#include <benchmarks/lonestar/delaunay/delaunay_point.hpp>

#include <string>
#include <cmath>
#include <math.h>



namespace stapl
{


namespace delaunay
{


struct point2d;

///////////////////////////////////////////////////////////////////////////////
/// @brief Returns the angle stored on the edge referenced by the provided
/// iterator.
///////////////////////////////////////////////////////////////////////////////
template <typename EIter>
auto get_angle(EIter&& e_it) -> decltype((*e_it).property().get_angle())
{ return (*e_it).property().get_angle(); }


///////////////////////////////////////////////////////////////////////////////
/// @brief Returns the angle stored on the edge referenced by the provided
/// edge_reference.
///////////////////////////////////////////////////////////////////////////////
template <typename Edge>
auto get_angle(Edge&& e) -> decltype(e.property().get_angle())
{ return e.property().get_angle(); }


///////////////////////////////////////////////////////////////////////////////
/// @brief Returns the angle stored on the edge referenced by the provided
/// edge_property.
///////////////////////////////////////////////////////////////////////////////
template <typename EdgeProp>
auto get_angle(EdgeProp&& e) -> decltype(e.get_angle())
{ return e.get_angle(); }


///////////////////////////////////////////////////////////////////////////////
/// @brief Comparison workfunction for sorting edges of a vertex.
///////////////////////////////////////////////////////////////////////////////
struct compare_edge_wf
{
  typedef bool result_type;

  template <typename Edge, typename EdgeProp>
  result_type operator()(Edge&& a, EdgeProp&& b) const
  { return get_angle(a) < get_angle(b); }
};


///////////////////////////////////////////////////////////////////////////////
/// @brief Insertion Sort Algorithm for sorting edges of Vertex
///////////////////////////////////////////////////////////////////////////////
struct sort_vertex_edges_wf
{
  typedef void result_type;


  template <typename Vertex>
  void operator()(Vertex&& v) const
  {
    auto size = v.size();
    if (size <= 1 ) {
      return;
    }

    auto const& vit_end = v.end();
    if (size > 2) {
      auto const& vit_b   = v.begin() - 1;
      auto const& vit_end1 = vit_end   - 1;
      auto vit_e = vit_end - 2;

      for (; vit_e != vit_b; --vit_e) {
        auto v_x(std::move(*vit_e));
        auto vit = vit_e;

        while ((vit != vit_end1) && (get_angle(v_x) > get_angle(vit+1))) {
          *vit = std::move(*(vit+1));
          ++vit;
        }
        *vit = std::move(v_x);
      }
    }

    if (get_angle(vit_end-2) > get_angle(vit_end-1)) {
      std::swap(*(vit_end-2), *(vit_end-1));
    }
  }
};


///////////////////////////////////////////////////////////////////////////////
/// @brief Core functions for @ref ccw_edge_wf and @cw_edge_wf that include
/// searching for the provided edge.
///////////////////////////////////////////////////////////////////////////////
template <typename Edge>
struct edge_wf
{
  typedef Edge result_type;
  typedef edge_descriptor_impl<size_t> edge_descriptor;

  Edge m_edge;


  edge_wf(Edge const& edge)
    : m_edge(edge)
  { }


  template <typename EIter>
  EIter find_edge(EIter const& e_itb, EIter const& e_ite) const
  {
    double const& edge_target = m_edge.target();
    for (auto e_it = e_itb; e_it != e_ite; ++e_it) {
      if ((*e_it).target() == edge_target) {
        return e_it;
      }
    }

    return find_edge_by_angle(e_itb, e_ite);
  }


  template <typename EIter>
  EIter find_edge_by_angle(EIter const& e_itb, EIter const& e_ite) const
  {
    auto e_it = e_itb;
    double first_angle = get_angle(e_it);
    double last_angle = get_angle(e_ite-1);
    double const& edge_angle = m_edge.get_angle();

    if (edge_angle < first_angle || last_angle < edge_angle) {
      double diff1 = fabs(first_angle - edge_angle);
      double diff2 = fabs(last_angle -  edge_angle);
      double angle_range = point2d::get_angle_range();
      diff1 = std::min(diff1, fabs((first_angle + angle_range) - edge_angle));
      diff2 = std::min(diff2, fabs(edge_angle - (last_angle - angle_range)));
      if (diff2 < diff1) {
        return e_ite-1;
      } else {
        return e_it;
      }
    } else {
      auto e_closest = e_ite;
      for (; e_it != e_ite; ++e_it) {
        if (get_angle(e_it) <= edge_angle) {
          e_closest = e_it;
        } else {
          auto diff1 = edge_angle - get_angle(e_closest);
          auto diff2 = get_angle(e_it) - edge_angle;
          if (diff2 < diff1) {
            return e_it;
          }
        }
      }
      return e_ite;
    }
  }


  template <typename Vertex>
  auto find_edge(Vertex const& v) const -> decltype(v.begin())
  { return find_edge(v.begin(), v.end()); }


  template <typename Vertex>
  auto find_edge_by_angle(Vertex const& v) const -> decltype(v.begin())
  { return find_edge_by_angle(v.begin(), v.end()); }


  template <typename Vertex>
  auto closest_edge_by_angle(Vertex const& v) const -> decltype(v.begin())
  {
    auto const& e_ite = v.end();
    auto e_closest = v.end();
    double closest_angle = 1000.0;
    for (auto e_it =  v.begin(); e_it != e_ite; ++e_it) {
      double diff = fabs(get_angle(e_it) - m_edge.get_angle());
      if (diff <= closest_angle) {
        closest_angle = diff;
        e_closest = e_it;
      }
    }
    return e_closest;
  }

  void define_type(stapl::typer& t)
  { t.member(m_edge); }
};


///////////////////////////////////////////////////////////////////////////////
/// @brief Workfunction to get the adjacent edge that is ccw of the given edge
/// of the vertex passed into workfunction.
///////////////////////////////////////////////////////////////////////////////
template <typename edge_property>
struct ccw_edge_wf
  : public edge_wf<edge_property>
{
  typedef edge_property result_type;
  typedef edge_descriptor_impl<size_t> edge_descriptor;

  ccw_edge_wf(edge_property const& edge)
    : edge_wf<edge_property>(edge)
  { }

  template <typename Vertex>
  edge_property operator()(Vertex&& v) const
  {
    if (v.size() <= 1) {
      return this->m_edge;
    }
    auto const& e_ite = v.end();
    auto e_it =  this->find_edge(v.begin(), e_ite);

    stapl_assert(e_it != e_ite || e_it == e_ite, "Could not find CCW edge");

    if (e_it == v.begin()) {
      return v.property().make_edge(v, (*(e_ite-1)));
    } else {
      return v.property().make_edge(v, (*(e_it-1)));
    }
  }
};


///////////////////////////////////////////////////////////////////////////////
/// @brief Workfunction to get the adjacent edge that is cw of the given edge
/// of the vertex passed into workfunction.
///////////////////////////////////////////////////////////////////////////////
template <typename edge_property>
struct cw_edge_wf
  : public edge_wf<edge_property>
{
  typedef edge_property result_type;
  typedef edge_descriptor_impl<size_t> edge_descriptor;

  cw_edge_wf(edge_property const& edge)
    : edge_wf<edge_property>(edge)
  { }

  template <typename Vertex>
  edge_property operator()(Vertex&& v) const
  {
    if (v.size() <= 1) {
      return this->m_edge;
    }

    auto const& e_itb = v.begin();
    auto const& e_ite = v.end();
    auto e_it =  this->find_edge(e_itb, e_ite);

    stapl_assert(e_it != e_ite, "Could not find CW edge");

    if (e_it == e_ite - 1) {
      return v.property().make_edge(v, (*(e_itb)));
    } else {
      return v.property().make_edge(v, (*(e_it+1)));
    }
  }
};


} // namespace delaunay


} // namespace stapl



#endif /* STAPL_BENCHMARK_LONESTAR_DELAUNAY_UTILITES_HPP */
