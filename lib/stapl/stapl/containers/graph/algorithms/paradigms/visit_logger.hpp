/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_PARADIGMS_VISIT_LOGGER_HPP
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_PARADIGMS_VISIT_LOGGER_HPP

#include <sstream>

namespace stapl {

namespace sgl {

struct visit_logger
{
#if defined(STAPL_SGL_ENABLE_VISIT_LOGGER)
  template <typename Vertex, typename NeighborOp>
  static void visit_from(Vertex&& v,
                         NeighborOp&& neighbor_op,
                         std::size_t level,
                         std::size_t max_level)
  {
    std::stringstream ss;
    ss << "sgl visit-from " << v.descriptor() << " " << level << " "
       << max_level << " (" << v.property() << ")"
       << " (" << neighbor_op << ")";

    printf("%s\n", ss.str().c_str());
  }

  template <typename Vertex>
  static void
  pre_vertex_op(Vertex&& v, std::size_t level, std::size_t max_level)
  {
    std::stringstream ss;
    ss << "sgl pre-vertex-op " << v.descriptor() << " " << level << " "
       << max_level << " (" << v.property() << ")";

    printf("%s\n", ss.str().c_str());
  }

  template <typename Vertex>
  static void post_vertex_op(Vertex&& v,
                             bool should_continue,
                             std::size_t level,
                             std::size_t max_level)
  {
    std::stringstream ss;
    ss << "sgl post-vertex-op " << v.descriptor() << " " << level << " "
       << max_level << " [" << should_continue << "]"
       << " (" << v.property() << ")";

    printf("%s\n", ss.str().c_str());
  }

  template<typename Vertex, typename NeighborOp>
  static void pre_neighbor_op(Vertex&& v, NeighborOp&& neighbor_op)
  {
    std::stringstream ss;
    ss << "sgl pre-neighbor-op " << v.descriptor()
       << " (" << v.property() << ")"
       << " (" << neighbor_op << ")";

    printf("%s\n", ss.str().c_str());
  }

  template <typename Vertex, typename NeighborOp>
  static void post_neighbor_op(Vertex&& v,
                               NeighborOp&& neighbor_op,
                               bool reinvoke,
                               bool can_repropagate)
  {
    std::stringstream ss;
    ss << "sgl post-neighbor-op " << v.descriptor()
       << " [" << reinvoke << "]"
       << " [" << can_repropagate << "]"
       << " (" << v.property() << ")"
       << " (" << neighbor_op << ")";

    printf("%s\n", ss.str().c_str());
  }

#else
  template<typename Vertex, typename NeighborOp>
  static void visit_from(Vertex&&, NeighborOp&&, std::size_t, std::size_t)
  { }

  template<typename Vertex>
  static void pre_vertex_op(Vertex&&, std::size_t, std::size_t)
  { }

  template<typename Vertex>
  static void post_vertex_op(Vertex&&, bool, std::size_t, std::size_t)
  { }

  template<typename Vertex, typename NeighborOp>
  static void pre_neighbor_op(Vertex&&, NeighborOp&&)
  { }

  template<typename Vertex, typename NeighborOp>
  static void post_neighbor_op(Vertex&&, NeighborOp&&, bool, bool)
  { }
#endif
};

} // namespace sgl


} // namespace stapl

#endif
