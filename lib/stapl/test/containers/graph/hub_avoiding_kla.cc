/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <iostream>
#include <utility>
#include <vector>

#include <stapl/containers/graph/multigraph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/algorithms/paradigms/kla_paradigm.hpp>
#include <stapl/containers/graph/algorithms/connected_components.hpp>
#include <stapl/containers/graph/generators/star.hpp>
#include <stapl/algorithms/algorithm.hpp>

#include "test_util.h"

using namespace stapl;

//////////////////////////////////////////////////////////////////////
/// @brief Property for the modified connected components algorithm.
///
/// We are storing per-vertex a cc id, how many times it has been visited
/// by the vertex operator and a flag to denote that the vertex is
/// active
//////////////////////////////////////////////////////////////////////
class forward_property
{
private:
  size_t m_cc;
  size_t m_touched;
  bool m_active;

public:
  forward_property(void)
    : m_cc(0),
      m_touched(0), m_active(true)
  { }

  void cc(size_t c)
  { m_cc = c; }

  size_t cc(void) const
  { return m_cc; }

  size_t touched(void) const
  { return m_touched; }

  void active(bool a)
  { m_active = a; }

  bool active(void) const
  { return m_active; }

  void touch()
  {
    m_touched++;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_cc);
    t.member(m_touched);
    t.member(m_active);
  }
};

namespace stapl {

STAPL_PROXY_HEADER(forward_property)
{
  STAPL_PROXY_DEFINES(forward_property)

  STAPL_PROXY_METHOD(cc, size_t)
  STAPL_PROXY_METHOD(active, bool)
  STAPL_PROXY_METHOD(touch)
  STAPL_PROXY_METHOD_RETURN(cc, size_t)
  STAPL_PROXY_METHOD_RETURN(active, bool)
  STAPL_PROXY_METHOD_RETURN(touched, size_t)
};

}

//////////////////////////////////////////////////////////////////////
/// @brief Neighbor operator for the modified connected component algorithm.
///
/// Upon being visited from a neighbor, we will check if the incoming
/// id is larger than the currently stored id. If so, we will update it
/// and mark ourself as active.
//////////////////////////////////////////////////////////////////////
class forward_neighbor_operator
{
public:

  std::size_t m_cc_id;

  forward_neighbor_operator(std::size_t const& cc_id =
                              std::numeric_limits<std::size_t>::max())
    : m_cc_id(cc_id)
  { }

  template <class Vertex>
  bool operator()(Vertex&& target) const
  {
    if (target.property().cc() < m_cc_id )
    {
      target.property().cc(m_cc_id);
      target.property().active(true);
      return true;
    }

    return false;
  }

  void define_type(typer& t)
  { t.member(m_cc_id); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Vertex operator for the modified connected component algorithm.
///
/// Each vertex will send its cc id to all of its neighbors that have a
/// descriptor that is less than it.
//////////////////////////////////////////////////////////////////////
class forward_vertex_operator
{
public:
  typedef bool result_type;

  template<class Vertex, class GraphVisitor>
  result_type operator()(Vertex&& v, GraphVisitor&& graph_visitor) const
  {
    if (v.property().active())
    {
      v.property().touch();

      const std::size_t id = v.descriptor();

      // visit all of the edges that are to vertices whose descriptors
      // are less than this vertex's descriptor
      graph_visitor.visit_all_edges_if(v,
        forward_neighbor_operator(v.property().cc()),
        [id](typename Vertex::adj_edges_type::edge_type const& e) {
          return e.target() < id;
        }
      );

      v.property().active(false);

      return true;
    }

    return false;
  }
};


std::pair<std::size_t, std::size_t>
star_traversal(std::size_t n, std::size_t root, bool hub_avoidance)
{
  typedef multigraph<forward_property> graph_type;
  typedef graph_view<graph_type> view_type;

  // anything that has more than 2 edges is a hub
  const std::size_t degree_threshold = 2;
  const std::size_t k = std::numeric_limits<std::size_t>::max()-1;

  // generate a star graph and initialize each vertex's cc value
  // to its descriptor
  view_type vw = generators::make_star<view_type>(n, root);
  map_func(cc_algo_detail::init_cc_wf(), vw);

  // set up the parameters to avoid hubs
  kla_params<view_type> params;
  params.avoid_hubs = hub_avoidance;
  params.degree_threshold = degree_threshold;

  // perform the traversal on the star graph. after traversal, the vertex with
  // the lowest id should have a cc value of the largest id
  kla_paradigm(
    forward_vertex_operator(), forward_neighbor_operator(), vw, k, params
  );

  const std::size_t highest_id = vw[0].property().cc();
  const std::size_t touched = vw[0].property().touched();

  rmi_fence();

  return std::make_pair(highest_id, touched);
}


stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 2) {
    std::cout << "usage: " << argv[0] << " n" << std::endl;
    exit(1);
  }

  const std::size_t n = atol(argv[1]);
  const std::size_t root = 1;

  one_print("Testing KLA with hub avoidance...\t\t");

  auto const with_hub_avoidance = star_traversal(n, root, true);
  auto const without_hub_avoidance = star_traversal(n, root, false);

  const bool same_answer =
    without_hub_avoidance.first == with_hub_avoidance.first;
  const bool lower_redone =
    without_hub_avoidance.second > with_hub_avoidance.second;

  const bool passed = same_answer && lower_redone;

  one_print(passed);

  return EXIT_SUCCESS;
}
