/*
 // Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
 // component of the Texas A&M University System.

 // All rights reserved.

 // The information and source code contained herein is the exclusive
 // property of TEES and may not be disclosed, examined or reproduced
 // in whole or in part without explicit written authorization from TEES.
 */

#include <cstdlib>
#include <iostream>

#include <stapl/runtime.hpp>
#include <stapl/array.hpp>
#include <stapl/skeletons/spans/misc.hpp>
#include <stapl/skeletons/operators/farm.hpp>
#include <stapl/skeletons/executors/execute.hpp>
#include <stapl/views/repeated_view.hpp>
#include <stapl/containers/graph/algorithms/breadth_first_search.hpp>
#include <stapl/containers/graph/algorithms/properties.hpp>
#include <stapl/containers/graph/digraph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/algorithms/properties.hpp>
#include <stapl/containers/graph/generators/mesh.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include "../expect.hpp"

using namespace stapl;

//////////////////////////////////////////////////////////////////////
/// @brief Receiving a farm instance and a vertex and distance, this
/// BFS operation first checks if the received distance is smaller
/// than the previously known value to the vertex. If so, it propagates
/// the information to its children.
///
/// @tparam VertexDescriptor the type of vertex descriptor to be used.
//////////////////////////////////////////////////////////////////////
struct bfs_op
{
  using result_type = void;

  template <typename Farm, typename Vertex, typename GraphView,
            typename Parent, typename Level>
  void operator()(Farm&& farm, Vertex&& vertex, GraphView& graph_view,
                  Parent&& parent,
                  Level&& level)
  {
    using vertex_descriptor_t =
      typename std::decay<GraphView>::type::vertex_descriptor;
    using pair_t = std::pair<GraphView&, vertex_descriptor_t>;

    if (vertex.property().level() > level) {
      vertex.property().level(level);
      vertex.property().parent(parent);

      for (auto&& e : vertex) {
        farm.add(*this,
                 pair_t{graph_view, e.target()},
                 graph_view,
                 vertex.descriptor(),
                 (level + 1));
      }
    }
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Creates the initial seed of the BFS algorithm.
///
/// @tparam VertexDescriptor the type of vertex descriptor to be used.
//////////////////////////////////////////////////////////////////////
template <typename VertexDescriptor>
class bfs_init
{
private:
  VertexDescriptor m_source;

public:
  explicit bfs_init(VertexDescriptor const& source)
    : m_source(source)
  { }

  template <typename Farm, typename GraphView, typename... Args>
  void operator()(Farm&& farm, GraphView& graph_view, Args&&... args)
  {
    farm.add(bfs_op(),
             std::make_pair(graph_view, m_source),
             graph_view,
             m_source,
             1ul);
  }

  void define_type(typer& t)
  {
    t.member(m_source);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief This class is used to set the level property of all vertices
/// in a graph to max.
//////////////////////////////////////////////////////////////////////
struct set_to_max
{
  using result_type = void;

  template <typename Vertex>
  void operator()(Vertex&& v)
  {
    using v_t = typename std::decay<Vertex>::type::vertex_descriptor;
    v.property().level(std::numeric_limits<v_t>::max());
    v.property().parent(v.descriptor());
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Reads the level property of a given vertex and stores it
/// in the other element.
//////////////////////////////////////////////////////////////////////
struct extract_level_wf
{
  typedef void result_type;

  template<typename V, typename E>
  void operator() (V v, E e) const
  {
    e = v.property().level();
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Outputs the graph in GraphViz Dot format into the given
/// output stream.
///
/// @param os an output stream
/// @param g  a directed graph
//////////////////////////////////////////////////////////////////////
template <typename GraphView>
void generate_graphviz_output(std::ostream& os, GraphView&& g)
{
  bool isDirected      = true;
  std::string edge_str = isDirected ? "->" : "--";
  using descriptor_t = typename std::decay<GraphView>::type::vertex_descriptor;
  auto create_edge   = [&](std::ostream& os, descriptor_t const& src,
                         descriptor_t const& tgt) {
    os << src << edge_str << tgt;
  };

  std::string graph_str = isDirected ? "digraph" : "graph";
  os << graph_str << " {\n";
  os << "node [shape=record];\n";
  for (auto&& v : g) {
    os << v.descriptor();
    os << "[label=\"vertex:" << v.descriptor() << "|level:";
    os << v.property().level();
    os << "\"]\n";
    for (auto&& edge : v) {
      create_edge(os, v.descriptor(), edge.target());
      os << "\n";
    }
  }
  os << "}\n";
}

stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 3) {
    std::cout << "<exec> <n> <m>" << std::endl;
    exit(1);
  }

  std::size_t nx = atol(argv[1]);
  std::size_t ny = atol(argv[2]);

  using namespace stapl;
  using skeletons::execute;
  using skeletons::default_execution_params;
  using skeletons::farm;

  using PGR          = stapl::digraph<properties::bfs_property>;
  using descriptor_t = typename PGR::vertex_descriptor;
  using graph_view_t = stapl::graph_view<PGR>;
  auto graph_view1   = stapl::generators::make_mesh<graph_view_t>(nx, ny);
  auto graph_view2   = stapl::generators::make_mesh<graph_view_t>(nx, ny);

  std::size_t size = nx * ny;

  // Containers to be used for sanity checks
  using sanity_container_t = stapl::static_array<std::size_t>;
  sanity_container_t farm_result{size};
  sanity_container_t async_result{size};

  auto farm_result_vw = make_array_view(farm_result);
  auto async_result_vw = make_array_view(async_result);

  stapl::counter<stapl::default_timer> my_timer;
  my_timer.start();


  stapl::map_func(set_to_max(), graph_view1);
  skeletons::execute(default_execution_params(),
                     farm(bfs_init<descriptor_t>(0ul)),
                     // TODO(mani) this counting view can needs to be removed.
                     // It is required now to avoid the problem of infinite
                     // domains in location_mapper.hpp
                     counting_view<int>(get_num_locations(), int{0}),
                     make_repeat_view(graph_view1));

  double t = my_timer.stop();

  stapl::do_once([&t](){
    std::cout << "Time in farm-based BFS:" << t << std::endl;
  });

  auto kla_exec_policy =
    stapl::sgl::make_execution_policy("kla", graph_view2,
                                      std::numeric_limits<size_t>::max() - 2);

  my_timer.reset();
  my_timer.start();

  stapl::breadth_first_search(kla_exec_policy, graph_view2, 0);

  t = my_timer.stop();

  stapl::do_once([&t](){
    std::cout << "Time in asynchronous BFS:" << t << std::endl;
  });

  stapl::map_func(extract_level_wf(), graph_view1, farm_result_vw);
  stapl::map_func(extract_level_wf(), graph_view2, async_result_vw);

  bool are_equal = stapl::equal(farm_result_vw, async_result_vw);

  stapl::do_once([are_equal](){
    stapl::tests::expect(are_equal) << "Asynchronous BFS and farm-based BFS";
  });

  return EXIT_SUCCESS;
}
