/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_ENVIRONMENTS_GRAPHVIZ_ENV_HPP
#define STAPL_SKELETONS_ENVIRONMENTS_GRAPHVIZ_ENV_HPP

#include <type_traits>
#include <ostream>
#include <iosfwd>
#include <sstream>
#include <fstream>
#include <memory>
#include <stapl/runtime/config/types.hpp>
#include <stapl/skeletons/flows/producer_info.hpp>
#include <stapl/skeletons/utility/printers.hpp>

namespace stapl {
namespace skeletons {

namespace debug_helpers {

//////////////////////////////////////////////////////////////////////
/// @brief This coloring is used in @c graphviz_env for coloring nodes
/// created by each location with a different color
///
/// @see graphviz_env
///
/// @ingroup skeletonsEnvironmentsInternal
//////////////////////////////////////////////////////////////////////
inline std::string get_color_name(std::size_t i)
{
  switch (i % 16) {
    case 0 : return "red";
    case 1 : return "blue";
    case 2 : return "green";
    case 3 : return "orange";
    case 4 : return "deeppink4";
    case 5 : return "hotpink";
    case 6 : return "khaki4";
    case 7 : return "navy";
    case 8 : return "purple1";
    case 9 : return "turquoise2";
    case 10 : return "tomato";
    case 11 : return "brown";
    case 12 : return "lavenderblush4";
    case 13 : return "gold";
    case 14 : return "chocolate4";
    case 15 : return "magenta2";
    default : return "purple";
  }
}

} // namespace debug_helpers

//////////////////////////////////////////////////////////////////////
/// @brief debug environment can be used along with an output stream
/// such as @c std::cout in order to dump a GraphViz representation of
/// the dependence graph in an output stream.
/// Remember, in the cases that a dynamic (conditional ) dependence
/// graphs such as @c do_while is used, you have to use this
/// along with a real execution environment such as @c taskgraph_env
///
/// @see do_while
/// @see taskgraph_env
///
/// @ingroup skeletonsEnvironments
//////////////////////////////////////////////////////////////////////
class graphviz_env
{
  struct node_info
  {
    bool m_is_result_node;
    std::size_t m_num_succs;
    std::string m_id;
    std::string m_description;

    node_info(bool is_result_node, std::size_t num_succs,
              std::string const& id,
              std::string const& description)
      : m_is_result_node(is_result_node),
        m_num_succs(num_succs),
        m_id(id),
        m_description(description)
    { }
  };

  enum class kind{task, notifications, view, constant, reflexive};

  struct edge_info
  {
    std::string m_source;
    std::string m_destination;
    kind m_kind;
    std::string m_description;

    edge_info(std::string const& source,
              std::string const& destination,
              kind k,
              std::string const& description)
      : m_source(source),
        m_destination(destination),
        m_kind(k),
        m_description(description)
    { }
  };

  struct graph_type
  {
    std::size_t m_id;
    std::vector<edge_info> m_edges;
    std::vector<node_info> m_nodes;
    std::vector<graph_type> m_inner_graphs;
    graph_type* m_parent;
    std::set<std::pair<std::string, std::string>> m_sources;
    std::string m_title;
  };

  std::shared_ptr<graph_type> m_graph;
  std::string m_output_filename;
  std::size_t m_num_PEs;
  runtime::location_id m_PE_id;
  graph_type* m_cur_graph;
public:
  graphviz_env(std::string const& name)
    : m_graph(std::make_shared<graph_type>()),
      m_output_filename(name + "." + std::to_string(get_location_id()) +
                        ".dot"),
      m_num_PEs(-1),
      m_PE_id(-1),
      m_cur_graph(m_graph.get())
  {
    // the main graph does not have a parent
    m_cur_graph->m_parent = nullptr;
  }

  static void output_graph(graph_type* graph, std::ostream& o)
  {
    o << "subgraph cluster_" << graph->m_id
      << "{\n"
      << "style=filled;\n"
      << "color="
      << std::to_string(std::hash<std::string>()(graph->m_title) % 5 + 1)
      << ";\n"
      << "label=\"" << graph->m_title << "\";\n";

    for (auto&& src : graph->m_sources) {
      o << src.first
        << "[label=\"" << src.second << "\","
        << R"(shape="box", style=filled, fillcolor=ivory3, color=black];)"
        << "\n";
    }
    for (auto&& node : graph->m_nodes) {
      o << node.m_id
        << "[label=\"{"
        << ((node.m_description == "") ? node.m_id : node.m_description)
        << "|num-succs = " << node.m_num_succs << "}\", "
        << "fillcolor=white, style=filled, color="
        << debug_helpers::get_color_name(get_location_id())
        << "];\n";
    }
    for (auto&& edge : graph->m_edges) {
      std::string description = edge.m_kind == kind::view ?
        ("[label=\"" + edge.m_description + "\", color=green]") : "";
      o << edge.m_source << "->" << edge.m_destination
        << description << ";\n";
    }
    for (auto&& g : graph->m_inner_graphs) {
      output_graph(&g, o);
    }
    o << "}\n";
  }

  ~graphviz_env()
  {
    if (m_graph.unique()) {
      std::ofstream out;
      out.open(m_output_filename);
      output_graph(m_graph.get(), out);
      out.close();
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Every @c spawn_element is converted to a node in the
  /// GraphViz output. The label for this node is in the format of
  /// <b>tid(num_succs)</b>. There is also a color representation which
  /// is associated with each task. This color changes based on the
  /// location id of each task. This is useful when the load balancing
  /// of an algorithm should be checked. Remember that this shows the
  /// load of each processor during the task creation process and does
  /// not necessarily represent the execution load balance.
  ///
  /// @param tid       the unique id assigned to the node in the graph
  /// @param result_id result_id of this task if it has one
  /// @param wf        the workfunction is discarded in here
  /// @param mapper    the output to output mapper for mapping the results
  /// @param num_succs  the exact number of successors for this element.
  ///                  Remember that in some cases this value is set to
  ///                  @c stapl::defer_specs (when the @c spawner is in
  ///                  @c SET_HOLD mode). This number is shown in
  ///                  parenthesis in the output graph.
  /// @param in        the producer information for the node's input
  ///                  arguments that will be translated as edge in the
  ///                  output graph
  //////////////////////////////////////////////////////////////////////
  template <bool isResult, typename WF, typename Mapper, typename... In>
  void spawn_element(std::size_t tid,
                     std::size_t result_id,
                     WF&& wf,
                     Mapper&& mapper,
                     std::size_t num_succs,
                     In&&... in) const
  {
    spawn_element<isResult>(
      tid, result_id, std::vector<std::size_t>(),
      std::forward<WF>(wf), std::forward<Mapper>(mapper),
      num_succs, std::forward<In>(in)...);
  }

  template <bool isResult, typename WF, typename Mapper, typename... In>
  void spawn_element(std::size_t tid,
                     std::size_t result_id,
                     std::vector<std::size_t> const& notifications,
                     WF&& wf,
                     Mapper&& mapper,
                     std::size_t num_succs,
                     In&&... in) const
  {
    std::string tid_str = "task_" + std::to_string(tid);
    std::string description = "task-id = " + std::to_string(tid);

#ifdef SKELETONS_FULL_NODE_NAME
    std::stringstream s;
    skeletons::show(wf, s);
    description = s.str();
    skeletons::replace_string(description, "<", "&lt;");
    skeletons::replace_string(description, ">", "&gt;");
#endif
    m_graph->m_nodes.emplace_back(isResult, num_succs, tid_str,
                                  description);
    add_edges(tid_str, std::forward<In>(in)...);
    add_notification_edges(tid_str, notifications);
  }

  template <typename Skeleton, typename... Arg>
  void pre_spawn(Skeleton&& skeleton,
                std::size_t lid_offset,
                Arg&&...)
  {
    graph_type g;
    std::stringstream s;
    skeletons::show(skeleton, s);

    std::string name = s.str();
    g.m_title = name;
    g.m_id = std::hash<std::string>()(name + std::to_string(lid_offset));
    g.m_parent = m_cur_graph;

    m_cur_graph->m_inner_graphs.push_back(g);
    m_cur_graph = &m_cur_graph->m_inner_graphs.back();
  }

  template <typename Skeleton, typename... Args>
  void post_spawn(Skeleton&& skeleton, Args&&... args)
  {
    m_cur_graph = m_cur_graph->m_parent;
  }

  void set_num_succs(std::size_t tid, std::size_t num_succs) const
  { }

  void init_location_info(std::size_t num_PEs, runtime::location_id PE_id)
  {
    m_num_PEs = num_PEs;
    m_PE_id = PE_id;
  }

  std::size_t get_num_PEs() const
  {
    return m_num_PEs;
  }

  runtime::location_id get_PE_id() const
  {
    return m_PE_id;
  }

private:
  template <typename C>
  std::string get_cluster_name(C const& container, std::true_type) const
  {
    std::stringstream o;
    o << container.get_rmi_handle().get_uid();
    std::string s(o.str());
    replace_string(s, "&", "");
    return s;
  }

  template <typename C>
  std::string get_cluster_name(C const& container, std::false_type) const
  {
    return "FunctorView";
  }

  template <typename V, typename CP>
  std::string get_cluster_name(
    view_coarsen_impl::coarsen_container<V, CP> const& c) const
  {
    return get_cluster_name(c.container().container(),
                            is_p_object<typename V::view_container_type>());
  }

  template <typename C>
  std::string get_cluster_name(C const& container) const
  {
    return get_cluster_name(container, is_p_object<C>());
  }

  template <typename View>
  std::tuple<std::string, kind, std::string>
  get_source_info(flows::view_element_producer<View> const& producer) const
  {
    std::string node_id = get_cluster_name(producer.get_element().container());

    std::stringstream s;
    skeletons::show(producer.get_element(), s);
    // add it to the sources
    m_cur_graph->m_sources.emplace(node_id, s.str());

    // get the edge label
    s.str("");
    show_value(producer.get_index(), s);

    return make_tuple(node_id, kind::view, s.str());
  }

  template <typename Element>
  std::tuple<std::string, kind, std::string>
  get_source_info(
    flows::reflexive_producer<Element> const& producer) const
  {
    std::stringstream s;
    skeletons::show(producer.get_element(), s);
    return make_tuple(s.str(), kind::reflexive, "");
  }

  template <typename T>
  std::tuple<std::string, kind, std::string>
  get_source_info(flows::constant_producer<T> const& producer) const
  {
    std::stringstream s;
    s << "C" << producer.get_element();

    // add it to the sources
    std::string node_id = s.str();

    s.str("");
    s << "Constant(" << producer.get_element() << ")";
    m_cur_graph->m_sources.emplace(node_id, s.str());

    return make_tuple(node_id, kind::constant, "");
  }

  template <typename Producer>
  std::tuple<std::string, kind, std::string>
  get_source_info(Producer const& producer) const
  {
    return make_tuple("task_" + std::to_string(producer.get_index()),
                      kind::task, "");
  }

  template <typename In0>
  void add_edges(std::string const& tid, In0 const& in0) const
  {
    auto source_info = get_source_info(in0);
    m_cur_graph->m_edges.emplace_back(
      stapl::get<0>(source_info),
      tid,
      stapl::get<1>(source_info),
      stapl::get<2>(source_info));
  }

  template <typename V, typename I, typename F>
  void add_edges(std::string const& tid,
    flows::indexed_producers<V, I, F> const& producer) const
  {
    for (auto&& i : producer.get_indices()) {
      m_cur_graph->m_edges.push_back(i, tid);
    }
  }

  template <typename In0, typename ...In>
  void add_edges(std::string const& tid,
                 In0 const& in0, In&&... in) const
  {
    add_edges(tid, in0);
    add_edges(tid, in...);
  }

  void add_notification_edges(
         std::string const& tid,
         std::vector<std::size_t> const& notifications) const
  {
    for (auto&& n_source : notifications) {
      m_cur_graph->m_edges.emplace_back(
        std::to_string(n_source),
        tid,
        kind::notifications,
        "no-data");
    }
  }
};


}
} // namespace stapl

#endif // STAPL_SKELETONS_ENVIRONMENTS_GRAPHVIZ_ENV_HPP
