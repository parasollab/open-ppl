/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/runtime.hpp>
#include <stapl/containers/graph/algorithms/paradigms/graph_paradigm.hpp>
#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/algorithms/properties.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/views/repeated_view.hpp>
#include <stapl/utility/tuple.hpp>
#include <algorithm>
#include <iostream>

namespace stapl {

namespace preflow_push_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Functor used to print an edge and its properties.
/// @param vd Vertex descriptor of the edge owner.
/// @param edge Edge being printed.
//////////////////////////////////////////////////////////////////////
template<typename E>
void print_edge(size_t vd, E edge)
{
  std::cout << "(" << vd << ", " << edge.target() << ") --> RC: "
    << edge.property().rc() << ", H: " << edge.property().height()
    << std::endl;
}

//////////////////////////////////////////////////////////////////////
/// @brief Functor used to print a vertex and its properties.
/// @param vertex Vertex that performs operation.
//////////////////////////////////////////////////////////////////////
template <typename Vertex>
void print_vertex(Vertex v)
{
  std::cout << "-- V: " << v.descriptor()
    << ", H: " << v.property().height()
    << ", E: " << v.property().excess()
    << ", MSGS: " << v.property().msgs()
    << "." << std::endl;

  for (auto ii = v.begin(); ii != v.end(); ++ii)
    print_edge((*ii));
}

//////////////////////////////////////////////////////////////////////
/// @brief Functor used to print a message that is being sent.
/// @param msg Message to print
/// @param process Flag to indicate whether the message is an push
/// message or a reject message.
//////////////////////////////////////////////////////////////////////
template <typename Message>
void print_message(Message msg, int process)
{
  std::cout << "(" << msg.sender_vd() << ", " << msg.receiver_vd()
    << ") --> " << "ProcessFlag: " << process << ", Flow: "
    << msg.flow() << ", SendersHeight: " << msg.height()
    << std::endl;
}

//////////////////////////////////////////////////////////////////////
/// @brief Functor used to print flows of the graph.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct print_flows
{
  typedef void result_type;
  template <typename Vertex>
  result_type operator()(Vertex v)
  {

    for (auto const& e : v)
      print_edge(v.descriptor(), e);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Functor that adds backward edges on the flow network.
/// Backward edges are used to send excess flow backwards to source,
/// after flow is maximal at sink.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct add_backward_edges
{
  typedef void result_type;

  template <typename Vertex, typename GraphView>
  result_type operator()(Vertex v, GraphView gView)
  {
    for (auto const& e : v) {
      gView.add_edge(e.target(), v.descriptor(),
        stapl::properties::preflowpush_edge_property(0, true, 0));
    }
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Visitor functor for init graph.
///
/// Sets height and flow properties on vertices adjacent to sink of
/// flow network.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct update_init_graph
{
public:
  typedef stapl::properties::message message_type;

  message_type m_msg;

  update_init_graph(void)
    : m_msg()
  { }

  update_init_graph(message_type msg)
    : m_msg(msg)
  { }

  typedef bool result_type;
  template <typename Vertex>
  result_type operator ()(Vertex&& target) const
  {
    target.property().inc_ex(m_msg.flow());

    auto ii = target.begin();
    auto ee = target.end();
    for (; ii != ee; ++ii)
    {
      if (m_msg.sender_vd() == (*ii).target())
      {
        (*ii).property().height(m_msg.height());
        (*ii).property().inc_rc(m_msg.flow());
      }
    }
    return true;
  }

  void define_type(stapl::typer& t)
  { t.member(m_msg); }
};

//////////////////////////////////////////////////////////////////////
/// @brief Work function of graph paradigm used for setting up preflow
/// properties in edges and nodes.
///
/// @ingroup pgraphAlgoDetails
/// @tparam GraphView View of the graph.
//////////////////////////////////////////////////////////////////////
template <typename GraphView>
struct init_graph
{
public:
  size_t m_source;
  size_t m_sink;

  init_graph(size_t source=0, size_t sink=0)
    : m_source(source), m_sink(sink)
  { }

  typedef bool result_type;

  template <typename Vertex, typename GraphVisitor>
  result_type operator()(Vertex&& v, GraphVisitor&& graph_visitor) const
  {
    typedef stapl::properties::message message_type;

    if (v.descriptor() == m_source) {

      v.property().height(int(m_sink+1));
      v.property().excess(0);

      auto it = v.begin();
      auto ee = v.end();
      for (; it != ee; ++it) {
        auto amount = (*it).property().rc();
        (*it).property().dec_rc(amount);

        auto edge_tg = (*it).target();
        message_type msg(false, amount, v.descriptor(), edge_tg,
          v.property().height());

        graph_visitor.visit(edge_tg, update_init_graph(msg));
      }
    }
    return false;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_source);
    t.member(m_sink);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Function that updates the height to 1 + minimum height of
/// neighbors that can accept flow.
/// @param v Vertex that is performing the relabel operation.
/// @param num_vertices Number of vertices in the graph.
//////////////////////////////////////////////////////////////////////
template <typename Vertex>
void relabel(Vertex v, int num_vertices)
{
  int min_height = std::numeric_limits<int>::max();
  int min_edge = 0;

  int current_edge = 0;

  auto ee = v.end();
  for (auto ii = v.begin(); ii != ee; ++ii, ++current_edge) {

    auto target_height = (*ii).property().height();

    if ((*ii).property().rc() > 0 && target_height < min_height) {
      min_height = target_height;
      min_edge = current_edge;
    }
  }

  ++min_height;

  if (min_height < num_vertices) {
    v.property().height(min_height);
    v.property().stop(min_edge);
  }
  else {
    v.property().height(num_vertices);
  }
}

//////////////////////////////////////////////////////////////////////
/// @brief Function that updates the edge properties when a vertex
/// receives flow.
/// @param v Vertex on which the function is called.
/// @param sender_vd Vertex descriptor of the node that sent flow.
/// @param height Height of the of the node that sent flow.
/// @param flow Amount of flow being received.
//////////////////////////////////////////////////////////////////////
template <typename Vertex, typename VertexDescriptor>
void update_edge_data(Vertex v, VertexDescriptor sender_vd, int height,
  double flow)
{
  auto ee = v.end();
  for (auto ii = v.begin(); ii != ee; ++ii)
    if ((*ii).target() == sender_vd) {
      (*ii).property().height(height);
      (*ii).property().inc_rc(flow);
      break;
    }
}

//////////////////////////////////////////////////////////////////////
/// @brief Visitor functor for push relabel.
///
/// Evaluates the message sent by an active vertex. Either accepts or
/// rejects flow based on the height constraint.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct update_push
{
public:
  typedef stapl::properties::message message_type;

  message_type m_msg;
  int m_process;
  size_t m_source;
  size_t m_sink;

  update_push(void)
    : m_msg(), m_process(0), m_source(), m_sink()
  { }

  update_push(message_type msg, int process, size_t source, size_t sink)
    : m_msg(msg), m_process(process), m_source(source), m_sink(sink)
  { }

  typedef bool result_type;
  template <typename Vertex>
  result_type operator()(Vertex&& target) const
  {
    auto msg_height = m_msg.height();
    auto msg_flow = m_msg.flow();

    if (target.descriptor() != m_sink) {

      if (m_process == 0) {

        if (msg_height == target.property().height()+1) {

          if (target.descriptor() != m_source)
            target.property().inc_ex(msg_flow);

          update_edge_data<Vertex, message_type::vd_type>(target,
            m_msg.sender_vd(), msg_height, msg_flow);
        }
        else {
          message_type msg(false, msg_flow, target.descriptor(),
            m_msg.sender_vd(), target.property().height());

          target.property().add_msg(msg);
        }
      }
      else if (m_process == 1) {

        if (m_msg.response() == false) {

          if (target.descriptor() != m_source)
            target.property().inc_ex(msg_flow);

          update_edge_data<Vertex, message_type::vd_type>(target,
            m_msg.sender_vd(), msg_height, msg_flow);
        }
      }
      else if (m_process == 2) {
        auto ii = target.begin();
        auto ee = target.end();
        for (; ii != ee; ++ii) {
          if (m_msg.sender_vd() == (*ii).target())
            (*ii).property().height(msg_height);
        }
      }
    }
    else {
      target.property().inc_ex(msg_flow);

      update_edge_data<Vertex, message_type::vd_type>(target,
        m_msg.sender_vd(), msg_height, msg_flow);
    }
    return false;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_msg);
    t.member(m_process);
    t.member(m_source);
    t.member(m_sink);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Work function of push_relabel algorithm.
///
/// Pushes flow to admissible edges and initiates visits to neighboring
/// vertices who will become active on the next iteration of graph paradigm.
/// @ingroup pgraphAlgoDetails
/// @param m_num_vertices Number of vertices in the graph.
/// @param m_source Flow network source.
/// @param m_sink Flow network sink.
//////////////////////////////////////////////////////////////////////
struct push_relabel
{
public:
  size_t m_num_vertices;
  size_t m_source;
  size_t m_sink;

  push_relabel(size_t num_vertices=0, size_t source=0, size_t sink=0)
    : m_num_vertices(num_vertices), m_source(source), m_sink(sink)
  { }

  typedef bool result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param v Vertex that performs the work function.
  /// @param graph_visitor Mechanism used to visit neighboring vertices.
  //////////////////////////////////////////////////////////////////////
  template <typename Vertex, typename GraphVisitor>
  result_type operator()(Vertex&& v, GraphVisitor&& graph_visitor) const
  {
    if (v.descriptor() == m_source || v.descriptor() == m_sink)
        return false;

    typedef stapl::properties::message message_type;

    bool another_iteration = false;
    auto msgs = v.property().msgs();

    while (msgs) {
      auto msg = v.property().get_msg();

      graph_visitor.visit(msg.receiver_vd(),
                         update_push(msg, 1, m_source, m_sink));

      v.property().pop_msg();
      --msgs;
      another_iteration = true;
    }

    while (v.property().excess() > 0) {

      bool finished = false;

      int stop = v.property().stop();
      auto ii = v.begin();
      std::advance(ii, stop);
      auto ee = v.end();

      for (; ii != ee; ++ii, ++stop) {

        auto edge_cap = (*ii).property().rc();

        if (edge_cap == 0)
          continue;

        if (v.property().height()-1 != (*ii).property().height())
          continue;

        int amount = std::min(v.property().excess(), edge_cap);
        v.property().dec_ex(amount);
        (*ii).property().dec_rc(amount);

        message_type msg(false, amount, v.descriptor(), (*ii).target(),
          v.property().height());

        graph_visitor.visit((*ii).target(), update_push(msg, 0, m_source,
          m_sink));

        if (v.property().excess() == 0) {
          finished = true;
          another_iteration = true;
          v.property().stop(stop);
          break;
        }
      }

      if (finished)
        break;

      relabel(v, m_num_vertices);

      if (v.property().height() >= (int) m_num_vertices)
        break;

      message_type msg(false, 0, v.descriptor(), 0, v.property().height());
      graph_visitor.visit_all_edges(v, update_push(msg, 2, m_source, m_sink));
    }

    return another_iteration;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_num_vertices);
    t.member(m_source);
    t.member(m_sink);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Visitor functor for global relabel.
///
/// Updates the height of target vertex with BFS-parent and BFS-level
/// information, if the target vertex has not been visited before, or
/// if the target's height is greater than the incoming height.
/// Updates heights of neighbors stored in heights.
/// @tparam VD Type of the vertex-descriptor.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
class gr_update
{
public:
  int            m_parent;
  int            m_height;

  typedef bool result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param p The BFS-parent of the target vertex.
  /// @param level The BFS-level of the target vertex.
  //////////////////////////////////////////////////////////////////////
  gr_update(int parent = 0, int level = 0)
    : m_parent(parent), m_height(level)
  { }

  template <class Vertex>
  result_type operator()(Vertex&& target) const
  {
    typedef typename std::decay<Vertex>::type::vertex_descriptor descriptor_t;

    auto ii = target.begin();
    auto ee = target.end();
    for (; ii != ee; ++ii) {

      if ((*ii).target() == (descriptor_t)m_parent) {
        (*ii).property().height(m_height-1);
        break;
      }
    }

    if (target.property().height() == 0 ||
        target.property().height() > m_height) {
      target.property().height(m_height);
      return true;
    }

    return false;
  }

  void define_type(typer& t)
  {
    t.member(m_parent);
    t.member(m_height);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Work function for global relabel algorithm.
///
/// Sets heights of residual paths in BFS manner for flow to reach
/// sink in smallest number of hops.
/// @ingroup pgraphAlgoDetails
/// @param v A vertex in the graph.
/// @param graph_visitor Mechanism used to visit neighboring vertices.
//////////////////////////////////////////////////////////////////////
struct gr_wf
{
  typedef bool result_type;

  template<class Vertex, class GraphVisitor>
  result_type operator()(Vertex&& v, GraphVisitor&& graph_visitor) const
  {
    typedef typename Vertex::vertex_descriptor descriptor_t;

    if (v.property().height() == (int)graph_visitor.level()) {

      for (auto const& e : v) {
        if (e.property().backward() && e.property().rc() == 0) {
          graph_visitor.visit(e.target(),
            gr_update(v.descriptor(), v.property().height()+1));
        }
      }  // if v is active.

      return true;
    }

    return false;
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Sets heghts of vertices in BFS manner.
/// @ingroup pgraphAlgoDetails
/// @param gView View of the graph.
/// @param source BFS source.
/// @param async Maximum value of asynchrony allowed in each phase.
//////////////////////////////////////////////////////////////////////
template<class GView>
size_t global_relabel(GView& g,
                            typename GView::vertex_descriptor const& source,
                            size_t k=0)
{
  return graph_paradigm(gr_wf(), gr_update(), g); // , k);
}

} // preflow_push_impl

//////////////////////////////////////////////////////////////////////
/// @brief Initializes the properties of vertices and edges for
/// the preflow push algorithm, and calculates the max flow in the
/// graph.
///
/// @note This algorithm mutates the graph by adding reverse edges.
///
/// @ingroup pgraphAlgo
/// @param gView View of the graph.
/// @param source Source of the flow network.
/// @param sink Sink of the flow network.
/// @param async Level of asynchrony to use for the graph paradigm.
//////////////////////////////////////////////////////////////////////
template <typename GraphView>
stapl::tuple<double, double, double, double> preflow_push(GraphView& gView,
  size_t source, size_t sink, int async)
{
  using namespace preflow_push_impl;

  stapl::counter<stapl::default_timer> init_paradigm;
  stapl::counter<stapl::default_timer> pr_paradigm;
  stapl::counter<stapl::default_timer> global_relabel_time;

  map_func(add_backward_edges(), gView, stapl::make_repeat_view(gView));

  global_relabel_time.start();
  global_relabel(gView, sink);
  double gl_time = global_relabel_time.stop();

  init_paradigm.start();
  graph_paradigm(init_graph<GraphView>(source, sink),
                  update_init_graph(), gView);
  double init_time = init_paradigm.stop();

  pr_paradigm.start();
  graph_paradigm(push_relabel(gView.num_vertices(), source, sink),
    update_push(), gView, async);
  double pr_time = pr_paradigm.stop();

  return stapl::make_tuple(gl_time, init_time, pr_time,
    gView[sink].property().excess());
}

} // namespace stapl
