/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_TEST_CONTAINERS_GRAPH_ALGORITHM_WRAPPERS_HPP
#define STAPL_TEST_CONTAINERS_GRAPH_ALGORITHM_WRAPPERS_HPP

#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/dynamic_graph.hpp>

#include <stapl/containers/graph/algorithms/mssp.hpp>
#include <stapl/containers/graph/algorithms/topological_sort.hpp>
#include <stapl/containers/graph/algorithms/pscc.hpp>
#include <stapl/containers/graph/algorithms/pscc_single.hpp>
#include <stapl/containers/graph/algorithms/pscc_schudy.hpp>
#include <stapl/containers/graph/algorithms/graph_coloring.hpp>
#include <stapl/containers/graph/views/property_maps.hpp>
#include <stapl/containers/graph/algorithms/properties.hpp>

#include <string>
#include <map>

//////////////////////////////////////////////////////////////////////
/// @brief Algorithm wrapper for @ref mssp.
//////////////////////////////////////////////////////////////////////
template<stapl::graph_attributes Directedness>
class mssp_algo_wrapper
{
  std::vector<size_t> m_sources;
  size_t m_num_sources;

public:
  typedef stapl::properties::mssp_property vertex_property;
  typedef double                           edge_property;
  typedef stapl::graph<Directedness,
                       stapl::NONMULTIEDGES,
                       vertex_property,
                       edge_property>                  graph_type;

  template<typename GraphView>
  mssp_algo_wrapper(GraphView const& graph, int argc, char** argv)
    : m_num_sources(3)
  {
    for (int i = 1; i < argc; i++) {
      if (!strcmp("--numsources", argv[i])) {
        m_num_sources = atoi(argv[i+1]);
      }
    }
    one_time_setup(graph);
  }

  template<typename GraphView>
  void one_time_setup(GraphView const& graph)
  {
    generate_sources(graph);

    if (!(m_num_sources > 1))
      stapl::abort(" More than 2 sources must be picked");
    if (!(m_sources.size() == m_num_sources))
      stapl::abort("Unable to find sources ");
  }

  template<typename GraphView>
  void generate_sources(GraphView const& graph)
  {
    typedef typename GraphView::vertex_descriptor vd_type;

    // Try to find n sources:
    for (size_t i = 0; i < m_num_sources; ++i)
    {
      vd_type current_source = rand() % graph.size();
      while (
        std::find(m_sources.begin(), m_sources.end(), current_source) !=
          m_sources.end() && graph[current_source].size() < 2)
        current_source = rand() % graph.size();

      m_sources.push_back(current_source);

      if (stapl::get_location_id()==0) {
        std::cout << "> Source #"<< i <<": "
          << current_source << "\n" << std::endl;
      }
    }

    stapl::rmi_fence();

  }

  std::string name(void) const
  { return "MSSP"; }

  template<typename GraphView>
  void initialize(GraphView const& graph) const
  { }

  template<typename GraphView>
  size_t run(GraphView& graph, size_t k) const
  { return stapl::mssp(graph, m_sources, k); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Algorithm wrapper for Topological sorting.
//////////////////////////////////////////////////////////////////////
template<stapl::graph_attributes Directedness>
class topsort_algo_wrapper
{

public:
  typedef stapl::properties::topological_sort_property vertex_property;
  typedef stapl::properties::no_property               edge_property;
  typedef stapl::graph<Directedness,
                       stapl::NONMULTIEDGES,
                       vertex_property,
                       edge_property>                  graph_type;

  template<typename GraphView>
  topsort_algo_wrapper(GraphView const& graph, int argc, char** argv)
  { }

  std::string name(void) const
  { return "TopSort"; }

  template<typename GraphView>
  void initialize(GraphView const& graph) const
  { }

  template<typename GraphView>
  size_t run(GraphView& graph, size_t k) const
  {
    stapl::topological_sort(graph, k);
    return 0;
  }
};


namespace pscc_algo_wrapper_details{
  enum class scc_type {
    pscc,
    pscc_single,
    pscc_schudy
  };
}

//////////////////////////////////////////////////////////////////////
/// @brief Algorithm wrapper for Strongly Connected Components
//////////////////////////////////////////////////////////////////////
template<stapl::graph_attributes Directedness>
class pscc_algo_wrapper
{
  typedef pscc_algo_wrapper_details::scc_type scc_type;
private:
  scc_type m_algo_type;
public:
  typedef stapl::properties::scc_property<size_t>      vertex_property;
  typedef stapl::properties::no_property               edge_property;
  typedef stapl::graph<Directedness,
                     stapl::NONMULTIEDGES,
                     vertex_property,
                     edge_property>                  graph_type;

  template<typename GraphView>
   pscc_algo_wrapper(GraphView const& graph, int argc, char** argv)
     : m_algo_type(scc_type::pscc)
  {
    std::string str_scc_type = "pscc";
    std::string str_pscc = "pscc";
    std::string str_pscc_single = "pscc_single";
    std::string str_pscc_schudy = "pscc_schudy";
    for (int i = 1; i < argc; i++) {
      if (!strcmp("--scc_type", argv[i]))
      str_scc_type = argv[i+1];
    }
    if (str_scc_type == str_pscc)
      m_algo_type = scc_type::pscc;
    else if (str_scc_type == str_pscc_single)
      m_algo_type = scc_type::pscc_single;
    else if (str_scc_type == str_pscc_schudy)
      m_algo_type = scc_type::pscc_schudy;
    else
      stapl::abort(" Unknown SCC algorithm");
  }


  std::string name(void) const
  {
    std::string res_name = "SCC" ;
    if (m_algo_type == scc_type::pscc_single)
      res_name+= " single";
    else if (m_algo_type == scc_type::pscc_schudy)
      res_name+= " schudy";
    return res_name;
  }

  template<typename GraphView>
  void initialize(GraphView const& graph) const
  { }

  template<typename GraphView>
  size_t run(GraphView& graph, size_t k) const
  {
    switch(m_algo_type) {
      case scc_type::pscc:
        stapl::pscc(graph);
        break;
      case scc_type::pscc_single:
        stapl::pscc_single(graph);
        break;
      case scc_type::pscc_schudy:
        stapl::pscc_schudy(graph);
        break;
    }
    return 0;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_algo_type);
  }
};

namespace color_algo_wrapper_details{
class access_color
{
public:
  typedef size_t value_type;

  template <typename VProperty>
  value_type get(VProperty const& vp)
  {
    return vp.get_color();
  }

  template <typename VProperty>
  void put(VProperty vp, value_type const& color)
  {
    return vp.set_color(color);
  }
};
}

//////////////////////////////////////////////////////////////////////
/// @brief Algorithm wrapper for Graph coloring.
//////////////////////////////////////////////////////////////////////
template<stapl::graph_attributes Directedness>
class coloring_algo_wrapper
{

public:
  typedef stapl::properties::coloring_property        vertex_property;
  typedef stapl::properties::no_property              edge_property;

  typedef stapl::dynamic_graph<Directedness,
                               stapl::NONMULTIEDGES,
                               vertex_property,
                               edge_property>         graph_type;
  typedef stapl::graph_view<graph_type> graph_view_t;

  typedef stapl::graph_internal_property_map<
    graph_view_t,
    color_algo_wrapper_details::access_color> ipm_type;

private:
  ipm_type m_pmap;

public:

  template<typename GraphView>
  coloring_algo_wrapper(GraphView & graph, int argc, char** argv)
     : m_pmap(graph)
  { }


  std::string name(void) const
  { return "Coloring"; }

  template<typename GraphView>
  void initialize(GraphView const& graph) const
  { }

  template<typename GraphView>
  size_t run(GraphView & graph, size_t k)
  {
    stapl::color_graph(graph, m_pmap);
    return 0;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_pmap);
  }
};

#endif
