/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_EXECUTION_POLICY_HPP
#define STAPL_CONTAINERS_EXECUTION_POLICY_HPP

#include <stapl/containers/graph/algorithms/create_level_machine.hpp>
#include <stapl/containers/graph/algorithms/create_level_hubs.hpp>

#include <boost/variant/variant.hpp>

namespace stapl {

namespace sgl {

//////////////////////////////////////////////////////////////////////
/// @brief Graph execution policy to execute a traversal using the
///        k-level-asynchronous strategy.
//////////////////////////////////////////////////////////////////////
class kla_policy
{
  size_t m_k;

public:
  kla_policy(size_t k = 0)
    : m_k(k)
  { }

  size_t k() const
  {
    return m_k;
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Graph execution policy to execute a traversal level-synchronously.
//////////////////////////////////////////////////////////////////////
class level_sync_policy
  : public kla_policy
{
  using base_type = kla_policy;
public:
  level_sync_policy() = default;
};

//////////////////////////////////////////////////////////////////////
/// @brief Graph execution policy to execute a traversal asynchronously.
//////////////////////////////////////////////////////////////////////
class async_policy
  : public kla_policy
{
  using base_type = kla_policy;
public:
  async_policy()
    : base_type(std::numeric_limits<size_t>::max()-1)
  { }
};


//////////////////////////////////////////////////////////////////////
/// @brief Graph execution policy to use the hierarchical strategy
///        for out-degree optimization for vertices with high degree.
///
/// @tparam View The input graph view
//////////////////////////////////////////////////////////////////////
template<typename View>
class hierarchical_policy
  : public kla_policy
{
  using base_type = kla_policy;
  using h_graph_type = digraph<
    create_level_machine_detail::super_vertex_machine_property<View>
  >;

public:
  using h_view_type = graph_view<h_graph_type>;

private:
  h_view_type m_h_view;

public:
  hierarchical_policy(h_view_type h_view)
    : m_h_view(std::move(h_view))
  { }

  h_view_type& h_view()
  {
    return m_h_view;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Graph execution policy to use the 2-level hierarchical strategy
///        for out-degree optimization.
///
/// @tparam View The input graph view
//////////////////////////////////////////////////////////////////////
template<typename View>
class hierarchical2_policy
  : public hierarchical_policy<View>
{
  using base_type = hierarchical_policy<View>;
  using h2_graph_type = digraph<
    create_level_machine_detail::super_vertex_machine_property<
      typename base_type::h_view_type>
    >;
  using h2_view_type = graph_view<h2_graph_type>;

  h2_view_type m_h2_view;

public:
  template<typename HView>
  hierarchical2_policy(HView h_view, h2_view_type h2_view)
    : base_type(std::move(h_view)),
      m_h2_view(std::move(h2_view))
  { }

  h2_view_type& h2_view()
  {
    return m_h2_view;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Graph execution policy to use the hierarchical strategy
///        for out-degree optimization as well as the hubs strategy
///        for in-degree optimization.
///
/// @tparam View The input graph view
//////////////////////////////////////////////////////////////////////
template<typename View>
class hierarchical_hubs_policy
  : public hierarchical_policy<View>
{
  using base_type = hierarchical_policy<View>;
  using hubs_graph_type = digraph<
    create_level_hubs_detail::super_vertex_hub_property<View>
  >;
  using hubs_view_type = graph_view<hubs_graph_type>;

  hubs_view_type m_hubs_view;
  std::size_t m_hub_degree;

public:
  template<typename HView>
  hierarchical_hubs_policy(HView h_view, hubs_view_type hubs_view,
                           std::size_t hub_degree)
    : base_type(std::move(h_view)),
      m_hubs_view(std::move(hubs_view)),
      m_hub_degree(hub_degree)
  { }

  hubs_view_type& hubs_view()
  {
    return m_hubs_view;
  }

  std::size_t hub_degree() const
  {
    return m_hub_degree;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief An execution policy for a graph algorithm's traversal.
///
/// @tparam View The input graph view
//////////////////////////////////////////////////////////////////////
template<typename View>
using execution_policy = boost::variant<
  level_sync_policy,
  kla_policy,
  async_policy,
  hierarchical_policy<View>,
  hierarchical_hubs_policy<View>,
  hierarchical2_policy<View>
>;


//////////////////////////////////////////////////////////////////////
/// @brief A helper to create execution policies for a given graph.
///
/// @param vw The input graph view
/// @param paradigm The paradigm to execute the graph algorithm
/// (lsync,async,kla,hier,hier2,hubs)
/// @param tunable_param A tunable parameter for use by the paradigm selected
/// e.g. 'k' in KLA, #hubs in hubs.
//////////////////////////////////////////////////////////////////////
template<typename View>
execution_policy<View> make_execution_policy(
    const std::string& paradigm, View& vw, size_t tunable_param = 0)
{
  if (paradigm == "hier") {
    auto h = create_level_machine(vw);
    return hierarchical_policy<View>{h};
  } else if (paradigm == "hier2") {
    auto h = create_level_machine(vw);
    auto h2 = create_level2_machine(h);
    return hierarchical2_policy<View>{h, h2};
  } else if (paradigm == "hubs") {
    const size_t hubs_degree = tunable_param;
    auto h = create_level_machine(vw);
    auto hubs = create_level_hubs(vw, hubs_degree);
    return hierarchical_hubs_policy<View>{h, hubs, hubs_degree};
  } else if (paradigm == "lsync") {
    return level_sync_policy{};
  } else if (paradigm == "async") {
    return async_policy{};
  } else if (paradigm == "kla") {
    const size_t k = tunable_param;
    return kla_policy{k};
  } else {
    stapl::abort("Unknown paradigm type " + paradigm + "\n");
    return level_sync_policy{};
  }
}

} // namespace sgl

} // stapl namespace

#endif
