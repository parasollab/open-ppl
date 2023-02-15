/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_ENVIRONMENTS_COMBINED_ENV_HPP
#define STAPL_SKELETONS_ENVIRONMENTS_COMBINED_ENV_HPP

#include <type_traits>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/utility/tuple/tuple.hpp>

namespace stapl {
namespace skeletons {

//////////////////////////////////////////////////////////////////////
/// @brief A @c combined_env is a collection of different environments.
/// When a skeleton is spawned in a @c combined_env, the
/// @c spawn_element or @c set_num_succs requests are passed down to
/// each environment.
///
/// A typical usage of a @c combined_env is to create tasks in a
/// @c taskgraph_env and create GraphViz graphs using @c graphviz_env.
///
/// @tparam Envs the list of enclosed environments
///
/// @ingroup skeletonsEnvironments
//////////////////////////////////////////////////////////////////////
template <typename... Envs>
class combined_env
{
  stapl::tuple<Envs...> m_envs;
public:
  using envs_type     = stapl::tuple<Envs...>                               ;
  using num_envs_type = std::integral_constant<int, sizeof...(Envs)> ;

  explicit combined_env(Envs const&... envs)
    : m_envs(envs...)
  { }

  std::size_t get_num_PEs() const
  {
    return stapl::get<0>(m_envs).get_num_PEs();
  }

  std::size_t get_PE_id() const
  {
    return stapl::get<0>(m_envs).get_PE_id();
  }

  template <typename... Args>
  void init_location_info(Args&&... args)
  {
    apply_init_location_info(std::integral_constant<int, sizeof...(Envs)-1>(),
                             std::forward<Args>(args)...);
  }

  template <bool isResult, typename... Args>
  void spawn_element(Args&&... args)
  {
    apply_spawn_element<isResult>(
      std::integral_constant<int, sizeof...(Envs)-1>(),
      std::forward<Args>(args)...);
  }

  template <typename... Args>
  void pre_spawn(Args&&... args)
  {
    apply_pre_spawn(std::integral_constant<int, sizeof...(Envs)-1>(),
                    std::forward<Args>(args)...);
  }

  template <typename... Args>
  void post_spawn(Args&&... args)
  {
    apply_post_spawn(std::integral_constant<int, sizeof...(Envs)-1>(),
                     std::forward<Args>(args)...);
  }

  template <typename... Args>
  void set_num_succs(Args&&... args)
  {
    apply_set_num_succs(std::integral_constant<int, sizeof...(Envs)-1>(),
                        std::forward<Args>(args)...);
  }

private:
  template <typename... Args>
  void apply_init_location_info(std::integral_constant<int, 0>, Args&&... args)
  {
    stapl::get<0>(m_envs).init_location_info(std::forward<Args>(args)...);
  }

  template <int i, typename... Args>
  void apply_init_location_info(std::integral_constant<int, i>, Args&&... args)
  {
    apply_init_location_info(std::integral_constant<int, i-1>(),
                             std::forward<Args>(args)...);
    stapl::get<i>(m_envs).init_location_info(std::forward<Args>(args)...);
  }

  template <bool isResult, typename... Args>
  void
  apply_spawn_element(std::integral_constant<int, 0>, Args&&... args)
  {
    stapl::get<0>(m_envs).template spawn_element<isResult>(
                                     std::forward<Args>(args)...);
  }

  template <bool isResult, int i, typename... Args>
  typename std::enable_if<(i > 0), void>::type
  apply_spawn_element(std::integral_constant<int, i>, Args&&... args)
  {
    apply_spawn_element<isResult>(std::integral_constant<int, i-1>(),
                                  std::forward<Args>(args)...);
    stapl::get<i>(m_envs).template spawn_element<isResult>(
                                     std::forward<Args>(args)...);
  }

  template <typename... Args>
  void
  apply_pre_spawn(std::integral_constant<int, 0>, Args&&... args)
  {
    stapl::get<0>(m_envs).pre_spawn(std::forward<Args>(args)...);
  }

  template <int i, typename... Args>
  typename std::enable_if<(i > 0), void>::type
  apply_pre_spawn(std::integral_constant<int, i>, Args&&... args)
  {
    apply_pre_spawn(std::integral_constant<int, i-1>(),
                    std::forward<Args>(args)...);
    stapl::get<i>(m_envs).pre_spawn(std::forward<Args>(args)...);
  }

  template <typename... Args>
  void
  apply_post_spawn(std::integral_constant<int, 0>, Args&&... args)
  {
    stapl::get<0>(m_envs).post_spawn(std::forward<Args>(args)...);
  }

  template <int i, typename... Args>
  typename std::enable_if<(i > 0), void>::type
  apply_post_spawn(std::integral_constant<int, i>, Args&&... args)
  {
    apply_post_spawn(std::integral_constant<int, i-1>(),
                     std::forward<Args>(args)...);
    stapl::get<i>(m_envs).post_spawn(std::forward<Args>(args)...);
  }

  template <typename... Args>
  void
  apply_set_num_succs(std::integral_constant<int, 0>, Args&&... args)
  {
    stapl::get<0>(m_envs).set_num_succs(std::forward<Args>(args)...);
  }

  template <int i, typename... Args>
  typename std::enable_if<(i > 0), void>::type
  apply_set_num_succs(std::integral_constant<int, i>, Args&&... args)
  {
    apply_set_num_succs(std::integral_constant<int, i-1>(),
                        std::forward<Args>(args)...);
    stapl::get<i>(m_envs).set_num_succs(std::forward<Args>(args)...);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief A helper to created combined environments. Combined environments
/// are used to spawn skeletons in various environments at the same time
/// (e.g., a @c taskgraph_env and a @c graphviz_env at the same time).
///
/// @ingroup skeletonsEnvironments
//////////////////////////////////////////////////////////////////////
template <typename... Envs>
combined_env<Envs...>
make_combined_env(Envs... envs)
{
  return combined_env<Envs...>(envs...);
}

} // namespace skeletons
} // namespace stapl
#endif // STAPL_SKELETONS_ENVIRONMENTS_COMBINED_ENV_HPP
