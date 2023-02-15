/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_TRANSFORMATIONS_COMPOSE_FUSED_WF_HPP
#define STAPL_SKELETONS_TRANSFORMATIONS_COMPOSE_FUSED_WF_HPP

#include <stapl/utility/tuple.hpp>
#include <stapl/utility/pack_ops.hpp>
namespace stapl {
namespace skeletons {
namespace transformations {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief A dummy value to represent to stand in for void results
/// in @see fused_wf.
//////////////////////////////////////////////////////////////////////
  struct dummy_value {};
} // namespace detail

//////////////////////////////////////////////////////////////////////
/// @brief A work function that executes a dag of other work functions
/// with the dependencies for each specified by an index_sequence.
///
/// @note This requires that the work functions are topologically sorted.
//////////////////////////////////////////////////////////////////////
template<class Wfs, class DepIndices>
struct fused_wf;

template<typename... Wfs, typename... DepIndices>
struct fused_wf<tuple<Wfs...>, tuple<DepIndices...>>
{
  static_assert(sizeof...(Wfs) > 1, "Need at least two work functions");
  static_assert(sizeof...(Wfs) == sizeof...(DepIndices),
      "Mismatch in number of work functions and dependencies");

private:
  tuple<Wfs...> m_wfs;

  template<size_t N>
  using dependencies = typename pack_ops::pack_element<N, DepIndices...>::type;

  static constexpr size_t last_wf_index = sizeof...(Wfs) - 1;
  using last_wf_index_t = std::integral_constant<size_t, last_wf_index>;

  template<size_t I, class Args,
    class = typename pack_ops::pack_element<I, DepIndices...>::type>
  struct result_of_wf_step;

  template<size_t I, class Args, size_t... Is>
  struct result_of_wf_step<I, Args, index_sequence<Is...>>
    : boost::result_of<typename pack_ops::pack_element<I, Wfs...>::type(
                                  typename tuple_element<Is, Args>::type...)>
  { };

  template<size_t I, class... Args>
  struct result_of_wf_from
    : result_of_wf_from<I+1, Args...,
        typename result_of_wf_step<I, tuple<Args...>>::type>
  { };

  template<class... Args>
  struct result_of_wf_from<last_wf_index + 1, Args...>
    : pack_ops::pack_element<sizeof...(Args) - 1, Args...>
  { };


  // Compute the result for the given wf now that we have its dependencies
  template<class Wf, std::size_t... Is, class... Args>
  typename boost::result_of<
    Wf(typename pack_ops::pack_element<Is, Args...>::type...)>::type
  compute(Wf&& wf, index_sequence<Is...>, Args&&... args)
  {
    return wf(pack_ops::get<Is>(args...)...);
  }

  // Base case, now compute and return the last result
  template<class Result, class... Args>
  Result call(std::true_type, last_wf_index_t, Args&&... args)
  {
    constexpr size_t I = last_wf_index;
    return this->compute(get<I>(m_wfs), dependencies<I>{}, args...);
  }

  // Compute the i'th work function's result and continue traversing the dag.
  template<class Result, size_t I, class... Args>
  Result call(std::false_type, std::integral_constant<size_t, I>,
              Args&&... args)
  {
    return this->call<Result>(
      std::integral_constant<bool, I+1 == last_wf_index>{},
      std::integral_constant<size_t, I+1>{}, args...,
      this->compute(get<I>(m_wfs), dependencies<I>{}, args...));
  }

public:

  template <typename R>
  struct result;

  template <typename F, typename... Args>
  struct result<F(Args...)>
    : result_of_wf_from<0, Args...>
  { };

  template<typename... WfRefs>
  fused_wf(WfRefs&&... wfs) : m_wfs(std::forward<WfRefs>(wfs)...)
  { }

  template<class... Args>
  typename result_of_wf_from<0, Args...>::type operator()(Args&&... args)
  {
    using result_type = typename result_of_wf_from<0, Args...>::type;
    return this->call<result_type>(
      std::false_type{},
      std::integral_constant<size_t, 0>{},
      std::forward<Args>(args)...
    );
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_wfs);
  }
};

template<class Deps, class... Wfs>
fused_wf<tuple<typename std::decay<Wfs>::type...>, Deps>
make_fused_wf(Wfs&&... wfs)
{
  return { std::forward<Wfs>(wfs)... };
}

} // namespace transformations
} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_TRANSFORMATIONS_COMPOSE_FUSED_WF_HPP
