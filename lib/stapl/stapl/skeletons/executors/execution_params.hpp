/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_EXECUTORS_EXECUTION_PARAMS_HPP
#define STAPL_SKELETONS_EXECUTORS_EXECUTION_PARAMS_HPP

#include <stapl/runtime/executor/scheduler/sched.hpp>
#include <stapl/views/metadata/coarseners/null.hpp>

namespace stapl {
namespace skeletons {
namespace skeletons_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Stores the execution parameters to be used for running a
/// a skeleton in STAPL. These parameters are currently:
/// @tparam ResultType the result type of the given skeleton if any.
/// @tparam isBlocking whether the execution should be blocking or not.
/// @tparam OptionalParams... the following optional parameters can
///                    be passed:
///                    1. Coarsener - data coarsener to be used
///                    2. ExtraEnv  - an extra spawning environment to
///                       be used in addition to the default @c taskgraph_env
///                    3. Scheduler - scheduler to be used for the generated
///                       taskgraph.
///
/// @ingroup skeletonsExecutors
//////////////////////////////////////////////////////////////////////
template <typename ResultType, bool isBlocking,
          typename... OptionalParams>
class execution_params
{
protected:
  using default_scheduler_type = default_scheduler;
  using default_param_types    = tuple<
                                   null_coarsener,     // coarsener
                                   stapl::use_default, // extra_env
                                   default_scheduler>; // scheduler
  using param_types            = typename compute_type_parameters<
                                   default_param_types,
                                   OptionalParams...>::type;
public:
  static constexpr bool is_blocking = isBlocking;

  using result_type    = ResultType;
  using coarsener_type = typename tuple_element<0, param_types>::type;
  using extra_env_type = typename tuple_element<1, param_types>::type;
  using scheduler_type = typename tuple_element<2, param_types>::type;

  execution_params(coarsener_type const& coarsener,
                   extra_env_type const& extra_env,
                   scheduler_type const& scheduler)
    : m_coarsener(coarsener),
      m_scheduler(scheduler),
      m_extra_env(extra_env)
  { }

  execution_params(coarsener_type const& coarsener,
                   extra_env_type const& extra_env)
    : execution_params(coarsener, extra_env, scheduler_type())
  { }

  execution_params(coarsener_type const& coarsener)
    : execution_params(coarsener, extra_env_type(), scheduler_type())
  { }

  execution_params()
    : execution_params(coarsener_type(), extra_env_type(), scheduler_type())
  { }

  coarsener_type const& coarsener() const
  { return m_coarsener; }

  scheduler_type const& scheduler() const
  { return m_scheduler; }

  extra_env_type const& extra_env() const
  { return m_extra_env; }

  void define_type(typer& t)
  {
    t.member(m_coarsener);
    t.member(m_scheduler);
    t.member(m_extra_env);
  }

private:
  coarsener_type m_coarsener;
  scheduler_type m_scheduler;
  extra_env_type m_extra_env;
};

//////////////////////////////////////////////////////////////////////
/// @brief Default execution parameters.
///
/// This class reduces the symbol size for default cases.
//////////////////////////////////////////////////////////////////////
struct default_execution_params
  : public execution_params<void, false>
{ };

}  // namespace skeletons_impl

//////////////////////////////////////////////////////////////////////
/// @brief Creates an @c execution_params instance with user provided
/// values. All the unprovided arguments will be assigned default values.
///
/// @tparam ResultType the return type of skeleton which is used in
///                    reducing skeletons such as @c reduce, @c map_reduce.
/// @tparam isBlocking whether the execution needs to be blocking or not.
/// @param OptionalParams... the following optional parameters can
///                    be passed:
///                    1. Coarsener - data coarsener to be used
///                    2. ExtraEnv  - an extra spawning environment to
///                       be used in addition to the default @c taskgraph_env
///                    3. Scheduler - scheduler to be used for the generated
///                       taskgraph.
//////////////////////////////////////////////////////////////////////
template <typename ResultType =
            skeletons_impl::default_execution_params::result_type,
          bool isBlocking     =
            skeletons_impl::default_execution_params::is_blocking,
          typename... OptionalParams>
skeletons_impl::execution_params<
  ResultType, isBlocking,
  typename std::decay<OptionalParams>::type...>
execution_params(OptionalParams&&... optional_params) {
  return skeletons_impl::execution_params<
           ResultType, isBlocking,
           typename std::decay<OptionalParams>::type...>(
             std::forward<OptionalParams>(optional_params)...);
}

//////////////////////////////////////////////////////////////////////
/// @brief Creates an @c execution_params instance with default values.
//////////////////////////////////////////////////////////////////////
inline skeletons_impl::default_execution_params
default_execution_params()
{
  return skeletons_impl::default_execution_params();
}

}  // namespace skeletons
}  // namespace stapl

#endif  // STAPL_SKELETONS_EXECUTORS_EXECUTION_PARAMS_HPP
