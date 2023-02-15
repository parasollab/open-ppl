/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_RUNTIME_EXECUTOR_TERMINATOR_HPP
#define STAPL_RUNTIME_EXECUTOR_TERMINATOR_HPP

#include "terminator_base.hpp"
#include "../collective/allreduce_object.hpp"
#include <functional>
#include <utility>
#include <boost/function.hpp>
#include <boost/optional.hpp>
#include <boost/utility/typed_in_place_factory.hpp>

namespace stapl {

void rmi_flush(void);


//////////////////////////////////////////////////////////////////////
/// @brief General termination detection.
///
/// @tparam T               Object type of the termination detection.
/// @tparam BinaryOperation Binary operation function object type to be applied.
///
/// This terminator does a reduction and compares the resulting value against a
/// known termination value. If they are the same, it calls the notify function.
///
/// It implements a phased termination consisting of 3 states:
/// -# State A: Do allreduce. If the return value is the same as the termination
///    value, goto State B, otherwise goto State A.
/// -# State B: Do allreduce. If the return value is the same as the termination
///    value, goto State C, otherwise goto State A.
/// -# State C: Termination detection succeeded.
///
/// @see terminator_base
/// @ingroup executors
///
/// @todo The termination detection should be correct if in State B we remain in
///       it if the allreduce has failed to converge, instead going to State A.
//////////////////////////////////////////////////////////////////////
template<typename T, typename BinaryOperation>
class terminator
  : public terminator_base
{
private:
  // Using boost::function instead of std::function to avoid mallocs.
  using function_type  = boost::function<T(void)>;
  using allreduce_type = runtime::allreduce_object<T, BinaryOperation>;

  //////////////////////////////////////////////////////////////////////
  /// @brief Implements State A.
  //////////////////////////////////////////////////////////////////////
  struct state_a_wf
  {
    terminator& m_t;

    explicit state_a_wf(terminator& t) noexcept
    : m_t(t)
    { }

    void operator()(future<T> f)
    {
      const auto val = f.get();

      if (val.first == 0)
      {
        // If no tasks added to any location escape that location,
        // then it is safe to terminate after one iteration.
        if (val.second)
          m_t.stop();
        // If tasks did escape, require a second, consecutive round
        // of a successful allreduce before terminating.
        else
          m_t.restart(state_b_wf{m_t});
      }
      else
        m_t.restart(state_a_wf{m_t});
    }
  };

  //////////////////////////////////////////////////////////////////////
  /// @brief Implements State B.
  //////////////////////////////////////////////////////////////////////
  struct state_b_wf
  {
    terminator& m_t;

    explicit state_b_wf(terminator& t) noexcept
    : m_t(t)
    { }

    void operator()(future<T> f)
    {
      const auto val = f.get();

      // We shouldn't arrive in this state if state_a_wf found no
      // escaped tasks.
      stapl_assert(!val.second, "Found no escaped tasks in state_b");

      if (val.first == 0)
        m_t.stop();
      else
        m_t.restart(state_a_wf{m_t});
    }
  };

  const function_type             m_value_function;
  boost::optional<allreduce_type> m_allreducer;
  size_type                       m_iterations;
  bool                            m_started;

public:
  //////////////////////////////////////////////////////////////////////
  /// @param op Binary operation used to aggregate results from multiple
  ///           locations.
  /// @param value_fun Function return value for a given location.
  /// @param termination_value The value that the aggregate of all locations
  ///        must be equal to for termination detection to succeed.
  /// @param b_local_only Runtime flag disabling use of global condition.
  ///        Each location independently terminates when its result of
  ///        @p value_fun equals @p termination_value.
  //////////////////////////////////////////////////////////////////////
  template<typename ValueFunction>
  terminator(BinaryOperation op,
             ValueFunction&& value_fun,
             bool b_only_local = false)
    : m_value_function(std::forward<ValueFunction>(value_fun)),
      m_iterations(0),
      m_started(false)
  {
    using namespace runtime;

    auto& ctx = this_context::get();
    if (ctx.get_gang_md().size() > 1 && !b_only_local)
    {
      STAPL_RUNTIME_CHECK(ctx.is_base(), "Only allowed when SPMD");
      m_allreducer =
        boost::in_place<allreduce_type>(std::ref(ctx), std::move(op));
    }
  }

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Stops the terminator.
  //////////////////////////////////////////////////////////////////////
  void stop(void)
  {
    STAPL_RUNTIME_ASSERT(m_started);
    m_started = false;
    call_notifier();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Restarts the terminator and in the next event it calls @p f.
  //////////////////////////////////////////////////////////////////////
  template<typename F>
  void restart(F&& f)
  {
    STAPL_RUNTIME_ASSERT(m_started);
    ++m_iterations;
    (*m_allreducer).async_then(std::forward<F>(f));
    (*m_allreducer)(m_value_function());
  }

public:
  void operator()(void) override
  {
    if (m_allreducer)
    {
      if (m_started)
        return;

      m_started = true;
      (*m_allreducer).async_then(state_a_wf{*this});
      (*m_allreducer)(m_value_function());
      m_iterations = 1;

      return;
    }

    // else
    // only one location or local_only flag was specified
    if (m_value_function().first == 0)
      call_notifier();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc terminator_base::iterations() const
  //////////////////////////////////////////////////////////////////////
  size_type iterations(void) const noexcept override
  { return m_iterations; }
};

} // namespace stapl

#endif
