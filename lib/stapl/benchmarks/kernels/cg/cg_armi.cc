/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


//////////////////////////////////////////////////////////////////////
/// @file
/// Benchmark for nested parallelism pattern found in NAS CG using STAPL-RTS
/// primitives and custom terminator refining  @ref stapl::terminator_base.
//////////////////////////////////////////////////////////////////////

#include <stapl/runtime.hpp>
#include <stapl/runtime/executor/terminator.hpp>
#include <stapl/utility/do_once.hpp>
#include <iostream>
#include <cmath>

struct dummy_init_p_object
: public stapl::p_object
{
  int m_count;

  dummy_init_p_object(void)
    : m_count(0)
  { }

  void notify_complete(void)
  { ++m_count; }

  int termination_value(void)
  { return m_count; }
};


struct dummy_p_object
: public stapl::p_object
{
  dummy_p_object(stapl::pointer_wrapper<stapl::p_object> parent,
                 stapl::location_type parent_location)
  {
    if (this->get_location_id() == 0)
    {
      stapl_assert(parent != nullptr, "Failed parent pg handle resolution");
      stapl::gang g(*parent);
      stapl::async_rmi(parent_location, parent->get_rmi_handle(),
                       &dummy_init_p_object::notify_complete);
      stapl::rmi_flush();
    }
    delete this;
  }
};


template<typename T, typename BinaryOperation>
class my_terminator
  : public stapl::terminator_base
{
public:
  using value_type     = T;

private:
  // Using boost::function instead of std::function to avoid mallocs.
  using function_type  = boost::function<value_type(void)>;
  using allreduce_type =
    stapl::runtime::allreduce_object<value_type, BinaryOperation>;

  //////////////////////////////////////////////////////////////////////
  /// @brief Implements State A.
  //////////////////////////////////////////////////////////////////////
  struct state_a_wf
  {
    my_terminator& m_t;

    explicit state_a_wf(my_terminator& t) noexcept
    : m_t(t)
    { }

    void operator()(stapl::future<T> f)
    {
      if (f.get()==m_t.m_termination_value)
        m_t.restart(state_b_wf{m_t});
      else
        m_t.restart(state_a_wf{m_t});
    }
  };

  //////////////////////////////////////////////////////////////////////
  /// @brief Implements State B.
  //////////////////////////////////////////////////////////////////////
  struct state_b_wf
  {
    my_terminator& m_t;

    explicit state_b_wf(my_terminator& t) noexcept
    : m_t(t)
    { }

    void operator()(stapl::future<T> f)
    {
      if (f.get()==m_t.m_termination_value)
        m_t.stop();
      else
        m_t.restart(state_a_wf{m_t});
    }
  };


  const function_type             m_value_function;
  const value_type                m_termination_value;
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
  my_terminator(BinaryOperation op,
             ValueFunction&& value_fun,
             T const& termination_value = {},
             bool b_only_local = false)
    : m_value_function(std::forward<ValueFunction>(value_fun)),
      m_termination_value(termination_value),
      m_iterations(0),
      m_started(false)
  {
    using namespace stapl::runtime;

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
    if (m_allreducer) {
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
    if (m_value_function() == m_termination_value)
      call_notifier();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc terminator_base::iterations() const
  //////////////////////////////////////////////////////////////////////
  size_type iterations(void) const noexcept override
  { return m_iterations; }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  using timer_type = stapl::counter<stapl::default_timer>;

  const auto N                     = 100;
  const auto nlocs                 = stapl::get_num_locations();
  const decltype(nlocs) col_length = std::sqrt(nlocs);
  const auto row_width =
    col_length * col_length == nlocs ? col_length : col_length * 2;

  stapl_assert(row_width * col_length == nlocs,
    "# locations must be a square or 2*square");

  const bool row_creator = (stapl::get_location_id()%row_width == 0);

  stapl::rmi_handle::reference o;
  stapl::p_object_delete<dummy_init_p_object> d;

  // create row objects
  if (row_creator) {
    std::vector<stapl::location_type> locs;
    locs.reserve(row_width);
    for (auto i = stapl::get_location_id();
           i < (stapl::get_location_id() + row_width);
             ++i)
      locs.push_back(i);

    auto f = stapl::construct<dummy_init_p_object>(stapl::location_range(locs));
    o = f.get();
  }

  timer_type benchmark_timer;

  stapl::rmi_fence();

  benchmark_timer.start();

  for (int i = 0; i < N; ++i)
  {
    bool done = false;
    dummy_init_p_object outer_pg;

    auto terminator_ptr =
      new my_terminator<int,std::plus<int>>(
        std::plus<int>(),
        [&outer_pg] { return outer_pg.termination_value(); },
        col_length);

    terminator_ptr->set_notifier(
      [&done] { done = true; });

    outer_pg.advance_epoch();

    if (row_creator) {
       stapl::async_construct<dummy_p_object>(
         [](dummy_p_object*) { },
         o, stapl::all_locations,
         stapl::pointer(&outer_pg), outer_pg.get_location_id()
       );
    }

    terminator_ptr->operator()();

    stapl::block_until([&done] { return done; });

    if (!done)
      stapl::abort("It should have been done!!");

    delete terminator_ptr;
  }

  const double benchmark_elapsed = benchmark_timer.stop();

  // delete row objects
  if (row_creator) {
    d(o);
  }

  stapl::do_once([benchmark_elapsed] {
    std::cout << "Benchmark completed\n";
    std::cout << "Time in seconds = " << benchmark_elapsed << "\n";
  });

  return EXIT_SUCCESS;
}
