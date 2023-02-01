/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/runtime/random_location_generator.hpp>
#include <stapl/paragraph/paragraph.hpp>
#include <stapl/utility/do_once.hpp>
#include <stapl/paragraph/factory_wf.hpp>

#include <algorithm>
#include <type_traits>

using namespace stapl;


//////////////////////////////////////////////////////////////////////
/// @brief Task placement policy that intercepts factory workfunctions
///   (i.e., nested paragraph invocations) and returns a random number
///   of randomly chosen locations in the parent paragraph's communication
///   group.
//////////////////////////////////////////////////////////////////////
class random_pg_placement_strategy
{
private:
  random_location_generator m_rng;

  std::vector<unsigned int>
  random_locations(unsigned int nlocs)
  {
    std::vector<unsigned int> v;

    v.reserve(nlocs);

    for (unsigned int i = 0; i < nlocs; ++i)
    {
      unsigned int lid = 0;

      do {
        lid = m_rng();
      } while (std::find(v.begin(), v.end(), lid) != v.end());

      v.push_back(lid);
    }

    auto iter = std::find(v.begin(), v.end(), get_location_id());

    // If I've selected the current location in the parent pg, make it
    // first.
    if (iter != v.end() && iter != (v.end() - 1))
      std::iter_swap(v.end() - 1, iter);

    return v;
  }

public:
 random_pg_placement_strategy(unsigned int seed)
    : m_rng(seed)
  { }

  // If it's not a factory (i.e., nested pg invocation), forward to the
  // default task placement policy.
  template<typename WF, typename ...Views>
  typename std::enable_if<!is_factory<WF>::value, locality_info>::type
  execution_location(WF const& wf, Views const&... views)
  { return default_task_placement()(wf, views...); }

  // If it's a nested pg, place it on a random number of randomly chosen
  // locations in the parent's communication group.
  template<typename WF, typename ...Views>
  typename std::enable_if<
    is_factory<WF>::value,
    std::pair<rmi_handle::reference, std::vector<unsigned int>>
  >::type
  execution_location(WF const& wf, Views const&... views)
  {
    return std::make_pair(rmi_handle::reference(),
                          random_locations(m_rng() + 1));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return priority of the PARAGRAPH being scheduled.
  ///
  /// Method required by paragraph::operator() when the executor for the
  /// paragraph is added to the parent executor.
  //////////////////////////////////////////////////////////////////////
  default_info get_sched_info(void) noexcept
  { return default_info(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Hook to allow tracking of when tasks are executed.
  //////////////////////////////////////////////////////////////////////
  void notify_finished(void) noexcept
  { }

  void define_type(typer &t)
  {
    typedef stapl::counter<stapl::default_timer> counter_t;
    counter_t timer;
    timer.start();

    t.transient(m_rng, timer.stop());
  }
};


class inner_factory
  : public factory_wf
{
private:
  affinity_tag         m_parent_affinity;

public:
  typedef void result_type;

  inner_factory(affinity_tag parent_affinity)
    : m_parent_affinity(parent_affinity)
  { }

  template<typename TGV>
  void operator()(TGV const& tgv) const
  {
    if (tgv.graph().get_location_id() != tgv.graph().get_num_locations() - 1)
      stapl_assert(get_affinity() != m_parent_affinity,
        "inner factory: unexpected match to parent's affinity");
  }

  void define_type(typer& t)
  {
    t.member(m_parent_affinity);
  }
};


class outer_factory
  : public factory_wf
{
private:
  size_t m_num_inner_pgs;
  size_t m_iter;

public:
  typedef void result_type;

  outer_factory(size_t num_inner_pgs, size_t iter)
    : m_num_inner_pgs(num_inner_pgs), m_iter(iter)
  { }

  template<typename TGV>
  void operator()(TGV const& tgv)
  {
    const size_t loc_id = tgv.graph().get_location_id();

    // Spawn a number of one-sided paragraph on n_locs / 2 arbitrary
    // locations in this outer paragraph's gang.
    for (size_t idx = 0; idx < m_num_inner_pgs; ++idx)
    {
      const size_t task_id   = idx * (loc_id + 1);
      const size_t num_succs = 0;

      inner_factory f(get_affinity());

      tgv.add_task(task_id, f, num_succs);
    }
  }

  void define_type(typer& t)
  {
    t.member(m_num_inner_pgs);
    t.member(m_iter);
  }
};


struct my_persistent_scheduler
  : public fifo_scheduler<random_pg_placement_strategy>
{
  typedef void enable_persistence;

  template<typename ...Args>
  my_persistent_scheduler(Args&&... args)
    : fifo_scheduler(std::forward<Args>(args)...)
  { }
};


exit_code stapl_main(int, char*[])
{
  const location_type loc_id = get_location_id();
  const location_type n_locs = get_num_locations();

  // Number of paragraph spawned per location of the outer factory.
  const int num_inner_pgs    = 10;

  stapl::do_once([&] {
    std::cout << "Ephemeral paragraph nested one-sided creation with "
              << n_locs << " locations... ";
  });

  typedef fifo_scheduler<random_pg_placement_strategy>   scheduler1_type;
  typedef paragraph<scheduler1_type, outer_factory>      paragraph1_t;

  for (int i = 0; i < 100; ++i)
  {
    outer_factory f(num_inner_pgs, i);

    const unsigned int seed = (loc_id + 1) + i;

    scheduler1_type sched(seed);

    paragraph1_t pg(f, sched);

    pg();
  }

  stapl::do_once([] { std::cout << "Passed\n"; });

  stapl::do_once([&] {
    std::cout << "Persistent paragraph nested one-sided creation with "
              << n_locs << " locations... ";
  });

  typedef my_persistent_scheduler                        scheduler2_type;
  typedef paragraph<scheduler2_type, outer_factory>      paragraph2_t;

  for (int i = 0; i < 100; ++i)
  {
    outer_factory f(num_inner_pgs, i);

    const unsigned int seed = (loc_id + 1) + i;

    scheduler2_type sched(seed);

    paragraph2_t pg(f, sched);

    for (int j = 0; j < 10; ++j)
    {
      pg();
    }
  }

  stapl::do_once([] { std::cout << "Passed\n"; });

  return EXIT_SUCCESS;
}
