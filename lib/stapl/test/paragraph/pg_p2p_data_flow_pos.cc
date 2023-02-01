/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/runtime/location_specific_storage.hpp>
#include <stapl/paragraph/paragraph.hpp>
#include <stapl/utility/tuple.hpp>
#include <stapl/containers/array/array.hpp>
#include <stapl/views/array_view.hpp>
#include <test/algorithms/test_utils.h>
#include <stapl/skeletons/spans/balanced.hpp>

using namespace stapl;

using flow_type = std::vector<int>;
using timer_type = counter<default_timer>;

location_specific_storage<std::vector<std::vector<int>>> vals;


class pg_placement_strategy
{
private:
  using coarsener_type = null_coarsener;
  using domain_type    = indexed_domain<std::size_t>;
  using partition_type = balanced_partition<domain_type>;

  partition_type m_sections;

public:
  pg_placement_strategy(size_t num_sections)
    : m_sections(domain_type(get_num_locations()), num_sections)
  { }

  template<typename WF, typename ...Views>
  typename std::enable_if<!is_factory<WF>::value, locality_info>::type
  execution_location(WF const& wf, Views const&... views)
  { return default_task_placement()(wf, views...); }


  template<typename WF>
  typename std::enable_if<
    is_factory<WF>::value,
    std::pair<rmi_handle::reference, std::vector<unsigned int>>
  >::type
  execution_location(WF const& wf)
  {
    auto locations = m_sections[0];

    std::vector<unsigned int> v;
    v.reserve(locations.size());

    for (auto idx = locations.first(); idx <= locations.last(); ++idx)
      v.push_back(idx);

    return std::make_pair(rmi_handle::reference(), std::move(v));
  }


  template<typename WF, typename View0>
  typename std::enable_if<
    is_factory<WF>::value,
    std::pair<rmi_handle::reference, std::vector<unsigned int>>
  >::type
  execution_location(WF const& wf, View0 const& v0)
  {
    auto locations = m_sections[v0.second + 1];

    std::vector<unsigned int> v;
    v.reserve(locations.size());

    for (auto idx = locations.first(); idx <= locations.last(); ++idx)
      v.push_back(idx);

    return std::make_pair(rmi_handle::reference(), std::move(v));
  }

  default_info get_sched_info(void) noexcept
  { return default_info(); }


  coarsener_type get_coarsener(void) const
  { return null_coarsener(); }

  void notify_finished(void) noexcept
  { }

  void define_type(typer &t)
  { t.member(m_sections); }
}; // class pg_placement_strategy


struct first_grind_wf
{
  using result_type = flow_type;

  result_type operator()(void) const
  {
    auto&& values = vals.get();

    result_type ret(values.size());

    for (size_t i = 0; i < values.size(); ++i)
    {
      ret[i] = i;

      for (size_t j = 0; j < values[i].size(); ++j)
        ret[i] += values[i][j];
    }

    return ret;
  }
};


struct grind_wf
{
  using result_type = flow_type;

  template<typename Boundary>
  result_type
  operator()(Boundary&& boundary) const
  {
    result_type ret(boundary.size());

    auto&& values = vals.get();

    for (size_t i = 0; i < values.size(); ++i)
    {
      ret[i] = boundary[i];

      for (size_t j = 0; j < values[i].size(); ++j)
        ret[i] += values[i][j];
    }

    return ret;
  }
};


struct stage_factory
  : factory_wf
{
  using result_type       = flow_type;
  using coarsener_type    = null_coarsener;
  using domain_type       = indexed_domain<std::size_t>;
  using partition_type    = balanced_partition<domain_type>;

  size_t m_overpartition_factor;

  partition_type m_tasks_ids;

  stage_factory(size_t overpartition_factor)
    : m_overpartition_factor(overpartition_factor)
  { }

  template<typename PGV>
  void operator()(PGV const& pgv)
  {
    const auto my_id     = pgv.graph().get_location_id();
    const auto n_locs    = pgv.graph().get_num_locations();
    const auto n_elems   = n_locs * m_overpartition_factor;

    partition_type part(domain_type(n_elems), n_locs);

    auto local_domain = part[my_id];

    for (auto idx = local_domain.first(); idx <= local_domain.last(); ++idx)
    {
      pgv.add_task(idx, first_grind_wf(), 1);
      pgv.set_result(idx, idx);
    }
  }

  template<typename PGV, typename View>
  void operator()(PGV const& pgv, View const& v0)
  {
    const auto my_id     = pgv.graph().get_location_id();
    const auto n_locs    = pgv.graph().get_num_locations();
    const auto n_elems   = n_locs * m_overpartition_factor;

    partition_type part(domain_type(n_elems), n_locs);

    auto local_domain = part[my_id];

    for (auto idx = local_domain.first(); idx <= local_domain.last(); ++idx)
    {
      pgv.add_task(idx, grind_wf(), 1, consume<flow_type>(v0, idx));
      v0.set_num_succs(idx, 1);
      pgv.set_result(idx, idx);
    }
  }

  coarsener_type get_coarsener(void) const
  { return null_coarsener(); }

  void define_type(typer &t)
  {
    t.member(m_overpartition_factor);
  }
};


struct outer_factory
  : public factory_wf
{
  using result_type = void;

  size_t m_num_sections;
  size_t m_num_elems;
  size_t m_overpartition_factor;

  outer_factory(size_t num_sections,
                size_t num_elems,
                size_t overpartition_factor)
    : m_num_sections(num_sections),
      m_num_elems(num_elems),
      m_overpartition_factor(overpartition_factor)
  { }

  template<typename PGV>
  void operator()(PGV const& pgv)
  {
    const auto my_id        = pgv.graph().get_location_id();
    const auto n_locs       = pgv.graph().get_num_locations();
    const auto section_size = n_locs / m_num_sections;

    for (size_t i = 0; i < m_num_sections; ++i)
      if (my_id == i*section_size)
      {
        const size_t num_succs = (i == (m_num_sections - 1)) ? 0 : 1;

        // std::cout << "Location " << my_id << " will spawn " << i << "\n";

        stage_factory f(m_overpartition_factor);

        if (i == 0)
          pgv.add_task(i, f, num_succs);
        else
          pgv.add_task(i, f, num_succs, consume_pg<flow_type>(pgv, i-1));

      }
  }

  void define_type(typer &t)
  {
    t.member(m_num_sections);
    t.member(m_num_elems);
    t.member(m_overpartition_factor);
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  stapl_assert(
    argc == 5, "./pg_p2p_data_flow section_size num_elem factor length");

  const size_t num_locs             = get_num_locations();
  const size_t nested_section_size  = atoi(argv[1]);
  const size_t num_elems            = atoi(argv[2]);
  const size_t overpartition_factor = atoi(argv[3]);
  const size_t length               = atoi(argv[4]);

  if (num_locs % nested_section_size != 0)
    abort("invalid section size specification");

  auto&& values = vals.get();

  values.resize(num_elems / nested_section_size / overpartition_factor);

  do_once(
    [&]() { std::cout << "Local store size is " << values.size() << "\n"; });

  for (auto&& v : values)
  {
    v.resize(length);
    for (auto&& elem : v)
      elem = rand() % 50000;
  }

  do_once(
    [&]()
    {
      std::cout << "Random Number Generation Done.\n";
      timer_type grind_timer;
      grind_timer.start();

      for (int iter = 0; iter < 30; ++iter)
        for (size_t i = 0; i<overpartition_factor; ++i)
          first_grind_wf();

      const double grind_elapsed = grind_timer.stop() / 30.;

      std::cout << "Estimated grind time per core = " << grind_elapsed << "\n";
    }
  );

  const size_t num_sections         = num_locs / nested_section_size;

  using scheduler_t = fifo_scheduler<pg_placement_strategy>;
  using paragraph_t = paragraph<scheduler_t, outer_factory>;

  outer_factory f(num_sections, num_elems, overpartition_factor);
  scheduler_t   sched(num_sections);

  paragraph_t pg(f, sched);
  pg();

  do_once([]() { std::cout << "Passed\n"; });

  return EXIT_SUCCESS;
}

