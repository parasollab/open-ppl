/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <string>
#include <vector>
#include <algorithm>

#include <stapl/views/metadata/coarseners/default.hpp>
#include <stapl/views/metadata/coarseners/multiview.hpp>
#include <stapl/paragraph/paragraph.hpp>

using namespace stapl;

using flow_type = std::vector<size_t>;


struct filter_wf
{
  using result_type = size_t;

  result_type operator()(flow_type const& v) const
  {
    return std::accumulate(v.begin(), v.end(), 0);
  }
}; // struct filter_wf


struct vec_pred_wf
{
  size_t m_rank;
  size_t m_flow_size;

  vec_pred_wf(size_t rank, size_t flow_size)
    : m_rank(rank),
      m_flow_size(flow_size)
  { }

  void define_type(typer& t)
  {
    t.member(m_rank);
    t.member(m_flow_size);
  }

  using result_type = flow_type;

  result_type operator()(void) const
  {
    result_type v(m_flow_size);
    std::iota(v.begin(), v.end(), 1 + m_rank);
    return v;
  }
}; // struct vec_pred_wf


struct size_t_pred_wf
{
  size_t m_rank;
  size_t m_flow_size;

  size_t_pred_wf(size_t rank, size_t flow_size)
    : m_rank(rank),
      m_flow_size(flow_size)
  { }

  void define_type(typer& t)
  {
    t.member(m_rank);
    t.member(m_flow_size);
  }

  using result_type = size_t;

  result_type operator()(void) const
  {
    flow_type v(m_flow_size);
    std::iota(v.begin(), v.end(), 1 + m_rank);
    v[rand() % m_flow_size] = rand();
    return std::accumulate(v.begin(), v.end(), 0);
  }
}; // struct pred_wf


struct vec_pred_factory
  : factory_wf
{
  using span_type         = int;
  using result_type       = flow_type;
  using coarsener_type    = null_coarsener;

  size_t m_flow_size;

  vec_pred_factory(size_t flow_size)
    : m_flow_size(flow_size)
  { }

  void define_type(typer& t)
  { t.member(m_flow_size); }  template<typename PGV>
  void operator()(PGV const& pgv) const
  {
    const auto my_id = pgv.graph().get_location_id();
    pgv.add_task(my_id, vec_pred_wf(my_id + 1, m_flow_size), 1);
    pgv.set_result(my_id, my_id);
  }

  coarsener_type get_coarsener(void) const
  { return null_coarsener(); }

}; // struct vec_pred_factory


struct size_t_pred_factory
  : factory_wf
{
  using span_type         = int;
  using result_type       = size_t;
  using coarsener_type    = null_coarsener;

  size_t m_flow_size;

  size_t_pred_factory(size_t flow_size)
    : m_flow_size(flow_size)
  { }

  void define_type(typer& t)
  { t.member(m_flow_size); }

  template<typename PGV>
  void operator()(PGV const& pgv) const
  {
    const auto my_id = pgv.graph().get_location_id();
    pgv.add_task(my_id, size_t_pred_wf(my_id + 1, m_flow_size), 1);
    pgv.set_result(my_id, my_id);
  }

  coarsener_type get_coarsener(void) const
  { return null_coarsener(); }

}; // struct size_t_factory


struct succ_wf
{
  size_t m_rank;
  size_t m_flow_size;

  succ_wf(size_t rank, size_t flow_size)
    : m_rank(rank),
      m_flow_size(flow_size)
  { }

  using result_type = void;

  void define_type(typer& t)
  {
    t.member(m_rank);
    t.member(m_flow_size);
  }

  template<typename Ref>
  void operator()(Ref r)
  {
    // Guard Overflow when running larger numbers for timing
    if (m_flow_size > 1000)
      return;

    size_t expected_value = (m_flow_size + m_rank) * (1+m_flow_size + m_rank)/2;
    expected_value       -= m_rank*(m_rank+1) / 2;

    if (expected_value != r)
      abort("incorrect dataflow");
  }
};


struct succ_factory
  : factory_wf
{
  using result_type = void;

  size_t m_flow_size;

  succ_factory(size_t flow_size)
    : m_flow_size(flow_size)
  { }

  void define_type(typer& t)
  { t.member(m_flow_size); }

  template<typename PGV, typename V0>
  void operator()(PGV const& pgv, V0 const& v0) const
  {
    const auto my_id = pgv.graph().get_location_id();
    v0.set_num_succs(my_id, 1);
    pgv.add_task(my_id, succ_wf(my_id + 1, m_flow_size), 0,
                 consume<size_t>(v0, my_id));
  }
}; // struct succ_factory


struct outer_factory
  : factory_wf
{
  using result_type = void;

  bool   m_b_filter;
  size_t m_flow_size;

  outer_factory(bool b_filter, size_t flow_size)
    : m_b_filter(b_filter),
      m_flow_size(flow_size)
  { }

  void define_type(typer& t)
  {
    t.member(m_b_filter);
    t.member(m_flow_size);
  }

  template<typename PGV>
  void operator()(PGV const& pgv) const
  {
    const auto my_id = pgv.graph().get_location_id();

    if (my_id == 0)
    {

      if (m_b_filter)
      {
        pgv.add_task(0, vec_pred_factory(m_flow_size), 1);
        pgv.add_task(1, succ_factory(m_flow_size), 0,
                     consume_pg<flow_type>(
                       pgv, 0, unconditional_flow(), filter_wf()));
      }
      else
      {
        pgv.add_task(0, size_t_pred_factory(m_flow_size), 1);
        pgv.add_task(1, succ_factory(m_flow_size), 0,
                     consume_pg<size_t>(
                       pgv, 0, unconditional_flow()));
      }
    }
  }
};


exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 2)
    abort("./pg_nested_filtered_dataflow_pos flow_size");

  const size_t flow_size = atoi(argv[1]);

  if (flow_size > 1000)
    do_once([] { std::cout << "Full validation only run for N < 1000\n"; });

  do_once([] { std::cout << "pg_nested_filtered_dataflow..."; });

  using timer_t = counter<default_timer>;

  // No Filter (instead, applied in user wf)
  timer_t no_filter_timer;
  no_filter_timer.start();
  make_paragraph(outer_factory(false, flow_size))();
  const double no_filter_elapsed = no_filter_timer.stop();

  // Filter
  timer_t filter_timer;
  filter_timer.start();
  make_paragraph(outer_factory(true, flow_size))();
  const double filter_elapsed = filter_timer.stop();

  do_once([no_filter_elapsed, filter_elapsed] {
    std::cout << " Passed\n";
    std::cout << "Filtering In UserWF time = " << no_filter_elapsed << "\n";
    std::cout << "Pin value filtering time = " << filter_elapsed << "\n";
  });

  return EXIT_SUCCESS;
}

