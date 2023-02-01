/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/paragraph/paragraph.hpp>
#include <stapl/utility/tuple.hpp>
#include <stapl/containers/array/array.hpp>
#include <stapl/views/array_view.hpp>
#include <test/algorithms/test_utils.h>

struct add17
{
  typedef int result_type;

  result_type operator()(result_type const& in) const
  {
    return in + 17;
  }

  bool operator==(add17 const&) const
  {
    return true;
  }
};


struct identity
{
  typedef int result_type;

  template <typename View>
  result_type operator()(View const& v)
  {
    return *v.begin();
  }
};


struct reduce_wf
{
  typedef int result_type;

  template <typename View>
  result_type operator()(View v) const
  {
    int tmp = 0;

    for (size_t i = 0; i < v.size(); ++i)
      tmp += v[i];

    return tmp;
  }
};


class filter_factory
  : public stapl::task_factory_base
{
public:
  using result_type = int;
  using coarsener_type = stapl::default_coarsener;

  coarsener_type get_coarsener() const
  { return coarsener_type(); }

  template <typename TGV, typename View>
  void operator()(TGV const& tgv, View& input_view)
  {
    typedef reduce_wf::result_type              task_result;
    typedef stapl::view_index_iterator<View>    id_iter_type;

    stapl::tuple<id_iter_type> id_it = partition_id_set(input_view);

    id_iter_type iter = stapl::get<0>(id_it);

    const std::size_t n_elements = stapl::get_num_locations();
    const std::size_t retval_id  = n_elements + stapl::get_location_id();

    for (; !iter.at_end(); ++iter)
    {
      std::size_t id = *iter;

      // std::cout << stapl::get_location_id() << ": Add Task(" << id << ")\n";
      //
      // Out degree decreases as rank increases.
      //
      tgv.add_task(
        id, identity(),
        (std::size_t) stapl::get_num_locations() - stapl::get_location_id(),
        std::make_pair(&input_view, id));
    }

    // In degree increases as rank increases.
    //
    const std::size_t num_preds = stapl::get_location_id() + 1;

    std::vector<std::size_t> preds(num_preds);

    for (size_t i = 0; i < num_preds; ++i)
    {
      // Switch the order the predecessors are specified.
      // Result should be the same.
      //
      if (stapl::get_location_id() % 2 == 0)
        preds[i] = i;
      else
        preds[num_preds - 1 - i] = i;
    }

    // std::cout << stapl::get_location_id() << ": Add retval Task("
    //           << retval_id << ")\n";
    //
    tgv.add_task(retval_id, reduce_wf(), (std::size_t) 1,
                 stapl::consume<task_result>(tgv, std::move(preds), add17()));

    tgv.set_result(retval_id);

    this->m_finished = true;
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  using stapl::array;
  using stapl::array_view;
  using stapl::paragraph;
  using stapl::default_scheduler;

  int loc_id = (int) stapl::get_location_id();
  int n_locs = (int) stapl::get_num_locations();

  if (loc_id == 0)
  {
    std::cout << "Running aggregated_dataflow_pos on "
              << n_locs << " locations... ";
  }

  typedef array<int>                                           cont_t;
  typedef array_view<cont_t>                                   view_t;
  typedef paragraph<default_scheduler, filter_factory, view_t> paragraph_t;

  cont_t cont(n_locs, 1);

  for (size_t i = 0; i < cont.size(); ++i)
    cont[i] = i;

  stapl::rmi_fence();

  view_t view(cont);

  paragraph_t pg(filter_factory(), view);

  const int res              = pg();
  const int validation_value = 17 * (loc_id + 1) + (loc_id) * (loc_id + 1) / 2;

  stapl::stapl_bool test_result(res == validation_value);

  test_result.reduce();

  if (loc_id == 0)
  {
    if (test_result.value())
      std::cout << "Passed\n";
    else
      std::cout << "Failed\n";
  }

  return EXIT_SUCCESS;
}
