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
#include <stapl/views/metadata/coarseners/default.hpp>
#include <stapl/views/array_view.hpp>
#include <test/algorithms/test_utils.h>

struct add17
{
  typedef std::vector<int> result_type;

  result_type operator()(result_type const& in) const
  {
    result_type out(in);

    out[0] += 17;

    return out;
  }

  bool operator==(add17 const&) const
  {
    return true;
  }
};


struct add5
{
  typedef std::vector<int> result_type;

  result_type operator()(result_type const& in) const
  {
    result_type out(in);

    out[0] += 5;

    return out;
  }

  bool operator==(add5 const&) const
  {
    return true;
  }
};


struct filter_odd
{
  typedef std::vector<int> result_type;

  std::vector<int> operator()(std::vector<int> const& in) const
  {
    std::vector<int> out(in);
    int cnt = 0;

    for (std::vector<int>::iterator i = out.begin(); i != out.end(); ++i, ++cnt)
    {
      if (cnt % 2 == 0)
      {
        *i = 0;
      }
    }

    return out;
  }

  bool operator==(filter_odd const&) const
  {
    return true;
  }
};


struct combine
{
  typedef std::vector<int> result_type;

  template <typename Ref1, typename Ref2>
  result_type operator()(Ref1 x, Ref2 y)
  {
    result_type tmp1 = x;
    result_type tmp2 = y;

    result_type tmp3;

    for (unsigned int i=0; i<tmp1.size(); ++i)
    {
      if (i == 0)
        tmp3.push_back(tmp1[i] + tmp2[i] - 1);
      else
        tmp3.push_back(tmp1[i]);
    }

    return tmp3;
  }
};


struct filtered_wf
{
  typedef std::vector<int> result_type;

  template <typename Ref>
  result_type operator()(Ref v) const
  {
    return v[0];
  }

  template <typename Ref, typename ResultRef>
  result_type operator()(Ref v, ResultRef in) const
  {
    result_type tmp;

    for (typename Ref::iterator i = v.begin(); i != v.end(); ++i)
    {
      tmp = *i;

      result_type in0 = in;

      std::transform(tmp.begin(), tmp.end(), in0.begin(),
                     tmp.begin(), std::plus<int>());

      *i = tmp;
    }

    return tmp;
  }
};


struct filter_factory
{
  using result_type = std::vector<int>;
  using coarsener_type = stapl::default_coarsener;

  coarsener_type get_coarsener() const
  { return coarsener_type(); }


  template <typename TGV, typename View>
  void operator()(TGV const& tgv, View& input_view)
  {
    typedef filtered_wf::result_type            task_result;
    typedef filter_odd                          result_filter;
    typedef stapl::view_index_iterator<View>    id_iter_type;

    using stapl::consume;

    stapl::tuple<id_iter_type> id_it = partition_id_set(input_view);

    id_iter_type iter = stapl::get<0>(id_it);

    const unsigned int n_locs     = stapl::get_num_locations();
    const std::size_t  n_elements = input_view.size();
    std::size_t        retval_id  = 0;

    const std::size_t num_succs   = n_locs == 1 ? 2 : 3;

    // Create a chain of partial consumption (and element of original view)
    // also have a partial consumer task colocated with each task above, to
    // test producer location partial edge consumption (and we play with
    // ordering of these activities).  Finally test, multiple filter
    // functionality by having 2nd row of tasks receive values filtered values
    // from two predecsssors.
    //
    //
    // E0           E1           E2
    // |            |            |
    // |            |            |
    // \/           \/           \/
    // T0 --filt--> T1 --filt--> T2
    // |          / |          / |
    // add17    /   add17    /   add17
    // |      /     |      /     |
    // \/   / add5  \/   / add5  \/
    // T3 <         T4 <         T5
    //
    for (; !iter.at_end(); ++iter)
    {
      std::size_t id = *iter;

      // Based on id, alternate specification of partial consumer on same
      // location either before or after the producer, just to test better.
      //
      if (id % 2 == 0)
      {
        const size_t pred1 = id;
        const size_t pred2 = n_locs == 1 ? 0 : (id + 1) % n_locs;

        tgv.add_task(
            n_elements + id, combine(), (std::size_t) 1,
            consume<task_result>(tgv, pred1, add17()),
            consume<task_result>(tgv, pred2, add5())
        );
      }

      if (id == 0)
      {
        tgv.add_task(id, filtered_wf(), num_succs,
          std::make_pair(&input_view, id));
      }
      else if (id == n_elements-1)
      {
        tgv.add_task(id, filtered_wf(), num_succs-1,
          std::make_pair(&input_view, id),
          consume<task_result>(tgv, (std::size_t) id-1, result_filter()));
      }

      // steady state, not first or last location...
      //
      else
      {
        tgv.add_task(id, filtered_wf(), num_succs,
          std::make_pair(&input_view, id),
          consume<task_result>(tgv, (std::size_t) id-1, result_filter()));
      }

      // Based on id, alternate specification of partial consumer on same
      // location either before or after the producer, just to test better.
      //
      if (id % 2 == 1)
      {
        const size_t pred1 = id;
        const size_t pred2 = n_locs == 1 ? 0 : (id + 1) % n_locs;

        tgv.add_task(
          n_elements + id, combine(), (std::size_t) 1,
          consume<task_result>(tgv, pred1, add17()),
          consume<task_result>(tgv, pred2, add5())
        );
      }

      retval_id = n_elements + id;
    }

    tgv.set_result(retval_id);
  }
};


struct create_vector_wf
{
  std::size_t m_size;

  create_vector_wf(std::size_t size)
    : m_size(size)
  { }

  typedef std::vector<int> result_type;

  result_type operator()(void) const
  {
    result_type v(m_size);
    std::iota(v.begin(), v.end(), 1);
    return v;
  }

  void define_type(stapl::typer &t)
  { t.member(m_size); }
};


template<typename T>
struct pick_i
{
  size_t m_idx;

  pick_i(size_t idx)
    : m_idx(idx)
  { }

  typedef T result_type;

  T operator()(std::vector<T> const& elems) const
  {
    return elems[m_idx];
  }

  void define_type(stapl::typer &t)
  { t.member(m_idx); }
};


struct forward_val
{
  int m_idx;

  typedef int result_type;

  forward_val(size_t idx)
    : m_idx(idx)
  { }

  void define_type(stapl::typer &t)
  { t.member(m_idx); }

  template<typename Ref>
  int operator()(Ref ref) const
  {
    return ref;
  }
};


struct check_vals_wf
{
  int m_sum;

  typedef bool result_type;

  check_vals_wf(int sum)
    : m_sum(sum)
  { }

  template<typename View>
  result_type operator()(View v) const
  {
    int sum = 0;
    for (size_t i = 0; i < v.size(); ++i)
      sum += v[i];

    return sum == m_sum;
  }

  void define_type(stapl::typer &t)
  { t.member(m_sum); }
};


// Location 0 creates a task with a vector of size num locations.
// Each location create two tasks consuming my_id and my_id + 1 % n_locs values.
// Aggregated consume used by result task to validate results.
struct filter_type_change_factory
{
  using result_type = bool;
  using coarsener_type = stapl::default_coarsener;

  coarsener_type get_coarsener() const
  { return coarsener_type(); }

  template<typename TGV>
  void operator()(TGV const& tgv)
  {
    std::size_t n_locs = tgv.graph().get_num_locations();
    std::size_t my_id  = tgv.graph().get_location_id();
    std::size_t root   = 0;
    const bool b_root  = my_id == root;

    if (b_root)
      tgv.add_task(0, create_vector_wf(n_locs), 2*n_locs);

    tgv.add_task(my_id + 1, forward_val(my_id), 1,
                 stapl::consume<std::vector<int>>(
                   tgv, root, pick_i<int>(my_id)));

    tgv.add_task(n_locs + my_id + 1, forward_val((my_id+1)%n_locs), 1,
                 stapl::consume<std::vector<int>>(
                   tgv, root, pick_i<int>((my_id+1)%n_locs)));


    const std::size_t retval_id = 2 * n_locs + 1;

    if (b_root)
    {
      std::vector<std::size_t> preds(2*n_locs);
      for (size_t i=0; i<n_locs*2; ++i)
        preds[i] = i + 1;

      tgv.add_task(retval_id, check_vals_wf(n_locs * (n_locs + 1)), n_locs,
                   stapl::consume<int>(tgv, std::move(preds)));
    }

    tgv.set_result(retval_id);
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
    std::cout << "Running filtered_dataflow_pos on "
              << n_locs << " locations... ";
  }

  typedef array<std::vector<int> >                             cont_t;
  typedef array_view<cont_t>                                   view_t;
  typedef paragraph<default_scheduler, filter_factory, view_t> paragraph_t;

  cont_t cont(n_locs, std::vector<int>(10, 1));

  view_t view(cont);

  paragraph_t pg(filter_factory(), view);

  std::vector<int> res = pg();

  int result_value     = std::accumulate(res.begin(), res.end(), 0);
  const int validation_value = (loc_id + 1) * 5 + 5 + 17 + 5;

  stapl::stapl_bool test_result(result_value == validation_value);

  test_result.reduce();

  if (loc_id == 0)
  {
    if (test_result.value())
      std::cout << "Passed\n";
    else
      std::cout << "Failed\n";
  }

  typedef paragraph<default_scheduler, filter_type_change_factory> tg_t;
  const bool b_passed = ((tg_t)(filter_type_change_factory()))();

  stapl::do_once([b_passed] {
    std::cout << "Running filtered_dataflow (type change)... ";

    if (b_passed)
      std::cout << "Passed" << std::endl;
    else
      std::cout << "Failed" << std::endl;
  });

  return EXIT_SUCCESS;
}
