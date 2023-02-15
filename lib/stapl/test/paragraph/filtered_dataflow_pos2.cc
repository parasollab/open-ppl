/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <iostream>
#include <vector>

#include <stapl/paragraph/paragraph.hpp>
#include <stapl/containers/array/array.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/algorithms/functional.hpp>

template <typename T>
struct do_something
{
  typedef T result_type;

  T operator()(void) const
  {
    T t(5);
    std::iota(t.begin(), t.end(), 0);
    return t;
  }
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


template <typename T>
struct pickfirst
{
  typedef T result_type;

  template <typename Ts>
  T operator()(Ts ts) const
  {
    return ts[0];
  }
};


template <typename T>
struct back_wf
{
  typedef std::vector<T> result_type;

  template <typename Ts>
  result_type operator()(Ts ts) const
  {
    result_type result(1);
    result.push_back(ts.back());
    return result;
  }

  bool operator==(back_wf const&) const
  {
    return true;
  }
};


struct check_val
{
  size_t m_idx;

  typedef bool result_type;

  check_val(size_t idx)
    : m_idx(idx)
  { }

  void define_type(stapl::typer &t)
  { t.member(m_idx); }

  template<typename Ref>
  bool operator()(Ref ref) const
  {
    return ref == m_idx;
  }
};


struct reduce_wf
{
  typedef bool result_type;

  template <typename View>
  result_type operator()(View v) const
  {
    for (size_t i = 0; i < v.size(); ++i)
      if (v[i] == false)
        return false;
    return true;
  }
};


struct viewless_factory
{
public:
  typedef bool result_type;

  template <typename TGV>
  void operator()(TGV const& tgv)
  {
    // get the tasks to be added from the id_calculator
    typedef std::vector<int> value_t;

    tgv.add_task((std::size_t) 0, do_something<value_t>(), (std::size_t) 7);

    tgv.add_task((std::size_t) 1, pickfirst<int>(), (std::size_t) 0,
                 stapl::consume<value_t>(tgv, 0, back_wf<int>()));

    tgv.add_task((std::size_t) 2, stapl::identity<value_t>(), (std::size_t) 0,
                 stapl::consume<value_t>(tgv, 0));

    for (int i=0; i<5; ++i)
      tgv.add_task((std::size_t) 3 + i, check_val(i), (std::size_t) 1,
                   stapl::consume<value_t>(tgv, 0, pick_i<int>(i)));

    std::vector<std::size_t> preds(5);

    for (size_t i = 0; i < 5; ++i)
      preds[i] = 3 + i;

    const std::size_t retval_id = 8;

    tgv.add_task(retval_id, reduce_wf(), (std::size_t) 1,
                 stapl::consume<bool>(tgv, std::move(preds)));

    tgv.set_result(retval_id);
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  using std::cout;
  using std::endl;

  using namespace stapl;

  if (stapl::get_num_locations() > 1) {
    std::cerr << argv[0] << ": this unit test is written for one location.\n";
    return EXIT_FAILURE;
  }

  cout << "Testing paragraph for filtered / non filtered consumption "
       << "mixing on producer location and type mutating filters...";

  typedef paragraph<default_scheduler, viewless_factory> tg_t;

  const bool b_passed = ((tg_t)(viewless_factory()))();

  // if it runs without assertion, it passes...
  stapl::do_once([b_passed] {
    if (b_passed)
      cout << "Passed" << endl;
    else
      cout << "Failed" << endl;
  });

  return EXIT_SUCCESS;
}
