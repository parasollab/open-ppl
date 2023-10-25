/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <cstdlib>
#include <stapl/views/adaptors/tbb_range_adaptor.hpp>
#include <stapl/containers/array/array.hpp>
#include <stapl/views/counting_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/numeric.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/views/type_traits/is_proxy.hpp>
#include <type_traits>
#include "../test_report.hpp"
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>

struct increase_elem
{
  typedef void result_type;

  template<typename T>
  void operator()(
         T t,
         typename std::enable_if<stapl::is_proxy<T>::value>::type* = 0) const
  {
    t = t + 1;
  }

  template<typename T>
  void operator()(
         T& t,
         typename std::enable_if<!stapl::is_proxy<T>::value>::type* = 0) const
  {
    std::for_each(t.begin(), t.end(), increase_elem());
  }
};


struct increase_wf
{
  typedef void result_type;

  template<typename View>
  void operator()(View& vw) const
  {
    tbb::parallel_for(stapl::make_tbb_range(vw), increase_elem());
  }
};


template<typename T>
struct accumulate_wf
{
  T m_t;

  explicit accumulate_wf()
    : m_t()
  { }

  accumulate_wf(accumulate_wf& other, tbb::split)
    : m_t()
  { }

  template<typename U>
  void operator()(
         U u, typename boost::enable_if<stapl::is_proxy<U> >::type* = 0)
  {
    m_t = m_t + u;
  }

  template<typename U>
  void operator()(
         U const& u, typename boost::disable_if<stapl::is_proxy<U> >::type* = 0)
  {
    m_t = m_t + std::accumulate(u.begin(), u.end(), T());
  }

  void join(accumulate_wf const& other)
  {
    m_t = m_t + other.m_t;
  }

  T const& get() const
  {
    return m_t;
  }
};


struct acc_wf
{
  template<typename View>
  typename View::value_type operator()(View vw)
  {
    accumulate_wf<typename View::value_type> wf;
    tbb::parallel_reduce(stapl::make_tbb_range(vw), wf);
    return wf.get();
  }
};

stapl::exit_code stapl_main(int argc, char* argv[])
{
  using namespace stapl;

  if (argc < 2) {
    std::cout << "usage: exe n" << std::endl;
    std::exit(1);
  }

  const unsigned int N = std::atoi(argv[1]);

  typedef unsigned int           value_type;
  typedef array<value_type>      array_type;
  typedef array_view<array_type> view_type;

  array_type a(N);
  view_type vw(a);

  // generate data
  copy(counting_view<value_type>(N), vw);
  const value_type sum = map_reduce(identity<value_type>(),
                                    plus<value_type>(),
                                    counting_view<value_type>(N));

  // increment all elements
  map_func<skeletons::tags::with_coarsened_wf>(increase_wf(), vw);
  const value_type nsum = accumulate(vw, 0);
  STAPL_TEST_REPORT( (sum + N)==nsum,
                     "Testing tbb_range_adaptor with tbb::parallel_for" );

  // add all elements
  plus<value_type> red_wf;
  const value_type tsum = map_reduce<skeletons::tags::with_coarsened_wf>
                            (acc_wf(), red_wf, vw);
  STAPL_TEST_REPORT( tsum==nsum,
                     "Testing tbb_range_adaptor with tbb::parallel_reduce" );

  return EXIT_SUCCESS;
}
