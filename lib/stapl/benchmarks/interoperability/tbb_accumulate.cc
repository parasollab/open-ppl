/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#include <cstring>
#include <iostream>
#include <iomanip>
#include <type_traits>
#include <stapl/runtime.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/containers/array/array.hpp>
#include <stapl/views/adaptors/tbb_range_adaptor.hpp>
#include <stapl/views/type_traits/is_proxy.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/views/counting_view.hpp>
#include <stapl/skeletons/map_reduce.hpp>
#include <stapl/profiler/base_profiler.hpp>
#include <stapl/utility/do_once.hpp>
#include <boost/lexical_cast.hpp>
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>
#include <tbb/task_scheduler_init.h>


//////////////////////////////////////////////////////////////////////
/// @brief Uses the base_profiler to benchmark and print results for this
///        benchmark.
///
/// @tparam Counter The counter type to be used.
///
/// @see base_profiler, counter
/// @ingroup performanceMonitor
//////////////////////////////////////////////////////////////////////
template <typename Counter = stapl::counter<stapl::default_timer> >
class my_profiler
: public stapl::base_profiler<Counter>
{
private:
  typedef stapl::base_profiler<Counter> base_type;

public:
  using base_type::report;

  explicit my_profiler(std::string const& inname,
                       int argc = 0, char **argv = 0)
  : base_type(inname, "STAPL+TBB", argc, argv) { }

  void report(std::stringstream& ss)
  {
    stapl::do_once([&,this] {
      ss << std::fixed << std::setprecision(8);
      ss << this->get_name()       << '\t'
         << this->get_iterations() << '\t'
         << this->get_avg()        << '\t'
         << this->get_min()        << '\t'
         << this->get_max()        << '\t'
         << this->get_conf()       << std::endl;
    });
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Function object that increases the value of the passed
///        object by 1.  If the object is a range, it increases all
///        its elements by 1.
//////////////////////////////////////////////////////////////////////
struct increase_elem
{
  template<typename T>
  void operator()(T t,
                  typename std::enable_if<
                    stapl::is_proxy<T>::value
                  >::type* = 0) const
  {
    t = t + 1;
  }

  template<typename T>
  void operator()(T& t,
                  typename std::enable_if<
                    !stapl::is_proxy<T>::value
                  >::type* = 0) const
  {
    std::for_each(t.begin(), t.end(), increase_elem());
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Function object that increases the value of the elements of
///        the given views object by 1 using Intel's
///        tbb::parallel_for().
//////////////////////////////////////////////////////////////////////
struct increase_wf
{
  typedef void result_type;

  template<typename View>
  void operator()(View vw) const
  {
    tbb::parallel_for(stapl::make_tbb_range(vw), increase_elem());
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Uses the base_profiler to benchmark the increase_wf
///        function object.
///
/// @tparam InputView The input view type.
///
/// @see base_profiler, increase_wf
//////////////////////////////////////////////////////////////////////
template<typename InputView>
class increment_all_prof
: public my_profiler<>
{
private:
  typedef my_profiler<> base_type;
  InputView& m_vw;

public:
  increment_all_prof(InputView& vw,
                     int argc = 0, char** argv = 0)
  : base_type("increment_all                ", argc, argv),
    m_vw(vw)
  { }

  void run(void)
  {
    stapl::map_func<stapl::skeletons::tags::with_coarsened_wf>
      (increase_wf(), m_vw);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Function object that accumulates the given value or
///        range. It provides the necessary functions to be used by
///        tbb::parallel_reduce().
//////////////////////////////////////////////////////////////////////
template<typename T>
struct accumulate_wf
{
  T m_t;

  explicit accumulate_wf(void)
  : m_t()
  { }

  accumulate_wf(accumulate_wf& other, tbb::split)
  : m_t()
  { }

  template<typename U>
  void operator()(U u,
                  typename std::enable_if<
                    stapl::is_proxy<U>::value
                  >::type* = 0)
  {
    m_t = m_t + u;
  }

  template<typename U>
  void operator()(U const& u,
                  typename std::enable_if<
                    !stapl::is_proxy<U>::value
                  >::type* = 0)
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


//////////////////////////////////////////////////////////////////////
/// @brief Function object that accumulates the elements of the given view. It
///        uses tbb::parallel_reduce() with accumulate_wf.
///
/// @see accumulate_wf
//////////////////////////////////////////////////////////////////////
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


//////////////////////////////////////////////////////////////////////
/// @brief Uses the base_profiler to benchmark the acc_wf function object.
///
/// @tparam InputView The input view type.
///
/// @see base_profiler, acc_wf
//////////////////////////////////////////////////////////////////////
template<typename InputView>
class acc_prof
: public my_profiler<>
{
private:
  typedef my_profiler<> base_type;
  InputView const& m_ivw;

public:
  acc_prof(InputView const& ivw, int argc = 0, char** argv = 0)
  : base_type("accumulate                   ", argc, argv), m_ivw(ivw)
  { }

  void run(void)
  {
    stapl::plus<typename InputView::value_type> red_wf;
    stapl::map_reduce<stapl::skeletons::tags::with_coarsened_wf>
      (acc_wf(), red_wf, m_ivw);
  }
};


static bool print = false;


stapl::exit_code stapl_main(int argc, char* argv[])
{
  using stapl::for_each;

  if (argc<2) {
    std::cerr << "Usage: " << argv[0] << " Size [--threads N]\n";
    return EXIT_FAILURE;
  }

  const unsigned int N = boost::lexical_cast<unsigned int>(argv[1]);
  print = N < 100;

  if (N==0) {
    std::cerr << argv[0] << ": container is empty.\n";
    return EXIT_FAILURE;
  }

  for (int i = 1; i<argc; ++i) {
    if ( std::strcmp("--threads", argv[i])==0 ) {
      unsigned int nthreads = boost::lexical_cast<unsigned int>(argv[++i]);
      if (nthreads==0) {
        std::cerr << argv[0] << ": number of threads cannot be 0.\n";
        return EXIT_FAILURE;
      }
      tbb::task_scheduler_init ts(nthreads);
    }
  }

  typedef int                           value_type;
  typedef stapl::array<value_type>      array_type;
  typedef stapl::array_view<array_type> view_type;

  array_type a(N);
  view_type vw(a);

  // generate data
  stapl::copy(stapl::counting_view<value_type>(N), vw);

  const auto num_locs = stapl::get_num_locations();

  stapl::do_once([&] {
    std::cout << "container_size\t" << N << '\n'
              << "locations     \t" << num_locs << std::endl;
  });

  // increment_all
  {
    increment_all_prof<view_type> pr(vw, argc, argv);
    pr.collect_profile();
    pr.report();
  }

  // accumulate
  {
    acc_prof<view_type> pr(vw, argc, argv);
    pr.collect_profile();
    pr.report();
  }

  if (print) {
    stapl::do_once([&] {
      std::cout << "----- array<int> ---" << std::endl;
      for (unsigned int i=0; i<N; ++i)
        std::cout << a[i] << ' ';
      std::cout << std::endl;
    });
  }

  return EXIT_SUCCESS;
}
