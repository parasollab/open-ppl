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
/// Benchmark for nested parallelism. Find minimum element in
/// @c stapl::array<std::vector<int>>.
//////////////////////////////////////////////////////////////////////

//#define BENCHMARK_RANDOM_GENERATION 1

#include <stapl/algorithm.hpp>
#include <stapl/containers/array/array.hpp>
#include <stapl/skeletons/serial.hpp>
#include <boost/lexical_cast.hpp>
#include <vector>
#include "../runtime_base_profiler.hpp"

#ifdef BENCHMARK_RANDOM_GENERATION
# include <random>
#endif


using namespace stapl;


class sizes_wf
{
private:
  typedef std::size_t size_type;

  size_type m_min_size;
  size_type m_max_size;

public:
  typedef void result_type;

  sizes_wf(size_type min, size_type max)
    : m_min_size(min),
      m_max_size(max)
  { }

  template<typename T, typename Index>
  void operator()(T t, Index i) const
  {
#if BENCHMARK_RANDOM_GENERATION
    size_type seed = i;
    std::mt19937                    gen(seed);
    std::uniform_int_distribution<> dist(m_min_size, m_max_size);
    t = dist(gen);
#else
    t = m_min_size + (i * m_max_size);
#endif
  }

  void define_type(typer& t)
  {
    t.member(m_min_size);
    t.member(m_max_size);
  }
};


struct outer_init_wf
{
  typedef void result_type;

  template <typename T1, typename T2>
  void operator()(T1 t, T2 size)
  {
    t.resize(size);
    std::iota(t.begin(), t.end(), 1);
  }
};


struct print_outer_wf
{
  typedef void result_type;

  template<typename T>
  void operator()(T t) const
  {
    for (auto i : t)
      std::cout << i << ' ';
    std::cout << std::endl;
  }
};


template <typename Op>
struct reduce_wf
: private Op
{
  typedef typename Op::result_type result_type;

  template<typename T>
  reduce_wf(T&& t)
    : Op(std::forward<T>(t))
  { }

  template <typename V>
  result_type operator()(V&& v) const
  {
    return *std::min_element(v.begin(), v.end(), static_cast<Op const&>(*this));
  }

  void define_type(typer& t)
  { t.base<Op>(*this); }
};


template<typename T, typename InputView>
class min_prof
: public runtime_base_profiler<>
{
private:
  InputView m_ivw;

public:
  min_prof(InputView ivw, int argc = 0, char** argv = nullptr)
  : runtime_base_profiler<>("min_element_flat", argc, argv),
    m_ivw(ivw)
  { }

  void run(void)
  {
    typedef stapl::min<T> op_type;
    op_type            reduce_op;
    reduce_wf<op_type> map_op(reduce_op);
    map_reduce(map_op, reduce_op, m_ivw);
  }
};


static bool print = false;


exit_code stapl_main(int argc, char* argv[])
{
  if (argc<4) {
    std::cerr << "Usage: " << argv[0]
              << " outer_container_size inner_random_start inner_random_end\n";
    return EXIT_FAILURE;
  }

  const auto OUTER_SIZE       = boost::lexical_cast<unsigned int>(argv[1]);
  const auto INNER_START_SIZE = boost::lexical_cast<unsigned int>(argv[2]);
  const auto INNER_END_SIZE   = boost::lexical_cast<unsigned int>(argv[3]);

  if (OUTER_SIZE==0
      || INNER_START_SIZE==0
      || INNER_END_SIZE==0
      || INNER_START_SIZE>INNER_END_SIZE) {
    std::cerr << argv[0] << ": incorrect sizes.\n";
    return EXIT_FAILURE;
  }

  print = (OUTER_SIZE <= 30) && (INNER_START_SIZE<10) && (INNER_END_SIZE<100);

  // sizes container
  typedef array<std::size_t>     sizes_ct_t;
  typedef array_view<sizes_ct_t> sizes_vw_t;

  sizes_ct_t sizes_ct(OUTER_SIZE);
  sizes_vw_t sizes_vw(sizes_ct);

  map_func(sizes_wf(INNER_START_SIZE, INNER_END_SIZE),
           sizes_vw, counting_view<int>(OUTER_SIZE));

  // container
  typedef int                              value_type;
  typedef std::vector<value_type>          inner_container_type;
  typedef array<inner_container_type>      outer_container_type;
  typedef array_view<inner_container_type> inner_view_type;
  typedef array_view<outer_container_type> outer_view_type;

  outer_container_type c(OUTER_SIZE);
  outer_view_type vw(c);

  // generate data
  map_func(outer_init_wf{}, vw, sizes_vw);

  do_once(
    [&] { std::cout << "container_size\t" << OUTER_SIZE << std::endl; });

  if (!print) {
    min_prof<value_type, outer_view_type> pr(vw, argc, argv);
    pr.collect_profile();
    pr.report();
    stapl::rmi_fence();
  }
  else {
    // print data
    do_once(
      [&] { std::cout << "----- array<std::vector<int>> ---" << std::endl; });
    serial_io(print_outer_wf{}, vw);

    // compute min_row()
    typedef stapl::min<value_type> op_type;
    op_type            reduce_op;
    reduce_wf<op_type> map_op(reduce_op);

    stapl::counter<stapl::default_timer> timer;
    timer.start();
    auto r = stapl::map_reduce(map_op, reduce_op, vw);
    const double elapsed = timer.stop();

    // print results
    do_once([&]
            {
              std::cout << "min_element_flat\n"
                        << "result: " << r                  << '\n'
                        << "time:   " << elapsed << " secs" << std::endl;
            });
  }

  return EXIT_SUCCESS;
}
