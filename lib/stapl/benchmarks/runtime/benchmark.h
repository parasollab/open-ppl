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
/// Provides a simple profiler for runtime benchmarks.
//////////////////////////////////////////////////////////////////////

#ifndef STAPL_BENCHMARKS_RUNTIME_BENCHMARK_HPP
#define STAPL_BENCHMARKS_RUNTIME_BENCHMARK_HPP

#include <stapl/runtime.hpp>
#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>
#include <boost/program_options.hpp>
#include <boost/accumulators/accumulators.hpp>

#ifdef BOOST_INTEL
// icc does not recognize __attribute__((__optimize__("no-associative-math")))
// used in boost::accumulators::sum_kahan/weighted_sum_kahan
# pragma warning disable 3175
#endif
#ifdef __clang__
// clang complains about
// boost/numeric/ublas/traits.hpp:45:27: error: unused function
//   'boost_numeric_ublas_abs' [-Werror,-Wunused-function]
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wunused-function"
#endif

#include <boost/accumulators/statistics.hpp>

#ifdef __clang__
# pragma clang diagnostic pop
#endif
#ifdef BOOST_INTEL
# pragma warning enable 3175
#endif

namespace stapl {

// The runtime_profiler is used for conducting benchmarks
template<typename Counter = counter<default_timer>>
class runtime_profiler
{
private:
  typedef Counter                           counter_type;
  typedef typename counter_type::value_type value_type;
  typedef std::size_t                       size_type;

  struct result_type
  {
    typedef std::vector<value_type>                 container_type;
    typedef typename container_type::size_type      size_type;
    typedef typename container_type::const_iterator const_iterator;

    std::string    m_name;
    value_type     m_total;
    container_type m_vals;

    explicit result_type(std::string const& s, const size_type sz = 0)
    : m_name(s)
    { m_vals.reserve(sz); }

    std::string const& name(void) const noexcept
    { return m_name; }

    void set_total(const value_type v)
    { m_total = v; }

    value_type get_total(void) const noexcept
    { return m_total; }

    void push_back(const value_type v)
    { m_vals.push_back(v); }

    const_iterator begin(void) const noexcept
    { return m_vals.begin(); }

    const_iterator end(void) const noexcept
    { return m_vals.end(); }

    size_type size(void) const noexcept
    { return m_vals.size(); }

    bool operator<(result_type const& other) const noexcept
    { return (m_name.length()<other.m_name.length()); }

    bool operator>(result_type const& other) const noexcept
    { return (m_name.length()>other.m_name.length()); }
  };

  struct max_vec
  {
    struct max
    {
      value_type const& operator()(value_type const& v1,
                                   value_type const& v2) const noexcept
      {
        if (v1 < v2)
          return v2;
        else
          return v1;
      }
    };

    std::vector<value_type> operator()(std::vector<value_type> const& v1,
                                       std::vector<value_type> const& v2) const
    {
      std::vector<value_type> v(v1.size());
      std::transform(v1.begin(), v1.end(),
                     v2.begin(),
                     v.begin(),
                     max());
      return v;
    }
  };

  typedef std::vector<result_type> result_container;

  static const size_type DEFAULT_MIN_ITER = 5;
  static const size_type DEFAULT_MAX_ITER = 100;

  std::string      m_title;     // benchmark title
  std::string      m_comment;   // comment for the benchmark
  std::string      m_text;      // additional text
  size_type        m_min_iters; // min number of iterations
  size_type        m_max_iters; // max number of iterations
  result_container m_results;   // results
  rmi_handle       m_handle;

  // Reads the program options
  void get_program_options(int argc, char **argv)
  {
    if (argc==0)
      return;

    // parse title
    m_title = argv[0];
    const std::size_t found = m_title.find_last_of("/\\");
    m_title = m_title.substr(found + 1);

    // parse options
    namespace po = boost::program_options;
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help",
           "Prints this help message")
        ("comment",
            po::value<std::string>(),
            "Adds a comment to the output")
        ("min_iterations",
            po::value<size_type>()->default_value(DEFAULT_MIN_ITER),
            "Minimum number of iterations")
        ("max_iterations",
            po::value<size_type>()->default_value(DEFAULT_MAX_ITER),
            "Maximum number of iterations")
        ("confidence_interval", po::value<double>(),
            "Required confidence interval");

    po::variables_map vm;
    po::parsed_options parsed = po::command_line_parser(argc, argv).
                                  options(desc).allow_unregistered().run();
    po::store(parsed, vm);
    po::notify(vm);

    // read configuration
    if (vm.count("help")) {
      std::cout << desc << std::endl;
      std::exit(1);
    }
    if (vm.count("comment"))
      m_comment = vm["comment"].as<std::string>();
    if (vm.count("min_iterations"))
      m_min_iters = vm["min_iterations"].as<size_type>();
    if (vm.count("max_iterations"))
      m_max_iters = vm["max_iterations"].as<size_type>();
  }

  template<typename T>
  T reflect_value(T const& t) const noexcept
  { return t; }

public:
  runtime_profiler(std::string const& title, int argc = 0, char **argv = 0) :
    m_title(title),
    m_min_iters(DEFAULT_MIN_ITER),
    m_max_iters(DEFAULT_MAX_ITER),
    m_handle(this)
  { get_program_options(argc, argv); }

  explicit runtime_profiler(int argc = 0, char **argv = 0) :
    m_title("unknown"),
    m_min_iters(DEFAULT_MIN_ITER),
    m_max_iters(DEFAULT_MAX_ITER),
    m_handle(this)
  { get_program_options(argc, argv); }

  ~runtime_profiler(void)
  {
    if (m_results.empty())
      return;
    std::size_t max_length = 0;
    if (get_location_id()==0) {
      // print initial information
      std::cout << std::fixed << std::setprecision(8)
                << "benchmark  " << m_title << std::endl;
      if (!m_comment.empty())
        std::cout << m_comment << std::endl;
      std::cout << "processes  " << get_num_processes() << '\n'
                << "locations  " << get_num_locations() << '\n'
                << "iterations " << m_results[0].size() << '\n'
                << m_text << std::endl;
      // find max string
      typename result_container::iterator it =
        std::max_element(m_results.begin(), m_results.end());
      max_length = it->name().length();
      if (max_length<7)
        max_length = 14;
      // print kernel info
      std::cout << "# " << "kernel" << std::string(max_length - 7, ' ')
                << "\ttotal  (s)\tmean   (s)\tmin    (s)\tmax    (s)"
                   "\tstddev     \tconfinterval" << std::endl;
    }

    // print averaged data
    for (typename result_container::const_iterator it = m_results.begin();
          it!=m_results.end();
            ++it) {
      result_type const& r = *it;
      // statistics
      boost::accumulators::accumulator_set<
        value_type,
        boost::accumulators::features<
          boost::accumulators::tag::mean,
          boost::accumulators::tag::min,
          boost::accumulators::tag::max,
          boost::accumulators::tag::variance>
      > acc;
      acc = std::for_each(r.begin(), r.end(), acc);
      std::vector<value_type> vals(6);
      vals[0] = r.get_total();
      vals[1] = boost::accumulators::mean(acc);
      vals[2] = boost::accumulators::min(acc);
      vals[3] = boost::accumulators::max(acc);
      vals[4] = std::sqrt(boost::accumulators::variance(acc));
      // 1.96 is z-table entry for 95%
      vals[5] = (1.96*vals[4]) / std::sqrt(double(r.size()));

      vals = allreduce_rmi(max_vec(),
                           m_handle,
                           &runtime_profiler::template reflect_value<
                             std::vector<value_type>
                           >,
                           vals).get();

      if (get_location_id()==0) {
        // pretty print name and values
        const std::string whitespace(max_length - r.name().length(), ' ');
        std::cout << r.name() << whitespace << '\t'
                  << vals[0] << '\t'
                  << vals[1] << '\t'
                  << vals[2] << '\t'
                  << vals[3] << '\t'
                  << vals[4] << '\t'
                  << vals[5] << std::endl;
      }
    }

    rmi_fence(); // finish printing everything before exiting
  }

  // Adds the given text to the printout
  void add_text(std::string const& s)
  {
    if (m_text.empty())
      m_text = s;
    else
      m_text = m_text + '\n' + s;
  }

  void add_text(const char* s)
  { add_text(std::string(s)); }

  // Warm-ups the system
  template<typename T>
  void warmup(T& wf)
  {
    rmi_fence(); // quiescence before warmup
    for (size_type i=0; i<m_max_iters; ++i)
      wf();
  }

  // Benchmarks the given work function
  template<typename T>
  void benchmark(T& wf, const unsigned int N = 1)
  {
    m_results.push_back(result_type(wf.name(), m_max_iters));
    result_type& r = m_results.back();

    counter_type cnt, total_cnt;

    rmi_fence(); // quiescence before benchmark
    total_cnt.start();
    for (size_type i=0; i<m_max_iters; ++i) {
      cnt.reset();
      cnt.start();
      for (unsigned int n=0; n<N; ++n)
        wf();
      r.push_back(cnt.stop()/N);
    }
    rmi_fence(); // wait for all RMI calls to finish before continuing
    r.set_total(total_cnt.stop());
  }

  void validate(void)
  {
    // 2, 4, 4, 4, 5, 5, 7, 9, mean=5, stddev=2
    m_results.push_back(result_type("validate"));
    result_type& r = m_results.back();
    r.push_back(typename counter_type::value_type(2));
    r.push_back(typename counter_type::value_type(4));
    r.push_back(typename counter_type::value_type(4));
    r.push_back(typename counter_type::value_type(4));
    r.push_back(typename counter_type::value_type(5));
    r.push_back(typename counter_type::value_type(5));
    r.push_back(typename counter_type::value_type(7));
    r.push_back(typename counter_type::value_type(9));
  }
};
template<typename Counter> const typename runtime_profiler<Counter>::size_type
  runtime_profiler<Counter>::DEFAULT_MIN_ITER;
template<typename Counter> const typename runtime_profiler<Counter>::size_type
  runtime_profiler<Counter>::DEFAULT_MAX_ITER;


//////////////////////////////////////////////////////////////////////
/// @brief Returns @c true if STAPL is running in mixed mode.
//////////////////////////////////////////////////////////////////////
inline bool is_in_mixed_mode(void) noexcept
{
  return (get_num_processes()!=process_id(get_num_locations()));
}

} // end namespace stapl

#endif
