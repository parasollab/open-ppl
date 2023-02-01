/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_PROFILE_BASE_PROFILER_HPP
#define STAPL_PROFILE_BASE_PROFILER_HPP

#ifdef _STAPL
#include <stapl/runtime.hpp>
#include <stapl/algorithms/functional.hpp>
#endif
#include <algorithm>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <sys/time.h>

#include "../../test/confint.hpp"
#include <boost/lexical_cast.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Print a message preceded by a time stamp.
///
/// Useful for coarse estimation of the time elapsed between two consecutive
/// invocations.
///
/// @ingroup profiling
//////////////////////////////////////////////////////////////////////
template <typename S>
void ts_print(S const& s)
{
#ifdef _STAPL
    if (get_location_id()!=0)
      return;
#endif
  struct timeval tv;
  gettimeofday(&tv,NULL);
  std::cout << "[" << tv.tv_sec << "," << tv.tv_usec << "]" << s;
}

//////////////////////////////////////////////////////////////////////
/// @brief Base class to built upon by programs that want to collect profile
///        information about certain functions.
///
/// Backed by @ref confidence_interval_controller to control the iteration until
/// desired confidence level has been reached, compute sample statistics and
/// report the results.
///
/// @tparam Counter Counter type used.
///
/// @todo Add command line option to specify the fraction of the profiling
/// size to be used for validation.
///
/// @todo Propagate Counter::value_type to confidence_interval_controller
///   (currently assumes that Counter::value_type is double).
///
/// @ingroup profiling
//////////////////////////////////////////////////////////////////////
template <typename Counter>
class base_profiler
#ifdef _STAPL
  : public p_object,
#endif
    protected confidence_interval_controller
{
private:
  typedef typename Counter::value_type metric_type;

protected:
  /// Basic name of the profiler.
  std::string                     name;

  /// Version of the profiler (e.g., STAPL, STL, etc.)
  std::string                     version;

  /// Starting time of this profiling run.
  time_t                          m_start_time;

  /// Total time taken by this profiling run.
  time_t                          m_wallclock_sec;

  /// Maximum allowed time for this profiling run before terminating it.
  std::size_t                     m_max_wallclock_sec;

  /// Cache size in MB to be flushed before every iteration.
  std::size_t                     m_cache_size;

  /// Temporary variable to prevent the compiler optimizing out cache flushing.
  std::size_t                     m_cache_tmp;

  /// Prefix for the filename to dump the profiler report into.
  std::string                     m_file_name_prefix;

  /// Performance counter.
  Counter                         m_counter;

  /// Flag indicating the validation status (@sa validate())
  bool                            m_passed;

  /// Helper function to facilitate RMIs used when collecting the results.
  /// @{
  metric_type reflect_metric(metric_type const& m) const
  { return m; }

  template<typename T>
  T reflect_value(T x) const { return x; }
  /// @}

  //////////////////////////////////////////////////////////////////////////////
  /// @brief Profilers may use multiple counters to collect different
  /// pieces of information, such as L2 or L3 cache misses; these may be
  /// reported in addition to the default counter (usually time) by overriding
  /// this method.
  //////////////////////////////////////////////////////////////////////////////
  virtual void print_extra_information(std::stringstream& ss)
  { }

public:
  base_profiler(std::string inname="untitled", std::string ver="STAPL",
                int argc=0, char** argv=nullptr)
    : confidence_interval_controller(1, 50, 0.),
      name(inname), version(ver),
      m_start_time(0), m_wallclock_sec(0), m_max_wallclock_sec(0),
      m_cache_size(0), m_cache_tmp(0),
      m_file_name_prefix(""),
      m_passed(true)
  {
    set_options(argc, argv);
  }

  virtual ~base_profiler(void) = default;

  unsigned int get_iterations(void) const
  { return m_samples.size(); }

  double get_avg(void) const
  { return m_stats.avg; }

  double get_min(void) const
  { return m_stats.min; }

  double get_max(void) const
  { return m_stats.max; }

  double get_std(void) const
  { return m_stats.stddev; }

  double get_conf(void) const
  { return m_stats.conf_interval; }

  std::string const& get_name(void) const
  { return name; }

  bool get_validation_result() const
  { return m_passed; }

  virtual void initialize(void)
  {
    //start the wall clock in case we don't want to run more than
    //a certain ammount of time
    m_start_time=time(NULL);
  }

  virtual void initialize_iteration(void)
  { }

  virtual void run(void) = 0;

  virtual void finalize_iteration(void)
  { }

  virtual bool continue_iterating(void)
  {
    time_t now = time(NULL);

    if (m_max_wallclock_sec > 0 &&
        std::size_t(now-m_start_time) > m_max_wallclock_sec)
      return false; // stop

    return confidence_interval_controller::iterate();
  }

  virtual void finalize(void)
  {
    time_t now = time(NULL);
    m_wallclock_sec=now-m_start_time;
  }

  //////////////////////////////////////////////////////////////////////////////
  /// @brief Method used in the general validation run (@ref validate()) to
  ///   actually check for the validity of the results and set #m_passed to
  ///   appropriate value.
  ///
  /// Specialized profilers may customize this method for simple validity
  /// checking based on the results of a single profiling run. If different
  /// code than the one used for profiling purposes has to be executed to obtain
  /// results for validation, the full @ref validate() method has to be
  /// overriden.
  ///
  /// @note #m_passed will be set locally -- a reduction will be typically
  ///   required to determine the global validity.
  //////////////////////////////////////////////////////////////////////////////
  virtual void check_validity(void)
  { }

  //////////////////////////////////////////////////////////////////////////////
  /// @brief General validation run sequence.
  //////////////////////////////////////////////////////////////////////////////
  virtual void validate(void)
  {
    /// General profiler initialization.
    initialize();

    /// Initialize the single profiling iteration.
    initialize_iteration();

    /// Make sure all RMIs have finished before running the experiment.
    rmi_fence();

    /// Run the profiling iteration to collect results used for validation.
    run();

    /// Make sure all RMIs have finished before checking the results.
    rmi_fence();

    /// Check the validity of results, set #m_passed to true/false accordingly.
    check_validity();

    /// Make sure all RMIs have finished to finalize the results checking.
    rmi_fence();

    /// Peform any per-iteration cleanup operations.
    finalize_iteration();

    /// Finalize the profiler to bring it to a pristine state again before
    /// running the actual profiling (complementaty to @ref initialize()).
    finalize();
  }

  virtual void collect_profile(void)
  {
    initialize();

    // for flushing the cache if specified
    m_cache_tmp = 0;
    const std::size_t N = (m_cache_size * 1024 * 1024);
    std::vector<char> tdata(N);
    for (std::size_t i=0; i<N; ++i)
      tdata[i] = char(rand() % 3);

    while (continue_iterating())
    {
      initialize_iteration();

#ifdef _STAPL
      rmi_fence();
#endif

      // flush cache each iteration if specified
      if (m_cache_size != 0) {
        for (std::size_t i=0; i<N; ++i) {
          // make sure to save the total because the compiler may optimize
          // away this code (note that this may not be enough with newer
          // compilers)
          m_cache_tmp += tdata[i];
        }
      }

      // Run the actual profiled code.
      m_counter.reset();
      m_counter.start();
      run();
      metric_type metric_val = m_counter.stop();

      // Get the profiling metric maximum across all locations
#ifdef _STAPL
      metric_type max_metric_val = allreduce_rmi(
        [](metric_type const& x, metric_type const& y) {
          return metric_type{std::max(x,y)};
        },
        this->get_rmi_handle(), &base_profiler::reflect_metric, metric_val
      ).get();
#else
      metric_type max_metric_val = metric_val;
#endif

      // Add the obtained metric value as a new sample to the profile collector
      confidence_interval_controller::push_back(max_metric_val);

      finalize_iteration();
    }

    finalize();
  }

  virtual void report(std::stringstream& ss)
  {
#ifdef _STAPL
    if (this->get_location_id() != 0)
      return;

    bool passed = reduce_rmi(stapl::logical_and<bool>(),
      this->get_rmi_handle(),
      &base_profiler::reflect_value<bool>, this->m_passed).get();
#else
    bool passed = this->m_passed;
#endif


    ss << "Test : " << name << "\n";
    ss << "Version : " << version << "\n";
    ss << "Status : " << (passed ? "PASS" : "FAIL") << "\n";

    confidence_interval_controller::report(ss);

    print_extra_information(ss);

    ss << "\n";
  }

  void report(void)
  {
#ifdef _STAPL
    if (this->get_location_id() != 0)
      return;
#endif

    std::stringstream ss;
    report(ss);

    if (!m_file_name_prefix.empty())
    {
      std::ostringstream os;
#ifdef _STAPL
      os << "_" << this->get_num_locations() << "_" << this->get_location_id();
#endif
      const std::string fname = m_file_name_prefix + "_" + name + os.str();

      std::ofstream of(fname.c_str(), std::ios::out);

      of << ss.str();
    }

    std::cerr << ss.str();
  }

  void set_options(int argc, char **argv)
  {
    for ( int i = 1; i < argc; i++) {
      if ( !strcmp("--miniterations", argv[i])) {
        m_min_iters = boost::lexical_cast<size_t>(argv[++i]);
        m_max_iters = std::max(m_min_iters, m_max_iters);
      }
      else if ( !strcmp("--flushcache", argv[i])) {
        m_cache_size = boost::lexical_cast<size_t>(argv[++i]);
      }
      else if ( !strcmp("--maxiterations", argv[i])) {
        m_max_iters = boost::lexical_cast<size_t>(argv[++i]);
      }
      else if ( !strcmp("--file", argv[i])) {
        m_file_name_prefix = argv[++i];
      }
      else if ( !strcmp("--maxwallclock", argv[i])) {
        m_max_wallclock_sec = boost::lexical_cast<size_t>(argv[++i]);
      }
      else if ( !strcmp("--useconfrange", argv[i])) {
        // this is a bit more complicated since the range is optional; first
        // check if there is another argument...
        if ( (++i < argc) ) {
          // ... and then if it is an integer, use it to define the confidence
          // interval width
          try {
            m_tolerance = 0.01 * boost::lexical_cast<size_t>(argv[i]);
          }
          catch (boost::bad_lexical_cast const&) {
            m_tolerance = 0.05;
            --i;
          }
        }
      }
    }
  }
};

void print_base_profiler_cmdline_help(std::ostream& os)
{
#ifdef _STAPL
  if (get_location_id()!=0)
    return;
#endif
  os << "Displaying the base profiler options.\n"
     << "--flushcache #cache_size (MB)\n"
     << "--miniterations #min\n"
     << "--maxiterations #max\n"
     << "--file fname  The prefix of the file where all raw results will be"
        " written (if not specified, std::cerr will be used). \n"
     << "--maxwallclock #secs  Maximum allowed time; evaluated between"
        " iterations\n"
     << "--useconfrange #p  The width of the 95% confidence interval is p%"
        " around the mean - the iteration will continue until the standard"
        " deviation falls below p% of the mean or maximum number of"
        " iterations is reached. Default: p = 5\n\n";
}

} // namespace stapl

#endif
