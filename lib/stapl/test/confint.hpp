/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_TEST_CONFINT_HPP
#define STAPL_TEST_CONFINT_HPP

#include <string>
#include <sstream>
#include <vector>
#include <stapl/utility/do_once.hpp>

//////////////////////////////////////////////////////////////////////
/// @brief Statistics computed from a set of samples
///
/// getters/setters are not provided because the struct is used only
/// in tests to collect timing information.
//////////////////////////////////////////////////////////////////////
struct stat_t
{
  double avg, min, max, stddev, variance, conf_interval;
  int num_samples;

  stat_t()
    : avg(0.), min(0.), max(0.), stddev(0.), variance(0.), conf_interval(0.),
      num_samples(0)
  { }
};


//////////////////////////////////////////////////////////////////////
/// @brief Compute statistics of a set of samples collected by repeated
/// execution of a test.
/// @param samples Timing information collected for a test
//////////////////////////////////////////////////////////////////////
stat_t compute_stats(std::vector<double> const& samples)
{
  typedef std::vector<double> cont_t;

  // t-table entries for 95% confidence interval
  const cont_t t_vals
    { 12.70620, 4.30265, 3.18245, 2.77645, 2.57058, 2.44691, 2.36462, 2.30600,
       2.26216, 2.22814, 2.20099, 2.17881, 2.16037, 2.14479, 2.13145, 2.11991,
       2.10982, 2.10092, 2.09302, 2.08596, 2.07961, 2.07387, 2.06866, 2.06390,
       2.05954, 2.05553, 2.05183, 2.04841, 2.04523, 2.04227 };
  const double inf_t_vals = 1.95996;

  stat_t result;
  result.num_samples = samples.size();

  //initialize min and max
  result.min = result.max = *(samples.begin());

  result.avg = 0.;
  for (cont_t::const_iterator i = samples.begin(); i != samples.end(); ++i) {
    result.avg += *i;
    if (*i < result.min) result.min = *i;
    if (*i > result.max) result.max = *i;
  }
  result.avg /= result.num_samples;

  result.variance = 0.;
  for (cont_t::const_iterator i = samples.begin(); i != samples.end(); ++i)
    result.variance += std::pow(*i-result.avg, static_cast<double>(2.0));
  result.variance /= result.num_samples;
  result.stddev = sqrt(result.variance);
  double t_val = result.num_samples > 30 ?
    inf_t_vals : t_vals[result.num_samples-1];
  result.conf_interval =
    (t_val*result.stddev) / sqrt(static_cast<double>(result.num_samples));

  return result;
}


//////////////////////////////////////////////////////////////////////
/// @brief Function used to dump stats information given a name
//////////////////////////////////////////////////////////////////////
void report(std::string const& name, std::vector<double> const& samples,
            bool passed)
{
  stat_t stats = compute_stats(samples);
  std::stringstream ostr;
  ostr << "Test : " << name << "\n";
  ostr << "Version : STAPL\n";
  ostr << "Status : ";
  if (passed)
    ostr << "PASS\n";
  else
    ostr << "FAIL\n";
  ostr << "Time : " << stats.avg << "\n";
  ostr << "Notes : (ci, min, max, stddev, num samples) ";
  ostr << stats.conf_interval << " " << stats.min << " " << stats.max << " "
       << stats.stddev << " " << stats.num_samples << "\n";
  stapl::do_once([&](void) {
    fprintf(stderr,"%s",ostr.str().c_str());
  });
}


//////////////////////////////////////////////////////////////////////
/// @brief Function used to report times that is used by tests.
///
/// Output format is that expected by testing and timing framework and
/// assumes stream already contains test name, version, and status information.
//////////////////////////////////////////////////////////////////////
void report(std::stringstream& out, std::vector<double> const& samples)
{
  stat_t stats = compute_stats(samples);
  out << "Time : " << stats.avg << "\n"
      << "Note : "
      << "conf int " << stats.conf_interval
      << " min " << stats.min << " max " << stats.max
      << " stddev " << stats.stddev << " samples " << stats.num_samples
      << "\n";
}


//////////////////////////////////////////////////////////////////////
/// @brief Provides the ability to control repeated execution of a test
/// based on the statistics of the observed execution times.
//////////////////////////////////////////////////////////////////////
struct confidence_interval_controller
{
protected:
  typedef std::vector<double> cont_t;

  /// Collection of samples observed
  cont_t m_samples;

  /// Minimum and maximum bounds on the number of samples to be collected.
  size_t m_min_iters, m_max_iters;

  /// @brief Percentage of the mean that the standard deviation must be
  /// less than before iteration can stop
  double m_tolerance;

  /// Statistics of the set of collected samples
  stat_t m_stats;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Initialize the controller with the bounds on the number of
  /// samples to collect and the desired percentage of the mean the standard
  /// deviation will fall below before iteration completes.
  ///
  /// @param min minimum number of samples to collect
  /// @param max maximum number of samples to collect
  /// @param tol percentage of mean which the standard deviation must fall
  /// below before iteration can stop
  //////////////////////////////////////////////////////////////////////
  confidence_interval_controller(size_t min = 5, size_t max = 50,
                                 double tol = 0.05)
    : m_min_iters(min), m_max_iters(max), m_tolerance(tol)
  { m_samples.reserve(m_min_iters); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a boolean indicating whether sample collection must
  /// continue or not.
  //////////////////////////////////////////////////////////////////////
  bool iterate(void)
  {
    if (m_samples.size() < m_min_iters)
      return true;

    m_stats = compute_stats(m_samples);

    if (m_samples.size() >= m_max_iters)
      return false;

    if (m_stats.conf_interval > m_tolerance*m_stats.avg)
      return true;
    else
      return false;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Receive a collected sample
  /// @param t Time reported by a timer
  //////////////////////////////////////////////////////////////////////
  void push_back(double const& t)
  { m_samples.push_back(t); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Reset the controller by clearing the vector of samples.
  //////////////////////////////////////////////////////////////////////
  void reset(void)
  { m_samples.clear(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return controller statistics for reporting in other formats
  //////////////////////////////////////////////////////////////////////
  stat_t stats(void)
  {
    m_stats = compute_stats(m_samples);
    return m_stats;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Print test status and controller statistics using the name
  ///   provided as the label.
  /// @param name label used in output to make statistics identifiable
  /// @param status test status (passed/failed)
  //////////////////////////////////////////////////////////////////////
  void report(std::string const& name, bool status = true)
  { ::report(name, m_samples, status); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Print statistics to the stream provided.
  /// @param out stream that contains test name, version, and statuss
  /// information.
  ///
  /// This method is used by the testing framework.
  //////////////////////////////////////////////////////////////////////
  void report(std::stringstream& out)
  { ::report(out, m_samples); }
};

#endif
