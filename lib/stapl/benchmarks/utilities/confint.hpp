/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <cmath>
#include <cstdio>
#include <iostream>
#include <string>
#include <cmath>
#include <vector>

#ifdef _STAPL
#include <stapl/runtime.hpp>
#include <stapl/runtime/counter/default_counters.hpp>

typedef stapl::counter<stapl::default_timer> counter_t;
#endif

struct stat_t
{
  double avg, min, max, stddev, variance, conf_interval;
  int num_samples;

  stat_t()
    : avg(0.), min(0.), max(0.), stddev(0.), variance(0.), conf_interval(0.),
      num_samples(0)
  { }
};


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


void report(std::string const& name, std::vector<double> const& samples)
{
  stat_t stats = compute_stats(samples);

#ifdef _STAPL
  if (stapl::get_location_id() == 0)
#endif
    std::cout << name << " " << stats.avg << " " << stats.conf_interval << " "
              << stats.min << " " << stats.max << " " << stats.stddev << " "
              << stats.num_samples << std::endl;
}


struct confidence_interval_controller
{
private:
  typedef std::vector<double> cont_t;

  cont_t                      samples;
  size_t                      min_iters;
  size_t                      max_iters;
  double                      tolerance;
  stat_t                      statistics;

public:
  confidence_interval_controller(size_t min = 5, size_t max = 50,
                                 double tol = 0.05)
    : min_iters(min), max_iters(max), tolerance(tol)
  {
    samples.reserve(min_iters);
  }

  bool iterate()
  {
    if (samples.size() < min_iters)
      return true;
    else if (samples.size() >= max_iters)
      return false;

    statistics = compute_stats(samples);

    return compute_stats(samples).conf_interval > tolerance*statistics.avg;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return controller statistics for reporting in other formats
  ///
  /// @warning This method may only be used in conjunction with iterate.
  //////////////////////////////////////////////////////////////////////
  stat_t stats(void) const
  { return statistics; }


  void push_back(double const& t)
  {
   samples.push_back(t);
   statistics = compute_stats(samples);
  }

  size_t iterations_run(void)
  {
    return samples.size();
  }

  void report(std::string const& name)
  {
    ::report(name, samples);
  }

  double average(void) const
  {
    return statistics.avg;
  }

  double confidence_interval(void) const
  {
    return statistics.conf_interval;
  }
};


