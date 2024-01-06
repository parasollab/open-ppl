/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <iostream>
#include <stapl/containers/graph/graph.hpp>

#ifndef TEST_UTIL_H
#define TEST_UTIL_H

void stapl_print(const char* s)
{
  stapl::do_once([s](void)->void
                 { std::cout << s << std::flush; });
}

void check_reset_error(bool& err) {
  if (err)
    stapl_print("Failed\n");
  else
    stapl_print("Passed\n");
  err = false;
}

double compute_stats(std::string name, std::vector<double>& data) {
  //The predicate can be different that m_pred here if we want
  //statistics on other fields of the counter; default will be on
  //time
  double avg, max,min,variance,stddev,confidenceinterval;
  size_t iterations = data.size();
  avg=0.0;
  min=max=data[0];
  for ( size_t i = 0; i < iterations; i++) {
    double d = data[i];
    avg += d;
    if ( d > max )
      max = d;
    if ( d < min )
      min = d;
  }
  avg /= iterations;
  variance=0.0;
  for ( size_t i = 0; i < iterations; i++) {
    double d = data[i] - avg;
    variance += d*d;
  }
  variance /= iterations;
  stddev = sqrt(variance);
  // 1.96 is z-table entry for 95%
  //The formula for min number of iteration is
  // n=[(1.96*stddev)/(useconfrange*avg)]^2
  //double nit = (1.96*stddev*1.96*stddev)/(0.05 * avg * 0.05 * avg);
  //std::cout<<"Min Number of iterations"<<nit<<"\n";
  confidenceinterval = (1.96*stddev) / sqrt((double) iterations);
  stapl::do_once([name, avg, min, max, confidenceinterval](void)->void
                 { std::cout << name << "=" << avg <<"\n";
                   std::cout << name << "_min=" << min <<"\n";
                   std::cout << name << "_max=" << max <<"\n";
                   std::cout << name << "_conf=" << confidenceinterval<<"\n\n";
                 });
  return avg;
}

#endif
