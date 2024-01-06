/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <iostream>

void compute_stats(std::string name, std::vector<double>& data) {
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
    if ( d > max ) max = d;
    if ( d < min ) min = d;
  }
  avg /= iterations;
  variance=0.0;
  for ( size_t i = 0; i < iterations; i++) {
    double d = data[i] - avg;
    variance += d*d;
  }
  variance /= iterations;
  stddev = std::sqrt(variance);
  // 1.96 is z-table entry for 95%
  //The formula for min number of iteration is:
  //n=[(1.96*stddev)/(useconfrange*avg)]^2
  //double nit = (1.96*stddev*1.96*stddev)/(0.05 * avg * 0.05 * avg);
  //std::cout<<"Min Number of iterations"<<nit<<"\n";
  confidenceinterval = (1.96*stddev) / std::sqrt((double) iterations);
  if (stapl::get_location_id()==0) {
    std::cout<<name<<"="<<avg<<"\n";
    std::cout<<name<<"_min="<<min<<"\n";
    std::cout<<name<<"_max="<<max<<"\n";
    std::cout<<name<<"_conf="<<confidenceinterval<<"\n\n";
  }
}


// Generator functor used for test_is_partitioned and
// test_partition_point
struct gen_func
{
  typedef size_t    index_type;
  typedef size_t    result_type;

  size_t m_nelem;

  gen_func(size_t nelem)
    : m_nelem(nelem)
  {
    std::srand(stapl::get_location_id());
  }

  result_type operator()(index_type idx) const
  {
    if (m_nelem%2 == 0) {
      if (idx < m_nelem/2) {
        return idx*2 +1;
      } else if (idx == m_nelem/2) {
        return 6;
      } else {
        return idx*2;
      }
    } else {
      if (idx < m_nelem/2) {
        return idx*2 +1;
      } else if (idx == (m_nelem-1)/2) {
        return 6;
      } else {
        return idx*2;
      }
    }
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_nelem);
  }
};

