/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_BENCHMARKS_KERNELS_RUNTIME_BASE_PROFILER_HPP
#define STAPL_BENCHMARKS_KERNELS_RUNTIME_BASE_PROFILER_HPP

#include <stapl/runtime.hpp>
#include <stapl/profiler/base_profiler.hpp>
#include <cstring>
#include <iostream>
#include <iomanip>

//////////////////////////////////////////////////////////////////////
/// @brief Base profiler implementation.
///
/// @ingroup performanceMonitor
//////////////////////////////////////////////////////////////////////
template<typename Counter = stapl::counter<stapl::default_timer>>
class runtime_base_profiler
: public stapl::base_profiler<Counter>
{
private:
  typedef stapl::base_profiler<Counter> base_type;

public:
  using base_type::report;

  explicit runtime_base_profiler(std::string const& inname,
                                 int argc = 0, char **argv = nullptr)
  : base_type(inname, "STAPL", argc, argv)
  { }

  void report(std::stringstream& ss)
  {
    if (stapl::get_location_id()==0) {
      ss << std::fixed << std::setprecision(8);
      ss << this->get_name()       << '\t'
         << this->get_iterations() << '\t'
         << this->get_avg()        << '\t'
         << this->get_min()        << '\t'
         << this->get_max()        << '\t'
         << this->get_conf()       << std::endl;
    }
  }
};


inline void print_parallelism(std::ostream& os)
{
  if (stapl::get_location_id()==0) {
    os << stapl::get_num_locations() << ' ';
    if (stapl::get_available_levels()>0)
      stapl::execute([&os] { print_parallelism(os); });
  }
}

#endif
