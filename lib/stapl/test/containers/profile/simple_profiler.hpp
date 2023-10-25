/*
 // Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
 // component of the Texas A&M University System.

 // All rights reserved.

 // The information and source code contained herein is the exclusive
 // property of TEES and may not be disclosed, examined or reproduced
 // in whole or in part without explicit written authorization from TEES.
 */

#ifndef STAPL_PROFILING_SIMPLE_PROFILER_HPP
#define STAPL_PROFILING_SIMPLE_PROFILER_HPP

#include <stapl/runtime.hpp>
#include "profiler_util.h"

namespace stapl {

namespace profiling {

template <typename Counter, typename Predicate>
class simple_profiler
  : public base_profiler<Counter, Predicate>
{
private:
  typedef base_profiler<Counter, Predicate> base_type;

protected:
  size_t n_times;
  bool m_time_per_invocation;

public:
  simple_profiler(std::string inname, int argc = 0, char **argv = NULL)
    : base_type(inname, argc, argv), m_time_per_invocation(false)
  {
    n_times = 1;
    for (int i = 1; i < argc; i++) {
      if (!strcmp("--ntimes", argv[i]))
        n_times = atoi(argv[++i]);
    }
  }

  void report(std::string str)
  {
    if (get_location_id() == 0) {
      std::cout << str << "  ";
      if (m_time_per_invocation) {
        std::cout << this->avg / this->m_test_size << " ";
        std::cout << this->confidenceinterval << " ";
        std::cout << this->min / this->m_test_size << " ";
        std::cout << this->max / this->m_test_size << " ";
        std::cout << this->iterations;
      }
      else {
        std::cout << this->avg << " ";
        std::cout << this->confidenceinterval << " ";
        std::cout << this->min << " ";
        std::cout << this->max << " ";
        std::cout << this->iterations;
      }
      print_extra_information<
        typename Counter::value_type, simple_profiler
      >::print(*this);
      std::cout << "\n";
    }
  }
};

} // namespace profiling

} // namespace stapl

#endif
