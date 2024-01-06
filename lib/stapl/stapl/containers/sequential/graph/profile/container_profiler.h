/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_SEQUENTIAL_GRAPH_CONTAINER_PROFILER_HPP
#define STAPL_CONTAINERS_SEQUENTIAL_GRAPH_CONTAINER_PROFILER_HPP

#include <stapl/profiler/base_profiler.hpp>

namespace stapl {

/**
 * Container profiler class; It can be customized with the Counter
 * and predicate to extract a metric from the value type of the
 * counter;
 */
template <class C, class Counter>
class container_profiler
  : public base_profiler<Counter>
{
private:
  typedef base_profiler<Counter> base_type;
protected:
  C* m_c;
  size_t n_times;

public:
  using base_type::report;

  container_profiler(C* c, std::string inname,
                     int argc = 0, char **argv = 0)
    : base_type(inname, "STAPL", argc, argv),
      m_c(c)
  {
    n_times = 1;
    for( int i = 1; i < argc; i++) {
      if( !strcmp("--ntimes", argv[i]))
        n_times=atoi(argv[++i]);
    }
  }

  void report(std::stringstream& ss)
  {
    ss << "Test:" << this->name << "\n";
    ss << this->name << "_iterations=" << this->get_iterations() << "\n";
    ss << this->name << "_num_invocations_per_iteration=" << this->n_times
      << "\n";
    ss << this->name << "=" << this->get_avg() << "\n";
    ss << this->name << "_conf=" << this->get_conf() << "\n";
    ss << this->name << "_min=" << this->get_min() << "\n";
    ss << this->name << "_max=" << this->get_max() << "\n";

    this->print_extra_information(ss);

    ss << "\n";
  }
};

}//end namespace stapl

#endif
