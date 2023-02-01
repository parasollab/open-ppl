/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#include "p_container_algo_profiler.hpp"
#include "value_type_util.h"
#include "profiler_util.h"

using namespace stapl;

////////////////////////////////////////////////////////////////////////////////
/// @brief Invokes all of the algorithm profilers.
///
/// @param name A string containing the name of the container used for testing
/// @param p The container to be used for testing
/// @param vw A view of the container
/// @param N The size of the container and the upper bound for random number
/// generation
////////////////////////////////////////////////////////////////////////////////
template <class pC, class View>
void profile(std::string name, pC& p, View& vw, size_t N, int argc, char** argv)
{
  p_generate_profiler<pC, View, counter_type > pgp(name,&p, vw, N, argc, argv);
  pgp.collect_profile();
  pgp.report();

  p_find_profiler<pC, View, counter_type > pfp(name,&p, vw, N+2000, argc, argv);
  pfp.collect_profile();
  pfp.report();

  p_for_each_profiler<pC, View, sum_op<typename pC::value_type>, counter_type>
    pfep(name, &p, vw, sum_op<typename pC::value_type>(), argc, argv);
  pfep.collect_profile();
  pfep.report();

  p_accumulate_profiler<pC, View, counter_type > pap(name,&p,vw, 0, argc, argv);
  pap.collect_profile();
  pap.report();

  p_partial_sum_profiler<pC, View, counter_type > pps(name,&p,vw,0, argc, argv);
  pps.collect_profile();
  pps.report();

  pC tmp(N);
  View vw2(tmp);
  p_copy_profiler<pC, counter_type > pcp(name, &p, &tmp, vw, vw2, argc, argv);
  pcp.collect_profile();
  pcp.report();
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Invokes all of the sequential algorithm profilers.
///
/// @param name A string containing the name of the container used for testing
/// @param p The container to be used for testing
/// @param N The size of the container and the upper bound for random number
/// generation
////////////////////////////////////////////////////////////////////////////////
template <class pC>
void profile_seq(std::string name, pC& p, size_t N, int argc, char** argv)
{
  generate_profiler<pC, counter_type > pgp(name, &p, N, argc, argv);
  pgp.collect_profile();
  pgp.report();

  find_profiler<pC, counter_type > pfp(name, &p, N+2000, argc, argv);
  pfp.collect_profile();
  pfp.report();

  for_each_profiler<pC, sum_op<typename pC::value_type>, counter_type>
    pfep(name, &p, sum_op<typename pC::value_type>(), argc, argv);
  pfep.collect_profile();
  pfep.report();

  accumulate_profiler<pC, counter_type > pap(name, &p, 0, argc, argv);
  pap.collect_profile();
  pap.report();

  partial_sum_profiler<pC, counter_type > pps(name, &p, 0, argc, argv);
  pps.collect_profile();
  pps.report();

  pC tmp(N);
  copy_profiler<pC, counter_type > pcp(name, &p, &tmp, argc, argv);
  pcp.collect_profile();
  pcp.report();
}
