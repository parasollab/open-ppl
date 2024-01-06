/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARKS_FMM_STAPL_ENTRY_H
#define STAPL_BENCHMARKS_FMM_STAPL_ENTRY_H

#include <stapl/utility/do_once.hpp>

#include <stapl/stream.hpp>
#include <stapl/runtime.hpp>

#include <stapl/array.hpp>
#include <stapl/vector.hpp>
#include <stapl/views/proxy/proxy.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/views/metadata/coarseners/all_but_last.hpp>
#include <stapl/views/metadata/coarseners/multiview.hpp>
#include <stapl/skeletons/functional/map.hpp>
#include <stapl/skeletons/functional/allreduce.hpp>
#include <stapl/skeletons/functional/alltoall.hpp>
#include <stapl/skeletons/functional/sink_value.hpp>
#include <stapl/skeletons/functional/broadcast_to_locs.hpp>
#include <stapl/skeletons/map.hpp>
#include <stapl/skeletons/executors/execute.hpp>
#include <stapl/skeletons/environments/graphviz_env.hpp>
#include <stapl/algorithms/algorithm_fwd.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>
#include <stapl/skeletons/functional/notify_map.hpp>
#include <boost/shared_ptr.hpp>
#include <stapl/skeletons/utility/lightweight_vector.hpp>
#include <stapl/skeletons/functional/allgather.hpp>
#include <iostream>
#include <typeinfo>


#define STRONG_SCALE(BODIES, SIZE) (BODIES)
#define WEAK_SCALE(BODIES, SIZE) (BODIES*SIZE)

#define STRONG_SCALE_RANK(BODIES, SIZE) (BODIES/SIZE)
#define WEAK_SCALE_RANK(BODIES, SIZE) (BODIES)
#define PRINT 0


using namespace std;
class EntrySTAPL
{
public:
  size_t rank;
  size_t size;

  EntrySTAPL()
  {
    rank = stapl::get_location_id();
    size = stapl::get_num_locations();
  }
};


template <typename V1, typename V2 = typename std::decay<V1>::type>
struct is_stapl_proxy
  : public false_type
{ };

template <typename V1, typename T, typename A>
struct is_stapl_proxy<V1, stapl::proxy<T, A>>
  : public true_type
{ };

#endif // STAPL_BENCHMARKS_FMM_STAPL_ENTRY_H
