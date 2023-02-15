/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef NESPAR_UTILITIES
#define NESPAR_UTILITIES

#include "../testutil.hpp"

#include "../rel_alpha_data.h"

#include <stapl/stream.hpp>
#include <stapl/set.hpp>
#include <stapl/vector.hpp>

typedef stapl::set<long long int> set_int;
typedef stapl::set_view<set_int> set_int_vw;

typedef stapl::vector<set_int> vec_set_int;
typedef stapl::vector_view<vec_set_int> vec_set_int_vw;

typedef stapl::identity<int> id_int_wf;
typedef stapl::identity<size_t> id_un_wf;
typedef stapl::negate<int> neg_int_wf;

typedef stapl::plus<int> add_int_wf;
typedef stapl::minus<int> sub_int_wf;
typedef stapl::min<int> min_int_wf;
typedef stapl::max<int> max_int_wf;

typedef stapl::bit_xor<size_t> xor_un_wf;
typedef stapl::bit_or<size_t> ior_un_wf;
typedef stapl::bit_and<size_t> and_un_wf;


namespace nestpar_perf {

//////////////////////////////////////////////////////////////////////
/// @brief Work function which returns the maximum of each index of a vector
//////////////////////////////////////////////////////////////////////
struct max_vec_wf
{
  typedef std::vector<double> result_type;

  result_type operator()(result_type x, result_type y) const
  {
    stapl_assert(x.size() == y.size(),
      "Cannot merge 2 vectors of different sizes");
    std::vector<double> maxed(x.size());
    for (size_t i = 0; i < x.size(); i++) {
      if (x[i] < y[i])
       maxed[i] = y[i];
      else
       maxed[i] = x[i];
    }

   return maxed;
  }
};

template <typename T>
struct roll_wf
{

  T m_object_to_copy;

  roll_wf(T const& obj)
    : m_object_to_copy(obj)
  { }

  typedef void result_type;

  template <typename T1>
  void operator()(T1 e)
  {
    e = m_object_to_copy;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_object_to_copy);
  }
};


}// namespace

#endif
