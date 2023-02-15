/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

////////////////////////////////////////////////////////////////////////////////
/// @file
/// Unit testing / profiling of methods of the stapl::vector and typical views
/// defined over it. std::vector is used for sequential comparison.
////////////////////////////////////////////////////////////////////////////////

#ifndef STAPL_PROFILING_VECTOR_HPP
#define STAPL_PROFILING_VECTOR_HPP

#include "list_vector.hpp"

namespace stapl {

namespace profiling {

template<typename ADT>
void add_vector_profilers(prof_cont_t<ADT>& p,
                          std::string const& name, ADT& adt,
                          std::vector<size_t> const& idx,
                          std::vector<typename ADT::value_type> const& vals,
                          int argc, char** argv)
{
  p.push_back(new push_back_profiler<ADT>(name, &adt, vals, argc, argv));
  p.push_back(new pop_back_profiler<ADT>(name, &adt, vals, argc, argv));
  p.push_back(new add_profiler<ADT>(name, &adt, vals, argc, argv));
  p.push_back(new insert_beg_profiler<ADT>(name, &adt, idx, vals, argc, argv));
  p.push_back(new erase_beg_profiler<ADT>(name, &adt, vals.size(), argc, argv));
}

template<typename T>
void add_vector_profilers(prof_cont_t<std::vector<T>>& p,
                          std::string const& name, std::vector<T>& vec,
                          std::vector<size_t> const& idx,
                          std::vector<T> const& vals,
                          int argc, char** argv)
{
  using ADT = std::vector<T>;

  p.push_back(new push_back_profiler<ADT>(name, &vec, vals, argc, argv));
  p.push_back(new pop_back_profiler<ADT>(name, &vec, vals, argc, argv));
  p.push_back(new insert_beg_profiler<ADT>(name, &vec, idx, vals, argc, argv));
  p.push_back(new erase_beg_profiler<ADT>(name, &vec, vals.size(), argc, argv));
}

} // namespace profiling

} // namespace stapl



#endif // STAPL_PROFILING_VECTOR_HPP
