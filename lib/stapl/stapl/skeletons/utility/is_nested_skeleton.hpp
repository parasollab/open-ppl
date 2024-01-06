/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_UTILITY_IS_NESTED_SKELETON_HPP
#define STAPL_SKELETONS_UTILITY_IS_NESTED_SKELETON_HPP

#include <stapl/skeletons/transformations/wrapped_skeleton_fwd.hpp>
#include <stapl/skeletons/transformations/transform.hpp>

namespace stapl {
namespace skeletons {


template <typename WF>
struct is_nested_skeleton
  : public std::false_type
{ };

template <typename S, typename Tag, typename... Args>
struct is_nested_skeleton<
         transformations::transformations_impl::transform<S, Tag, Args...>>
  : public is_nested_skeleton<
             typename transformations::transformations_impl::
               transform<S, Tag, Args...>::base_type>
{ };

template <typename S, bool B1, bool B2, typename ExecutionParams>
struct is_nested_skeleton<
         wrapped_skeleton<S, tags::nested_execution<B1>, ExecutionParams, B2>>
  : public std::true_type
{ };


template <typename WF>
struct is_interpg_skeleton
  : public std::false_type
{ };


template <typename S, typename Tag, typename... Args>
struct is_interpg_skeleton<
         transformations::transformations_impl::transform<S, Tag, Args...>>
  : public is_interpg_skeleton<
             typename transformations::transformations_impl::
               transform<S, Tag, Args...>::base_type>
{ };

template <typename S, bool B1, bool B2, typename ExecutionParams>
struct is_interpg_skeleton<
         wrapped_skeleton<S, tags::nested_execution<B1>, ExecutionParams, B2>>
  : public std::integral_constant<bool, !B1>
{ };


} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_UTILITY_IS_NESTED_SKELETON_HPP
