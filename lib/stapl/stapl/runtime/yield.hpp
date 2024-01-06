/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_YIELD_HPP
#define STAPL_RUNTIME_YIELD_HPP

#include "context.hpp"
#include "runqueue.hpp"
#include "tags.hpp"
#include <utility>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Yields the location to execute some requests.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
void scheduling_point(context&);


//////////////////////////////////////////////////////////////////////
/// @brief Yields until @p pred returns @c true.
///
/// This function will flush aggregated requests if @p pred returns @c false.
///
/// @return @c true if it yielded, otherwise @c false.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename Predicate>
bool yield_until(context& ctx, Predicate&& pred)
{
  if (pred())
    return false;
  ctx.flush_requests();
  runqueue::wait(std::forward<Predicate>(pred));
  return true;
}


//////////////////////////////////////////////////////////////////////
/// @brief Yields until @p pred returns @c true.
///
/// This function does not flush aggregated requests.
///
/// @return @c true if it yielded, otherwise @c false.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename Predicate>
bool yield_until(const no_context_t, Predicate&& pred)
{
  if (pred())
    return false;
  runqueue::wait(std::forward<Predicate>(pred));
  return true;
}


//////////////////////////////////////////////////////////////////////
/// @brief Yields until @p pred returns @c true.
///
/// This function will flush aggregated requests if a @ref context exists and
/// @p pred returns @c false.
///
/// @return @c true if it yielded, otherwise @c false.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename Predicate>
bool yield_until(Predicate&& pred)
{
  if (pred())
    return false;
  context* const ctx = this_context::try_get();
  if (ctx)
    ctx->flush_requests();
  runqueue::wait(std::forward<Predicate>(pred));
  return true;
}


//////////////////////////////////////////////////////////////////////
/// @brief Yields if @p pred does not return @c true.
///
/// This function will flush aggregated requests if @p pred returns @c false.
///
/// @return The result of @p pred.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename Predicate>
bool yield_if_not(context& ctx, Predicate&& pred)
{
  if (pred())
    return true;
  ctx.flush_requests();
  runqueue::yield();
  return pred();
}


//////////////////////////////////////////////////////////////////////
/// @brief Yields if @p pred does not return @c true.
///
/// This function does not flush aggregated requests.
///
/// @return The result of @p pred.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename Predicate>
bool yield_if_not(const no_context_t, Predicate&& pred)
{
  if (pred())
    return true;
  runqueue::yield();
  return pred();
}


//////////////////////////////////////////////////////////////////////
/// @brief Yields if @p pred does not return @c true.
///
/// This function will flush aggregated requests if a @ref context exists and
/// @p pred returns @c false.
///
/// @return The result of @p pred.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename Predicate>
bool yield_if_not(Predicate&& pred)
{
  if (pred())
    return true;
  context* const ctx = this_context::try_get();
  if (ctx)
    ctx->flush_requests();
  runqueue::yield();
  return pred();
}

} // namespace runtime

} // namespace stapl

#endif
