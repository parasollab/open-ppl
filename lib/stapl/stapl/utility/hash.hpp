/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_HASH_HPP
#define STAPL_UTILITY_HASH_HPP

#include "hash_fwd.hpp"
#include <stapl/utility/tuple.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Default hash function for sequential hash containers.
/// @ingroup utility
//////////////////////////////////////////////////////////////////////
template<typename T>
struct hash
  : public boost::hash<T>
{ };


namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Recursive, static function to call @p boost::hash_combine()
///  on each member of the input tuple and update @p seed parameter
///  accordingly.
//////////////////////////////////////////////////////////////////////
template<typename Tuple, int N = tuple_size<Tuple>::value-1>
struct tuple_hash_impl
{
  void static apply(std::size_t& seed, Tuple const& t)
  {
    tuple_hash_impl<Tuple, N-1>::apply(seed, t);

    boost::hash_combine(seed, get<N>(t));
  }
};


template<typename Tuple>
struct tuple_hash_impl<Tuple, 0>
{
  void static apply(std::size_t& seed, Tuple const& t)
  {
    boost::hash_combine(seed, get<0>(t));
  }
};

} // namespace detail


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of hash for tuple. Use @p boost::hash_combine()
///   to incorporate all elements of tuple into hash value.
//////////////////////////////////////////////////////////////////////
template<typename ...Args>
struct hash<tuple<Args...>>
{
  std::size_t operator()(tuple<Args...> const& t) const
  {
    std::size_t seed = 0;

    detail::tuple_hash_impl<tuple<Args...>>::apply(seed, t);

    return seed;
  }
};

} // namespace stapl

#endif // STAPL_UTILITY_HASH_HPP

