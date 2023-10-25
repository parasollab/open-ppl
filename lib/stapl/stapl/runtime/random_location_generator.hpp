/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_RANDOM_LOCATION_GENERATOR_HPP
#define STAPL_RUNTIME_RANDOM_LOCATION_GENERATOR_HPP

#include "runtime_fwd.hpp"
#include <cstdint>
#include <random>

namespace stapl {

namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Returns a suitable random number generator for the given unsigned
///        integral type size.
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
template<std::size_t>
struct uint_rng;


//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref uint_rng for unsigned integral types with size
///        equal to @c sizeof(std::unint32_t).
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
template<>
struct uint_rng<sizeof(std::uint32_t)>
{
  typedef std::mt19937 type;
};


//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref uint_rng for unsigned integral types with size
///        equal to @c sizeof(std::unint64_t).
///
/// @ingroup runtimeUtility
//////////////////////////////////////////////////////////////////////
template<>
struct uint_rng<sizeof(std::uint64_t)>
{
  typedef std::mt19937_64 type;
};

} // namespace runtime


//////////////////////////////////////////////////////////////////////
/// @brief Random location id generator.
///
/// @ingroup ARMIUtilities
//////////////////////////////////////////////////////////////////////
class random_location_generator
{
public:
  typedef runtime::location_id result_type;

private:
  runtime::uint_rng<sizeof(result_type)>::type m_gen;
  std::uniform_int_distribution<result_type>   m_dist;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a new @ref random_location_generator.
  ///
  /// @param seed  Random number generator seed.
  /// @param nlocs Number of locations.
  //////////////////////////////////////////////////////////////////////
  explicit random_location_generator(unsigned int seed = get_location_id(),
                                     unsigned int nlocs = get_num_locations())
  : m_gen(seed),
    m_dist(0, (nlocs-1))
  { }

  result_type operator()(void)
  { return m_dist(m_gen); }

  result_type min(void) const
  { return m_dist.min(); }

  result_type max(void) const
  { return m_dist.max(); }
};

} // namespace stapl

#endif
