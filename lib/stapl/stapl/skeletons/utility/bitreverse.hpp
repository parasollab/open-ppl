/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_UTILITY_BITREVERSE_HPP
#define STAPL_SKELETONS_UTILITY_BITREVERSE_HPP

#include <cmath>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Given a value this functor computes its bitreversed value by
/// considering the least signficant bits specified by ceil(log2(max))
//////////////////////////////////////////////////////////////////////
template <typename T>
struct bitreverse
{
  unsigned int m_number_of_bits;

  //////////////////////////////////////////////////////////////////////
  /// @brief Set the number of bits to use by determining the number
  ///        of bits needed to represent the value provided.
  /// @param max - ceil(log_2(max)) determines the number of bits in
  ///            bitreverse
  //////////////////////////////////////////////////////////////////////
  bitreverse(T max)
    : m_number_of_bits(std::ceil(std::log2(max)))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Convert a number to its bitreverse value. For example, if the
  ///        struct is constructed with a max of 32(100000) and the
  ///        operator receives 2(000010) the operator will return
  ///        010000.
  //////////////////////////////////////////////////////////////////////
  T operator()(T input) const
  {
    unsigned long bitreverse = 0;

    for (unsigned int i = 0; i < m_number_of_bits; ++i)
    {
      bitreverse <<= 1;

      if (input & 1)
        bitreverse += 1;

      input >>= 1;
    }

    return bitreverse;
  }


  void define_type(typer& t)
  {
    t.member(m_number_of_bits);
  }
};

} // namespace stapl

#endif //STAPL_SKELETONS_UTILITY_BITREVERSE_HPP
