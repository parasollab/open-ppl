/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_METADATA_CONTAINER_UTILITY_HPP
#define STAPL_VIEWS_METADATA_CONTAINER_UTILITY_HPP

#include <stapl/algorithms/identity_value.hpp>

namespace stapl {

namespace metadata {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Functor to determine if all the metadata that is required
///        to be stored in current location was assigned.
//////////////////////////////////////////////////////////////////////
template<typename ForwardIterator>
class all_info_set
{
public:
  typedef bool result_type;

private:
  ForwardIterator m_first;
  ForwardIterator m_last;

public:
  all_info_set(ForwardIterator first, ForwardIterator last)
    : m_first(first), m_last(last)
  { }

  bool operator()(void) const
  {
    for (ForwardIterator it = m_first; it != m_last; ++it)
    {
      if (!it->is_info_set())
      {
        return false;
      }
    }
    return true;
  }
}; // class all_info_set


//////////////////////////////////////////////////////////////////////
/// @brief Operator used by @ref growable_container for prefix scan
///   algorithm call.
///
/// @tparam Operand The operand type this binary operator receives.
///
/// @tparam Size Unused template parameter used to make this type dependent on
///   @ref growable_container, thus delaying the full instantiation of
///   the class template until after the prefix_scan call is parsed. This lazy
///   evaluation breaks the cyclic file dependence between the paragraph and
///   coarsening.
///
/// @todo Replace with stapl::plus when cycle dependency is fixed
///   for include functional.hpp (and the paragraph).
//////////////////////////////////////////////////////////////////////
template<typename Operand, typename Size>
struct plus
{
  typedef Operand               first_argument_type;
  typedef Operand               second_argument_type;
  typedef Operand               result_type;

  template<typename Reference1, typename Reference2>
  Operand operator()(Reference1 x, Reference2 y) const
  {
    return x + y;
  }
};

} // namespace detail

} // namespace metadata

template<typename MD>
struct identity_value<metadata::detail::plus<size_t, MD>, size_t>
{
  static size_t value(void)
  {
    return 0;
  }
};

} // namespace stapl

#endif // STAPL_VIEWS_METADATA_CONTAINER_UTILITY_HPP
