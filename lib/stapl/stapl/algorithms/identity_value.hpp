/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_IDENTITY_VALUE_HPP
#define STAPL_IDENTITY_VALUE_HPP

#include <boost/static_assert.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief  Reflects the identity value for the given operation and operand
///   types.
/// @tparam Operator The operation type for which the identity value is valid.
/// @tparam Operand The operand type from whose domain the identity value is
///   taken.
///
/// Primary template unconditionally statically asserts, as means a valid
/// specialization for the Operator/Operand pair is not defined.
/// @ingroup functionObjects
//////////////////////////////////////////////////////////////////////
template<typename Operator, typename Operand>
struct identity_value
{
  BOOST_STATIC_ASSERT_MSG(
    sizeof(Operator) == 0,
    "Need to define identity_value specialization for operator / operand pair."
  );
};


//////////////////////////////////////////////////////////////////////
/// @brief Macro that reduces boilerplate code for defining identity values
///   when it's explicit specialization (i.e., no partial specialization of
///   either the Operator or Operand parameters.
//////////////////////////////////////////////////////////////////////
#define STAPL_DEFINE_IDENTITY_VALUE(operation, operand, v) \
  template<>                                               \
  struct identity_value<operation, operand>                \
  {                                                        \
    static operand value(void) { return v; }               \
  };

} // namespace stapl

#endif
