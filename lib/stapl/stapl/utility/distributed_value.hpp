/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_DISTRIBUTED_VALUE_HPP
#define STAPL_UTILITY_DISTRIBUTED_VALUE_HPP

#include <stapl/runtime.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief A value that has a representative on all locations and can
///        perform an allreduce.
///
/// @tparam The value type to store
//////////////////////////////////////////////////////////////////////
template<typename T>
class distributed_value
 : p_object
{
private:
  T m_val;

public:
  distributed_value(T v)
    : m_val(std::move(v))
  { }

  T value(void) const
  { return m_val; }

  template<typename ReduceOp>
  stapl::future<T> reduce(ReduceOp&& red)
  {
    return allreduce_rmi(std::forward<ReduceOp>(red),
                        this->get_rmi_handle(), &distributed_value::value);
  }
};

} // namespace stapl

#endif
