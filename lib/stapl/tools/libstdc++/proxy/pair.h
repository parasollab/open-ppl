/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_PROXY_PAIR_H
#define STAPL_PROXY_PAIR_H

#include <stapl/views/proxy_macros.hpp>

namespace stapl {

STAPL_PROXY_HEADER_TEMPLATE(std::pair, T, U)
{
  STAPL_PROXY_TYPES(STAPL_PROXY_CONCAT(std::pair<T, U>), Accessor)
  STAPL_PROXY_IMPORT_TYPES(first_type, second_type)
  STAPL_PROXY_METHODS(STAPL_PROXY_CONCAT(std::pair<T, U>), Accessor)

  STAPL_PROXY_MEMBER(first, T)
  STAPL_PROXY_MEMBER(second, U)

  explicit proxy(Accessor const& acc)
   : Accessor(acc),
     first(member_referencer<first_accessor>()(acc)),
     second(member_referencer<second_accessor>()(acc))
  { }
};

} // namespace stapl

#endif
