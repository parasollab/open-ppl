/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_BASE_VIEW_HPP
#define STAPL_VIEWS_BASE_VIEW_HPP

#include <stapl/runtime/p_object.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Base class to tag views
//////////////////////////////////////////////////////////////////////
struct base_view
#ifdef _STAPL
  : public p_object
#endif
{ };

} // namespace stapl

#endif // STAPL_VIEWS_BASE_VIEW_HPP
