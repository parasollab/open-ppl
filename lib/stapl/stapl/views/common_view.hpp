/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_COMMON_VIEW_HPP
#define STAPL_VIEWS_COMMON_VIEW_HPP

#include <boost/utility.hpp>
#include <boost/make_shared.hpp>
#include <stapl/runtime/serialization_fwd.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief This class contains functionality that EVERY view must provide.
///
/// All views must inherit from this class.
/// If a class doesn't inherit from this class, it isn't a view.
/// Don't put functionality here unless every view can support it.
//////////////////////////////////////////////////////////////////////
class common_view
{

}; // class common_view

} // namespace stapl

#endif // STAPL_VIEWS_CORE_VIEW_HPP
