/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_UTILITY_DYNAMIC_WF_HPP
#define STAPL_SKELETONS_UTILITY_DYNAMIC_WF_HPP

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Used to indicate that a work function deriving from this class
///   should be provided the @ref paragraph_impl::paragraph_view as the first
///   argument to its function operator to allow it to add additional tasks to
///   the PARAGRAPH.
///
/// @ingroup skeletonsUtilities
//////////////////////////////////////////////////////////////////////
struct dynamic_wf
{ };

} // namespace stapl

#endif
