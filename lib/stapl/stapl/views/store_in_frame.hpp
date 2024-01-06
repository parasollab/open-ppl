/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_STORE_IN_FRAME_HPP
#define STAPL_VIEWS_STORE_IN_FRAME_HPP

namespace stapl {
namespace view_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Tag type used by the segmented views framework to annotate
/// that the underlying view should be held in frame.
///
/// It is used by @ref segmented_view and @ref view_container, which,
/// unlike other views, does not implement an ownership mode for
/// underlying containers.
///
/// @todo Views in general should be extended to allow allow taking ownership
/// of the underlying container, whether it be stack or heap allocated.
//////////////////////////////////////////////////////////////////////
struct store_in_frame
{ };

} // namespace view_impl
} // namespace stapl

#endif /* STAPL_VIEWS_STORE_IN_FRAME_HPP */
