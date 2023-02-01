/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_PARAGRAPH_EDGE_VIEW_FWD_H
#define STAPL_PARAGRAPH_EDGE_VIEW_FWD_H

#include <stapl/paragraph/edge_container/utility.hpp>

namespace stapl {

#ifdef STAPL_DOCUMENTATION_ONLY

//////////////////////////////////////////////////////////////////////
/// @brief Provides a view over the @p edge_container allowing PARAGRAPH tasks
///  to create data flow value consumption.
/// @ingroup pgEdgeViews
///
/// @tparam T The edge value type (i.e., the return type of the consumed task).
/// @tparam Filter Type of a functor applied to the the value produced by a task
///   of the edge_container prior data flow to the consumer.
///
/// The underlying edge container is non templated and is agnostic to the
/// edge value type.  The @p edge_view informs the @p edge_container of the
/// type via explicit template parameter specification when invoking methods.
//////////////////////////////////////////////////////////////////////
template<typename T,
         typename Filter =
           detail::df_identity<typename df_stored_type<T>::type>>
class edge_view;

#else

template<typename T, typename ...OptionalFilter>
class edge_view;

#endif // STAPL_DOCUMENTATION_ONLY

} // namespace stapl

#endif // ifndef STAPL_PARAGRAPH_EDGE_VIEW_FWD_H

