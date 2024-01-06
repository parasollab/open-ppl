/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_THIS_CONTEXT_HPP
#define STAPL_RUNTIME_THIS_CONTEXT_HPP

#include "context_id.hpp"
#include <boost/optional.hpp>

namespace stapl {

namespace runtime {

class context;
class location_md;


//////////////////////////////////////////////////////////////////////
/// @brief STAPL Runtime System RMI execution context management.
//////////////////////////////////////////////////////////////////////
namespace this_context {

//////////////////////////////////////////////////////////////////////
/// @brief Pushes a base @ref context on the stack.
///
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
void push_base(context&);


//////////////////////////////////////////////////////////////////////
/// @brief Pops the base @ref context from the stack.
///
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
void pop_base(context&);


//////////////////////////////////////////////////////////////////////
/// @brief Pushes a placeholder for a new @ref context on the stack.
///
/// @see gang
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
void push_placeholder(boost::optional<context>&);


//////////////////////////////////////////////////////////////////////
/// @brief Pops the placeholder from the stack.
///
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
void pop_placeholder(void);


//////////////////////////////////////////////////////////////////////
/// @brief Switches to the base @ref context of @p l.
///
/// The location metadata @p l will be used to either create a new base context
/// or to switch to an existing one.
///
/// @see gang
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
void switch_to(location_md&, boost::optional<context>&);


//////////////////////////////////////////////////////////////////////
/// @brief Unswitches from the @ref context on the stack.
///
/// @see gang
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
void unswitch(void);


//////////////////////////////////////////////////////////////////////
/// @brief Returns the current @ref context from the stack.
///
/// If the @ref context creation was deferred (e.g. one location gangs defer
/// the creation of all metadata) then this function will create all the
/// required metadata and return the associated @ref context object.
///
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
context& get(void);

//////////////////////////////////////////////////////////////////////
/// @brief Returns the @ref base context of the context at the top of
///        the stack.
///
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
context& base_of_top(void);


//////////////////////////////////////////////////////////////////////
/// @brief Returns a pointer to the current @ref context from the stack.
///
/// If the @ref context creation was deferred, then it returns @c nullptr.
///
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
context* try_get(void) noexcept;


//////////////////////////////////////////////////////////////////////
/// @brief Returns the current context id from the stack.
///
/// If the @ref context creation was deferred (e.g. one location gangs defer
/// the creation of all metadata) then this function will create all the
/// required metadata and return the associated @ref context object.
///
/// @see this_context::get()
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
context_id const& get_id(void);


//////////////////////////////////////////////////////////////////////
/// @brief Returns a pointer to the location metadata of the given gang id if
///        it is in the stack, otherwise @c nullptr.
///
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
location_md* try_get_location_md(const gang_id) noexcept;


//////////////////////////////////////////////////////////////////////
/// @brief Returns @c true if the execution can be restored for location @p l.
///
/// @warning This is an expensive operation that restores an SPMD section, used
///          in @ref restore().
///
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
bool can_restore(location_md& l);

} // namespace this_context

} // namespace runtime

} // namespace stapl

#endif
