/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_UTILITY_HPP
#define STAPL_UTILITY_UTILITY_HPP

//////////////////////////////////////////////////////////////////////
/// @brief Simple macro to avoid redundant typing of expression when in simple
///   functions where body is relatively trivial.
/// @ingroup Tuple
//////////////////////////////////////////////////////////////////////
#define STAPL_AUTO_RETURN( EXPR ) -> decltype( EXPR ) \
{ return EXPR; }

#endif // STAPL_UTILITY_UTILITY_HPP