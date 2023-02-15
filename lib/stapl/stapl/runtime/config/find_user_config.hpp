/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_CONFIG_FIND_USER_CONFIG_HPP
#define STAPL_RUNTIME_CONFIG_FIND_USER_CONFIG_HPP

//////////////////////////////////////////////////////////////////////
/// @file
/// If @c STAPL_USER_CONFIG is defined, then the file that is in the macro is
/// picked up as user-defined configuration.
//////////////////////////////////////////////////////////////////////

#if !defined(STAPL_USER_CONFIG) && !defined(STAPL_NO_USER_CONFIG)
// If a user config was not given, use the default
# define STAPL_USER_CONFIG <stapl/runtime/config/user.hpp>
#endif

#if defined(STAPL_USER_CONFIG)
// Include the file
# include STAPL_USER_CONFIG
#endif

#endif
