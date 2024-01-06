/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_CONFIG_HPP
#define STAPL_RUNTIME_CONFIG_HPP

//////////////////////////////////////////////////////////////////////
/// @file
/// Configures the runtime system based on user preferences and platform defined
/// macros.
//////////////////////////////////////////////////////////////////////

// user configuration
#include "config/find_user_config.hpp"

// debug mode
#include "config/debug.hpp"
#include "new.hpp"

#include "config/types.hpp"
#include "config/compiler.hpp"
#include "config/platform.hpp"

// last thing to do for config
#include "config/suffix.hpp"

#endif
