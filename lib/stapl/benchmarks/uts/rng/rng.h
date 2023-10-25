/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef _RNG_H
#define _RNG_H

/***********************************************************
 *                                                         *
 *  splitable random number generator to use:              *
 *     (default)  sha1 hash                                *
 *     (UTS_ALFG) additive lagged fibonacci generator      *
 *                                                         *
 ***********************************************************/

#if defined(UTS_ALFG)
#  include "alfg.h"
#  define RNG_TYPE 1
#elif defined(BRG_RNG)
#  include "brg_sha1.h"
#  define RNG_TYPE 0
#elif defined(DEVINE_RNG)
#  include "devine_sha1.h"
#  define RNG_TYPE 0
#else
#  error "No random number generator selected."
#endif

#endif /* _RNG_H */
