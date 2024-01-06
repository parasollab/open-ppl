/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_STL_OTHERS_STL_LOCATION_H
#define STAPL_STL_OTHERS_STL_LOCATION_H

#ifndef STAPL_STL_DIR
# error "You did not provide the STL implementation directory"
#endif

#include <boost/preprocessor/stringize.hpp>
#define STAPL_STL_INCLUDE(F) BOOST_PP_STRINGIZE(STAPL_STL_DIR/F)

#endif

