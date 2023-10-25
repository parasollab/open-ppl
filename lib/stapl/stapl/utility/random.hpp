/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_RANDOM_HPP
#define STAPL_UTILITY_RANDOM_HPP

#include <random>
#include <boost/random/mersenne_twister.hpp>

STAPL_IS_BASIC_TYPE(boost::random::mt19937)
STAPL_IS_BASIC_TYPE(std::mt19937)

#endif // STAPL_UTILITY_RANDOM_HPP
