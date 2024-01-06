/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_HASH_FWD_HPP
#define STAPL_UTILITY_HASH_FWD_HPP

#include <boost/config.hpp>

// This is needed to allow the boost hash for std::tuple, std::type_index to
// be available.
#ifdef __clang__
#undef BOOST_NO_CXX11_HDR_TUPLE
#undef BOOST_NO_CXX11_HDR_TYPEINDEX
#endif

#include <boost/functional/hash.hpp>

#endif // STAPL_UTILITY_HASH_FWD_HPP

