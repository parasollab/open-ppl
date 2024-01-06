/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef BOUNDARIES_COMMON_HEADERS_HPP
#define BOUNDARIES_COMMON_HEADERS_HPP

#include <array>
#include <vector>
#include <memory>

#include <stapl/skeletons/utility/position.hpp>
#include <stapl/containers/generators/functor.hpp>
#include <stapl/array.hpp>
#include <stapl/multiarray.hpp>

#include "boundary_spec.hpp"
#include <Kripke.h>

using stapl::skeletons::direction;
using stapl::functor_view_type;
using stapl::functor_container;
using stapl::functor_view;
using stapl::multiarray;
using stapl::multiarray_view;
using stapl::lightweight_vector;
using stapl::indexed_domain;

#endif // BOUNDARIES_COMMON_HEADERS_hpp
