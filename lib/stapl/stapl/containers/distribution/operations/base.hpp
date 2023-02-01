/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_DISTRIBUTION_OPERATIONS_BASE_HPP
#define STAPL_CONTAINERS_DISTRIBUTION_OPERATIONS_BASE_HPP

#include <stapl/containers/distribution/operations/settable.hpp>
#include <stapl/containers/distribution/operations/gettable.hpp>
#include <stapl/containers/distribution/operations/applyable.hpp>
#include <stapl/containers/distribution/operations/migratable.hpp>

namespace stapl {

namespace operations {


//////////////////////////////////////////////////////////////////////
/// @brief Operations class for container distributions
/// that provides basic functionality for standard containers including
/// getting, setting and applying.
///
/// Uses the CRTP pattern.
///
/// @tparam Derived The most derived distribution class
////////////////////////////////////////////////////////////////////////
template<typename Distribution>
struct base
  : public operations::settable<Distribution>,
    public operations::gettable<Distribution>,
    public operations::applyable<Distribution>
{ };

} // namespace operations

} // namespace stapl

#endif // STAPL_CONTAINERS_DISTRIBUTION_OPERATIONS_BASE_HPP
