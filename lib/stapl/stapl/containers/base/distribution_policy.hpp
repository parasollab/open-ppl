/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_DISTRIBUTION_POLICY_HPP
#define STAPL_CONTAINERS_DISTRIBUTION_POLICY_HPP

namespace stapl {

namespace policy {

enum distribution_enum { Here, Full };

//////////////////////////////////////////////////////////////////////
/// @brief Tag-class describing the distribution policies for nested-pContainer
/// construction. This class specified that the nested-pContainer will be
/// distributed locally.
//////////////////////////////////////////////////////////////////////
struct here
{ };

//////////////////////////////////////////////////////////////////////
/// @brief Tag-class describing the distribution policies for nested-pContainer
/// construction. This class specified that the nested-pContainer will be
/// distributed globally.
//////////////////////////////////////////////////////////////////////
struct full
{ };

} // namespace policy

} // namespace stapl

#endif // STAPL_CONTAINERS_DISTRIBUTION_POLICY_HPP

