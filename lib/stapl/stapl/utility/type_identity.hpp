/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_TYPE_IDENTITY_HPP
#define STAPL_UTILITY_TYPE_IDENTITY_HPP

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief An identity metafunction.
//////////////////////////////////////////////////////////////////////
template<class A>
using type_identity = std::enable_if<true, A>;


} // namespace stapl
#endif // STAPL_UTILITY_TYPE_IDENTITY_HPP
