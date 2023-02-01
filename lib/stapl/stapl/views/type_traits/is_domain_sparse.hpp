/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_TYPE_TRAITS_IS_DOMAIN_SPARSE_HPP
#define STAPL_VIEWS_TYPE_TRAITS_IS_DOMAIN_SPARSE_HPP

#include <boost/mpl/bool.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Type checker to determine if a domain is a sparse domain.
//////////////////////////////////////////////////////////////////////
template <typename Dom>
struct is_domain_sparse
  : boost::mpl::false_
{ };

} // stapl namespace


#endif /* STAPL_VIEWS_TYPE_TRAITS_IS_DOMAIN_SPARSE_HPP */
