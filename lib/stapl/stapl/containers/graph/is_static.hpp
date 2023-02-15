/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_CONTAINERS_GRAPH_IS_STATIC_HPP
#define STAPL_CONTAINERS_GRAPH_IS_STATIC_HPP

#include <boost/mpl/bool.hpp>
#include <stapl/domains/iterator_domain.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Class for differentiating static and dynamic graphs based on
/// domain-type of the graph.
/// @tparam T type of the container.
/// @ingroup pgraphImpl
//////////////////////////////////////////////////////////////////////
template <typename T>
struct is_static
  : public boost::true_type
{ };

//////////////////////////////////////////////////////////////////////
/// @brief Class for differentiating static and dynamic graphs based on
/// domain-type of the graph.
/// @tparam C type of the container.
/// @tparam D type of the domain.
/// @ingroup pgraphImpl
//////////////////////////////////////////////////////////////////////
template <typename C, typename D>
struct is_static<iterator_domain<C, D> >
  : public boost::false_type
{ };

} // namespace stapl

#endif /* STAPL_CONTAINERS_GRAPH_IS_STATIC_HPP */
