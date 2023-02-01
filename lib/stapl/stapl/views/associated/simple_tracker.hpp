/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_ASSOCIATE_SIMPLE_TRACKER
#define STAPL_CONTAINERS_ASSOCIATE_SIMPLE_TRACKER

#include <stapl/containers/array/array.hpp>
#include <stapl/views/array_view.hpp>

#include <stapl/containers/map/map.hpp>
#include <stapl/views/map_view.hpp>

#include <vector>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Defines an object to track index relations between the two
///        given views.
///
/// @tparam V0 Main view.
/// @tparam V1 Follower view.
/// @todo Storage should logically be a map.
//////////////////////////////////////////////////////////////////////
template<typename V0, typename V1>
class simple_tracker
{
  typedef typename view_traits<V0>::index_type       key_type;
  typedef typename view_traits<V1>::index_type       associated_value_type;
  typedef array<std::vector<associated_value_type> > storage_type;
  typedef array_view<storage_type>                   map_type;

  //typedef map<key_type, std::vector<associated_value_type> > storage_type;
  //typedef map_view<storage_type> map_type;

  map_type m_tracking_map;

public:
  typedef typename storage_type::value_type value_type;

  simple_tracker(V0 const& v)
    : m_tracking_map(new storage_type(v.size()))
  { }

  simple_tracker()
    : m_tracking_map()
  { }

  //////////////////////////////////////////////////////////////////////
  /// @todo Remove duplicates.
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  value_type operator()(T const& x, key_type const& i) const
  {
    value_type v = m_tracking_map.get_element(i);
    return v;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Associate the given index @p j with the index @p i.
  //////////////////////////////////////////////////////////////////////
  void track(key_type const& i, associated_value_type const& j)
  {
    m_tracking_map[i].push_back(j);
  }

  void define_type(typer&)
  {
    abort("simple_tracker: Incorrect define_type().");
  }
};

} // end namespace stapl

#endif
