/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_UNORDERED_MAP_MANAGER_HPP
#define STAPL_CONTAINERS_UNORDERED_MAP_MANAGER_HPP

#include "manager.hpp"

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Responsible for determining the home location for elements of
///   the @ref unordered_map.
/// @tparam Partitioner Used to specify how to partition the elements. The
///   default use the default hash function to map GIDs to CIDs.
/// @tparam Mapper Maps the different partitions to the available locations.
//////////////////////////////////////////////////////////////////////
template<typename Partitioner, typename Mapper>
class unordered_map_manager
  : public directory_impl::manager<Partitioner, Mapper>
{
public:
  typedef size_t                                       location_type;
  typedef typename Partitioner::value_type::index_type index_type;

  typedef index_type                                   gid_type;
  typedef location_type                                value_type;
  typedef std::pair<index_type, index_type>            interval_t;
  typedef std::map<interval_t,location_type>           map_result_type;

  typedef directory_impl::manager<Partitioner, Mapper>  base_t;

public:

  //////////////////////////////////////////////////////////////////////
  /// @brief Initializes the manager with a specific partitioner and mapper.
  /// @param part The partitioner. By default calls the partitioner default
  ///   constructor that will hash the GID.
  /// @param mapper The mapper.
  //////////////////////////////////////////////////////////////////////
  unordered_map_manager(const Partitioner& part, const Mapper& mapper)
    : base_t(part,mapper)
  { }

  std::pair<gid_type, gid_type>
  range_of() const
  {
    stapl_assert(this->partition().size() > 0, "default ctor partition used");

    return std::make_pair(this->partition().global_domain().first(),
                          this->partition().global_domain().last());
  }

};

} // stapl namespace

#endif
