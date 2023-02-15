/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_LOCAL_PARTITION_UTILS_HPP
#define STAPL_CONTAINERS_LOCAL_PARTITION_UTILS_HPP

namespace stapl {

namespace cm_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Comparator used to sort partial domain information in ascending
/// order of location id and then partition id.
//////////////////////////////////////////////////////////////////////
template <typename PartitionInfo>
struct loc_and_part_id_less
{
  typedef bool result_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns whether the location id of the first parameter is less
  /// than the location id of the second parameter, and if they are equal
  /// whether the partition id of the first is less than the second.
  ///
  /// @param lh tuple of partial domain, partition id, and location id
  /// @param rh tuple of partial domain, partition id, and location id
  /// @return true if the partition id of lh is less than the
  /// partition id of rh
  //////////////////////////////////////////////////////////////////////
  template <typename PartitionInfoRef0, typename PartitionInfoRef1>
  result_type operator()(PartitionInfoRef0 lh, PartitionInfoRef1 rh)
  {
    return get<2>(lh) != get<2>(rh) ?
             get<2>(lh) < get<2>(rh) :
             get<1>(lh) < get<1>(rh);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Comparator used to identify adjacent elements in the vector
/// of partition information that belong to different locations.
//////////////////////////////////////////////////////////////////////
struct part_loc_neq
{
  typedef bool result_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns whether the location id of the first parameter is
  /// not equal to the location id of the second parameter.
  /// @param lh tuple of partial domain, partition id, and location id
  /// @param rh tuple of partial domain, partition id, and location id
  /// @return true if the location id of lh is not equal the location id of rh
  //////////////////////////////////////////////////////////////////////
  template <typename PartitionInfoRef>
  result_type operator()(PartitionInfoRef lh, PartitionInfoRef rh)
  {
    return get<2>(lh) != get<2>(rh);
  }
};

} // namespace cm_impl

} // namespace stapl

#endif
