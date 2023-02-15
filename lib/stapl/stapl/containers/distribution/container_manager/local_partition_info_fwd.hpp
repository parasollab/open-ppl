/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_LOCAL_PARTITION_INFO_FWD_HPP
#define STAPL_CONTAINERS_LOCAL_PARTITION_INFO_FWD_HPP

#include <stapl/utility/tuple.hpp>
#include <boost/icl/interval_set.hpp>

namespace stapl {

namespace cm_impl {

template <typename Partition, typename Mapper>
std::vector<tuple<
  boost::icl::interval_set<typename Partition::value_type::index_type>,
  typename Mapper::cid_type, location_type> >
get_partial_partition_info(Partition const& partition, Mapper const& mapper);

template <typename PartitionContainer>
std::vector<tuple<
  boost::icl::interval_set<unsigned long int>, unsigned long int, location_type
>>
get_partial_partition_info(PartitionContainer const* const part_cont);

template <typename PartInfo>
PartInfo get_local_partition_info(PartInfo const& partial_info);

} // namespace cm_impl

} // namespace stapl

#endif
