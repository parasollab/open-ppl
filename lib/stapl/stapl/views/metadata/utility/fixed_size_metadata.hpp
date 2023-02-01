
/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_VIEWS_METADATA_UTILITY_FIXED_SIZE_METADATA_HPP
#define STAPL_VIEWS_METADATA_UTILITY_FIXED_SIZE_METADATA_HPP

#include <stapl/containers/array/static_array_fwd.hpp>
#include <stapl/domains/indexed_fwd.hpp>
#include <stapl/containers/distribution/is_distribution_view_fwd.hpp>

namespace stapl {

template<typename Dom, bool Integral>
struct balanced_partition;

template<typename CID>
struct mapper;

namespace metadata {

template<typename D>
struct container_wrapper;


template<typename Partition, typename Mapper>
struct check_balanced_distribution
  : public std::false_type
{ };

template<>
struct
check_balanced_distribution<balanced_partition<indexed_domain<size_t>>,
                            mapper<size_t>>
  : public std::true_type
{ };


template<typename Container>
bool is_balanced_distribution(Container*)
{
  return std::is_base_of<static_array<typename Container::value_type>,
                         Container>::value;
}


template<typename Distribution>
bool is_balanced_distribution(container_wrapper<Distribution>*)
{
  return check_balanced_distribution<
           typename Distribution::partition_type,
           typename Distribution::mapper_type>::value;
}

template<typename View, typename Part>
bool is_fixed_size_md(View const* view, Part const* part)
{
  // Disable static metadata optimization for arbitrary distributions
  if (!is_arbitrary_view_based<typename
        View::view_container_type::distribution_type::partition_type>()(view))
  {
    auto&& view_dom  = view->domain();
    size_t cont_size = view->container().size();
    size_t delta     = cont_size - view_dom.size();

    return is_balanced_distribution(part) &&
           delta < cont_size / view->get_num_locations();
  }
  // else
  return false;
}

} // namespace metadata

} // namespace stapl

#endif // STAPL_VIEWS_METADATA_UTILITY_FIXED_SIZE_METADATA_HPP
