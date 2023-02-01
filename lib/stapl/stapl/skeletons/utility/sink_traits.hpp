/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_UTILITY_SINK_TRAITS_HPP
#define STAPL_SKELETONS_UTILITY_SINK_TRAITS_HPP

#include <stapl/skeletons/utility/mappers.hpp>

namespace stapl {
namespace skeletons {


struct location_id
{
  template<typename... Arg>
  int operator()(Arg&&... arg) const
  {
    return 0;
  }
};


template <typename SkeletonTag, int dims = 2>
struct sink_traits
{
  using domain_type     = indexed_domain<std::size_t, dims>;
  using index_type      = typename domain_type::index_type;
  using traversal_t     = typename domain_type::traversal_type;
  using linearize_t     = nd_linearize<index_type, traversal_t>;

  using output_to_output_mapper_type = default_output_to_output_mapper<dims>;
  using output_to_input_mapper_type  = default_output_to_input_mapper<dims>;
  using input_to_input_mapper_type   = default_input_to_input_mapper<dims>;
  using result_mapper_type           = linearize_t;
  using should_set_type              = should_flow<dims, SkeletonTag>;
  using should_flow_t                = should_flow<dims, SkeletonTag>;
};


template <int dims>
struct sink_traits<tags::broadcast_to_locs, dims>
{
  using domain_type     = indexed_domain<std::size_t, dims>;
  using index_type      = typename domain_type::index_type;
  using traversal_t     = typename domain_type::traversal_type;
  using linearize_t     = nd_linearize<index_type, traversal_t>;

  using output_to_output_mapper_type = default_output_to_output_mapper<dims>;
  using output_to_input_mapper_type  = linearize_t;
  using result_mapper_type           = location_id;
  using should_set_type              = spans::per_location;
};


template <int dims>
struct sink_traits<tags::expand_from_pow_two, dims>
{
  using domain_type     = indexed_domain<std::size_t, dims>;
  using index_type      = typename domain_type::index_type;
  using traversal_t     = typename domain_type::traversal_type;
  using linearize_t     = nd_linearize<index_type, traversal_t>;

  using output_to_output_mapper_type = default_output_to_output_mapper<dims>;
  using output_to_input_mapper_type  = linearize_t;
  using result_mapper_type           = location_id;
  using should_set_type              = spans::per_location;
};


template <int dims, int nested_dims>
struct sink_traits<tags::wavefront<dims>, nested_dims>
{
  using output_to_output_mapper_type =
    wf_output_to_output_mapper<nested_dims, dims>;

  using output_to_input_mapper_type  =
    wf_output_to_input_mapper<nested_dims, dims>;

  using input_to_input_mapper_type   =
    wf_input_to_input_mapper<nested_dims, dims>;

  using result_mapper_type = wf_result_mapping<nested_dims>;
  using should_set_type    = result_mapper_type;
  using should_flow_t      = should_flow<nested_dims, tags::wavefront<dims>>;
};

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_UTILITY_SINK_TRAITS_HPP
