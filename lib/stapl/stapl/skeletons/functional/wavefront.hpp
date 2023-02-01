/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_FUNCTIONAL_WAVEFRONT_HPP
#define STAPL_SKELETONS_FUNCTIONAL_WAVEFRONT_HPP

#include <array>
#include <stapl/skeletons/functional/skeleton_traits.hpp>
#include <stapl/skeletons/operators/elem.hpp>
#include <stapl/skeletons/param_deps/wavefront_pd.hpp>
#include <stapl/skeletons/spans/blocked.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/utility/skeleton.hpp>
#include <stapl/skeletons/utility/position.hpp>
#include <stapl/skeletons/executors/execution_params.hpp>

namespace stapl {
namespace skeletons {
namespace skeletons_impl {

//////////////////////////////////////////////////////////////////////
/// @brief This class abstracts the semantics of a wavefront skeleton
/// by exposing only the necessary information in its representation.
///
/// This abstraction not only makes the reconstruction of a
/// wavefront skeleton easier, but also provides access to the
/// underlying operation of the wavefront skeleton. Furthermore,
/// it reduces the symbol size for a wavefront skeleton, hence, reducing
/// the total compilation time.
///
/// Currently we support 2D and 3D wavefronts from all corners.
///
/// @tparam num_inputs     number of inputs not coming from the wavefront
///                        data flows.
/// @tparam dims           determines the dimensionality of the wavefront
/// @tparam Op             operation to be applied at each point.
/// @tparam SkeletonTraits define the filter, mapper, set_result, and
///                        span of this skeleton.
///
/// @ingroup skeletonsFunctionalInternal
//////////////////////////////////////////////////////////////////////
template <std::size_t num_inputs, std::size_t dims,
          typename Op, typename SkeletonTraits>
struct wavefront
  : public decltype(
             skeletons::elem<
               stapl::default_type<
                 typename SkeletonTraits::span_type,
                 spans::blocked<dims>>,
               flows::elem_f::doacross
             >(skeletons::wavefront_pd<num_inputs, SkeletonTraits::set_result>(
                 std::declval<Op>(),
                 std::array<skeletons::position, dims>(),
                 std::declval<SkeletonTraits>().get_filter(),
                 std::declval<SkeletonTraits>().get_mapper())))
{
  using skeleton_tag_type = tags::wavefront<dims>;
  using op_type           = Op;
  using corners_type      = std::array<skeletons::position, dims>;
  using filter_type       = typename SkeletonTraits::filter_type;
  using mapper_type       = typename SkeletonTraits::mapper_type;

  static constexpr bool set_result = SkeletonTraits::set_result;
  static constexpr std::size_t number_of_dimensions = dims;
  static constexpr std::size_t number_of_inputs = num_inputs;

private:
  using span_t = stapl::default_type<typename SkeletonTraits::span_type,
                                     spans::blocked<dims>>;
  using base_t = decltype(
                      skeletons::elem<
                        span_t, flows::elem_f::doacross
                      >(skeletons::wavefront_pd<num_inputs, set_result>(
                        std::declval<Op>(),
                        std::array<skeletons::position, dims>(),
                        std::declval<filter_type>(),
                        std::declval<mapper_type>())));

public:
  wavefront(Op const& op, corners_type const& corners,
            SkeletonTraits const& traits)
    : base_t(
        skeletons::elem<span_t, flows::elem_f::doacross>(
          skeletons::wavefront_pd<num_inputs, set_result>(
            op, corners, traits.get_filter(), traits.get_mapper()))
      )
  { }

  Op const& get_op() const
  {
    return base_t::nested_skeleton().get_op();
  }

  corners_type const& get_start_corner() const
  {
    return base_t::nested_skeleton().get_start_corner();
  }

  filter_type get_filter() const
  {
    return base_t::nested_skeleton().get_filter();
  }

  void define_type(typer& t)
  {
    t.base<base_t>(*this);
  }
};

} // namespace skeletons_impl

namespace result_of {

template <std::size_t num_inputs, std::size_t dims,
          typename Op, typename SkeletonTraits>
using wavefront = skeletons_impl::wavefront<
                    num_inputs,
                    dims,
                    typename std::decay<Op>::type,
                    typename std::decay<SkeletonTraits>::type>;

} // namespace result_of


//////////////////////////////////////////////////////////////////////
/// @brief Creates an n-dimensional wavefront skeleton.
///
/// @tparam num_inputs number of inputs not coming from the wavefront
///                    data flows.
/// @tparam dims       wavefront number of dimensions
/// @param  op         operation to be applied at each point.
/// @param  corners    determines the starting corners of the wavefront.
/// @param  traits     the traits to be used (default = default_skeleton_traits)
/// @param  execution_params the execution parameters for the nested section
///
/// @ingroup skeletonsFunctional
//////////////////////////////////////////////////////////////////////
template <std::size_t num_inputs = 1,
          std::size_t dims,
          typename ExecutionParams = skeletons_impl::default_execution_params,
          typename Op,
          typename SkeletonTraits = skeletons_impl::default_skeleton_traits,
          typename = typename std::enable_if<
            !is_skeleton<typename std::decay<Op>::type>::value>::type>
result_of::wavefront<num_inputs, dims, Op, SkeletonTraits>
wavefront(Op&& op, std::array<skeletons::position, dims> const& corners,
          SkeletonTraits&& traits = SkeletonTraits(),
          ExecutionParams&& execution_params = ExecutionParams())
{
  return result_of::wavefront<num_inputs, dims, Op, SkeletonTraits>(
           std::forward<Op>(op), corners,
           std::forward<SkeletonTraits>(traits));
}

//////////////////////////////////////////////////////////////////////
/// @brief Creates an n-dimensional wavefront skeleton over a nested
/// skeleton by transforming the inner skeleton to a suitable skeleton for
/// nested execution.
///
/// @tparam num_inputs       number of inputs not coming from the wavefront
///                          data flows.
/// @tparam dims             wavefront number of dimensions
/// @param  op               operation to be applied at each point.
/// @param  corners          determines the starting corners of the wavefront.
/// @param  traits           the traits to be used
///                          (default = default_skeleton_traits)
/// @param  execution_params the execution parameters for the nested section
///
/// @ingroup skeletonsFunctional
//////////////////////////////////////////////////////////////////////
template <std::size_t num_inputs = 1,
          std::size_t dims,
          typename ExecutionParams = skeletons_impl::default_execution_params,
          typename S,
          typename SkeletonTraits = skeletons_impl::default_skeleton_traits,
          typename = typename std::enable_if<
            is_skeleton<typename std::decay<S>::type>::value>::type>
result_of::wavefront<num_inputs,
                     dims,
                     decltype(skeletons::transform<tags::nest>(
                     std::declval<S>(), std::declval<ExecutionParams>())),
                     SkeletonTraits>
wavefront(S&& skeleton,
          std::array<skeletons::position, dims> const& corners,
          SkeletonTraits&& traits = SkeletonTraits(),
          ExecutionParams&& execution_params = ExecutionParams())
{
  return skeletons::wavefront<num_inputs>(
    skeletons::transform<tags::nest>(
      std::forward<S>(skeleton),
      std::forward<ExecutionParams>(execution_params)),
    corners,
    std::forward<SkeletonTraits>(traits));
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_FUNCTIONAL_WAVEFRONT_HPP
