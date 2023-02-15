/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_TRANSFORMATIONS_COARSE_SCAN_HPP
#define STAPL_SKELETONS_TRANSFORMATIONS_COARSE_SCAN_HPP

#include <type_traits>
#include <stapl/utility/utility.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/transformations/wrapped_skeleton.hpp>
#include <stapl/skeletons/transformations/transform.hpp>

#include <stapl/skeletons/utility/skeleton.hpp>
#include <stapl/skeletons/transformations/coarse/coarse.hpp>
#include <stapl/skeletons/transformations/optimizers/scan.hpp>
#include <stapl/skeletons/transformations/optimizers/reduce.hpp>

#include <stapl/skeletons/transformations/wrapped_skeleton.hpp>
#include <stapl/skeletons/operators/elem.hpp>
#include <stapl/skeletons/functional/reduce.hpp>
#include <stapl/skeletons/functional/map.hpp>
#include <stapl/skeletons/functional/scan.hpp>
#include <stapl/skeletons/param_deps/shifted_first_pd.hpp>
#include <stapl/skeletons/param_deps/zip_pd.hpp>
#include <stapl/algorithms/identity_value.hpp>


namespace stapl {
namespace skeletons {
namespace scan_helpers {

//////////////////////////////////////////////////////////////////////
/// @brief This struct uses the @c Op and computes the result of
/// the application of @c Op to the pair of an offset given by @c Value
/// and elements of an input given by @c Values.
///
/// @tparam Op the operation to be applied
///
/// @ingroup skeletonsTransformationsInternal
/////////////////////////////////////////////////////////////////////
template <typename Op>
struct add_to_all_op
  : public Op
{
  using value_type  = typename Op::result_type;
  using result_type = std::vector<value_type>;
public:
  explicit add_to_all_op(Op const& op)
    : Op(op)
  { }

  template <typename Value, typename Values>
  result_type operator()(Value v, Values const& ts) const
  {
    result_type res(ts.size());
    std::transform(ts.begin(), ts.end(), res.begin(),
                   boost::bind(static_cast<Op const&>(*this), _1, v));
    return res;
  }

  void define_type(typer& t)
  {
    t.base<Op>(*this);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief This filter is used in naive coarsened scan to read the
/// value which is filtered by @c back_op.
///
/// @tparam T the fine-grain element value type
///
/// @todo With multi-type filters this struct would not be needed.
///
/// @ingroup skeletonsTransformationsInternal
//////////////////////////////////////////////////////////////////////
template <typename T>
struct pickfirst
{
  using result_type = T;

  template <typename Ts>
  T operator()(Ts ts) const
  { return ts[0]; }
};


//////////////////////////////////////////////////////////////////////
/// @brief This filter is used in naive coarsened version of scan to
/// limit the size of data passed to the intermediate scan.
///
/// Only the last element of a task on an edge with a @c back_op filter
/// will be sent to its successors.
///
/// @tparam T the fine-grain element value type
///
/// @todo Currently, filters do not allow type transformations, that
///       is the reason why a vector of size 1 is returned from this
///       filter. Later on, when filters are changed to allow type
///       transformations, this filter would be merged with @c pick_first.
///
/// @ingroup skeletonsTransformationsInternal
//////////////////////////////////////////////////////////////////////
template <typename T>
struct back_op
{
  using result_type = std::vector<T>;

  template <typename Ts>
  result_type operator()(Ts const& ts) const
  {
    std::vector<T> result;
    result.push_back(ts.back());
    return result;
  }

  template <typename... Args>
  int configure_filter(Args&&... args)
  {
    return 0;
  }

  bool operator==(back_op const&) const
  { return true; }
};


template <typename Op, typename Tag>
struct offset_scan_copy_op;


//////////////////////////////////////////////////////////////////////
/// @brief This operation is used in optimized version of coarsened
/// inclusive scan. It computes scan of an input, given an offset @c v
/// and an input @c values, and immediately writes the result to a
/// given output view.
///
/// @tparam Op the operation to be used for computing scan
///
/// @ingroup skeletonsTransformationsInternal
//////////////////////////////////////////////////////////////////////
template <typename Op>
struct offset_scan_copy_op<Op, tags::inclusive>
  : private Op
{
  using value_type  = typename Op::result_type;
  using result_type = void;

  explicit offset_scan_copy_op(Op const& op)
    : Op(op)
  { }

  template <typename Value, typename Values, typename OutValues>
  void operator()(Value const v, Values const& values,
                  OutValues& out_values) const
  {
    value_type sum = v;
    auto&& out_it = out_values.begin();
    for (auto&& elem : values)
    {
      sum = *out_it = Op::operator()(sum, elem);
      ++out_it;
    }
  }

  void define_type(typer& t)
  {
    t.base<Op>(*this);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief This operation is used in optimized version of coarsened
/// exclusive scan. It computes the exclusive scan of an input, given
/// an offset @c v and an input @c values, and immediately writes the
/// result to a given output view.
///
/// @tparam Op the operation to be used for computing scan
///
/// @ingroup skeletonsTransformationsInternal
//////////////////////////////////////////////////////////////////////
template <typename Op>
struct offset_scan_copy_op<Op, tags::exclusive>
  : private Op
{
  using value_type  = typename Op::result_type;
  using result_type = void;

  explicit offset_scan_copy_op(Op const& op)
    : Op(op)
  { }

  template <typename Value, typename Values, typename OutValues>
  void operator()(Value const v, Values const& values,
                  OutValues& out_values) const
  {
    value_type sum = v;
    auto&& out_it = out_values.begin();
    for (auto&& elem : values)
    {
      // if input and output view are the same we have to keep a copy
      // of the current value before overwriting it.
      value_type cur_val = elem;
      *out_it = sum;
      sum = Op::operator()(sum, cur_val);
      ++out_it;
    }
  }

  void define_type(typer& t)
  {
    t.base<Op>(*this);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief This operation is used in optimized coarsened scans. It
/// computes scan of an input, given an offset @c v and an input
/// @c values and returns the result of computation.
///
/// @tparam Op  the operation to be used for computing scan
/// @tparam Alg whether this is an inclusive or exclusive scan
///
/// @ingroup skeletonsTransformationsInternal
//////////////////////////////////////////////////////////////////////
template <typename Op, typename Alg>
struct offset_scan_op
  : private Op
{
  using value_type  = typename Op::result_type;
  using result_type = std::vector<value_type>;

  explicit offset_scan_op(Op const& op)
    : Op(op)
  { }

  template <typename Value, typename Values>
  result_type operator()(Value const v, Values const& values) const
  {
    result_type out_values;
    out_values.reserve(values.size());
    std::size_t sum = v;
    for (auto&& elem : values)
    {
      if (std::is_same<Alg, tags::inclusive>::value)
      {
        sum = Op::operator()(sum, elem);
        out_values.push_back(sum);
      }
      else {
        out_values.push_back(sum);
        sum = Op::operator()(sum, elem);
      }
    }
    return out_values;
  }

  void define_type(typer& t)
  {
    t.base<Op>(*this);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief The update phase in a coarsened scan adds the offset
/// computed by the intermediate scan to the scan result of each
/// chunk of data passed to it.
///
/// @ingroup skeletonsTransformationsInternal
//////////////////////////////////////////////////////////////////////
struct find_scan_update_phase_pd
{

  template <typename Op>
  static auto apply_call(std::false_type, tags::exclusive, Op const& op)
  STAPL_AUTO_RETURN((
    zip_pd<2>(scan_helpers::offset_scan_op<Op, tags::exclusive>(op))
  ))

  template <typename Op>
  static auto apply_call(std::true_type, tags::exclusive, Op const& op)
  STAPL_AUTO_RETURN((
    zip_pd<3>(scan_helpers::offset_scan_copy_op<Op, tags::exclusive>(op))
  ))

  template <typename Op>
  static auto apply_call(std::false_type, tags::inclusive, Op const& op)
  STAPL_AUTO_RETURN((
    shifted_first_pd<2>(
      scan_helpers::offset_scan_op<Op, tags::inclusive>(op),
      identity_value<Op, typename scan_operand_type<Op>::type>::value())
  ))

  template <typename Op>
  static auto apply_call(std::true_type, tags::inclusive, Op const& op)
  STAPL_AUTO_RETURN((
    shifted_first_pd<3>(
      scan_helpers::offset_scan_copy_op<Op, tags::inclusive>(op),
      identity_value<Op, typename scan_operand_type<Op>::type>::value())
  ))

  template <typename Alg, typename CoarseTag, typename Op>
  static auto call(Op const& op)
  STAPL_AUTO_RETURN((
    apply_call(
      std::integral_constant<
        bool, (std::is_same<CoarseTag, stapl::use_default>::value) or
              (std::is_same<CoarseTag, tags::naive>::value)>(),
      Alg(), op)
  ))
};
} // namespace scan_helpers


namespace transformations {

template <typename S, typename Tag, typename CoarseTag>
struct transform;


//////////////////////////////////////////////////////////////////////
/// @brief A coarsened version of scan algorithm. The type of the
/// scan skeleton to be used is determined by @c Tag and @c Type
///
/// Similar to other coarsened skeletons, the @c coarse_tag is used to
/// determine the static optimization to be applied on the coarsened
/// skeleton.
///
/// @tparam S            the scan skeleton
/// @tparam Tag          scan algorithm to be used for the conquering phase
/// @tparam Alg          type of the scan algorithm (exclusive, inclusive)
/// @tparam CoarseTag    a tag to specify the required specialization for
///                      coarsening
/// @tparam ExecutionTag a tag to specify the execution method used for
///                      the coarsened chunks
///
/// @see skeletonsTagsCoarse
/// @see skeletonsTagsExecution
///
/// @ingroup skeletonsTransformationsCoarse
//////////////////////////////////////////////////////////////////////
template <typename S, typename Tag, typename Alg,
          typename CoarseTag, typename ExecutionTag>
struct transform<S, tags::scan<Tag, Alg>,
                 tags::coarse<CoarseTag, ExecutionTag>>
{
  static auto call(S const& skeleton)
  STAPL_AUTO_RETURN((
    skeletons::compose<flows::compose_flows::input_to_last>(
      skeletons::map(
        skeletons::wrap<ExecutionTag>(skeletons::reduce(skeleton.get_op()))),
      skeleton,
      skeletons::elem(
        scan_helpers::find_scan_update_phase_pd::call<Alg, CoarseTag>(
          skeleton.get_op())))
  ))
};


//////////////////////////////////////////////////////////////////////
/// @brief This version of coarse scan skeleton is only kept for
/// comparison purposes. This is the slowest version of a coarsened
/// scan skeleton, since it has 4N memory accesses (2N accesses in
/// computing the divide phase scan and 2N accesses in t he update
/// phase).
///
/// Extra accesses to memory cause a significant slow down of the
/// algorithm, but more importantly, the allocation of intermediate
/// buffers by each divide phase scan tasks causes  this simple scan
/// skeleton to be unusable in real applications.
///
/// @tparam S            the scan skeleton
/// @tparam Tag          type of exclusive scan to use for the conquering phase
/// @tparam CoarseTag    a tag to specify the required specialization for
///                      coarsening
/// @tparam ExecutionTag a tag to specify the execution method used for
///                      the coarsened chunks
///
/// @see filtered_map
/// @see wrapped_skeleton
///
/// @ingroup skeletonsTransformationsCoarse
//////////////////////////////////////////////////////////////////////
template<typename S, typename Tag, typename ExecutionTag>
struct transform<S, tags::scan<Tag, tags::exclusive>,
                 tags::coarse<tags::naive, ExecutionTag>>
{
private:
  using value_t = typename decltype(std::declval<S>().get_op())::result_type;

public:
  static auto call(S const& skeleton)
  STAPL_AUTO_RETURN((
    skeletons::compose<flows::compose_flows::input_to_last>(
      skeletons::map(
        skeletons::wrap<ExecutionTag>(
          skeletons::scan(skeleton.get_op()))),
      skeletons::map(
        scan_helpers::pickfirst<value_t>(),
        skeletons::skeleton_traits(scan_helpers::back_op<value_t>())),
      skeleton,
      skeletons::elem(
        scan_helpers::find_scan_update_phase_pd::call<
          tags::exclusive, tags::naive
        >(skeleton.get_op())))
  ))
};

} // namespace transformations
} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_TRANSFORMATIONS_COARSE_SCAN_HPP
