/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_ALGORITHMS_ALGORITHM_HPP
#define STAPL_ALGORITHMS_ALGORITHM_HPP

#include <numeric>
#include <stapl/paragraph/paragraph.hpp>
#include "generator.hpp"
#include "functional.hpp"
#include "numeric.hpp"
#include <boost/random/uniform_int_distribution.hpp>
#include <stapl/utility/hash.hpp>
#include <stapl/utility/random.hpp>
#include <stapl/views/proxy/proxy.hpp>
#include <stapl/views/overlap_view.hpp>
#include <stapl/views/balance_view.hpp>
#include <stapl/views/native_view.hpp>
#include <stapl/views/repeated_view.hpp>
#include <stapl/views/counting_view.hpp>
#include <stapl/views/reverse_view.hpp>
#include <stapl/views/list_view.hpp>
#include <stapl/views/transform_view.hpp>
#include <stapl/views/type_traits/strip_fast_view.hpp>
#include <stapl/containers/partitions/splitter.hpp>
#include <stapl/views/functor_view.hpp>
#include <stapl/containers/generators/functor.hpp>
#include <stapl/containers/array/static_array.hpp>
#include <stapl/containers/list/list.hpp>
#include <stapl/skeletons/executors/execute.hpp>
#include <stapl/containers/base/exchange.hpp>

#include "algorithm_fwd.hpp"
#include "non_modifying.hpp"
#include "binary_search.hpp"
#include "minmax.hpp"

#include "algo_detail.hpp"

#include <stapl/views/type_traits/has_iterator.hpp>
namespace stapl {

namespace algo_details {

//////////////////////////////////////////////////////////////////////
/// @brief Work function for @ref replace_if(), which assigns the stored value
///   to the argument if the predicate returns true.
/// @tparam Predicate Unary functor which is called on the argument.
//////////////////////////////////////////////////////////////////////
template<typename Predicate, typename T>
struct assign_if
{
  Predicate m_pred;
  T m_new_value;

  typedef void result_type;

  assign_if(Predicate const& pred,  T const& new_value)
    : m_pred(pred), m_new_value(new_value)
  { }

  void define_type(typer& t)
  {
    t.member(m_pred);
    t.member(m_new_value);
  }

  template<typename Reference1>
  void operator()(Reference1&& x)
  {
    if (m_pred(x))
      x = m_new_value;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function for @ref replace(), which calls a range-based
///        version of std::replace.
/// @tparam new_value Value that will replace any old_value.
/// @tparam old_value Value that will be replaced by new_value.
//////////////////////////////////////////////////////////////////////
template<typename T>
class serial_replace
{
private:
  T m_old_value;
  T m_new_value;

public:
  typedef void result_type;

  serial_replace(T old_value, T new_value)
    : m_old_value(std::move(old_value)), m_new_value(std::move(new_value))
  { }

  void define_type(typer& t)
  {
    t.member(m_old_value);
    t.member(m_new_value);
  }

  template<typename View>
  void operator()(View&& view)
  {
    std::replace(view.begin(), view.end(), m_old_value , m_new_value);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function used to place a given element into one of two output
///   views, depending on whether the given functor returns true or false.
/// @tparam Pred Unary functor which is used to place the element.
/// @tparam ValueType Type of the values which are being partitioned.
//////////////////////////////////////////////////////////////////////
template <typename Pred, typename ValueType>
struct partition_apply_pred
{
private:
  typedef list<ValueType> p_list_type;

  Pred          m_pred;
  p_list_type*  m_pl_satisfy_pred;
  p_list_type*  m_pl_not_satisfy_pred;

public:
  typedef void result_type;

  partition_apply_pred(Pred pred,
                       p_list_type* pl_satisfy_pred,
                       p_list_type* pl_not_satisfy_pred)
    : m_pred(pred),
      m_pl_satisfy_pred(pl_satisfy_pred),
      m_pl_not_satisfy_pred(pl_not_satisfy_pred)
  { }

  template <typename View>
  void operator()(View elem_view)
  {
    if (m_pred(elem_view))
      m_pl_satisfy_pred->add(elem_view);
    else
      m_pl_not_satisfy_pred->add(elem_view);
  }

  void define_type(typer& t)
  {
    t.member(m_pred);
    t.member(m_pl_satisfy_pred);
    t.member(m_pl_not_satisfy_pred);
  }
};

} // namespace algo_details


namespace algo_details {

//////////////////////////////////////////////////////////////////////
/// @brief Random number generator that returns a random number between 0 and
///   the input size using the given generator.
/// @tparam UniformRandomNumberGenerator Generator to use for number generation.
//////////////////////////////////////////////////////////////////////
template<class UniformRandomNumberGenerator>
class shuffle_random_number_generator
{
private:
  UniformRandomNumberGenerator m_rng;

public:
  shuffle_random_number_generator(UniformRandomNumberGenerator const& rng)
    : m_rng(rng) {}
  int operator()(int n)
  {
    typedef boost::random::uniform_int_distribution<size_t> distr_type;
    typedef typename distr_type::param_type                 p_type;

    distr_type d;

    return d(m_rng, p_type(0, n-1));
  }


  void define_type(typer& t)
  {
    t.member(m_rng);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper function to invoke @ref coarse_assign on source and
/// destination view.  The source view's domain is restricted with the
/// @p first_read and @p last_read offsets.  The destination view's
/// domain is restricted to begin with @p write_offset and span enough
/// elements to match the size of the read domain
/// (ie., last_read - first_read).
//////////////////////////////////////////////////////////////////////
template<typename SrcView, typename DstView>
void
set_elements(SrcView&& src_vw, DstView&& dst_vw,
             size_t first_read, size_t last_read, size_t write_offset)
{
  // SrcView is expected to be a stapl::mix_view instance. We need the
  // underlying type in order to construct the view for the set_elements call.
  using src_view_t =
    typename detail::strip_mix_view<
      typename detail::strip_fast_view<
        typename std::decay<SrcView>::type>::type
    >::type;

  using src_dom_t  = typename src_view_t::domain_type;

  using dst_view_t =
    typename detail::strip_repeat_view<
      typename detail::strip_mix_view<
        typename detail::strip_fast_view<
          typename std::decay<DstView>::type
        >::type
      >::type
    >::type;

  using dst_dom_t = typename dst_view_t::domain_type;

  auto src_first =
    src_vw.domain().advance(src_vw.domain().first(), first_read);
  auto src_last  =
    src_vw.domain().advance(src_first, last_read - first_read);

  auto dst_first =
    dst_vw.domain().advance(dst_vw.domain().first(), write_offset);
  auto dst_last  =
     dst_vw.domain().advance(dst_first, last_read - first_read);

  // Views to use for coarse assignment, domains must be set before use.
  src_view_t src_view(src_vw.container(), src_dom_t(src_first, src_last));
  dst_view_t dst_view(dst_vw.container(), dst_dom_t(dst_first, dst_last));

  coarse_assign()(src_view, dst_view);
}


//////////////////////////////////////////////////////////////////////
/// @brief Invokes a base partition algorithm on a range and returns
/// counts of matches and non-matches in a pair.
/// @tparam RangePartitioner A functor invoked to perform the partition
/// on each range.
//////////////////////////////////////////////////////////////////////
template <typename RangePartitioner>
class partition_range
{
private:
  RangePartitioner m_range_partitioner;

public:
  using result_type = std::pair<unsigned int, unsigned int>;

  partition_range(RangePartitioner range_partitioner)
    : m_range_partitioner(std::move(range_partitioner))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Signature used by algorithms that mutate input sequence.
  //////////////////////////////////////////////////////////////////////
  template <typename Range>
  result_type operator()(Range&& range) const
  {
    auto iter      = m_range_partitioner(std::forward<Range>(range));
    size_t matches = std::distance(range.begin(), iter);

    return result_type{matches, range.size() - matches};
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Signature used by algorithms that write results of partitioning
  /// to a second output sequence (e.g., the *_copy family of algorithms.
  //////////////////////////////////////////////////////////////////////
  template <typename Range0, typename Range1>
  result_type operator()(Range0&& range0, Range1&& range1) const
  {
    auto iter      = m_range_partitioner(std::forward<Range0>(range0),
                                         std::forward<Range1>(range1));
    size_t matches = std::distance(range1.begin(), iter);

    return result_type{matches, range0.size() - matches};
  }

  void define_type(typer& t)
  { t.member(m_range_partitioner); }
}; // class partition_range


//////////////////////////////////////////////////////////////////////
/// @brief Work function which receives counts and offsets from previous
/// steps in skeleton composition and writes to two output variables.
///
/// @todo Refactoring skeleton composition just a little further can
/// probably enable the removal of this functor / step.
//////////////////////////////////////////////////////////////////////
struct partition_write
{
  using result_type = void;

  template<typename Counts, typename Offsets,
           typename OutCounts, typename OutOffsets>
  void operator()(Counts&& counts, Offsets&& offsets,
                  OutCounts&& out_counts, OutOffsets&& out_offsets) const
  {
    out_counts  = counts;
    out_offsets = offsets;
  }
}; // class partition_write


//////////////////////////////////////////////////////////////////////
/// @brief Elementwise addition of pair count match / mismatch counts.
/// Reduction operator for @ref partition.
//////////////////////////////////////////////////////////////////////
struct partition_reduce
{
  using result_type = std::pair<unsigned int, unsigned int>;

  template<typename LHS, typename RHS>
  result_type operator()(LHS&& lhs_param, RHS&& rhs_param) const
  {
    result_type lhs = lhs_param;
    result_type rhs = rhs_param;
    return result_type{lhs.first + rhs.first, lhs.second + rhs.second};
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function which receives an already locally partitioned
/// input from previous phases of the partition algorithm and writes
/// to the global output the matches and nonmatches to the correct place
/// given the offsets and global match sum.
///
/// Has a return value so that the composed skeleton can do a reduction
/// broadcast prior to writeback to the source view, ensuring that
/// all invocations of this work function have completed.
//////////////////////////////////////////////////////////////////////
class partition_general_write
{
private:
  /// Guards writing of non matches, so that in use cases such as
  /// remove() which use partition, we don't do writes not required
  /// by the algorithm.
  bool           m_b_write_nmatches;

public:
  using result_type = size_t;

  partition_general_write(bool b_write_nmatches)
    : m_b_write_nmatches(b_write_nmatches)
  { }

  template<typename SrcView, typename DstView,
           typename LocalCounts, typename GlobalCounts,
           typename MatchSum>
  result_type
  operator()(SrcView&& src_vw, DstView&& dst_vw,
             LocalCounts&& local_counts, GlobalCounts&& global_offets,
             MatchSum&& global_sums) const
  {
    const size_t local_match_count   = local_counts.first;
    const size_t match_write_offset  = global_offets.first;

    set_elements(src_vw, *dst_vw.begin(),
                 0, local_match_count-1, match_write_offset);

    if (m_b_write_nmatches)
    {
      const size_t match_sum           = global_sums.first;
      const size_t nmatch_write_offset = match_sum + global_offets.second;

      set_elements(src_vw, *dst_vw.begin(),
                   local_match_count, src_vw.size()-1,
                   nmatch_write_offset);
    }

    return 0;
  }

  void define_type(typer& t)
  { t.member(m_b_write_nmatches); }
}; // class partition_general_write


struct partition_write_back
{
  using result_type = void;

  template<typename Ref, typename Src, typename Dst>
  result_type operator()(Ref&&, Src&& src, Dst&& dst) const
  { coarse_assign()(std::forward<Src>(src), std::forward<Dst>(dst)); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Partition the input such that all elements for which the predicate
///   returns true are ordered before those for which it returns false.
///   This signature called when partition is performed in place on the
///   the single input view.
///
/// @param view One-dimensional view of the input.  Mutated to store result
///   of partitioning.
/// @param partitioner Functor used to partition each range.
/// @param b_maintain_stability Denotes whether exchange process should maintain
///   the relative order between elements on different locations headed to the
///   same target partition or if this can be relaxed to minimize communication
///   and thus increase performance.
/// @param b_write_nmatches If false, the mutated input will not be
///   guaranteed to be the elements from input for the non match portion of the
///   partition. Useful for algorithms such as remove.
/// @return The number of elements for which the predicate returns true.
/// @ingroup reorderingAlgorithms
//////////////////////////////////////////////////////////////////////
template <typename View, typename RangePartitioner>
size_t partition_impl(View const& view,
                      RangePartitioner partitioner,
                      bool b_maintain_stability,
                      bool b_write_nmatches)
{
  using namespace skeletons;

  auto const& part = view.container().distribution().partition();

  using part_type = typename std::decay<decltype(part)>::type;

  if (is_balanced_distribution<part_type>()(part))
  {
    auto views = default_coarsener()(tuple<View>(view));

    const int num_ranges = get<0>(views).size();

    using container_t = static_array<std::pair<unsigned int, unsigned int>>;
    using view_t      = array_view<container_t>;

    container_t offsets_ct(num_ranges);
    container_t counts_ct(num_ranges);

    view_t offsets_vw(offsets_ct);
    view_t counts_vw(counts_ct);

    DECLARE_INLINE_INPUT_PLACEHOLDERS(3, in)
    DECLARE_INLINE_PLACEHOLDERS(6, x)

    using count_wf      = partition_range<RangePartitioner>;
    using reduce_wf     = partition_reduce;
    using count_type    = reduce_wf::result_type;
    using write_wf      = partition_write;

    count_type sums =
      execute(
        execution_params<count_type>(null_coarsener()),
        compose<tags::inline_flow>(
          x0 << zip<1>(count_wf(std::move(partitioner))) | (in0),
          x1 << scan(reduce_wf(), count_type(0,0))       | (x0),
          x2 << reduce(reduce_wf())                      | (x0),
          x3 << broadcast(stapl::identity_op())          | (x2),
          x4 << zip<4>(write_wf())                       | (x0, x1, in1, in2),
          x5 << broadcast_to_locs<true>()                | (x2)),
        get<0>(views), counts_vw, offsets_vw);

    exchange(view, counts_vw, offsets_vw, sums,
             b_maintain_stability, b_write_nmatches);

    return sums.first;
  }
  else
  {
    using tmp_ct_t = static_array<typename View::value_type, no_initialization>;
    using tmp_vw_t = array_view<tmp_ct_t>;

    tmp_ct_t dst_ct(view.size());
    tmp_vw_t dst_vw(dst_ct);

    auto vw2      = make_repeat_view(dst_vw);
    auto views    = default_coarsener()(make_tuple(view, vw2, dst_vw));

    DECLARE_INLINE_INPUT_PLACEHOLDERS(3, in)
    DECLARE_INLINE_PLACEHOLDERS(9, x)

    using count_wf      = partition_range<RangePartitioner>;
    using reduce_wf     = partition_reduce;
    using count_type    = reduce_wf::result_type;
    using write_wf      = partition_general_write;
    using write_back_wf = partition_write_back;

    /// Skeleton composition for algorithm.
    /// x0 does a local partition and returns match / nmatch count.
    /// x1 scans these counts to setup the write to temporary storage.
    /// x2, x3, and x8 reduce and broadcast the sum of counts both for
    /// the output of algorithm and the temporary storage write.
    /// x4, x5, and x6 complete the temporary writeback and ensure
    /// all updates are done before x7 copies the results back to the
    /// original input.
    count_type counts =
      execute(
        execution_params<count_type>(null_coarsener()),
        compose<tags::inline_flow>(
          x0 << zip<1>(count_wf(std::move(partitioner))) | (in0),
          x1 << scan(reduce_wf(), count_type(0,0))       | (x0),
          x2 << reduce(reduce_wf())                      | (x0),
          x3 << broadcast(stapl::identity_op())          | (x2),
          x4 << zip<5>(write_wf(b_write_nmatches))       | (in0,in1,x0,x1,x3),
          x5 << reduce(stapl::plus<size_t>())            | (x4),
          x6 << broadcast(stapl::identity_op())          | (x5),
          x7 << zip<3>(write_back_wf())                  | (x6, in2, in0),
          x8 << broadcast_to_locs<true>()                | (x2)),
        get<0>(views), get<1>(views), get<2>(views));

    return counts.first;
  }
}


//////////////////////////////////////////////////////////////////////
/// @brief Partition the input such that all elements for which the predicate
///   returns true are ordered before those for which it returns false.
///   This signature called when partitioned data is placed in a second view
////  provided as input.
///
/// @param src_view One-dimensional view of the input.  Non mutated.
/// @param dst_view One-dimensional view where partitioned sequence is written.
/// @param partitioner Functor used to partition each range.
/// @param b_maintain_stability Denotes whether exchange process should maintain
///   the relative order between elements on different locations headed to the
///   same target partition or if this can be relaxed to minimize communication
///   and thus increase performance.
/// @param b_write_nmatches If false, the mutated input will not be
///   guaranteed to be the elements from input for the non match portion of the
///   partition. Useful for algorithms such as remove.
/// @return The number of elements for which the predicate returns true.
/// @ingroup reorderingAlgorithms
/// @todo Merge as much of implementation as possible with single view version.
//////////////////////////////////////////////////////////////////////
template <typename SrcView, typename DstView, typename RangePartitioner>
size_t partition_impl(SrcView const& src_view,
                      DstView const& dst_view,
                      RangePartitioner partitioner,
                      bool b_maintain_stability,
                      bool b_write_nmatches)
{
  using namespace skeletons;

  auto const& src_part = src_view.container().distribution().partition();
  auto const& dst_part = dst_view.container().distribution().partition();

  using src_part_type = typename std::decay<decltype(src_part)>::type;
  using dst_part_type = typename std::decay<decltype(dst_part)>::type;

  if (is_balanced_distribution<src_part_type>()(src_part)
      && is_balanced_distribution<dst_part_type>()(dst_part))
  {
    auto views =
      default_coarsener()(tuple<SrcView, DstView>(src_view, dst_view));

    const int num_ranges = get<0>(views).size();

    using container_t = static_array<std::pair<unsigned int, unsigned int>>;
    using view_t      = array_view<container_t>;

    container_t offsets_ct(num_ranges);
    container_t counts_ct(num_ranges);

    view_t offsets_vw(offsets_ct);
    view_t counts_vw(counts_ct);

    DECLARE_INLINE_INPUT_PLACEHOLDERS(4, in)
    DECLARE_INLINE_PLACEHOLDERS(6, x)

    using count_wf      = partition_range<RangePartitioner>;
    using reduce_wf     = partition_reduce;
    using count_type    = reduce_wf::result_type;
    using write_wf      = partition_write;

    count_type sums =
      execute(
        execution_params<count_type>(null_coarsener()),
        compose<tags::inline_flow>(
          x0 << zip<2>(count_wf(std::move(partitioner))) | (in0, in1),
          x1 << scan(reduce_wf(), count_type(0,0))       | (x0),
          x2 << reduce(reduce_wf())                      | (x0),
          x3 << broadcast(stapl::identity_op())          | (x2),
          x4 << zip<4>(write_wf())                       | (x0, x1, in2, in3),
          x5 << broadcast_to_locs<true>()                | (x2)),
        get<0>(views), get<1>(views), counts_vw, offsets_vw);

    exchange(dst_view, counts_vw, offsets_vw, sums,
             b_maintain_stability, b_write_nmatches);

    return sums.first;
  }
  else
  {
    using tmp_ct_t =
      static_array<typename SrcView::value_type, no_initialization>;
    using tmp_vw_t = array_view<tmp_ct_t>;

    tmp_ct_t tmp_ct(src_view.size());
    tmp_vw_t tmp_vw(tmp_ct);

    auto vw2      = make_repeat_view(tmp_vw);
    auto views    =
      default_coarsener()(make_tuple(src_view, dst_view, vw2, tmp_vw));

    DECLARE_INLINE_INPUT_PLACEHOLDERS(4, in)
    DECLARE_INLINE_PLACEHOLDERS(9, x)

    using count_wf      = partition_range<RangePartitioner>;
    using reduce_wf     = partition_reduce;
    using count_type    = reduce_wf::result_type;
    using write_wf      = partition_general_write;
    using write_back_wf = partition_write_back;

    /// Skeleton composition for algorithm.
    /// x0 does a local partition and returns match / nmatch count.
    /// x1 scans these counts to setup the write to temporary storage.
    /// x2, x3, and x8 reduce and broadcast the sum of counts both for
    /// the output of algorithm and the temporary storage write.
    /// x4, x5, and x6 complete the temporary writeback and ensure
    /// all updates are done before x7 copies the results back to the
    /// original input.
    count_type counts =
      execute(
        execution_params<count_type>(null_coarsener()),
        compose<tags::inline_flow>(
          x0 << zip<2>(count_wf(std::move(partitioner))) | (in0, in1),
          x1 << scan(reduce_wf(), count_type(0,0))       | (x0),
          x2 << reduce(reduce_wf())                      | (x0),
          x3 << broadcast(stapl::identity_op())          | (x2),
          x4 << zip<5>(write_wf(b_write_nmatches))       | (in1,in2,x0,x1,x3),
          x5 << reduce(stapl::plus<size_t>())            | (x4),
          x6 << broadcast(stapl::identity_op())          | (x5),
          x7 << zip<3>(write_back_wf())                  | (x6, in3, in1),
          x8 << broadcast_to_locs<true>()                | (x2)),
        get<0>(views), get<1>(views), get<2>(views), get<3>(views));

    return counts.first;
  }
}


//////////////////////////////////////////////////////////////////////
/// @brief Copies the first argument to the
///   second unless the first meets the predicate provided, in which case
///   the given value is copied to the second argument.
/// @tparam Predicate Unary functor to test for the need for replacement.
/// @tparam T Type of the value to replace.
//////////////////////////////////////////////////////////////////////
template<typename Predicate, typename T>
struct copy_with_replacement
{
  Predicate m_pred;
  T m_new_value;

  typedef void result_type;

  copy_with_replacement(Predicate const& pred, T const& new_value)
    : m_pred(pred), m_new_value(new_value)
  { }

  template<typename Reference1, typename Reference2>
  void operator()(Reference1&& x, Reference2&& y)
  {
    if (m_pred(x))
      y = m_new_value;
    else
      y = x;
  }

  void define_type(typer& t)
  {
    t.member(m_pred);
    t.member(m_new_value);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function which assigns the new value to the input element if
///   the input element is equal to the given value.
/// @tparam T Types of the old and new values.
//////////////////////////////////////////////////////////////////////
template<typename T>
struct assign_if_equal
{
  const T m_old_value;
  const T m_new_value;

  typedef void   result_type;

  assign_if_equal(T const& old_value, T const& new_value)
    : m_old_value(old_value), m_new_value(new_value)
  { }

  void define_type(typer& t)
  {
    t.member(m_old_value);
    t.member(m_new_value);
  }

  template<typename Reference1>
  void operator()(Reference1 x) const
  {
    if (x == m_old_value)
      x = m_new_value;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Random number generator used by default in @ref random_shuffle
//////////////////////////////////////////////////////////////////////
template <typename Difference>
class default_random_number_generator
{
  typedef boost::random::uniform_int_distribution<Difference> distrib_type;
  boost::random::mt19937 m_gen;
public:
  default_random_number_generator(unsigned int seed)
    : m_gen(seed)
  { }

  Difference operator()(void)
  {
    return distrib_type()(m_gen);
  }

  Difference operator()(Difference n)
  {
    return distrib_type(0, n)(m_gen);
  }

  void define_type(typer& t)
  {
    t.member(m_gen);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function for @ref random_shuffle(), which computes the
///        random shuffle over the input, computes the output's index
///        targets.
///
/// The given set of elements are locally shuffled and the values are
/// scattered to specific well-defined positions on the output (@p v2)
/// avoiding write collisions among different locations.
///
/// @tparam RandomNumberGenerator The random number generator used to
///         shuffle.
//////////////////////////////////////////////////////////////////////
template<typename RandomNumberGenerator>
class shuffle_wf
{
  size_t m_num_partitions;
  RandomNumberGenerator m_rng;

public:
  typedef void result_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Work function constructor with a given number of
  ///        partitions (@p n) and the random number generator used to
  ///        compute the local shuffle.
  //////////////////////////////////////////////////////////////////////
  shuffle_wf(size_t n, RandomNumberGenerator rng)
    : m_num_partitions(n)
    , m_rng(rng)
  { }

  template<class View1, class View2>
  void operator()(View1&& v1, View2&& v2)
  {
    size_t target_size  = v2.size();

    std::random_shuffle(v1.begin(), v1.end(), m_rng);

    size_t blksz = round(target_size/m_num_partitions);

    // Generating the scatter target indices
    size_t j,pid,w,k=0;

    for (size_t i = v1.domain().first(); i <= v1.domain().last(); ++i) {
      j = i % blksz;          // relative local index
      pid = i / blksz;        // partition id
      w = pid + (j * blksz);  // scatter target position
      k = (w>=target_size) ? i : w;
      v2[k] = v1[i];
    }
  }

  void define_type(typer &t)
  {
    t.member(m_num_partitions);
    t.member(m_rng);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Returns a random access view to the given view. If the
///   input view does not have a scalar domain (e.g., domain used for
///   stapl::list), the elements are copied into a new container.
/// @tparam View Type of the input view.
/// @tparam Is_Index_Scalar Type of the view domain.
//////////////////////////////////////////////////////////////////////
template <typename View,typename Is_Index_Scalar>
struct get_input_view
{
  typedef static_array<typename View::value_type>  tmp_container_t;
  typedef array_view<tmp_container_t>              view_type;
  view_type operator()(const View& vw)
  {
    view_type tmp_view(new tmp_container_t(vw.size()));
    copy(vw,tmp_view);
    return tmp_view;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref get_input_view, which just returns the input
///   view, because it already is random access.
/// @tparam View The type of the input view.
/// @todo Specialize more based on the domain (sparse?).
//////////////////////////////////////////////////////////////////////
template <typename View>
struct get_input_view<View,size_t>
{
  typedef View view_type;
  view_type operator()(const View& vw)
  {
    return vw;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper function for @ref copy implementation that enables
/// use of coarsened assigment function using aggregate set_elements
/// when iterators are available (denoted by std::true_type parameter).
//////////////////////////////////////////////////////////////////////
template<typename View0, typename View1>
void copy_impl(std::true_type, View0 const& vw0, View1 const& vw1)
{
  map_func<skeletons::tags::with_coarsened_wf>(coarse_assign(), vw0, vw1);
}


//////////////////////////////////////////////////////////////////////
/// @brief Helper function for @ref copy implementation that uses
/// default, per element assignment function when iterators are not
/// available (denoted by std::false_type parameter).
//////////////////////////////////////////////////////////////////////
template<typename View0, typename View1>
void copy_impl(std::false_type, View0 const& vw0, View1 const& vw1)
{
  map_func(assign<typename View1::value_type>(), vw0, vw1);
}

} //namespace algo_details


//////////////////////////////////////////////////////////////////////
/// @brief Copy the elements of the input view to the output view.
/// @param vw0 One-dimensional view of the input.
/// @param vw1 One-dimensional view of the output.
/// @ingroup generatingAlgorithms
///
/// This algorithm mutates the output.
//////////////////////////////////////////////////////////////////////
template<typename View0, typename View1>
void copy(View0 const& vw0, View1 const& vw1)
{
  algo_details::copy_impl(
    skeletons::optimizers::helpers::pack_has_iterator<View0, View1>(),
    vw0, vw1);
}


//////////////////////////////////////////////////////////////////////
/// @brief Copy the first n elements from the input view to the output view.
/// @param vw0 One-dimensional view of the input.
/// @param vw1 One-dimensional view of the output.
/// @param n Number of elements to copy.
/// @ingroup generatingAlgorithms
///
/// This algorithm mutates the output.
//////////////////////////////////////////////////////////////////////
template<typename View0, typename View1, typename Size>
void copy_n(View0 const& vw0, View1 const& vw1, Size n)
{
  stapl_assert(n >= 0, "requested copy of negative number of elements");

  if (n == 0)
    return;

  typedef typename View0::domain_type dom_t;
  typedef typename View1::domain_type dom_t2;
  copy(
    View0(vw0.container(),
          dom_t(vw0.domain().first(),
                vw0.domain().advance(vw0.domain().first(),n-1))),
    View1(vw1.container(),
          dom_t2(vw1.domain().first(),
                 vw1.domain().advance(vw1.domain().first(),n-1))));
}


//////////////////////////////////////////////////////////////////////
/// @brief Copy elements of the input view to the output view in reverse order.
/// @param vw0 One-dimensional view of the input.
/// @param vw1 One-dimensional view of the output.
/// @ingroup reorderingAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename View0, typename View1>
void reverse_copy(View0 const& vw0, View1 const& vw1)
{
  copy(vw0, reverse_view(vw1));
}


//////////////////////////////////////////////////////////////////////
/// @brief Reverse the order of the elements in the input view.
/// @param vw0 One-dimensional view of the input.
/// @ingroup reorderingAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename View0>
void reverse(View0 const& vw0)
{
  using tmp_ct_t = static_array<typename View0::value_type>;

  tmp_ct_t             tmp_ct(vw0.size());
  array_view<tmp_ct_t> tmp_vw(tmp_ct);

  copy(vw0, tmp_vw);
  reverse_copy(tmp_vw, vw0);
}


//////////////////////////////////////////////////////////////////////
/// @brief Copy the elements in the input view to the output view rotated
///   to the left by k positions.
/// @param vw0 One-dimensional view of the input.
/// @param vw1 One-dimensional view of the output.
/// @param k Number of positions to shift the elements.
/// @ingroup reorderingAlgorithms
///
/// This algorithm mutates the output.
///
/// @todo Investigate more elegant way to construct a view with
/// a restricted domain.
///
/// @todo Remove fallback to non-composed skeleton when issue with
/// non equal spans is resolved.
//////////////////////////////////////////////////////////////////////
template<typename View0, typename View1>
void rotate_copy(View0 const& vw0, View1 const& vw1, int k)
{
  using dom0_t = typename View0::domain_type;
  using dom1_t = typename View1::domain_type;

  dom0_t dom0 = vw0.domain();
  dom1_t dom1 = vw1.domain();

  const size_t size_vw = vw0.size();

  if (size_vw == 0)
    return;

  k = k % size_vw;

  if (k == 0)
  {
    copy(vw0, vw1);
    return;
  }

  // else
  if (k < 0)
    k += size_vw;

  const size_t size1 = k;
  const size_t size2 = size_vw - size1;

  dom0_t dom0_1(dom0.first(), dom0.advance(dom0.first(), size1-1), dom0);
  dom1_t dom1_1(dom1.advance(dom1.first(), size2), dom1.last(), dom1);

  dom0_t dom0_2(dom0.advance(dom0.first(),size1), dom0.last(), dom0);
  dom1_t dom1_2(dom1.first(), dom1.advance(dom1.first(),size2-1), dom1);

  View0 v0(vw0.container(), dom0_1);
  View1 v1(vw1.container(), dom1_1);
  View0 v2(vw0.container(), dom0_2);
  View1 v3(vw1.container(), dom1_2);

  auto views0 = default_coarsener()(make_tuple(v0, v1));
  auto views1 = default_coarsener()(make_tuple(v2, v3));

  using namespace skeletons;

  // Skeleton composition needs support for unequal spans to
  // unconditionally use compose.
  if (get<0>(views0).size() == get<0>(views1).size())
  {
    DECLARE_INLINE_INPUT_PLACEHOLDERS(4, in)
    DECLARE_INLINE_PLACEHOLDERS(2, x)

    skeletons::execute(
      skeletons::execution_params(null_coarsener()),
      compose<tags::inline_flow>(
        x0 << zip(coarse_assign{}) | (in0, in1),
        x1 << zip(coarse_assign{}) | (in2, in3)),
      get<0>(views0), get<1>(views0),
      get<0>(views1), get<1>(views1));

    return;
  }

  skeletons::execute(
    skeletons::execution_params(null_coarsener()),
     zip<2>(coarse_assign{}), get<0>(views0), get<1>(views0));

  skeletons::execute(
    skeletons::execution_params(null_coarsener()),
     zip<2>(coarse_assign{}), get<0>(views1), get<1>(views1));
}


//////////////////////////////////////////////////////////////////////
/// @brief Rotate the elements in the view to the left by k positions.
/// @param vw1 One-dimensional view of the input.
/// @param k Number of positions to shift the elements.
/// @ingroup reorderingAlgorithms
///
/// This algorithm mutates the input.
//////////////////////////////////////////////////////////////////////
template<typename View0>
void rotate(View0 const& vw1, int k)
{
  using new_value = typename View0::value_type;

  static_array<new_value>             tmp_array(vw1.size());
  array_view<static_array<new_value>> tmp_vw(tmp_array);

  copy(vw1, tmp_vw);
  rotate_copy(tmp_vw, vw1, k);
}


//////////////////////////////////////////////////////////////////////
/// @brief Work function invokes sequential (ie STL) @p partition.
//////////////////////////////////////////////////////////////////////
template<typename Predicate>
class serial_partition
{
private:
  Predicate m_predicate;

public:
  serial_partition(Predicate predicate)
    : m_predicate(std::move(predicate))
  { }

  template<typename View>
  typename std::decay<View>::type::iterator
  operator()(View&& vw) const
  { return std::partition(vw.begin(), vw.end(), m_predicate); }

  void define_type(typer& t)
  { t.member(m_predicate); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function invokes sequential (ie STL) @p stable_partition.
//////////////////////////////////////////////////////////////////////
template<typename Predicate>
class serial_stable_partition
{
private:
  Predicate m_predicate;

public:
  serial_stable_partition(Predicate predicate)
    : m_predicate(std::move(predicate))
  { }

  template<typename View>
  typename std::decay<View>::type::iterator
  operator()(View&& vw) const
  { return std::stable_partition(vw.begin(), vw.end(), m_predicate); }

  void define_type(typer& t)
  { t.member(m_predicate); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Partition the input such that all elements for which the predicate
///   returns true are ordered before those for which it returns false, while
///   also maintaining the relative ordering of the elements.
/// @param pview One-dimensional view of the input.
/// @param predicate Unary functor used to partition the input.
/// @return The number of elements for which the predicate returns true.
/// @ingroup reorderingAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename View, typename Predicate>
size_t stable_partition(View const& view, Predicate predicate)
{
  return algo_details::partition_impl(
    view, serial_stable_partition<Predicate>(std::move(predicate)), true, true);
}


//////////////////////////////////////////////////////////////////////
/// @brief Copies all elements from the input for which the functor returns
///   true into the first output view, and all others into the second output.
/// @param pview0 One-dimensional view of the input.
/// @param pview1 One-dimensional view of the output of elements returning true.
/// @param pview2 One-dimensional view of the output of elements returning
///   false.
/// @param predicate Unary function used to partition the input.
/// @return Pair containing views over the copied output ranges.
/// @ingroup reorderingAlgorithms
///
/// This algorithm mutates the two output views.
//////////////////////////////////////////////////////////////////////
template <typename View0, typename View1, typename View2, typename Pred>
std::pair<View1, View2>
partition_copy(View0 const& pview0,
               View1 const& pview1,
               View2 const& pview2,
               Pred predicate)
{
  typedef list<typename View0::value_type> plist_type;
  typedef list_view <plist_type>           plist_view_type;
  typedef typename View1::domain_type      dom_t1;
  typedef typename View2::domain_type      dom_t2;

  plist_type                               list_pred;
  plist_type                               list_not_pred;

  typedef algo_details::partition_apply_pred<
    Pred, typename View0::value_type
  > wf_t;

  map_func(wf_t(predicate, &list_pred, &list_not_pred), pview0);

  size_t n1;
  size_t n2;

  if (list_pred.size() != 0)
    n1 = list_pred.size() - 1;
  else
    n1 = list_pred.size();

  if (list_not_pred.size() != 0)
    n2 = list_not_pred.size() - 1;
  else
    n2 = list_not_pred.size();

  View1 nv1(pview1.container(),
            dom_t1(pview1.domain().first(),
                   pview1.domain().advance(pview1.domain().first(),n1)),
            pview1.mapfunc());

  View2 nv2(pview2.container(),dom_t2(pview2.domain().first(),
            pview2.domain().advance(pview2.domain().first(),n2)),
            pview2.mapfunc() );

  if (list_pred.size() != 0)
  {
    copy(plist_view_type(list_pred), nv1);
  }

  if (list_not_pred.size()!=0)
  {
    copy(plist_view_type(list_not_pred), nv2);
  }

  return std::make_pair(nv1, nv2);
}


//////////////////////////////////////////////////////////////////////
/// @brief Partition the input such that all elements for which the predicate
///   returns true are ordered before those for which it returns false.
/// @param pview One-dimensional view of the input.
/// @param predicate Unary functor used to partition the input.
/// @ingroup reorderingAlgorithms
//////////////////////////////////////////////////////////////////////
template <typename View, typename Predicate>
size_t partition(View const& view, Predicate predicate)
{
  return algo_details::partition_impl(
    view, serial_partition<Predicate>(std::move(predicate)), false, true);
}


//////////////////////////////////////////////////////////////////////
/// @brief Assign each value of the input view to the result of calling the
///   provided functor.
/// @param view One-dimensional view of the input.
/// @param gen Nullary functor which is called to generate elements.
/// @ingroup generatingAlgorithms
///
/// This algorithm mutates the input view.
//////////////////////////////////////////////////////////////////////
template<typename View, typename Generator>
void generate(View const& view, Generator gen)
{
  typedef typename View::value_type             value_t;
  copy(functor_view(view.size(),
                    offset_gen<std::size_t,value_t,Generator>(gen)),
       view);
}

//////////////////////////////////////////////////////////////////////
/// @brief Assign the n values of the input view starting at the given element
///   to the result of calling the provided functor.
/// @param view One-dimensional view of the input.
/// @param first_elem First element to fill with a generated value.
/// @param n Number of elements to fill with generated values.
/// @param gen Nullary functor which is called to generate elements.
/// @ingroup generatingAlgorithms
///
/// This algorithm mutates the input view.
//////////////////////////////////////////////////////////////////////
template <typename View, typename Generator>
void generate_n(View const& view, size_t first_elem, size_t n, Generator gen)
{
  typedef typename View::domain_type dom_t;
  dom_t dom = view.domain();
  dom_t new_dom(dom.advance(dom.first(),first_elem),
                dom.advance(dom.first(),first_elem + n - 1), dom);
  View view2(view.container(), new_dom);
  generate(view2, gen);
}

//////////////////////////////////////////////////////////////////////
/// @brief Replace the values from the input view for which the given predicate
///   returns true with the new value.
/// @param vw One-dimensional view of the input.
/// @param pred Unary functor which returns true for replaced elements.
/// @param new_value Value used to replace elements.
/// @ingroup generatingAlgorithms
///
/// This algorithm mutates the input view.
//////////////////////////////////////////////////////////////////////
template<typename View, typename Predicate>
void replace_if(View& vw, Predicate pred,
                typename View::value_type const& new_value)
{
  stapl::map_func(algo_details::assign_if<Predicate, typename View::value_type>
                   (pred, new_value), vw);
}

//////////////////////////////////////////////////////////////////////
/// @brief Replace the given value in the input with the new value.
/// @param vw One-dimensional view of the input.
/// @param old_value Value replaced in the input.
/// @param new_value Value used to replace old values.
/// @ingroup generatingAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename View>
void replace(View& vw, typename View::value_type const& old_value,
             typename View::value_type const& new_value)
{
  map_func<skeletons::tags::with_coarsened_wf>(
      algo_details::serial_replace<typename View::value_type>(
        old_value, new_value),
      vw);
}



//////////////////////////////////////////////////////////////////////
/// @brief Copy the values from the input view to the output, except for those
///   elements for which the given predicate returns true, which are replaced
///   with the given value.
/// @param vw0 One-dimensional view of the input.
/// @param vw1 One-dimensional view of the output.
/// @param pred Unary functor which returns true for replaced elements.
/// @param new_value Value used to replace elements for which the functor
///   returns true.
/// @return Iterator pointing to the end of the output view.
/// @ingroup generatingAlgorithms
///
/// This algorithm mutates the output view. The input and output views must be
/// the same size.
//////////////////////////////////////////////////////////////////////
template<typename View0, typename View1, typename Predicate>
typename View1::iterator
replace_copy_if(View0 const& vw0, View1 const& vw1, Predicate pred,
                typename View0::value_type new_value)
{
  stapl::map_func(
      algo_details::copy_with_replacement<Predicate, typename View0::value_type>
        (pred, new_value), vw0, vw1);
  return vw1.end();
}

//////////////////////////////////////////////////////////////////////
/// @brief Copy the elements from the input to the output, replacing the given
///   old_value with the new_value.
/// @param vw0 One-dimensional view of the input.
/// @param vw1 One-dimensional view of the output.
/// @param old_value The old value to replace.
/// @param new_value The new value to substitute for occurrences of old_value.
/// @return Iterator to the end of the newly copied view.
/// @ingroup generatingAlgorithms
///
/// This algorithm mutates the output view, and requires a view with iterator
/// support. It uses stapl::equal_to for comparisons.
///
/// @todo Track down why std::bind1st seems to be seeping into stapl namespace
/// with icc compiler and requiring explicit qualification of stapl::bind1st.
//////////////////////////////////////////////////////////////////////
template<typename View0, typename View1>
typename View1::iterator
replace_copy(View0& vw0, View1& vw1, typename View0::value_type old_value,
             typename View0::value_type new_value)
{
  replace_copy_if(vw0, vw1,
                  stapl::bind2nd(equal_to<typename View0::value_type>(),
                                 old_value),
                   new_value);
  return vw1.end();
}


namespace algo_details{

template<typename T>
struct coarsened_fill
{
private:
  T m_value;

public:

  typedef void result_type;

  coarsened_fill(T value)
    : m_value(std::move(value))
  { }

  template<typename View>
  void operator()(View&& view)
  {
    std::fill(view.begin(), view.end(), m_value);
  }

  void define_type(typer& t)
  {
    t.member(m_value);
  }
};

} //namespace algo_details

//////////////////////////////////////////////////////////////////////
/// @brief Assigns the given value to the first n elements of the input view.
/// @param vw One-dimensional view of the input.
/// @param value The value to fill into the input.
/// @param n Number of elements to fill.
/// @ingroup generatingAlgorithms
///
/// This algorithm mutates the input view. The input must be at least n in size.
///
/// @todo Track down why std::bind1st seems to be seeping into stapl namespace
/// with icc compiler and requiring explicit qualification of stapl::bind1st.
//////////////////////////////////////////////////////////////////////
template<typename View, typename Size>
void fill_n(View& vw, typename View::value_type value, Size n)
{
  if (n == 0)
    return;

  View nview(vw.container(),
             typename View::domain_type(vw.domain().first(),
                                        vw.domain().first() + n - 1));

  map_func<skeletons::tags::with_coarsened_wf>(
      algo_details::coarsened_fill<typename View::value_type>
        (value), nview);
}

//////////////////////////////////////////////////////////////////////
/// @brief Assigns the given value to the elements of the input view. Fill
///        implementation for Views that have an iterator type for range-based
///        usage.
/// @param vw One-dimensional view of the input.
/// @param value The value to fill into the input.
/// @ingroup generatingAlgorithms
///
/// This algorithm mutates the input view.
///
/// @todo Track down why std::bind1st seems to be seeping into stapl namespace
/// with icc compiler and requiring explicit qualification of stapl::bind1st.
//////////////////////////////////////////////////////////////////////
template<typename View,
         typename std::enable_if<has_iterator<View>::value, int>::type = 0 >
void fill(View const& vw, typename View::value_type value)
{
  map_func<skeletons::tags::with_coarsened_wf>(
      algo_details::coarsened_fill<typename View::value_type>
        (value), vw);
}

//////////////////////////////////////////////////////////////////////
/// @brief Assigns the given value to the elements of the input view. Fill
///        implementation for Views that does not have iterator type.
/// @param vw One-dimensional view of the input.
/// @param value The value to fill into the input.
/// @ingroup generatingAlgorithms
///
/// This algorithm mutates the input view.
///
/// @todo Track down why std::bind1st seems to be seeping into stapl namespace
/// with icc compiler and requiring explicit qualification of stapl::bind1st.
//////////////////////////////////////////////////////////////////////
template<typename View,
         typename std::enable_if<!has_iterator<View>::value, int>::type = 0 >
void fill(View const& vw, typename View::value_type value)
{
  stapl::map_func(stapl::bind1st(assign<typename View::value_type>(), value),
                  vw);
}


//////////////////////////////////////////////////////////////////////
/// @brief Work function performs sequential keep_if by using negated
/// predicate with STL @p remove_if.
//////////////////////////////////////////////////////////////////////
template<typename Predicate>
class serial_keep_if
{
private:
  stapl::unary_negate<Predicate> m_negated_predicate;

public:
  serial_keep_if(Predicate predicate)
    : m_negated_predicate(std::move(predicate))
  { }

  template<typename View>
  typename std::decay<View>::type::iterator
  operator()(View&& vw) const
  { return std::remove_if(vw.begin(), vw.end(), m_negated_predicate); }

  void define_type(typer& t)
  { t.member(m_negated_predicate); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function performs sequential remove.
//////////////////////////////////////////////////////////////////////
template<typename Value>
class serial_remove
{
private:
  Value m_value;

public:
  serial_remove(Value value)
    : m_value(std::move(value))
  { }

  template<typename View>
  typename std::decay<View>::type::iterator
  operator()(View&& vw) const
  { return std::remove(vw.begin(), vw.end(), m_value); }

  void define_type(typer& t)
  { t.member(m_value); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function performs sequential remove_copy.
//////////////////////////////////////////////////////////////////////
template<typename Value>
class serial_remove_copy
{
private:
  Value m_value;

public:
  serial_remove_copy(Value value)
    : m_value(std::move(value))
  { }

  template<typename SrcView, typename DstView>
  typename std::decay<DstView>::type::iterator
  operator()(SrcView&& src_vw, DstView&& dst_vw) const
  {
    return std::remove_copy(src_vw.begin(), src_vw.end(),
                            dst_vw.begin(), m_value);
  }

  void define_type(typer& t)
  { t.member(m_value); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function performs sequential remove_if.
//////////////////////////////////////////////////////////////////////
template<typename Predicate>
class serial_remove_if
{
private:
  Predicate m_predicate;

public:
  serial_remove_if(Predicate predicate)
    : m_predicate(std::move(predicate))
  { }

  template<typename View>
  typename std::decay<View>::type::iterator
  operator()(View&& vw) const
  { return std::remove_if(vw.begin(), vw.end(), m_predicate); }

  void define_type(typer& t)
  { t.member(m_predicate); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function performs sequential copy_if.
//////////////////////////////////////////////////////////////////////
template<typename Predicate>
class serial_copy_if
{
private:
  Predicate m_predicate;

public:
  serial_copy_if(Predicate predicate)
    : m_predicate(std::move(predicate))
  { }

  template<typename SrcView, typename DstView>
  typename std::decay<DstView>::type::iterator
  operator()(SrcView&& src_vw, DstView&& dst_vw) const
  {
    return std::copy_if(src_vw.begin(), src_vw.end(),
                        dst_vw.begin(), m_predicate);
  }

  void define_type(typer& t)
  { t.member(m_predicate); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function performs sequential remove_copy_if.
//////////////////////////////////////////////////////////////////////
template<typename Predicate>
class serial_remove_copy_if
{
private:
  Predicate m_predicate;

public:
  serial_remove_copy_if(Predicate predicate)
    : m_predicate(std::move(predicate))
  { }

  template<typename SrcView, typename DstView>
  typename std::decay<DstView>::type::iterator
  operator()(SrcView&& src_vw, DstView&& dst_vw) const
  {
    return std::remove_copy_if(src_vw.begin(), src_vw.end(),
                               dst_vw.begin(), m_predicate);
  }

  void define_type(typer& t)
  { t.member(m_predicate); }
};


template<typename Predicate, typename View0, typename... Views>
View0
remove_impl(Predicate predicate, View0 const& view0, Views const&... views)
{
  using domain_type = typename View0::domain_type;

  const size_t true_size = algo_details::partition_impl(
    view0, views..., std::move(predicate), true, false);

  if (true_size != 0)
  {
    domain_type new_dom(
      view0.domain().first(),
      view0.domain().advance(view0.domain().first(), true_size - 1),
      view0.domain());

    return View0(view0.container(), new_dom);
  }

  // else
  return View0(view0.container(), domain_type());
}


//////////////////////////////////////////////////////////////////////
/// @brief Remove the values from the input view for which the given predicate
///   returns false.
/// @param view One-dimensional view of the input.
/// @param predicate Unary functor which returns true for elements to be kept.
/// @return View over the elements remaining in the input, which is the same or
///   smaller size as the provided input view.
/// @ingroup removingAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename View, typename Predicate>
View keep_if(View const& view, Predicate predicate)
{
  return remove_impl(serial_keep_if<Predicate>(std::move(predicate)), view);
}


//////////////////////////////////////////////////////////////////////
/// @brief Remove the values from the input view for which the given predicate
///   returns true.
/// @param view One-dimensional view of the input.
/// @param predicate Unary functor which returns true for removed elements.
/// @return View over the elements remaining in the input, which is the same or
///   smaller size than the provided input view.
/// @ingroup removingAlgorithms
///
/// This algorithm mutates the input view.
//////////////////////////////////////////////////////////////////////
template<typename View, typename Predicate>
View remove_if(View const& view, Predicate predicate)
{
  return remove_impl(serial_remove_if<Predicate>(std::move(predicate)), view);
}


//////////////////////////////////////////////////////////////////////
/// @brief Copy the values from the input view to the output those
///   elements for which the given predicate returns true.
/// @param vw0 One-dimensional view of the input.
/// @param vw1 One-dimensional view of the output.
/// @param predicate Unary functor which returns true for copied elements.
/// @return View over the copied range, which is the same as or smaller size
///   than the provided output view.
/// @ingroup removingAlgorithms
///
/// This algorithm mutates the output view. The input and output views must be
/// the same size.
//////////////////////////////////////////////////////////////////////
template<typename View0, typename View1, typename Predicate>
View1 copy_if(View0 const& vw0, View1 & vw1, Predicate predicate)
{
  return remove_impl(serial_copy_if<Predicate>(std::move(predicate)), vw0, vw1);
}

//////////////////////////////////////////////////////////////////////
/// @brief Copy the values from the input view to the output, except for those
///   elements for which the given predicate returns true.
/// @param vw0 One-dimensional view of the input.
/// @param vw1 One-dimensional view of the output.
/// @param predicate Unary functor which returns true for removed elements.
/// @return View over the copied range, which is the same as or smaller size
///   than the provided output view.
/// @ingroup removingAlgorithms
///
/// This algorithm mutates the output view. The input and output views must be
/// the same size.
//////////////////////////////////////////////////////////////////////
template<typename View0, typename View1, typename Predicate>
View1 remove_copy_if(View0 const& vw0, View1 & vw1, Predicate predicate)
{
  return remove_impl(serial_copy_if<Predicate>(std::move(predicate)), vw0, vw1);
}

//////////////////////////////////////////////////////////////////////
/// @brief Remove the given value from the input.
/// @param vw0 One-dimensional view of the input.
/// @param valuetoremove Value removed from the input.
/// @return View over the new range, which is the same as or smaller than the
///   provided input view.
/// @ingroup removingAlgorithms
///
/// This algorithm mutates the input view. The comparison is done with
/// stapl::equal_to.
//////////////////////////////////////////////////////////////////////
template<typename View>
View remove(View & vw0, typename View::value_type valuetoremove)
{
  return remove_impl(
    serial_remove<typename View::value_type>(std::move(valuetoremove)), vw0);
}

//////////////////////////////////////////////////////////////////////
/// @brief Copy the values from the input view to the output, except for those
///   elements which are equal to the given value.
/// @param vw0 One-dimensional view of the input.
/// @param vw1 One-dimensional view of the output.
/// @param valuetoremove Value removed from the output.
/// @return View over the copied range, which is the same as or smaller size
///   than the provided output view.
/// @ingroup removingAlgorithms
///
/// This algorithm mutates the output view. The comparison is done with
/// stapl::equal_to.
//////////////////////////////////////////////////////////////////////
template<typename View0, typename View1>
View1 remove_copy(View0 const& vw0, View1 & vw1,
    typename View0::value_type valuetoremove)
{
  return remove_impl(
    serial_remove_copy<typename View0::value_type>(std::move(valuetoremove)),
    vw0, vw1);
}


//////////////////////////////////////////////////////////////////////
/// @brief Swaps the elements of the two input views.
/// @param vw0 One-dimensional view of the first input.
/// @param vw1 One-dimensional view of the second input.
/// @ingroup generatingAlgorithms
///
/// This algorithm mutates both input views, and requires that both views are
/// the same size.
//////////////////////////////////////////////////////////////////////
template<typename View>
void swap_ranges(View& vw0, View& vw1)
{
  stapl::map_func<skeletons::tags::with_coarsened_wf>(
       coarse_swap_ranges<typename View::value_type>(),vw0,vw1);
}




namespace algo_details {

//////////////////////////////////////////////////////////////////////
/// @brief Function which applies given functor to a range of elements.
/// @tparam Functor Functor to apply to the range.
/// @ingroup mutatingFunctionObjects
//////////////////////////////////////////////////////////////////////
template<typename Functor>
struct for_each_coarsened
  : private Functor
{
  using result_type = void;

  for_each_coarsened(Functor f)
    : Functor(std::move(f))
  { }

  void define_type(typer& t)
  {
    t.base<Functor>(*this);
  }

  template<typename Ref1>
  void operator()(Ref1&& x)
  {
    std::for_each(x.begin(), x.end(), static_cast<Functor>(*this));
  }

};

}// namespace algo_details


//////////////////////////////////////////////////////////////////////
/// @brief Applies the given functor to all of the elements in the input.
/// @param vw0 One-dimensional view over the input.
/// @param func Unary functor to apply to the elements.
/// @return The functor that was passed as input.
/// @ingroup generatingAlgorithms
///
/// This algorithm will mutate the input view.
//////////////////////////////////////////////////////////////////////
template<typename View0, typename Function>
Function
for_each(const View0& vw0, Function func)
{
  stapl::map_func<skeletons::tags::with_coarsened_wf>(
      algo_details::for_each_coarsened<Function>(func), vw0);
  return func;
}


namespace algo_details {

//////////////////////////////////////////////////////////////////////
/// @brief Function which applies given functor (unary or binary) to a range of
///        elements and stores the result in other range.
/// @tparam Functor Functor to apply to the range.
/// @ingroup mutatingFunctionObjects
//////////////////////////////////////////////////////////////////////
template<typename Functor>
struct transform_assign_coarsened
{
private:
  Functor m_f;

public:

  typedef void result_type;

  transform_assign_coarsened(Functor f)
    : m_f(std::move(f))
  { }

  void define_type(typer& t)
  {
    t.member<Functor>(m_f);
  }

  template<typename Ref1, typename Ref2>
  void operator()(Ref1&& x, Ref2&& y)
  {
    std::transform(x.begin(), x.end(), y.begin(), m_f);
  }

  template<typename Ref1, typename Ref2, typename Ref3>
  void operator()(Ref1&& x, Ref2&& y, Ref3&& z)
  {
    std::transform(x.begin(), x.end(), y.begin(), z.begin(), m_f);
  }
};

}// namespace algo_details

//////////////////////////////////////////////////////////////////////
/// @brief Applies the given function to the input, and stores the result in
///   the output.
/// @param vw0 One-dimensional view over the input.
/// @param vw1 One-dimensional view over the output.
/// @param func Unary function which is applied to all of the input elements.
/// @ingroup generatingAlgorithms
///
/// This algorithm mutates the output view only.
//////////////////////////////////////////////////////////////////////
template<typename View0, typename View1, typename Function>
void transform(const View0& vw0, const View1& vw1, Function func)
{
  map_func<skeletons::tags::with_coarsened_wf>(
    algo_details::transform_assign_coarsened<Function>(func), vw0, vw1);
}


//////////////////////////////////////////////////////////////////////
/// @brief Applies the given function to the inputs, and stores the result in
///   the output.
/// @param vw0 One-dimensional view over the first input.
/// @param vw1 One-dimensional view over the second input.
/// @param vw2 One-dimensional view over the output.
/// @param func Binary function which is applied to all of the input elements.
/// @ingroup generatingAlgorithms
///
/// This algorithm mutates the output view only.
//////////////////////////////////////////////////////////////////////
template<typename View0, typename View1, typename View2, typename Function>
void transform(View0& vw0, View1& vw1, View2& vw2, Function func)
{
  stapl::map_func(transform_assign<Function>(func), vw0, vw1, vw2);
}


//////////////////////////////////////////////////////////////////////
/// @brief Finds the position of the first element for which the functor returns
///   false, indicating the partition point.
/// @param pview One-dimensional view of the input, which must be partitioned.
/// @param predicate Unary functor used to partition the input.
/// @return A reference to the first element for which the functor returns
///   false.
/// @ingroup searchAlgorithms
///
/// This algorithm is non-mutating, and requires that the input view be
/// partitioned.
//////////////////////////////////////////////////////////////////////
template <typename View, typename Pred>
typename View::reference
partition_point(View const& pview, Pred predicate)
{
  return find_if_not(pview, predicate);
};


namespace algo_details {

//////////////////////////////////////////////////////////////////////
/// @brief Convert boolean return value of partition predicate passed to
///   @ref is_partitioned() to int, so that it is in the tribool form
///   expected by the @ref is_partitioned_reduce reduction operator
///   of the @ref stapl::map_reduce.
//////////////////////////////////////////////////////////////////////
template<typename WF>
class convert_to_int
{
private:
  WF m_wf;

public:
  convert_to_int(WF const& wf)
    : m_wf(wf)
  { }

  typedef int result_type;

  template<typename View>
  int operator()(View&& elem) const
  {
    auto result = std::is_partitioned(elem.begin(), elem.end(), m_wf);

    // If this set of elements is partitioned then the result of the predicate
    // on the last element is converted for the reduction operator.
    // Otherwise, return 2 to indicate that a non-partitioned set of elements
    // has been found.
    if (result)
      return m_wf(*std::prev(elem.end()));
    else
      return 2;
  }

  void define_type(typer& t)
  {
    t.member(m_wf);
  }
}; // class convert_to_int


//////////////////////////////////////////////////////////////////////
/// @brief Work function used in @ref is_partitioned() to determine whether the
///   input is partitioned.
//////////////////////////////////////////////////////////////////////
template<typename T>
struct is_partitioned_reduce
{
  typedef int result_type;

  template<typename Reference1, typename Reference2>
  int operator()(Reference1&& elem1, Reference2&& elem2) const
  {
    if (elem1 == 1 && elem2 == 1)
      return 1;

    if (elem1 == 1 && elem2 == 0)
      return 0;

    if (elem1 == 0 && elem2 == 0)
      return 0;

    if (elem1 == 0 && elem2 == 1)
      return 2;

    if (elem1 == 2 || elem2 == 2)
      return 2;

    // else
    return 2;
  }
}; // struct is_partitioned_reduce

} // namespace algo_details


//////////////////////////////////////////////////////////////////////
/// @brief Decides if the input view is partitioned according to the given
///   functor, in that all elements which return true precede all those that do
///   not.
/// @param pview One-dimensional view of the input.
/// @param predicate Unary functor used to check the partitioning.
/// @return True if the input is partitioned, false otherwise.
/// @ingroup summaryAlgorithms
///
/// This algorithm is non-mutating.
//////////////////////////////////////////////////////////////////////
template <typename View, typename Predicate>
bool
is_partitioned(View const& pview, Predicate predicate)
{
  const int result = map_reduce<skeletons::tags::with_coarsened_wf>(
    algo_details::convert_to_int<Predicate>(predicate),
    algo_details::is_partitioned_reduce<int>(),
    pview
  );

  if (result == 0 || result == 1)
    return true;

  // else
 return false;
}


//////////////////////////////////////////////////////////////////////
/// @brief Computes a random shuffle of elements in the input view.
///
/// @param view One-dimensional view of the input.
/// @param rng The random number generator to use.
/// @ingroup reorderingAlgorithms
///
/// This algorithm is mutating.
/////////////////////////////////////////////////////////////////////
template <typename View, typename RandomNumberGenerator>
void random_shuffle(View const& view, RandomNumberGenerator const& rng)
{
  typedef static_array<typename View::value_type>   tmp_container_t;
  typedef array_view<tmp_container_t>               tmp_view_t;
  typedef algo_details::get_input_view<
    View, typename View::index_type>                input_view_t;
  typedef typename stapl::result_of::native_view<
    typename input_view_t::view_type>::type         partitioner_t;

  if (view.size()==0) return;

  // Temporary result
  tmp_view_t result_view(new tmp_container_t(view.size()));

  auto input_view = input_view_t()(view);
  partitioner_t pview = stapl::native_view(input_view);

  stapl::map_func(algo_details::shuffle_wf<RandomNumberGenerator>(
                    pview.size(),rng), pview, make_repeat_view(result_view));

  copy(result_view,view);
}

//////////////////////////////////////////////////////////////////////
/// @brief Computes a random shuffle of elements in the input view
/// @param view One-dimensional view of the input.
/// @ingroup reorderingAlgorithms
///
/// This version calls the predicated version with a default predicate of
/// stapl::algo_details::default_random_number_generator.
/// This algorithm is mutating.
//////////////////////////////////////////////////////////////////////
template <typename View>
void random_shuffle(View const& view)
{
  typedef typename View::const_iterator::difference_type difference_type;
  random_shuffle(view,
    algo_details::default_random_number_generator<difference_type>(
      view.get_location_id()));
}


//////////////////////////////////////////////////////////////////////
/// @brief Computes a random shuffle of elements in the input view, using the
///   given uniform random number generator.
/// @param view One-dimensional view of the input.
/// @param rng The random number generator to use.
/// @ingroup reorderingAlgorithms
///
/// This algorithm is mutating.
//////////////////////////////////////////////////////////////////////
template <typename View, typename UniformRandomNumberGenerator>
void shuffle(const View& view, const UniformRandomNumberGenerator& rng)
{
  algo_details::shuffle_random_number_generator<UniformRandomNumberGenerator>
    srg(rng);
  random_shuffle(view, srg);
}


//////////////////////////////////////////////////////////////////////
/// @brief Work function for first phase of @ref unique_copy.  Scans
/// the given range of elements and returns the number of unique
/// (ie non adjacent identical elements) in the range and a vector
/// representing the domains of the input these uniques occur in.
/// @sa range_adjacent_find
//////////////////////////////////////////////////////////////////////
class unique_compute_offsets
{
private:
  using vector_t = std::vector<std::pair<size_t, size_t>>;

public:
  using result_type = std::pair<size_t, vector_t>;

  template<typename View>
  result_type operator()(View&& vw1) const
  {
    size_t write_offset = 0;

    auto vw = vw1.container().view();
    using domain_type = typename decltype(vw)::domain_type;
    auto const& last_idx = vw.domain().last();

    vector_t ranges;

    if (vw1.domain().last() == last_idx) {
      if (vw1.domain().size() == 1)
        return std::make_pair(last_idx+1, ranges);

      vw.set_domain(
        domain_type(vw1.domain().first(), vw1.domain().last()));
    }
    else
      vw.set_domain(
        domain_type(vw1.domain().first(), vw1.domain().last() + 1));

    const size_t end_adjustment =
      vw.domain().last() == last_idx ? 1 : 2;

    auto iter   = vw.begin();
    auto e_iter = vw.end();
    auto b_iter = iter;

    // Iterate until we've processed all of vw
    while (iter != e_iter)
    {
      auto dupe_iter = std::adjacent_find(iter, e_iter);

      if (dupe_iter == iter)
      {
        ++iter;
        continue;
      }

      const size_t first_read = std::distance(b_iter, iter);
      size_t last_read        = std::distance(b_iter, dupe_iter);

      // Adjust range to include the first element of the duplicate pair.
      if (dupe_iter != e_iter)
        ++iter;
      else
        last_read -= end_adjustment;

      // Adjust offset to write to in next iteration
      write_offset += last_read - first_read + 1;

      ranges.push_back(std::make_pair(first_read, last_read));

      // Move forward the beginning marker. Iterate past the other duplicate
      // if there was one (i.e., we are not at the end).
      iter = dupe_iter;

      if (iter != e_iter)
        ++iter;
    }

    return std::make_pair(write_offset, std::move(ranges));
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper workfunction to strip off first element of pair and
/// forward to next step in skeleton composition of @ref unique_copy.
//////////////////////////////////////////////////////////////////////
struct unique_take_first
{
  using result_type = size_t;

  template<typename Reference>
  result_type operator()(Reference&& ref) const
  { return ref.first; }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function for last phase of unique_copy. For given input
/// view, write unique elements (defined by @p ranges input) to the
/// destination view, starting at the @p offset position.
/// @sa range_adjacent_find
//////////////////////////////////////////////////////////////////////
struct unique_update
{
  using result_type = void;

  template<typename SrcView, typename DstView,
           typename OffsetRef, typename Ranges>
  result_type
  operator()(SrcView&& src_vw1, DstView&& dst_vw,
             OffsetRef&& offset, Ranges&& ranges) const
  {
    size_t write_offset = offset;
    auto src_vw         = src_vw1.container().view();

    using domain_type = typename decltype(src_vw)::domain_type;

    src_vw.set_domain(
      domain_type(src_vw1.domain().first(), src_vw1.domain().last() + 1));

    for (size_t idx = 0; idx < ranges.second.size(); ++idx)
    {
      auto range = ranges.second[idx];

      algo_details::set_elements(src_vw, *dst_vw.begin(),
                                 range.first, range.second, write_offset);

      write_offset += range.second - range.first + 1;
    }
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function invokes STL @p unique with given predicate.
//////////////////////////////////////////////////////////////////////
template<typename Predicate>
class serial_unique
{
private:
  Predicate m_predicate;

public:
  serial_unique(Predicate predicate)
    : m_predicate(std::move(predicate))
  { }

  template<typename View>
  typename std::decay<View>::type::iterator
  operator()(View&& vw) const
  { return std::unique(vw.begin(), vw.end(), m_predicate); }

  void define_type(typer& t)
  { t.member(m_predicate); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function invokes sequential (ie STL) @ref unique_copy.
//////////////////////////////////////////////////////////////////////
struct serial_unique_copy
{
  using result_type = size_t;

  template<typename SrcView, typename DstView>
  result_type operator()(SrcView&& src_vw, DstView&& dst_vw) const
  {
    auto iter = std::unique_copy(src_vw.begin(), src_vw.end(), dst_vw.begin());

    return std::distance(dst_vw.begin(), iter);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Copies all of the elements from the source to the destination view,
///   except those that are consecutive duplicates (equal to the preceding
///   element), which are moved to the end of the destination view.
/// @param src_view One-dimensional view of the input elements.
/// @param dest_view One-dimensional view of the output container.
/// @param bin_predicate Binary functor which implements equality.
/// @return std::pair containing a view over the unique elements, and a view
///   over the remaining elements of @p dest_view.
/// @ingroup removingAlgorithms
///
/// This algorithm is non-mutating to the input, and requires a view with
/// iterator support. Supplied predicate should be commutative. The
/// destination container will be updated.
///
/// @todo Support (or at least guard) in production mode when there
///   are multiple ranges.
///
/// @todo The range-based kernel assumes that the domain of the input view
///   is contiguous. Investigate the behavior with enumerated domains like
///   @ref domset1D.
//////////////////////////////////////////////////////////////////////
template <typename View, typename DestView, typename BinPredicate>
std::pair<DestView, DestView>
unique_copy(View& src_view, DestView& dest_view, BinPredicate bin_predicate)
{
  stapl_assert(src_view.domain().is_contiguous(), "unique_copy for views"
    " with non-contiguous domains is currently not supported");

  using namespace skeletons;

  size_t num_unique = 0;

#ifdef STAPL_PRODUCTION
  if (get_num_locations() == 1)
  {
    num_unique =
      map_reduce<skeletons::tags::with_coarsened_wf>(
        serial_unique_copy(), stapl::plus<size_t>(), src_view, dest_view);
  }
  else
#endif
  {
    auto vw1      = make_overlap_view(src_view, 1, 0, 1);
    auto vw2      = make_repeat_view(dest_view);
    auto views    = default_coarsener()(make_tuple(vw1, vw2));

    DECLARE_INLINE_INPUT_PLACEHOLDERS(2, in)
    DECLARE_INLINE_PLACEHOLDERS(5, x)

    num_unique =
      skeletons::execute(
        skeletons::execution_params<size_t>(null_coarsener()),
        compose<tags::inline_flow>(
            x0 << zip<1>(unique_compute_offsets()) | (in0),
            x1 << zip<1>(unique_take_first())      | (x0),
            x2 << scan(stapl::plus<size_t>(), 0)   | (x1),
            x3 << zip<4>(unique_update())          | (in0, in1, x2, x0),
            x4 << reduce_to_locs<true>(
                    stapl::plus<size_t>())         | (x1)),
        get<0>(views), get<1>(views));
  }

  using src_view_t      =  View;
  using src_dom_t       =  typename src_view_t::domain_type;
  using dest_view_t     =  DestView;
  using dest_dom_t      =  typename dest_view_t::domain_type;
  using dom_t           =  dest_dom_t;

  DestView v1(
    dest_view.container(),
    dom_t(
      dest_view.domain().first(),
      dest_view.domain().advance(
        dest_view.domain().first(), num_unique - 1),
      dest_view.domain()),
    dest_view.mapfunc());

  DestView v2(
    dest_view.container(),
    dom_t(
      dest_view.domain().advance(
        dest_view.domain().first(), num_unique),
      dest_view.domain().last(),
      dest_view.domain()),
    dest_view.mapfunc());

  return std::make_pair(v1, v2);
}


//////////////////////////////////////////////////////////////////////
/// @brief Copies all of the elements from the source to the destination view,
///   except those that are consecutive duplicates (equal to the preceding
///   element), which are moved to the end of the destination view.
/// @param src_view One-dimensional view of the input elements.
/// @param dest_view One-dimensional view of the output container.
/// @return std::pair containing a view over the unique elements, and a view
///   over the remaining elements.
/// @ingroup removingAlgorithms
///
/// This version calls the predicated version with a default predicate of
/// stapl::equal_to.
//////////////////////////////////////////////////////////////////////
template <typename View0, typename View1>
std::pair<View1, View1>
unique_copy(View0& src_view, View1& dest_view)
{
  return unique_copy(src_view, dest_view,
          stapl::equal_to<typename std::decay<View0>::type::value_type>());
}


//////////////////////////////////////////////////////////////////////
/// @brief Remove all duplicate elements from the given view.
/// @param view One-dimensional view of the input elements.
/// @param predicate Binary functor which implements the equal operation.
/// @return std::pair containing a view over the unique elements, and a view
///   over the remaining elements.
/// @ingroup removingAlgorithms
///
/// This algorithm is mutating, so the input view must be Read-Write and support
/// iterators. The underlying container is also modified.
//////////////////////////////////////////////////////////////////////
template <typename View, typename Predicate>
std::pair<View, View>
unique(View& view, Predicate predicate)
{
  const size_t num_unique = algo_details::partition_impl(
    view, serial_unique<Predicate>(std::move(predicate)), true, false);

  return std::make_pair(
    View(
      view.container(),
      typename View::domain_type(
        view.domain().first(),
        view.domain().advance(
          view.domain().first(), num_unique - 1),
        view.domain()),
      view.mapfunc()),
    View(
      view.container(),
      typename View::domain_type(
        view.domain().advance(
          view.domain().first(), num_unique),
        view.domain().last(),
        view.domain()),
      view.mapfunc()));
}


//////////////////////////////////////////////////////////////////////
/// @brief Remove all duplicate elements from the given view.
/// @param view One-dimensional view of the input elements.
/// @return std::pair containing a view over the unique elements, and a view
///   over the remaining elements.
/// @ingroup removingAlgorithms
///
/// This version calls the predicated version with a default predicate of
/// stapl::equal_to.
//////////////////////////////////////////////////////////////////////
template <typename View>
std::pair<View, View>
unique(View& view)
{
  return unique(view,
    stapl::equal_to<typename std::decay<View>::type::value_type>());
}

} //namespace stapl

#endif

