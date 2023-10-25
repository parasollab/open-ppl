/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_ALGORITHMS_SORTING_HPP
#define STAPL_ALGORITHMS_SORTING_HPP

#include <vector>
#include <set>
#include <algorithm>
#include <functional>
#include <utility>
#include <numeric>
#include <sstream>

#include "functional.hpp"
#include <stapl/runtime.hpp>

#include <stapl/paragraph/paragraph.hpp>
#include <stapl/utility/do_once.hpp>
#include <stapl/utility/tuple.hpp>

#include <stapl/containers/array/array.hpp>
#include <stapl/containers/array/static_array.hpp>

#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/numeric.hpp>
#include <stapl/algorithms/generator.hpp>
#include <stapl/algorithms/functional.hpp>

#include <stapl/views/strided_view.hpp>
#include <stapl/views/segmented_view.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/views/metadata/coarseners/null.hpp>
#include <stapl/views/metadata/coarseners/default.hpp>
#include <stapl/views/metadata/coarseners/all_but_last.hpp>

#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/utility/lightweight_vector.hpp>
#include <stapl/skeletons/executors/execute.hpp>
#include <stapl/skeletons/functional/butterfly.hpp>
#include <stapl/skeletons/functional/reduce_to_pow_two.hpp>
#include <stapl/skeletons/functional/expand_from_pow_two.hpp>
#include <stapl/skeletons/functional/sink.hpp>
#include <stapl/skeletons/spans/balanced.hpp>
#include <stapl/skeletons/spans/nearest_pow_two.hpp>

#include <stapl/containers/partitions/splitter.hpp>
#include <stapl/algorithms/column_sort.hpp>

#ifdef USE_NEW_NOTATION
#include <stapl/skeletons/operators/define_dag.hpp>
#endif


namespace stapl {

// @todo find a better spot or method for these (tags?)
enum {EVEN, SEMIRANDOM, RANDOM, BLOCK};


// forward declarations
template<typename View, typename T>
typename View::size_type count(View const&, T const&);


template<typename View, typename Compare>
typename View::value_type max_value(View const&, Compare);


template<typename View0, typename View1>
void copy(View0 const&, View1 const&);


template<typename InputView, typename SplittersView, typename Compare,
         typename Functor>
segmented_view<InputView,
                 splitter_partition<typename InputView::domain_type > >
n_partition(InputView&, SplittersView const&, Compare const&, Functor const&);


template<typename View, typename Generator>
void generate(View const&, Generator);


namespace algo_details {

//////////////////////////////////////////////////////////////////////
/// @brief Compare wrapper used by stable_sort when calling n_partition
///        to wrap the defined comparator and use a std::pair to preserve
///        the initial position of the elements.
/// @tparam Compare the defined comparator for sorting.
//////////////////////////////////////////////////////////////////////
template <typename Compare>
struct comparator_wrapper
{
private:
  Compare m_comp;

public:
  using first_argument_type  =
    std::pair<typename Compare::first_argument_type, unsigned long>;
  using second_argument_type =
    std::pair<typename Compare::second_argument_type, unsigned long>;

  comparator_wrapper(Compare comp)
    : m_comp(comp)
  { }

  template <typename LRef, typename RRef>
  bool operator()(LRef&& lhs, RRef&& rhs)
  {
    return m_comp(lhs.first, rhs.first) == m_comp(rhs.first, lhs.first)
              ? lhs.second < rhs.second
              : m_comp(lhs.first, rhs.first);
  }

  Compare const& base_comparator(void) const
  {
    return m_comp;
  }

  void define_type(typer& t)
  {
    t.member(m_comp);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function that sorts elements, used by sample_sort() to sort each
///   partition of elements.
/// @tparam IsStable Indicates whether the sort to perform should be stable
/// @tparam Compare Comparator used for sorting.
//////////////////////////////////////////////////////////////////////
template <bool IsStable, typename Comparator>
class serial_sort_wf
{
private:
  Comparator m_comparator;

public:
  using result_type = void;

  explicit
  serial_sort_wf(Comparator comparator)
    : m_comparator(std::move(comparator))
  { }

  template<typename View0>
  void operator()(View0&& view0) const
  {
    std::sort(view0.begin(), view0.end(), m_comparator);
  }

  void define_type(typer& t)
  {
    t.member(m_comparator);
  }
}; //class serial_sort_wf


//////////////////////////////////////////////////////////////////////
/// @brief Specialization that handles the case where a stable sort is
/// required, invoking std::stable_sort instead of std::sort.
/// @tparam Compare Comparator used for sorting.
//////////////////////////////////////////////////////////////////////
template <typename Comparator>
class serial_sort_wf<true, Comparator>
{
private:
  Comparator m_comparator;

public:
  using result_type = void;

  explicit
  serial_sort_wf(Comparator comparator)
    : m_comparator(std::move(comparator))
  { }

  template<typename View0>
  void operator()(View0&& view0) const
  {
    std::stable_sort(view0.begin(), view0.end(), m_comparator);
  }

  void define_type(typer& t)
  {
    t.member(m_comparator);
  }
}; //class serial_sort_wf


//////////////////////////////////////////////////////////////////////
/// @brief Work function used to collect a set of samples from an input view.
///   Sampling methods : 0:even, 1: semi-random, 2: random, 3: block
//////////////////////////////////////////////////////////////////////
template <typename T>
class sample_wf
{
  size_t m_over_sampling_ratio;
  int m_num_components;
  int m_sampling_method;

public:
  typedef lightweight_vector<T> result_type;

  sample_wf(int over_sampling, int num_comp, int sm)
    : m_over_sampling_ratio(over_sampling),
      m_num_components(num_comp),
      m_sampling_method(sm)
  {}

  template<typename View0>
  result_type operator()(View0&& data) const
  {
    if (!data.size())
      abort("sample_wf invoked on empty subview");

    size_t num_samples = data.size() > m_over_sampling_ratio ?
      m_over_sampling_ratio : data.size();

    result_type result;
    result.reserve(num_samples);

    typedef typename std::decay<View0>::type::iterator data_iter;

    data_iter data_it  = data.begin();

    size_t data_size = data.size();

    //Option 0: Evenly Spaced
    if (m_sampling_method == EVEN) {
      size_t step = (data_size-1) / num_samples;

      if (step == 0)
        step = 1;

      for (size_t i = 0; i != num_samples; ++i, data_it += step)
        result.push_back(*data_it);
      return result;
    }

    abort("sampling method selected not tested. Use evenly spaced.");
#if 0
// These sampling methods will be uncommented and used as they are unit tested.

    //Option 1: Semi-random step, semi-evenly spaced
    if (m_sampling_method == SEMIRANDOM) {
      typedef std::uniform_int_distribution<int> rng_dist_t;
      std::random_device rd;
      std::mt19937 gen(rd());

      data_iter data_end  = data.end();

      int step = (data_size-1)/num_samples;
      int rand_step = step;
      if (step == 0) {
        step = 1;
        rand_step = 1;
      }
      for (size_t i = 0; i != num_samples && data_it != data_end;
           ++i, data_it += step) {
        result.push_back(*data_it);

        //Keep the step from going out of bounds
        step = rng_dist_t(1, rand_step - 1)(gen);
      }
      return result;
    }

    if (m_sampling_method == RANDOM) {
      //Option 2: random, not evenly spaced
      //set used to prevent duplicate indexes from being chosen
      typedef std::set<data_iter>             index_set;
      typedef typename index_set::iterator    iter;
      typedef std::uniform_int_distribution<int> rng_dist_t;
      std::random_device rd;
      std::mt19937 gen(rd());

      size_t step;

      index_set indexes;
      std::pair<iter, bool>  already_there;

      data_iter data_beg  = data.begin();

      for (size_t i = 0; i != num_samples && indexes.size() != data_size; ++i)
      {
        do {
          step = rng_dist_t(0, data_size - 1)(gen);
          data_it = data_beg;
          data_it += step;
          already_there = indexes.insert(data_it);
        } while (!already_there.second);
        result.push_back(*data_it);
      }
      return result;
    }

    if (m_sampling_method == BLOCK) {
      //Option 3: first block of size over_sampling_ratio
      //data elements
      for (size_t i = 0; i != num_samples; ++i, ++data_it)
        result.push_back(*data_it);
      return result;
    }
#endif
    return result;
  }

  void define_type(typer& t)
  {
    t.member(m_over_sampling_ratio);
    t.member(m_num_components);
    t.member(m_sampling_method);
  }

}; //class sample_wf


//////////////////////////////////////////////////////////////////////
/// @brief Work function which does nothing, used by n_partition() to bypass
///   processing of the partitions.
//////////////////////////////////////////////////////////////////////
struct neutral_functor
{
  using result_type = void;

  template<typename Bucket, typename Offset>
  result_type operator()(Bucket&&, Offset&&)
  { }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function which processes sample_sort() partitions.
/// @tparam Compare Comparator used for sorting.
//////////////////////////////////////////////////////////////////////
template<typename Compare>
class sample_sort_pf
{
private:
  Compare m_comp;

public:
  using result_type =
    lightweight_vector<typename Compare::first_argument_type>;

  sample_sort_pf(Compare c)
    : m_comp(std::move(c))
  { }

  template<typename Bucket>
  result_type operator()(Bucket&& b)
  {
    lightweight_vector<typename Compare::first_argument_type> res(*b.begin());
    std::sort(res.begin(), res.end(), m_comp);
    return res;
  }

  void define_type(typer& t)
  {
    t.member(m_comp);
  }
}; // sample_sort_pf


//////////////////////////////////////////////////////////////////////
/// @brief Work function which processes nth_element() partitions.
/// @tparam Compare Comparator used for sorting.
//////////////////////////////////////////////////////////////////////
template<typename Compare>
struct nth_element_pf
{
  using result_type = void;

  Compare m_comp;
  const std::size_t m_nth;

  nth_element_pf(Compare c, const std::size_t o = 0)
    : m_comp(c), m_nth(o)
  {}

  template<typename Bucket, typename Offset>
  result_type operator()(Bucket&& b, Offset&& o)
  {
    lightweight_vector<typename Compare::first_argument_type> tmp(b);
    if (o + b.size() > m_nth && m_nth >= o)
      std::nth_element(tmp.begin(), tmp.begin() + (m_nth - o),
                       tmp.end(), m_comp);
    b = tmp;
  }

  void define_type(typer& t)
  {
    t.member(m_comp);
    t.member(m_nth);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function which processes partial_sort() partitions.
/// @tparam Compare Comparator used for sorting.
//////////////////////////////////////////////////////////////////////
template<typename Compare>
struct partial_sort_pf
{
  using result_type = void;

  Compare m_comp;
  const std::size_t m_nth;

  partial_sort_pf(Compare c, const std::size_t o = 0)
    : m_comp(c), m_nth(o)
  {}

  template<typename Bucket, typename Offset>
  result_type operator()(Bucket&& b, Offset&& o)
  {
    if (o <= m_nth)
    {
      lightweight_vector<typename Compare::first_argument_type> tmp(b);
      std::sort(tmp.begin(), tmp.end(), m_comp);
      b = tmp;
    }
  }

  void define_type(typer& t)
  {
    t.member(m_comp);
    t.member(m_nth);
  }
};

template<typename T>
struct reduce_vector_wf
{
  typedef lightweight_vector<T> result_type;

  template<typename Element1, typename Element2>
  result_type operator()(Element1&& elt1, Element2&& elt2)
  {
    result_type result(std::move((lightweight_vector<T>)elt1));
    result.reserve(result.size() + elt2.size());
    std::copy(elt2.cbegin(), elt2.cend(), std::back_inserter(result));
    return result;
  }
};

template<typename Comparator>
struct extract_base_comparator
{
  using type = Comparator;

  using element = typename Comparator::first_argument_type;

  static
  Comparator const& apply(Comparator const& comp)
  { return comp; }
};

template<typename Comparator>
struct extract_base_comparator<comparator_wrapper<Comparator>>
{
  using type = Comparator;

  using element =
    std::pair<typename Comparator::first_argument_type, unsigned long>;

  static
  Comparator const& apply(comparator_wrapper<Comparator> const& comp)
  { return comp.base_comparator(); }
};

//////////////////////////////////////////////////////////////////////
/// @brief Implementation of the partial_sort() algorithm.
/// @param data              Reference to the input view to be sorted.
/// @param comp              Comparator used for sorting.
/// @param sample_method     Enumerated integer for selecting the sample method.
/// @param over_partitioning Number of buckets per location.
/// @param over_sampling     Number of samples taken per bucket.
/// @todo Replace the sequential sort of the samples call with another
/// sorting algorithm (p_merge_sort?) once one is implemented.
/// @note a sequential vector is used to collect the samples, resulting in
/// its replication across locations.
/// @todo for num_buckets == 1 case, ensure data is clustered into only
/// one group when underlying container has multiple base containers.
//////////////////////////////////////////////////////////////////////
template <typename View, typename Compare>
void sample_sort_impl(View& data, Compare comp,
                      size_t sample_method = EVEN,
                      size_t over_partitioning = 1,
                      size_t over_sampling = 128)
{
  stapl_assert(over_partitioning == 1,
               "sample sorting over_partitioning > 1 has not been tested\n");

  using value_t = typename View::value_type;

  if (over_sampling < over_partitioning)
    over_sampling = over_partitioning;

  const std::size_t num_buckets = data.get_num_locations();

  using extractor = extract_base_comparator<Compare>;
  typename extractor::type const& real_comp = extractor::apply(comp);

  if (num_buckets == 1 || data.size() < num_buckets)
  {
    map_func<skeletons::tags::with_coarsened_wf>(
      serial_sort_wf<!std::is_same<Compare, typename extractor::type>::value,
                     typename extractor::type>(real_comp), data);

    return;
  }

  const size_t num_op_buckets   = num_buckets * over_partitioning;

  //Allocate Sample Container (Fill with Maximum Value of Data) and
  //Instantiate Sample View

  //Collect Samples
  sample_wf<value_t> sampler(over_sampling, num_buckets, sample_method);

  std::vector<value_t>
  samples_vect(*map_reduce<skeletons::tags::with_coarsened_wf>(sampler,
                   reduce_vector_wf<value_t>(), data).get());

  //Sort Samples
  std::sort(samples_vect.begin(), samples_vect.end(), real_comp);

  size_t num_real_samples = samples_vect.size();

  //  Calculate Step Size
  size_t splitter_size = num_op_buckets - 1;
  size_t step = num_real_samples / num_op_buckets;

  // Copy every stepth element into splitters, starting from the middle of the
  // range for each step to avoid picking the smallest sample.
  std::vector<value_t> splitters;
  splitters.reserve(splitter_size);
  for (size_t index = step-1;
       index < num_real_samples && splitters.size() != splitter_size;
       index += step)
  {
    splitters.push_back(samples_vect[index]);
  }

  // Create partitions and sort each partition regarding to the given functor
  sample_sort_pf<Compare> pf(comp);

  partition_and_sort(data, splitters, comp, pf);
}


//////////////////////////////////////////////////////////////////////
/// @brief Copy the input data into the output container, starting at
///   the given position
//////////////////////////////////////////////////////////////////////
struct copy_back_wf
{
  typedef void result_type;

  template<typename InputRef, typename OffsetRef, typename OutputView>
  result_type operator()(InputRef&& in, OffsetRef&& pos, OutputView&& out)
  {
    std::size_t i = pos;
    for (auto&& element : in)
    {
      out.set_element(i, element);
      ++i;
    }
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Copy split input back into the output container used by n_partition.
///        It differs from copy_back_wf in that this functor works with
///        the data that has the partition id added to each element.
//////////////////////////////////////////////////////////////////////
template<typename Comparator>
struct copy_to_input_wf
{
  using result_type = void;

  template<typename InputRef, typename OffsetRef, typename OutputView>
  void operator()(InputRef&& in, OffsetRef&& pos, OutputView&& out)
  {
    // Functor to invoke bulk copy if it is available
    typename std::decay<OutputView>::type::domain_type
      dest_dom(pos, pos + in.size()-1);
    typename std::decay<OutputView>::type dest_view(out.container(), dest_dom);

    coarse_assign assign_wf;
    assign_wf(std::move(
      (lightweight_vector<typename Comparator::first_argument_type>)in),
      dest_view);
  }
};


// Specialization used for stable_sort implementation
template<typename Comparator>
struct copy_to_input_wf<comparator_wrapper<Comparator>>
{
  using result_type = void;

  template<typename InputRef, typename OffsetRef, typename OutputView>
  void operator()(InputRef&& in, OffsetRef&& pos, OutputView&& out)
  {
    // The first argument to the comparator is a pair<T, index>
    using first_argument_type =
      typename comparator_wrapper<Comparator>::first_argument_type;

    // Copy vector to remove calls to proxy
    lightweight_vector<first_argument_type> tmp(in);

    // Temporary container to hold elements as they're moved out of the pair
    lightweight_vector<typename first_argument_type::first_type> tmp_out;
    tmp_out.reserve(tmp.size());

    for (auto&& element : tmp)
      tmp_out.emplace_back(std::move(element.first));

    // Functor to invoke bulk copy if it is available
    typename std::decay<OutputView>::type::domain_type
      dest_dom(pos, pos + in.size()-1);
    typename std::decay<OutputView>::type dest_view(out.container(), dest_dom);

    coarse_assign assign_wf;
    assign_wf(std::move(tmp_out), dest_view);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Compute buckets sizes after splitting data.
//////////////////////////////////////////////////////////////////////
struct buckets_sizes_wf
{
  typedef void result_type;

  template<typename InputRef, typename OutputRef>
  result_type operator()(InputRef&& in, OutputRef&& out)
  {
    out = in.size();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Compute buckets sizes after splitting data and returns the
/// value.
///
/// Functor is used in the skeleton composition in @ref partition_and_sort.
//////////////////////////////////////////////////////////////////////
struct return_buckets_sizes_wf
{
  typedef std::size_t result_type;

  template<typename InputRef>
  result_type operator()(InputRef&& in)
  {
    return (*in.begin()).size();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Wrap a type in a std::vector<>.
/// @tparam T Value type to be wrapped.
//////////////////////////////////////////////////////////////////////
template<typename T>
struct to_vector_wf
{
  typedef std::vector<T> result_type;

  template<typename Ref>
  result_type operator()(Ref&& in)
  {
    return result_type(1, in);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Concatenate two vectors.
/// @tparam T Value type of the containers.
//////////////////////////////////////////////////////////////////////
template<typename T>
struct concat_vector_wf
{
  typedef std::vector<T> result_type;

  template<typename Ref1, typename Ref2>
  result_type operator()(Ref1&& lhs, Ref2&& rhs)
  {
    result_type dest(lhs);
    dest.reserve(dest.size() + rhs.size());
    std::copy(rhs.begin(), rhs.end(), std::back_inserter(dest));
    return dest;
  }
};

#if 0
// fill local buckets work functions will be used when butterfly is reintroduced
// for partitioning elements.

//////////////////////////////////////////////////////////////////////
/// @brief Fill local buckets with input data, used by n_partition().
/// @tparam Compare       Comparator used for sorting.
/// @tparam SplittersView Vector of splitters used to separate data.
//////////////////////////////////////////////////////////////////////
template<typename Compare, typename SplittersView>
struct fill_local_buckets
{
  using result_type = lightweight_vector<lightweight_vector<
                        typename Compare::first_argument_type>>;

private:
  Compare m_comp;
  SplittersView const* const m_splitters;

public:
  fill_local_buckets(Compare c, SplittersView const& s)
    : m_comp(c), m_splitters(&s)
  {}

  template<typename DataView>
  result_type operator()(DataView&& view)
  {
    using buckets_type =
      lightweight_vector<lightweight_vector<
        typename std::decay<DataView>::type::value_type>>;
    buckets_type tmp(m_splitters->size() + 1);

    auto num_elems = view.size() / tmp.size();
    for (auto&& bucket : tmp)
      bucket.reserve(num_elems);

    auto splitters_begin = m_splitters->begin();
    auto splitters_end = m_splitters->end();
    for (auto&& element : view)
    {
      // element is copied here to avoid multiple calls to proxy::read in
      // lower_bound.
      typename SplittersView::value_type val(element);

      auto spl_it =
        std::lower_bound(splitters_begin, splitters_end, val, m_comp);

      std::size_t index = std::distance(splitters_begin, spl_it);
      tmp[index].emplace_back(element);
    }

    return tmp;
  }

  void define_type(typer& t)
  {
    t.member(m_comp);
    t.member(m_splitters);
  }
};


template<typename Compare, typename SplittersView>
struct fill_local_buckets<comparator_wrapper<Compare>, SplittersView>
{
  using result_type =
    lightweight_vector<lightweight_vector<
      std::pair<typename Compare::first_argument_type, unsigned long>>>;

private:
  using extractor = extract_base_comparator<comparator_wrapper<Compare>>;

  typename extractor::type m_comp;

  SplittersView const* const m_splitters;

public:
  fill_local_buckets(comparator_wrapper<Compare> c, SplittersView const& s)
    : m_comp(extractor::apply(c)), m_splitters(&s)
  {}

  template<typename DataView>
  result_type operator()(DataView&& view)
  {
    using buckets_type =
      lightweight_vector<lightweight_vector<
        std::pair<typename DataView::value_type, unsigned long>>>;

    buckets_type tmp(m_splitters->size() + 1);

    auto num_elems = view.size() / tmp.size();
    for (auto&& bucket : tmp)
      bucket.reserve(num_elems);

    unsigned long item_idx = 0;
    unsigned long offset = stapl::index_of(*view.begin());

    using extractor = extract_base_comparator<Compare>;
    typename extractor::type const& real_comp = extractor::apply(m_comp);

    auto splitters_begin = m_splitters->begin();
    auto splitters_end = m_splitters->end();
    for (auto&& element : view)
    {
      // element is copied here to avoid multiple calls to proxy::read in
      // lower_bound.
      typename SplittersView::value_type val(element);

      auto spl_it =
        std::lower_bound(splitters_begin, splitters_end, val, real_comp);

      std::size_t index = std::distance(splitters_begin, spl_it);
      tmp[index].emplace_back(element, (offset + item_idx++));
    }

    return tmp;
  }

  void define_type(typer& t)
  {
    t.member(m_comp);
    t.member(m_splitters);
  }
};
#endif


//////////////////////////////////////////////////////////////////////
/// @brief Copies the elements provided at construction to
/// the end of a bucket of elements.
///
/// Used as the functor passed to the @ref array apply_set function
/// when it is called from fill_and_merge_buckets to write a local
/// partition to the location responsible for processing that partition
/// once the contribution from all other locations has been received.
//////////////////////////////////////////////////////////////////////
template<typename Bucket>
struct append_bucket
{
private:
  /// Local partition being moved into container of aggregated partitions
  Bucket m_bucket;

  /// @brief Approximation of the total number of elements based on the size of
  /// the local partition.
  std::size_t m_size;

public:
  append_bucket(Bucket const& bucket, std::size_t size)
    : m_bucket(bucket), m_size(size)
  { }

  template<typename Reference>
  void operator()(Reference&& r) const
  {
    if (r.size() == 0) {
      r = m_bucket;
      r.reserve(m_size);
    }
    else
      std::move(m_bucket.begin(), m_bucket.end(), std::back_inserter(r));
  }

  void define_type(typer& t)
  {
    t.member(m_bucket);
    t.member(m_size);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Partitions input data based on the splitters provided and
/// writes each partition to a container of composed buckets to effectively
/// merge the local partition for each splitter into a single partition of
/// elements.
///
/// Used in @ref sample_sort_impl to process the local data and move it
/// to the locations that will be responsible for processing the partitions
/// in which the local data belongs.
//////////////////////////////////////////////////////////////////////
template<typename Compare, typename SplittersView>
struct fill_and_merge_buckets
{
  using result_type = void;

private:
  Compare m_comp;
  SplittersView const* const m_splitters;

public:
  fill_and_merge_buckets(Compare c, SplittersView const& s)
    : m_comp(c), m_splitters(&s)
  {}

  template<typename DataView, typename BucketsView>
  result_type operator()(DataView&& view, BucketsView&& buckets)
  {
    using bucket_type =
      lightweight_vector<typename std::decay<DataView>::type::value_type>;

    // set of local partitions
    std::vector<bucket_type> tmp(m_splitters->size() + 1);

    // reservation is made to reduce the number of calls to allocator as
    // elements are added to the bucket
    auto num_elems = view.size() / (m_splitters->size() + 1);
    for (auto&& bucket : tmp)
      bucket.reserve(num_elems*2);

    // Place elements in a local partition
    auto splitters_begin = m_splitters->begin();
    auto splitters_end = m_splitters->end();
    for (auto&& element : view)
    {
      typename SplittersView::value_type val(element);
      auto spl_it =
        std::lower_bound(splitters_begin, splitters_end, val, m_comp);
      std::size_t index = std::distance(splitters_begin, spl_it);
      tmp[index].emplace_back(std::move(val));
    }

    // Send local partitions to the container element where they are aggregated.
    std::size_t nlocs = (*buckets.begin()).container().get_num_locations();
    std::size_t nbuckets = m_splitters->size() + 1;
    std::size_t bucket_id = (*buckets.begin()).container().get_location_id();
    for (size_t cnt = 0; cnt != nbuckets; ++cnt)
    {
      // each location begins writing in a different element to avoid hot spots
      bucket_id = bucket_id < nbuckets-1 ? bucket_id + 1 : 0;
      if (tmp[bucket_id].size())
        (*buckets.begin()).apply_set(bucket_id,
          append_bucket<bucket_type>(tmp[bucket_id],
                                     (nlocs-1)*tmp[bucket_id].size()));
    }
  }

  void define_type(typer& t)
  {
    t.member(m_comp);
    t.member(m_splitters);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Partitions input data based on the splitters provided and
/// writes each partition to a container of composed buckets to effectively
/// merge the local partition for each splitter into a single partition of
/// elements.
///
/// Used in @ref stable_sort to process the local data and move it
/// to the locations that will be responsible for processing the partitions
/// in which the local data belongs. When elements are placed in the local
/// partition their index is added.  This allows the relative initial order
/// of equal elements to be maintained.
//////////////////////////////////////////////////////////////////////
template<typename Compare, typename SplittersView>
struct fill_and_merge_buckets<comparator_wrapper<Compare>, SplittersView>
{
  using result_type = void;

private:
  using extractor = extract_base_comparator<comparator_wrapper<Compare>>;
  typename extractor::type m_comp;
  SplittersView const* const  m_splitters;

public:
  fill_and_merge_buckets(comparator_wrapper<Compare> c, SplittersView const& s)
    : m_comp(extractor::apply(c)), m_splitters(&s)
  {}

  template<typename DataView, typename BucketsView>
  result_type operator()(DataView&& view, BucketsView&& buckets)
  {
    using value_type = typename std::decay<DataView>::type::value_type;
    using bucket_type =
      lightweight_vector<std::pair<value_type, unsigned long>>;
    std::vector<bucket_type> tmp(m_splitters->size() + 1);

    unsigned long item_idx = 0;
    unsigned long offset = stapl::index_of(*view.begin());

    auto num_elems = view.size() / (m_splitters->size() + 1);
    for (auto&& bucket : tmp)
      bucket.reserve(num_elems*2);

    auto splitters_begin = m_splitters->begin();
    auto splitters_end = m_splitters->end();
    for (auto&& element : view)
    {
      typename SplittersView::value_type val(element);
      auto spl_it =
        std::lower_bound(splitters_begin, splitters_end, val, m_comp);
      std::size_t index = std::distance(splitters_begin, spl_it);
      tmp[index].emplace_back(
        std::make_pair(std::move(val), (offset + item_idx++)));
    }

    // Send local partitions to the container element where they are aggregated.
    std::size_t nlocs = (*buckets.begin()).container().get_num_locations();
    std::size_t nbuckets = m_splitters->size() + 1;
    std::size_t bucket_id = (*buckets.begin()).container().get_location_id();
    for (size_t cnt = 0; cnt != nbuckets; ++cnt)
    {
      // each location begins writing in a different element to avoid hot spots
      bucket_id = bucket_id < nbuckets-1 ? bucket_id + 1 : 0;
      if (tmp[bucket_id].size())
        (*buckets.begin()).apply_set(bucket_id,
          append_bucket<bucket_type>(tmp[bucket_id],
                                     (nlocs-1)*tmp[bucket_id].size()));
    }
  }

  void define_type(typer& t)
  {
    t.member(m_comp);
    t.member(m_splitters);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Reduce two buckets in one bucket, used by n_partition().
/// @tparam BucketElement Element stored in bucket representing a single
///   input element.
///
/// For unstable sort the bucket element is the value_type of the input
/// view.  Stable sort uses a pair<value_type, unsigned long> to maintain
/// relative ordering of equivalent input values.
///
/// @todo Making a copy - el2_copy - to avoid crashes due to the member_iterator
///   assuming the data it refers to is local.
//////////////////////////////////////////////////////////////////////
template<typename BucketElement>
struct bucket_reduce_wf
{
  using result_type = lightweight_vector<lightweight_vector<BucketElement> >;

  //////////////////////////////////////////////////////////////////////
  /// @todo Making a copy - el2_copy - to avoid crashes due to the
  ///   member_iterator assuming the data it refers to is local.
  //////////////////////////////////////////////////////////////////////
  template <typename Element1, typename Element2>
  result_type operator()(Element1 const& el1, Element2 const& el2) const
  {
    result_type result(el1);
    result_type el2_copy(el2);
    std::size_t i = 0;
    for (typename result_type::const_iterator el2_it  = el2_copy.begin();
                                              el2_it != el2_copy.end();
                                              ++el2_it, ++i)
      std::copy((*el2_it).begin(), (*el2_it).end(),
                std::back_inserter(result[i]));

    return result;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Split one bucket in two buckets, used by n_partition().
/// @tparam BucketElement Element stored in bucket representing a single
///   input element.
///
/// For unstable sort the bucket element is the value_type of the input
/// view.  Stable sort uses a pair<value_type, unsigned long> to maintain
/// relative ordering of equivalent input values.
//////////////////////////////////////////////////////////////////////
template<typename BucketElement>
struct split_bucket_wf
{
  using result_type = lightweight_vector<lightweight_vector<BucketElement> >;

private:
  bool m_i;

public:
  split_bucket_wf(void)
    : m_i(false)
  { }

  void set_position(std::size_t index, bool)
  {
    m_i = (index % 2 != 0);
  }

  template<typename Element>
  result_type operator()(Element const& el) const
  {
    result_type result((!m_i) ? ceil(el.size() / 2.) : floor(el.size() / 2.));
    std::size_t sz = result.size();

    result_type el_copy(el);
    std::size_t start = (!m_i) ? 0 : ceil(el.size() / 2.);
    std::size_t i = 0;
    for (typename result_type::const_iterator it = el_copy.begin() + start;
                                     i != sz;
                                     ++it, ++i)
        std::move((*it).begin(), (*it).end(), std::back_inserter(result[i]));

    return result;
  }

  void define_type(typer& t)
  {
    t.member(m_i);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function given to the butterfly skeleton to merge the
/// partitioned data from different locations, used by n_partition().
/// @tparam BucketElement Element stored in bucket representing a single
///   input element.
///
/// For unstable sort the bucket element is the value_type of the input
/// view.  Stable sort uses a pair<value_type, unsigned long> to maintain
/// relative ordering of equivalent input values.
//////////////////////////////////////////////////////////////////////
template<typename BucketElement>
struct bucket_merge_wf
{
  using result_type = lightweight_vector<lightweight_vector<BucketElement> >;

private:
  std::size_t m_butterfly_size;
  std::size_t m_index1;
  std::size_t m_index2;
  std::size_t m_nb_pairs;
  std::size_t m_nb_locs;

public:
  bucket_merge_wf(int const& locs)
    : m_butterfly_size(0), m_index1(0), m_index2(0), m_nb_locs(locs)
  {
    m_nb_pairs = locs - pow(2, floor(log(locs) / log(2)));
  }

  void set_position(std::size_t butterfly_size, std::size_t index1,
                    std::size_t index2, std::size_t /* ignored */)
  {
    m_butterfly_size = butterfly_size;
    m_index1 = index1;
    m_index2 = index2;
  }

  template <typename Element1, typename Element2>
  result_type operator()(Element1&& el1, Element2&& el2) const
  {
    result_type result;

    // # of pairs = limit offset between pairs and identities
    std::size_t L = m_nb_pairs;
    // number of btf at the current level of the graph
    std::size_t nbBtfNodes = m_butterfly_size * 2;
    // btf number
    std::size_t currentBtfNb = std::floor(m_index1 / nbBtfNodes);
    // btf median's offset
    std::size_t M = m_butterfly_size + currentBtfNb * nbBtfNodes;
    // left bound of the current btf
    std::size_t leftBound = M - nbBtfNodes / 2;
    // number of elements (nodes) on the left side of the current btf
    std::size_t N1 = ( L <= M )
                      ? ( ( leftBound < L )
                        ? 2 * ( L - leftBound ) + M - L
                        : M - leftBound )
                      : 2 * ( M - leftBound );
    // ... and on the right side
    std::size_t N2 = ( L <= M )
                      ? M - leftBound
                      : ( L >= M + ( M - leftBound ) )
                        ? 2 * ( M - leftBound )
                        : 2 * ( L - M ) + ( M + ( M - leftBound ) - L );

    std::size_t R = el1.size() - ( N1 + N2 ) * floor(el1.size() / ( N1 + N2) );
    std::size_t lim = floor(el1.size() / (N1 + N2)) * N1 + ((R >= N1) ? N1 : R);
    std::size_t start = (m_index1 < M) ? 0 : lim;
    std::size_t end = (m_index1 < M) ? lim : el1.size();

    // the result size is the number of buckets elements
    result.resize(end - start);

    for (std::size_t elmt = start; elmt != end ; ++elmt) {
      // extract the left input buckets
      result[elmt - start] = el2[elmt];

      // extract the right input buckets
      lightweight_vector<BucketElement> el1e_copy(el1[elmt]);
      result[elmt - start].reserve(
        result[elmt - start].size() + el1[elmt].size());
      std::move(el1e_copy.begin(), el1e_copy.end(),
        std::back_inserter(result[elmt - start]));
    }
    return result;
  }

  void define_type(typer& t)
  {
    t.member(m_butterfly_size);
    t.member(m_index1);
    t.member(m_index2);
    t.member(m_nb_pairs);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Performs the core operations of a parallel sort.
/// @param input_v set of elements to be sorted
/// @param splitters_v set of splitters used to partition the input
/// @param comp Functor that implements the strict weak ordering comparison
///   required for sorting
/// @param partition_functor Functor that implements the operation to be applied
///   on each partition in order to effect the desired sort.
///
/// This function is called by multiple algorithms that vary the
/// partition_functor used.  For example, @ref sample_sort sorts each partition,
/// while @ref partial_sort sorts only the partitions that have indices lower
/// than the position beyond which sorting isn't required.
//////////////////////////////////////////////////////////////////////
template<typename InputView, typename SplittersView, typename Compare,
         typename Functor>
void
partition_and_sort(InputView& input_v, SplittersView const& splitters_v,
                   Compare const& comp, Functor const& partition_functor)
{
  using element_t = typename extract_base_comparator<Compare>::element;
  using split_input_t = static_array<lightweight_vector<element_t>>;
  using split_input_vt = array_view<split_input_t>;

  // 1) compute split input
  const std::size_t counted_num_buckets = splitters_v.size()+1;

  split_input_t split_input(counted_num_buckets > get_num_locations() ?
                            counted_num_buckets : get_num_locations());
  split_input_vt split_input_v(split_input);

  // not composed with phases below because the termination detection in
  // the paragraph is required to ensure apply_set calls in the work
  // function have completed and all local partitions have been received
  // on the location responsible for processing them.
  map_func<skeletons::tags::with_coarsened_wf>(
    fill_and_merge_buckets<Compare, SplittersView>(comp, splitters_v),
    input_v, make_repeat_view(split_input_v));

  // 2) compute buckets sizes
  // 3) compute global offsets, with shifting
  // 4) use the functor to process the elements in each partition
  // 5) copy sorted elements back in input view

  using namespace skeletons;
#ifndef USE_NEW_NOTATION
  DECLARE_INLINE_PLACEHOLDERS(4, x);

  namespace ph = flows::inline_flows::placeholders;
  ph::input<0> buckets_in;
  ph::input<1> repeated_input_in;

  auto phases_skel =
    compose<tags::inline_flow>(
      x0 << skeletons::map(partition_functor) | buckets_in,
      x1 << skeletons::map(return_buckets_sizes_wf()) | buckets_in,
      x2 << scan<tags::blelloch>(stapl::plus<std::size_t>(), 0) | x1,
      x3 << zip<3>(copy_to_input_wf<Compare>()) | (x0, x2, repeated_input_in)
    );

#else
    auto partition_buckets = skeletons::map(partition_functor);

    auto scan_buckets = ser(
                          skeletons::map(return_buckets_sizes_wf()),
                          scan<tags::blelloch>(stapl::plus<std::size_t>(), 0));

    auto copy_skel = zip<3>(copy_to_input_wf<Compare>());

    auto partition_and_scan = branch(partition_buckets, scan_buckets);

    auto phases = add_input(partition_and_scan, copy_skel);
    auto phases_skel = to_skeleton(phases);
#endif

  execute(
    skeletons::execution_params(coarsen_all_but_last<default_coarsener>()),
    phases_skel,
    split_input_v, make_repeat_view(input_v));
}


//////////////////////////////////////////////////////////////////////
/// @brief Implementation of the n_partition() algorithm.
/// @param input_v            The input view to be partitioned.
/// @param splitters_v        The splitters used to determine an element's
///   partition.
/// @param comp               The comparator used to compare elements to
///   splitters.
/// @param partition_functor  Work function which processes partitions.
///
/// @todo combine factory calls in the following lines into one skeleton
/// @todo The @ref map_reduce() should automatically handle views that have
///       less elements than the number of locations.
//////////////////////////////////////////////////////////////////////
template<typename InputView, typename SplittersView, typename Compare,
         typename Functor>
segmented_view<InputView,
                 splitter_partition<typename InputView::domain_type> >
n_partition_impl(InputView& input_v, SplittersView const& splitters_v,
                 Compare const& comp, Functor const& partition_functor)
{
  typedef typename extract_base_comparator<Compare>::element element_t;

  // 1) compute split input
  using split_input_t = static_array<lightweight_vector<element_t>>;
  using split_input_vt = array_view<split_input_t>;

  const size_t counted_num_buckets = splitters_v.size()+1;

  split_input_t split_input(counted_num_buckets > get_num_locations() ?
                            counted_num_buckets : get_num_locations());
  split_input_vt split_input_v(split_input);

  // not composed with phases below because the termination detection in
  // the paragraph is required to ensure apply_set calls in the work
  // function have completed and all local partitions have been received
  // on the location responsible for processing them.
  map_func<skeletons::tags::with_coarsened_wf>(
    fill_and_merge_buckets<Compare, SplittersView>(comp, splitters_v),
    input_v, make_repeat_view(split_input_v));

  // 2) compute buckets sizes
  typedef static_array<std::size_t> s_array_t;
  typedef array_view<s_array_t>     s_array_vt;

  s_array_t buckets_sizes(split_input_v.size());
  s_array_vt buckets_sizes_view(buckets_sizes);
  map_func(buckets_sizes_wf(), split_input_v, buckets_sizes_view);

  // 3) compute global offsets, with shifting
  partial_sum(buckets_sizes_view, buckets_sizes_view, true);

  // 4) use the functor to process the elements in each partition
  map_func(partition_functor, split_input_v, buckets_sizes_view);

  // 5) copy_back split input into the input InputView
  map_func(copy_to_input_wf<Compare>(), split_input_v, buckets_sizes_view,
           make_repeat_view(input_v));

  // 6) reduce all the buckets sizes into the domain offsets
  typedef std::vector<std::size_t> offsets_t;

  offsets_t domain_offsets;

  if (buckets_sizes_view.size() < get_num_locations()) {
    // Remove this code when the @todo about map_reduce() is resolved.
    domain_offsets = map_reduce<skeletons::tags::no_coarsening>(
                        to_vector_wf<std::size_t>(),
                        concat_vector_wf<std::size_t>(),
                        buckets_sizes_view);
  }
  else {
    domain_offsets = map_reduce(to_vector_wf<std::size_t>(),
                                concat_vector_wf<std::size_t>(),
                                buckets_sizes_view);
  }

  // 7) create the segmented_view, allowing empties
  typedef typename InputView::domain_type           domain_t;
  typedef splitter_partition<domain_t>              split_part_t;
  typedef segmented_view<InputView, split_part_t> split_t;

  domain_offsets.erase(domain_offsets.begin());
  split_part_t split_part(input_v.domain(), domain_offsets, true);
  split_t split_pv(input_v, split_part);

  return split_pv;
}


//////////////////////////////////////////////////////////////////////
/// @brief Implementation of the partial_sort() algorithms.
/// @param input_v The input view to be sorted.
/// @param comp    The comparator used for sorting.
/// @param f       Work function which processes partitions.
/// @todo Replace the sequential sort of the splitters call with another
/// sorting algorithm (p_merge_sort?) once one is implemented.
/// @note a sequential vector is used to collect the splitters, resulting in
/// its replication across locations.
///
/// It is also used by the the nth_element() algorithm.
//////////////////////////////////////////////////////////////////////
template<typename InputView, typename Compare, typename Functor>
void partial_sort_impl(InputView input_v, Compare comp, Functor f)
{
  typedef typename InputView::value_type value_type;
  typedef std::vector<value_type>        s_array_t;

  // pick unique random splitters from input_v
  const std::size_t nb_locations = input_v.get_num_locations();

  std::size_t num_splitters = nb_locations == 1 ? 1 : nb_locations - 1;

  sample_wf<value_type> sampler(1, nb_locations, EVEN);
  s_array_t splitters(*map_reduce(sampler, reduce_vector_wf<value_type>(),
                                  balance_view(input_v, num_splitters)).get());

  // sort the splitters
  std::sort(splitters.begin(), splitters.end(), comp);

  // delete duplicates
  splitters.resize(std::distance(splitters.begin(),
                                  std::unique(splitters.begin(),
                                              splitters.end())));

  n_partition(input_v, splitters, comp, f);
}


//////////////////////////////////////////////////////////////////////
/// @brief Count the occurrences of each radix in the input view.
//////////////////////////////////////////////////////////////////////
class radix_sort_count_wf
{
private:
  int m_r;
  int m_pass;
  int m_mask;
  int m_two_to_r;

public:
  typedef void result_type;

  radix_sort_count_wf(int r0, int p, int m, int twotor)
    : m_r(r0), m_pass(p), m_mask(m), m_two_to_r(twotor)
  {}

  template<typename DataView, typename CountRef>
  result_type operator()(DataView dataView, CountRef count_ref)
  {
    typename CountRef::value_type cv(m_two_to_r, 0);
    typename DataView::value_type value;

    for (typename DataView::iterator it = dataView.begin();
                                     it != dataView.end();
                                     ++it) {
      value = ((*it)>>(m_r * m_pass)) & m_mask;
      ++cv[value];
    }
    count_ref = cv;
  }

  void define_type(typer& t)
  {
    t.member(m_r);
    t.member(m_pass);
    t.member(m_mask);
    t.member(m_two_to_r);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Binary function to add two vectors, used by radix_sort().
/// @tparam T The integral type used to count occurrences of each radix.
//////////////////////////////////////////////////////////////////////
template<typename T>
class radix_sort_vectorplus
  : public stapl::ro_binary_function<T, T, T>
{
public:
  typedef T result_type;

  result_type operator()(T const& v1, T const& v2) const
  {
    if (v1.size() == 0)
      return v2;

    if (v2.size() == 0)
      return v1;

    int len = v1.size();
    result_type v(len);
    typename T::iterator it0 = v.begin();

    for (typename T::const_iterator it1 = v1.begin(), it2 = v2.begin();
                                    it1 != v1.end() && it2 != v2.end();
                                    ++it1, ++it2, ++it0) {
      *it0 = *it1 + *it2;
    }
    return v;
  }
};

} // namespace algo_details


template<typename T>
struct identity_value<algo_details::radix_sort_vectorplus<T>, T>
{
  static T value(void) { return T(); }
};


namespace algo_details {

//////////////////////////////////////////////////////////////////////
/// @brief Used by radix_sort() to copy values to their final positions in
///   the destination view.
/// @tparam T The data container type.
//////////////////////////////////////////////////////////////////////
template<typename T>
class radix_sort_pos_wf
{
private:
  typedef typename T::value_type    countvector_t;

  countvector_t m_totalsumsp;
  int m_r;
  int m_pass;
  int m_mask;

public:
  typedef void result_type;

  radix_sort_pos_wf(countvector_t totalsums, int r, int p, int m)
    : m_totalsumsp(totalsums), m_r(r), m_pass(p), m_mask(m)
  {}

  template<typename DataView, typename Dest, typename CountView>
  result_type operator()(DataView dataNatView, Dest dest, CountView countView)
  {
    // update the counts
    countvector_t counts = countView;
    int len = m_totalsumsp.size();
    for (int j=0; j<len; ++j) {
      counts[j] = m_totalsumsp[j] + counts[j];
    }

    // place data
    typename DataView::value_type j;
    for (typename DataView::iterator it = dataNatView.begin();
                                     it != dataNatView.end();
                                     ++it) {
      j = ((*it)>>(m_r * m_pass)) & m_mask;
      dest.set_element(counts[j], *it);

      ++counts[j];
    }
  }

  void define_type(typer& t)
  {
    t.member(m_totalsumsp);
    t.member(m_r);
    t.member(m_pass);
    t.member(m_mask);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Reset the first vectors of counts, used in radix_sort().
/// @tparam T The data container type.
//////////////////////////////////////////////////////////////////////
template<typename T>
struct reset_counts_head_wf
{
  typedef void result_type;

  std::size_t m_len;

  reset_counts_head_wf(std::size_t l)
    : m_len(l)
  {}

  template<typename View>
  result_type operator()(View v)
  {
      T t = *(v.begin());
      t.assign(m_len, 0);
      *(v.begin()) = t;
  }

  void define_type(typer& t)
  {
    t.member(m_len);
  }
};
} //namespace algo_details


//////////////////////////////////////////////////////////////////////
/// @brief Sorts the elements of the input view according to the comparator
///   provided using a sample-based approach.
/// @param[in,out] view                    The view of elements to sort.
/// @param[in]     comp                    The strict weak ordering comparison
///   functor.
/// @param[in]     sampling_method         Method used for sampling
///   (0: even, 1: semi-random, 2:random, 3: block)
/// @param[in]     over_partitioning_ratio Number of buckets per location.
/// @param[in]     over_sampling_ratio     Number of samples taken per bucket.
/// @ingroup sortAlgorithms
///
/// The order of equal elements is not guaranteed to be preserved.
/// The given comparison function comp is used to compare the elements.
/// The given method sampling_method is used to select the sampling process.
/// The given ratio over_partitioning_ratio is used to determine the number
///   of buckets per location.
/// The given ratio over_sampling_ratio is used to determine the number of
///   samples per bucket.
//////////////////////////////////////////////////////////////////////
template<typename View, typename Compare>
inline
void sample_sort(View& view,
                 Compare comp,
                 size_t sampling_method,
                 size_t over_partitioning_ratio = 1,
                 size_t over_sampling_ratio = 128)
{
  using algo_details::sample_sort_impl;

  sample_sort_impl(view, comp,
                   sampling_method, over_partitioning_ratio,
                   over_sampling_ratio);
}


//////////////////////////////////////////////////////////////////////
/// @brief Sorts the elements of the input view according to the comparator
///   provided using a sample-based approach.
/// @param[in,out] view                    The view of elements to sort.
///   true if the first argument is less than the second.
/// @param[in]     sampling_method         Method used for sampling
///   (0: even, 1: semi-random, 2:random, 3: block)
/// @param[in]     over_partitioning_ratio Number of buckets per location.
/// @param[in]     over_sampling_ratio     Number of samples taken per bucket.
/// @ingroup sortAlgorithms
///
/// The order of equal elements is not guaranteed to be preserved.
/// The comparator @ref less is used to compare the elements.
/// The given method sampling_method is used to select the sampling process.
/// The given ratio over_partitioning_ratio is used to determine the number
///   of buckets per location.
/// The given ratio over_sampling_ratio is used to determine the number of
///   samples per bucket.
//////////////////////////////////////////////////////////////////////
template<typename View>
inline
void sample_sort(View& view,
                 size_t sampling_method,
                 size_t over_partitioning_ratio = 1,
                 size_t over_sampling_ratio = 128)
{
  using algo_details::sample_sort_impl;

  sample_sort_impl(view, std::less<typename View::value_type>(),
                   sampling_method, over_partitioning_ratio,
                   over_sampling_ratio);
}


//////////////////////////////////////////////////////////////////////
/// @brief Sorts the elements of the input view according to the comparator
///   provided using a sample-based approach.
/// @param[in,out] view                    The view of elements to sort.
/// @param[in]     comp                    The strict weak ordering comparison
///   functor.
/// @ingroup sortAlgorithms
///
/// The order of equal elements is not guaranteed to be preserved.
/// The given comparison function comp is used to compare the elements.
/// The default 'EVEN' sampling method is used.
/// The default '1' over partitioning ratio is used to determine the number
///   of buckets per location.
/// The default '128' over sampling ratio is used to determine the number of
///   samples per bucket.
//////////////////////////////////////////////////////////////////////
template<typename View, typename Compare>
inline
void sample_sort(View& view, Compare comp)
{
  using algo_details::sample_sort_impl;
  sample_sort_impl(view, comp, EVEN, 1, 128);
}


//////////////////////////////////////////////////////////////////////
/// @brief Sorts the elements of the input view according to the comparator
///   provided using a sample-based approach.
/// @param[in,out] view                    The view of elements to sort.
/// @ingroup sortAlgorithms
///
/// The order of equal elements is not guaranteed to be preserved.
/// The comparator @ref less is used to compare the elements.
/// The default 'EVEN' sampling method is used.
/// The default '1' over partitioning ratio is used to determine the number
///   of buckets per location.
/// The default '128' over sampling ratio is used to determine the number of
///   samples per bucket.
//////////////////////////////////////////////////////////////////////
template<typename View>
inline
void sample_sort(View& view)
{
  using algo_details::sample_sort_impl;
  using value_type = typename View::value_type;

  sample_sort_impl(view, less<value_type>(), EVEN, 1, 128);
}


//////////////////////////////////////////////////////////////////////
/// @brief Sorts the elements of the input view according to the comparator
///   provided.
/// @param[in,out] view The view to be sorted.
/// @param[in]     comp The strict weak ordering comparison functor.
/// @ingroup sortAlgorithms
///
/// sample_sort() is used for sorting.
/// The order of equal elements is not guaranteed to be preserved.
/// The given comparison function pred is used to compare the elements.
//////////////////////////////////////////////////////////////////////
template<typename View, typename Comparator>
void sort(View& view, Comparator comp)
{
  sample_sort(view, comp);
}


//////////////////////////////////////////////////////////////////////
/// @brief Sorts the elements of the input view according to the comparator
///   provided.
/// @param[in,out] view The view to be sorted.
/// @ingroup sortAlgorithms
///
/// sample_sort() is used for sorting.
/// The order of equal elements is not guaranteed to be preserved.
/// The comparator @ref less is used to compare the elements.
//////////////////////////////////////////////////////////////////////
template<typename View>
void sort(View& view)
{
  sample_sort(view);
}


//////////////////////////////////////////////////////////////////////
/// @brief Sorts the elements of the input view according to the comparator
///        provided.
///        The order of equal elements is guaranteed to be preserved.
/// @param[in,out] view The view to be sorted.
/// @param[in]     comp The strict weak ordering comparison functor.
/// @ingroup sortAlgorithms
///
/// sample_sort() is used for sorting.
/// The given comparison function pred is used to compare the elements.
//////////////////////////////////////////////////////////////////////
template<typename View, typename Comparator>
void stable_sort(View& view, Comparator comp)
{
   sample_sort(view, algo_details::comparator_wrapper<Comparator>(comp));
}

//////////////////////////////////////////////////////////////////////
/// @brief Sorts the elements of the input view according to the '<'
///        comparison function of the elements.
///        The order of equal elements is guaranteed to be preserved.
/// @param[in,out] view The view to be sorted.
/// @ingroup sortAlgorithms
///
/// sample_sort() is used for sorting.
/// The comparator @ref less is used to compare the elements.
//////////////////////////////////////////////////////////////////////
template<typename View>
void stable_sort(View& view)
{
  using value_type = typename View::value_type;

  sample_sort(view, algo_details::comparator_wrapper<less<value_type>>(
                      less<value_type>()));
}


//////////////////////////////////////////////////////////////////////
/// @brief Reorders the elements in the input view in such a way that all
///   elements for which the comparator returns true for a splitter s
///   - within the input splitters set - precede the elements for which the
///   compare function returns false.
///   Each set of partitioned elements is processed by the given
///     partition_functor function.
///   The relative ordering of the elements is not preserved.
/// @param[in,out] input_v         The input view to be partitioned.
/// @param[in]     splitters       The set of splitters to partition the input.
/// @param[in]     comp            The strict weak ordering comparison functor.
/// @param[in]     partition_functor Functor processing each partition.
/// @return segmented_view A view over the segments of elements.
/// @ingroup sortAlgorithms
/// @note a sequential vector is used to collect the splitters, resulting in
/// its replication across locations.
///
/// The set of splitters must be sorted with the same comparator prior to
/// being passed to the algorithm.
///
/// @todo The @ref map_reduce() should automatically handle views that have
///       less elements than the number of locations.
//////////////////////////////////////////////////////////////////////
template<typename InputView, typename SplittersView, typename Compare,
         typename Functor>
segmented_view<InputView,
                 splitter_partition<typename InputView::domain_type> >
n_partition(InputView& input_v,
            SplittersView const& splitters,
            Compare const& comp,
            Functor const& partition_functor)
{
  typedef typename SplittersView::value_type splitters_value_t;

  // share splitters
  typedef std::vector<splitters_value_t> splitters_t;

  splitters_t shared_splitters;

  if (splitters.size() < get_num_locations()) {
    // Remove this code when the @todo about map_reduce() is resolved.
    shared_splitters = map_reduce<skeletons::tags::no_coarsening>(
                            algo_details::to_vector_wf<splitters_value_t>(),
                            algo_details::concat_vector_wf<splitters_value_t>(),
                            splitters);
  }
  else {
    shared_splitters = map_reduce(
                            algo_details::to_vector_wf<splitters_value_t>(),
                            algo_details::concat_vector_wf<splitters_value_t>(),
                            splitters);
  }

  return algo_details::n_partition_impl(input_v, shared_splitters, comp,
                          partition_functor);
}


//////////////////////////////////////////////////////////////////////
/// @brief Reorders the elements in the input view in such a way that all
///   elements for which the comparator returns true for a splitter s
///   - within the input splitters set - precede the elements for which the
///   compare function returns false.
///   The relative ordering of the elements is not preserved.
/// @param[in,out] input_v         The input view to be partitioned.
/// @param[in]     splitters       The set of splitters to partition the input.
/// @param[in]     comp            The strict weak ordering comparison functor.
/// @return segmented_view A view over the segments of elements.
/// @ingroup sortAlgorithms
///
/// This function calls the algorithm that accepts a partition functor with
/// a functor that performs no work.
/// The set of splitters must be sorted with the same comparator prior to
/// being passed to the algorithm.
//////////////////////////////////////////////////////////////////////
template<typename InputView, typename SplittersView, typename Compare>
segmented_view<InputView,
                 splitter_partition<typename InputView::domain_type> >
n_partition(InputView& input_v,
            SplittersView const& splitters,
            Compare const& comp)
{
  algo_details::neutral_functor f;
  return n_partition(input_v, splitters, comp, f);
}


//////////////////////////////////////////////////////////////////////
/// @brief Reorders the elements in the input view in such a way that all
///   elements for which the default '<' comparison function returns
///   true for a splitter s - within the input splitters set - precede the
///   elements for which the compare function returns false.
///   The relative ordering of the elements is not preserved.
/// @param[in,out] input_v         The input view to be partitioned.
/// @param[in]     splitters       The set of splitters to partition the input.
/// @return segmented_view A view over the segments of elements.
/// @ingroup sortAlgorithms
///
/// This function defines the comparator used to be @ref less and the partition
/// functor to be one that performs no work.
/// The set of splitters must be sorted with the '<' operator.
//////////////////////////////////////////////////////////////////////
template<typename InputView, typename SplittersView>
segmented_view<InputView,
                 splitter_partition<typename InputView::domain_type> >
n_partition(InputView& input_v, SplittersView const& splitters)
{
  return n_partition(input_v, splitters,
                     less<typename InputView::value_type>());
}


//////////////////////////////////////////////////////////////////////
/// @brief Reorders the elements in the input view in such a way that all
///   elements for which the comparator returns true for a splitter s
///   - within the input splitters set - precede the elements for which the
///   compare function returns false.
///   Each set of partitioned elements is processed by the given
///     partition_functor function.
///   The relative ordering of the elements is not preserved.
/// @param[in,out] input_v          The input view to be partitioned.
/// @param[in]     splitters        The set of splitters to partition the input.
/// @param[in]     comp             The strict weak ordering comparison functor.
/// @param[in]     partition_functor Functor processing each partition.
/// @return segmented_view A view over the segments of elements.
/// @ingroup sortAlgorithms
///
/// The set of splitters is std::vector<InputView::value_type>.
/// The set of splitters must be sorted with the same comparator prior to
/// being passed to the algorithm.
//////////////////////////////////////////////////////////////////////
template<typename InputView, typename Compare, typename Functor>
segmented_view<InputView,
                 splitter_partition<typename InputView::domain_type> >
n_partition(InputView& input_v,
            std::vector<typename InputView::value_type> const& splitters,
            Compare const& comp,
            Functor const& partition_functor)
{
  return
    algo_details::n_partition_impl(input_v, splitters, comp, partition_functor);
}


//////////////////////////////////////////////////////////////////////
/// @brief Performs a partial sort of the data in the input view using the
///   comparator provided such that all elements before the nth position are
///   less than the elements that follow the nth element, and the value stored
///   in the nth element is the same as if the input view were completely
///   sorted.
/// @param[in,out]  input_v The input view to be sorted.
/// @param[in]      nth     The iterator defining the sort partition point.
/// @param[in]      comp    Functor used to compare elements to determine their
///   relative ordering.
///
/// The nth iterator must be the same type as InputView::iterator.
//////////////////////////////////////////////////////////////////////
template<typename InputView, typename Compare>
void nth_element(InputView input_v,
                 typename InputView::iterator nth,
                 Compare comp)
{
  if (nth < input_v.begin() || input_v.end() < nth)
    return;

  algo_details::nth_element_pf<Compare> pf(comp, nth - input_v.begin());
  return algo_details::partial_sort_impl(input_v, comp, pf);
}


//////////////////////////////////////////////////////////////////////
/// @brief Performs a partial sort of the data in the input view using the
///   comparator provided such that all elements before the nth position are
///   less than the elements that follow the nth element, and the value stored
///   in the nth element is the same as if the input view were completely/
///   sorted.
/// @param[in,out]  input_v The input view to be sorted.
/// @param[in]      nth     The iterator defining the sort partition point.
///
/// This algorithm calls the version that accepts a comparator function with
/// @ref less.
/// The nth iterator must be the same type as InputView::iterator.
//////////////////////////////////////////////////////////////////////
template<typename InputView>
void nth_element(InputView input_v,
                 typename InputView::iterator nth)
{
  return nth_element(input_v, nth, less<typename InputView::value_type>());
}


//////////////////////////////////////////////////////////////////////
/// @brief Performs a partial sort of the data in the input view using the
///   comparator provided such that all elements before the nth position are
///   sorted using the comparator.
/// @param[in,out] input_v The input view to be sorted.
/// @param[in]     nth     The iterator to sort up to.
/// @param[in]     comp    The strict weak ordering comparison functor.
/// @ingroup sortAlgorithms
///
/// The nth iterator must be the same type as InputView::iterator.
/// The order of equal elements is not guaranteed to be preserved.
//////////////////////////////////////////////////////////////////////
template<typename InputView, typename Compare>
void partial_sort(InputView input_v,
                  typename InputView::iterator nth,
                  Compare comp)
{
  if (nth < input_v.begin() || input_v.end() < nth)
    return;

  algo_details::partial_sort_pf<Compare> pf(comp, nth - input_v.begin());
  return algo_details::partial_sort_impl(input_v, comp, pf);
}


//////////////////////////////////////////////////////////////////////
/// @brief Performs a partial sort of the data in the input view using the
///   comparator provided such that all elements before the nth position are
///   sorted using the comparator.
/// @param[in,out] input_v The input view to be sorted.
/// @param[in]     nth     The iterator to sort up to.
/// @ingroup sortAlgorithms
///
/// The nth iterator must be the same type as InputView::iterator.
/// The order of equal elements is not guaranteed to be preserved.
/// The @ref less comparison function is used to compare the elements.
//////////////////////////////////////////////////////////////////////
template<typename InputView>
void partial_sort(InputView input_v,
                  typename InputView::iterator nth)
{
  return partial_sort(input_v, nth, less<typename InputView::value_type>());
}


//////////////////////////////////////////////////////////////////////
/// @brief Performs a partial sort of the input view data into the output view
///   using the comparator provided such that all elements before the nth
///   position are sorted using the comparator.
/// @param[in]  input_v  The input view to read from.
/// @param[out] output_v The output view to write to.
/// @param[in]  comp     The strict weak ordering comparison functor.
/// @return Returns a view with the sorted elements.
/// @ingroup sortAlgorithms
///
/// The order of equal elements is not guaranteed to be preserved.
//////////////////////////////////////////////////////////////////////
template<typename InputView, typename OutputView, typename Compare>
OutputView partial_sort_copy(InputView input_v,
                             OutputView output_v,
                             Compare comp)
{
  typedef static_array<typename InputView::value_type> s_array_t;
  typedef array_view<s_array_t>                        s_array_vt;

  std::size_t N = std::min<std::size_t>(input_v.size(), output_v.size());

  // tmp will contain the whole set of elements to be sorted
  s_array_t tmp(input_v.size());
  s_array_vt tmp_v(tmp);
  copy(input_v, tmp_v);

  partial_sort(tmp_v, tmp_v.begin() + N, comp);

  copy(s_array_vt(tmp_v.container(),
                  typename s_array_vt::domain_type(0, N - 1)),
       OutputView(output_v.container(),
                  typename OutputView::domain_type(0, N - 1)));
  return OutputView(output_v.container(),
                    typename OutputView::domain_type(0, N - 1));
}


//////////////////////////////////////////////////////////////////////
/// @brief Performs a partial sort of the input view data into the output view
///   using the comparator provided such that all elements before the nth
///   position are sorted using the comparator.
/// @param[in]  input_v  The input view to read from.
/// @param[out] output_v The output view to write to.
/// @return Returns a view with the sorted elements.
/// @ingroup sortAlgorithms
///
/// The order of equal elements is not guaranteed to be preserved.
/// The @ref less comparison function is used to compare the elements.
//////////////////////////////////////////////////////////////////////
template<typename InputView, typename OutputView>
OutputView partial_sort_copy(InputView input_v,
                             OutputView output_v)
{
  return partial_sort_copy(input_v, output_v,
                           less<typename InputView::value_type>());
}


//////////////////////////////////////////////////////////////////////
/// @brief Sorts the element in the input view according to the radix-sort
///   algorithm.
/// @param[in,out] input_v The input view to be sorted.
/// @ingroup sortAlgorithms
///
/// Only accepts integers as data type.
/// @todo combine factory calls in the following lines into one pattern
//////////////////////////////////////////////////////////////////////
template<typename InputView>
void radix_sort(InputView input_v)
{
  typedef typename InputView::value_type                value_t;
  typedef typename InputView::iterator::difference_type count_t;
  typedef typename std::vector<count_t>                 countvector_t;

  // 1) find maximal value => nb of needed passes
  value_t global_max = max_element(input_v);
  int range = static_cast<int>
    (log10(static_cast<double>(global_max))/log10(2.0) + 1);
  int passes = range/16 + 1;
  if (range % passes != 0) {
    range = (range/passes + 1) * passes;
  }

  std::size_t bits     = range;
  std::size_t n        = passes;
  std::size_t r        = bits/n;    //consider r bits at a time
  std::size_t two_to_r = (std::size_t) (pow((float) (2), r));
  std::size_t mask     = two_to_r - 1;

  // 2) create a copy to manipulate data, then we'll copy back the data
  // PURPOSE: switch between this container and the input at each pass
  typedef array<value_t>        ipcont_t;
  typedef array_view<ipcont_t>  iview_t;
  ipcont_t temp_pcont(input_v.size());
  iview_t temp_v(temp_pcont);
  copy(input_v, temp_v);

  // 3) store counts of values
  typedef array<countvector_t> cpcont_t;
  typedef array_view<cpcont_t> cview_t;

  // construct native view to allow count_wf to process a range
  typedef typename
    stapl::result_of::native_view<InputView>::type native_input_vt;
  native_input_vt native_input_view = stapl::native_view(input_v);
  typedef typename stapl::result_of::native_view<iview_t>::type native_temp_vt;
  native_temp_vt native_temp_view = stapl::native_view(temp_v);

  // create the array to hold count results from each range
  const std::size_t nat_data_sz = native_input_view.size();
  cpcont_t count(nat_data_sz);
  cview_t count_v(count);

  // cpcont_t count2(nat_data_sz);
  // cview_t count_v2(count2);

  // 4) For the ith block of r bits do: (going from LSB to MSB)
  for (std::size_t i=0; i<n; ++i) {
    // 4.a) count the # element per bucket on each location
    algo_details::radix_sort_count_wf RSC_wf(r, i, mask, two_to_r);
    if (i % 2 == 0) { // even: input view
      map_func(RSC_wf, native_input_view, count_v);
      // map_func(RSC_wf, native_input_view, count_v2);
    } else { // odd: temp view
      map_func(RSC_wf, native_temp_view, count_v);
      // map_func(RSC_wf, native_temp_view, count_v2);
    }

    // 4.b) compute the offsets at which eachrange of data on a location should
     // be copy to in the destination container
    algo_details::radix_sort_vectorplus<countvector_t> vp;

    countvector_t totalsums =
      partial_sum_accumulate(count_v, count_v, countvector_t(), vp, true);

    // then reset the first vector with {0}
    do_once(
      std::bind(algo_details::reset_counts_head_wf<countvector_t>(two_to_r),
                count_v));

    // 4.c) compute the prefix sums of total counts for every value, then shift
    std::partial_sum(totalsums.begin(), totalsums.end(), totalsums.begin());
    totalsums.insert(totalsums.begin(), 0);
    totalsums.resize(totalsums.size() - 1);

    // 4.d) put values at the right positions
    if (i % 2 == 0) { //even: input view
      algo_details::radix_sort_pos_wf<cview_t> RSP_wf(totalsums, r, i, mask);
      map_func(RSP_wf, native_input_view, make_repeat_view(temp_v), count_v);
    } else { // odd: temp view
      algo_details::radix_sort_pos_wf<cview_t> RSP_wf(totalsums, r, i, mask);
      map_func(RSP_wf, native_temp_view, make_repeat_view(input_v), count_v);
    }
  } //end for loop

  // 5) copy back temp sorted data into the input if #passes is odd
  if (n % 2 == 1){
    copy(temp_v, input_v);
  }

  return;
}


namespace algo_details {

//////////////////////////////////////////////////////////////////////
/// @brief Work function which implements a lexicographical ordering based on
///   the provided functor.
/// @tparam Pred Binary functor implementing the less operation.
//////////////////////////////////////////////////////////////////////
template<typename Pred>
class lexico_compare
{
private:
  Pred m_pred;

public:
  lexico_compare(Pred const& pred)
    : m_pred(pred) {}

  void define_type(typer& t)
  {
    t.member(m_pred);
  }

  typedef int result_type;

  template <typename Elm1, typename Elm2>
  int operator()(Elm1 element1, Elm2 element2) const
  {
    if (m_pred(element1,element2))
      return 1;
    else if (m_pred(element2,element1))
      return -1;

    return 0;
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Work function which returns the second argument if the first
///   compares equal to 0, and the first otherwise.
/// @tparam T The type of the arguments.
//////////////////////////////////////////////////////////////////////
template <typename T>
struct lexico_reduce
  : public ro_binary_function<T, T, T>
{
  template <typename Elm>
  T operator()(Elm elem1, Elm elem2){
    if (elem1 == 0){
      return (T)elem2;
    } else {
      return (T)elem1;
    }
  }
};

} //namespace algo_details


//////////////////////////////////////////////////////////////////////
/// @brief Determines if the first view is lexicographically less than the
///   second view, using the given functor.
/// @param pview1 One-dimensional view of the first input.
/// @param pview2 One-dimensional view of the second input.
/// @param pred Binary functor which implements the less operation.
/// @return True if the first input is lexicographically less than the second.
/// @ingroup sortrelatedAlgorithms
///
/// This algorithm is non-mutating.
////////////////////////////////////////////////////////////////////
template <typename View, typename View2, typename Pred>
bool lexicographical_compare(View const& pview1, View2 const& pview2,
                             Pred const& pred)
{
  int minelem = 0;
  int result;
  int size1 = pview1.size();
  int size2 = pview2.size();
  if (size1 > size2){
    minelem = size2;
    View viewtrunc(pview1.container(), typename View::domain_type(0,minelem-1));
    result = stapl::map_reduce(algo_details::lexico_compare<Pred>(pred),
                               algo_details::lexico_reduce<int>(),
                               viewtrunc, pview2);
  }
  else if (size1 < size2) {
    minelem = size1;
    View2 viewtrunc(pview2.container(),typename View::domain_type(0,minelem-1));
    result = stapl::map_reduce(algo_details::lexico_compare<Pred>(pred),
                               algo_details::lexico_reduce<int>(),
                               pview1, viewtrunc);
  }
  else {
    result = stapl::map_reduce(algo_details::lexico_compare<Pred>(pred),
                               algo_details::lexico_reduce<int>(),
                               pview1, pview2);
  }

  if (result == 1){
    return true;
  }
  else if (result == -1){
    return false;
  }
  return size1<size2;
}


//////////////////////////////////////////////////////////////////////
/// @brief Determines if the first view is lexicographically less than the
///   second view.
/// @param pview1 One-dimensional view of the first input.
/// @param pview2 One-dimensional view of the second input.
/// @return True if the first input is lexicographically less than the second.
/// @ingroup sortrelatedAlgorithms
///
/// This version calls the predicated version with a default predicate of
/// stapl::less.
////////////////////////////////////////////////////////////////////
template <typename View, typename View2>
bool lexicographical_compare(View const& pview1, View2 const& pview2)
{
  return lexicographical_compare(pview1, pview2,
                                 less<typename View::value_type>());
}


namespace algo_details {

//////////////////////////////////////////////////////////////////////
/// @brief Work function which takes an input map and sets the given key equal
///   to the given value.
/// @tparam Value Type of the value to set.
/// @see merge
//////////////////////////////////////////////////////////////////////
template<typename Value>
struct merge_send_leader
{
  size_t m_sender;
  Value  m_val;

  merge_send_leader() : m_sender(0), m_val() {}

  merge_send_leader(size_t sender, Value val)
    : m_sender(sender)
    , m_val(val) {}

  template<typename Leaders>
  void operator()(Leaders& leaders) const {
    leaders[m_sender] = m_val;
  }

  void define_type(typer& t)
  {
    t.member(m_sender);
    t.member(m_val);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function which merges two local splitters into the global
///   set of splitters.
/// @see merge
//////////////////////////////////////////////////////////////////////
class merge_splitters
{
private:
  location_type m_me;

public:
  typedef void result_type;

  merge_splitters(location_type me)
    : m_me(me)
  { }

  template<typename LocalView, typename LocalSplitsA, typename LocalSplitsB,
           typename GlobalIndices>
  void operator()(LocalView A, LocalSplitsA locA, LocalSplitsB locB,
                  GlobalIndices globAix)
  {
    // note that locA/locB are assumed to be std::vectors
    // and globAix is assumed to be a static_array
    typedef typename LocalSplitsA::iterator     locAIter;
    typedef typename LocalSplitsB::iterator     locBIter;

    //find the corresponding bounds in the splitters of B
    locAIter mystart = locA.begin() + m_me;
    locBIter c = std::lower_bound(locB.begin(), locB.end(), *(mystart));
    locBIter d = (mystart+1 != locA.end())
                ? std::lower_bound(locB.begin(), locB.end(), *(mystart+1))
                : locB.end();

    //there are me splitters before this (from other partitions)
    // there are dist(c) splitters before this (from locB)
    size_t ix = std::distance(locB.begin(), c) + m_me;
    globAix[ix] = A.domain().first();

    for (++ix; c != d; ++c, ++ix) {
      globAix[ix] =
        A.domain().advance(A.domain().first(),
                           std::distance(A.begin(),
                                         std::upper_bound(A.begin(), A.end(),
                                                          *c)));
    }
  }

  void define_type(typer& t)
  {
    t.member(m_me);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function which sets the third argument equal to the sums of the
///   sizes of the two input views.
/// @see merge
//////////////////////////////////////////////////////////////////////
struct merge_output_sizes
{
  typedef void result_type;

  template<typename View1, typename View2, typename SizeType>
  void operator()(View1 const& view1, View2 const& view2, SizeType size)
  {
    size = view1.size() + view2.size();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Compute the index of the nth element of the provided view.
/// @see merge
//////////////////////////////////////////////////////////////////////
struct merge_output_indices
{
  typedef void result_type;

  template<typename Size, typename Index, typename View>
  void operator()(Size const& size, Index ix, View const& view)
  {
    ix = view.domain().advance(view.domain().first(), size);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function which merges the two input views into the output view.
/// @see merge
//////////////////////////////////////////////////////////////////////
struct merge_map
{
  typedef void result_type;

  template<typename View1, typename View2, typename MergeView>
  void operator()(View1 const& view1, View2 const& view2, MergeView merged)
  {
    std::merge(view1.begin(), view1.end(), view2.begin(), view2.end(),
               merged.begin());
  }
};

} //namespace algo_details


//////////////////////////////////////////////////////////////////////
/// @brief Merges the two sorted input views into the output view in sorted
///   order.
/// @param view1 One-dimensional view of the first sorted input.
/// @param view2 One-dimensional view of the second sorted input.
/// @param merged One-dimensional view of the output.
/// @ingroup sortrelatedAlgorithms
///
/// This algorithm mutates the output.
//////////////////////////////////////////////////////////////////////
template<typename View1, typename View2, typename MergeView>
void merge(View1 const& view1, View2 const& view2, MergeView& merged)
{
  //TYPEDEFS:

  //Balanced Partitionings
  typedef typename View1::domain_type               dom1_t;
  typedef balanced_partition<dom1_t>                balanced_part1_t;
  typedef segmented_view<View1, balanced_part1_t>   balanced1_t;

  typedef typename View2::domain_type               dom2_t;
  typedef balanced_partition<dom2_t>                balanced_part2_t;
  typedef segmented_view<View2, balanced_part2_t>   balanced2_t;

  typedef typename MergeView::domain_type           domM_t;

  //Distribution of Local Splitters
  typedef typename View1::value_type          val1_t;
  typedef std::vector<val1_t>                 vals1_t;
  typedef static_array<vals1_t>               vals_array1_t;
  typedef array_view<vals_array1_t>           vals_array_view1_t;

  typedef typename View2::value_type          val2_t;
  typedef std::vector<val2_t>                 vals2_t;
  typedef static_array<vals2_t>               vals_array2_t;
  typedef array_view<vals_array2_t>           vals_array_view2_t;

  //Local Local Splitters
  typedef algo_details::merge_send_leader<val1_t>   send_leader1;

  typedef algo_details::merge_send_leader<val2_t>   send_leader2;

  //Splitters (Indices)
  typedef typename View1::index_type                  index1_t;
  typedef static_array<index1_t>                      indices1_t;
  typedef array_view<indices1_t>                      indices1_view_t;

  typedef typename View2::index_type                  index2_t;
  typedef static_array<index2_t>                      indices2_t;
  typedef array_view<indices2_t>                      indices2_view_t;

  typedef typename MergeView::index_type              indexM_t;
  typedef static_array<indexM_t>                      indicesM_t;
  typedef array_view<indicesM_t>                      indicesM_view_t;

  //Segmented Partitionings
  typedef splitter_partition<dom1_t, indices1_view_t> split_part1_t;
  typedef segmented_view<View1, split_part1_t>        split1_t;

  typedef splitter_partition<dom2_t, indices2_view_t> split_part2_t;
  typedef segmented_view<View2, split_part2_t>        split2_t;

  typedef splitter_partition<domM_t, indicesM_view_t> split_partM_t;
  typedef segmented_view<MergeView, split_partM_t>    splitM_t;

  //Sizes of Each Partition
  typedef static_array<size_t>                        sizes_t;
  typedef array_view<sizes_t>                         sizes_view_t;
  typedef typename sizes_view_t::domain_type          sizes_domain_t;

  //ALGORITHM:

  size_t num_locs = get_num_locations();
  size_t my_part = get_location_id();
  size_t num_split = num_locs*2;

  //partition views naturally
  balanced_part1_t    balanced_part1(view1.domain(), num_locs);
  balanced1_t         balanced1(view1, balanced_part1);
  balanced_part2_t    balanced_part2(view2.domain(), num_locs);
  balanced2_t         balanced2(view2, balanced_part2);

  //copy the list of leaders to each location
  //FIXME: use a repeated view of the leaders, not a view of vectors
  vals_array1_t       loc1array(num_locs, vals1_t(num_locs));
  vals_array_view1_t  loc1view(loc1array);
  send_leader1 send_func1(my_part,
                 balanced1[my_part][balanced1[my_part].domain().first()]);

  vals_array2_t       loc2array(num_locs, vals2_t(num_locs));
  vals_array_view2_t  loc2view(loc2array);
  send_leader2 send_func2(my_part,
                 balanced2[my_part][balanced2[my_part].domain().first()]);

  for (size_t i=0; i<num_locs; ++i) {
    loc1view.apply_set(i, send_func1);
    loc2view.apply_set(i, send_func2);
  }
  rmi_fence();

  //find the indices
  indices1_t        indices1(num_split, view1.domain().first());
  indices1_view_t   indices1_view(indices1);
  indices2_t        indices2(num_split, view2.domain().first());
  indices2_view_t   indices2_view(indices2);
  map_func(algo_details::merge_splitters(get_location_id()),
           balanced1, loc1view, loc2view, make_repeat_view(indices1_view));
  map_func(algo_details::merge_splitters(get_location_id()),
           balanced2, loc2view, loc1view, make_repeat_view(indices2_view));

  //partition view2 (so that ranges are same as view1)
  split_part1_t   split_part1(view1.domain(), indices1_view, true);
  split1_t        split1(view1, split_part1);

  split_part2_t   split_part2(view2.domain(), indices2_view, true);
  split2_t        split2(view2, split_part2);

  //TODO: combining merge_output_sizes, partial_sum, and merge_output_indices
  //      could reduce the time significantly on domains where advance() does
  //      not work in constant time

  //find partition on merged by finding sizes and advancing through the domain
  sizes_t       out_sizes(num_split+1);
  sizes_view_t  out_sizev(out_sizes);

  map_func(algo_details::merge_output_sizes(), split1, split2, out_sizev);

  //remove the last size to attain splitters
  sizes_domain_t cut_dom(0, num_split-1);
  out_sizev.set_domain(cut_dom);

  partial_sum(out_sizev, out_sizev);

  indicesM_t        ixMs(num_split);
  indicesM_view_t   ixMv(ixMs);
  map_func(algo_details::merge_output_indices(),
           out_sizev, ixMv, make_repeat_view(merged));

  split_partM_t split_partM(merged.domain(), ixMv, true);
  splitM_t      splitM(merged, split_partM);

  //merge individual pieces with std::merge
  map_func(algo_details::merge_map(), split1, split2, splitM);

  rmi_fence();
}


namespace algo_details{

//////////////////////////////////////////////////////////////////////
/// @brief Work function invokes sequential (i.e., STL) @p is_sorted.
//////////////////////////////////////////////////////////////////////
template<typename Comparator>
class range_is_sorted
{
private:
  Comparator m_comparator;

public:
  range_is_sorted(Comparator comparator)
    : m_comparator(std::move(comparator))
  { }

  template<typename View>
  bool operator()(View const& vw) const
  {
    auto source_vw = vw.container().view();

    using domain_type = typename decltype(source_vw)::domain_type;
    using result_type = typename View::reference::reference;

    if (vw.domain().last() == source_vw.domain().last()) {
      if (vw.domain().size() == 1)
        return result_type(null_reference());

      source_vw.set_domain(
        domain_type(vw.domain().first(), vw.domain().last()));
    }
    else {
      source_vw.set_domain(
        domain_type(vw.domain().first(), vw.domain().last() + 1));
    }

    return std::is_sorted(source_vw.begin(), source_vw.end(), m_comparator);
  }

  void define_type(typer& t)
  { t.member(m_comparator); }
};

} //namespace algo_details


//////////////////////////////////////////////////////////////////////
/// @brief Computes whether the input view is sorted.
/// @param view One-dimensional view of the input.
/// @param comparator The strict weak ordering comparison functor.
/// @return True if the input is sorted, false otherwise.
/// @ingroup sortrelatedAlgorithms
///
/// This algorithm is non-mutating.
//////////////////////////////////////////////////////////////////////
template<typename View, typename Comparator>
bool is_sorted(View const& view, Comparator comparator)
{
  return map_reduce<skeletons::tags::with_coarsened_wf>(
    algo_details::range_is_sorted<Comparator>(std::move(comparator)),
    logical_and<bool>(),
    make_overlap_view(view, 1, 0, 1));
}


//////////////////////////////////////////////////////////////////////
/// @brief Computes whether the input view is sorted.
/// @param view One-dimensional view of the input.
/// @return True if the input is sorted, false otherwise.
/// @ingroup sortrelatedAlgorithms
///
/// This version calls the predicated version with a default predicate of
/// stapl::less.
//////////////////////////////////////////////////////////////////////
template<typename View>
bool is_sorted(View const& view)
{
  return is_sorted(view, less<typename View::value_type>());
}


namespace algo_details {

//////////////////////////////////////////////////////////////////////
/// @brief Work function which invokes sequential (i.e., STL)
///   @p is_sorted_until.
//////////////////////////////////////////////////////////////////////
template<typename Comparator>
class range_is_sorted_until
{
private:
  Comparator m_comparator;

public:
  range_is_sorted_until(Comparator comparator)
    : m_comparator(std::move(comparator))
  { }

  template<typename View>
  typename View::reference::reference
  operator()(View const& vw) const
  {
    auto source_vw = vw.container().view();

    using domain_type = typename decltype(source_vw)::domain_type;
    using result_type = typename View::reference::reference;

    if (vw.domain().last() == source_vw.domain().last()) {
      if (vw.domain().size() == 1)
        return result_type(null_reference());

      source_vw.set_domain(
        domain_type(vw.domain().first(), vw.domain().last()));
    }
    else {
      source_vw.set_domain(
        domain_type(vw.domain().first(), vw.domain().last() + 1));
    }

    auto iter =
      std::is_sorted_until(source_vw.begin(), source_vw.end(), m_comparator);

    return iter != source_vw.end() ?
      *iter : result_type(null_reference());
  }

  void define_type(typer& t)
  { t.member(m_comparator); }
}; // class range_is_sorted_until

} // namespace algo_details


//////////////////////////////////////////////////////////////////////
/// @brief Finds the range of elements in the input which are sorted.
/// @param view One-dimensional view of the input.
/// @param comparator The strict weak ordering comparison functor.
/// @return A view over the sorted range.
/// @ingroup sortrelatedAlgorithms
///
/// This algorithm is non-mutating.
//////////////////////////////////////////////////////////////////////
template<typename View, typename Comparator>
View is_sorted_until(View const& view, Comparator comparator)
{
  typename View::reference result =
    map_reduce<skeletons::tags::with_coarsened_wf>(
      algo_details::range_is_sorted_until<Comparator>(std::move(comparator)),
      algo_details::find_reduce(),
      make_overlap_view(view, 1, 0, 1));

  if (is_null_reference(result))
    return view;

  return View(
    view.container(),
    typename View::domain_type(view.domain().first(), index_of(result)));
}


//////////////////////////////////////////////////////////////////////
/// @brief Finds the range of elements in the input which are sorted.
/// @param v One-dimensional view of the input.
/// @return A view over the sorted range.
/// @ingroup sortrelatedAlgorithms
///
/// This version calls the predicated version with a default predicate of
/// stapl::less.
//////////////////////////////////////////////////////////////////////
template<typename View>
View is_sorted_until(View const& view)
{
  return is_sorted_until(view, less<typename View::value_type>());
}


namespace algo_details {

//////////////////////////////////////////////////////////////////////
/// @brief Swap input variables of a binary function.
/// @param pred The predicate to be swapped.
/// @return A binary predicate with its input swapped.
//////////////////////////////////////////////////////////////////////
template<typename T, typename Predicate>
class swapping_vars
  : public ro_binary_function<T, T, bool>
{
private:
  Predicate m_pred;

public:
  swapping_vars(Predicate pred)
  : m_pred(pred)
  { }

  template<typename Reference1, typename Reference2>
  bool operator()(Reference1&& x, Reference2&& y) const
  { return m_pred(y, x); }

  void define_type(stapl::typer& t)
  {
    t.member(m_pred);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Computes the next lexicographical permutation of the input view
///   according to the given predicate.
/// @param vw One-dimensional input view.
/// @param pred Functor which implements an ordering operation.
///
/// This function is used to implement the @ref prev_permutation() and
/// @ref next_permutation() operations.
///
/// @todo The range-based kernel assumes that the domain of the input view
///   is contiguous. Investigate the behavior with enumerated domains like
///   @ref domset1D.
//////////////////////////////////////////////////////////////////////
template<typename View, typename Predicate>
bool lexicographical_permutation(View& vw, Predicate pred)
{
  using ref      = typename View::reference;
  using dom_t    = typename View::domain_type;
  using cont_t   = static_array<int>;
  using view_t   = array_view<cont_t>;

  stapl_assert(vw.domain().is_contiguous(), "lexicographical_permutation for "
    "views with non-contiguous domains is currently not supported");

  if (vw.size() <= 1)
    return false;

  // Find largest index 'i' such that pred(str[i], str[i+1]) returns true.
  ref head_last =
    map_reduce<skeletons::tags::with_coarsened_wf>(
      algo_details::range_reverse_adjacent_find<Predicate>(pred),
      algo_details::find_end_reduce(),
      make_overlap_view(vw, 1, 0, 1));

  // If it is the first permutation element, return the last permutation
  // element.
  if (is_null_reference(head_last))
  {
    stapl::reverse(vw);
    return false;
  }

  size_t tail_first = index_of(head_last);
  tail_first++;

  // View domain from tail_first to end
  View tailvw(vw.container(), dom_t(tail_first, vw.size() - 1));
  // reverse it
  // find_if > head_last

  cont_t c(tailvw.size());
  view_t rev_tail_copy(c);

  copy(tailvw, rev_tail_copy);

  // Find largest index 'j' such that j > i and pred(str[i], str[j]) returns
  // true.
  view_t::reference ref_to_swap =
    map_reduce<skeletons::tags::with_coarsened_wf>(
    algo_details::range_reverse_find<binder1st<Predicate,ref,true>>
                                                (bind1st(pred,head_last)),
    algo_details::find_end_reduce(), rev_tail_copy);

  // Swap references 'i' and 'j'.
  stapl::do_once([&]{
      //std::swap(head_last,ref_to_swap);// <-- ERROR : Don't work
      typename View::value_type temp_val1 = ref_to_swap;
      ref_to_swap = head_last;
      head_last = temp_val1;
  });

  // Reversing tail and copying to original view.
  copy(reverse_view(rev_tail_copy), tailvw);

  return true;
}

} // namespace algo_details


//////////////////////////////////////////////////////////////////////
/// @brief Computes the next lexicographic ordering of the input view
///   (where the highest is sorted in decreasing order), or if input is already
///   in highest order, places it in the lowest permutation (increasing order).
/// @param vw One-dimensional input view.
/// @param pred Functor which implements the less-than operation.
/// @return True if the next permutation was computed, false if the input
///   was placed into the lowest permutation.
/// @ingroup permutingAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename View, typename Predicate>
bool next_permutation(View& vw, Predicate pred)
{
  return algo_details::lexicographical_permutation(vw, pred);
}

//////////////////////////////////////////////////////////////////////
/// @brief Computes the next lexicographic ordering of the input view
///   (where the highest is sorted in decreasing order), or if input is already
///   in highest order, places it in the lowest permutation (increasing order).
/// @param vw One-dimensional input view.
/// @return True if the next permutation was computed, false if the input
///   was placed into the lowest permutation.
/// @ingroup permutingAlgorithms
///
/// This version calls the predicated version with a default predicate of
/// stapl::less.
//////////////////////////////////////////////////////////////////////
template<typename View>
bool next_permutation(View& vw)
{
  return next_permutation(vw, stapl::less<typename View::value_type >());
}

//////////////////////////////////////////////////////////////////////
/// @brief Computes the previous lexicographic ordering of the input view
///   (where the lowest is sorted in increasing order), or if input is already
///   in lowest order, places it in the highest permutation (decreasing order).
/// @param vw One-dimensional input view.
/// @param pred Functor which implements the greater-than operation.
/// @return True if the previous permutation was computed, false if the input
///   was placed into the highest permutation.
/// @ingroup permutingAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename View, typename Predicate>
bool prev_permutation(View& vw, Predicate pred)
{
  algo_details::swapping_vars<typename Predicate::first_argument_type,
                 Predicate> swapper(pred);
  return algo_details::lexicographical_permutation(vw, swapper);
}

//////////////////////////////////////////////////////////////////////
/// @brief Computes the previous lexicographic ordering of the input view
///   (where the lowest is sorted in increasing order), or if input is already
///   in lowest order, places it in the highest permutation (decreasing order).
/// @param vw One-dimensional input view.
/// @return True if the previous permutation was computed, false if the input
///   was placed into the highest permutation.
/// @ingroup permutingAlgorithms
///
/// This version calls the predicated version with a default predicate of
/// stapl::greater.
//////////////////////////////////////////////////////////////////////
template<typename View>
bool prev_permutation(View& vw)
{
  return prev_permutation(vw, stapl::less<typename View::value_type >());
}


} //namespace stapl

#endif
