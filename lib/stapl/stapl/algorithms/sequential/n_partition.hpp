/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_ALGORITHMS_SEQUENTIAL_N_PARTITION_HPP
#define STAPL_ALGORITHMS_SEQUENTIAL_N_PARTITION_HPP

#include <vector>
#include <algorithm>

#include <stapl/containers/partitions/splitter.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/views/segmented_view.hpp>

namespace stapl {
namespace sequential {
namespace result_of {

//////////////////////////////////////////////////////////////////////
/// @brief Type definition shortcuts.
/// @tparam InputView The input view to be n-partitioned.
//////////////////////////////////////////////////////////////////////
template<typename InputView>
struct n_partition
{
  typedef indexed_domain<std::size_t>           index_type;
  typedef splitter_partition<index_type>        spl_type;
  typedef segmented_view<InputView, spl_type> type;
};

} // namespace result_of


//////////////////////////////////////////////////////////////////////
/// @brief Reorders the elements in the input view in such a way that all
///   elements for which the compare function comp returns true for a splitter s
///   - within the input splitters set - precede the elements for which the
///   compare function returns false.
///   Each set of partitioned elements is processed by the given
///     partition_functor function.
///   The relative order of the elements is not preserved.
/// @param[in,out] input_v         The input view to be partitioned.
/// @param[in]     splitters       The set of splitters to partition the input.
/// @param[in]     comp            The strict weak ordering comparison functor.
/// @param[in]     partition_functor Functor processing each partition.
/// @return segmented_view A view over the segments of elements.
///
/// The set of splitters must be sorted with the same comparator prior to be
/// given to the algorithm.
//////////////////////////////////////////////////////////////////////
template<typename InputView, typename SplittersView, typename Compare>
inline typename result_of::n_partition<InputView>::type
n_partition(InputView input_v,
            SplittersView splitters,
            Compare comp)
{
  typedef typename InputView::iterator                              input_it;
  typedef typename SplittersView::iterator                          spl_it;

  typedef std::vector<std::vector<typename InputView::value_type> > tmp_ct;
  typedef typename stapl::array_view<tmp_ct>                        tmp_vt;
  typedef typename tmp_vt::iterator                                 tmp_it;

  tmp_ct tmp(splitters.size() + 1);
  tmp_vt tmp_v(tmp);

  // 1) compute split data
  for (input_it el = input_v.begin(); el != input_v.end(); ++el)
  {
    spl_it spl_it = std::lower_bound(splitters.begin(), splitters.end(), *el,
                                     comp);
    std::size_t index = std::distance(splitters.begin(), spl_it);
    tmp[index].push_back(*el);
  }

  // 2) copy back data into the input view
  std::vector<std::size_t> domain_offsets(tmp_v.size() + 1, 0);
  std::size_t i = 0;
  input_it index = input_v.begin();
  for (tmp_it it = tmp_v.begin(); it != tmp_v.end(); ++it) {
    std::copy((*it).begin(), (*it).end(), index);
    domain_offsets[i + 1] = domain_offsets[i] + (*it).size();
    index += (*it).size();
    ++i;
  }
  domain_offsets.pop_back();
  domain_offsets.erase(domain_offsets.begin());

  typedef typename result_of::n_partition<InputView>  n_partition_t;

  typename n_partition_t::index_type  domain(0, input_v.size() - 1);
  typename n_partition_t::spl_type    part_split(domain, domain_offsets, true);
  typename n_partition_t::type        part(input_v, part_split);

  return part;
}


//////////////////////////////////////////////////////////////////////
/// @brief Reorders the elements in the input view in such a way that all
///   elements for which the default '<' comparison function returns
///   true for a splitter s - within the input splitters set - precede the
///   elements for which the compare function returns false.
///   The relative order of the elements is not preserved.
/// @param[in,out] input_v         The input view to be partitioned.
/// @param[in]     splitters       The set of splitters to partition the input.
/// @return segmented_view A view over the segments of elements.
///
/// The set of splitters must be sorted with the '<' operator.
//////////////////////////////////////////////////////////////////////
template<typename InputView, typename SplittersView>
inline typename result_of::n_partition<InputView>::type
n_partition(InputView input_v, SplittersView splitters)
{
  return n_partition(input_v, splitters,
                     std::less<typename InputView::value_type>());
}

} } // namespace stapl::sequential

#endif

