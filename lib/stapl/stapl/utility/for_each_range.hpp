/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_FOR_EACH_RANGE_HPP
#define STAPL_UTILITY_FOR_EACH_RANGE_HPP

#include <iterator>

namespace stapl {

namespace utility {

//////////////////////////////////////////////////////////////////////
/// @brief Apply a function on a range of iterators for which all of
///        the elements in that range are equal according to a comparator.
///
/// @param begin The begin iterator
/// @param end The end iterator
/// @param f The function to apply to the range. It should accept exactly
///        two arguments: the begin and end iterator of the range
/// @param comp An equality comparison operator to compare elements
///        of the range
//////////////////////////////////////////////////////////////////////
template <typename Iterator, typename F, typename Comp>
void for_each_range(Iterator begin, Iterator end, F&& f, Comp&& comp)
{
  Iterator r_begin = begin;

  if (begin == end)
    return;

  for (begin = std::next(r_begin); begin != end; ++begin) {
    if (!comp(*r_begin, *begin)) {
      f(r_begin, begin);
      r_begin = begin;
    }
  }

  f(r_begin, end);
}

//////////////////////////////////////////////////////////////////////
/// @brief Apply a function on a range of iterators for which all of
///        the elements in that range are equal according to the
///        value extracted by a given extraction function.
///
///        This is useful as an alternative to for_each_range where the
///        comparison function is a heavyweight operation. With
///        for_each_range_by, the extractor function is guaranteed to be
///        invoked at most once per element in the input range.
///
/// @param begin The begin iterator
/// @param end The end iterator
/// @param f The function to apply to the range. It should accept exactly
///        three arguments: the begin and end iterator of the range and the
///        value for which the elements are equal that was extracted with
///        the extractor
/// @param extractor A function that is called on each element that provides
///        the value by which to compare. Comparison will happen using
///        std::equal_to
///        of the values returned by the extractor.
//////////////////////////////////////////////////////////////////////
template <typename Iterator, typename F, typename Extractor>
void for_each_range_by(
  Iterator begin, Iterator end, F&& f, Extractor&& extractor)
{
  using comparison_type = decltype(std::forward<Extractor>(extractor)(*begin));
  auto is_equal = std::equal_to<comparison_type>{};

  Iterator r_begin = begin;

  if (begin == end)
    return;

  auto running_value = std::forward<Extractor>(extractor)(*r_begin);

  for (begin = std::next(r_begin); begin != end; ++begin) {
    auto this_value = std::forward<Extractor>(extractor)(*begin);
    if (!is_equal(running_value, this_value)) {
      std::forward<F>(f)(r_begin, begin, running_value);
      r_begin = begin;
      running_value = std::move(this_value);
    }
  }

  std::forward<F>(f)(r_begin, end, running_value);
}

//////////////////////////////////////////////////////////////////////
/// @brief Apply a function on a range of iterators for which all of
///        the elements in that range are equal according to a comparator
///        and all of the elements satisfy a predicate.
///
/// @param begin The begin iterator
/// @param end The end iterator
/// @param f The function to apply to the range. It should accept exactly
///        two arguments: the begin and end iterator of the range
/// @param comp An equality comparison operator to compare elements
///        of the range
/// @param pred A unary predicate to determine whether or not an element
///        should be included in any range.
//////////////////////////////////////////////////////////////////////
template <typename Iterator, typename F, typename Comp, typename Pred>
void for_each_filtered_range(
  Iterator begin, Iterator end, F&& f, Comp&& comp, Pred&& pred)
{
  if (begin == end)
    return;

  do {
    Iterator r_begin = begin;

    // Advance until we find an element that satisfies the predicate
    while (r_begin != end && !pred(*r_begin))
      ++r_begin;

    if (r_begin == end)
      break;

    Iterator r_end = std::next(r_begin);

    // Advance until we find an element that doesn't match the start
    // of the range, or one that doesn't satisfy the predicate
    while (r_end != end && comp(*r_begin, *r_end) && pred(*r_end))
      ++r_end;

    // Now we have a range of equal values that match the predicate
    // so call the user function
    f(r_begin, r_end);

    begin = r_end;
  } while (begin != end);
}

} // namespace utility

} // namespace stapl

#endif // STAPL_UTILITY_FOR_EACH_RANGE
