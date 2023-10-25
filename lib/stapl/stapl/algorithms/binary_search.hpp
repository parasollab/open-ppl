/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_ALGORITHMS_BINARY_SEARCH_HPP
#define STAPL_ALGORITHMS_BINARY_SEARCH_HPP

namespace stapl {

//
// lower_bound, upper_bound, binary_search, equal_range
//

namespace algo_details {

//////////////////////////////////////////////////////////////////////
/// @brief Work function for @ref lower_bound(), which takes a view over
///   a sorted input, and computes the std::lower_bound on it, or
///  @ref null_reference if the range is all less than the given value.
/// @tparam Value Type of value being searched.
/// @tparam StrictWeakOrdering Binary functor which implements the less
///   operation.
/// @ingroup binarysearchAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename Value, typename StrictWeakOrdering>
class range_lower_bound
{
private:
  Value              m_value;
  StrictWeakOrdering m_comparator;

public:
  range_lower_bound(Value value, StrictWeakOrdering comparator)
    : m_value(std::move(value)),
      m_comparator(std::move(comparator))
  { }

  template<typename View>
  typename View::reference
  operator()(View const& view) const
  {
    auto begin_ref = *view.begin();

    if (m_comparator(m_value, begin_ref))
      return begin_ref;

    auto end_ref = *std::prev(view.end());

    if (m_comparator(end_ref, m_value))
      return typename View::reference(null_reference());

    auto iter =
      std::lower_bound(view.begin(), view.end(), m_value, m_comparator);

    if (iter == view.end())
      return typename View::reference(null_reference());

    return *iter;
  }

  void define_type(typer& t)
  {
    t.member(m_value);
    t.member(m_comparator);
  }
}; // class range_lower_bound


//////////////////////////////////////////////////////////////////////
/// @brief Work function for @ref upper_bound(), which takes a view over
///   a sorted input, and computes the std::upper_bound on it, or
///   @ref null_reference if the range is all less than or equal to the
///   given value.
/// @tparam Value Type of value being searched.
/// @tparam StrictWeakOrdering Binary functor which implements the less
///   operation.
/// @ingroup binarysearchAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename Value, typename StrictWeakOrdering>
class upper_bound_map_wf
{
private:
  Value              m_value;
  StrictWeakOrdering m_comparator;

public:
  upper_bound_map_wf(Value value, StrictWeakOrdering comparator)
    : m_value(std::move(value)),
      m_comparator(std::move(comparator))
  { }

  template<typename View>
  typename View::reference
  operator()(View const& view) const
  {
    auto begin_ref = *view.begin();

    if (m_comparator(m_value, begin_ref))
      return begin_ref;

    auto end_ref = *std::prev(view.end());

    if (m_comparator(end_ref, m_value))
      return typename View::reference(null_reference());

    auto iter =
      std::upper_bound(view.begin(), view.end(), m_value, m_comparator);

    if (iter == view.end())
      return typename View::reference(null_reference());

    return *iter;
  }

  void define_type(typer& t)
  {
    t.member(m_value);
    t.member(m_comparator);
  }
}; // class upper_bound_map_wf


//////////////////////////////////////////////////////////////////////
/// @brief Work function which invokes @p std::binary_search.
/// @tparam Value Type of the searched value.
/// @tparam StrictWeakOrdering Binary functor which implements the less
///   operation.
/// @ingroup binarysearchAlgorithms
//////////////////////////////////////////////////////////////////////
template <typename Value, typename StrictWeakOrdering>
class binary_search_map_wf
{
private:
  Value              m_value;
  StrictWeakOrdering m_comparator;

public:
  binary_search_map_wf(Value value, StrictWeakOrdering comparator)
    : m_value(std::move(value)),
      m_comparator(std::move(comparator))
  { }

  template<typename View>
  bool operator()(View const& view) const
  {
    auto begin_ref = *view.begin();

    if (m_comparator(m_value, begin_ref))
      return false;

    auto end_ref = *std::prev(view.end());

    if (m_comparator(end_ref, m_value))
      return false;

    return std::binary_search(view.begin(), view.end(), m_value, m_comparator);
  }

  void define_type(typer& t)
  {
    t.member(m_value);
    t.member(m_comparator);
  }
}; // class binary_search_map_wf


//////////////////////////////////////////////////////////////////////
/// @brief Work function for @ref equal_range() which returns the position of
///   its argument if the argument compares equal to the given value according
///   to the given ordering, or @ref null_reference position otherwise.
/// @tparam Value Type of the elements being compared.
/// @tparam StrictWeakOrdering Binary functor which, given two values, will
///   return true if the first precedes the second. If the two values return
///   false for both orderings, then they are equal.
/// @tparam Return The type of the domain of the input view, used to construct
///   the range of equality for the output.
/// @ingroup binarysearchAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename Value, typename StrictWeakOrdering, typename Return>
class range_equal_range
{
private:
  Value              m_value;
  StrictWeakOrdering m_comparator;

public:
  range_equal_range(Value value, StrictWeakOrdering comparator)
    : m_value(std::move(value)),
      m_comparator(std::move(comparator))
  { }

  template<typename View>
  Return operator()(View const& view) const
  {
    auto range_pair =
      std::equal_range(view.begin(), view.end(), m_value, m_comparator);

    if (range_pair.first == view.end())
     return Return();

    return Return(index_of(*range_pair.first), index_of(*range_pair.second));
  }

  void define_type(typer& t)
  {
    t.member(m_value);
    t.member(m_comparator);
  }
}; // class range_equal_range


//////////////////////////////////////////////////////////////////////
/// @brief Work function for @ref equal_range() which, given two domains,
///   returns the one which is non-empty, or merges them together.
/// @tparam Return The type of the domains to merge.
/// @ingroup binarysearchAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename Return>
struct equal_range_reduce_wf
{
  Return operator()(Return lhs, Return rhs) const
  {
    if (lhs.empty())
      return rhs;

    if (rhs.empty())
      return lhs;

    return Return(lhs.first(), rhs.last());
  }
};

} // namespace algo_details


//////////////////////////////////////////////////////////////////////
/// @brief Finds the first element in the input view which compares greater than
///   or equal to the given value.
/// @param view One-dimensional view of the input (which is sorted).
/// @param value Value to search for in the input (of same type as inputs).
/// @param comparator Binary functor which implements the less than operation.
/// @return A reference to the first value in the input which is greater than
///   or equal to the given value.
/// @ingroup binarysearchAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename View, typename T, typename StrictWeakOrdering>
typename View::reference
lower_bound(View const& view, T value, StrictWeakOrdering comparator)
{
  return map_reduce<skeletons::tags::with_coarsened_wf>(
    algo_details::range_lower_bound<
      typename View::value_type, StrictWeakOrdering>(
        std::move(value), std::move(comparator)),
    algo_details::find_reduce(),
    view);
}


//////////////////////////////////////////////////////////////////////
/// @brief Finds the first element in the input view which compares greater than
///   or equal to the given value.
/// @param view One-dimensional view of the input (which is sorted).
/// @param value Value to search for in the input (of same type as inputs).
/// @return A reference to the first value in the input which is greater than
///   or equal to the given value.
/// @ingroup binarysearchAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename View, typename T>
typename View::reference
lower_bound(View const& view, T const& value)
{
  return lower_bound(
    view, value, stapl::less<typename View::value_type>());
}


//////////////////////////////////////////////////////////////////////
/// @brief Finds the first element in the input view which compares greater than
///   the given value.
/// @param view One-dimensional view of the input (which is sorted).
/// @param value Value to search for in the input (of same type as inputs).
/// @param comparator Binary functor which implements the less than operation.
/// @return A reference to the first value in the input which is greater than
///   the given value.
/// @ingroup binarysearchAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename View, typename T, typename StrictWeakOrdering>
typename View::reference
upper_bound(View const& view, T value, StrictWeakOrdering comparator)
{
  return map_reduce<skeletons::tags::with_coarsened_wf>(
    algo_details::upper_bound_map_wf<
      typename View::value_type, StrictWeakOrdering>(
        std::move(value), std::move(comparator)),
    algo_details::find_reduce(),
    view);
}


//////////////////////////////////////////////////////////////////////
/// @brief Finds the first element in the input view which compares greater than
///   the given value.
/// @param view One-dimensional view of the input (which is sorted).
/// @param value Value to search for in the input (of same type as inputs).
/// @return A reference to the first value in the input which is greater than
///   the given value.
/// @ingroup binarysearchAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename View, typename T>
typename View::reference
upper_bound(View const& view, T const& value)
{
  return upper_bound(view, value, stapl::less<typename View::value_type>());
}


//////////////////////////////////////////////////////////////////////
/// @brief Searches the input view for the given value using a binary
///   search, and returns true if that value exists in the input.
/// @param view One-dimensional view of the input.
/// @param value Value to search for in the input (of same type as inputs).
/// @param comparator Binary functor used to compare elements.
/// @return True if the value is found in the input, false otherwise.
/// @ingroup binarysearchAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename View, typename StrictWeakOrdering>
bool
binary_search(View const& view,
              typename View::value_type value,
              StrictWeakOrdering comparator)
{
  return map_reduce<skeletons::tags::with_coarsened_wf>(
    algo_details::binary_search_map_wf<
      typename View::value_type, StrictWeakOrdering>(
      std::move(value), std::move(comparator)),
    stapl::logical_or<bool>(),
    view);
}


//////////////////////////////////////////////////////////////////////
/// @brief Searches the input view for the given value using a binary
///   search, and returns true if that value exists in the input.
/// @param view One-dimensional view of the input.
/// @param value Value to search for in the input (of same type as inputs).
/// @return True if the value is found in the input, false otherwise.
/// @ingroup binarysearchAlgorithms
//////////////////////////////////////////////////////////////////////
template < typename View >
bool
binary_search(View const& view, typename View::value_type const& value)
{
  return binary_search(view, value, stapl::less<typename View::value_type>());
}


//////////////////////////////////////////////////////////////////////
/// @brief Computes the range of elements which are equal to the given value.
/// @param view One-dimensional view of input (which is sorted).
/// @param value Value to search for in the input (of same type as inputs).
/// @param comparator Functor which provides ordering operation.
/// @return A view over the elements which compare equal to the given value.
/// @ingroup binarysearchAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename View, typename StrictWeakOrdering>
View
equal_range(View const& view,
            typename View::value_type value,
            StrictWeakOrdering comparator)
{
  using domain_type = typename View::domain_type;

  domain_type res =
    map_reduce<skeletons::tags::with_coarsened_wf>(
      algo_details::range_equal_range<
        typename View::value_type, StrictWeakOrdering, domain_type>(
          std::move(value), std::move(comparator)),
      algo_details::equal_range_reduce_wf<domain_type>(),
      view);

  return View(view.container(), res, view.mapfunc());
}


//////////////////////////////////////////////////////////////////////
/// @brief Computes the range of elements which are equal to the given value.
/// @param view One-dimensional view of input (which is sorted).
/// @param value Value to search for in the input (of same type as inputs).
/// @return A view over the elements which compare equal to the given value.
/// @ingroup binarysearchAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename View>
View
equal_range(View const& view, typename View::value_type const& value)
{
  return equal_range(view, value, stapl::less<typename View::value_type>());
}

} // namespace stapl
#endif

