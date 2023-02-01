/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_ALGORITHMS_NON_MODIFYING_HPP
#define STAPL_ALGORITHMS_NON_MODIFYING_HPP


#include <numeric>
#include <stapl/paragraph/paragraph.hpp>
#include "generator.hpp"
#include <stapl/utility/random.hpp>

#include <stapl/skeletons/map.hpp>
#include <stapl/skeletons/map_reduce.hpp>

#include <stapl/views/overlap_view.hpp>
#include <stapl/views/transform_view.hpp>

#include "algo_detail.hpp"

namespace stapl {

//
// Implementation classes used by multiple algorithms.
//

namespace algo_details {


//////////////////////////////////////////////////////////////////////
/// @brief Work function that performs the adjacent_find operation and
///        returns a reference to the last matching elements in the
///        range. This is done by iterating through the view's domain
///        in reverse order.
//////////////////////////////////////////////////////////////////////
template<typename Predicate>
class range_reverse_adjacent_find
{
private:
  Predicate m_predicate;

public:
  range_reverse_adjacent_find(Predicate predicate)
    : m_predicate(std::move(predicate))
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

    if (source_vw.size() > 1)
    {
      auto iter  = std::prev(source_vw.end(), 2);
      auto iter2 = std::prev(source_vw.end());

      for(; iter != source_vw.begin() ;std::advance(iter, -1),
                                       std::advance(iter2, -1))
      {
        if (m_predicate(*iter, *iter2))
        {
          break;
        }
      } 

      if (iter != source_vw.begin())
      {
        return *iter;
      }
      else if (iter == source_vw.begin() && m_predicate(*iter, *iter2))
      {
        return *iter;
      }
      else
        return result_type(null_reference()); 
    }
    else
    {
      return result_type(null_reference());
    }
  }

  void define_type(typer& t)
  { t.member(m_predicate); }
}; // class range_reverse_adjacent_find

//////////////////////////////////////////////////////////////////////
/// @brief Work function that performs the range_find operation and
///        returns a reference to the last matching element in the
///        range. This is done by iterating through the view in
///        reverse order.
//////////////////////////////////////////////////////////////////////
template<typename Predicate>
class range_reverse_find
{
private:
  Predicate m_predicate;

public:
  range_reverse_find(Predicate predicate)
    : m_predicate(std::move(predicate))
  { }

  template<typename View>
  typename View::reference
  operator()(View const& view) const
  {
    if (view.size() > 0)
    {
      auto iter = std::prev(view.end());
  
      for (; iter != view.begin(); std::advance(iter,-1))
      {
        if (m_predicate(*iter))
        {
          break;
        }
      }
 
      if (iter != view.begin())
        return *iter;
      else if (iter == view.begin() && m_predicate(*iter) ) 
        return *iter;
      else
        return typename View::reference(null_reference());
    }
    else
      return typename View::reference(null_reference());
      
  }

  void define_type(typer& t)
  { t.member(m_predicate); }
}; // class range_reverse_find


//
// all_of, none_of, any_of
//

//////////////////////////////////////////////////////////////////////
/// @brief Work function invokes sequential (i.e., STL) @p all_of.
/// @ingroup nonModifyingAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename Predicate>
class range_all_of
{
private:
  Predicate m_predicate;

public:
  range_all_of(Predicate predicate)
    : m_predicate(std::move(predicate))
  { }

  template<typename View>
  bool operator()(View const& vw) const
  { return std::all_of(vw.begin(), vw.end(), m_predicate); }

  void define_type(typer& t)
  { t.member(m_predicate); }
}; // class range_all_of


//////////////////////////////////////////////////////////////////////
/// @brief Work function invokes sequential (i.e., STL) @p none_of.
/// @ingroup nonModifyingAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename Predicate>
class range_none_of
{
private:
  Predicate m_predicate;

public:
  range_none_of(Predicate predicate)
    : m_predicate(std::move(predicate))
  { }

  template<typename View>
  bool operator()(View const& vw) const
  { return std::none_of(vw.begin(), vw.end(), m_predicate); }

  void define_type(typer& t)
  { t.member(m_predicate); }
}; // class range_none_of


//////////////////////////////////////////////////////////////////////
/// @brief Work function invokes sequential (i.e., STL) @p any_of.
/// @ingroup nonModifyingAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename Predicate>
class range_any_of
{
private:
  Predicate m_predicate;

public:
  range_any_of(Predicate predicate)
    : m_predicate(std::move(predicate))
  { }

  template<typename View>
  bool operator()(View const& vw) const
  { return std::any_of(vw.begin(), vw.end(), m_predicate); }

  void define_type(typer& t)
  { t.member(m_predicate); }
}; // class range_any_of

} // namespace algo_details


//////////////////////////////////////////////////////////////////////
/// @brief Returns true if the given predicate returns true for all of the
///   elements in the input view.
/// @param view One-dimensional view of the input.
/// @param predicate Unary functor which is called on the input elements.
/// @return True if the functor returns true for all elements, false otherwise.
/// @ingroup nonModifyingAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename View0, typename Predicate>
bool all_of(View0 const& view, Predicate predicate)
{
  return map_reduce<skeletons::tags::with_coarsened_wf>(
    algo_details::range_all_of<Predicate>(std::move(predicate)),
    stapl::logical_and<bool>(),
    view);
}


//////////////////////////////////////////////////////////////////////
/// @brief Returns true if the given predicate returns false for all of the
///   elements in the input view, or the view is empty.
/// @param view One-dimensional view of the input.
/// @param predicate Unary functor which is called on the input elements.
/// @return True if the functor returns false for all elements, false otherwise.
/// @ingroup nonModifyingAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename View0, typename Predicate>
bool none_of(View0 const& view, Predicate predicate)
{
  return map_reduce<skeletons::tags::with_coarsened_wf>(
    algo_details::range_none_of<Predicate>(std::move(predicate)),
    stapl::logical_and<bool>(),
    view);
}


//////////////////////////////////////////////////////////////////////
/// @brief Returns true if the given predicate returns true for any of the
///   elements in the input view.
/// @param view One-dimensional view of the input.
/// @param predicate Unary functor which is called on the input elements.
/// @return True if the functor returns true for any element, false otherwise.
/// @ingroup nonModifyingAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename View0, typename Predicate>
bool any_of(View0 const& view, Predicate predicate)
{
  return map_reduce<skeletons::tags::with_coarsened_wf>(
    algo_details::range_any_of<Predicate>(std::move(predicate)),
    stapl::logical_or<bool>(),
    view);
}


//
// count_if,  count
//

namespace algo_details {

//////////////////////////////////////////////////////////////////////
/// @brief Work function invokes sequential (i.e., STL) @p count_if.
/// @ingroup nonModifyingAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename Predicate>
class range_count_if
{
private:
  Predicate m_predicate;

public:
  range_count_if(Predicate predicate)
    : m_predicate(std::move(predicate))
  { }

  template<typename View>
  typename View::size_type
  operator()(View const& vw) const
  { return std::count_if(vw.begin(), vw.end(), m_predicate); }

  void define_type(typer& t)
  { t.member(m_predicate); }
}; // class range_count_if


} // namespace algo_details


//////////////////////////////////////////////////////////////////////
/// @brief Computes the number of elements in the input view for which the
///   given functor returns true.
/// @param view One-dimensional view over the input elements.
/// @param predicate Functor which is used to test the elements for counting.
/// @return The number of elements for which the functor returns true.
/// @ingroup nonModifyingAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename View, typename Predicate>
typename View::size_type
count_if(View const& view, Predicate predicate)
{
  return map_reduce<skeletons::tags::with_coarsened_wf>(
    algo_details::range_count_if<Predicate>(std::move(predicate)),
    stapl::plus<typename View::size_type>(),
    view);
}


//////////////////////////////////////////////////////////////////////
/// @brief Computes the number of elements in the input view which compare
///   equal to the given value.
/// @param view One-dimensional view over the input elements.
/// @param value Value to count the occurrences of in the input.
/// @return The number of occurrences of the given element in the input.
/// @ingroup nonModifyingAlgorithms
///
/// @todo Track down why std::bind2nd seems to be seeping into
/// this namespace (icc).
//////////////////////////////////////////////////////////////////////
template<typename View, typename T>
typename View::size_type
count(View const& view, T const& value)
{
  return count_if(view, stapl::bind2nd(equal_to<T>(), value));
}


//
// equal, mismatch
//
namespace algo_details {

//////////////////////////////////////////////////////////////////////
/// @brief Work function invokes sequential (i.e., STL) @p equal.
/// @ingroup nonModifyingAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename Predicate>
class range_equal
{
private:
  Predicate m_predicate;

public:
  using result_type = bool;

  range_equal(Predicate predicate)
    : m_predicate(std::move(predicate))
  { }

  template<typename View0, typename View1>
  result_type operator()(View0 const& vw0, View1 const& vw1) const
  { return std::equal(vw0.begin(), vw0.end(), vw1.begin(), m_predicate); }

  void define_type(typer& t)
  { t.member(m_predicate); }
}; // class range_equal


//////////////////////////////////////////////////////////////////////
/// @brief Work function invokes sequential (i.e., STL) @p mismatch.
/// @ingroup nonModifyingAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename Predicate>
class range_mismatch
{
private:
  Predicate m_predicate;

public:
  range_mismatch(Predicate predicate)
    : m_predicate(std::move(predicate))
  { }

  template<typename View1, typename View2>
  typename View1::reference
  operator()(View1 const& view1, View2 const& view2) const
  {
    auto iter_pair =
      std::mismatch(view1.begin(), view1.end(), view2.begin(), m_predicate);

    return iter_pair.first != view1.end() ?
      *(iter_pair.first) : typename View1::reference(null_reference());
  }

  void define_type(typer& t)
  { t.member(m_predicate); }
};

} // namespace algo_details


//////////////////////////////////////////////////////////////////////
/// @brief Compares the two input views and returns true if all of their
///   elements compare pairwise equal.
/// @param view0 One-dimensional input view.
/// @param view1 One-dimensional input view.
/// @param predicate Binary functor which implements equality.
/// @return True if the elements in the views are equal, false otherwise.
/// @ingroup nonModifyingAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename View0, typename View1, typename Predicate>
bool equal(View0 const& view0, View1 const& view1, Predicate predicate)
{
  return map_reduce<skeletons::tags::with_coarsened_wf>(
    algo_details::range_equal<Predicate>(std::move(predicate)),
    logical_and<bool>(),
    view0, view1);
}


//////////////////////////////////////////////////////////////////////
/// @brief Compares the two input views and returns true if all of their
///   elements compare pairwise equal.
/// @param view0 One-dimensional input view.
/// @param view1 One-dimensional input view.
/// @ingroup nonModifyingAlgorithms
/// @ingroup summaryAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename View0, typename View1>
bool equal(View0 const& view0, View1 const& view1)
{
  return map_reduce<skeletons::tags::with_coarsened_wf>(
    algo_details::range_equal<equal_to<typename View0::value_type>>(
      equal_to<typename View0::value_type>()),
    logical_and<bool>(),
    view0, view1);
}


//////////////////////////////////////////////////////////////////////
/// @brief Given two input views, returns the positions of the first elements
///   which do not match.
/// @param view1 One-dimensional view of input.
/// @param view2 One-dimensional view of input.
/// @param predicate Functor that evaluates whether elements match.
/// @return A pair containing references to the first mismatched element in each
///   input view, or a pair of @ref null_reference instances if the views are
///   equal.
/// @ingroup nonModifyingAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename View1, typename View2, typename Predicate>
std::pair<typename View1::reference, typename View2::reference>
mismatch(View1 const& view1, View2 const& view2, Predicate predicate)
{
  using ref1_t = typename View1::reference;
  using ref2_t = typename View2::reference;

  ref1_t result =
    map_reduce<skeletons::tags::with_coarsened_wf>(
      algo_details::range_mismatch<Predicate>(std::move(predicate)),
      algo_details::find_reduce(),
      view1, view2);

  if (is_null_reference(result))
    return std::make_pair(ref1_t(null_reference()), ref2_t(null_reference()));
  else
    return std::make_pair(result, view2[index_of(result)]);
}


//////////////////////////////////////////////////////////////////////
/// @brief Given two input views, returns the positions of the first elements
///   which do not match.
/// @param view1 One-dimensional view of input.
/// @param view2 One-dimensional view of input.
/// @return A pair containing references to the first mismatched element in each
///   input view, or a pair of @ref null_reference instances if the views are
///   equal.
/// @ingroup nonModifyingAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename View1, typename View2>
std::pair<typename View1::reference, typename View2::reference>
mismatch(View1 const& view1, View2 const& view2)
{
  return mismatch(view1, view2, equal_to<typename View1::reference>());
}


//
// find, find_if, find_if_not, adjacent_find
//
namespace algo_details {

//////////////////////////////////////////////////////////////////////
/// @brief Work function invokes sequential (i.e., STL) @p find.
/// @ingroup nonModifyingAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename Predicate>
class range_find
{
private:
  Predicate m_predicate;

public:
  range_find(Predicate predicate)
    : m_predicate(std::move(predicate))
  { }

  template<typename View>
  typename View::reference
  operator()(View const& view) const
  {
    auto iter = std::find_if(view.begin(), view.end(), m_predicate);

    return iter != view.end() ?
      *iter : typename View::reference(null_reference());
  }

  void define_type(typer& t)
  { t.member(m_predicate); }
}; // class range_find


} // namespace algo_details


//////////////////////////////////////////////////////////////////////
/// @brief Finds the first element in the input for which the predicate returns
///   true, or null_reference if none exist.
/// @param view One-dimensional view of the input.
/// @param predicate Functor used to test the elements.
/// @return A reference to the first element found, or a
///         @ref null_reference if none found.
/// @ingroup nonModifyingAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename View, typename Predicate>
typename View::reference
find_if(View const& view, Predicate predicate)
{
  return map_reduce<skeletons::tags::with_coarsened_wf>(
    algo_details::range_find<Predicate>(std::move(predicate)),
    algo_details::find_reduce(),
    view);
}


//////////////////////////////////////////////////////////////////////
/// @brief Finds the first element in the input for which the predicate returns
///   false, or null_reference if none exist.
/// @param view One-dimensional view of the input.
/// @param predicate Functor used to test the elements.
/// @return A reference to the first element found, or a
///         @ref null_reference if none found.
/// @ingroup nonModifyingAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename View, typename Predicate>
typename View::reference
find_if_not(View const& view, Predicate predicate)
{
  using negate_t = stapl::unary_negate<typename std::decay<Predicate>::type>;

  return find_if(view, negate_t(std::move(predicate)));
}


//////////////////////////////////////////////////////////////////////
/// @brief Finds the first occurrence of the given value in the input,
///   null_reference if it is not found.
/// @param view One-dimensional view of the input.
/// @param value Value to find.
/// @return A reference to the first element found, or a
///         @ref null_reference if none found.
/// @ingroup nonModifyingAlgorithms
///
/// @todo Track down why std::bind2nd seems to be seeping into
/// this namespace (icc).
//////////////////////////////////////////////////////////////////////
template<typename View, typename T>
typename View::reference
find(View const& view, T const& value)
{
  return find_if(view, stapl::bind2nd(equal_to<T>(), value));
}


//////////////////////////////////////////////////////////////////////
/// @brief Return the position of the first adjacent pair of equal elements.
/// @param view One-dimensional view of the input elements.
/// @param predicate Predicate functor which implements the equal operation.
/// @return A reference to the first element of the pair, or a
///         @ref null_reference if none exist.
/// @ingroup nonModifyingAlgorithms
///
/// @todo The range-based kernel assumes that the domain of the input view
///       is contiguous. Investigate the behavior with enumerated domains like
///       @ref domset1D.
//////////////////////////////////////////////////////////////////////
template<typename View, typename Predicate>
typename View::reference
adjacent_find(View const& view, Predicate predicate)
{
  stapl_assert(view.domain().is_contiguous(), "adjacent_find for views"
    " with non-contiguous domains is currently not supported");

  return map_reduce<skeletons::tags::with_coarsened_wf>(
    algo_details::range_adjacent_find<Predicate>(std::move(predicate)),
    algo_details::find_reduce(),
    make_overlap_view(view, 1, 0, 1));
}


//////////////////////////////////////////////////////////////////////
/// @brief Return the position of the first adjacent pair of equal elements.
/// @param view One-dimensional view of the input elements.
/// @return A reference to the first element of the pair, or a
///         @ref null_reference if none exist.
/// @ingroup nonModifyingAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename View>
typename View::reference
adjacent_find(View const& view)
{
  return adjacent_find(view, stapl::equal_to<typename View::value_type>());
}


//
// find_end, find_first_of, search, search_n
//

namespace algo_details {

//////////////////////////////////////////////////////////////////////
/// @brief Work function invokes sequential (i.e., STL) @p find_end.
/// @sa range_adjacent_find
/// @ingroup nonModifyingAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename Predicate>
class range_find_end
{
private:
  Predicate m_predicate;

public:
  range_find_end(Predicate predicate)
    : m_predicate(std::move(predicate))
  { }

  template <typename View0, typename View1>
  typename View0::reference::reference
  operator()(View0 const& source, View1 const& search_str)
  {
    auto search_vw = *search_str.begin();
    auto source_vw = source.container().view();

    using domain_type = typename decltype(source_vw)::domain_type;
    using result_type = typename View0::reference::reference;

    if (source.domain().last() == source_vw.domain().last()) {
      if (source.domain().size() < search_vw.domain().size())
        return result_type(null_reference());

      source_vw.set_domain(
        domain_type(source.domain().first(), source.domain().last()));
    }
    else {
      source_vw.set_domain(
      domain_type(
        source.domain().first(),
        source.domain().last() + search_vw.domain().size()-1));
    }

    auto iter = std::find_end(source_vw.begin(), source_vw.end(),
                              search_vw.begin(), search_vw.end());

    return iter == source_vw.end() ? result_type(null_reference()) : *iter;
  }

  void define_type(typer& t)
  { t.member(m_predicate); }
}; // class range_find_end


//////////////////////////////////////////////////////////////////////
/// @brief Work function invokes sequential (i.e., STL) @p find_first_of.
/// @ingroup nonModifyingAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename Predicate>
class range_find_first_of
{
private:
  Predicate m_predicate;

public:
  range_find_first_of(Predicate predicate)
    : m_predicate(std::move(predicate))
  { }

  template<typename View0, typename View1>
  typename View0::reference
  operator()(View0 const& view0, View1 const& view1) const
  {
    auto search_vw = *view1.begin();
    auto iter = std::find_first_of(view0.begin(), view0.end(),
                                   search_vw.begin(), search_vw.end(),
                                   m_predicate);

    return iter != view0.end() ?
      *iter : typename View0::reference(null_reference());
  }

  void define_type(typer& t)
  { t.member(m_predicate); }
}; // class range_find_first_of


//////////////////////////////////////////////////////////////////////
/// @brief Work function invokes sequential (i.e., STL) @p search.
/// @tparam Predicate Functor which is used to compare the arguments for
///   equality.
/// @sa range_adjacent_find
/// @ingroup nonModifyingAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename Predicate>
class range_search
{
private:
  Predicate m_binary_pred;

public:
  range_search(Predicate binary_pred)
    : m_binary_pred(std::move(binary_pred))
  { }

  template<typename View0, typename View1>
  typename View0::reference::reference
  operator()(View0 const& source, View1 const& search_str) const
  {
    auto search_vw = *search_str.begin();
    auto source_vw = source.container().view();

    using domain_type = typename decltype(source_vw)::domain_type;
    using result_type = typename View0::reference::reference;

    if (source.domain().last() == source_vw.domain().last()) {
      if (source.domain().size() < search_vw.domain().size())
        return result_type(null_reference());

      source_vw.set_domain(
        domain_type(source.domain().first(), source.domain().last()));
    }
    else {
      source_vw.set_domain(
      domain_type(
        source.domain().first(),
        source.domain().last() + search_vw.domain().size()-1));
    }

    auto iter = std::search(source_vw.begin(), source_vw.end(),
                            search_vw.begin(), search_vw.end());

    return iter == source_vw.end() ? result_type(null_reference()) : *iter;
  }

  void define_type(typer& t)
  { t.member(m_binary_pred); }
}; // class range_search


//////////////////////////////////////////////////////////////////////
/// @brief Work function invokes sequential (i.e., STL) @p search_n.
/// Returns information about matches found in the range as well as
/// partial matches found at the front and back of the range.
/// @tparam T Type of the value which is being searched for.
/// @tparam Predicate Functor which is used to compare element equality.
/// @ingroup nonModifyingAlgorithms
/// @note A more sophisticated algorithm could synthesize the front and
///   back partial discovery into the main search_n call,
///   at the expense of not being able to use @p std::search_n.
/// @ingroup nonModifyingAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename T, typename Predicate>
class range_search_n
{
private:
  size_t       m_count;
  T            m_value;
  Predicate    m_predicate;

public:
  range_search_n(size_t count, T value, const Predicate predicate)
    : m_count(count),
      m_value(std::move(value)),
      m_predicate(std::move(predicate))
  { }

  template<typename View>
  tuple<size_t,
        bool, typename View::index_type,
        typename View::index_type, size_t>
  operator()(View const& v) const
  {
    using index_type = typename View::index_type;

    //
    // Find any partial match at front of range.
    //
    size_t front_count     = 0;
    auto iter              = v.begin();
    const size_t front_end = std::min(v.size(), m_count-1);

    for (size_t idx = 0; idx < front_end; ++idx, ++front_count, ++iter)
      if (!m_predicate(*iter, m_value))
        break;

    //
    // Find first (if any) complete match in range.
    //
    auto contained_match_iter =
      std::search_n(v.begin(), v.end(), m_count, m_value);

    bool b_contained_found = false;
    index_type contained_index;

    if (contained_match_iter != v.end())
    {
      b_contained_found = true;
      contained_index   = index_of(*contained_match_iter);
    }

    //
    // Find any partial match at back of range.
    //
    size_t end_count = 0;
    iter             = std::prev(v.end());

    for (size_t idx = 0; idx < front_end; ++idx, --iter, ++end_count)
      if (!m_predicate(*iter, m_value))
        break;

    ++iter;

    index_type end_match_index;

    if (iter != v.end())
      end_match_index = index_of(*iter);

    //
    // Return summary of full matches internally found and partial matches on
    // front and back or range.
    //
    return tuple<size_t, bool, index_type, index_type, size_t>
      (front_count,
       b_contained_found, contained_index,
       end_match_index, end_count);
  }

  void define_type(typer& t)
  {
    t.member(m_count);
    t.member(m_value);
    t.member(m_predicate);
  }
}; // class range_search_n


//////////////////////////////////////////////////////////////////////
/// @brief Reduction work function for @ref search_n which synthesizes
/// the the lhs and rhs full and partial match information.
//////////////////////////////////////////////////////////////////////
template<typename Index>
class search_n_reduce_wf
{
private:
  size_t m_count;

public:
  using result_type = tuple<size_t, bool, Index, Index, size_t>;

  search_n_reduce_wf(size_t count)
    : m_count(count)
  { }

  template<typename Reference1, typename Reference2>
  result_type operator()(Reference1&& lhs_ref, Reference2&& rhs_ref) const
  {
    result_type lhs = lhs_ref;
    result_type rhs = rhs_ref;

    // If there's a match on the left, just return left.
    if (get<1>(lhs))
      return lhs;

    // If there's a match that starts on left and ends on right,
    // construct synthesized result.
    if (get<4>(lhs) + get<0>(rhs) >= m_count)
      return result_type(
        get<0>(lhs), true, get<2>(lhs), get<3>(rhs), get<4>(rhs));

    // Check if entirety of lhs is a partial match.
    // If so, include rhs front in return value.
    size_t front_count =
      get<0>(lhs) == get<4>(lhs) ?  get<0>(lhs) + get<0>(rhs) : get<0>(lhs);

    // If there's a match on the right, return it and front partial of lhs.
    if (get<1>(rhs))
      return result_type(
        front_count, true, get<2>(rhs), get<3>(rhs), get<4>(rhs));

    //
    // No full matches, use synthesized front and back counts.
    //

    // Check if entirety of rhs is a partial match.
    // If so, include lhs back in return value.
    size_t back_count = get<4>(rhs);
    Index  back_index = get<3>(rhs);

    if (get<0>(rhs) == get<4>(rhs))
    {
      back_count += get<4>(lhs);
      back_index = get<3>(lhs);
    }

    return result_type(front_count, false, get<2>(lhs), back_index, back_count);
  }

  void define_type(typer& t)
  { t.member(m_count); }
};

} // namespace algo_details


//////////////////////////////////////////////////////////////////////
/// @brief Finds the last occurrence of the given pattern in the input sequence.
/// @param sequence A one-dimensional view of the input.
/// @param pattern The pattern to search for in the input.
/// @param predicate Functor which implements the equal operation.
/// @return A reference to the first element of the last occurrence of the
///   pattern in the input, or a @ref null_reference if the pattern is not
///   found.
/// @ingroup nonModifyingAlgorithms
///
/// @todo The range-based kernel assumes that the domain of the input view
///   is contiguous. Investigate the behavior with enumerated domains like
///   @ref domset1D.
//////////////////////////////////////////////////////////////////////
template<typename View1, typename View2, typename Predicate>
typename View1::reference
find_end(View1 const& sequence, View2 const& pattern, Predicate predicate)
{
  stapl_assert(sequence.domain().is_contiguous(), "find_end for views"
    " with non-contiguous domains is currently not supported");

  if (pattern.size() == 0 || pattern.size() > sequence.size())
    return typename View1::reference(null_reference());

  return stapl::map_reduce<skeletons::tags::with_coarsened_wf>(
    algo_details::range_find_end<Predicate>(std::move(predicate)),
    algo_details::find_end_reduce(),
    make_overlap_view(sequence, 1, 0, pattern.size()-1),
    make_repeat_ro_view(pattern));
}


//////////////////////////////////////////////////////////////////////
/// @brief Finds the last occurrence of the given pattern in the input sequence.
/// @param sequence A one-dimensional view of the input.
/// @param pattern The pattern to search for in the input.
/// @return A reference to the first element of the last occurrence of the
///   pattern in the input, or a @ref null_reference if the pattern is not
///   found.
/// @ingroup nonModifyingAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename View1, typename View2>
typename View1::reference
find_end(View1 const& sequence, View2 const& pattern)
{
  return find_end(sequence, pattern, equal_to<typename View1::reference>());
}


//////////////////////////////////////////////////////////////////////
/// @brief Finds the first element in the input which matches any of the
///   elements in the given view, according to the given predicate.
/// @param view0 One-dimensional view of the input.
/// @param view1 One-dimensional view of the elements to find.
/// @param predicate Binary functor which implements the less operation.
/// @return A reference to the first element that matches, or a
///   @ref null_reference otherwise.
/// @ingroup nonModifyingAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename View0, typename View1, typename Predicate>
typename View0::reference
find_first_of(View0 const& view0, View1 const& view1, Predicate predicate)
{
  return map_reduce<skeletons::tags::with_coarsened_wf>(
    algo_details::range_find_first_of<Predicate>(std::move(predicate)),
    algo_details::find_reduce(),
    view0, make_repeat_ro_view(view1));
}


//////////////////////////////////////////////////////////////////////
/// @brief Finds the first element in the input which matches any of the
///   elements in the given view.
/// @param view0 One-dimensional view of the input.
/// @param view1 One-dimensional view of the elements to find.
/// @return A reference to the first element that matches, or a
///   @ref null_reference otherwise.
/// @ingroup nonModifyingAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename View0, typename View1>
typename View0::reference
find_first_of(View0 const& view0, View1 const& view1)
{
  return find_first_of(view0, view1, equal_to<typename View0::reference>());
}

//////////////////////////////////////////////////////////////////////
/// @brief Return the position of the first occurrence of the given
///   search sequence within the input, or a @ref null_reference instance
///   if it is not found.
/// @param v1 One-dimensional view of the input elements.
/// @param v2 One-dimensional view of the search sequence.
/// @param predicate Functor that evaluates whether elements are equal.
/// @return A reference to the first element of the pair, or
///   null_reference if none exist.
/// @ingroup nonModifyingAlgorithms
///
/// @todo The range-based kernel assumes that the domain of the input view
///   is contiguous. Investigate the behavior with enumerated domains like
///   @ref domset1D.
//////////////////////////////////////////////////////////////////////
template<typename View1, typename View2, typename Predicate>
typename View1::reference
search(View1 const& v1, View2 const& v2, Predicate predicate)
{
  stapl_assert(v1.domain().is_contiguous(), "search for views"
    " with non-contiguous domains is currently not supported");

  if (v2.size() == 0 || v2.size() > v1.size())
    return typename View1::reference(null_reference());

  return map_reduce<skeletons::tags::with_coarsened_wf>(
    algo_details::range_search<Predicate>(std::move(predicate)),
    algo_details::find_reduce(),
    make_overlap_view(v1, 1, 0, v2.size()-1),
    make_repeat_ro_view(v2));
}


//////////////////////////////////////////////////////////////////////
/// @brief Return the position of the first occurrence of the given sequence
///   within the input, or a @ref null_reference instance if the sequence is
///   not found.
/// @param v1 One-dimensional view of the input elements.
/// @param v2 One-dimensional view of the search sequence.
/// @return A reference to the first element of the pair, or
///   null_reference if none exist.
/// @ingroup nonModifyingAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename View1, typename View2>
typename View1::reference
search(View1 const& v1, View2 const& v2)
{
  return search(v1, v2, stapl::equal_to<typename View1::value_type>());
}


//////////////////////////////////////////////////////////////////////
/// @brief Return the position of the first occurrence of a sequence of the
///   given value which is of the given length, or a @ref null_reference
///   instance if it is not found.
///
/// @param v One-dimensional view of input elements.
/// @param count Length of sequence of repeated elements to find.
/// @param value Value which is repeated in the searched sequence.
/// @param predicate Functor which implements the equality operation.
/// @return A reference to the first element of the repeated sequence in the
///   input, or a @ref null_reference instance if no sequence is found.
/// @ingroup nonModifyingAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename View, typename Predicate>
typename View::reference
search_n(View const& v,
         size_t count,
         typename View::value_type value,
         Predicate predicate)
{
  if (count == 0)
    return *v.begin();

  using index_type = typename View::index_type;
  using reference  = typename View::reference;
  using value_type = typename View::value_type;

  tuple<size_t, bool, index_type, index_type, size_t> ret_val =
    map_reduce<skeletons::tags::with_coarsened_wf>(
      algo_details::range_search_n<value_type, Predicate>(
        count, std::move(value), std::move(predicate)),
      algo_details::search_n_reduce_wf<index_type>(count),
      v);

  return get<1>(ret_val) ? v[get<2>(ret_val)] : reference(null_reference());
}


//////////////////////////////////////////////////////////////////////
/// @brief Return the position of the first occurrence of a sequence
///   of the given value which is of the given length, or null_reference
///   if none exists.
/// @param v1 One-dimensional view of input elements.
/// @param count Length of sequence of repeated elements to search.
/// @param value Value which is repeated in the searched sequence.
/// @return A reference to the first element of the repeated sequence in the
///   input, or null_reference if none exists.
/// @ingroup nonModifyingAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename View1>
typename View1::reference
search_n(View1 const& v1, size_t count, typename View1::value_type value)
{
  return search_n(
    v1, count, std::move(value), stapl::equal_to<typename View1::value_type>());
}


namespace algo_details{

//////////////////////////////////////////////////////////////////////
/// @brief Work function which increments the value in the input map for the
///   given key.
/// @tparam T Type of the key.
//////////////////////////////////////////////////////////////////////
template<typename T>
struct map_inc
{
  typedef T      first_argument_type;
  typedef void   result_type;

private:
  first_argument_type m_key;

public:
  map_inc(const T& key) : m_key(key) { }

  template<typename Reference>
  void operator()(Reference& y) const
  {
    y[m_key]++;
  }

  void define_type(typer& t)
  {
    t.member(m_key);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function which creates a histogram from
///   from the given input.
/// @see next_permutation
/// @see prev_permutation
/// @see is_permutation
//////////////////////////////////////////////////////////////////////
template<class T>
class histogram
{
public:
  typedef void result_type;

  histogram(static_array<std::unordered_map<T,int> >* samples, int min, int step)
    : m_samples(samples), m_min(min), m_step(step) {}

  template<class Elem>
  void operator()(Elem elem)
  {
    m_samples->apply_set((stapl::hash<T>()(elem)-m_min)/m_step,
                         map_inc<int>(elem));
  }

  void define_type(typer& t)
  {
    t.member(m_samples);
    t.member(m_min);
    t.member(m_step);
  }
private:
  static_array<std::unordered_map<T,int> >* m_samples;
  int m_min;
  int m_step;
};

} // namespace algo_details


//////////////////////////////////////////////////////////////////////
/// @brief Computes whether all the elements in the first view are contained in
///   the second view, even in a different order.
/// @param view1 One-dimensional input view.
/// @param view2 One-dimensional input view.
/// @param pred Functor which implements the equal operation.
/// @return True if the second view is a permutation of the first, else false.
/// @ingroup summaryAlgorithms
///
/// This algorithm is non-mutating.
//////////////////////////////////////////////////////////////////////
template<typename View1, typename View2, typename Pred>
bool is_permutation(View1 const& view1, View2 const& view2, Pred pred)
{
  typedef typename View1::value_type                  value_type;
  typedef std::unordered_map<value_type, int>         map_type;
  typedef static_array<map_type>                      sample_container_type;
  typedef array_view<sample_container_type>           sample_view_type;
  typedef stapl::hash<value_type>                     hash_type;

  sample_container_type samples1(get_num_locations());
  sample_container_type samples2(get_num_locations());
  sample_view_type samples_view1(samples1);
  sample_view_type samples_view2(samples2);

  std::pair<value_type,value_type> p1 =
    minmax_value(transform_view<View1, hash_type>(view1, hash_type()));
  std::pair<value_type,value_type> p2 =
    minmax_value(transform_view<View2, hash_type>(view2, hash_type()));

  if ((p1.first!=p2.first) || (p1.second!=p2.second))
    return false;

  int step = ((p1.second - p1.first) / get_num_locations()) + 1;

  map_func(algo_details::histogram<value_type>(&samples1,p1.first,step),view1);
  map_func(algo_details::histogram<value_type>(&samples2,p1.first,step),view2);

  return stapl::equal(samples_view1, samples_view2);
}


//////////////////////////////////////////////////////////////////////
/// @brief Computes whether all the elements in the first view are contained in
///   the second view, even in a different order.
/// @param view1 One-dimensional input view.
/// @param view2 One-dimensional input view.
/// @return True if the second view is a permutation of the first, else false.
/// @ingroup summaryAlgorithms
///
/// This version calls the predicated version with a default predicate of
/// stapl::equal_to.
//////////////////////////////////////////////////////////////////////
template<typename View1, typename View2>
bool is_permutation(View1 const& view1, View2 const& view2)
{
  return is_permutation(
    view1, view2, stapl::equal_to<typename View1::value_type>());
}


} // namespace stapl
#endif

