/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_ALGORITHMS_MINMAX_HPP
#define STAPL_ALGORITHMS_MINMAX_HPP

//
// max_element, max_value
//

namespace stapl {

namespace algo_details {

//////////////////////////////////////////////////////////////////////
/// @brief Work function invokes sequential (i.e., STL) @p max_element,
/// returning the index and value of the corresponding element.
/// @ingroup extremaAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename Comparator>
class range_max_element
{
private:
  Comparator m_comparator;

public:
  range_max_element(Comparator comparator)
    : m_comparator(std::move(comparator))
  { }

  template<typename View>
  std::pair<typename View::index_type, typename View::value_type>
  operator()(View const& vw) const
  {
    auto ref = *std::max_element(vw.begin(), vw.end(), m_comparator);

    return std::pair<typename View::index_type, typename View::value_type>(
      index_of(ref), ref);
  }

  void define_type(typer& t)
  { t.member(m_comparator); }
}; // class range_max_element


//////////////////////////////////////////////////////////////////////
/// @brief Work function which reduces two pairs of index and values,
/// selecting the pair with maximum value.
/// @tparam T Pair type which is reduced.
/// @tparam Comparator Comparison function which is called to determine the max.
/// @ingroup extremaAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename T, typename Comparator>
struct max_element_reduce
{
private:
  Comparator m_comparator;

public:
  max_element_reduce(Comparator comparator)
    : m_comparator(std::move(comparator))
  { }

  template<typename Reference>
  T operator()(Reference&& a, Reference&& b) const
  {
    T a_val = a;
    T b_val = b;

    if (m_comparator(a_val.second, b_val.second))
      return b_val;
    else
      return a_val;
  }

  void define_type(typer& t)
  { t.member(m_comparator); }
}; // class max_element_reduce


//////////////////////////////////////////////////////////////////////
/// @brief Implementation function of @ref max_element
/// @param view One-dimensional view of the input.
/// @param comparator Binary functor which implements the less operation.
/// @returns Pair where @p first is the index of the selected element and
/// @p second is the value of the selected element.
//////////////////////////////////////////////////////////////////////
template<typename View, typename Comparator>
std::pair<typename View::index_type, typename View::value_type>
max_impl(View const& view, Comparator const& comparator)
{
  using reduce_val_t =
    std::pair<typename View::index_type, typename View::value_type>;

  return stapl::map_reduce<skeletons::tags::with_coarsened_wf>(
    algo_details::range_max_element<Comparator>(comparator),
    algo_details::max_element_reduce<reduce_val_t, Comparator>(comparator),
    view);
}


//////////////////////////////////////////////////////////////////////
/// @brief Work function invokes sequential (i.e., STL) @p max_element,
/// returning the value of the corresponding element.
/// @ingroup extremaAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename Comparator>
class range_max_value
{
private:
  Comparator m_comparator;

public:
  range_max_value(Comparator comparator)
    : m_comparator(std::move(comparator))
  { }

  template<typename View>
  typename View::value_type
  operator()(View const& vw) const
  {
    return *std::max_element(vw.begin(), vw.end(), m_comparator);
  }

  void define_type(typer& t)
  { t.member(m_comparator); }
}; // class range_max_value


//////////////////////////////////////////////////////////////////////
/// @brief Work function which reduces two values,
/// selecting the maximum value.
/// @tparam T Pair type which is reduced.
/// @tparam Comparator Comparison function which is called to determine the max.
/// @ingroup extremaAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename T, typename Comparator>
struct max_value_reduce
{
private:
  Comparator m_comparator;

public:
  max_value_reduce(Comparator comparator)
    : m_comparator(std::move(comparator))
  { }

  template<typename Reference>
  T operator()(Reference&& a, Reference&& b) const
  {
    T a_val = a;
    T b_val = b;

    if (m_comparator(a_val, b_val))
      return b_val;
    else
      return a_val;
  }

  void define_type(typer& t)
  { t.member(m_comparator); }
}; // class max_value_reduce


//////////////////////////////////////////////////////////////////////
/// @brief Implementation function of @ref max_value
/// @param view One-dimensional view of the input.
/// @param comparator Binary functor which implements the less operation.
/// @returns maximum element
//////////////////////////////////////////////////////////////////////
template<typename View, typename Comparator>
typename View::value_type
max_value_impl(View const& view, Comparator const& comparator)
{
  using reduce_val_t = typename View::value_type;

  return stapl::map_reduce<skeletons::tags::with_coarsened_wf>(
    algo_details::range_max_value<Comparator>(comparator),
    algo_details::max_value_reduce<reduce_val_t, Comparator>(comparator),
    view);
}

} // namespace algo_details


//////////////////////////////////////////////////////////////////////
/// @brief Finds the largest element in the input view (or the first largest
///   if there are multiple), which does not compare less than any other element
///   using the given functor.
/// @param view One-dimensional view of the input.
/// @param comparator Binary functor which implements the less operation.
/// @return Reference to the largest element in the input view.
/// @ingroup extremaAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename View, typename Comparator>
typename View::reference
max_element(View const& view, Comparator const& comparator)
{
  return view[algo_details::max_impl(view, comparator).first];
}


//////////////////////////////////////////////////////////////////////
/// @brief Finds the largest element in the input view (or the first largest
///   if there are multiple).
/// @param view One-dimensional view of the input.
/// @return Reference to the largest element in the input view.
/// @ingroup extremaAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename View>
typename View::reference
max_element(View const& view)
{
  return
    view[algo_details::max_impl(view, less<typename View::value_type>()).first];
}


//////////////////////////////////////////////////////////////////////
/// @brief Finds the largest value in the input view.
/// @param view One-dimensional view of the input.
/// @param comparator Binary functor used to compare elements.
/// @return The largest value in the input.
/// @ingroup extremaAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename View, typename Comparator>
typename View::value_type
max_value(View const& view, Comparator comparator)
{
  return algo_details::max_value_impl(view, comparator);
}


//////////////////////////////////////////////////////////////////////
/// @brief Finds the largest value in the input view.
/// @param view One-dimensional view of the input.
/// @return The largest value in the input.
/// @ingroup extremaAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename View>
typename View::value_type
max_value(View const& view)
{
  return algo_details::max_value_impl(view, less<typename View::value_type>());
}


//
// min_element, min_value
//

namespace algo_details {

//////////////////////////////////////////////////////////////////////
/// @brief Work function invokes sequential (i.e., STL) @p min_element,
/// returning the index and value of the corresponding element.
/// @ingroup extremaAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename Comparator>
class range_min_element
{
private:
  Comparator m_comparator;

public:
  range_min_element(Comparator comparator)
    : m_comparator(std::move(comparator))
  { }

  template<typename View>
  std::pair<typename View::index_type, typename View::value_type>
  operator()(View const& vw) const
  {
    auto ref = *std::min_element(vw.begin(), vw.end(), m_comparator);

    return std::pair<typename View::index_type, typename View::value_type>(
      index_of(ref), ref);
  }

  void define_type(typer& t)
  { t.member(m_comparator); }
}; // class range_min_element


//////////////////////////////////////////////////////////////////////
/// @brief Work function which reduces two pairs of index and values,
/// selecting the pair with minimumvalue.
/// @tparam T Pair type which is reduced.
/// @tparam Comparator Comparison function which is called to determine the min.
/// @ingroup extremaAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename T, typename Comparator>
class min_element_reduce
{
private:
  Comparator m_comparator;

public:
  min_element_reduce(Comparator comparator)
    : m_comparator(std::move(comparator))
  { }

  template<typename Reference>
  T operator()(Reference&& a, Reference&& b) const
  {
    T a_val = a;
    T b_val = b;

    if (m_comparator(b_val.second, a_val.second))
      return b_val;
    else
      return a_val;
  }

  void define_type(typer& t)
  { t.member(m_comparator); }
}; // class min_element_reduce


//////////////////////////////////////////////////////////////////////
/// @brief Implementation function of @ref min_element
/// @param view One-dimensional view of the input.
/// @param comparator Binary functor which implements the less operation.
/// @returns Pair where @p first is the index of the selected element and
/// @p second is the value of the selected element.
//////////////////////////////////////////////////////////////////////
template<typename View, typename Comparator>
std::pair<typename View::index_type, typename View::value_type>
min_impl(View const& view, Comparator comparator)
{
  using reduce_val_t =
    std::pair<typename View::index_type, typename View::value_type>;

  return stapl::map_reduce<skeletons::tags::with_coarsened_wf>(
    algo_details::range_min_element<Comparator>(comparator),
    algo_details::min_element_reduce<reduce_val_t, Comparator>(comparator),
    view);
}


//////////////////////////////////////////////////////////////////////
/// @brief Work function invokes sequential (i.e., STL) @p min_element,
/// returning the value of the corresponding element.
/// @ingroup extremaAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename Comparator>
class range_min_value
{
private:
  Comparator m_comparator;

public:
  range_min_value(Comparator comparator)
    : m_comparator(std::move(comparator))
  { }

  template<typename View>
  typename View::value_type
  operator()(View const& vw) const
  {
    return *std::min_element(vw.begin(), vw.end(), m_comparator);
  }

  void define_type(typer& t)
  { t.member(m_comparator); }
}; // class range_min_value


//////////////////////////////////////////////////////////////////////
/// @brief Work function which reduces two values,
/// selecting the minimum value.
/// @tparam T Pair type which is reduced.
/// @tparam Comparator Comparison function which is called to determine the min.
/// @ingroup extremaAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename T, typename Comparator>
class min_value_reduce
{
private:
  Comparator m_comparator;

public:
  min_value_reduce(Comparator comparator)
    : m_comparator(std::move(comparator))
  { }

  template<typename Reference>
  T operator()(Reference&& a, Reference&& b) const
  {
    T a_val = a;
    T b_val = b;

    if (m_comparator(b_val, a_val))
      return b_val;
    else
      return a_val;
  }

  void define_type(typer& t)
  { t.member(m_comparator); }
}; // class min_value_reduce


//////////////////////////////////////////////////////////////////////
/// @brief Implementation function of @ref min_value
/// @param view One-dimensional view of the input.
/// @param comparator Binary functor which implements the less operation.
/// @returns minimum value
//////////////////////////////////////////////////////////////////////
template<typename View, typename Comparator>
typename View::value_type
min_value_impl(View const& view, Comparator comparator)
{
  using reduce_val_t = typename View::value_type;

  return stapl::map_reduce<skeletons::tags::with_coarsened_wf>(
    algo_details::range_min_value<Comparator>(comparator),
    algo_details::min_value_reduce<reduce_val_t, Comparator>(comparator),
    view);
}


} // namespace algo_details


//////////////////////////////////////////////////////////////////////
/// @brief Finds the smallest element in the input view (or the first smallest
///   if there are multiple), which compares less than any other element using
///   the given functor.
/// @param view One-dimensional view of the input.
/// @param comparator Binary functor which implements the less operation.
/// @return Reference to the smallest element in the input view.
/// @ingroup extremaAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename View, typename Comparator>
typename View::reference
min_element(View const& view, Comparator const& comparator)
{
  return view[algo_details::min_impl(view, comparator).first];
}


//////////////////////////////////////////////////////////////////////
/// @brief Finds the smallest element in the input view (or the first smallest
///   if there are multiple).
/// @param view One-dimensional view of the input.
/// @return Reference to the smallest element in the input view.
/// @ingroup extremaAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename View>
typename View::reference
min_element(View const& view)
{
  return
    view[algo_details::min_impl(view, less<typename View::value_type>()).first];
}


//////////////////////////////////////////////////////////////////////
/// @brief Finds the smallest value in the input view.
/// @param view One-dimensional view of the input.
/// @param comp Binary functor used to compare elements.
/// @return The smallest value in the input.
/// @ingroup extremaAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename View, typename Comparator>
typename View::value_type
min_value(View const& view, Comparator const& comparator)
{
  return algo_details::min_value_impl(view, comparator);
}


//////////////////////////////////////////////////////////////////////
/// @brief Finds the smallest value in the input view.
/// @param view One-dimensional view of the input.
/// @return The smallest value in the input.
/// @ingroup extremaAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename View>
typename View::value_type
min_value(View const& view)
{
  return algo_details::min_value_impl(view, less<typename View::value_type>());
}


//
// minmax_element, minmax_value
//

namespace algo_details {

//////////////////////////////////////////////////////////////////////
/// @brief Work function invokes sequential (i.e., STL) @p minmax_element.
/// returning the index and value of both corresponding elements.
/// @ingroup extremaAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename Comparator>
class range_minmax_element
{
private:
  Comparator m_comparator;

public:
  range_minmax_element(Comparator comparator)
    : m_comparator(std::move(comparator))
  { }

  template<typename View>
  tuple<typename View::index_type, typename View::value_type,
        typename View::index_type, typename View::value_type>
  operator()(View const& vw) const
  {
    auto ret_pair = std::minmax_element(vw.begin(), vw.end(), m_comparator);

    return tuple<typename View::index_type, typename View::value_type,
                 typename View::index_type, typename View::value_type>(
      index_of(*ret_pair.first),  *ret_pair.first,
      index_of(*ret_pair.second), *ret_pair.second
    );
  }

  void define_type(typer& t)
  { t.member(m_comparator); }
}; // class range_min_element


//////////////////////////////////////////////////////////////////////
/// @brief Work function which reduces inputs to select
/// min and max values from ranges and their corresponding indices.
/// @tparam T Tuple type storing min and max  values and their indices.
/// @tparam Comparator Comparison function which is called to determine
/// the min and max.
/// @ingroup extremaAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename T, typename Comparator>
class minmax_element_reduce
{
private:
  Comparator m_comparator;

public:
  minmax_element_reduce(Comparator comparator)
    : m_comparator(std::move(comparator))
  { }

  template<typename Reference>
  T operator()(Reference&& a, Reference&& b) const
  {
    T a_val = a;
    T b_val = b;

    const bool b1 =  m_comparator(get<1>(a_val), get<1>(b_val));
    const bool b2 = !m_comparator(get<3>(a_val), get<3>(b_val));

    return T(b1 ? get<0>(a_val) : get<0>(b_val),
             b1 ? get<1>(a_val) : get<1>(b_val),
             b2 ? get<2>(a_val) : get<2>(b_val),
             b2 ? get<3>(a_val) : get<3>(b_val));
  }

  void define_type(typer& t)
  { t.member(m_comparator); }
}; // class minmax_element_reduce


//////////////////////////////////////////////////////////////////////
/// @brief Implementation function of @ref minmax_element
/// and @ref minmax_value.
/// @param view One-dimensional view of the input.
/// @param comparator Binary functor which implements the less operation.
/// @returns Tuple with index and values for both selected elements.
//////////////////////////////////////////////////////////////////////
template<typename View, typename Comparator>
tuple<typename View::index_type, typename View::value_type,
      typename View::index_type, typename View::value_type>
minmax_impl(View const& view, Comparator comparator)
{
  using reduce_val_t =
    tuple<typename View::index_type, typename View::value_type,
          typename View::index_type, typename View::value_type>;

  return stapl::map_reduce<skeletons::tags::with_coarsened_wf>(
    algo_details::range_minmax_element<Comparator>(comparator),
    algo_details::minmax_element_reduce<reduce_val_t, Comparator>(comparator),
    view);
}


//////////////////////////////////////////////////////////////////////
/// @brief Work function invokes sequential (i.e., STL) @p minmax_element.
/// returning the value of both corresponding elements.
/// @ingroup extremaAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename Comparator>
class range_minmax_value
{
private:
  Comparator m_comparator;

public:
  range_minmax_value(Comparator comparator)
    : m_comparator(std::move(comparator))
  { }

  template<typename View>
  tuple<typename View::value_type, typename View::value_type>
  operator()(View const& vw) const
  {
    auto ret_pair = std::minmax_element(vw.begin(), vw.end(), m_comparator);

    return tuple<typename View::value_type, typename View::value_type>(
      *ret_pair.first, *ret_pair.second
    );
  }

  void define_type(typer& t)
  { t.member(m_comparator); }
}; // class range_minmax_value


//////////////////////////////////////////////////////////////////////
/// @brief Work function which reduces inputs to select
/// min and max values from ranges
/// @tparam T Tuple type storing min and max values
/// @tparam Comparator Comparison function which is called to determine
/// the min and max.
/// @ingroup extremaAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename T, typename Comparator>
class minmax_value_reduce
{
private:
  Comparator m_comparator;

public:
  minmax_value_reduce(Comparator comparator)
    : m_comparator(std::move(comparator))
  { }

  template<typename Reference>
  T operator()(Reference&& a, Reference&& b) const
  {
    T a_val = a;
    T b_val = b;

    const bool b1 =  m_comparator(get<0>(a_val), get<0>(b_val));
    const bool b2 = !m_comparator(get<1>(a_val), get<1>(b_val));

    return T(b1 ? get<0>(a_val) : get<0>(b_val),
             b2 ? get<1>(a_val) : get<1>(b_val));
  }

  void define_type(typer& t)
  { t.member(m_comparator); }
}; // class minmax_value_reduce

//////////////////////////////////////////////////////////////////////
/// @brief Implementation function of @ref minmax_value.
/// @param view One-dimensional view of the input.
/// @param comparator Binary functor which implements the less operation.
/// @returns Tuple with values for both selected elements.
//////////////////////////////////////////////////////////////////////
template<typename View, typename Comparator>
tuple<typename View::value_type, typename View::value_type>
minmax_value_impl(View const& view, Comparator comparator)
{
  using reduce_val_t =
    tuple<typename View::value_type, typename View::value_type>;

  return stapl::map_reduce<skeletons::tags::with_coarsened_wf>(
    algo_details::range_minmax_value<Comparator>(comparator),
    algo_details::minmax_value_reduce<reduce_val_t, Comparator>(comparator),
    view);
}


} // namespace algo_details


//////////////////////////////////////////////////////////////////////
/// @brief Returns a pair containing references to the minimum and
/// maximum elements returned by the given comparator when called on
/// all values in the input view.
/// @param view One-dimensional view of the input.
/// @param comparator Binary functor which implements the less operation.
/// @return A pair containing references to the minimum and maximum elements.
/// @ingroup extremaAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename View, typename Comparator>
std::pair<typename View::reference, typename View::reference>
minmax_element(View const& view, Comparator const& comparator)
{
  auto ret_val = algo_details::minmax_impl(view, comparator);

  return std::pair<typename View::reference, typename View::reference>(
    view[get<0>(ret_val)], view[get<2>(ret_val)]);
}


//////////////////////////////////////////////////////////////////////
/// @brief Returns a pair containing references to the minimum and
/// maximum elements returned when using @ref less as a comparator on
/// all values in the input view.
/// @param view One-dimensional view of the input.
/// @return A pair containing references to the minimum and maximum elements.
/// @ingroup extremaAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename View>
std::pair<typename View::reference, typename View::reference>
minmax_element(View const& view)
{
  auto ret_val =
    algo_details::minmax_impl(view, less<typename View::value_type>());

  return std::pair<typename View::reference, typename View::reference>(
    view[get<0>(ret_val)], view[get<2>(ret_val)]);
}


//////////////////////////////////////////////////////////////////////
/// @brief Returns a pair containing the minimum and maximum values returned
///   by the given comparator when called on all values in the input view.
/// @param view One-dimensional view of the input.
/// @param comp Binary functor used to compare elements.
/// @return A pair containing the minimum and maximum result values.
/// @ingroup extremaAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename View, typename Comparator>
std::pair<typename View::value_type, typename View::value_type>
minmax_value(View const& view, Comparator const& comparator)
{
  auto ret_val = algo_details::minmax_value_impl(view, comparator);

  return std::pair<typename View::value_type, typename View::value_type>(
    std::move(get<0>(ret_val)), std::move(get<1>(ret_val)));
}


//////////////////////////////////////////////////////////////////////
/// @brief Returns a pair containing the minimum and maximum values returned
///   when using @ref less as a comparator.
/// @param view One-dimensional view of the input.
/// @return A pair containing the minimum and maximum result values.
/// @ingroup extremaAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename View>
std::pair<typename View::value_type, typename View::value_type>
minmax_value(View const& view)
{
  auto ret_val =
    algo_details::minmax_value_impl(view, less<typename View::value_type>());

  return std::pair<typename View::value_type, typename View::value_type>(
    std::move(get<0>(ret_val)), std::move(get<1>(ret_val)));
}

} // namespace stapl

#endif

