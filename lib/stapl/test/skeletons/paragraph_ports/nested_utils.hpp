/*
 // Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
 // component of the Texas A&M University System.

 // All rights reserved.

 // The information and source code contained herein is the exclusive
 // property of TEES and may not be disclosed, examined or reproduced
 // in whole or in part without explicit written authorization from TEES.
 */

#include <cstdlib>
#include <vector>
#include <tuple>
#include <iomanip>
#include <stapl/runtime.hpp>
#include <stapl/utility/integer_sequence.hpp>
#include <stapl/utility/tuple.hpp>
#include <stapl/array.hpp>


#ifndef STAPL_TEST_SKELETONS_NESTED_UTILS_HPP
#define STAPL_TEST_SKELETONS_NESTED_UTILS_HPP


using namespace stapl;
using namespace skeletons;


void print_timer_values(std::string title, double elapsed_time)
{
  do_once([&]()
  {
    std::cout << "==========================================================\n";
    std::cout << title <<" time: " << elapsed_time << std::endl;
  });
}


template <int i, typename T>
struct sum_op;

template <typename T>
struct sum_op<3, T>
{
  typedef T result_type;

  template <typename V1, typename V2,
            typename V3, typename V4>
  result_type operator()(V1&& v1, V2&& v2,
                         V3&& v3, V4&& v4) const
  {
    v1 = v1 + v2 + v3 + v4;
    return v1;
  }
};


template <typename T>
struct sum_op<2, T>
{
  typedef T result_type;

  template <typename V1, typename V2, typename V3>
  result_type operator()(V1&& v1, V2&& v2, V3&& v3) const
  {
    v1 = v1 + v2 + v3;
    return v1;
  }
};


template <typename T>
struct sum_op<1, T>
{
  typedef T result_type;

  template <typename V1, typename V2>
  result_type operator()(V1&& v1, V2&& v2) const
  {
    v1 = v1 + v2;
    return v1;
  }
};


template <std::size_t dim>
std::array<skeletons::position, dim>
generate_corner(skeletons::position start)
{
  std::array<skeletons::position, dim> corner;
  corner.fill(start);
  return corner;
}


template<typename T,
         typename TT = typename std::remove_reference<T>::type, size_t... I>
auto reverse_impl(T&& t, index_sequence<I...>)
-> std::tuple<typename std::tuple_element<sizeof...(I) - 1 - I, TT>::type...>
{
  /// @note icc-16 fails to expand an expression if the @c Indices shows up
  /// twice in the expansion.
  constexpr size_t size = sizeof...(I);
  return std::make_tuple(
           std::get<size - 1 - I>(std::forward<T>(t))...);
}


template<typename T, typename TT = typename std::remove_reference<T>::type>
auto reverse_tuple(T&& t)
-> decltype(reverse_impl(std::forward<T>(t),
                         make_index_sequence<std::tuple_size<TT>::value>()))
{
  return reverse_impl(std::forward<T>(t),
           make_index_sequence<std::tuple_size<TT>::value>());
}

size_t reverse_tuple(size_t t)
{
  return t;
}

template <int cur_level, int level, typename View, typename IdxTuple>
bool check_helper(
       std::true_type, View&& view0, View&& view1, IdxTuple&& idx_tuple)
{
  auto&& cur_idx    = std::get<cur_level>(idx_tuple);
  auto&& mapped_idx = reverse_tuple(cur_idx);
  return view0[cur_idx] == view1[mapped_idx];
}


template <int cur_level, int level, typename View, typename IdxTuple>
bool check_helper(
       std::false_type, View&& view0, View&& view1, IdxTuple&& idx_tuple)
{
  auto&& cur_idx    = std::get<cur_level>(idx_tuple);
  auto&& mapped_idx = reverse_tuple(cur_idx);

  return check_helper<cur_level+1, level>(
          std::integral_constant<bool, cur_level + 1 == level - 1>(),
          view0[cur_idx],
          view1[mapped_idx],
          idx_tuple);
}


template <int level, typename View, typename IdxTuple>
bool symmetery_check(View&& view0, View&& view1, IdxTuple&& idx_tuple)
{
  return check_helper<0, level>(
           std::integral_constant<bool, 1 == level>(),
           std::forward<View>(view0),
           std::forward<View>(view1),
           std::forward<IdxTuple>(idx_tuple));
}

template <int level, typename View, typename V, typename IdxTuple>
bool verify(std::false_type, View&& view, V&& v, IdxTuple&& idx_tuple)
{
  auto dom = v.domain();
  auto cur = dom.first();
  auto sz  = dom.size();

  for (std::size_t i = 0; i < sz; ++i)
  {
    auto&& new_idx_tuple = stapl::tuple_cat(idx_tuple, stapl::make_tuple(cur));

    if (!symmetery_check<level>(
          std::forward<View>(view), std::forward<View>(view), new_idx_tuple))
      return false;
    cur = dom.advance(cur, 1);
  }

  return true;
}


template <int level, typename View, typename V, typename IdxTuple>
bool verify(std::true_type, View&& view, V&& v, IdxTuple&& idx_tuple)
{
  using is_nested =
    std::integral_constant<
    bool,
    is_container<
      typename std::decay<V>::type::value_type::value_type>::value>;

  auto dom = v.domain();
  auto cur = dom.first();
  auto sz  = dom.size();

  for (std::size_t i = 0; i < sz; ++i)
  {
    auto&& new_idx_tuple = stapl::tuple_cat(idx_tuple, stapl::make_tuple(cur));

    if (!verify<level>(
        is_nested(), std::forward<View>(view), v[cur], new_idx_tuple))
      return false;

    cur = dom.advance(cur, 1);
  }

  return true;
}


template <int level, typename V>
bool verify(V&& v)
{
  using is_nested =
    std::integral_constant<
    bool,
    is_container<
      typename std::decay<V>::type::value_type>::value>;

  return verify<level>(
    is_nested(), std::forward<V>(v), std::forward<V>(v), stapl::make_tuple());
}

#endif // STAPL_TEST_SKELETONS_NESTED_UTILS_HPP
