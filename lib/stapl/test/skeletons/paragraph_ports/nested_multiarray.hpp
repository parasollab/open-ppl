/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef TEST_SKELETONS_PARAGRAPH_PORTS_NESTED_MULTIARRAY_HPP
#define TEST_SKELETONS_PARAGRAPH_PORTS_NESTED_MULTIARRAY_HPP

#include <cstdlib>
#include <iostream>
#include <stapl/runtime.hpp>
#include <stapl/array.hpp>
#include <stapl/multiarray.hpp>
#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/utility/tuple.hpp>

namespace stapl {

template <typename V, typename Value>
void init(std::false_type, V&& v, Value&& value)
{
  auto dom = v.domain();
  auto cur = dom.first();
  auto sz  = dom.size();

  for (std::size_t i = 0; i < sz; ++i)
  {
    v[cur] = value;
    cur    = dom.advance(cur, 1);
  }
}

template <typename V, typename Value>
void init(std::true_type, V&& v, Value&& value)
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
    init(is_nested(), v[cur], value);
    cur = dom.advance(cur, 1);
  }
}


template <typename V, typename Value>
void init(V&& v, Value&& value)
{
  using is_nested =
    std::integral_constant<
    bool,
    is_container<
      typename std::decay<V>::type::value_type>::value>;

  init(is_nested(), v, value);
}


template <std::size_t level, typename DimsTuple, typename ValueType>
struct nested_multiarray
{
  static constexpr size_t num_levels = tuple_size<DimsTuple>::value;

  static constexpr size_t ndims =
    tuple_size<
      typename tuple_element<num_levels - level - 1, DimsTuple>::type>::value;

  using type =
  typename std::conditional<
        ndims == 1,
        array<
          typename nested_multiarray<level - 1, DimsTuple, ValueType>::type>,
        multiarray<
          ndims,
          typename nested_multiarray<level - 1, DimsTuple,ValueType>::type>
        >::type;
};


template <typename DimsTuple, typename ValueType>
struct nested_multiarray<0, DimsTuple, ValueType>
{
  static constexpr size_t num_levels = tuple_size<DimsTuple>::value;

  static constexpr size_t ndims =
    tuple_size<typename tuple_element<num_levels -1, DimsTuple>::type>::value;

  using type =
    typename std::conditional<ndims == 1,
                              array<ValueType>,
                              multiarray<ndims, ValueType>>::type;
};

template <size_t level, typename DimsTuple>
struct make_nested_dims_container
{
  static constexpr size_t num_levels = tuple_size<DimsTuple>::value;

  static constexpr size_t cur_dims = tuple_size<
    typename tuple_element<num_levels - level - 1, DimsTuple>::type>::value;

  using container_t =
    typename std::conditional<
      cur_dims == 1,
      array<
        typename make_nested_dims_container<level - 1, DimsTuple>::container_t>,
      multiarray<
        cur_dims,
        typename make_nested_dims_container<level - 1, DimsTuple>::container_t>
    >::type;


  using view_t =
    typename std::conditional<
      cur_dims == 1,
      array_view<container_t>,
      multiarray_view<container_t>
    >::type;

  static view_t create(DimsTuple const& dims_tuple)
  {
    auto&& x = make_nested_dims_container<
      level - 1, typename tuple_ops::result_of::pop_back<DimsTuple>::type>::
      create(tuple_ops::pop_back(dims_tuple));

    container_t* nested_cont = new container_t(x);
    view_t nested_view(nested_cont);

    init(nested_view, tuple_ops::extract_1D(get<level>(dims_tuple)));
    rmi_fence();
    return nested_view;
  }
};


template <typename DimsTuple>
struct make_nested_dims_container<1, DimsTuple>
{
  static constexpr size_t num_levels = tuple_size<DimsTuple>::value;

  static constexpr size_t value_type_dims =
    tuple_size<typename tuple_element<num_levels - 1, DimsTuple>::type>::value;

  using value_type =
    typename std::conditional<
      value_type_dims == 1,
      size_t,
      typename tuple_element<num_levels - 1, DimsTuple>::type
      >::type;

  static constexpr size_t cur_dims =
    tuple_size<typename tuple_element<num_levels - 2, DimsTuple>::type>::value;

  using container_t =
    typename std::conditional<
      cur_dims == 1,
      array<value_type>,
      multiarray<cur_dims, value_type>
    >::type;

  using view_t =
    typename std::conditional<
      cur_dims == 1,
      array_view<container_t>,
      multiarray_view<container_t>
    >::type;

  static view_t create(DimsTuple const& dims_tuple)
  {
    container_t* nested_cont =
      new container_t(tuple_ops::extract_1D(get<num_levels - 2>(dims_tuple)),
                      tuple_ops::extract_1D(get<num_levels - 1>(dims_tuple)));
    view_t nested_view(nested_cont);

    return nested_view;
  }
};

} // namespace stapl

#endif // TEST_SKELETONS_PARAGRAPH_PORTS_NESTED_MULTIARRAY_HPP
