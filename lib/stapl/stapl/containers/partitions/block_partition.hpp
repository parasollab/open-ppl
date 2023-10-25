/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_MULTIARRAY_BLOCK_PARTITION_HPP
#define STAPL_CONTAINERS_MULTIARRAY_BLOCK_PARTITION_HPP

#include <boost/mpl/int.hpp>
#include <stapl/containers/partitions/ndim_partition.hpp>
#include <stapl/containers/partitions/balanced.hpp>
#include <stapl/domains/indexed.hpp>
#include <stapl/utility/tuple.hpp>
#include <cmath>
#include <algorithm>
#include <array>
#include <numeric>
#include <iterator>

#include "partitions_generator.hpp"

#include <stapl/containers/type_traits/is_invertible_partition.hpp>

namespace stapl {

namespace multiarray_impl {

////////////////////////////////////////////////////////////////////////
/// @brief Determines if a number is not prime.
////////////////////////////////////////////////////////////////////////
struct not_prime
{
  bool operator()(size_t x) const
  {
    for (size_t i = floor(sqrt(x)); i >= 2; --i)
    {
      if (x % i == 0 && i != x)
        return true;
    }

    return false;
  }
};


////////////////////////////////////////////////////////////////////////
/// @brief Function object that initializes a tuple of n elements that is
/// used to create an n-dimensional multiarray partition.
////////////////////////////////////////////////////////////////////////
template<size_t N, typename = make_index_sequence<N>>
struct make_multiarray_size;


template <size_t N, std::size_t... Indices>
struct make_multiarray_size<N, index_sequence<Indices...>>
{
  using result_type = typename homogeneous_tuple_type<N, size_t>::type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Partitions the elements of the multiarray appropriately in
  /// each dimension.
  /// @param p The number of processors.
  //////////////////////////////////////////////////////////////////////
  result_type operator()(size_t p) const
  {
    std::array<size_t, N> v;

    v[0] = p;

    std::fill(v.begin() + 1, v.end(), 1);

    auto equal_one = [](size_t const& val) { return val == 1; };
    auto prime     = [](size_t const& val) { return !not_prime()(val); };

    while (std::count_if(v.begin(), v.end(), equal_one) > 0
        && std::count_if(v.begin(), v.end(), not_prime()) > 0)
    {
      auto filtered = std::partition(v.begin(), v.end(), prime);
      auto val1     = std::max_element(filtered, v.end());
      auto val2     = std::find_if(v.begin(), v.end(), equal_one);

      for (size_t i = floor(sqrt(*val1)); i >= 2; i--)
      {
        if (*val1 % i == 0) {
          *val2 = *val1/i;
          *val1 = i;
          break;
        }
      }
    }

    return result_type(get<N-Indices-1>(v)...);
  }
}; // struct make_multiarray_size


//////////////////////////////////////////////////////////////////////
/// @brief n-dimensional partition consisting of balanced_partitions of
/// indexed_domain. This class will partition a multidimensional domain
/// into multidimensional blocks with a certain number of blocks in each
/// dimension. This is the default partition for the @ref multiarray
/// container.
///
/// @tparam Traversal Multidimensional traversal type
/// @see multiarray
//////////////////////////////////////////////////////////////////////
template <typename Traversal>
class block_partition
  : public nd_partition<
      typename homogeneous_tuple_type<
        tuple_size<Traversal>::value,
        balanced_partition<indexed_domain<size_t>>>::type,
      Traversal>
{
private:
  /// The number of dimensions for this partition
  using dimension_type = boost::mpl::int_<tuple_size<Traversal>::value>;
  using base_type      = nd_partition<
                           typename homogeneous_tuple_type<
                             tuple_size<Traversal>::value,
                             balanced_partition<indexed_domain<size_t>>>::type,
                           Traversal>;

public:
  using partitions_type = typename base_type::partitions_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a multidimensional partition based on a given n-dimensional
  /// domain that is balanced based on the number of locations in each
  /// dimension.
  /// @param dom The original domain to partition.
  //////////////////////////////////////////////////////////////////////
  template<typename Dom>
  explicit
  block_partition(const Dom& dom)
    : base_type(partitions_impl::partitions_generator<
                  tuple_size<Traversal>::value, Dom, partitions_type
                >(dom)(partitions_type()))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a multidimensional partition based on a given n-dimensional
  /// domain and a tuple of the number of partitions in each dimension.
  /// @param dom The original domain to partition.
  /// @param nparts Tuple of the number of partitions in each dimension
  //////////////////////////////////////////////////////////////////////
  template<typename Dom, typename NParts>
  block_partition(Dom const& dom, NParts const& nparts)
    : base_type(partitions_impl::partitions_generator<
                  tuple_size<Traversal>::value, Dom, partitions_type
                >(dom)(partitions_type(), nparts))
  { }
}; // class block_partition

} // namespace multiarray_impl


template<typename Traversal>
struct is_invertible_partition<multiarray_impl::block_partition<Traversal>>
  : public std::integral_constant<bool, true>
{ };

} // namespace stapl

#endif // STAPL_CONTAINERS_MULTIARRAY_BLOCK_PARTITION_HPP
