/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <vector>
#include <array>
#include <utility>
#include <tuple>
#include <stapl/utility/integer_sequence.hpp>
#include <stapl/utility/utility.hpp>

using stapl::index_sequence;
using stapl::make_index_sequence;

namespace stl {

template<class T>
constexpr T pack_prod(T t)
{
  return t;
}

template<class T, class... Ts>
constexpr T pack_prod(T t, Ts... ts)
{
  return t * pack_prod(ts...);
}

template<class T, size_t N, size_t... Is>
constexpr T array_prod_impl(std::array<T,N> const& arr, index_sequence<Is...>)
{
  return pack_prod(std::get<Is>(arr)...);
}

template<class T, size_t N>
constexpr T array_prod(std::array<T,N> const& arr)
{
  return array_prod_impl(arr, make_index_sequence<N>{});
}

////////////////////////////////////////////////////////////////////////////////
/// @brief An adaptor for std::vector<T> which supports multidimensional
/// indexing via operator()(Indices...).
////////////////////////////////////////////////////////////////////////////////
template<typename T, size_t N>
class multiarray
{
private:
  using storage_t = std::vector<T>;
  using dims_t = std::array<size_t, N>;
  using index_t = std::array<size_t, N>;

  storage_t storage;
  dims_t dims;

  template<typename Index, typename... Indices>
  constexpr std::size_t linearize(size_t sum, Index&& i, Indices&&... is) const
  {
    return linearize((sum + std::get<sizeof...(Indices)>(dims)) * i,
                        std::forward<Indices>(is)...);
  }

  template<typename Index>
  constexpr std::size_t linearize(size_t sum, Index&& i) const
  {
    return sum + i;
  }

public:

  using value_type = T;
  using iterator = typename storage_t::iterator;
  using const_iterator = typename storage_t::const_iterator;

  multiarray(dims_t const& d, T const& t = {})
    : storage(array_prod(d), t), dims(d)
  { }

  template<typename... Indices>
  T const& operator()(Indices&&... is) const
  {
    return storage[linearize(0, std::forward<Indices>(is)...)];
  }
  template<typename... Indices>
  T& operator()(Indices&&... is)
  {
    return storage[linearize(0, std::forward<Indices>(is)...)];
  }

  T const& operator[](size_t idx) const { return storage[idx]; }
  T& operator[](size_t idx) { return storage[idx]; }

  iterator begin() { return storage.begin(); }
  iterator end()   { return storage.end(); }
  const_iterator begin() const { return storage.begin(); }
  const_iterator end()   const { return storage.end(); }

  constexpr dims_t const& dimensions() const { return dims; }
  size_t size() const { return storage.size(); }
};

// Base case of each_impl: call f with the index
template<class T, size_t N, class F, class... Indices>
inline void each_impl(multiarray<T,N> const& m, F& f,
    std::integral_constant<size_t, N>, Indices&&... ids)
{
  f(std::array<size_t,N>{std::forward<Indices>(ids)...});
}

// Loop over the I'th dim, calling each_impl<I+1>
template<class T, size_t N, class F, size_t I, class... Indices>
void each_impl(multiarray<T,N> const& m, F& f,
    std::integral_constant<size_t, I>, Indices&&... ids)
{
  const size_t end = std::get<I>(m.dimensions());
  for (size_t i = 0; i < end; ++i)
    each_impl(m, f, std::integral_constant<size_t, I+1>{},
        std::forward<Indices>(ids)..., i);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Call a functor with every index in the given multiarray, passed as a
/// std::array<size_t, N>. The functor may be stateful, but it is taken by copy.
////////////////////////////////////////////////////////////////////////////////
template<class T, size_t N, class F>
inline void each(multiarray<T,N> const& m, F f)
{
  each_impl(m, f, std::integral_constant<size_t, 0>{});
}

} // namespace stl
