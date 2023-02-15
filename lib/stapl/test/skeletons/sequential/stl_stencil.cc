/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include "stl_multiarray.hpp"
#include <tuple>
#include <iostream>

#include <stapl/utility/integer_sequence.hpp>
#include <stapl/utility/tuple/ignore_index.hpp>
#include <stapl/skeletons/param_deps/stencil_utils.hpp>

// The following macros enable some optimizations
// #define SPECIALIZE_1D
// #define USE_FUNCTOR_BOUNDARY

// N_MACRO and P_MACRO control which stencil is compiled, default = 1d-5pt
#ifndef N_MACRO
#define N_MACRO 1
#endif

#ifndef P_MACRO
#define P_MACRO 2
#endif

using std::tuple;
using std::get;

namespace stl {

using stapl::skeletons::stencil_impl::traversal_order;

template<size_t N, size_t P>
struct stencil_tag { };


////////////////////////////////////////////////////////////////////////////////
/// @brief Runs a stencil based on the tag. If boundary data is provided, then
/// it executes a nonperiodic stencil and periodic otherwise.
////////////////////////////////////////////////////////////////////////////////
template<size_t N, size_t P, class... Args>
void stencil(stencil_tag<N,P> t, Args&&... args) {
  stencil_impl(t, std::forward<Args>(args)...,
      make_index_sequence<N>{},
      traversal_order<make_index_sequence<N>, make_index_sequence<P>>{});
}


// Reads from a boundary on the "left"  side (i.e. the offset is negative)
template<class Point, class B, class Idx, size_t... Is>
constexpr auto read_nonper_left(B const& b, Idx const& idx,
  index_sequence<Is...>) STAPL_AUTO_RETURN((
    get<Point::dimension>(b)(
      Point::dimension == Is ?
        Point::offset - 1 - get<Is>(idx) : get<Is>(idx)...)
))


// Reads from a boundary on the "right"  side (i.e. the offset is positive)
template<class Point, class B, class Idx, size_t... Is>
constexpr auto read_nonper_right(B const& b, Idx const& idx, Idx const& m,
  index_sequence<Is...>) STAPL_AUTO_RETURN((
  get<Point::dimension + std::tuple_size<Idx>::value>(b)(
    Point::dimension == Is ?
      Point::offset + get<Is>(idx) - get<Is>(m) : get<Is>(idx)...)
))

// Checks if an index would go past the right bounds for a given Point
template<class Point, class Idx, class Dims>
constexpr bool past_right(Idx const& idx, Dims const& m) {
  return get<Point::dimension>(idx) + Point::offset > get<Point::dimension>(m);
}

// Read data for a nonperiodic stencil, i.e. one with boundary params
template<class Point, class Data, class Idx, class B, size_t... Is>
typename Data::value_type read_nonperiodic(Data const& d,
  Idx const& idx, Idx const& m, B const& b, index_sequence<Is...> is)
{
   if (Point::sign < 0 && get<Point::dimension>(idx) < Point::offset)
    return read_nonper_left<Point>(b, idx, is);
   if (Point::sign > 0 && past_right<Point>(idx, m))
     return read_nonper_right<Point>(b, idx, m, is);
   return d(get<Is>(idx)...);
}

#ifdef SPECIALIZE_1D
template<class Point, class Data, class Idx, class B>
typename Data::value_type read_nonperiodic(Data const& d, Idx const& i,
  Idx const& m, B const& b, index_sequence<0> is)
{
  if (Point::sign < 0 && i < Point::offset)
    return get<0>(b)[Point::offset - 1 - i];
  if (Point::sign > 0 && i + Point::offset > m)
    return get<1>(b)[Point::offset + i - m];
  return d(i);
}
#endif


// Read data for a periodic stencil
template<class Point, class Data, class Idx, size_t... Is>
constexpr auto read_periodic(Data const& d, Idx const& idx, Idx const& m,
  index_sequence<Is...>) STAPL_AUTO_RETURN((

  d((get<Is>(m) + get<Is>(idx) +
      (Is == Point::dimension ? Point::signed_offset : 0)) % get<Is>(m)...)
))

#ifdef SPECIALIZE_1D
template<class Point, class Data, class Idx>
constexpr auto read_periodic(Data const& d, Idx const& i, Idx const& m,
  index_sequence<0>) STAPL_AUTO_RETURN((

  d[(m + Point::signed_offset + i) % m]
))
#endif

// Nonperiodic case
template<size_t N, size_t P, typename Op, typename Data, typename Bounds,
  size_t... Is, typename... Points>
void stencil_impl(stencil_tag<N,P>, Op const& op, Data const& center,
  Data& write, Bounds const& bounds, index_sequence<Is...> is, tuple<Points...>)
{
  const auto m = center.dimensions();

  each(center, [&](std::array<size_t, N>&& idx) {
    write(get<Is>(idx)...) = op(center(get<Is>(idx)...),
        read_nonperiodic<Points>(center, idx, m, bounds, is)...);
  });

}

#ifdef SPECIALIZE_1D
template<size_t P, typename Op, typename Data, typename Bounds, size_t... Is,
  typename... Points>
void stencil_impl(stencil_tag<1,P>, Op const& op, Data const& center,
  Data& write, Bounds const& bounds, index_sequence<0> is, tuple<Points...>)
{
  const size_t m = center.size();
  for (size_t i = 0; i < m; ++i)
    write[i] = op(center[i],
        read_nonperiodic<Points>(center, i, m, bounds, is)...);
}
#endif


// Periodic case
template<size_t N, size_t P, typename Op, typename Data,  size_t... Is,
  typename... Points>
void stencil_impl(stencil_tag<N,P>, Op const& op, Data const& center,
  Data& write, index_sequence<Is...> is, tuple<Points...>)
{
  const auto m = center.dimensions();

  each(center, [&](std::array<size_t, N>&& idx) {
    write(get<Is>(idx)...) = op(center(get<Is>(idx)...),
        read_periodic<Points>(center, idx, m, is)...);
  });
}


#ifdef SPECIALIZE_1D
template<size_t P, typename Op, typename Data,  size_t... Is,
  typename... Points>
void stencil_impl(stencil_tag<1,P>, Op const& op, Data const& center,
  Data& write, index_sequence<0> is, tuple<Points...>)
{
  const size_t m = center.size();
  for (size_t i = 0; i < m; ++i)
    write[i] = op(center[i], read_periodic<Points>(center, i, m, is)...);
}
#endif

} // namespace stl


struct stencil_op {
  template<class T>
  constexpr T operator()(T&& t) const {
    return t;
  }

  template<class T, class... Ts>
  constexpr T operator()(T t, Ts&&... ts) const {
    return t + (*this)(std::forward<Ts>(ts)...);
  }
};


template<class R, class T, size_t... Is>
constexpr R init_with_n(T const& t, index_sequence<Is...>) {
  return R{ stapl::ignore_index<Is>(t)... };
}

template<size_t N, class T>
constexpr std::array<T,N> array_with(T const& t) {
  return init_with_n<std::array<T,N>>(t, make_index_sequence<N>{});
}


#ifdef USE_FUNCTOR_BOUNDARY
// functor that always returns a t
// applying get<N> on it returns itself
template<class T>
struct repeated_value {
  T t;

  using value_type = T;

  template<typename... Ts>
  constexpr T const& operator()(Ts...) const { return t; }
  template<typename Ts>
  constexpr T const& operator[](Ts&&) const { return t; }
};
template<size_t N, class T>
constexpr repeated_value<T> const& get(repeated_value<T> const& r) { return r; }
#endif

#include <chrono>

int main(int argc, const char *argv[]){
  using namespace stl;
  using value_t = int;

  constexpr size_t N = N_MACRO;
  constexpr size_t P = P_MACRO;
  constexpr size_t Points = 2*N*P+1;
  using data_t = multiarray<value_t, N>;

  if (argc < 2) {
    std::cerr << "Usage: <n>\n";
    return EXIT_FAILURE;
  }
  const size_t n = atol(argv[1]);
  constexpr value_t v = 3;

  stencil_op op;
  stencil_tag<N,P> tag;

  const data_t center = { array_with<N>(n), v };
  data_t out = center;

  // Initialize a boundary with the same value
#ifdef USE_FUNCTOR_BOUNDARY
  constexpr repeated_value<value_t> b = { v };
#else
  auto const b = array_with<2*N>(center);
#endif

  using clock = std::chrono::high_resolution_clock;
  auto start = clock::now();
  stencil(tag, op, center, out);
  stencil(tag, op, center, out, b);
  auto end = clock::now();

  using seconds = std::ratio<1,1>;

  auto t = std::chrono::duration<double, seconds>(end - start).count();


#ifdef VALIDATE_RESULTS
  bool passed = true;
  const value_t expected = v * Points;

  for (auto const& v : out) {
    passed &= v == expected;
  }
  std::cout << "Testing periodic and nonperiodic " << N << "-d " << Points <<
    "-point stencils";
  if (passed)
    std::cout << " : PASSED\n";
  else
    std::cout << " : FAILED\n";

  std::cout << "Time: " << t << "s\n";
#else
  std::cout << "Periodic and Nonperiodic " << N << "-d " << Points <<
    "-point stencils time:  " << t << "s\n";
#endif

  return 0;
}
