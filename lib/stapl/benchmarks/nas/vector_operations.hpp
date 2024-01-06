/*
// Copyright (c) 2000-2010, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef VECTOR_OPERATIONS_HPP
#define VECTOR_OPERATIONS_HPP

#include <stapl/algorithms/algorithm.hpp>
#include "counting_view.hpp"

namespace stapl {

namespace detail {

template<typename T>
struct const_value
{
  T m_val;

  typedef T result_type;

  const_value(T const& value)
    : m_val(value)
  { }

  void define_type(typer& t)
  {
    t.member(m_val);
  }

  template<typename Ref>
  result_type operator()(Ref) const
  {
    return m_val;
  }
}; // struct const_value

} // namespace detail


// FIXME - const_value allows us to return a constant value while receiving an
// unused element of the input view that exists solely to define the size / 
// iteration space.  If we had a bind() that behaved like the standard
// (i.e., ignored unused paramaters passed to the created functor), then the 
// following could be a shorter & cleaner implementation of vector_fill_n:
//
// map_func(bind(identity(), 5), counting_view(n));
//
template<typename T>
inline
auto vector_fill_n(T const& t, std::size_t n)
  -> decltype(
       composition::map_func(
         detail::const_value<T>(t),
         nas::counting_view(n)))
{
  return composition::map_func(detail::const_value<T>(t), nas::counting_view(n));
}


template<typename T>
inline
auto row_vector_fill_n(T const& t, std::size_t n, std::size_t row_size)
  -> decltype(
       composition::map_func(
         detail::const_value<T>(t),
         nas::counting_view(n, row_size)))
{
  return composition::map_func(detail::const_value<T>(t), nas::counting_view(n, row_size));
}



template<typename View2D, typename View1D>
inline
auto matvec(View2D& A, View1D& x)
  -> decltype(composition::map_func(
        stapl::bind2nd(prototype::functional::inner_product(), x),
        A.rows()))
{
  using prototype::functional::inner_product;
  using composition::map_func;

  return map_func(stapl::bind2nd(inner_product(), x), A.rows());
}


template<typename V>
inline
auto
operator-(composition::map_view<V> const& lhs,
          composition::map_view<V> const& rhs)
  -> decltype(composition::map_func(minus<typename composition::map_view<V>::value_type>(), lhs, rhs))
{
  return composition::map_func(minus<typename composition::map_view<V>::value_type>(), lhs, rhs);
}


template<typename V>
inline
auto
operator+(composition::map_view<V> const& lhs,
          composition::map_view<V> const& rhs)
  -> decltype(composition::map_func(plus<typename composition::map_view<V>::value_type>(), lhs, rhs))
{
  return composition::map_func(plus<typename composition::map_view<V>::value_type>(), lhs, rhs);
}


template<typename T, typename A, typename V>
inline
auto
operator*(proxy<T,A> const& lhs,
          composition::map_view<V> const& rhs)
//  -> decltype(composition::map_func(bind1st(multiplies<T>(), lhs),rhs))
-> typename composition::result_of::map_func<decltype(bind1st(multiplies<T>(), lhs)), composition::map_view<V> >::type
{
  return composition::map_func(bind1st(multiplies<T>(), lhs), rhs);
}


template<typename Matrix, typename V>
inline
auto operator*(my_matrix_view<Matrix> const& lhs,
               composition::map_view<V> const& rhs)
  -> decltype(matvec(lhs, rhs))
{
  return matvec(lhs, rhs);
}


template<typename View1D>
inline
auto euclidean_norm(View1D const& v)
  -> decltype(
       prototype::sqrt(
         prototype::inner_product(v,v)))
{
  using prototype::inner_product;
  using prototype::sqrt;

  return sqrt(inner_product(v,v));
}

} // namespace stapl

// NOTE - Fused vector operations that I'd like to do automagically with
// an expression templates approach.
//
/*
// template<typename T, typename Operation>
// struct scaled_offset_wf
//   : private multiplies<T>,
//     private Operation
// {
//   typedef double third_argument_type;
//   typedef double result_type;
//
//   scaled_offset_wf(Operation const& op)
//     : multiplies<T>(),
//       Operation(op)
//   { }
//
//   multiplies<T> const& mult(void) const
//   {
//     return *this;
//   }
//
//   Operation const& add(void) const
//   {
//     return *this;
//   }
//
//   template<typename Ref1, typename Ref2, typename Ref3>
//   result_type
//   operator()(Ref1 x, Ref2 y, Ref3 alpha) const
//   {
//     return add()(x, mult()(alpha, y));
//   }
//
//   void define_type(typer& t)
//   {
//     t.base<multiplies<T> >(*this);
//     t.base<Operation>(*this);
//   }
// }; // struct scaled_offset_wf
//
//
// // compute x OP alpha * y
// //
// template<typename View1D, typename Operation>
// auto scaled_vector_operation(View1D& x,
//                              View1D& y,
//                              Operation const& op,
//                              typename View1D::value_type const& alpha)
//   -> decltype(
//        composition::map_func(
//          bind3rd(
//            scaled_offset_wf<typename View1D::value_type, Operation>(op),
//            alpha),
//          x, y))
// {
//   typedef typename View1D::value_type           value_t;
//   typedef scaled_offset_wf<value_t, Operation>  wf_t; 
// 
//   return composition::map_func(bind3rd(wf_t(op), alpha), x, y);
// }
*/

#endif // ifndef VECTOR_OPERATIONS_HPP
