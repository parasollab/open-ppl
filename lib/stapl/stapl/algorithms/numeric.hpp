/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_ALGORITHMS_NUMERIC_HPP
#define STAPL_ALGORITHMS_NUMERIC_HPP

#include "functional.hpp"

#include <stapl/algorithms/numeric_fwd.hpp>
#include <stapl/views/counting_view.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/map.hpp>
#include <stapl/skeletons/map_reduce.hpp>
#include <stapl/skeletons/reduce.hpp>
#include <stapl/skeletons/scan.hpp>
#include <stapl/skeletons/executors/execute.hpp>
#include <stapl/skeletons/scan_reduce.hpp>
#include <stapl/utility/tuple.hpp>

namespace stapl {

namespace algo_details {

//////////////////////////////////////////////////////////////////////
/// @class adjacent_diff
/// @brief Work function to compute the difference of two adjacent elements in
///   a view.
/// @tparam Functor Binary functor implementing the difference operation.
//////////////////////////////////////////////////////////////////////
template <typename Functor, typename Index>
struct adjacent_diff
{
private:
  /// Operator implementing the difference operation
  Functor m_f;

  /// Index of the first element of the output view
  Index   m_first;

public:
  typedef void result_type;

  adjacent_diff(Functor f, Index first)
    : m_f(std::move(f)), m_first(std::move(first))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Compute the difference of two elements.
  /// @param x1 Anterior view of the source view provided
  ///   to the adjacent_difference function.
  /// @param x2 Posterior view of the source view provided
  ///   to the adjacent_difference function.
  /// @param y1 Anterior view of the output view provided
  ///   to the adjacent_difference function. Used only to set the first element.
  /// @param y2 Posterior view of the output view provided
  ///   to the adjacent_difference function.
  //////////////////////////////////////////////////////////////////////
  template<typename View0, typename View1, typename View2, typename View3>
  void operator()(View0&& x1, View1&& x2, View2&& y1, View3&& y2) const
  {
    using data_t = typename std::decay<View0>::type::value_type;

    data_t val = *(x1.begin());

    // Set the first element of the output view to the first element of the
    // input view
    if (y1.domain().first() == m_first)
      *y1.begin() = val;

    auto y_it = y2.begin();
    auto x2_it = x2.begin();
    auto x2_end = x2.end();

    for (; x2_it != x2_end; ++x2_it, ++y_it)
    {
      data_t curr = *x2_it;
      *y_it = m_f(curr, val);
      val = std::move(curr);
    }
  }

  void define_type(typer &t)
  { t.member(m_f); }
};


//////////////////////////////////////////////////////////////////////
/// @class zip_inner_product_reduce_wf
/// @brief Reduce two vectors of products to form the final result of the
///   inner_product of a view with the components of a zip_view.
/// @tparam T Type of the elements of the input view.
//////////////////////////////////////////////////////////////////////
template<class T>
struct zip_inner_product_reduce_wf
{
  typedef std::vector<T> result_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Compute the element-wise sum of two input vectors.
  /// @param elt0 Reference of a std::vector of products from an invocation of
  ///   zip_inner_product_map_wf.
  /// @param elt1 Reference of a std::vector of products from an invocation of
  ///   zip_inner_product_map_wf.
  /// @return std::vector of the element-wise sum of the two input vectors.
  //////////////////////////////////////////////////////////////////////
  template<typename Elt0, typename Elt1>
  std::vector<T>
  operator()(Elt0 const& elt0, Elt1 const& elt1) const
  {
    std::vector<T> vect0=elt0;
    std::vector<T> vect1=elt1;
    std::vector<T> result(vect0.size(),0);

    for (size_t i=0;i<vect0.size();++i)
      result[i]=vect0[i]+vect1[i];

    return result;
  }
};


//////////////////////////////////////////////////////////////////////
/// @class zip_inner_product_map_wf
/// @brief Work function that implements the pair-wise inner product of a view
///   with each element of a tuple of views passed to one of the inner_product
///   functions.
/// @tparam T value type returned by the sum of pair-wise products.
//////////////////////////////////////////////////////////////////////
template<class T>
struct zip_inner_product_map_wf
{
private:
  //////////////////////////////////////////////////////////////////////
  /// @class product
  /// @brief Computes the product of an input element with each of a set of
  ///  elements.
  ///
  /// The function operator of zip_inner_product_map_wf has to compute the
  /// product of an element with each of the elements of a tuple.  This class
  /// is the operation passed to tuple_ops::for_each to perform the
  /// multiplication.
  //////////////////////////////////////////////////////////////////////
  struct product
  {
    typedef std::vector<T> result_type;

    mutable result_type    m_res;
    T                      m_x;

    //////////////////////////////////////////////////////////////////////
    /// @brief Store the element that is multiplied with each element of a tuple
    ///   in the m_x data member.
    /// @param x Numeric value to be multiplied with each tuple element.
    //////////////////////////////////////////////////////////////////////
    product(T const& x)
      : m_x(x)
    { }

    //////////////////////////////////////////////////////////////////////
    /// @brief Compute the product of the input view element with one tuple
    ///   element and store it in the result vector.
    /// @param y Reference of tuple element to be multiplied with the input
    ///   element.
    //////////////////////////////////////////////////////////////////////
    template<typename Y>
    void operator()(Y const& y) const
    { m_res.push_back(m_x*y); }

    //////////////////////////////////////////////////////////////////////
    /// @brief Returns the result of the products of the input element with each
    ///   element of the input tuple.
    /// @return std::vector of the results of the products.
    //////////////////////////////////////////////////////////////////////
    result_type result() const
    { return m_res; }
  };

public:
  typedef std::vector<T> result_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Computes the product of an input element with each element of a
  ///   tuple.
  /// @tparam Elt0 Reference to an element of the input view.
  /// @tparam W Tuple type represented by the proxy.
  /// @tparam A Accessor type used by the proxy over a tuple from the zip_view
  ///   passed to inner_product.
  /// @param elt0 Reference of element from the input view.
  /// @param elts Reference of tuple of elements to be multiplied with elt0.
  /// @return std::vector of the results of the products of the input element
  ///   with each element of the tuple.
  //////////////////////////////////////////////////////////////////////
  template<typename Elt0, typename W, typename A>
  std::vector<T>
  operator()(Elt0 const& elt0, proxy<W,A> const& elts) const
  {
    product prod(elt0);

    W w = elts;

    tuple_ops::for_each(w, prod);

    return prod.result();
  }
};

//////////////////////////////////////////////////////////////////////
/// @class wt_inner_product_map_wf
/// @brief Work function that implements the weighted product operation used
///   in weighted_inner_product.
/// @tparam Op Binary functor that implements multiplication.
//////////////////////////////////////////////////////////////////////
template <typename Op>
struct wt_inner_product_map_wf
{
private:
  Op m_op;

public:
  typedef typename Op::result_type result_type;

  wt_inner_product_map_wf(Op const& op)
    : m_op(op)
  {}

  void define_type(typer& t)
  { t.member(m_op); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Computes the product of two input elements and a weight factor.
  /// @param r1 Reference of element from the first view passed to
  ///   weighted_inner_product.
  /// @param r2 Reference of element from the second view passed to
  ///   weighted_inner_product.
  /// @param wt Reference of element from the view of weights passed to
  ///   weighted_inner_product.
  /// @return The product of the weight with the product of the input elements.
  //////////////////////////////////////////////////////////////////////
  template <typename Ref1, typename Ref2, typename WtRef>
  result_type operator()(Ref1 const& r1, Ref2 const& r2, WtRef const& wt)
  { return m_op(m_op(r1, r2), wt); }
};
} // algo_details namespace

/// @addtogroup numericAlgorithms
/// @{

//////////////////////////////////////////////////////////////////////
/// @brief Compute the sum of the elements and the initial value.
/// @param view One-dimensional view of elements to be accumulated.
/// @param init Initial value that will be added to the sum of the
///   elements.
/// @param oper Binary functor implementing the accumulation operation.
/// @return Accumulation of initial value and the elements of the view.
///
/// If the view is empty the initial value is returned.
//////////////////////////////////////////////////////////////////////
template<typename View, typename Oper>
typename view_traits<View>::value_type
accumulate(View const& view,
           typename view_traits<View>::value_type init, Oper oper)
{
  typedef typename view_traits<View>::value_type value_t;
  return oper(init,
              stapl::map_reduce(stapl::identity<value_t>(), oper, view));
}


//////////////////////////////////////////////////////////////////////
/// @brief Compute the sum of the elements and the initial value.
/// @param view One-dimensional view of elements to be accumulated.
/// @param init Initial value that will be added to the sum of the elements.
///
/// This function calls the previous accumulate function, specifying that the
/// stapl::plus functor be used as the binary operator for accumulation.
//////////////////////////////////////////////////////////////////////
template<typename View>
typename view_traits<View>::value_type
accumulate(View const& view, typename view_traits<View>::value_type init)
{
  typedef typename view_traits<View>::value_type value_t;
  return accumulate(view,init,stapl::plus<value_t>());
}


//////////////////////////////////////////////////////////////////////
/// @brief Assign each element of the output the difference between the
///   corresponding input element and the input element that precedes it.
/// @param view1 One-dimensional view of elements whose difference with the
///   preceding element will be computed.
/// @param view2 One-dimensional view where the differences between input
///   elements will be written.
/// @param oper Binary functor that implements the difference operation.
///
/// The first element of the output view is assigned the first element of the
/// input view.
//////////////////////////////////////////////////////////////////////
template<typename View1, typename View2, typename Oper>
inline
void adjacent_difference(View1 const& view1, View2& view2, Oper oper)
{
  using view1_t = View1;
  using view1_dom_t  = typename view1_t::domain_type;

  using view2_t = View1;
  using view2_dom_t  = typename view2_t::domain_type;

  view1_t view1_ant(view1.container(),
      view1_dom_t(view1.domain().first(),
        view1.domain().advance(
          view1.domain().first(), view1.size() - 2),
        view1.domain()));

  view1_t view1_post(view1.container(),
      view1_dom_t(view1.domain().advance(view1.domain().first(), 1),
        view1.domain().last(), view1.domain()));

  view2_t view2_ant(view2.container(),
      view1_dom_t(view2.domain().first(),
        view2.domain().advance(
          view2.domain().first(), view2.size() - 2),
        view2.domain()));

  view2_t view2_post(view2.container(),
      view2_dom_t(view2.domain().advance(view2.domain().first(), 1),
        view2.domain().last(), view2.domain()));

  map_func<skeletons::tags::with_coarsened_wf>(
    algo_details::adjacent_diff<Oper, typename view2_dom_t::index_type>(
      oper, view2.domain().first()),
    view1_ant, view1_post, view2_ant, view2_post);
}


//////////////////////////////////////////////////////////////////////
/// @brief Assign each element of the output the difference between the
///   corresponding input element and the input element that precedes it.
/// @tparam View1 One-dimensional view over elements of a numeric type.
/// @tparam View2 One-dimensional view over elements of a numeric type to which
///   View1::value_type is convertible.
/// @param view1 One-dimensional view of elements whose difference with the
///   preceding element will be computed.
/// @param view2 One-dimensional view where the differences between input
///   elements will be written.
///
/// The first element of the output view is assigned the first element of the
/// input view.  This function invokes the previous adjacent_difference function
/// specifying the stapl::minus functor be used as the operator.
//////////////////////////////////////////////////////////////////////
template<typename View1, typename View2>
inline
void adjacent_difference(View1 const& view1, View2& view2)
{
  stapl::adjacent_difference(view1, view2,
                             stapl::minus<typename View1::value_type>());
}


//////////////////////////////////////////////////////////////////////
/// @brief Compute the inner product of the elements of two input views.  Inner
/// product is defined as the sum of pair-wise products of the elements of two
/// input views.
/// @param view1 One-dimensional view of the first sequence of input elements.
/// @param view2 One-dimensional view of the second sequence of input elements.
/// @param init Initial value that will be added to the sum of the products.
/// @param op1 Binary functor that implements the addition operation.
/// @param op2 Binary functor that implements the product operation.
/// @return The sum of the initial value and the pair-wise products.
///
/// The input views must have the same size.
//////////////////////////////////////////////////////////////////////
template<typename View1, typename View2, typename Sum,
         typename Product>
inline
typename view_traits<View1>::value_type
inner_product(View1 const& view1, View2 const& view2,
              typename view_traits<View1>::value_type init, Sum op1,
              Product op2)
{
  return op1(init, stapl::map_reduce(op2, op1, view1, view2));
}


//////////////////////////////////////////////////////////////////////
/// @brief Compute the inner product of the elements of two input views.  Inner
/// product is defined as the sum of pair-wise products of the elements of two
/// input views.
/// @param view1 One-dimensional view of the first sequence of input elements.
/// @param view2 One-dimensional view of the second sequence of input elements.
/// @param init Initial value that will be added to the sum of the products.
/// @return The sum of the initial value and the pair-wise products.
///
/// The input views must have the same size.  This function invokes the previous
/// inner_product function specifying stapl::multiplies for the product operator
/// and stapl::plus for the addition operator.
//////////////////////////////////////////////////////////////////////
template<typename View1, typename View2>
inline
typename view_traits<View1>::value_type
inner_product(View1 const& view1, View2 const& view2,
              typename view_traits<View1>::value_type init)
{
  typedef typename view_traits<View1>::value_type value_t;
  return inner_product(view1, view2, init,
                       stapl::plus<value_t>(),
                       stapl::multiplies<value_t>());
}


//////////////////////////////////////////////////////////////////////
/// @brief Compute the inner product of the elements of one view and the
///   elements of each of a set of views.
/// @param va One-dimensional view of the first sequence of input elements.
/// @param views Tuple of one-dimensional views of sequences of input elements.
/// @param init Vector of initial values to use in each of the inner products
///   performed.
/// @return A vector of the sums of the initial values and the pair-wise
///   products of the first view with each of the views in the tuple provided.
///
/// The input views must have the same size.
//////////////////////////////////////////////////////////////////////
template <typename T, typename ViewA, typename Views>
std::vector<T>
inner_product(const ViewA& va, const Views& views, std::vector<T> init)
{
  algo_details::zip_inner_product_reduce_wf<T> op1;
  algo_details::zip_inner_product_map_wf<T>    op2;
  return op1(init, stapl::map_reduce(op2, op1, va, views));
}


//////////////////////////////////////////////////////////////////////
/// @brief Compute a weighted inner product of the elements of two views, where
///   each pair-wise product is multiplied by a weight factor before the sum of
///   products is performed.
/// @param view1 One-dimensional view of the first sequence of input elements.
/// @param view2 One-dimensional view of the second sequence of input elements.
/// @param wt One-dimensional view of the sequence of weights.
/// @param init Initial value that will be added to the sum of the weighted
///   products.
/// @param op1 Binary functor implementing addition.
/// @param op2 Binary functor implementing multiplication used to compute the
///   product of two elements and the product of the weight and the result of
///   the element product.
/// @return The sum of the initial value and the weighted pair-wise products.
///
/// The input views must have the same size.
//////////////////////////////////////////////////////////////////////
template <typename View1, typename View2, typename View3,
          typename Sum, typename Product>
typename view_traits<View1>::value_type
weighted_inner_product(View1 const& view1, View2 const& view2, View3 const& wt,
                       typename view_traits<View1>::value_type init,
                       Sum op1, Product op2)
{
  return op1(init,
             map_reduce(algo_details::wt_inner_product_map_wf<Product>(op2),
             op1, view1, view2, wt));
}

//////////////////////////////////////////////////////////////////////
/// @brief Compute a weighted inner product of the elements of two views, where
///   each pair-wise product is multiplied by a weight factor before the sum of
///   products is performed.
/// @param view1 One-dimensional view of the first sequence of input elements.
/// @param view2 One-dimensional view of the second sequence of input elements.
/// @param wt One-dimensional view of the sequence of weights.
/// @param init Initial value that will be added to the sum of the weighted
///   products.
/// @return The sum of the initial value and the weighted pair-wise products.
///
/// The input views must have the same size. This function invokes the previous
/// weighted_inner_product function specifying stapl::multiplies for the product
/// operator and stapl::plus for the addition operator.
//////////////////////////////////////////////////////////////////////
template <typename View1, typename View2, typename View3>
typename view_traits<View1>::value_type
weighted_inner_product(View1 const& view1, View2 const& view2, View3 const& wt,
                       typename view_traits<View1>::value_type init)
{
  typedef stapl::multiplies<typename View1::value_type> prod_op;
  typedef stapl::plus<typename View1::value_type> sum_op;
  return weighted_inner_product(view1, view2, wt, init, sum_op(), prod_op());
}


//////////////////////////////////////////////////////////////////////
/// @brief Compute a weighted normal of the elements of a view.
/// @param view1 One-dimensional view of the sequence of input elements.
/// @param wt One-dimensional view of the sequence of weights.
/// @param op1 Binary functor implementing addition.
/// @param op2 Binary functor implementing multiplication used to compute the
///   product of an element with itself and the product of the weight and the
///   element's square.
/// @return The square root of the sum of weighted squares of the input
///   elements.
///
/// The input views must have the same size.
//////////////////////////////////////////////////////////////////////
template <typename View1, typename View2, typename Sum, typename Product>
typename View1::value_type
weighted_norm(View1 const& view1, View2 const& wt, Sum op1, Product op2)
{
  return std::sqrt(
    weighted_inner_product(view1, view1, wt, typename View1::value_type(0),
                           op1, op2)
  );
}


//////////////////////////////////////////////////////////////////////
/// @brief Compute a weighted normal of the elements of a view.
/// @param view1 One-dimensional view of the sequence of input elements.
/// @param wt One-dimensional view of the sequence of weights.
/// @return The square root of the sum of weighted squares of the input
///   elements.
///
/// The input views must have the same size.  This function invokes the previous
/// weighted_norm function specifying stapl::multiplies for the product
/// operator and stapl::plus for the addition operator.
//////////////////////////////////////////////////////////////////////
template <typename View1, typename View2>
typename View1::value_type
weighted_norm(View1 const& view1, View2 const& wt)
{
  return std::sqrt(
    weighted_inner_product(view1, view1, wt, typename View1::value_type(0))
  );
}


#ifdef STAPL_PRODUCTION
//////////////////////////////////////////////////////////////////////
/// @brief Work function invokes sequential @ref partial_sum.
//////////////////////////////////////////////////////////////////////
template<typename BinaryOp>
class serial_partial_sum_wf
{
private:
  BinaryOp m_binary_op;

public:
  using result_type = void;

  serial_partial_sum_wf(BinaryOp binary_op)
    : m_binary_op(std::move(binary_op))
  { }

  template<typename SrcView, typename DstView>
  void operator()(SrcView&& src_vw, DstView&& dst_vw) const
  {
    std::partial_sum(src_vw.begin(), src_vw.end(),
                     dst_vw.begin(), std::move(m_binary_op));
  }

  void define_type(typer& t)
  { t.member(m_binary_op); }
};
#endif


//////////////////////////////////////////////////////////////////////
/// @brief Computes the prefix sum of the elements of the input view and stores
///   the result in the output view.
/// @param view0 A one-dimensional view over the input elements that are of a
///   numeric type.
/// @param view1 A one-dimensional view over the elements where the result of
///   the partial sum will be written.
/// @param binary_op The binary functor that will be used to sum elements.
/// @param shift Indicates if the partial sum is inclusive or exclusive.
///
/// An inclusive partial sum assigns the value of the first input element to the
/// first element in the output view, which is the definition of partial sum.
/// An exclusive partial sum is a prescan [Blelloch 1990]. It sets the elements
/// of the output view to the sum of all preceding elements in the input view,
/// with the first element assigned the identity value for addition of the
/// element type.
//////////////////////////////////////////////////////////////////////
template<typename View0, typename View1, typename BinaryOp>
void
partial_sum(View0 const& view0, View1 const& view1, BinaryOp binary_op,
            const bool shift)
{
#ifdef STAPL_PRODUCTION
  if (get_num_locations() == 1 && !shift)
  {
    skeletons::execute(
      skeletons::execution_params(default_coarsener()),
      skeletons::zip<2>(serial_partial_sum_wf<BinaryOp>(std::move(binary_op))),
      view0, view1);

    return;
  }
#endif

  scan(view0, view1, binary_op, shift);
}


//////////////////////////////////////////////////////////////////////
/// @brief Computes the prefix sum of the elements of the input view and stores
///   the result in the output view.
/// @param view0 A one-dimensional view over the input elements that are of a
///   numeric type.
/// @param view1 A one-dimensional view over the elements where the result of
///   the partial sum will be written.
/// @param shift Indicates if the partial sum is inclusive or exclusive.
///
/// This function calls the previous partial_sum function, specifying that the
/// stapl::plus functor be used as the binary operator for addition.
//////////////////////////////////////////////////////////////////////
template<typename View0, typename View1>
void
partial_sum(View0 const& view0, View1 const& view1, const bool shift)
{
  stapl::partial_sum(view0, view1,
                     stapl::plus<typename View0::value_type>(), shift);
}

//////////////////////////////////////////////////////////////////////
/// @brief Combination the prefix sum and accumulate of the elements
///   of the input view. The prefix sum results are stored in the
//    output view and result of the accumulate is returned.
///
/// @param view0 A one-dimensional view over the input elements that are of a
///   numeric type.
/// @param view1 A one-dimensional view over the elements where the result of
///   the partial sum will be written.
/// @param binary_op The binary functor that will be used to sum elements.
/// @param init_value the initial value needed for accumulate
/// @param shift Indicates if the partial sum is inclusive or exclusive.
///
//////////////////////////////////////////////////////////////////////
template <typename View0, typename View1, typename BinaryFunction>
typename View0::value_type
partial_sum_accumulate(View0 const& view0, View1 const& view1,
                       typename view_traits<View0>::value_type init_value,
                       BinaryFunction binary_op,
                       const bool shift)
{
  return binary_op(init_value, scan_reduce(view0, view1, binary_op, shift));
}

//////////////////////////////////////////////////////////////////////
/// @brief Combination the prefix sum and accumulate of the elements
///   of the input view. The prefix sum results are stored in the
//    output view and result of the accumulate is returned.
///
/// @param view0 A one-dimensional view over the input elements that are of a
///   numeric type.
/// @param view1 A one-dimensional view over the elements where the result of
///   the partial sum will be written.
/// @param init_value the initial value needed for accumulate
/// @param shift Indicates if the partial sum is inclusive or exclusive.
///
/// This function calls the previous partial_sum_accumulate function,
/// specifying that the stapl::plus functor be used as the binary
/// operator for addition.
//////////////////////////////////////////////////////////////////////
template <typename View0, typename View1>
typename View0::value_type
partial_sum_accumulate(
  View0 const& view0, View1 const& view1,
  typename view_traits<View0>::value_type init_value, const bool shift)
{
  return stapl::partial_sum_accumulate(
    view0, view1, init_value, stapl::plus<typename View0::value_type>(), shift);
}

/// @}

//////////////////////////////////////////////////////////////////////
/// @brief Initializes the elements of the view such that the first element
///   is assigned value, the next element value+1, etc.
/// @param view A one-dimensional view over elements of a numeric type that
///   are going to be initialized.
/// @param value The first value in the sequence assigned to the elements of
///   the input view.
/// @ingroup generatingAlgorithms
///
/// @note This algorithm is defined in the standard as a
/// @ref numericAlgorithms "numeric algorithm".  It is included in the
/// generating algorithms here because it behaves like a generating
/// algorithm that requires that the view's element type is numeric.
//////////////////////////////////////////////////////////////////////
template<typename View0>
void iota(View0 const& view, typename View0::value_type const& value)
{
  map_func(assign<typename View0::value_type>(),
    counting_view<typename View0::value_type>(view.size(), value), view);
}

} // namespace stapl
#endif
