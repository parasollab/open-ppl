/*
// Copyright (c) 2000-2010, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef BENCHMARKS_NAS_CG_NUMERIC_HPP
#define BENCHMARKS_NAS_CG_NUMERIC_HPP

namespace stapl {
namespace functional {

struct inner_product
{
  // FIXME - b/c of bind2nd usage...
  typedef double first_argument_type;
  typedef double second_argument_type;
  typedef double result_type;

  template<typename View1, typename View2>
  typename View1::value_type
  operator()(View1 const& v1, View2 const& v2) const
  {
    typedef typename View1::value_type value_t;

    typedef stapl::multiplies<value_t> wf1_t;
    typedef stapl::plus<value_t>       wf2_t;

    return map_reduce(wf1_t(), wf2_t(), v1, v2);
  }
}; // struct inner_product

} // namesapce functional


namespace prototype {

namespace result_of {

template<typename View1,
         typename View2,
         typename Init    = typename View1::value_type,
         typename Product = stapl::multiplies<typename View1::value_type>,
         typename Sum     = stapl::plus<Init> >
struct inner_product
  : public ::stapl::prototype::result_of::map_reduce<Product, Sum, View1, View2>
{ };

} // namespace result_of


template<typename View1, typename View2, typename Init, typename Product, typename Sum>
inline
typename result_of::inner_product<View1, View2, Init, Product, Sum>::type
inner_product(View1 const& view1, View2 const& view2, Init init, Product pred1, Sum pred2)
{
  // FIXME incorporate init
  //
  // return binary_op1(init, result);
  //
  //stapl_assert(view1.size() != 0, "inner_product found view1 size == 0"); 
  //stapl_assert(view2.size() != 0, "inner_product found view2 size == 0");

  return ::stapl::prototype::map_reduce(pred1, pred2, view1, view2);
}


template<typename View1, typename View2, typename Init>
inline
typename result_of::inner_product<View1, View2, Init>::type
inner_product(View1 const& view1, View2 const& view2, Init init)
{
  return inner_product(view1, view2, init,
                       stapl::plus<Init>(),
                       stapl::multiplies<typename View1::value_type>());
}


template<typename View1, typename View2>
inline
typename result_of::inner_product<View1, View2>::type
inner_product(View1 const& view1, View2 const& view2)
{
  //stapl_assert(view1.size() != 0, "inner_product found view1 size == 0");
  //stapl_assert(view2.size() != 0, "inner_product found view2 size == 0");

  typedef typename View1::value_type value_t;

  return ::stapl::prototype::map_reduce(
    stapl::multiplies<value_t>(),
    stapl::plus<value_t>(),
    view1, 
    view2
  );
}


namespace functional {

struct inner_product
{
  // FIXME - b/c of bind2nd usage...
  typedef double first_argument_type;
  typedef double second_argument_type;
  typedef double result_type;

  template<typename View1, typename View2>
  typename ::stapl::prototype::result_of::inner_product<View1, View2>::type
  operator()(View1 const& v1, View2 const& v2) const
  {
    return ::stapl::prototype::inner_product(v1, v2);
  }
}; // struct inner_product

} // namespace functional

namespace detail {

 // FIXME - is there a good way to inline this?
 // 
 // FIXME - need to compute return type instead of just going to double
 //
 struct sqrt_wf
 {
   typedef double result_type;

   template<typename Reference>
   // FIXME - actually for now, it's not ref by value type being passed in...
   // not sure ATM which is better / correct.
   double operator()(Reference ref) const
   {
     // typedef typename proxy_core_access::value_type<Reference>::type value_t;

     return std::sqrt(ref);
   }
 };
} // namespace detail


namespace result_of {

template<typename Reference>
struct sqrt
  : public ::stapl::result_of::transform_reference<Reference, detail::sqrt_wf>
{ }; 

} // namespace result_of


template<typename Reference>
typename result_of::sqrt<Reference>::type
sqrt(Reference ref)
{
  return transform_reference(ref, detail::sqrt_wf()); 
}

} // namespace prototype
} // namespace stapl
#endif
