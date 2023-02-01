/*
// Copyright (c) 2000-2010, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef SERIAL_DO_LOOP_HPP
#define SERIAL_DO_LOOP_HPP

namespace stapl {

// The Beautiful Recursive version that is slow & blows out the stack,
// but really is the essence of the algorithm...
/*
//  if (n != 1)
//  {
//    auto res = serial_do_loop(n - 1, wf, z, r, p, rho);
//
//    return wf(n, get<0>(res), get<1>(res), get<2>(res), get<3>(res));
//  }
//
//  return wf(n, z, r, p, rho);
*/

namespace detail {

struct reset_element
{
  template<typename V>
  void operator()(composition::map_view<V>& lhs,
                  composition::map_view<V> const& rhs) const
  {
    lhs = rhs;
  }

  // FIXME - a unified proxy overload is just fine (all below specializations
  // do the same thing).  Good for debugging as written now, as now proxies I'm
  // not expecting slide by...
  //

  template<typename T, typename View>
  void operator()(proxy<T, edge_accessor<View> >& lhs,
                  proxy<T, edge_accessor<View> > const& rhs) const
  {
    proxy_core_access::reset(lhs, rhs);
  }


  template<typename T, typename Reference, typename Functor>
  void operator()(proxy<T, unary_tm_accessor<Reference, Functor> >& lhs,
                  proxy<T, unary_tm_accessor<Reference, Functor> > const& rhs) const
  {
    proxy_core_access::reset(lhs, rhs);
  }

  template<typename T, typename Reference1, typename Reference2, typename Functor>
  void operator()(proxy<T, binary_tm_accessor<Reference1, Reference2, Functor> >& lhs,
                  proxy<T, binary_tm_accessor<Reference1, Reference2, Functor> > const& rhs) const
  {
    proxy_core_access::reset(lhs, rhs);
  }

}; // struct reset_element

} // namespace detail


// FIXME - generalize to arbitrary number of arguments, and collapse these two
// function template signatures into that...
//
template<typename WF, typename View1D, typename Reference>
auto
serial_do_loop(std::size_t n, WF const& wf,
               View1D const& z, View1D const& r, View1D const& p, Reference rho)
  -> decltype(wf(n, z, r, p, rho))
{
  stapl_assert(n > 0, "serial_do_loop encountered n == 0");

  auto result = wf(0, z, r, p, rho);

  for (std::size_t i = 1; i < n; ++i)
  {
    vs_map(detail::reset_element(),
           result,
           wf(i, get<0>(result), get<1>(result), get<2>(result), get<3>(result))
    );
  }

  return result;
}


template<typename WF, typename View1D, typename ZetaRef>
auto
serial_do_loop(std::size_t n, WF wf, View1D x, ZetaRef zeta)
  -> decltype(wf(n, x, zeta))
{
  stapl_assert(n > 0, "serial_do_loop encountered n == 0");

  auto result = wf(0, x, zeta);
               
  for (std::size_t i = 1; i < n; ++i)
  {
    vs_map(detail::reset_element(),
           result, 
           wf(i, get<0>(result), get<1>(result))
    );
  }

  return result;
}

} // namespace stapl

#endif // ifndef SERIAL_DO_LOOP_HPP
