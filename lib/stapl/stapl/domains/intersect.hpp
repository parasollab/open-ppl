/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_DOMAINS_INTERSECT_HPP
#define STAPL_DOMAINS_INTERSECT_HPP

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @todo This is never specialized. What is its purpose?
//////////////////////////////////////////////////////////////////////
template <typename T>
struct is_contiguous
  : public std::true_type
{ };

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Helper functor to compute the intersection between two
///        given domains (@p d0 and @p d1).
/// @pre D0::index_type == D1::index_type
//////////////////////////////////////////////////////////////////////
template <typename D0, typename D1>
class domain_intersection
{
public:
  using return_type =
    typename std::conditional<is_contiguous<D1>::value, D0, D1>::type;

private:
  // D1 is contiguous
  static return_type intersection_helper(D0 const& d0, D1 const& d1,
                                         std::true_type)
  {
    return d0 & d1;
  }

  // D1 is not contiguous
  static return_type intersection_helper(D0 const& d0, D1 const& d1,
                                         std::false_type)
  {
    return d1 & d0;
  }

public:
  return_type operator()(D0 const& d0, D1 const& d1) const
  {
    return intersection_helper(d0,d1,is_contiguous<D1>());
  }
};

} // namespace detail


namespace result_of {

template <typename D0, typename D1>
struct intersect
{
  typedef typename detail::domain_intersection<D0,D1>::return_type type;
};

} // namespace result_of



//////////////////////////////////////////////////////////////////////
/// @brief Function to compute the intersection between two given
///        domains (@p d0 and @p d1).
//////////////////////////////////////////////////////////////////////
template <typename D0, typename D1>
typename result_of::intersect<D0,D1>::type
intersect(D0 const& d0, D1 const& d1)
{
  return detail::domain_intersection<D0,D1>()(d0, d1);
}

} // stapl namespace

#endif /*STAPL_DOMAINS_INTERSECT_HPP*/
