/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_DOMAINS_INVERSE_DOMAIN_HPP
#define STAPL_DOMAINS_INVERSE_DOMAIN_HPP

#include <stapl/views/mapping_functions/mapping_functions.hpp>
#include <stapl/domains/indexed.hpp>
#include <stapl/utility/static_match.hpp>

namespace stapl {

namespace metadata {

//////////////////////////////////////////////////////////////////////
/// @brief Helper functor to compute the inverse domain of a given
///        domain when the mapping function is both monotonic and
///        injective.
///
/// The functor maps indices in the given domain @p dom to indices of
/// a domain of type @c DomT, using the provided mapping function @p
/// imf.
//////////////////////////////////////////////////////////////////////
template <typename DomT, typename DomS, typename IMF>
struct monotonic_injective_domain_inversion
{
  static DomT apply(DomS const& dom, IMF const& imf)
  {
    auto first = dom.first();
    auto last = dom.last();

    // advance the first element forwards until we find an element
    // that has an inverse defined
    while (!imf.defined(first) && first != dom.last())
      first = dom.advance(first, 1);

    // advance the last element backwards until we find an element
    // that has an inverse defined, or we crossed to the first
    while (!imf.defined(last) && last != first)
      last = dom.advance(last, -1);

    // if we have converged to the same element and it's not
    // defined, that means there is no inverse in this range so
    // we'll return an empty domain
    if (first == last && !imf.defined(first))
      return DomT();

    // otherwise, construct a domain with the inverse of the first and last
    // we just found that are defined
    DomT d(imf(first), imf(last));
    return d;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper functor to compute the inverse domain of a given
///        domain when the mapping function is both monotonic and
///        bijective.
///
/// The functor maps indices in the given domain @p dom to indices of
/// a domain of type @c DomT, using the provided mapping function @p
/// imf.
//////////////////////////////////////////////////////////////////////
template <typename DomT, typename DomS, typename IMF>
struct monotonic_bijective_domain_inversion
{
  static DomT apply(DomS const& dom, IMF const& imf)
  {
    auto const first = dom.first();
    auto const last = dom.last();

    DomT d(imf(first), imf(last));

    return d;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper functor to compute the inverse domain of a given
///        domain when the mapping function is both monotonic and
///        surjective.
///
/// @todo  The semantics of this operation is not well defined.
//////////////////////////////////////////////////////////////////////
template <typename DomT, typename DomS, typename IMF>
struct monotonic_surjective_domain_inversion
{
  template<typename D>
  static DomT apply(D const& dom, IMF const&)
  {
    static_assert(sizeof(D) == 0,
      "Inversion not implemented for non-injective surjective "
      "mapping functions."
    );
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization when the provided mapping function is
///        identity.
//////////////////////////////////////////////////////////////////////
template <typename DomT, typename DomS, typename IMF>
struct identity_domain_inversion
{
  static DomT apply(DomT const& dom, IMF const&)
  {
    return dom;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to select the appropriate domain inversion
///        algorithm to use based on the injectivity of the mapping
///        function.
//////////////////////////////////////////////////////////////////////
template<typename DomT, typename DomS, typename IMF>
struct select_domain_inversion
{
  // the type of the forward mapping
  typedef typename IMF::inverse forward;
  typedef typename is_injective<forward>::type is_injective;
  typedef typename is_surjective<forward>::type is_surjective;
  typedef typename is_bijective<forward>::type is_bijective;

  typedef std::tuple<
    // bijective
    std::pair<
      std::tuple<std::true_type, dont_care, dont_care>,
      monotonic_bijective_domain_inversion<DomT, DomS, IMF>
    >,
    // injective and non-surjective
    std::pair<
      std::tuple<std::false_type, std::true_type, std::false_type>,
      monotonic_injective_domain_inversion<DomT, DomS, IMF>
    >,
    // non-injective and surjective
    std::pair<
      std::tuple<std::false_type, std::false_type, std::true_type>,
      monotonic_surjective_domain_inversion<DomT, DomS, IMF>
    >,
    // injective and surjective (bijective)
    std::pair<
      std::tuple<dont_care, std::true_type, std::true_type>,
      monotonic_bijective_domain_inversion<DomT, DomS, IMF>
    >
  > options;

  typedef typename static_match<
    std::tuple<is_bijective, is_injective, is_surjective>, options
  >::type type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization when the provided mapping function is
///        identity.
//////////////////////////////////////////////////////////////////////
template<typename DomT, typename DomS, typename T>
struct select_domain_inversion<DomT, DomS, f_ident<T> >
{
  typedef identity_domain_inversion<DomT, DomS, f_ident<T> > type;
};

} // namespace detail


//////////////////////////////////////////////////////////////////////
/// @brief Function to compute the inverse mapping of a given domain
///        (@p dom) applying the provided mapping function @p imf.
//////////////////////////////////////////////////////////////////////
template <typename DomT, typename DomS, typename IMF>
DomT invert_domain(DomS const& dom, IMF const& imf)
{
  typedef typename metadata::select_domain_inversion<
    DomT, DomS, IMF
  >::type invert_t;

  static_assert(!std::is_same<invert_t, not_matched>::value,
    "The view's mapping function defines an inverse, but does not claim "
    "its injectivity.");

  return invert_t::apply(dom,imf);
}

} // namespace stapl

#endif /* STAPL_DOMAINS_INVERSE_DOMAIN_HPP */
