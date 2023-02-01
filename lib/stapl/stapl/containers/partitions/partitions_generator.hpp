/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_PARTITIONS_GENERATOR_HPP
#define STAPL_CONTAINERS_PARTITIONS_GENERATOR_HPP

#include <stapl/runtime.hpp>
#include <stapl/utility/tuple.hpp>

namespace stapl {

namespace partitions_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Function object that initializes an n-dimensional
/// tuple of partitions to  n balanced partitions, each partitioned by
/// the number of locations.
/// @tparam I The dimension of the partition that is currently being
/// populated.
/// @tparam Domain The domain that is to be partitioned
/// @tparam Tuple The type of the tuple to populate
//////////////////////////////////////////////////////////////////////
template<int I, typename Domain, typename Tuple>
struct partitions_generator
{
  Domain m_domain;
  location_type m_nlocs;

  partitions_generator(Domain const& domain)
    : m_domain(domain), m_nlocs(get_num_locations())
  { }

  partitions_generator(Domain const& domain, location_type nlocs)
    : m_domain(domain), m_nlocs(nlocs)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Function operator to initialize the tuple.
  /// @param t The tuple of partitions that is to be initialized.
  /// @todo This can possibly be optimized to pass-by-const ref if we don't
  /// discard qualifiers somewhere down the chain
  //////////////////////////////////////////////////////////////////////
  Tuple operator()(Tuple t)
  {
    typedef typename tuple_element<I-1, Tuple>::type   dom_t;

    get<I-1>(t) = dom_t(m_domain.template get_domain<I-1>(),
                        m_nlocs);

    return partitions_generator<I-1, Domain, Tuple>(m_domain, m_nlocs)(t);
  }

  /////////////////////////////////////////////////////////////////////
  /// @brief Function operator to initialize the tuple.
  /// @param t The tuple of partitions that is to be initialized.
  /// @param nparts The number of partitions to generate in this dimension
  /// @todo This can possibly be optimized to pass-by-const ref if we don't
  /// discard qualifiers somewhere down the chain.
  //////////////////////////////////////////////////////////////////////
  template <typename NParts>
  Tuple operator()(Tuple t, NParts nparts)
  {
    size_t p = get<I-1>(nparts);
    typedef typename tuple_element<I-1, Tuple>::type   dom_t;

    get<I-1>(t) = dom_t(m_domain.template get_domain<I-1>(), p);

    return
      partitions_generator<I-1, Domain, Tuple>(m_domain, m_nlocs)(t, nparts);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Base case to end recursion for @ref partitions_generator
//////////////////////////////////////////////////////////////////////
template<typename Domain, typename Tuple>
struct partitions_generator<1, Domain, Tuple>
{
  Domain m_domain;
  location_type m_nlocs;

  partitions_generator(Domain const& domain)
    : m_domain(domain), m_nlocs(get_num_locations())
  { }

  partitions_generator(Domain const& domain, location_type nlocs)
    : m_domain(domain), m_nlocs(nlocs)
  { }

  Tuple operator()(Tuple t)
  {
    typedef typename tuple_element<0, Tuple>::type   dom_t;
    get<0>(t) = dom_t(m_domain.template get_domain<0>(), m_nlocs);
    return t;
  }

  template <typename NParts>
  Tuple operator()(Tuple t, NParts nparts)
  {
    typedef typename tuple_element<0, Tuple>::type dom_t;
    get<0>(t) = dom_t(m_domain.template get_domain<0>(), get<0>(nparts));
    return t;
  }
};

} // namespace partitions_impl

} // namespace stapl

#endif // STAPL_CONTAINERS_PARTITIONS_GENERATOR_HPP
