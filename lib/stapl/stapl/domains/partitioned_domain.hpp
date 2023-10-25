/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_DOMAINS_PARTITIONED_DOMAIN_HPP
#define STAPL_DOMAINS_PARTITIONED_DOMAIN_HPP

#include <stapl/runtime.hpp>

namespace stapl {

template<typename Domain>
class partitioned_domain
{
private:
  Domain m_domain;

  class subdomain_holder
  {
  private:
    Domain m_domain;

  public:
    typedef Domain           value_type;
    typedef Domain const&    reference;
    typedef size_t           size_type;
    typedef unsigned int     index_type;

    explicit
    subdomain_holder(Domain const& dom)
      : m_domain(dom)
    { }

    reference operator[](index_type idx) const
    {
      stapl_assert(idx == 0, "invalid subdomain id");

      return m_domain;
    }

    size_type size(void) const
    {
      return 1;
    }
  };

public:
  typedef size_t                 size_type;
  typedef subdomain_holder       subdomains_view_type;

  partitioned_domain(Domain const& dom)
    : m_domain(dom)
  { }


  Domain const& domain(void) const
  {
    return m_domain;
  }


  subdomain_holder local_subdomains(void) const
  {
    typedef std::size_t    dom_size_t;
    typedef typename Domain::index_type   index_t;

    const dom_size_t n_partitions = get_num_locations();
    const dom_size_t idx          = get_location_id();

    index_t first = index_bounds<index_t>::invalid();
    index_t last  = index_bounds<index_t>::invalid();

    if (n_partitions > m_domain.size())
    {
      if (idx < m_domain.size())
      {
        first = m_domain.advance(m_domain.first(), idx);
        last = m_domain.advance(m_domain.first(), idx);
      }
    }
    else {
      // remainder
      dom_size_t rem      = m_domain.size() % n_partitions;
      // block size lower
      dom_size_t bszl     = m_domain.size() / n_partitions;

      // block size greater
      dom_size_t bszg     = bszl + (rem > 0 ? 1 : 0);

      first = (idx <= rem)
             ? m_domain.advance(m_domain.first(), idx*bszg)
             : m_domain.advance(m_domain.first(), (rem*bszg + (idx-rem)*bszl));

      last = (idx == n_partitions-1)
             ? m_domain.last()
             : m_domain.advance(first, ((idx < rem) ? bszg-1 : bszl-1));
    }
    return subdomain_holder(Domain(first, last, m_domain));
  }


  size_type num_subdomains(void) const
  {
    return stapl::get_num_locations();
  }


  void define_type(typer& t)
  {
    t.member(m_domain);
  }
};

} // namespace stapl

#endif // STAPL_DOMAINS_PARTITIONED_DOMAIN_HPP

