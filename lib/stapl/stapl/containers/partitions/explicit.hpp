/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_EXPLICIT_PARTITION_HPP
#define STAPL_EXPLICIT_PARTITION_HPP

#include <stapl/domains/indexed.hpp>
#include <stapl/containers/type_traits/index_bounds.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Partition a one-dimensional domain explicitly into subdomains
/// that are stored in a container.
///
/// @tparam Dom Type of the domain to be partitioned
/// @tparam SubDom Type of the subdomains that are produced
/// @tparam Storage Container that explicitly stores the subdomains
//////////////////////////////////////////////////////////////////////
template <typename Dom,typename SubDom=Dom,
          typename Storage=std::vector<Dom> >
class explicit_partition
{
   typedef Storage                             vec_dom_type;

public:
  /// The original domain to partition
  typedef Dom                                  view_domain_type;
  /// The domain of the partitioner itself (i.e., the domain [0, ..., p-1])
  typedef indexed_domain<size_t>               domain_type;
  /// Type of the subdomains produced by the partition
  typedef SubDom                               value_type;
  /// Type of the GIDs in the subdomains
  typedef typename value_type::index_type      gid_type;
  /// Type used to describe the i'th subdomain
  typedef typename domain_type::index_type     index_type;

  /// Explict storage for the subdomains
  vec_dom_type     m_part;
  /// The original domain to partition
  view_domain_type m_domain;

  explicit_partition() = default;

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a partition of a domain based on an explicit container of
  /// subdomains.
  /// @param original_dom The original domain to partition
  /// @param vdom Container of subdomains
  //////////////////////////////////////////////////////////////////////
  explicit_partition(Dom const& original_dom, vec_dom_type const& vdom)
    : m_part(vdom), m_domain(original_dom)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the subdomain at a given index. This is the inverse
  /// of @ref find.
  /// @param idx The index of the subdomain, which should be from 0 to
  /// @ref size() - 1.
  //////////////////////////////////////////////////////////////////////
  value_type operator[](size_t idx) const
  {
    return const_cast<explicit_partition*>(this)->m_part.operator[](idx);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the number of partitions generated.
  //////////////////////////////////////////////////////////////////////
  size_t size() const
  {
    return m_part.size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the index of the partition that contains the gid @c g
  /// @param g gid to find
  /// @return The index of the subdomain that contains @c g
  //////////////////////////////////////////////////////////////////////
  index_type find(gid_type const& g) const
  {
    size_t found_at = 0;
    for (typename vec_dom_type::const_iterator it = m_part.begin();
         it != m_part.end(); ++it, ++found_at) {
      if (it->contains(g)) return found_at;
    }
    return index_bounds<index_type>::invalid();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Determine which partition has the elements referenced
  ///        for the given domain.
  ///
  /// The returned information is a collection (possibly empty) of
  /// pairs. Each pair contains information about which partitions are
  /// included in the given domain and how they are included (True: if
  /// is fully contained, False: if is partially included). The returned
  /// collection only has elements if there is at least one partition
  /// that contains elements on the given domain.
  ///
  /// @par Example:
  ///    Partition: [0..3],[4..6],[7..9],[10..13]<br/>
  ///    Given domain: [2..9]<br/>
  ///    Returns:  {([0..0],False),([1..2],True)}<br/>
  ///
  /// @param dom Domain to compare
  /// @param mfg Mapping function generator used to get the associated
  ///            mapping function to each partition. The generated
  ///            mapping function is used to project generated
  ///            partitioned domains into the given domain.
  /// @return a vector of pairs.
  //////////////////////////////////////////////////////////////////////
  template <typename ODom, typename MFG>
  std::vector<std::pair<domain_type,bool> >
  contained_in(ODom const& dom, MFG)
  {
    std::vector<std::pair<domain_type,bool> > doms;

    for (index_type j = 0; j < this->size(); ++j) {
      value_type domj = m_part[j];
      value_type tmpdom = domj & dom;
      if (tmpdom.size() == domj.size()) {
        doms.push_back(std::make_pair(domain_type(j,j),true));
      }
      else if (dom.contains(domj.first())) {
        doms.push_back(std::make_pair(domain_type(j,j),false));
      }
    }
    return doms;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Get the partition's domain [0, .., p-1]
  //////////////////////////////////////////////////////////////////////
  domain_type domain() const
  {
    return domain_type(0, size()-1);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the original domain that was used to create subdomains.
  //////////////////////////////////////////////////////////////////////
  value_type const& global_domain() const
  {
    return m_domain;
  }

  void define_type(typer& t)
  {
    t.member(m_part);
    t.member(m_domain);
  }

};

} // namespace stapl

#endif /* STAPL_EXPLICIT_PARTITION_HPP */
