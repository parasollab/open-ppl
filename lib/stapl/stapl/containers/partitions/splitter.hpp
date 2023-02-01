/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_CONTAINERS_SPLITTER_PARTITION_HPP
#define STAPL_CONTAINERS_SPLITTER_PARTITION_HPP

#include <vector>
#include <algorithm>
#include <stapl/domains/indexed.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Partition a one-dimensional domain explicitly into subdomains
/// based on a fixed set of splitter GIDs.
///
/// If the original domain is [0..99] and the set of splitters
/// is {6, 25, 88}, then the resulting subdomains produced will be [0..5],
/// [6..24], [25..87], [88..99].
///
/// @tparam Dom Type of the domain to be partitioned
/// @tparam SplitterType Container that explicitly stores the splitters
//////////////////////////////////////////////////////////////////////
template<typename Dom,
         typename SplitterType = std::vector<typename Dom::index_type> >
class splitter_partition
{
  typedef SplitterType                  vec_dom_type;

public:
  /// The original domain to partition
  typedef Dom                           view_domain_type;
  /// The domain of the partitioner itself (i.e., the domain [0, ..., p-1])
  typedef indexed_domain<size_t>        domain_type;
  /// Type of the subdomains produced by the partition
  typedef Dom                           value_type;
  /// Type of the GIDs in the subdomains
  typedef typename Dom::index_type      gid_type;
  /// Type used to describe the i'th subdomain
  typedef size_t                        index_type;

  /// Container of splitters
  vec_dom_type      m_part;
  /// Original domain
  view_domain_type  m_domain;
  /// Flag indicating whether or not empty subdomains are allowed
  bool              m_allow_empties;

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a partition of a domain based on splitters.
  ///
  /// The container of splitters must be nonempty and sorted. In addition,
  /// if allow_empties is not set, there must be no duplicate splitters.
  ///
  /// @param original_dom The original domain to partition.
  /// @param vdom A vector of splitters
  /// @param allow_empties Flag indicating whether empty domains is allowed.
  /// Effectively, the same splitter may not appear twice if allow_empties
  /// is false.
  //////////////////////////////////////////////////////////////////////
  splitter_partition(Dom const& original_dom, vec_dom_type const& vdom,
                     bool allow_empties = false)
    : m_part(vdom), m_domain(original_dom), m_allow_empties(allow_empties)
  {
    stapl_assert(vdom.size(), "No splitters defined");

    if (m_allow_empties)
    {
      stapl_assert(std::adjacent_find(vdom.begin(),vdom.end(),
        std::greater<typename vec_dom_type::value_type>())==vdom.end(),
        "Splitter is not sorted");
    }
    else
    {
      stapl_assert(std::adjacent_find(vdom.begin(),vdom.end(),
        std::greater_equal<typename vec_dom_type::value_type>())==vdom.end(),
        "Splitter is not sorted or has duplicates");
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the subdomain at a given index. This is the inverse
  /// of @ref find.
  /// @param idx The index of the subdomain, which should be from 0 to
  /// @ref size() - 1.
  //////////////////////////////////////////////////////////////////////
  value_type operator[](index_type idx) const
  {
    size_t tempsz=this->size();
    stapl_assert(idx<tempsz,"Index out of bounds");

    gid_type lower,upper;
    if (idx == 0)             //first partition
    {
      lower = m_domain.first();

      if (lower == m_part[0])
      {
        if (m_allow_empties)
          return value_type();
        else
        {
          if (m_part.size() > 1)
            upper = m_domain.advance(m_part[1], -1);
          else
            upper = m_domain.last();
        }
      }
      else
        upper = m_domain.advance(m_part[0], -1);
    }
    else if (idx == tempsz-1) //last partition
    {
      size_t vec_last_ix = m_part.size()-1;

      upper = m_domain.last();
      lower = m_part[vec_last_ix];

      if (m_domain.advance(upper, 1) == lower)
      {
        if (m_allow_empties)
          return value_type();
        else
          lower = m_part[vec_last_ix-1];
      }
    }
    else                      //some middle partition
    {
      index_type temp = idx;
      if (!m_allow_empties && m_part[0] == m_domain.first())
        ++temp;

      lower = m_part[temp-1];
      upper = m_part[temp];

      if (lower == upper)
        return value_type();
      else
        upper = m_domain.advance(m_part[temp], -1);
    }

    return value_type(lower, upper);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the number of partitions generated.
  //////////////////////////////////////////////////////////////////////
  size_t size() const
  {
    //an extra partition will occur before the first index
    if (m_allow_empties)
      return m_part.size()+1;

    //but that partition and the last partition may be empty
    size_t last_ix = m_part.size()-1;
    return last_ix
      + (m_part[0] == m_domain.first() ? 0 : 1)
      + (m_part[last_ix] == m_domain.advance(m_domain.last(), 1) ? 0 : 1);
  }

protected:
  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the index of the partition that contains the gid @c g.
  /// @param g gid to find
  /// @param left Whether we should search left-to-right or right-to-left
  /// in the partition list. If left is true, the index is with respect
  /// to the first partition. Otherwise, the index is with respect to the
  /// last partition, traversing backward.
  /// @return The index of the subdomain that contains @c g
  //////////////////////////////////////////////////////////////////////
  index_type in_domain(gid_type const& g, bool left)
  {
    if (!m_domain.contains(g))
      return index_bounds<index_type>::invalid();

    size_t dist;
    if (left) {
      if (g == m_domain.first()) {
        dist = 0;
      }
      else {
        dist = std::distance(m_part.begin(),
          std::lower_bound(m_part.begin(), m_part.end(), g));

        if (dist < m_part.size() && g == m_part[dist])
          ++dist;

        if (!m_allow_empties && m_part[0] == m_domain.first())
          --dist;
      }
    }
    else {
      if (g == m_domain.last()) {
        dist = this->size()-1;
      }
      else {
        dist = std::distance(m_part.begin(),
          std::upper_bound(m_part.begin(), m_part.end(), g));

        if (!m_allow_empties && m_part[0] == m_domain.first())
          --dist;
      }
    }

    return dist;
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Get the partition's domain [0, .., p-1].
  //////////////////////////////////////////////////////////////////////
  domain_type domain() const
  {
    return domain_type(this->size());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the original domain that was used to create subdomains.
  //////////////////////////////////////////////////////////////////////
  value_type const& global_domain() const
  {
    return m_domain;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the index of the partition that contains the gid @c g.
  /// @param g gid to find
  /// @return The index of the subdomain that contains @c g
  //////////////////////////////////////////////////////////////////////
  index_type find(gid_type const& g)
  {
    return in_domain(g,true);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc explicit_partition::contained_in
  //////////////////////////////////////////////////////////////////////
  template <typename ODom, typename MFG>
  std::vector<std::pair<domain_type,bool> >
  contained_in(ODom const& dom, MFG x)
  {
    std::vector<std::pair<domain_type,bool> > doms;

    index_type first = in_domain(dom.first(), true);
    index_type last = in_domain(dom.last(), false);

    value_type tmp_dom = this->operator[](last);
    if (tmp_dom.last() == dom.last())
    {
      if (this->operator[](first).size()>0 &&
          dom.first() != this->operator[](first).first())
        ++first;

      if (first <= last) {
        doms.push_back(std::make_pair(domain_type(first,last), true));
      }
    }
    else
    {   // the last partition is only partially contained
      if (this->operator[](first).size()>0 && dom.first()
          != this->operator[](first).first())
        ++first;

      if (first < last) {
        doms.push_back(std::make_pair(domain_type(first,last-1), true));
      }
      if (first <= last) {
        if (first == last && tmp_dom.contains(dom.first())&&
            tmp_dom.contains(dom.last()))
          return doms;
        doms.push_back(std::make_pair(domain_type(last,last), false));
      }
    }
    return doms;
  }

};

} // stapl namespace

#endif /* STAPL_SPLITTER_PARTITION_HPP */
