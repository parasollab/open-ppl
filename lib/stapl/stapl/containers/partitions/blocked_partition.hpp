/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_CONTAINERS_PARTITIONS_BLOCK_PARTITION_HPP
#define STAPL_CONTAINERS_PARTITIONS_BLOCK_PARTITION_HPP

#include <stapl/domains/indexed.hpp>
#include <stapl/containers/type_traits/index_bounds.hpp>
#include <vector>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Partition a one-dimensional domain into subdomains with a
/// fixed block size.
///
/// If the given domain contains the GIDs [0, 1, ..., n-1] and the block
/// size is b, then the number of subdomains generated is n/b.
/// In the case that the number of elements does not evenly divide the
/// block size, the remaining elements will be placed in the last partition
/// or ignored completely, based on a policy specified at construction.
///
/// @tparam Dom Type of the domain to be partitioned
/// @todo Rename this to blocked_partition
/// @todo Reevaluate the "throw leftovers into last bucket" policy
/// @todo Implement contained_in
//////////////////////////////////////////////////////////////////////
template <typename Dom>
class block_partitioner
{
public:
  /// The original domain to partition
  typedef Dom                                          view_domain_type;
  /// The domain of the partitioner itself (i.e., the domain [0, ..., p-1])
  typedef indexed_domain<size_t>                       domain_type;
  /// Type of the subdomains produced by the partition
  typedef Dom                                          value_type;
  /// Type of the GIDs in the subdomains
  typedef typename value_type::index_type              gid_type;
  /// Type used to describe the i'th subdomain
  typedef typename domain_type::index_type             index_type;

private:
  /// Original domain
  view_domain_type  m_domain;
  /// Number of elements in each subdomain
  size_t            m_block_size;
  /// Boolean dictating policy for leftover elements
  bool              m_ignore_last;
  /// Total number of partitions generated
  size_t            m_size;

public:
  block_partitioner()
   : m_block_size(),
     m_ignore_last(),
     m_size()
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a partition of a domain with a certain block size.
  ///
  /// @param domain The original domain to partition.
  /// @param block_size The size of each subdomain (except possibly the last
  /// subdomain)
  /// @param ignore Flag indicating whether the leftover elements should
  /// be included in the partition, or stored in the last subdomain.
  /// If ignore is false, all subdomains will have the same size.
  //////////////////////////////////////////////////////////////////////
  block_partitioner(view_domain_type const& domain, size_t block_size,
                    bool ignore = false)
    : m_domain(domain), m_block_size(block_size), m_ignore_last(ignore)
  {
    stapl_assert(m_block_size > 0, "Block size should be greater than 0");
    size_t tempsize = (size_t)(m_domain.size() / m_block_size);
    size_t remainder = m_domain.size() % m_block_size;
    size_t final_size = tempsize + (remainder == 0 ? 0 : 1);
    if (m_ignore_last==true && remainder!=0)
      final_size--;
    m_size=final_size;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Copy constructor using instead the given @p domain.
  //////////////////////////////////////////////////////////////////////
  block_partitioner(view_domain_type const& domain,
                    block_partitioner const& other)
    : m_domain(domain),
      m_block_size(other.m_block_size),
      m_ignore_last(other.m_ignore_last)
  {
    stapl_assert(m_block_size > 0, "Block size should be greater than 0");
    size_t tempsize = (size_t)(m_domain.size() / m_block_size);
    size_t remainder = m_domain.size() % m_block_size;
    size_t final_size = tempsize + (remainder == 0 ? 0 : 1);
    if (m_ignore_last==true && remainder!=0)
      final_size--;
    m_size=final_size;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a partition without a domain with a certain block size.
  ///
  /// The partition created will be unusable until a domain is provided
  /// via @ref set_domain.
  ///
  /// @param block_size The size of each subdomain (except possibly the last
  /// subdomain)
  /// @param ignore Flag indicating whether the leftover elements should
  /// be included in the partition, or stored in the last subdomain.
  /// If ignore is false, all subdomains will have the same size.
  /// @todo Should this constructor be available?
  //////////////////////////////////////////////////////////////////////
  block_partitioner(size_t block_size, bool ignore = false)
    : m_block_size(block_size),
      m_ignore_last(ignore),
      m_size()
  {
    stapl_assert(block_size > 0, "Size of block should be > 0");
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Set the global domain to partition
  /// @param domain The domain to partition
  //////////////////////////////////////////////////////////////////////
  void set_domain(view_domain_type const& domain)
  {
    m_domain = domain;
    size_t tempsize = (size_t)(m_domain.size() / m_block_size);
    size_t remainder = m_domain.size() % m_block_size;
    size_t final_size = tempsize + (remainder == 0 ? 0 : 1);
    if (m_ignore_last==true && remainder!=0)
      final_size--;
    m_size=final_size;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the subdomain at a given index. This is the inverse
  /// of @ref find.
  /// @param idx The index of the subdomain, which should be from 0 to
  /// @ref size() - 1.
  /// @return The subdomain at idx
  //////////////////////////////////////////////////////////////////////
  value_type operator[](index_type idx) const
  {
    stapl_assert(idx<this->size(), "Index out of bounds");
    index_type first= idx*m_block_size;
    index_type last= first+m_block_size;
    if (idx== m_size-1 && m_ignore_last==false)
      return value_type(m_domain.advance(m_domain.first(),first),
                        m_domain.last());
    return value_type(m_domain.advance(m_domain.first(),first),
                      m_domain.advance(m_domain.first(),last-1));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the number of partitions generated.
  //////////////////////////////////////////////////////////////////////
  size_t size() const
  {
    return m_size;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Get the partition's domain [0, .., p-1]
  //////////////////////////////////////////////////////////////////////
  domain_type domain() const
  {
    return domain_type(size());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the original domain that was used to create subdomains.
  //////////////////////////////////////////////////////////////////////
  value_type const& global_domain() const
  {
    return m_domain;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the index of the partition that contains the gid @c g
  /// @param g gid to find
  /// @return The index of the subdomain that contains @c g
  //////////////////////////////////////////////////////////////////////
  index_type find(gid_type const& g) const
  {
    if (!m_domain.contains(g))
      return index_bounds<index_type>::invalid();
    size_t found = m_domain.distance(m_domain.first(),g);
    return found/m_block_size;
  }

  // TODO: This method is required for the partition to be used in a view
  /*returns a vector of pairs indicating if given domain is fully contained
   * by the sub domains
  template<typename ODom, typename MFG>
  std::vector<std::pair<domain_type,bool> >
  contained_in(const ODom& dom, MFG x)
  {
    std::vector<std::pair<domain_type,bool> > doms;
    index_type temp=in_domain(dom.first(),false);
    index_type temp1=in_domain(dom.last(),false);
    if ((*this)[temp].first()<dom.first())
    {
      doms.push_back(std::make_pair(domain_type(temp,temp),false));
      temp+=1;
    }
    if (temp<temp1)
    {
      if ((*this)[temp1].last() > dom.last())
      {
        --temp1;
      }
      doms.push_back(std::make_pair(domain_type(temp,temp1),true));
    }
    return doms;
  }*/
}; //class block_partitioner

} //stapl namespace

#endif // STAPL_CONTAINERS_PARTITIONS_BLOCK_PARTITION_HPP
