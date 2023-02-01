/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_UNORDERED_MAP_PARTITION_HPP
#define STAPL_CONTAINERS_UNORDERED_MAP_PARTITION_HPP

#include <stapl/domains/indexed.hpp>
#include <stapl/containers/type_traits/index_bounds.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Partition a one-dimensional domain  into subdomains
/// that are stored in a container with the hash.
///
/// @tparam Hasher The hasher used by the unordered_map.
/// @tparam Dom Type of the domain to be partitioned
/// @tparam SubDom Type of the subdomains that are produced
//////////////////////////////////////////////////////////////////////
template <typename Hasher, typename Dom,typename SubDom=Dom >
class unordered_map_partition
{

public:
  /// The original domain to partition
  typedef Dom                                  view_domain_type;
  /// The domain of the partition itself (i.e., the domain [0, ..., p-1])
  typedef indexed_domain<size_t>               domain_type;
  /// Type of the subdomains produced by the partition
  typedef SubDom                               value_type;
  /// Type of the GIDs in the subdomains
  typedef typename value_type::index_type      gid_type;
  /// Type used to describe the i'th subdomain
  typedef typename domain_type::index_type     index_type;

  /// The number of partitions
  size_t m_numpart;
 /// The hasher used.
  Hasher m_hash;
  /// The original domain to partition
  view_domain_type m_domain;

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a partition of a domain based on the hash function and a
  ///   specific number of partitions.
  /// @param hasher_func The hasher object.
  /// @param num_part The number of partitions. By default the number of
  ///   locations.
  //////////////////////////////////////////////////////////////////////
  unordered_map_partition(Hasher const& hasher_func = Hasher(),
                          size_t const& num_part = stapl::get_num_locations())
   : m_numpart(num_part), m_hash(hasher_func)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the subdomain at a given index. This is the inverse
  /// of @ref find.
  /// @param idx The index of the subdomain, which should be from 0 to
  /// @ref size() - 1.
  //////////////////////////////////////////////////////////////////////
  value_type operator[](size_t idx) const
  {
    stapl_assert(false,"No operator [ ] for this partitioner");
    return value_type();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the number of partitions generated.
  //////////////////////////////////////////////////////////////////////
  size_t size() const
  {
    return m_numpart;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return hash function
  //////////////////////////////////////////////////////////////////////
  Hasher hash_function() const
  {
    return m_hash;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the index of the partition that contains the gid @c g
  /// @param g gid to find
  /// @return The index of the subdomain that contains @c g
  //////////////////////////////////////////////////////////////////////
  index_type find(gid_type const& g) const
  {
    return m_hash(g) % m_numpart;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Get the partition's domain [0, .., p-1]
  //////////////////////////////////////////////////////////////////////
  domain_type domain() const
  {
    return domain_type(0, m_numpart-1);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the original domain that was used to create subdomains.
  //////////////////////////////////////////////////////////////////////
  view_domain_type global_domain() const
  {
    return view_domain_type();
  }

  void define_type(typer& t)
  {
    t.member(m_numpart);
    t.member(m_hash);
    t.member(m_domain);
  }

};

} // namespace stapl

#endif /* STAPL_CONTAINERS_UNORDERED_MAP_PARTITION_HPP */
