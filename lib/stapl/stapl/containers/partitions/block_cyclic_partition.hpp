/*
 // Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
 // component of the Texas A&M University System.

 // All rights reserved.

 // The information and source code contained herein is the exclusive
 // property of TEES and may not be disclosed, examined or reproduced
 // in whole or in part without explicit written authorization from TEES.
 */

#ifndef STAPL_VIEW_BLOCK_CYCLIC_PARTITION_HPP
#define STAPL_VIEW_BLOCK_CYCLIC_PARTITION_HPP

#include <stapl/domains/domains.hpp>
#include <stapl/containers/type_traits/index_bounds.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Partition a one-dimensional domain cyclically into subdomains with a
/// fixed block size.
///
/// Example: Consider an original domain of [0...15], block size = 2 and
/// cyclicity = 4. This means that there will be 4 total partitions, each
/// partition containing 4 elements in runs of 2:
///
/// D = [ 0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15]
///      P0  P0  P1  P1  P2  P2  P3  P3  P0  P0  P1  P1  P2  P2  P3  P3
///
/// Note that the elements in a subdomain are not necessarily contiguous.
///
/// @tparam Dom Type of the domain to be partitioned
/// @todo This partition needs to be updated to the new partition interface
//////////////////////////////////////////////////////////////////////
template < typename Dom >
class blk_cyclic_part
{
public:
  /// The original domain to partition
  typedef Dom view_domain_type;
  /// The domain of the partitioner itself (i.e., the domain [0, ..., p-1])
  typedef indexed_domain< size_t > domain_type;
  /// Type of the subdomains produced by the partition
  typedef dom1Dbc< typename Dom::index_type > value_type;
  /// Type of the GIDs in the subdomains
  typedef typename value_type::index_type gid_type;
  /// Type used to describe the i'th subdomain
  typedef typename domain_type::index_type index_type;
private:
  /// Block size
  size_t m_blksz;
  /// Number of blocks to hop for a partition
  size_t m_cyclicity;
  /// Total size of the original domain
  size_t m_size;
  /// Original domain to partition
  Dom m_dom;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Create block-cyclic domain with a given domain, block size
  /// and cyclicity.
  /// @param dom The domain to partition
  /// @param blksz The size of contiguous chunks
  /// @param cyclicity The number of blocks to skip over until the next
  /// block for a partition. This is equivalent to the total number of
  /// partitions generated.
  //////////////////////////////////////////////////////////////////////
  blk_cyclic_part(Dom dom, size_t blksz = 1, size_t cyclicity = 1)
  {
    m_dom = dom;
    stapl_assert(blksz>0,"Block size should be > 0");
    stapl_assert(cyclicity>0,"Cyclicity shoudl be > 0");
    m_blksz = blksz;
    m_cyclicity = cyclicity;
    m_size = dom.size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the number of partitions generated.
  //////////////////////////////////////////////////////////////////////
  size_t
  size() const
  {
    return m_cyclicity;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the subdomain at a given index. This is the inverse
  /// of @ref find.
  /// @param idx The index of the subdomain, which should be from 0 to
  /// @ref size() - 1.
  /// @return The subdomain at idx
  //////////////////////////////////////////////////////////////////////
  value_type
  operator [](index_type idx) const
  {
    stapl_assert(idx<size(),"index out of bounds");
    size_t part_offset = m_dom.first() + m_blksz * idx;
    size_t coarse_chunks = (m_size / m_blksz) + (m_size % m_blksz > 0 ? 1 : 0);
    size_t temp = coarse_chunks % m_cyclicity;
    temp = temp == 0 ? (m_cyclicity) : temp;
    size_t remainder = m_size % m_blksz;
    if (remainder == 0)
    {
      remainder = m_blksz;
    }
    size_t num_row_elems = ((coarse_chunks / m_cyclicity) +
     ((coarse_chunks % m_cyclicity) > 0 ? 1 : 0)) - 1;

    size_t total;
    if (idx + 1 == temp)
    {
      total = num_row_elems * m_blksz + remainder;
    }
    else if (idx + 1 < temp)
    {
      total = (num_row_elems + 1) * m_blksz;
    }
    else
    {
      total = num_row_elems * m_blksz;
    }
    return value_type(m_blksz, m_cyclicity, total, part_offset);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the index of the partition that contains the gid @c g.
  /// @param g gid to find
  /// @return The index of the subdomain that contains @c g
  //////////////////////////////////////////////////////////////////////
  index_type find(gid_type const& g) const
  {
    if (!m_dom.contains(g))
      return index_bounds<index_type>::invalid();
    size_t temp = m_blksz * m_cyclicity;
    size_t offset_g = g - m_dom.first();
    size_t t_begin = (offset_g / temp) * temp;
    size_t diff = g - t_begin;
    size_t value = diff / m_blksz;
    return value;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the original domain that was used to create subdomains.
  //////////////////////////////////////////////////////////////////////
  Dom const& global_domain() const { return m_dom; }

};

} //namespace stapl
#endif
