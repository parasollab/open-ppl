/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_BALANCED_PARTITION_HPP
#define STAPL_CONTAINERS_BALANCED_PARTITION_HPP

#include <stapl/domains/indexed.hpp>
#include <stapl/runtime.hpp>
#include <vector>

namespace stapl {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Base class for partitions that encapsulates the logic needed
/// to determine if shifting can be used in place of division to map a
/// GID to a partition id.
///
/// Currently only one-dimensional partitions are supported.
///
/// @todo Generalize to n-dimensional domains.
//////////////////////////////////////////////////////////////////////
template<int Dimension>
struct partition_base
{
protected:
  /// @brief The right-shift distance for gid to pid mapping when
  /// the number of partitions is a power of two.
  size_t     m_shift_dist;

  /// The size of a partition, if all partitions are the same size.
  size_t     m_small_part_size;

  /// @brief The number of partitions that are one element larger in cases
  /// where the domain size isn't evenly divisible by the number of partittions.
  size_t     m_num_large_parts;

  /// Indicate whether shifting optimization is used
  bool       m_shift;

  template<typename Domain>
  void compute_shiftable(size_t, Domain const&)
  {
    m_shift = false;
    m_shift_dist = 0;
    m_small_part_size = 0;
    m_num_large_parts = 0;
  }

public:
  void define_type(typer& t)
  {
    t.member(m_shift_dist);
    t.member(m_small_part_size);
    t.member(m_num_large_parts);
    t.member(m_shift);
  }
};

template<>
struct partition_base<1>
{
protected:
  /// @brief The right-shift distance for gid to pid mapping when
  /// the number of partitions is a power of two.
  size_t     m_shift_dist;

  /// The size of a partition, if all partitions are the same size.
  size_t     m_small_part_size;

  /// @brief The number of partitions that are one element larger in cases
  /// where the domain size isn't evenly divisible by the number of partittions.
  size_t     m_num_large_parts;

  /// Indicate whether shifting optimization is used
  bool       m_shift;

  template<typename Domain>
  void compute_shiftable(size_t num_parts, Domain const& domain)
  {
    using gid_type    = typename Domain::index_type;

    const bool zero_based = domain.first() ==
      identity_value<stapl::plus<gid_type>, gid_type>::value();

    auto dom_size = domain.size();

    if (dom_size <= num_parts && zero_based)
    {
      m_shift = true;
      m_shift_dist = 0;
      m_num_large_parts = 0;
      m_small_part_size = 1;
      return;
    }

    if (num_parts == 0)
    {
      m_shift = false;
      m_shift_dist = 0;
      m_num_large_parts = 0;
      m_small_part_size = dom_size;
      return;
    }

    // else
    auto num_bits     = std::log2(num_parts);
    auto num_dom_bits = std::log2(dom_size);

    // Shifting is enabled if the number of partitions is a power of two
    // and the GID domain begins at the value equivalent to 0.
    // The GID == 0 check can be relaxed at the expense of adding a
    // subtraction operation to the optimized path in find(gid).
    m_shift =
      zero_based &&
      (std::ceil(num_bits) - std::floor(num_bits) < 0.000001) &&
      (std::ceil(num_dom_bits) - std::floor(num_dom_bits) < 0.000001);

    // If the domain is [0, 1] and is partitioned in to two parts,
    // no shift is needed.
    if (m_shift)
    {
      // if there are two elements, shift 1 bit to map into a single partition,
      // otherwise don't shift.
      // For other element sizes, if there's only one partition then we shift
      // far enough so all elements map into partition 0.
      if (dom_size == 2)
        m_shift_dist = num_parts == 1 ? 1 : 0;
      else
        m_shift_dist = num_parts == 1 ?
                         std::ceil(std::log2(domain.last())) :
                         std::ceil(std::log2(domain.last())) - num_bits;
    }
    else
      m_shift_dist = 0;

    m_num_large_parts = dom_size % num_parts;
    m_small_part_size = dom_size / num_parts;
  }

public:
  void define_type(typer& t)
  {
    t.member(m_shift_dist);
    t.member(m_small_part_size);
    t.member(m_num_large_parts);
    t.member(m_shift);
  }
};

} // namespace detail


//////////////////////////////////////////////////////////////////////
/// @brief Partition a one-dimensional domain into balanced subdomains.
///
/// If the given domain contains the GIDs [0, 1, ..., n-1] and the number
/// of partitions to generate is p, then each subdomain will have
/// size n/p and will contain contiguous chunks of GID in domain order.
/// In the case that the number of elements does not evenly divide the
/// requested number of partitions, the remaining elements are evenly
/// distributed across the subdomains, guaranteeing that no two subdomains
/// differ in size by more than one.
///
/// @tparam Dom Type of the domain to be partitioned
//////////////////////////////////////////////////////////////////////
template <typename Dom,
          bool Integral = std::is_integral<typename Dom::index_type>::value>
struct balanced_partition
{
  /// The domain of the partitioner itself (i.e., the domain [0, ..., p-1])
  using domain_type = indexed_domain<size_t>;

  /// Type used to describe the i'th subdomain
  using index_type  = typename domain_type::index_type;

  /// Type of the subdomains produced by the partition
  using value_type  = Dom;

  /// Type of the GIDs in the subdomains
  using gid_type    = typename value_type::index_type;

  /// Type of the size of the subdomains
  using size_type   = typename value_type::size_type;

protected:
  /// The global original domain
  value_type m_domain;

  /// The number of partitions
  size_t     m_npart;

public:
  //////////////////////////////////////////////////////////////////////
  /// Create a partition with a domain and a partitioning factor.
  /// @param dom The domain to partition
  /// @param p The number of partitions to generate
  //////////////////////////////////////////////////////////////////////
  balanced_partition(value_type const& dom, size_t p = get_num_locations())
    : m_domain(dom), m_npart(p)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a partition with no domain. Leaves the partition in
  /// an inconsistent state and must be used in conjunction with @ref
  /// set_domain.
  //////////////////////////////////////////////////////////////////////
  balanced_partition()
    : m_npart(get_num_locations())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Copy constructor using instead the given @p domain.
  //////////////////////////////////////////////////////////////////////
  balanced_partition(value_type const& domain,
                     balanced_partition const& other)
    : m_domain(domain), m_npart(other.m_npart)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Get the partition's domain [0, .., p-1].
  //////////////////////////////////////////////////////////////////////
  domain_type domain() const
  {
    return domain_type(0, m_npart - 1);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Set the global domain to partition.
  /// @param dom The domain to partition
  //////////////////////////////////////////////////////////////////////
  void set_domain(value_type const& dom)
  {
    m_domain = dom;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the subdomain at a given index. This is the inverse
  /// of @ref find.
  /// @param idx The index of the subdomain, which should be from 0 to
  /// @ref size() - 1.
  //////////////////////////////////////////////////////////////////////
  value_type operator[](index_type idx) const
  {
    stapl_assert(idx < m_npart, "partition index out of bound\n");
    if (m_domain.size() <= idx)
      return value_type();

    // remainder
    size_t rem      = m_domain.size() % m_npart;
    // block size lower
    size_t bszl     = m_domain.size() / m_npart;
    // block size greater
    size_t bszg     = bszl + (rem > 0 ? 1 : 0);

    gid_type first = (idx <= rem)
           ? m_domain.advance(m_domain.first(), idx*bszg)
           : m_domain.advance(m_domain.first(), (rem*bszg + (idx-rem)*bszl));

    gid_type last = (idx==m_npart-1)
           ? m_domain.last()
           : m_domain.advance(first, ((idx < rem) ? bszg-1 : bszl-1));
    return value_type(first, last, m_domain);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the domain of gids that map to the specified partition
  ///   if the mapping from GID to partition id is recognized.
  ///
  /// This is functionally equivalent to the index operator for this partition.
  ///
  /// @param pid id of the partition whose domain is required
  //////////////////////////////////////////////////////////////////////
  boost::optional<value_type> try_get_domain(index_type pid) const
  {
    return this->operator[](pid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the number of partitions generated.
  //////////////////////////////////////////////////////////////////////
  size_t size() const
  {
    if (m_domain.size() < m_npart)
      return m_domain.size();
    return m_npart;
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
  index_type find(gid_type const& g) const
  {
    if (!m_domain.contains(g))
      return m_npart;

    const size_t rem  = m_domain.size() % m_npart;
    const size_t bszl = m_domain.size() / m_npart;
    const size_t bszg = bszl + (rem > 0 ? 1 : 0);

    const size_t d = m_domain.distance(m_domain.first(), g);
    const size_t i = (d <= rem*bszg) ? (d/bszg)
                               : rem + ((d - rem * bszg)/bszl);

    stapl_assert(i<m_npart, "calculated index is out of bounds");
    return i;
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
  /// @todo The is_contiguous specialization has a bug in it exposed
  /// by usage in set_elements().  Can be exposed with unique_copy
  /// valladolid benchmark.  Debug and enable.
  //////////////////////////////////////////////////////////////////////
  template <typename ODom, typename MFG>
  std::vector<std::pair<domain_type,bool> >
  contained_in(ODom const& dom, MFG const& mfg)
  {
    std::vector<std::pair<domain_type,bool>> doms;

#if 0
    if (dom.is_contiguous()) {
      // Inverse mapping

      // Index of partition containing first in target domain
      size_t cj = this->find(dom.first());

      // Index of partition containing last in target domain
      size_t dj = this->find(dom.last());

      // if domain is outside that covered by this partition
      if (cj >= m_npart)
        return doms;

      // First element in partition containing first in target domain.
      auto ai = mfg[cj]((*this)[cj].first());

      // Last element in partition containing last in target domain.
      auto bi = mfg[dj]((*this)[dj].last());


      if (cj < m_npart && ai<dom.first())
        ++cj;

      if (cj < dj) {
        if (dom.last()<bi) {
          doms.push_back(std::make_pair(domain_type(cj,dj-1),true));
          doms.push_back(std::make_pair(domain_type(dj,dj),false));
        }
        if (dom.last()==bi) {
          doms.push_back(std::make_pair(domain_type(cj,dj),true));
        }
      }

      if (cj == dj) {
        if (dom.last()<bi) {
          doms.push_back(std::make_pair(domain_type(cj,dj),false));
        }
        if (dom.last()==bi) {
          doms.push_back(std::make_pair(domain_type(cj,dj),true));
        }
      }
    }
    else
#endif
    {
      for (index_type j = 0; j < this->size(); ++j) {
        value_type domj = (*this)[j];
        value_type tmpdom = (domj & const_cast<ODom&>(dom));
        if (tmpdom.size() == domj.size()) {
          doms.push_back(std::make_pair(domain_type(j,j),true));
        }
        else
          if (dom.contains(domj.first())) {
            doms.push_back(std::make_pair(domain_type(j,j),false));
          }
        }
    }
    return doms;
  }

  void define_type(typer& t)
  {
    t.member(m_domain);
    t.member(m_npart);
  }
}; // struct balanced_partition


template <typename T>
struct balanced_partition<indexed_domain<T, 1>, true>
  : public detail::partition_base<1>
{
  using Dom = indexed_domain<T, 1>;

  /// The domain of the partitioner itself (i.e., the domain [0, ..., p-1])
  using domain_type = indexed_domain<size_t>;

  /// Type used to describe the i'th subdomain
  using index_type  = typename domain_type::index_type;

  /// Type of the subdomains produced by the partition
  using value_type  = Dom;

  /// Type of the GIDs in the subdomains
  using gid_type    = typename value_type::index_type;

  /// Type of the size of the subdomains
  using size_type   = typename value_type::size_type;

protected:
  /// The global original domain
  value_type m_domain;

  /// The number of partitions
  size_t     m_npart;

public:
  //////////////////////////////////////////////////////////////////////
  /// Create a partition with a domain and a partitioning factor.
  /// @param dom The domain to partition
  /// @param p The number of partitions to generate
  //////////////////////////////////////////////////////////////////////
  balanced_partition(value_type const& dom, size_t p = get_num_locations())
    : m_domain(dom), m_npart(p)
  {
    this->compute_shiftable(m_npart, m_domain);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a partition with no domain. Leaves the partition in
  /// an inconsistent state and must be used in conjunction with @ref
  /// set_domain.
  ///
  /// The data members related to shifting are initialized to 0
  /// because the domain is invalid.
  //////////////////////////////////////////////////////////////////////
  balanced_partition()
    : m_npart(get_num_locations())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Copy constructor using instead the given @p domain.
  //////////////////////////////////////////////////////////////////////
  balanced_partition(value_type const& domain,
                     balanced_partition const& other)
    : m_domain(domain), m_npart(other.m_npart)
  {
    this->compute_shiftable(m_npart, m_domain);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Get the partition's domain [0, .., p-1].
  //////////////////////////////////////////////////////////////////////
  domain_type domain() const
  {
    return domain_type(0, m_npart - 1);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Set the global domain to partition.
  /// @param dom The domain to partition
  //////////////////////////////////////////////////////////////////////
  void set_domain(value_type const& dom)
  {
    m_domain = dom;

    this->compute_shiftable(m_npart, m_domain);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the subdomain at a given index. This is the inverse
  /// of @ref find.
  /// @param idx The index of the subdomain, which should be from 0 to
  /// @ref size() - 1.
  //////////////////////////////////////////////////////////////////////
  value_type operator[](index_type idx) const
  {
    stapl_assert(idx < m_npart, "partition index out of bound\n");
    if (m_domain.size() <= idx)
      return value_type();

    if (this->m_shift)
    {
      gid_type first = (idx == 0) ?
        m_domain.first() :
        m_domain.advance(m_domain.first(), idx*this->m_small_part_size);

      if (idx < this->m_num_large_parts)
        first = m_domain.advance(first, idx);
      else if (this->m_num_large_parts)
        first = m_domain.advance(first, this->m_num_large_parts);

      auto dist = (idx < this-> m_num_large_parts) ?
        this->m_small_part_size : this->m_small_part_size - 1;

      gid_type last = (idx == m_npart-1) ?
        m_domain.last() :
        m_domain.advance(first, dist);

      return value_type(first, last, m_domain);
    }
    else
    {
      // remainder
      size_t rem      = m_domain.size() % m_npart;
      // block size lower
      size_t bszl     = m_domain.size() / m_npart;
      // block size greater
      size_t bszg     = bszl + (rem > 0 ? 1 : 0);

      gid_type first = (idx <= rem)
             ? m_domain.advance(m_domain.first(), idx*bszg)
             : m_domain.advance(m_domain.first(), (rem*bszg + (idx-rem)*bszl));

      gid_type last = (idx==m_npart-1)
             ? m_domain.last()
             : m_domain.advance(first, ((idx < rem) ? bszg-1 : bszl-1));
      return value_type(first, last, m_domain);
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the domain of gids that map to the specified partition
  ///   if the mapping from GID to partition id is recognized.
  ///
  /// This is functionally equivalent to the index operator for this partition.
  ///
  /// @param pid id of the partition whose domain is required
  //////////////////////////////////////////////////////////////////////
  boost::optional<value_type> try_get_domain(index_type pid) const
  {
    return this->operator[](pid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the number of partitions generated.
  //////////////////////////////////////////////////////////////////////
  size_t size() const
  {
    if (m_domain.size() < m_npart)
      return m_domain.size();
    return m_npart;
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
  index_type find(gid_type const& g) const
  {

    if (this->m_shift)
    {
      return g>>this->m_shift_dist;
    }
    else
    {

      if (!m_domain.contains(g))
        return m_npart;

      const size_t rem  = m_domain.size() % m_npart;
      const size_t bszl = m_domain.size() / m_npart;
      const size_t bszg = bszl + (rem > 0 ? 1 : 0);

      const size_t d = m_domain.distance(m_domain.first(), g);
      const size_t i = (d <= rem*bszg) ? (d/bszg)
                                 : rem + ((d - rem * bszg)/bszl);

      stapl_assert(i<m_npart, "calculated index is out of bounds");
      return i;
    }
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
  /// @todo The is_contiguous specialization has a bug in it exposed
  /// by usage in set_elements().  Can be exposed with unique_copy
  /// valladolid benchmark.  Debug and enable.
  //////////////////////////////////////////////////////////////////////
  template <typename ODom, typename MFG>
  std::vector<std::pair<domain_type,bool> >
  contained_in(ODom const& dom, MFG const& mfg)
  {
    std::vector<std::pair<domain_type,bool> > doms;

#if 0
   if (dom.is_contiguous()) {
     // Inverse mapping
     size_t cj = this->find(dom.first());
     size_t dj = this->find(dom.last());

     if (cj>=m_npart)
       return doms;

     typename ODom::index_type ai = mfg[cj]((*this)[cj].first());
     typename ODom::index_type bi = mfg[dj]((*this)[dj].last());

     if (cj < m_npart && ai<dom.first())
       ++cj;

     if (cj < dj) {
       if (dom.last()<bi) {
         doms.push_back(std::make_pair(domain_type(cj,dj-1),true));
         doms.push_back(std::make_pair(domain_type(dj,dj),false));
       }
       if (dom.last()==bi) {
         doms.push_back(std::make_pair(domain_type(cj,dj),true));
       }
     }

     if (cj == dj) {
       if (dom.last()<bi) {
         doms.push_back(std::make_pair(domain_type(cj,dj),false));
       }
       if (dom.last()==bi) {
         doms.push_back(std::make_pair(domain_type(cj,dj),true));
       }
     }
   }
   else
#endif
   {
      for (index_type j = 0; j < this->size(); ++j)
      {
        value_type domj   = (*this)[j];
        value_type tmpdom = (domj & const_cast<ODom&>(dom));
        if (tmpdom.size() == domj.size()) {
          doms.push_back(std::make_pair(domain_type(j,j),true));
        }
        else
          if (tmpdom.size() != 0)
            doms.push_back(std::make_pair(domain_type(j,j),false));
      }
    }
    return doms;
  }

  void define_type(typer& t)
  {
    t.base<detail::partition_base<Dom::dimension_type::value>>(*this);
    t.member(m_domain);
    t.member(m_npart);
  }
}; // struct balanced_partition

} // namespace stapl

#endif // STAPL_CONTAINERS_BALANCED_PARTITION_HPP
