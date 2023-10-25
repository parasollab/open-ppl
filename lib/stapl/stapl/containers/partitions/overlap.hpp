/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_PARTITIONS_OVERLAP_HPP
#define STAPL_CONTAINERS_PARTITIONS_OVERLAP_HPP

#include <boost/iterator/transform_iterator.hpp>
#include <stapl/domains/intersect.hpp>
#include <stapl/domains/indexed.hpp>
#include <stapl/views/type_traits/is_identity.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Overlapping partition for one dimensional domains.
///
/// A domain is partitioned based on the specified number of elements
///   overlapped.
/// @tparam Dom Domain type.
//////////////////////////////////////////////////////////////////////
template <typename Dom>
struct overlap_partition
{
public:
  typedef indexed_domain<size_t>               domain_type;
  typedef Dom                                  value_type;
  typedef typename value_type::index_type      gid_type;
  typedef typename domain_type::index_type     index_type;

private:
  Dom    m_domain;   ///< Original domain to be partitioned.
  size_t m_core;     ///< Number of elements in the core of each subdomain
  size_t m_left;     ///< Number of elements to the left of the core
  size_t m_right;    ///< Number of elements to the right of the core
  size_t m_dom_size; ///< Size of the original domain; kept separately because
                     ///< domain.size() calls might be expensive (e.g. for
                     ///< @ref domset1D domain with a big number of intervals).

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Overlap partition constructor.
  ///
  /// The overlapped domains are defined by specifying the number of
  /// elements that form the non-overlapped core subdomains (@p c),
  /// the number of elements overlapped on the left of each core subdomain
  /// (@p l) and the number of elements overlapped on the right (@p r).
  ///
  /// The number of generated subdomains is always equal to the number of
  /// core subdomains and their size is @p l+c+r, except for the boundary
  /// subdomains, which are clipped so as to fit into the original domain.
  ///
  /// @par Example:
  ///     Domain to partition: [0..10]<br/>
  ///     non overlap   (c): 3<br/>
  ///     left overlap  (l): 2<br/>
  ///     right overlap (r): 1<br/>
  ///     Core subdomains (non-overlapping): {[0..2],[3..5],[6..8],[9..10]}<br/>
  ///     Resulting partition: {[0..3],[1..6],[4..9],[7..10]}<br/>
  ///
  /// @param domain Domain to partition.
  /// @param c Number of elements in the core of each subdomain (not overlapped)
  /// @param l Number of elements to the left of the core (overlapping the
  ///          previous subdomain(s))
  /// @param r Number of elements to the right of the core (overlapping the
  ///          next subdomain(s))
  //////////////////////////////////////////////////////////////////////
  overlap_partition(value_type const& domain,
                    size_t c=1, size_t l=0, size_t r=0)
    : m_domain(domain), m_core(c), m_left(l), m_right(r),
      m_dom_size(domain.size())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Copy constructor that replaces the domain with the domain provided.
  //////////////////////////////////////////////////////////////////////
  overlap_partition(value_type const& domain,
               overlap_partition const& other)
    : m_domain(domain),
      m_core(other.m_core), m_left(other.m_left), m_right(other.m_right),
      m_dom_size(domain.size())
  { }

  Dom const& global_domain() const
  { return m_domain; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the number of partitions generated.
  //////////////////////////////////////////////////////////////////////
  size_t size() const
  {
    size_t i = m_dom_size/m_core;
    if (m_dom_size%m_core != 0)
      i++;
    return i;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Return the first gid in the @p idx partition.
  //////////////////////////////////////////////////////////////////////
  gid_type compute_first(size_t idx) const
  {
    const std::ptrdiff_t dist_to_left = idx*m_core - m_left;

    return dist_to_left <= 0 ?
      m_domain.first()
      : m_domain.advance(m_domain.first(), dist_to_left);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Return the last gid in the @p idx partition.
  //////////////////////////////////////////////////////////////////////
  gid_type compute_last(size_t idx) const
  {
    const size_t dist_to_right = idx*m_core + (m_core-1) + m_right;

    return dist_to_right >= m_dom_size-1 ?
      m_domain.last()
      : m_domain.advance(m_domain.first(), dist_to_right);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the @p idx-th domain in the partition.
  //////////////////////////////////////////////////////////////////////
  value_type operator[](size_t idx) const
  {
    return value_type(compute_first(idx), compute_last(idx), m_domain);
  }

private:

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the index of the subdomain that contains the GID @p g
  ///   in its core part.
  ///
  /// If @p g is not in any subdomain core, returns and invalid index.
  //////////////////////////////////////////////////////////////////////
  index_type in_domain(gid_type const& g)
  {
    size_t dist = m_domain.distance(m_domain.first(), g);

    size_t pos = dist / m_core;
    if (pos >= this->size())
      return index_bounds<index_type>::invalid();

    return pos;
  }

public:

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the index of the partition that contains the GID
  ///        @p g.
  //////////////////////////////////////////////////////////////////////
  index_type find(gid_type const& g) const
  {
    stapl_assert(false,
                 "This partition should not be used to specify "
                 "the partition of a container");
    return index_type();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Determine which partition has the elements referenced
  ///        for the given domain.
  ///
  /// The returned information is a collection (possibly empty) of
  /// pairs. Each pair contains information about which partitions are
  /// included in the given domain and how they are included (True: if
  /// it is fully contained, False: if it is partially included). The
  /// returned collection contains elements for the partitions that
  /// contain elements on the given domain.
  ///
  /// @par Example:
  ///    Partition: [{[0..3],[1..6],[4..9],[7..10]}<br/>
  ///    Given domain: [0..4]<br/>
  ///    Returns:  {([0..0],True),([1..2],False)}<br/>
  ///
  /// Used by @ref segmented_metadata_projection.
  ///
  /// @param dom Domain to compare
  /// @param mfg Mapping function generator used to get the associated
  ///            mapping function to each partition. Note that overlap_view
  ///            fixes all mapping functions generated by mfg to identity_mf;
  ///            these are used to map each overlapped subview domain to the
  ///            original view domain, which itself can be mapped to the
  ///            underlying container domain by an arbitrary mapping function
  ///            of the original view.
  /// @return a vector of pairs.
  //////////////////////////////////////////////////////////////////////
  template <typename ODom, typename MFG>
  std::vector<std::pair<domain_type,bool> >
  contained_in(ODom const& dom, MFG const& mfg)
  {
    stapl_assert(is_identity<typename MFG::mapfunc_type>::value,
      "Invalid mapping function type generated for overlap_view.");

    std::vector<std::pair<domain_type,bool> > doms;

    // Execute the faster algorithm using the knowledge that the original domain
    // is contiguous (the second condition is needed to make sure that we are
    // not dealing with a local contiguous subdomain of a non-contiguous global
    // domain).
    if (dom.is_contiguous() && m_domain.is_contiguous())
    {
      // Get the index of the partition having the first element of the input
      // domain in its core.
      const index_type pid_start = this->in_domain(dom.first());
      index_type pid_first = pid_start;

      // Get the index of the partition having the last element of the input
      // domain in its core.
      const index_type pid_final = this->in_domain(dom.last());
      index_type pid_last = pid_final;

      if (pid_first > pid_last ||
          pid_first == index_bounds<index_type>::invalid() ||
          pid_last == index_bounds<index_type>::invalid())
        return doms;

      // If the core of the overlapping partition spans multiple locations,
      // we need to make sure that the partition will be added only on one
      // location. The core of the boundary partitions might be trimmed so
      // as to fit the partition into the original global domain; then, if
      // there is no overlap on the other side, the partition will be fully
      // contained within the domain.
      if (m_core > dom.size())
      {
        if (dom.contains(m_domain.advance(m_domain.first(), m_core*pid_first)))
        {
          const bool within_dom = (*this)[pid_first].size() <= dom.size();
          doms.push_back(
            std::make_pair(domain_type(pid_first,pid_first), within_dom));
        }
        else if (dom.contains(
          m_domain.advance(m_domain.first(), m_core*pid_last)))
        {
          const bool within_dom = (*this)[pid_last].size() <= dom.size();
          doms.push_back(
            std::make_pair(domain_type(pid_last,pid_last), within_dom));
        }

        return doms;
      }

      // Bracket the partitions referencing only elements in the input domain.

      // Discard partitions that start before the first element of the input
      // domain.
      while ( pid_first <= pid_last &&
              !dom.contains((*this)[pid_first].first()) )
      {
        ++pid_first;
      }

      // Discard partitions that end after the last element of the input domain.
      // The second condition stops the iteration when pid_last falls into
      // negative range.
      while ( pid_last >= pid_first && pid_last <= pid_final &&
              !dom.contains((*this)[pid_last].last()) )
      {
        --pid_last;
      }

      // Add partitions covering the input domain to the resulting vector.

      // Add partitions referencing elements within and before the input domain
      if (pid_first > pid_start)
        doms.push_back(
          std::make_pair(domain_type(pid_start,pid_first-1), false));

      // Add partitions only referencing elements within the input domain.
      if (pid_first <= pid_last && pid_last <= pid_final)
        doms.push_back(std::make_pair(domain_type(pid_first,pid_last), true));

      // Add partitions referencing elements within and beyond the input domain
      if (pid_last < pid_final)
        doms.push_back(
          std::make_pair(domain_type(pid_last+1,pid_final), false));
      // The following condition holds true when the only partition that
      // contains the input domain starting with index 0 ends beyond the end of
      // that domain; in this case, pid_last falls below 0 and is wrapped to
      // max<size_t>.
      else if (pid_last > pid_final)
        doms.push_back(std::make_pair(domain_type(0,0), false));
    }
    // If we cannot assure that the original domain is contiguous, the more
    // general algorithm has to be used.
    else {
      for (index_type j = 0; j < this->size(); ++j) {
        value_type domj = (*this)[j];

        value_type tmpdom = intersect(dom, domj);

        if (tmpdom.empty())
          continue;

        if (tmpdom.size() == domj.size())
          doms.push_back(std::make_pair(domain_type(j,j),true));
        else if (dom.contains(domj.first()))
          doms.push_back(std::make_pair(domain_type(j,j),false));
      }
    }
    return doms;
  }
}; // struct overlap_partition

} // namespace stapl

#endif // STAPL_VIEWS_PARTITIONS_OVERLAP_HPP
