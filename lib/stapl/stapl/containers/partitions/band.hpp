/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_PARTITIONS_BAND_HPP
#define STAPL_CONTAINERS_PARTITIONS_BAND_HPP

#include <boost/iterator/transform_iterator.hpp>
#include <stapl/domains/indexed.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Partition to construct reference the elements in a band matrix
///
/// @tparam Dom A two dimensional domain type.
//////////////////////////////////////////////////////////////////////
template <typename Dom>
struct band_partition
{
  typedef indexed_domain<size_t>               domain_type;
  typedef Dom                                  value_type;
  typedef typename value_type::index_type      gid_type;
  typedef typename domain_type::index_type     index_type;

private:
  Dom    m_domain;
  size_t m_left;
  size_t m_right;

public:

  //////////////////////////////////////////////////////////////////////
  /// @brief Band partition constructor.
  ///
  /// The overlapped domains are defined specifying the number of
  /// elements that are overlapped to the left (@p l), the number of
  /// elements that are not overlapped (@c c) and the number of
  /// elements overlapped to the right (@c r). Each subdomain has
  /// size: l+r+1 (diagonal).
  /// @par Example:
  ///     Domain to partition: [0..3]x[0..3]<br/>
  ///     left band (l): 1<br/>
  ///     right band (r): 1<br/>
  ///     Resulting partition: {[0..0]x[0..1],[1..1]x[0..2],[2..2]x[1..3],
  ///                           [3..3]x[2..3]}<br/>
  /// @param domain Domain to partition.
  /// @param l Number of elements to the left of the diagonal.
  /// @param r Number of elements to the right of the diagonal.
  //////////////////////////////////////////////////////////////////////
  band_partition(value_type const& domain,
                 size_t l=0, size_t r=0)
    : m_domain(domain), m_left(l), m_right(r)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Copy constructor that replaces the domain with the domain provided.
  //////////////////////////////////////////////////////////////////////
  band_partition(value_type const& domain,
               band_partition const& other)
    : m_domain(domain), m_left(other.m_left), m_right(other.m_right)
  { }

  Dom const& global_domain() const
  {
    return m_domain;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the @p idx-th domain in the partition.
  //////////////////////////////////////////////////////////////////////
  value_type operator[](size_t idx) const
  {
    typedef typename value_type::index_type id_type;
    size_t left  = idx == get<1>(m_domain.first()) ? 0 : m_left;
    size_t right = idx == get<1>(m_domain.last())  ? 0 : m_right;
    size_t lower = get<1>(m_domain.first())+ idx - left;
    size_t upper = lower + left + right;
    return value_type(id_type(idx,lower),id_type(idx,upper));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the number of partitions generated.
  //////////////////////////////////////////////////////////////////////
  size_t size() const
  {
    // One for each row
    return !m_domain.empty() ?
      get<0>(m_domain.last()) - get<0>(m_domain.first()) + 1 : 0;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the domain of the partition
  //////////////////////////////////////////////////////////////////////
  domain_type domain() const
  {
    return domain_type(this->size());
  }

private:

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the index of the subdomain that contains the GID @p g.
  ///
  /// If the flag @p first is true, the returned index of the first
  /// subdomain that contains the given GID @p g, otherwise returns
  /// the index of the last subdomain that contains @p g.
  //////////////////////////////////////////////////////////////////////
  index_type in_domain(gid_type const& g, bool first=true)
  {
    return get<0>(g) - get<0>(m_domain.first()); //Row index
  }

public:

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the index of the partition that contains the GID
  ///        @p g.
  //////////////////////////////////////////////////////////////////////
  index_type find(gid_type const& g) const
  {
    stapl::abort("This partition should not be used to specify "
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
  /// @param dom Domain to compare
  /// @param mfg Mapping function generator used to get the associated
  ///            mapping function to each partition. The generated
  ///            mapping function is used to project generated
  ///            partitioned domains into the given domain.
  /// @return a vector of pairs.
  //////////////////////////////////////////////////////////////////////
  template <typename ODom, typename MFG>
  std::vector<std::pair<domain_type,bool> >
  contained_in(ODom const& dom, MFG const& mfg)
  {
    std::vector<std::pair<domain_type,bool> > doms;

    typedef typename value_type::index_type id_type;

    // Type of the idex of a row or column is the same as the index_type is a
    // homogeneous tuple.
    typedef typename tuple_element<0, id_type>::type rc_id_type;

    rc_id_type row_i = get<0>(dom.first());
    rc_id_type row_f = get<0>(dom.last());

    rc_id_type col_i = get<1>(dom.first());
    rc_id_type col_f = get<1>(dom.last());

    rc_id_type row_diag_i = std::max(row_i,col_i);
    long int left_dist = col_i - (row_diag_i - m_left);

    if (left_dist<0) left_dist = 0;

    rc_id_type row_local_i = row_diag_i + left_dist;

    size_t min_distance = std::min(row_f - row_diag_i, col_f - row_diag_i);

    rc_id_type row_diag_f = row_diag_i + min_distance;

    size_t right_dist = m_right;
    if (row_diag_f+m_right<=col_f)
      right_dist = 0;
    rc_id_type row_local_f = row_diag_f - right_dist;

    if (row_i==get<0>(m_domain.first()) &&
        col_i==get<1>(m_domain.first()))
      row_local_i = row_i;

    if (row_f==get<0>(m_domain.last()) &&
        col_f==get<1>(m_domain.last()))
      row_local_f= row_f;

    if (row_local_i<=row_local_f && row_local_i>=row_i &&
        row_local_f<=row_f && row_local_i>=col_i && row_local_f<=col_f) {
      doms.push_back(std::make_pair(domain_type(row_local_i,row_local_f),true));
    }

    size_t reminder_rows = m_left + m_right;
    index_type row_non_local_i = row_local_f + 1;
    if (row_non_local_i <= row_f && row_diag_i < row_non_local_i &&
        row_non_local_i < get<0>(m_domain.last()) ) {
      doms.push_back(std::make_pair(
              domain_type(row_non_local_i,row_local_f+reminder_rows),false));
    }

    return doms;
  }
}; // struct band_partition

} // namespace stapl

#endif // STAPL_CONTAINERS_PARTITIONS_BAND_HPP
