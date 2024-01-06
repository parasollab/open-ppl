/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_PARTITIONS_ROWS_PARTITION_HPP
#define STAPL_CONTAINERS_PARTITIONS_ROWS_PARTITION_HPP

#include <boost/iterator/transform_iterator.hpp>
#include <stapl/domains/indexed.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Partition in rows the given 2D domain
///
/// @tparam Dom A two dimensional domain type.
//////////////////////////////////////////////////////////////////////
template <typename Dom>
struct rows_partition
{
  typedef indexed_domain<size_t>               domain_type;
  typedef domain_type                          value_type;
  typedef typename value_type::index_type      gid_type;
  typedef typename domain_type::index_type     index_type;

private:
  Dom    m_domain;

public:

  //////////////////////////////////////////////////////////////////////
  /// @brief Rows partition constructor.
  ///
  /// @par Example:
  ///     Domain to partition: [0..3]x[0..3]<br/>
  ///     Resulting partition: {[0..0]x0..3],[1..1]x[0..3],[2..2]x[0..3],
  ///                           [3..3]x[0..3]}<br/>
  /// @param domain Domain to partition.
  //////////////////////////////////////////////////////////////////////
  rows_partition(Dom const& domain)
    : m_domain(domain)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Copy constructor that replaces the domain with the domain provided.
  //////////////////////////////////////////////////////////////////////
  rows_partition(Dom const& domain,
                 rows_partition const& other)
    : m_domain(domain)
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
    return m_domain.template get_domain<1>();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the number of partitions generated. One for each row.
  //////////////////////////////////////////////////////////////////////
  size_t size() const
  {
    return m_domain.template get_domain<0>().size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the domain of the partition
  //////////////////////////////////////////////////////////////////////
  domain_type domain() const
  {
    return domain_type(this->size());
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

    auto row_domain = dom.template get_domain<0>();
    auto col_domain = dom.template get_domain<1>();

    auto col_global_domain = m_domain.template get_domain<1>();

    index_type col_i = col_domain.first();

    if (col_i == col_global_domain.first()) {
      bool full_included = col_domain.size()==col_global_domain.size();
      doms.push_back(std::make_pair(row_domain,full_included));
    }
    return doms;
  }
}; // struct rows_partition

} // namespace stapl

#endif // STAPL_CONTAINERS_PARTITIONS_ROWS_PARTITION_HPP
