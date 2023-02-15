/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_PARTITIONS_SLICED_HPP
#define STAPL_CONTAINERS_PARTITIONS_SLICED_HPP

#include <stapl/views/sliced_view.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Partitioner that slices off dimensions of a d-dimensional domain
///        to produce subdomains of dimension d-|Slices|.
///
///        This partitioner is only valid on domains that have homogeneous
///        GID types.
///
/// @tparam Slices A tuple of compile time integral constants to specify
///                the dimensions to slice off
/// @tparam Dom The multidimensional domain type that is being sliced
//////////////////////////////////////////////////////////////////////
template<typename Slices, typename Dom>
class sliced_partition
{
  typedef typename Dom::gid_type global_gid_type;
  typedef typename tuple_element<0,
    global_gid_type
  >::type                                              scalar_index_t;
  typedef typename stapl::tuple_size<Slices>::type     number_of_slices_t;
  typedef typename dimension_traits<
    global_gid_type
  >::type                                              original_dimension_t;

public:
  /// The type of the produces subdomains. They have dimensionality d-|Slices|
  typedef typename detail::SLICED_view_domain<
    original_dimension_t::value-number_of_slices_t::value, scalar_index_t
  >::type                                             value_type;

  /// The type of the domain of this partition, which has
  /// dimensionality |Slices|
  typedef typename detail::SLICED_view_domain<
    number_of_slices_t::value, scalar_index_t
  >::type                                             domain_type;

  /// The type of the GIDs in the produced subdomains
  typedef typename value_type::index_type             gid_type;

  /// The type used to index subdomains in this partition
  typedef typename domain_type::gid_type              index_type;

  typedef Slices slices_type;

private:
  Dom         m_global_domain;
  domain_type m_domain;

public:
  //////////////////////////////////////////////////////////////////////
  /// @param domain The global domain to partition
  //////////////////////////////////////////////////////////////////////
  sliced_partition(Dom const& domain)
    : m_global_domain(domain),
      m_domain(
        tuple_ops::filter<Slices>(domain.first()),
        tuple_ops::filter<Slices>(domain.last())
      )
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief The original domain that is being partitioned
  //////////////////////////////////////////////////////////////////////
  Dom const& global_domain() const
  {
    return m_global_domain;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief The domain that contains the indices of the subdomains
  //////////////////////////////////////////////////////////////////////
  domain_type const& domain() const
  {
    return m_domain;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief The size in each dimension of the number of subdomains
  //////////////////////////////////////////////////////////////////////
  typename domain_type::size_type dimensions() const
  {
    return m_domain.dimensions();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the @p idx-th domain in the partition.
  //////////////////////////////////////////////////////////////////////
  value_type operator[](index_type const& idx) const
  {
    return value_type(
      tuple_ops::discard<Slices>(m_global_domain.first()),
      tuple_ops::discard<Slices>(m_global_domain.last())
    );
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the number of partitions generated.
  //////////////////////////////////////////////////////////////////////
  size_t size() const
  {
    return m_domain.size();
  }

  void define_type(typer& t)
  {
    t.member(m_global_domain);
    t.member(m_domain);
  }

}; // struct sliced_partition

} // namespace stapl

#endif // STAPL_VIEWS_PARTITIONS_SLICED_HPP
