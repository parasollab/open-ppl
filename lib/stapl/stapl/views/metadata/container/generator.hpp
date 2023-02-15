/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_METADATA_CONTAINER_GENERATOR_HPP
#define STAPL_VIEWS_METADATA_CONTAINER_GENERATOR_HPP

#include <stapl/containers/partitions/balanced.hpp>
#include <stapl/containers/partitions/block_partition.hpp>
#include <stapl/domains/indexed.hpp>

#include <stapl/views/mapping_functions/linearization.hpp>
#include <stapl/containers/type_traits/dimension_traits.hpp>

#include <stapl/views/metadata/metadata_traits.hpp>

namespace stapl {

namespace metadata {

namespace detail {

template<int N>
struct balanced_partition_type
{
  using type = multiarray_impl::block_partition<
    typename default_traversal<N>::type
  >;

  static homogeneous_tuple_type_t<N, std::size_t>
  get_nparts(std::size_t nparts)
  { return homogeneous_tuple<N>(nparts); }
};

template<>
struct balanced_partition_type<1>
{
  using type = balanced_partition<indexed_domain<std::size_t>>;

  static std::size_t get_nparts(std::size_t nparts)
  { return nparts; }
};

//////////////////////////////////////////////////////////////////////
/// @brief Helper static polymorphic function to compute view container
///   size based on either size() or dimensions() call, based on the
///   the dimensionality of the partitioned data it represents.
//////////////////////////////////////////////////////////////////////
template<typename Domain, int = dimension_traits<Domain>::type::value>
struct domain_size
{
private:
  using dimensions_type = typename Domain::dimensions_type;
  static int constexpr dim = dimension_traits<dimensions_type>::type::value;

public:
  static dimensions_type apply(Domain const& vc)
  { return vc.dimensions(); }

  static dimensions_type apply_local(void)
  { return homogeneous_tuple<dim, std::size_t>(1); }
};


template<typename ViewContainer>
struct domain_size<ViewContainer, 1>
{
  static size_t apply(ViewContainer const& vc)
  { return vc.size(); }

  static size_t apply_local(void)
  { return 1; }
};

} // namespace detail

//////////////////////////////////////////////////////////////////////
/// @brief Metadata container to create metadata information on demand
///        using a balance partition of the view's domain to determine
///        the metadata information.
///
/// @tparam MD Type of the metadata used to represent the metadata
///            information.
//////////////////////////////////////////////////////////////////////
template<typename MD>
class generator_container
{
  using domain_t = typename MD::domain_type;

public:
  using dimension_type = typename dimension_traits<
    typename domain_t::gid_type
  >::type;

private:

  using part_t = typename detail::balanced_partition_type<
    dimension_type::value
  >::type;

  using reverse_linearizer_t = nd_reverse_linearize<
    typename domain_t::gid_type, typename default_traversal<
      dimension_type::value
    >::type
  >;

public:
  using domain_type = typename part_t::domain_type;
  using index_type = typename part_t::index_type;
  using value_type = MD;
  using component_type = typename value_type::component_type;
  using reference = value_type;
  using iterator = value_type*;
  using dimensions_type = index_type;

private:
  part_t               m_part;
  reverse_linearizer_t m_reverse_linearizer;
  value_type           m_value;

public:
  void define_type(typer& t)
  {
    t.member(m_part);
    t.member(m_value);
  }

  generator_container(component_type c)
    : m_part(c->domain(),
             detail::balanced_partition_type<dimension_type::value>::get_nparts(
               std::min(static_cast<long unsigned int>(get_num_locations()),
                        c->domain().size()))),
      m_reverse_linearizer(detail::domain_size<domain_type>::apply(
        this->domain()
      )),
      m_value()
  {
    auto&& id = m_reverse_linearizer(get_location_id());

    typename std::remove_pointer<component_type>::type::domain_type dom;
    if (id < m_part.domain().dimensions())
      dom = m_part[id];

    // convert from nd_domain to indexed_domain
    domain_t new_dom(dom.first(), dom.last());

    m_value = value_type(
      id, new_dom, c, LQ_CERTAIN, get_affinity(),
      c->get_distribution()->get_rmi_handle(), get_location_id()
    );
  }

  iterator begin()
  {
    return &m_value;
  }

  iterator end()
  {
    return &m_value + 1;
  }

  reference operator[](index_type const& index)
  {
    return value_type(
      index, m_part[index], m_value.component(), LQ_CERTAIN,
      get_affinity(), m_value.handle(), m_value.location()
    );
  }

  size_t size() const
  {
    return m_part.size();
  }

  dimensions_type dimensions() const
  {
    return detail::domain_size<domain_type>::apply(
      this->domain()
    );
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc growable_container::get_local_vid(size_t)
  //////////////////////////////////////////////////////////////////////
  index_type get_local_vid(index_type const& index)
  {
    return m_value.id();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc growable_container::push_back_here(MD const&)
  /// This class does not support @c push_back_here.
  //////////////////////////////////////////////////////////////////////
  template <typename T>
  void push_back_here(T const&)
  {
    stapl_assert(false, "Static metadata container cannot add entries");
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc growable_container::local_size
  //////////////////////////////////////////////////////////////////////
  size_t local_size(void) const
  {
    return m_value.location() < this->domain().size() ? 1 : 0;
  }

  dimensions_type local_dimensions() const
  {
    return detail::domain_size<domain_type>::apply_local();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc growable_container::get_location_element(size_t)
  //////////////////////////////////////////////////////////////////////
  location_type get_location_element(index_type const&) const
  {
    return m_value.location();
  }

  domain_type domain(void) const
  {
    return m_part.domain();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc flat_container::update
  /// This class does not require any action.  The method exists for
  /// interface compatibility
  //////////////////////////////////////////////////////////////////////
  void update(void)
  { }
};

} // namespace metadata

template<typename MD>
struct metadata_traits<metadata::generator_container<MD>>
{
  using is_isomorphic = std::integral_constant<bool, true>;
  using value_type    = MD;
};

} // namespace stapl

#endif // STAPL_VIEWS_METADATA_EXTRACTION_GENERATOR_HPP
