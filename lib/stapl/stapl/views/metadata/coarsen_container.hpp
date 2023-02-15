/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_METADATA_COARSEN_CONTAINER_HPP
#define STAPL_VIEWS_METADATA_COARSEN_CONTAINER_HPP

#include <vector>

#include <stapl/runtime.hpp>
#include <stapl/domains/indexed.hpp>
#include <stapl/views/metadata/mix_view.hpp>
#include <stapl/views/metadata/metadata_traits.hpp>
#include <stapl/containers/type_traits/dimension_traits.hpp>
#include <stapl/domains/indexed.hpp>

namespace stapl {

namespace view_coarsen_impl {

template<int D, bool Isomorphic>
struct select_coarsen_container_domain
{
  using type = indexed_domain<
    std::size_t, D, typename default_traversal<D>::type
  >;
};

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to compute the type of the sliced view's domain.
///        Specialization for the one-dimensional case.
//////////////////////////////////////////////////////////////////////
template<int D>
struct select_coarsen_container_domain<D, false>
{
  using type = indexed_domain<std::size_t>;
};

//////////////////////////////////////////////////////////////////////
/// @brief Defines a container used to produce subviews with extra
///        information about locality (mix_views), based on the given
///        the metadata information (metadata container).
///
/// @tparam View Type of view that is used to extract locality.
/// @tparam CP Metadata container type.
//////////////////////////////////////////////////////////////////////
template <typename View, typename CP>
class coarsen_container
  : public p_object
{
  typedef View                                        container_type;

  typedef typename View::domain_type                  view_domain_type;
  typedef CP                                          metadata_container_type;
  typedef typename CP::value_type                     md_info_t;
  typedef typename dimension_traits<
    typename md_info_t::index_type
  >::type                                             dimension_t;

  View   m_view;
  CP     m_partition;

public:

  typedef typename select_coarsen_container_domain<
    dimension_t::value,
    metadata_traits<CP>::is_isomorphic::value &&
    dimension_t::value != 1
  >::type                                             domain_type;

  typedef typename domain_type::gid_type              gid_type;
  typedef typename dimension_traits<gid_type>::type   dimension_type;

  typedef mix_view<View, md_info_t, gid_type>         value_type;
  typedef value_type                                  reference;

  // Container returns copies as references,
  // making this type a const-qualified copy.
  typedef const value_type                            const_reference;

  View* get_container(void) const
  {
    return &m_view;
  }

  View const& container(void) const
  {
    return m_view;
  }

  metadata_container_type const& get_partition(void) const
  {
    return m_partition;
  }

  coarsen_container(View const& view, metadata_container_type const& new_part)
    : m_view(view), m_partition(new_part)
  { }

  size_t version(void) const
  {
     return 0;
  }

  value_type operator[](gid_type const& index)
  {
    return get_element(index);
  }

  const_reference operator[](gid_type const& index) const
  {
    return get_element(index);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a view based on the metadata associated with the
  ///        position @p index.
  ///
  /// The view behaves as is defined by the type View and contains
  /// extra information about data locality of the elements the
  /// generated view references.
  //////////////////////////////////////////////////////////////////////
  value_type get_element(gid_type const& index)
  {
    auto const md = m_partition[index];

    return value_type(
      m_view, md.domain(), m_view.mapfunc(), index, md
    );
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a const view based on the metadata associated with the
  ///        position @p index.
  ///
  /// The view behaves as is defined by the type View and contains
  /// extra information about data locality of the elements the
  /// generated view references.
  //////////////////////////////////////////////////////////////////////
  const_reference get_element(gid_type const& index) const
  {
    auto const md = m_partition[index];

    return value_type(
      m_view, md.domain(), m_view.mapfunc(), index, md
    );
  }

  size_t size(void) const
  {
    return m_partition.size();
  }


  gid_type dimensions(void) const
  {
    return m_partition.dimensions();
  }

  domain_type domain() const
  {
    return domain_type(m_partition.size());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of local elements associated with this
  ///        location.
  //////////////////////////////////////////////////////////////////////
  size_t local_size(void) const
  {
    return m_partition.local_size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the global index of the given local @p index.
  //////////////////////////////////////////////////////////////////////
  size_t get_local_vid(gid_type const& index) const
  {
    return m_partition.get_local_vid(index);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return locality information about the element specified by
  ///   the gid.
  /// @return A locality_qualifier, affinity specifier, as well as this
  ///   object's handle and associated location for this affinity.
  //////////////////////////////////////////////////////////////////////
  locality_info locality(gid_type const& index)
  {
    auto const md = m_partition[index];

    return locality_info(
      md.location_qualifier(), md.affinity(), md.handle(), md.location()
    );
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the location where the metadata indexed by @p
  ///        index is located.
  //////////////////////////////////////////////////////////////////////
  location_type get_location_element(gid_type const& index) const
  {
    return m_partition.get_location_element(index);
  }
}; // class coarsen_container

} // namespace view_coarsen_impl

} // namespace stapl


#endif /// STAPL_VIEWS_METADATA_COARSEN_CONTAINER_HPP
