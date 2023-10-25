/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_METADATA_CONTAINER_BY_DEMAND_HPP
#define STAPL_VIEWS_METADATA_CONTAINER_BY_DEMAND_HPP

#include <stapl/runtime.hpp>
#include <stapl/views/metadata/metadata_entry.hpp>
#include <stapl/domains/interval.hpp>
#include <stapl/domains/domain_interval.hpp>
#include <stapl/domains/indexed.hpp>
#include <boost/iterator/transform_iterator.hpp>

namespace stapl {

namespace metadata {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Default method for converting between different domains.
///
/// Constructs the target domain by iterating through the source domain
/// and adding one element after another.
//////////////////////////////////////////////////////////////////////
template <typename TargetDomain, typename SourceDomain>
TargetDomain construct_domain_impl(SourceDomain const& dom)
{
  TargetDomain ret_val;
  typename SourceDomain::index_type i      = dom.first();
  typename SourceDomain::index_type i_last = dom.last();

  for (; i != i_last; i = dom.advance(i, 1))
  {
    ret_val += i;
  }

  ret_val += dom.last();
  return ret_val;
}

//////////////////////////////////////////////////////////////////////
/// @brief Functor to convert between different domain types.
///
/// Used in projected_container to construct the view metadata
/// from the container metadata directly.
//////////////////////////////////////////////////////////////////////
template <typename TargetDomain>
struct construct_domain
{
  TargetDomain operator()(TargetDomain const& dom)
  {
    return dom;
  }

  template <typename SourceDomain>
  TargetDomain operator()(SourceDomain const& dom)
  {
    return construct_domain_impl<TargetDomain>(dom);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Functor to convert from a given domain to @ref domset1D domain.
///
/// Used in projected_container to construct the view metadata
/// from the container metadata directly.
//////////////////////////////////////////////////////////////////////
template <typename T>
struct construct_domain<domset1D<T>>
{
  using TargetDomain = domset1D<T>;

  /// Source domain is the same as the target domain - no conversion needed.
  TargetDomain operator()(TargetDomain const& dom)
  {
    return dom;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Convert from an @ref indexed_domain.
  ///
  /// Creates a @ref domset1D domain with just one interval equal to the
  /// @p indexed_domain.
  //////////////////////////////////////////////////////////////////////
  template <typename TT>
  TargetDomain operator()(indexed_domain<TT> const& dom)
  {
    return { dom.first(), dom.last(), dom.is_same_container_domain() };
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Convert from a @ref domainset1D domain.
  ///
  /// Uses the default conversion method for two general domains if the
  /// @p domainset1D domain is distributed, otherwise extracts the
  /// @ref domset1D domain wrapped by the @p domainset1D.
  //////////////////////////////////////////////////////////////////////
  template <typename Dist>
  TargetDomain operator()(domainset1D<Dist> const& dom)
  {
    if (dom.get_distribution())
      return construct_domain_impl<TargetDomain>(dom);

    return dom.get_sparse_domain();
  }

  template <typename SourceDomain>
  TargetDomain operator()(SourceDomain const& dom)
  {
    return construct_domain_impl<TargetDomain>(dom);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Functor to convert from a given domain to @ref domainset1D
///   domain.
///
/// Used in projected_container to construct the view metadata
/// from the container metadata directly.
//////////////////////////////////////////////////////////////////////
template <typename T>
struct construct_domain<domainset1D<T>>
{
  using TargetDomain = domainset1D<T>;

  /// Source domain is the same as the target domain - no conversion needed.
  TargetDomain operator()(TargetDomain const& dom)
  {
    return dom;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Convert from an @ref indexed_domain.
  ///
  /// Wraps a @ref domset1D domain with just one interval equal to the
  /// @p indexed_domain.
  //////////////////////////////////////////////////////////////////////
  template <typename TT>
  TargetDomain operator()(indexed_domain<TT> const& dom)
  {
    return { dom.first(), dom.last(), dom.is_same_container_domain() };
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Convert from a @ref domset1D domain.
  //////////////////////////////////////////////////////////////////////
  template <typename Dist>
  TargetDomain operator()(domset1D<Dist> const& dom)
  {
    return { dom };
  }

  template <typename SourceDomain>
  TargetDomain operator()(SourceDomain const& dom)
  {
    return construct_domain_impl<TargetDomain>(dom);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Functor used to compute the view's domain metadata from the
///        container's domain.
//////////////////////////////////////////////////////////////////////
template<typename View, typename MetadataEntry>
struct project_metadata_transfomer
{
  using type = metadata_entry<typename View::domain_type,
                              typename MetadataEntry::component_type,
                              typename MetadataEntry::cid_type>;

  template <typename T>
  type operator()(T const& pmd) const
  {
    // Copied to remove the proxy type of T
    MetadataEntry md = pmd;

    auto projected_domain
      = detail::construct_domain<typename View::domain_type>()(md.domain());

    return type{ md.id(),        std::move(projected_domain),
                 md.component(), md.location_qualifier(),
                 md.affinity(),  md.handle(),
                 md.location() };
  }
};

} // namespace detail

//////////////////////////////////////////////////////////////////////
/// @brief Wrapper around a metadata container to create metadata
///        information on demand, by projecting the metadata entry's domain
///        into the view's domain.
///
/// @param View Type of view from which the metadata is created.
/// @tparam MetadataContainer Type of the metadata container used to store
///         entries from extraction
//////////////////////////////////////////////////////////////////////
template<typename View, typename MetadataContainer>
class projected_container
{
  using container_type = MetadataContainer;
  using transform_type
    = detail::project_metadata_transfomer<View,
                                          typename container_type::value_type>;

  std::unique_ptr<container_type> m_container;
  size_t m_version;

public:
  STAPL_IMPORT_DTYPES_3(container_type, domain_type, dimensions_type,
                        index_type);

  using value_type = typename transform_type::type;
  using reference = value_type;
  using iterator = boost::transform_iterator<transform_type,
                                             typename container_type::iterator,
                                             value_type,
                                             value_type>;

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a projected_container wrapper over the
  ///        @p c view using the given metadata container @p md.
  //////////////////////////////////////////////////////////////////////
  projected_container(View const& view, container_type* md)
    : m_container(md), m_version(view.version())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the current version number
  //////////////////////////////////////////////////////////////////////
  size_t version(void) const
  {
    return m_version;
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc growable_container::size(void)
  //////////////////////////////////////////////////////////////////////
  size_t size() const
  {
   return m_container->size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc growable_container::dimensions(void)
  //////////////////////////////////////////////////////////////////////
  dimensions_type dimensions(void) const
  {
    return m_container->dimensions();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc growable_container::domain(void)
  //////////////////////////////////////////////////////////////////////
  domain_type domain(void) const
  {
    return m_container->domain();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the metadata entry at global position @p index
  //////////////////////////////////////////////////////////////////////
  reference operator[](index_type index)
  {
    return transform_type()(m_container->operator[](index));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return a begin iterator to the metadata entries that are
  ///        local to this location.
  //////////////////////////////////////////////////////////////////////
  iterator begin()
  {
    return iterator{m_container->begin(), transform_type{}};
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return an end iterator to the metadata entries that are
  ///        local to this location.
  //////////////////////////////////////////////////////////////////////
  iterator end()
  {
    return iterator{m_container->end(), transform_type{}};
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc growable_container::get_local_vid(size_t)
  //////////////////////////////////////////////////////////////////////
  index_type get_local_vid(index_type index) const
  {
    return m_container->get_local_vid(index);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc growable_container::push_back_here(MD const&)
  ///
  /// This class does not support @c push_back_here.
  //////////////////////////////////////////////////////////////////////
  template <typename T>
  void push_back_here(T const&)
  {
    stapl::abort("Mutate method called on static metadata container");
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc growable_container::local_size(void)
  //////////////////////////////////////////////////////////////////////
  size_t local_size(void) const
  {
    return m_container->local_size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc growable_container::local_dimensions(void)
  //////////////////////////////////////////////////////////////////////
  dimensions_type local_dimensions() const
  {
    return m_container->local_dimensions();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc growable_container::get_location_element(size_t)
  //////////////////////////////////////////////////////////////////////
  location_type get_location_element(index_type index) const
  {
    return m_container->get_location_element(index);
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc flat_container::update
  /// This class does not support @c push_back_here.
  //////////////////////////////////////////////////////////////////////
  void update()
  {
    stapl::abort("Mutate method called on static metadata container");
  }

  void define_type(typer& t)
  {
    t.member(m_container);
    t.member(m_version);
  }
};

} // namespace metadata

template<typename View, typename MetadataContainer>
struct metadata_traits<metadata::projected_container<View, MetadataContainer>>
{
  using is_isomorphic =
    typename metadata_traits<MetadataContainer>::is_isomorphic;
};

} // namespace stapl

#endif // STAPL_VIEWS_METADATA_CONTAINER_BY_DEMAND_HPP
