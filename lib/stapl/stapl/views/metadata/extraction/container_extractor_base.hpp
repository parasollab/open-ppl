/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_METADATA_EXTRACTION_CONTAINER_BASE_HPP
#define STAPL_VIEWS_METADATA_EXTRACTION_CONTAINER_BASE_HPP

#include <stapl/containers/partitions/balanced.hpp>
#include <stapl/views/metadata/metadata_entry.hpp>
#include <stapl/views/metadata/utility/invoker.hpp>
#include <stapl/containers/distribution/is_distribution_view_fwd.hpp>

namespace stapl {

namespace metadata {

template<typename MD>
class flat_container;

template<typename MD>
class multidimensional_container;

namespace detail {

template<typename MD, int D>
struct select_container_type
{
  using type = multidimensional_container<MD>;
};

template<typename MD>
struct select_container_type<MD, 1>
{
  using type = flat_container<MD>;
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper functor to create an extractor on a remote location and
/// return result of calling @ref local_entries.
///
/// Used in @ref container_extractor_base::single_location_extraction
/// instead of lambda to reduce compiler pressure.
//////////////////////////////////////////////////////////////////////
template<typename Extractor>
struct extract_local
{
  using result_type       = typename Extractor::md_collection_type;
  using distribution_type = typename Extractor::distribution_type;

  result_type operator()(distribution_type const& dist) const
  {
    Extractor extractor;
    return extractor.local_entries(const_cast<distribution_type*>(&dist));
  }
};

} // namespace detail


//////////////////////////////////////////////////////////////////////
/// @brief Traits class specialized by derived classes of
/// @ref container_extractor_base to provide access to distribution type
/// during inheritance with CRTP to avoid additional template parameter
/// which unnecessarily stresses compiler.
//////////////////////////////////////////////////////////////////////
template<typename Extractor>
struct extractor_traits
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Class for computing the metadata of static containers that
/// use @ref container_manager_static that inherits from the base-container,
/// and thus, is the base-container itself. This type of container manager
/// has only one component per location.
/// @tparam Distribution Type of the Distribution.
/// @note this is different from the @ref static_array_metadata
/// primarily due to the metadata used. This class uses the
/// @ref flat_container whereas the @ref static_array_metadata
/// uses the metadata_container_wrapper.
//////////////////////////////////////////////////////////////////////
template<typename Derived>
class container_extractor_base
{
public:
  using distribution_type =
    typename extractor_traits<Derived>::distribution_type;

private:
  STAPL_IMPORT_TYPE(typename distribution_type, container_manager_type)
  STAPL_IMPORT_TYPE(typename distribution_type, container_type)
  STAPL_IMPORT_TYPE(typename container_manager_type, base_container_type)
  STAPL_IMPORT_TYPE(typename base_container_type, domain_type)
  STAPL_IMPORT_TYPE(typename base_container_type, cid_type)
  STAPL_IMPORT_TYPE(typename base_container_type, gid_type)

  using index_type = gid_type;

  using dimension_type = typename dimension_traits<index_type>::type;

  using md_entry_type =
    metadata_entry<domain_type, base_container_type*, index_type>;

  using md_container_type =
    typename detail::select_container_type<
      md_entry_type, dimension_type::value
    >::type;

public:
  using md_collection_type = std::vector<std::pair<index_type, md_entry_type>>;

  using return_type = std::pair<bool, md_container_type*>;

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a collection of metadata entries for all of the entries
  ///        on this location.
  /// @param dist The distribution for the container that the extraction
  ///             process is for
  /// @return A collection of pairs, where the first element is the index
  ///         of an entry and the second element is the entry itself
  //////////////////////////////////////////////////////////////////////
  md_collection_type local_entries(distribution_type* dist) const
  {
    md_collection_type coll;
    coll.reserve(dist->container_manager().size());

    // for each base container
    for (auto&& bc : dist->container_manager())
    {
      if (bc.domain().empty())
        continue;

      domain_type new_dom(bc.domain().first(), bc.domain().last());

      // compute the correct index and cid
      auto const index = derived().index_of(dist, bc.cid());
      auto const cid   = derived().cid_of(dist, index);

      // add to the collection of entries
      coll.push_back(std::make_pair(
        index,
        md_entry_type(
          cid, new_dom, &bc, LQ_CERTAIN, get_affinity(),
          dist->get_rmi_handle(), dist->get_location_id()
        ))
      );
    }

    return coll;
  }


private:
  Derived const& derived() const
  {
    return static_cast<Derived const&>(*this);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Populate a metadata container with metadata entries for
  ///        a container that is in the same gang as the calling context.
  ///
  /// @param dist The distribution for the container that the extraction
  ///             process is for
  /// @param md_cont The metadata container to populate
  //////////////////////////////////////////////////////////////////////
  return_type
  same_cardinality_extraction(distribution_type* dist,
                              md_container_type* md_cont) const
  {
    auto&& entries = local_entries(dist);

    // for each local entry
    for (auto&& entry : entries)
      md_cont->operator[](entry.first) = entry.second;

    md_cont->update();

    // Fixed size metadata container created in same gang,
    // enable static metadata optimization in projection and alignment
    // unless the distribution is view-based
    return std::make_pair(
      !is_arbitrary_view_based<
        typename distribution_type::partition_type>()(dist) &&
      md_cont->size() == md_cont->get_num_locations(),
      md_cont);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Populate a metadata container with metadata entries from a
  ///        a single location for a container that is in a different gang.
  ///
  /// @param dist The distribution for the container that the extraction
  ///             process is for
  /// @param md_cont The metadata container to populate
  ///
  /// @todo This is not a scalable method for extraction. Ideally, we should
  ///       reinvoke coarsening on the same set of locations in the same gang
  ///       as the container.
  //////////////////////////////////////////////////////////////////////
  return_type
  single_location_extraction(distribution_type* dist,
                             md_container_type* md_cont) const
  {
    using extract_func_t = detail::extract_local<container_extractor_base>;
    using invoker_type   = invoker<distribution_type, extract_func_t>;

    auto invoker = make_invoker(*dist, extract_func_t());

    // call extract_local on all of the locations of the distribution
    auto h1 = opaque_rmi(all_locations, invoker,
                         &invoker_type::invoke_and_delete,
                         dist->get_rmi_handle());

    // for each location in the container
    for (size_t i = 0; i < h1.size(); ++i)
    {
      auto&& mds = h1.get(i);

      // for each entry on that location
      for (auto&& entry : mds)
        md_cont->operator[](entry.first) = entry.second;
    }

    md_cont->update();

    return std::make_pair(false, md_cont);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Populate a metadata container with metadata entries for
  ///        a container that is in a different gang, where the current
  ///        gang and the container's gang have differing number of
  ///        locations.
  ///
  /// @param dist The distribution for the container that the extraction
  ///             process is for
  /// @param md_cont The metadata container to populate
  //////////////////////////////////////////////////////////////////////
  return_type
  differing_cardinality_extraction(distribution_type* dist,
                                   md_container_type* md_cont) const
  {
    abort("Metadata extraction for a view from a different gang.");

    return std::make_pair(false, md_cont);
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Return the metadata of a reference to a container.
  /// @see metadata_entry.
  /// @param cont A pointer to the container.
  ///
  /// Calls the operator on the distribution of the provided container.
  //////////////////////////////////////////////////////////////////////
  template <typename Container, typename Accessor>
  return_type operator()(proxy<Container, Accessor>* proxy_cont) const
  {
    return operator()(proxy_cont->get_distribution());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the metadata of the specified container.
  /// @see metadata_entry.
  /// @param cont A pointer to the container.
  ///
  /// Calls the operator on the distribution of the provided container.
  //////////////////////////////////////////////////////////////////////
  return_type operator()(container_type* cont) const
  {
    return operator()(cont->get_distribution());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the metadata of the specified distribution.
  /// @see metadata_entry.
  /// @param dist A pointer to the distribution.
  //////////////////////////////////////////////////////////////////////
  return_type operator()(distribution_type* dist) const
  {
    auto const num_comp  = derived().num_entries(dist);

    md_container_type* md_cont = new md_container_type(num_comp);

    const bool b_gangs_conformable =
      compare_gangs(md_cont->get_rmi_handle(), dist->get_rmi_handle()) >= 0;

    // if the container is in the same gang of the current calling context
    if (b_gangs_conformable)
      return same_cardinality_extraction(dist, md_cont);

    // the container exists in a different gang at this point

    // execution in one location, data in different locations
    if (md_cont->get_num_locations() == 1)
      return single_location_extraction(dist, md_cont);

    // the container's gang is different, but it has the same number of
    // locations
    else if (md_cont->get_num_locations() == dist->get_num_locations())
      return same_cardinality_extraction(dist, md_cont);

     // the container's gang is different from the current gang in terms
     // of the number of locations (n-to-m)
    else
      return differing_cardinality_extraction(dist, md_cont);
  }
}; // class container_extractor_base

} // namespace metadata

} // namespace stapl

#endif
