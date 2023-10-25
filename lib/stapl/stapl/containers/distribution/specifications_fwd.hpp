/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_DISTRIBUTION_SPECIFICATIONS_FWD_HPP
#define STAPL_CONTAINERS_DISTRIBUTION_SPECIFICATIONS_FWD_HPP

#include <stapl/views/system_view.hpp>
#include <stapl/views/distribution_spec_view.hpp>
#include <stapl/utility/integer_sequence.hpp>
#include <stapl/domains/deferred.hpp>
#include "specification_functors.hpp"
#include <stapl/containers/partitions/block_partition.hpp>

namespace stapl {

namespace dist_spec_impl {

template <typename GIDDomain = stapl::indexed_domain<unsigned long int>>
struct distribution_spec
{
  typedef distribution_spec_view<
            distribution_spec_view<
              dist_view_impl::system_container,
              stapl::indexed_domain<location_type>,
              location_type, location_type
            >,
            deferred_domain<indexed_domain<unsigned long int>>,
            typename GIDDomain::index_type, unsigned long int
          >                                              mapping_view_type;
  typedef distribution_spec_view<
            mapping_view_type,
            GIDDomain,
            typename GIDDomain::index_type, unsigned long int
          >                                              partitioning_view_type;

  typedef partitioning_view_type type;
};

} // namespace dist_spec_impl


//////////////////////////////////////////////////////////////////////
/// @brief The type of the view that provides the specification
/// of a one-dimensional container distribution.
//////////////////////////////////////////////////////////////////////
template<typename GIDDomain = stapl::indexed_domain<unsigned long int>>
using distribution_spec =
  typename dist_spec_impl::distribution_spec<GIDDomain>::type;


template<typename Traversal>
class md_distribution_spec_view;


//////////////////////////////////////////////////////////////////////
/// @brief Reflects the type of the view that provides the specification
/// of a volumetric distribution of a multi-dimensional array.
//////////////////////////////////////////////////////////////////////
template<typename Traversal>
struct md_distribution_spec;


template<std::size_t... TraversalIndices>
struct md_distribution_spec<index_sequence<TraversalIndices...>>
{
private:
  enum { dimensions = sizeof...(TraversalIndices) };

public:
  typedef tuple<boost::mpl::int_<TraversalIndices>...>   traversal_type;

  typedef stapl::indexed_domain<
    unsigned long int, dimensions, traversal_type>       domain_type;

  typedef stapl::indexed_domain<
    unsigned long int, dimensions, traversal_type>       partition_domain_type;

  STAPL_IMPORT_DTYPE(domain_type, gid_type);

  typedef distribution_spec_view<
    dist_view_impl::system_container,
    stapl::indexed_domain<location_type>,
    gid_type, gid_type>                                  system_view_type;

  typedef distribution_spec_view<
    system_view_type,
    deferred_domain<domain_type>,
    gid_type, gid_type>                                  mapping_view_type;

  typedef distribution_spec_view<
    mapping_view_type, domain_type, gid_type, gid_type>  partitioning_view_type;

  typedef md_distribution_spec_view<
    index_sequence<TraversalIndices...>>                    type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Wrapper class around md_distribution nest of views.
///  Exists for clarity and to reduce type's symbol size.
//////////////////////////////////////////////////////////////////////
template<std::size_t... TraversalIndices>
class md_distribution_spec_view<
  index_sequence<TraversalIndices...>>
  : public md_distribution_spec<
      index_sequence<TraversalIndices...>
    >::partitioning_view_type
{
private:
  typedef typename md_distribution_spec<
    index_sequence<TraversalIndices...>
  >::partitioning_view_type                             base_type;

public:
  typedef tuple<boost::mpl::int_<TraversalIndices>...>  traversal_type;

  template<typename... Args>
  md_distribution_spec_view(Args&&... args)
    : base_type(std::forward<Args>(args)...)
  { }
}; // class md_distribution_spec_view


template<typename PartitionDimensions, typename Traversal>
class sliced_md_distribution_spec_view;


//////////////////////////////////////////////////////////////////////
/// @brief Reflects the type of the view that provides the specification
///   of a volumetric distribution of a multi-dimensional array based on a
///   subset (possibly permuted) of the dimensions of the container.
//////////////////////////////////////////////////////////////////////
template<typename PartitionDimensions, typename Traversal>
struct sliced_md_distribution_spec;


template<std::size_t... PartitionIndices, std::size_t... TraversalIndices>
struct sliced_md_distribution_spec<
  index_sequence<PartitionIndices...>, index_sequence<TraversalIndices...>>
{
  typedef tuple<boost::mpl::int_<TraversalIndices>...> traversal_type;

  typedef stapl::indexed_domain<
    unsigned long int,
    sizeof...(TraversalIndices),
    traversal_type>                                    domain_type;

  STAPL_IMPORT_DTYPE(domain_type, gid_type);

  static constexpr int PartitionDimensions = sizeof...(PartitionIndices);

  typedef stapl::indexed_domain<
    unsigned long int, PartitionDimensions,
    typename default_traversal<
      PartitionDimensions>::type>                      partition_domain_type;

  typedef typename partition_domain_type::gid_type     partition_gid_type;

  typedef distribution_spec_view<
      dist_view_impl::system_container,
      stapl::indexed_domain<location_type>,
      gid_type, partition_gid_type>                    system_view_type;

  typedef distribution_spec_view<
    system_view_type,
    deferred_domain<partition_domain_type>,
    gid_type, partition_gid_type>                      mapping_view_type;

  typedef distribution_spec_view<
    mapping_view_type, domain_type,
    gid_type, partition_gid_type>                      partitioning_view_type;

  typedef sliced_md_distribution_spec_view<
    index_sequence<PartitionIndices...>,
    index_sequence<TraversalIndices...>>                  type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Wrapper class around sliced_md_distribution nest of views.
///  Exists for clarity and to reduce type's symbol size.
//////////////////////////////////////////////////////////////////////
template<std::size_t... PartitionIndices, std::size_t... TraversalIndices>
class sliced_md_distribution_spec_view<
  index_sequence<PartitionIndices...>, index_sequence<TraversalIndices...>>
  : public sliced_md_distribution_spec<
      index_sequence<PartitionIndices...>, index_sequence<TraversalIndices...>
    >::partitioning_view_type
{
private:
  typedef typename sliced_md_distribution_spec<
    index_sequence<PartitionIndices...>, index_sequence<TraversalIndices...>
  >::partitioning_view_type                             base_type;

public:
  typedef tuple<boost::mpl::int_<TraversalIndices>...>  traversal_type;
  typedef index_sequence<PartitionIndices...>   partition_dimensions_type;

  template<typename... Args>
  sliced_md_distribution_spec_view(Args&&... args)
    : base_type(std::forward<Args>(args)...)
  { }
}; // class sliced_md_distribution_spec_view


namespace dist_spec_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Function invoked by view-based partition and mapper instances
/// to initialize any deferred domain and mapping functions they contain.
///
/// The initialize flag is provided to allow the invocation of the mapping
/// functions to be avoided in cases where it is known that they are not
/// instances of @ref deferred_map, specifically, the specification of
/// arbitrary distributions using containers of @ref arb_partition_info
/// instances.
///
/// @param spec View-based specification of container distribution that may
/// contain instances of @ref deferred_domain and @ref deferred_map
/// @param initialize flag indicating whether initialization of
/// @ref deferred_map instances should be performed
/// @return A reference to the instance passed to the function
///
/// This function is invoked inline in container constructors where the
/// partition and mapper instances are constructed before they're passed
/// to the distribution constructor.
//////////////////////////////////////////////////////////////////////
template <typename DistSpec>
std::shared_ptr<DistSpec>
initialize_deferred(std::shared_ptr<DistSpec> spec, bool initialize = true)
{
  if (initialize)
  {
    // Trigger initialization of deferred_map functor in partition view
    spec->mapfunc()->operator()(typename DistSpec::index_type());

    // Trigger initialization of deferred_map functor in mapping view
    spec->container().mapfunc()->operator()(
      typename DistSpec::view_container_type::index_type());
  }

  // Trigger initialization of deferred_domain in mapping view
  spec->container().domain().first();

  return spec;
}

} // namespace dist_spec_impl

//////////////////////////////////////////////////////////////////////
/// @brief Representation of a partition in the specification of an
/// arbitrary distribution.
///
/// The partition information required by the mapping functions used
/// in the view-based distribution specification is the range of GIDs
/// in the partition and the location to which the partition will be mapped.
/// The partition id is implicit and is the index of the partition info
/// in the container of partition information that is passed to
/// @ref arbitrary(View const& part_view,std::vector<location_type> const& locs)
//////////////////////////////////////////////////////////////////////
struct arbitrary_partition_info
{
private:
  unsigned long int m_first;
  unsigned long int m_last;
  unsigned int      m_loc;

public:
  arbitrary_partition_info()
    : m_first(-1), m_last(-1), m_loc(-1)
  { }

  arbitrary_partition_info(unsigned long int first, unsigned long int last,
                           unsigned int location)
    : m_first(first), m_last(last), m_loc(location)
  { }

  std::pair<unsigned long int, unsigned long int> domain(void) const
  { return std::make_pair(m_first, m_last); }

  unsigned int location(void) const
  { return m_loc; }

  void define_type(stapl::typer& t)
  {
    t.member(m_first);
    t.member(m_last);
    t.member(m_loc);
  }
};


STAPL_PROXY_HEADER(arbitrary_partition_info)
{
  STAPL_PROXY_DEFINES(arbitrary_partition_info)
  STAPL_PROXY_METHOD_RETURN(location, unsigned int)
  STAPL_PROXY_METHOD_RETURN_0(domain,
    STAPL_PROXY_CONCAT(std::pair<unsigned long int, unsigned long int>))
};



//////////////////////////////////////////////////////////////////////
/// @brief Distribution specification of nested containers.
///
/// In order to construct the elements of a nested container instance
/// with different distributions a specification of the distribution
/// for each element is needed. The class is constructed with a user
/// defined generator object whose operator[] receives the multi-dimensional
/// index of the element being constructed and returns the distribution
/// specification for the element.
//////////////////////////////////////////////////////////////////////
class composed_dist_spec;

namespace detail {
template<typename GID, typename Index>
struct balance_map_factory;
}
//////////////////////////////////////////////////////////////////////
/// @brief Construct the specification of a blocked distribution.
///
/// The GIDs will be blocked into partitions, and the partition ids are
/// balanced across the locations.
///
/// @param n Number of elements in the data to be distributed
/// @param block_size Number of elements in each partition
/// @param lvl Tag indicating the level of the system hierarchy across which
/// the data will be distributed.
/// @return A read-only view with the domains and mapping functions that
/// result in a blocked distribution of GIDs.
/// @ingroup distribution_specs
//////////////////////////////////////////////////////////////////////
distribution_spec<>
block(unsigned long int n, unsigned long int block_size,
      level lvl = current_level);


//////////////////////////////////////////////////////////////////////
/// @brief Construct the specification of a blocked distribution
/// that will span a specific set of locations.
///
/// The GIDs will be blocked into partitions, and the partition ids are
/// balanced across the locations in the new communication group
/// constructed over the locations specified from the parent
/// communication group.
///
/// @param n Number of elements in the data to be distributed
/// @param block_size Number of elements in each partition
/// @param locs Explicit specification of the locations across
/// which the data will be distributed.
/// @return A read-only view with the domains and mapping functions that
/// result in a blocked distribution of GIDs.
/// @ingroup distribution_specs
//////////////////////////////////////////////////////////////////////
distribution_spec<>
block(unsigned long int n, unsigned long int block_size,
      std::vector<location_type> const& locs);


//////////////////////////////////////////////////////////////////////
/// @brief Construct the specification of a cyclic distribution.
///
/// The GIDs will not be blocked.  Each single-element partition will be
/// mapped to a location in a round-robin manner beginning with location 0.
///
/// @param n Number of elements in the data to be distributed
/// @param lvl Tag indicating the level of the system hierarchy across which
/// the data will be distributed.
/// @return A read-only view with the domains and mapping functions that
/// result in a cyclic distribution of GIDs.
/// @ingroup distribution_specs
///
//////////////////////////////////////////////////////////////////////
distribution_spec<>
cyclic(unsigned long int n, level lvl = current_level);


//////////////////////////////////////////////////////////////////////
/// @brief Construct the specification of a cyclic distribution
/// that will span a specific set of locations.
///
/// The GIDs will not be blocked.  Each single-element partition will be
/// mapped to a location in a round-robin manner beginning with location 0
/// in the new communication group constructed over the locations
/// specified from the parent communication group.
///
/// @param n Number of elements in the data to be distributed
/// @param locs Explicit specification of the locations across
/// which the data will be distributed.
/// @return A read-only view with the domains and mapping functions that
/// result in a cyclic distribution of GIDs.
/// @ingroup distribution_specs
//////////////////////////////////////////////////////////////////////
distribution_spec<>
cyclic(unsigned long int n, std::vector<location_type> const& locs);


//////////////////////////////////////////////////////////////////////
/// @brief Construct the specification of a block-cyclic distribution.
///
/// The GIDs will be mapped into partitions of size @p block_size. Each
/// partition will be mapped to a location in a round-robin manner
/// beginning with location 0.
///
/// @param n Number of elements in the data to be distributed
/// @param block_size Number of elements in each partition
/// @param lvl Tag indicating the level of the system hierarchy across which
/// the data will be distributed.
/// @return A read-only view with the domains and mapping functions that
/// result in a block-cyclic distribution of GIDs.
/// @ingroup distribution_specs
///
//////////////////////////////////////////////////////////////////////
distribution_spec<>
block_cyclic(unsigned long int n, unsigned long int block_size,
             level lvl = current_level);


//////////////////////////////////////////////////////////////////////
/// @brief Construct the specification of a block-cyclic distribution
/// that will span a specific set of locations.
///
/// The GIDs will be mapped into partitions of size @p block_size. Each
/// partition will be mapped to a location in a round-robin manner
/// beginning with location 0 in the new communication group constructed
/// over the locations specified from the parent communication group.
///
/// @param n Number of elements in the data to be distributed
/// @param block_size Number of elements in each partition
/// @param locs Explicit specification of the locations across
/// which the data will be distributed.
/// @return A read-only view with the domains and mapping functions that
/// result in a block-cyclic distribution of GIDs.
/// @ingroup distribution_specs
//////////////////////////////////////////////////////////////////////
distribution_spec<>
block_cyclic(unsigned long int n, unsigned long int block_size,
             std::vector<location_type> const& locs);


//////////////////////////////////////////////////////////////////////
/// @brief Construct the specification of a balanced distribution.
///
/// The GIDs will be mapped into partitions of near even size. Each
/// of the @p num_locs partitions will be mapped to a unique location
/// beginning with location 0.
///
/// @param n Number of elements in the data to be distributed
/// @param lvl Tag indicating the level of the system hierarchy across which
/// the data will be distributed.
/// @return A read-only view with the domains and mapping functions that
/// result in a balanced distribution of GIDs.
/// @ingroup distribution_specs
///
//////////////////////////////////////////////////////////////////////
distribution_spec<>
balance(unsigned long int n, level lvl = current_level);


//////////////////////////////////////////////////////////////////////
/// @brief Construct the specification of a balanced distribution
/// that will span a specific set of locations.
///
/// The GIDs will be mapped into partitions of near even size. Each
/// of the @p num_locs partitions will be mapped to a unique location
/// beginning with location 0 in the new communication group constructed
/// over the locations specified from the parent communication group.
///
/// @param n Number of elements in the data to be distributed
/// @param locs Explicit specification of the locations across
/// which the data will be distributed.
/// @return A read-only view with the domains and mapping functions that
/// result in a balanced distribution of GIDs.
/// @ingroup distribution_specs
//////////////////////////////////////////////////////////////////////
distribution_spec<>
balance(unsigned long int n,
        std::vector<location_type> const& locs);


//////////////////////////////////////////////////////////////////////
/// @brief Construct the specification of an arbitrary distribution.
///
/// The mapping functions to map an element GID to a partition id and
/// a partition id to a location id are function objects that have the
/// appropriately defined function operators.  Given a functor to map
/// GIDS to partition ids of type @p GIDMap and a functor to map partition
/// ids to location ids of type @p PIDMap the interfaces required are:
/// @verbatim
/// unsigned long int GIDMap::operator()(unsigned long int) const;
/// stapl::location_type PIDMap::operator()(unsigned long int) const;
/// @endverbatim
///
/// @param n Number of elements in the data to be distributed
/// @param nparts Number of partitions the elements are mapped to as
/// part of the distribution process.
/// @param gid_to_pid Functor mapping element GIDs to partition ids
/// @param pid_to_lid Functor mapping partition ids to location ids
/// @param lvl Tag indicating the level of the system hierarchy across which
/// the data will be distributed.
/// @return A read-only view with the domains and mapping functions that
/// result in the arbitrary distribution of GIDS specified by the mapping
/// functions provided.
/// @ingroup distribution_specs
///
//////////////////////////////////////////////////////////////////////
template <typename GIDMapFunc, typename PIDMapFunc>
distribution_spec<>
arbitrary(unsigned long int n, unsigned long int nparts,
          GIDMapFunc const &gid_to_pid,
          PIDMapFunc const &pid_to_lid,
          level lvl = current_level);

//////////////////////////////////////////////////////////////////////
/// @brief Construct the specification of an arbitrary distribution.
///
/// The mapping functions to map an element GID to a partition id and
/// a partition id to a location id are function objects that have the
/// appropriately defined function operators.  Given a functor to map
/// GIDS to partition ids of type @p GIDMap and a functor to map partition
/// ids to location ids of type @p PIDMap the interfaces required are:
/// @verbatim
/// unsigned long int GIDMap::operator()(gid_type) const;
/// stapl::location_type PIDMap::operator()(unsigned long int) const;
/// @endverbatim
///
/// @param dom The domain used for the container.
/// @param nparts Number of partitions the elements are mapped to as
/// part of the distribution process.
/// @param gid_to_pid Functor mapping element GIDs to partition ids
/// @param pid_to_lid Functor mapping partition ids to location ids
/// @param lvl Tag indicating the level of the system hierarchy across which
/// the data will be distributed.
/// @return A read-only view with the domains and mapping functions that
/// result in the arbitrary distribution of GIDS specified by the mapping
/// functions provided.
/// @ingroup distribution_specs
///
/// @note The enable_if is here to help the compiler differentiate with
/// the arbitrary(unsigned long int n, unsigned long int nparts, ...) function
//////////////////////////////////////////////////////////////////////
template <typename Dom, typename GIDMapFunc, typename PIDMapFunc,
 typename = typename std::enable_if<!std::is_integral<Dom>::value>::type
>
distribution_spec<Dom>
arbitrary(Dom const& dom,
          unsigned long int nparts,
          GIDMapFunc const &gid_to_pid,
          PIDMapFunc const &pid_to_lid,
          level lvl = current_level);


//////////////////////////////////////////////////////////////////////
/// @brief Construct the specification of an arbitrary distribution
/// that will span a specific set of locations.
///
/// The mapping functions to map an element GID to a partition id and
/// a partition id to a location id are function objects that have the
/// appropriately defined function operators.  Given a functor to map
/// GIDS to partition ids of type @p GIDMap and a functor to map partition
/// ids to location ids of type @p PIDMap the interfaces required are:
/// @verbatim
/// unsigned long int GIDMap::operator()(unsigned long int) const;
/// stapl::location_type PIDMap::operator()(unsigned long int) const;
/// @endverbatim
///
/// The locations to which the partitions will be mapped begin with
/// location 0 in the new communication group constructed over the
/// locations specified from the parent communication group.
///
/// @param n Number of elements in the data to be distributed
/// @param nparts Number of partitions the elements are mapped to as
/// part of the distribution process.
/// @param gid_to_pid Functor mapping element GIDs to partition ids
/// @param pid_to_lid Functor mapping partition ids to location ids
/// @param locs Explicit specification of the locations across
/// which the data will be distributed.
/// @return A read-only view with the domains and mapping functions that
/// result in the arbitrary distribution of GIDS specified by the mapping
/// functions provided.
/// @ingroup distribution_specs
//////////////////////////////////////////////////////////////////////
template <typename GIDMapFunc, typename PIDMapFunc>
distribution_spec<>
arbitrary(unsigned long int n, unsigned long int nparts,
          GIDMapFunc const &gid_to_pid,
          PIDMapFunc const &pid_to_lid,
          std::vector<location_type> const& locs);


//////////////////////////////////////////////////////////////////////
/// @brief Construct the specification of an arbitrary distribution.
///
/// @param part_view A view whose elements define the partitions of the
/// distribution and the location to which each one is mapped.
/// @param lvl Tag indicating the level of the system hierarchy across which
/// the data will be distributed.
/// @return A read-only view with the domains and mapping functions that
/// result in the arbitrary distribution of GIDs specified by the partition
/// information provided.
///
/// Each element of the view can be an instance of
/// @ref arbitrary_partition_info.  If another type is used the interface it
/// must provide is:
/// @beginverbatim
/// std::pair<unsigned long int, unsigned long int> domain(void) const;
/// unsigned int location(void) const;
/// @endverbatim
///
/// These methods provide the information needed to construct each partition
/// in the container whose distribution is being specified.  The partition id
/// isn't explicitly provided.  Instead the index of the element in the
/// container is used as an implicit partition id.
/// @ingroup distribution_specs
///
//////////////////////////////////////////////////////////////////////
template <typename View>
distribution_spec<>
arbitrary(View const& part_view, level lvl = current_level);


//////////////////////////////////////////////////////////////////////
/// @brief Construct the specification of an arbitrary distribution
/// that will span a specific set of locations.
///
/// @param part_view A view whose elements define the partitions of the
/// distribution and the location to which each one is mapped.
/// @param locs Explicit specification of the locations across
/// which the data will be distributed.
/// @return A read-only view with the domains and mapping functions that
/// result in the arbitrary distribution of GIDs specified by the partition
/// information provided.
///
/// Each element of the view can be an instance of
/// @ref arbitrary_partition_info.  If another type is used the interface it
/// must provide is:
/// @beginverbatim
/// std::pair<unsigned long int, unsigned long int> domain(void) const;
/// unsigned int location(void) const;
/// @endverbatim
///
/// These methods provide the information needed to construct each partition
/// in the container whose distribution is being specified.  The partition id
/// isn't explicitly provided.  Instead the index of the element in the
/// container is used as an implicit partition id.
///
/// The locations to which the partitions will be mapped begin with
/// location 0 in the new communication group constructed over the
/// locations specified from the parent communication group.
/// @ingroup distribution_specs
//////////////////////////////////////////////////////////////////////
template <typename View>
distribution_spec<>
arbitrary(View const& part_view, std::vector<location_type> const& locs);


//////////////////////////////////////////////////////////////////////
/// @brief Construct the specification of a volumetric distribution of
/// a multidimensional container.
///
/// A volumetric distribution creates a location layout that matches
/// the dimensionality of the data structure to be distributed with
/// the number of locations in each dimension as balanced as possible.
/// The data is then distributed in a balanced manner in each dimension
/// across the number of locations in that dimension.
///
/// The GIDs are multidimensional and map to multidimensional partition ids
/// The mapping of partition ids to location ids will perform a linearization
/// to a scalar as part of the mapping.  Each partition will be mapped to a
/// unique location beginning with location 0.
///
/// The location layout used is organized in a major order that proceeds from
/// the final dimension to the first.  For example, in a three-dimensional
/// layout of eight locations location 0 would be in (0,0,0) of the layout,
/// location 1 would be in (0,0,1), location 2 would be in (0,1,0), etc.,
/// with location 7 at the final position of (1,1,1).
///
/// @param n Number of elements in the data to be distributed
/// @param lvl Tag indicating the level of the system hierarchy across which
/// the data will be distributed.
/// @return A read-only view with the domains and mapping functions that
/// result in a balanced distribution of GIDs.
/// @ingroup distribution_specs
//////////////////////////////////////////////////////////////////////
template <typename Traversal, typename Size>
typename md_distribution_spec<Traversal>::type
volumetric(Size const& n, level lvl = current_level);


//////////////////////////////////////////////////////////////////////
/// @brief Construct the specification of a volumetric distribution of
/// a multidimensional container.
///
/// A volumetric distribution creates a location layout that matches
/// the dimensionality of the data structure to be distributed with
/// the number of locations in each dimension as balanced as possible.
/// The data is then distributed in a balanced manner in each dimension
/// across the number of locations in that dimension.
///
/// The GIDs are multidimensional and map to multidimensional partition ids
/// The mapping of partition ids to location ids will perform a linearization
/// to a scalar as part of the mapping.  Each partition will be mapped to a
/// unique location beginning with location 0.
///
/// The location layout used is organized in a major order that proceeds from
/// the final dimension to the first.  For example, in a three-dimensional
/// layout of eight locations location 0 would be in (0,0,0) of the layout,
/// location 1 would be in (0,0,1), location 2 would be in (0,1,0), etc.,
/// with location 7 at the final position of (1,1,1). The locations used in
/// the layout are in a new communication group constructed over the locations
/// specified from the parent communication group.
///
/// @tparam Traversal Explicitly specified tuple type of integral constants
///   which are the dimension traversal order for container elements.
/// @param n Number of elements in the data to be distributed
/// @param locs Explicit specification of the locations across
/// which the data will be distributed.
/// @return A read-only view with the domains and mapping functions that
/// result in a balanced distribution of GIDs.
/// @ingroup distribution_specs
//////////////////////////////////////////////////////////////////////
template <typename Traversal, typename Size>
typename md_distribution_spec<Traversal>::type
volumetric(Size const& n, std::vector<location_type> const& locs);


//////////////////////////////////////////////////////////////////////
/// @brief Construct the specification of a volumetric distribution of a
/// multidimensional container based on a subset (possibly permuted)
/// of the dimensions of the container.
///
/// The location layout dimensionality matches this slice of the
/// multiarray's dimensions.
///
/// @see volumetric
/// @tparam Traversal Explicitly specified tuple type of integral constants
///   which are the dimension traversal order for container elements.
/// @tparam PartitionDimensions Explicitly specified tuple type of integral
///   which are the indices used in the sliced volumetric distribution.
/// @param n Number of elements in the data to be distributed
/// @param lvl Tag indicating the level of the system hierarchy across which
/// the data will be distributed.
/// @return A read-only view with the domains and mapping functions that
/// result in a balanced distribution of GIDs.
/// @ingroup distribution_specs
//////////////////////////////////////////////////////////////////////
template<typename Traversal, typename PartitionDimensions, typename Size>
typename sliced_md_distribution_spec<
  PartitionDimensions, Traversal
>::type
sliced_volumetric(Size const& n);


//////////////////////////////////////////////////////////////////////
/// @brief Construct the specification of a volumetric distribution of a
/// multidimensional container based on a subset (possibly permuted)
/// of the dimensions of the container.
///
/// @param n Size of the container to construct
/// @param sys_param specification of the set of locations that will store
///   the container
/// @param start GID of the first element in the container
/// @param agg_size combined size of a set of containers of which this
///   instance may be part
/// @ingroup distribution_specs
/// @todo Further generalized @ref nd_impl so that this distribution
///   can reuse its implementation.
//////////////////////////////////////////////////////////////////////
template<typename Traversal, typename PartitionDimensions,
         typename Size, typename SystemParam>
typename sliced_md_distribution_spec<
  PartitionDimensions, Traversal
>::type
sliced_volumetric(Size const& n, SystemParam&& sys_param,
                  Size const& start = Size(), Size const& agg_size = Size());


//////////////////////////////////////////////////////////////////////
/// @brief Construct the specification of a balanced distribution of a
/// a multidimensional container.
///
/// A balanced multidimensional distribution partitions elements among the
/// locations in an n-dimensional space, as defined by an input parameter.
///
/// @param n number of elements in the data to be distributed.
/// @param loc_dims a tuple with the number of locations to be
///   allocated in each dimension of the multidimensional layout.
/// @param lvl tag indicating the level of the system hierarchy across which
///   the data will be distributed.
/// @return a read-only view with the domains and mapping functions that
///   result in a balanced distribution of gids.
/// @ingroup distribution_specs
//////////////////////////////////////////////////////////////////////
template <typename Traversal, typename Size, typename LocDimensions>
typename md_distribution_spec<Traversal>::type
balanced_nd(Size const& n, LocDimensions const& loc_dims,
            level lvl = current_level);


//////////////////////////////////////////////////////////////////////
/// @brief Construct the specification of a balanced distribution of a
/// a multidimensional container.
///
/// A balanced multidimensional distribution partitions elements among the
/// locations in an n-dimensional space, as defined by an input parameter.
///
/// @param n number of elements in the data to be distributed.
/// @param loc_dims a tuple with the number of locations to be
///   allocated in each dimension of the multidimensional layout.
/// @param sys_param specification of the set of locations that will store
///   the container
/// @ingroup distribution_specs
//////////////////////////////////////////////////////////////////////
template <typename Traversal, typename Size,
          typename LocDimensions, typename SystemParam>
typename md_distribution_spec<Traversal>::type
balanced_nd(Size const& n,
            LocDimensions const& loc_dims,
            SystemParam&& sys_param);


//////////////////////////////////////////////////////////////////////
/// @brief Construct the specification of a uniform distribution of a
/// a multidimensional container.
///
/// A uniform multidimensional distribution partitions elements among the
/// locations in an n-dimensional space, as defined by an input parameter.
/// Unlike @ref balanced_nd, the expectation is that dimensions of @ref n
/// will be smaller than @ref loc_dims.  Input gids will be spaced out in
/// the location layout, making it useful as the top distribution of
/// composed distribution specification.
/// @param n Number of elements in the data to be distributed.
/// @param loc_dims A tuple with the number of locations to be
///   allocated in each dimension of the multidimensional layout.
/// @param lvl Tag indicating the level of the system hierarchy across which
///   the data will be distributed.
/// @return A read-only view with the domains and mapping functions that
///   result in a uniform distribution of GIDs.
/// @ingroup distribution_specs
//////////////////////////////////////////////////////////////////////
template <typename Traversal, typename Size, typename LocDimensions>
typename md_distribution_spec<Traversal>::type
uniform_nd(Size const& n, LocDimensions const& loc_dims,
           level lvl = current_level);


//////////////////////////////////////////////////////////////////////
/// @brief Construct the specification of a uniform distribution of a
/// a multidimensional container.
///
/// A uniform multidimensional distribution partitions elements among the
/// locations in an n-dimensional space, as defined by an input parameter.
/// Unlike @ref balanced_nd, the expectation is that dimensions of @ref n
/// will be smaller than @ref loc_dims.  Input gids will be spaced out in
/// the location layout, making it useful as the top distribution of
/// composed distribution specification.
/// @param n Number of elements in the data to be distributed.
/// @param loc_dims A tuple with the number of locations to be
///   allocated in each dimension of the multidimensional layout.
/// @param sys_param specification of the set of locations that will store
///   the container
/// @ingroup distribution_specs
//////////////////////////////////////////////////////////////////////
template <typename Traversal, typename Size,
          typename LocDimensions, typename SystemParam>
typename md_distribution_spec<Traversal>::type
uniform_nd(Size const& n,
           LocDimensions const& loc_dims,
           SystemParam&& sys_param);

} // namespace stapl

#endif // STAPL_CONTAINERS_DISTRIBUTION_SPECIFICATIONS_FWD_HPP
