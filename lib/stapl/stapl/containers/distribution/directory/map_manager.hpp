/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_MAP_MANAGER_H
#define STAPL_CONTAINERS_MAP_MANAGER_H

#include <boost/icl/type_traits/is_discrete.hpp>
#include <stapl/containers/distribution/container_manager/local_partition_info.hpp>

namespace stapl {

namespace detail {

template <typename CID>
struct cid_eq
{
private:
  CID m_cid;
public:
  cid_eq(CID cid)
    : m_cid(cid)
  {}

  template<typename Ref>
  bool operator()(Ref r) const
  {
    if (get<1>(r) != m_cid)
      return false;
    else
      return true;
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Extract the partition information from a partition and mapper
/// instance that represent an arbitrary distribution. The function is
/// used in map_manager to find the set of local domains.
///
/// @param partition The container's partition object
/// @param mapper The container's mapper object.
/// @return A vector of the partition information extracted from the partition
//////////////////////////////////////////////////////////////////////
template <typename Partition, typename Mapper>
std::vector<std::tuple<
  boost::icl::interval_set<typename Partition::value_type::index_type>,
  typename Mapper::cid_type, location_type>>
compute_local_domains(Partition const& partition, Mapper const& mapper)
{
  typedef typename Partition::value_type::index_type index_type;
  typedef std::tuple<boost::icl::interval_set<index_type>,
    typename Mapper::cid_type, location_type> bc_info_type;

  std::vector<bc_info_type> local_doms =
    cm_impl::get_local_partition_info(
      cm_impl::get_partial_partition_info(partition, mapper));

  // check to see if local domains need to be merged.
  std::set<typename Mapper::cid_type> local_cids;
  for (bc_info_type const& dom : local_doms)
    local_cids.insert(get<1>(dom));

  // if set size differs from local_doms size merging is needed.
  if (local_cids.size() != local_doms.size())
  {
    // new set of merge domain information
    std::vector<bc_info_type> merged_domains;
    merged_domains.reserve(local_cids.size());

    // Find all domain info for a cid and combine it into
    // a single interval_set.
    for (typename Mapper::cid_type const& cid : local_cids)
    {
      // Find first set of info for cid
      typename std::vector<bc_info_type>::iterator first_info =
        std::find_if(local_doms.begin(), local_doms.end(),
                     cid_eq<typename Mapper::cid_type>(cid));

      // scan remainder of vector, merging in to first_info's interval_set
      typename std::vector<bc_info_type>::iterator info = first_info + 1;
      for (; info != local_doms.end(); ++info)
      {
        if (get<1>(*info) == cid)
        {
          typedef boost::icl::interval_set<index_type> int_set_t;
          for (typename int_set_t::iterator int_it = get<0>(*info).begin();
               int_it != get<0>(*info).end(); ++int_it)
            get<0>(*first_info) += *int_it;
        }
      }

      // Add the merged info to merged_domains
      merged_domains.push_back(*first_info);
    }

    // merged_domains is populated, and local_doms has redundant info.
    // swap the contents of the contents of the vectors, so redundant info
    // will be deleted when merged_doms leaves scope.
    local_doms.swap(merged_domains);
  }

  return local_doms;
}

//////////////////////////////////////////////////////////////////////
/// @brief Extract the partition information from a balanced partition
/// instance. The function is used in map_manager to find the set of
/// local domains.
///
/// This function is a specialization for @ref balanced_partition instances
/// that utilizes the inverse mapping operation provided by the partition's
/// bracket operator.
///
/// @param partition The container's partition object
/// @param mapper The container's mapper object.
/// @return A vector of the partition information extracted from the partition
//////////////////////////////////////////////////////////////////////
template <typename Domain, typename Mapper>
std::vector<std::tuple<
  boost::icl::interval_set<typename Domain::index_type>,
  typename Mapper::cid_type, location_type>>
compute_local_domains(balanced_partition<Domain> const& partition,
                      Mapper const& mapper)
{
  location_type loc = stapl::get_location_id();
  Domain d(partition[loc]);

  typedef typename Domain::index_type index_type;
  typedef boost::icl::interval_set<index_type> interval_set_type;
  typedef std::vector<std::tuple<
    boost::icl::interval_set<typename Domain::index_type>,
    typename Mapper::cid_type, location_type>> result_type;
  return result_type(1,
    std::make_tuple(
      interval_set_type(
        boost::icl::interval<index_type>::closed(d.first(), d.last())),
        loc, loc));
}


//////////////////////////////////////////////////////////////////////
/// @brief Extract the partition information from a specification of
/// an arbitrary distribution. The function is used in map_manager to
/// find the set of local domains.
///
/// @param part_cont Container of @ref arb_partition_info elements that
/// specifies an arbitrary distribution
/// @return A vector of the partition information extracted from the local
/// elements of @p part_cont
//////////////////////////////////////////////////////////////////////
template <typename PartitionContainer>
std::vector<std::tuple<
  boost::icl::interval_set<unsigned long int>, unsigned long int,
  location_type>>
compute_local_domains(PartitionContainer const* const part_cont)
{
  typedef unsigned long int index_type;
  typedef std::tuple<boost::icl::interval_set<index_type>,
    unsigned long int, location_type> bc_info_type;

  std::vector<bc_info_type> local_doms =
    cm_impl::get_local_partition_info(
      cm_impl::get_partial_partition_info(part_cont));

  return local_doms;
}

template <bool is_view_based>
struct map_manager_key_to_loc
{
  template <typename Partition, typename Mapper>
  static std::pair<location_type, loc_qual>
  apply(Partition& partition, Mapper& mapper,
        typename Partition::gid_type const& gid,
        typename Partition::gid_type const& block_size)
  {
    auto cid = partition.find(gid);
    return std::make_pair(mapper.valid(cid) ? mapper.map(cid) :
             (gid / block_size) % mapper.get_num_locations(), LQ_CERTAIN);
  }
};

template <>
struct map_manager_key_to_loc<true>
{
  template <typename Partition, typename Mapper>
  static std::pair<location_type, loc_qual>
  apply(Partition& partition, Mapper& mapper,
        typename Partition::gid_type const& gid,
        typename Partition::gid_type const& block_size)
  {
    auto cid = partition.find(gid);
    if (std::get<2>(cid) == LQ_LOOKUP)
      return std::make_pair(std::get<1>(cid), LQ_LOOKUP);
    else if (mapper.valid(std::get<0>(cid)))
      return std::make_pair(mapper.map(std::get<0>(cid)), LQ_CERTAIN);
    else
      return std::make_pair((gid / block_size) % mapper.get_num_locations(),
               LQ_CERTAIN);
  }
};
} // namespace detail

//////////////////////////////////////////////////////////////////////
/// @brief Responsible for determining the home location for elements of
/// the @ref map container, specifically for non-discrete GID types.
///
/// This differs from the discrete case in that it cannot map GIDs in
/// fixed blocks, and thus it simply uses the original partition and
/// mapper information.
///
/// @tparam Partitioner Used to specify how to partition the elements.
/// @tparam Mapper Maps the different partitions to the available locations.
//////////////////////////////////////////////////////////////////////
template<typename Partition,
         typename Mapper,
         bool = boost::icl::is_discrete<
                  typename Partition::gid_type
                >::type::value,
         bool = is_view_based<Partition>::value>
class map_manager
{
public:
  /// GID of the container
  typedef typename Partition::value_type::index_type gid_type;
  /// GID of the container
  typedef gid_type                                   key_type;
  /// Simple intervals of GIDs
  typedef std::pair<gid_type,gid_type>               interval_t;
  /// The result of invoking the function operator
  typedef location_type                              value_type;
  /// Partition ID type
  typedef size_t                                     cid_type;

  typedef Mapper                                     mapper_type;

private:
  /// Partition of the container
  Partition m_partition;

  /// Mapper of the container
  Mapper m_mapper;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Create a manager with a given partition and mapper.
  /// @param ps The partition of the container
  /// @param map The mapper of the container
  //////////////////////////////////////////////////////////////////////
  map_manager(Partition const& ps, Mapper const& map)
    : m_partition(ps), m_mapper(map)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Compute the home / manager location of a given GID.
  /// @param index The GID to map
  /// @return The home location
  //////////////////////////////////////////////////////////////////////
  std::pair<location_type, loc_qual> operator()(gid_type const& index) const
  {
    cid_type cid = m_partition.find(index);
    return std::make_pair(
      m_mapper.valid(cid) ? m_mapper.map(cid) :
                            index_bounds<location_type>::invalid(), LQ_CERTAIN);
  }

  std::tuple<boost::icl::interval_set<key_type>, cid_type>
  range_of(gid_type const& index) const
  {
    stapl_assert(m_partition.size() > 0, "default ctor partition used");
    typedef typename Partition::value_type domain_type;

    const cid_type    cid = m_partition.find(index);
    const domain_type dom = m_partition[cid];

    return std::make_tuple(
             boost::icl::interval_set<key_type>(
               boost::icl::interval<key_type>::closed(dom.first(), dom.last())
             ),
             cid);
  }


  Partition const& partition() const
  {
    return m_partition;
  }

  Mapper& mapper()
  {
    return m_mapper;
  }

  Mapper const& mapper() const
  {
    return m_mapper;
  }
}; // class map_manager ( continuous )


//////////////////////////////////////////////////////////////////////
/// @brief Responsible for determining the home location for elements of
///   the @ref map container, specifically for discrete GID types.
///
/// This specialization operates on discrete key types, and supports
/// a wider range of partition classes.  The non-discrete implementation
/// requires a partition class to provide the inverse mapping operation from
/// a GID to the partition that contains it.
///
/// @tparam Partitioner Used to specify how to partition the elements.
/// @tparam Mapper Maps the different partitions to the available locations.
//////////////////////////////////////////////////////////////////////
template<typename Partition, typename Mapper>
class map_manager<Partition, Mapper, true, false>
{
private:
  typedef typename Partition::value_type                  domain_type;
  typedef typename domain_type::index_type                index_type;
  typedef typename boost::icl::interval<index_type>::type interval_type;

  typedef std::tuple<
    boost::icl::interval_set<index_type>, typename Mapper::cid_type,
    location_type
  > bc_info_type;

public:
  /// GID of the container
  typedef typename Partition::value_type::index_type gid_type;
  /// Simple intervals of GIDs
  typedef std::pair<gid_type,gid_type>               interval_t;
  /// Result type of a multiple mapping
  typedef std::map<interval_t, location_type>        map_result_type;
  /// The result of invoking the function operator
  typedef location_type                              value_type;
  /// Partition ID type
  typedef size_t                                     cid_type;

  typedef Partition                                  partition_type;
  typedef Mapper                                     mapper_type;

private:
  /// Block size of original mapping
  size_t                                              m_block_size;

  /// Partition of the container
  Partition                                           m_partition;

  /// Mapper of the container
  Mapper                                              m_mapper;

  std::vector<bc_info_type> m_local_doms;

public:

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a manager with a given block size, which will be used
  ///   to map GIDs to location in a blocked manner.
  /// @param block_size The number of elements in a block
  //////////////////////////////////////////////////////////////////////
  map_manager(size_t block_size)
    : m_block_size(block_size),
      m_mapper(typename Partition::domain_type(get_num_locations()))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a manager with a given partition and mapper. The block
  /// size will be determined by the size of the first partition.
  /// @param p The partition of the container
  /// @param m The mapper of the container
  //////////////////////////////////////////////////////////////////////
  map_manager(Partition const& ps, Mapper const& m)
    : m_block_size(ps.global_domain().size() != 0 && ps.size() != 0 ?
                   ps.global_domain().size() / ps.size() : 10),
      m_partition(ps), m_mapper(m)
  { m_local_doms = detail::compute_local_domains(m_partition, m_mapper); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a manager for an arbitrary distribution specified by
  /// the elements of @p part_cont and represented by the @p partition and
  /// @p mapper parameters. The block size will be determined by the size
  /// of the first partition.
  ///
  /// This constructor allows more efficient extraction of the partition
  /// information from the container specifying an arbitrary distribution
  /// than the element-wise extraction of the information that is performed
  /// by the @ref detail::compute_local_domains() function when a view-based
  /// distribution that utilizes closed-form mapping functions is provided.
  ///
  /// @param p The partition of the container
  /// @param m The mapper of the container
  //////////////////////////////////////////////////////////////////////
  template <typename PartitionContainer>
  map_manager(PartitionContainer const* const part_cont,
              Partition const& partition, Mapper const& mapper)
    : m_block_size(partition.global_domain().size() != 0 &&
                   partition.size() != 0 ?
                   partition.global_domain().size() / partition.size() : 10),
      m_partition(partition), m_mapper(mapper)
  { m_local_doms = detail::compute_local_domains(part_cont); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Compute the home / manager location of a given GID.
  /// @param index The GID to map
  /// @return The home location
  //////////////////////////////////////////////////////////////////////
  std::pair<location_type, loc_qual> operator()(gid_type const& index) const
  {
    return detail::map_manager_key_to_loc<is_view_based<partition_type>::value>
      ::apply(m_partition, m_mapper, index, m_block_size);
  }

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Compute the home location of a GID.
  /// @param gid GID for which the home location is needed.
  ///
  /// @internal
  /// This function encapsulates the mapping of gid to cid and then the use
  /// of cid to determine the home location.  This is done to avoid confusion
  /// regarding what attribute should be used to define the intervals in the
  /// map returned by the function operator below.
  //////////////////////////////////////////////////////////////////////
  location_type home(gid_type const& gid) const
  {
    std::pair<location_type, loc_qual> loc =
      detail::map_manager_key_to_loc<is_view_based<partition_type>::value>
        ::apply(m_partition, m_mapper, gid, m_block_size);
    stapl_assert(loc.second != LQ_LOOKUP,
      "map_manager::home instructed to forward request.");
    return loc.first;
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Compute the home / manager location of a range of GIDs.
  /// @param interval A collection of GIDS to map, represented as a
  /// pair of GIDs defining the lower and upper boundary of a contiguous
  /// range.
  ///
  /// The range of GID provided in interval may not all be in the same
  /// bContainers, and as such may map into different cids.  The cid
  /// information is used only to compute the home location, and shouldn't
  /// be mistaken as the attribute that determines when a new interval is
  /// needed.  Only a change in the home location signifies the need of a new
  /// interval in the output map.
  ///
  /// @return Return the computed home location as a map from intervals
  /// to locations
  //////////////////////////////////////////////////////////////////////
  map_result_type
  operator()(std::pair<gid_type, gid_type> const& interval) const
  {
    map_result_type result;

    gid_type first       = interval.first;
    gid_type prev        = first;
    gid_type curr        = first + 1;
    const gid_type upper = interval.second;

    location_type lid = home(first);
    for (; curr <= upper; ++curr)
    {
      location_type curr_lid = home(curr);
      if (lid != curr_lid)
      {
        result.insert(std::make_pair(std::make_pair(first, prev), lid));
        first = curr;
        lid = curr_lid;
      }
      prev = curr;
    }
    location_type loc = home(prev);
    result.insert(std::make_pair(std::make_pair(first, prev), loc));
    return result;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief For a given key, returns in which interval it is in.
  //////////////////////////////////////////////////////////////////////
  std::tuple<boost::icl::interval_set<index_type>, cid_type>
  range_of(gid_type const& index)
  {
    if (m_partition.size() > 0)
    {
      cid_type cid = m_partition.find(index);
      typedef typename Partition::value_type domain_type;

      typename std::vector<bc_info_type>::iterator i
        = m_local_doms.begin();
      for (; i != m_local_doms.end(); ++i)
      {
        if (boost::icl::contains(get<0>(*i), index))
          return std::make_tuple(get<0>(*i), get<1>(*i));
      }
      i = m_local_doms.begin();
      for (; i != m_local_doms.end(); ++i)
      {
        if (cid == get<1>(*i))
        {
          // update local range information
          get<0>(*i).add(index);
          return std::make_tuple(get<0>(*i), get<1>(*i));
        }
      }
      abort("No range found containing key to be inserted.");
      return std::make_tuple(get<0>(*i), cid_type());
    }
    abort("Invalid partition passed to map manager.");
    return std::make_tuple(boost::icl::interval_set<index_type>(), cid_type());
  }

  Partition const& partition(void) const
  {
    return m_partition;
  }

  Partition& partition(void)
  {
    return m_partition;
  }

  void partition(Partition const& p)
  {
    m_partition = p;
  }

  Mapper const& mapper(void) const
  {
    return m_mapper;
  }

  Mapper& mapper(void)
  {
    return m_mapper;
  }

  void mapper(Mapper const& m)
  {
    m_mapper = m;
  }
}; // class map_manager<Partition, Mapper, true, false>



//////////////////////////////////////////////////////////////////////
/// @brief Responsible for determining the home location for elements of
///   the @ref map container, specifically for discrete GID types.
///
/// This specialization operates on discrete key types, and supports
/// the view-based partition class.  The non-discrete implementation
/// requires a partition class to provide the inverse mapping operation from
/// a GID to the partition that contains it.
///
/// @tparam Partitioner Used to specify how to partition the elements.
/// @tparam Mapper Maps the different partitions to the available locations.
//////////////////////////////////////////////////////////////////////
template<typename Partition, typename Mapper>
class map_manager<Partition, Mapper, true, true>
{
private:
  typedef typename Partition::value_type                  domain_type;
  typedef typename domain_type::index_type                index_type;
  typedef typename boost::icl::interval<index_type>::type interval_type;

  typedef std::tuple<
    boost::icl::interval_set<index_type>, typename Mapper::cid_type,
    location_type
  > bc_info_type;

public:
  /// GID of the container
  typedef typename Partition::value_type::index_type gid_type;
  /// Simple intervals of GIDs
  typedef std::pair<gid_type,gid_type>               interval_t;
  /// Result type of a multiple mapping
  typedef std::map<interval_t, location_type>        map_result_type;
  /// The result of invoking the function operator
  typedef location_type                              value_type;
  /// Partition ID type
  typedef size_t                                     cid_type;

  typedef Mapper                                     mapper_type;

private:
  /// Block size of original mapping
  size_t                                              m_block_size;

  /// Partition of the container
  Partition                                           m_partition;

  /// Mapper of the container
  Mapper                                              m_mapper;

  std::vector<bc_info_type> m_local_doms;

public:

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a manager with a given block size, which will be used
  ///   to map GIDs to location in a blocked manner.
  /// @param block_size The number of elements in a block
  //////////////////////////////////////////////////////////////////////
  map_manager(size_t block_size)
    : m_block_size(block_size),
      m_mapper(typename Partition::domain_type(get_num_locations()))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a manager with a given partition and mapper. The block
  /// size will be determined by the size of the first partition.
  /// @param p The partition of the container
  /// @param m The mapper of the container
  //////////////////////////////////////////////////////////////////////
  map_manager(Partition const& ps, Mapper const& m)
    : m_block_size(ps.global_domain().size() != 0 && ps.size() != 0 ?
                   ps.global_domain().size() / ps.size() : 10),
      m_partition(ps), m_mapper(m)
  { m_local_doms = detail::compute_local_domains(m_partition, m_mapper); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a manager for an arbitrary distribution specified by
  /// the elements of @p part_cont and represented by the @p partition and
  /// @p mapper parameters. The block size will be determined by the size
  /// of the first partition.
  ///
  /// This constructor allows more efficient extraction of the partition
  /// information from the container specifying an arbitrary distribution
  /// than the element-wise extraction of the information that is performed
  /// by the @ref detail::compute_local_domains() function when a view-based
  /// distribution that utilizes closed-form mapping functions is provided.
  ///
  /// @param p The partition of the container
  /// @param m The mapper of the container
  //////////////////////////////////////////////////////////////////////
  template <typename PartitionContainer>
  map_manager(PartitionContainer const* const part_cont,
              Partition const& partition, Mapper const& mapper)
    : m_block_size(partition.global_domain().size() != 0 &&
                   partition.size() != 0 ?
                   partition.global_domain().size() / partition.size() : 10),
      m_partition(partition), m_mapper(mapper)
  { m_local_doms = detail::compute_local_domains(part_cont); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Compute the home / manager location of a given GID.
  /// @param index The GID to map
  /// @return The home location
  //////////////////////////////////////////////////////////////////////
  std::pair<location_type, loc_qual> operator()(gid_type const& index) const
  {
    return detail::map_manager_key_to_loc<is_view_based<Partition>::value>
      ::apply(m_partition, m_mapper, index, m_block_size);
  }

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Compute the home location of a GID.
  /// @param gid GID for which the home location is needed.
  ///
  /// @internal
  /// This function encapsulates the mapping of gid to cid and then the use
  /// of cid to determine the home location.  This is done to avoid confusion
  /// regarding what attribute should be used to define the intervals in the
  /// map returned by the function operator below.
  //////////////////////////////////////////////////////////////////////
  location_type home(gid_type const& gid) const
  {
    std::pair<location_type, loc_qual> loc =
      detail::map_manager_key_to_loc<is_view_based<Partition>::value>::apply(
        m_partition, m_mapper, gid, m_block_size);
    stapl_assert(loc.second != LQ_LOOKUP,
      "map_manager::home instructed to forward request.");
    return loc.first;
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Compute the home / manager location of a range of GIDs.
  /// @param interval A collection of GIDS to map, represented as a
  /// pair of GIDs defining the lower and upper boundary of a contiguous
  /// range.
  ///
  /// The range of GID provided in interval may not all be in the same
  /// bContainers, and as such may map into different cids.  The cid
  /// information is used only to compute the home location, and shouldn't
  /// be mistaken as the attribute that determines when a new interval is
  /// needed.  Only a change in the home location signifies the need of a new
  /// interval in the output map.
  ///
  /// @return Return the computed home location as a map from intervals
  /// to locations
  //////////////////////////////////////////////////////////////////////
  map_result_type
  operator()(std::pair<gid_type, gid_type> const& interval) const
  {
    map_result_type result;

    gid_type first       = interval.first;
    gid_type prev        = first;
    gid_type curr        = first + 1;
    const gid_type upper = interval.second;

    location_type lid = home(first);
    for (; curr <= upper; ++curr)
    {
      location_type curr_lid = home(curr);
      if (lid != curr_lid)
      {
        result.insert(std::make_pair(std::make_pair(first, prev), lid));
        first = curr;
        lid = curr_lid;
      }
      prev = curr;
    }
    location_type loc = home(prev);
    result.insert(std::make_pair(std::make_pair(first, prev), loc));
    return result;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief For a given key, returns in which interval it is in.
  //////////////////////////////////////////////////////////////////////
  std::tuple<boost::icl::interval_set<index_type>, cid_type>
  range_of(gid_type const& index)
  {
    if (m_partition.size() > 0)
    {
      auto cid_info = m_partition.find(index);
      stapl_assert(std::get<2>(cid_info) != LQ_LOOKUP,
        "map_manager::range_of given invalid partition id");
      typedef typename Partition::value_type domain_type;

      typename std::vector<bc_info_type>::iterator i
        = m_local_doms.begin();
      for (; i != m_local_doms.end(); ++i)
      {
        if (boost::icl::contains(get<0>(*i), index))
          return std::make_tuple(get<0>(*i), get<1>(*i));
      }
      i = m_local_doms.begin();
      for (; i != m_local_doms.end(); ++i)
      {
        if (std::get<0>(cid_info) == get<1>(*i))
        {
          // update local range information
          get<0>(*i).add(index);
          return std::make_tuple(get<0>(*i), get<1>(*i));
        }
      }
      abort("No range found containing key to be inserted.");
      return std::make_tuple(get<0>(*i), cid_type());
    }
    abort("Invalid partition passed to map manager.");
    return std::make_tuple(boost::icl::interval_set<index_type>(), cid_type());
  }

  Partition const& partition(void) const
  {
    return m_partition;
  }

  Partition& partition(void)
  {
    return m_partition;
  }

  void partition(Partition const& p)
  {
    m_partition = p;
  }

  Mapper const& mapper(void) const
  {
    return m_mapper;
  }

  Mapper& mapper(void)
  {
    return m_mapper;
  }

  void mapper(Mapper const& m)
  {
    m_mapper = m;
  }
}; // class map_manager<Partition, Mapper, true, true>

} // namespace stapl

#endif // STAPL_CONTAINERS_MAP_MANAGER
