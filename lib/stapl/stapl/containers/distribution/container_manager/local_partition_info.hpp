/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_LOCAL_PARTITION_INFO_HPP
#define STAPL_CONTAINERS_LOCAL_PARTITION_INFO_HPP

#include <vector>
#include <boost/icl/interval_set.hpp>
#include <stapl/containers/array/static_array.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/domains/partitioned_domain.hpp>
#include <stapl/domains/indexed.hpp>
#include <stapl/utility/tuple.hpp>
#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/executors/execute.hpp>
#include <stapl/skeletons/functional/butterfly.hpp>
#include <stapl/skeletons/functional/reduce_to_pow_two.hpp>
#include <stapl/skeletons/functional/expand_from_pow_two.hpp>
#include <stapl/skeletons/functional/sink.hpp>
#include <stapl/skeletons/spans/balanced.hpp>
#include <stapl/skeletons/spans/nearest_pow_two.hpp>

#include "local_partition_utils.hpp"


namespace stapl {

namespace cm_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Helper struct to merge two tuples representing partial
/// partition information together.
///
/// This struct is needed because the merge_partial_info is used during
/// container construction and redistribution.  When used in redistribution
/// the tuple has an additional field that represents the set of locations
/// that have data that will be stored in the partition.
//////////////////////////////////////////////////////////////////////
template <typename PartitionInfo, int Size = tuple_size<PartitionInfo>::value>
struct merge_partial_info_helper
{
  typedef PartitionInfo result_type;

  static
  result_type apply(PartitionInfo const& lh, PartitionInfo const& rh)
  {
    typedef typename tuple_element<0, PartitionInfo>::type interval_set_type;
    interval_set_type merged_intervals(get<0>(lh));
    typename interval_set_type::iterator interval_it = get<0>(rh).begin();
    typename interval_set_type::iterator interval_end = get<0>(rh).end();
    for (; interval_it != interval_end; ++interval_it)
      merged_intervals += *interval_it;
    return result_type(merged_intervals, get<1>(lh), get<2>(lh));
  }
};

template <typename PartitionInfo>
struct merge_partial_info_helper<PartitionInfo, 4>
{
  typedef PartitionInfo result_type;

  static
  result_type apply(PartitionInfo const& lh, PartitionInfo const& rh)
  {
    typedef typename tuple_element<0, PartitionInfo>::type interval_set_type;
    interval_set_type merged_intervals(get<0>(lh));

    typename interval_set_type::iterator interval_it = get<0>(rh).begin();
    typename interval_set_type::iterator interval_end = get<0>(rh).end();
    for (; interval_it != interval_end; ++interval_it)
      merged_intervals += *interval_it;

    typedef typename tuple_element<3, PartitionInfo>::type location_set_type;
    location_set_type merged_locations;
    merged_locations.reserve(get<3>(lh).size());

    typename location_set_type::const_iterator lh_it = get<3>(lh).begin();
    typename location_set_type::const_iterator lh_end = get<3>(lh).end();
    typename location_set_type::const_iterator rh_it = get<3>(rh).begin();
    typename location_set_type::const_iterator rh_end = get<3>(rh).end();
    for (; lh_it != lh_end && rh_it != rh_end;)
    {
      if (*lh_it <= *rh_it)
      {
        merged_locations.push_back(*lh_it);
        ++lh_it;
      }
      else
      {
        merged_locations.push_back(*rh_it);
        ++rh_it;
      }
    }
    // Only one of the following loops will have any iteration.
    for (; lh_it != lh_end; ++lh_it)
      merged_locations.push_back(*lh_it);
    for (; rh_it != rh_end; ++rh_it)
      merged_locations.push_back(*rh_it);

    auto end = std::unique(merged_locations.begin(), merged_locations.end());
    merged_locations.resize(std::distance(merged_locations.begin(), end));
    return result_type(merged_intervals, get<1>(lh), get<2>(lh),
             merged_locations);
  }
};

template<typename GID>
struct min_max_interval;

template<typename GID, typename CID>
struct merge_partial_info_helper<
         tuple<min_max_interval<GID>, CID, location_type, location_type>, 4>
{
  using PartitionInfo =
    tuple<min_max_interval<GID>, CID, location_type, location_type>;

  using result_type =
    tuple<min_max_interval<GID>, CID, location_type, location_type>;

  static
  result_type apply(result_type const& lh, result_type const& rh)
  {
    typedef typename tuple_element<0, PartitionInfo>::type interval_set_type;
    interval_set_type merged_intervals(get<0>(lh));

    merged_intervals += get<0>(rh);

    location_type merged_count = get<3>(lh) + get<3>(rh);

    return result_type(merged_intervals, get<1>(lh), get<2>(lh),
             merged_count);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Merge two vectors of partial partition information.
/// @param lh a vector of tuples representing partial partition information
/// @param rh a vector of tuples representing partial partition information
/// @return a vector of tuples that represent the merged partition information
///
/// The method performs a merge of the two input vectors, merging domains
/// if the partition id is equal.  It's used as the first step of the
/// butterfly skeleton when the number of locations is not a power of two.
//////////////////////////////////////////////////////////////////////
template <typename PartitionInfoVector>
PartitionInfoVector
merge_partial_info(PartitionInfoVector const& lh,
                   PartitionInfoVector const& rh)
{
  typedef PartitionInfoVector vector_type;
  typedef typename vector_type::value_type tuple_type;
  typedef typename tuple_element<0, tuple_type>::type interval_set_type;

  // comparator used to maintain sorting based on partition id.
  loc_and_part_id_less<tuple_type> id_less;

  vector_type result;
  typedef typename vector_type::const_iterator ref_it;
  ref_it lh_it = lh.begin();
  ref_it rh_it = rh.begin();
  while (lh_it != lh.end() && rh_it != rh.end())
  {
    // if the location ids are not equal, push the smaller of the two.
    if (get<2>(*lh_it) != get<2>(*rh_it))
    {
      if (id_less(*lh_it, *rh_it))
      {
        result.push_back(*lh_it);
        ++lh_it;
      }
      else
      {
        result.push_back(*rh_it);
        ++rh_it;
      }
    }
    // location ids are equal. if partition ids are equal, merge the domains.
    else if (get<1>(*lh_it) == get<1>(*rh_it))
    {
      result.push_back(
        merge_partial_info_helper<tuple_type>::apply(*lh_it, *rh_it));
      // An element from both vectors has been merged.
      // Advance both iterators.
      ++lh_it;
      ++rh_it;
    }
    else if (id_less(*lh_it, *rh_it))
    {
      // maintain sorting order based on partition id.
      // The code in the first if block maintains the ascending order of
      // locations.
      result.push_back(*lh_it);
      ++lh_it;
    }
    else
    {
      result.push_back(*rh_it);
      ++rh_it;
    }
  }

  // Determine which of the vectors has been processed and append the remaining
  // values of the other to the result.
  if (lh_it != lh.end())
  {
    for (; lh_it != lh.end(); ++lh_it)
      result.push_back(*lh_it);
  }
  else if (rh_it != rh.end())
  {
    for (; rh_it != rh.end(); ++rh_it)
      result.push_back(*rh_it);
  }

  return result;
}


//////////////////////////////////////////////////////////////////////
/// @brief Work function used to merge the partial partition information
/// from two locations when the number of locations is not a power of two.
//////////////////////////////////////////////////////////////////////
template <typename PartitionInfo>
struct combine_partition_info
{
  typedef std::vector<PartitionInfo> result_type;

  //////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////
  template <typename PartitionInfoVectorRef>
  result_type operator()(PartitionInfoVectorRef const& lh,
                         PartitionInfoVectorRef const& rh)
  {
    result_type lh_copy(lh), rh_copy(rh);
    return merge_partial_info(lh_copy, rh_copy);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function used to split the final partition information
/// for two locations when the number of locations is not a power of two.
//////////////////////////////////////////////////////////////////////
template <typename PartitionInfo>
struct split_partition_info
{
  typedef std::vector<PartitionInfo> result_type;

private:
  bool m_rhs;

public:
  split_partition_info(void)
    : m_rhs(false)
  {}

  void set_position(std::size_t index, bool)
  {
    m_rhs = (index % 2 != 0);
  }

  //////////////////////////////////////////////////////////////////////
  /// @todo Remove copy of el and replace with iteration using member_iterator
  /// when gForge Bug #778 is resolved.
  //////////////////////////////////////////////////////////////////////
  template <typename PartitionInfoVectorRef>
  result_type operator()(PartitionInfoVectorRef const& el)
  {
    result_type el_copy(el);
    typename result_type::iterator split =
      std::adjacent_find(el_copy.begin(), el_copy.end(), part_loc_neq());
    if (split != el_copy.end())
    {
      // advance split to the first element of the second range.
      ++split;
      if (m_rhs)
        return result_type(split, el_copy.end());
      else
        return result_type(el_copy.begin(), split);
    }
    else
    {
      // all elements belong to the location on the left.
      if (m_rhs)
        return result_type();
      else
        return el_copy;
    }
  }

  void define_type(typer& t)
  { t.member(m_rhs); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Comparator used to find partition information elements for
/// a specific location id.
//////////////////////////////////////////////////////////////////////
struct loc_equal
{
private:
  location_type m_loc;

public:
  typedef bool result_type;

  loc_equal(location_type loc)
    : m_loc(loc)
  {}

  template <typename PartitionInfoRef>
  bool operator()(PartitionInfoRef const& val)
  { return get<2>(val) == m_loc; }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function passed to butterfly to construct a PARAGRAPH
/// that takes the partial partition information from each location,
/// merges it with the others, and in the end places the partition
/// information for a given location on that location.
//////////////////////////////////////////////////////////////////////
template <typename PartitionInfo>
struct merge_partition_info
{
private:
  /// Size of the butterfly at the current level of the graph
  unsigned int m_butterfly_size;
  /// Index of the first input to the node
  unsigned int m_index1;
  /// Index of the second input to the node.
  unsigned int m_index2;
  /// Number of pairs of nodes that contain information for two locations
  unsigned int m_nb_pairs;
  /// Number of locations
  unsigned int m_nb_locs;

public:
  typedef std::vector<PartitionInfo> result_type;

  merge_partition_info(unsigned int locs)
    : m_butterfly_size(0), m_index1(0), m_index2(0), m_nb_locs(locs)
  { m_nb_pairs = locs - pow(2, floor(log(locs) / log(2))); }

  void set_position(unsigned int butterfly_size, unsigned int index1,
                    unsigned int index2, std::size_t /* ignored */)
  {
    m_butterfly_size = butterfly_size;
    m_index1 = index1;
    m_index2 = index2;
  }

  template <typename PartitionInfoVectorRef>
  result_type operator()(PartitionInfoVectorRef const& el1,
                         PartitionInfoVectorRef const& el2)
  {
    // # of pairs = limit offset between pairs and identities
    std::size_t L = m_nb_pairs;
    // number of btf at the current level of the graph
    std::size_t nbBtfNodes = m_butterfly_size * 2;
    // btf number
    std::size_t currentBtfNb = std::floor(m_index1 / nbBtfNodes);
    // btf median's offset
    std::size_t M = m_butterfly_size + currentBtfNb * nbBtfNodes;
    // left bound of the current btf
    std::size_t leftBound = M - nbBtfNodes / 2;
    // number of elements (nodes) on the left side of the current btf
    std::size_t N1 = ( L <= M )
                      ? ( ( leftBound < L )
                        ? 2 * ( L - leftBound ) + M - L
                        : M - leftBound )
                      : 2 * ( M - leftBound );
    // ... and on the right side
    std::size_t N2 = ( L <= M )
                      ? M - leftBound
                      : ( L >= M + ( M - leftBound ) )
                        ? 2 * ( M - leftBound )
                        : 2 * ( L - M ) + ( M + ( M - leftBound ) - L );
    bool first_level = m_butterfly_size*2 + m_nb_pairs == m_nb_locs ?
                       true : false;
    bool on_lhs = m_index1 < m_index2;
    location_type nlocs_reachable = on_lhs ? N1 : N2;
    location_type first_loc, last_loc;
    if (first_level)
    {
      if (on_lhs)
        first_loc = 0;
      else
        first_loc = N1;
    }
    else
    {
      // the number of previous locations is the number of previous nodes
      // plus the number of pairs related to the nodes.
      std::size_t loc_offset = currentBtfNb * nbBtfNodes;
      if (m_nb_pairs)
      {
        if (m_nb_pairs >= loc_offset)
          loc_offset += currentBtfNb * nbBtfNodes;
        else
          loc_offset += m_nb_pairs;
      }
      if (on_lhs)
        first_loc = loc_offset;
      else
        first_loc = loc_offset + N1;
    }
    last_loc = first_loc + nlocs_reachable - 1;

    result_type el1_copy(el1), el2_copy(el2);
    result_type complete_result = merge_partial_info(el1_copy, el2_copy);

    // Find the set of values this task should return
    typename result_type::iterator first_elem = complete_result.end();
    typename result_type::iterator last_elem = complete_result.end();
    typename result_type::iterator i_end = complete_result.end();
    for (typename result_type::iterator i = complete_result.begin(); i != i_end;
         ++i)
    {
      // Find the first element destined for locations in our assigned range.
      if (first_elem == complete_result.end() &&
          get<2>(*i) >= first_loc && get<2>(*i) <= last_loc)
        first_elem = i;
      // The first element must be identified before the last element.
      if (first_elem != complete_result.end() &&
          get<2>(*i) >= first_loc && get<2>(*i) <= last_loc)
        last_elem = i;
    }
    // If there are no elements to return, then  default construct the vector.
    if (first_elem == complete_result.end())
      return result_type();

    // Increment last_elem to create the half-open range [first_elem, last_elem)
    // required to construct the result for this task.
    ++last_elem;
    return result_type(first_elem, last_elem);
  }

  void define_type(typer& t)
  {
    t.member(m_butterfly_size);
    t.member(m_index1);
    t.member(m_index2);
    t.member(m_nb_pairs);
    t.member(m_nb_locs);
  }
};

template <typename Partition>
std::tuple<typename Partition::index_type, location_type, loc_qual>
get_cid_info(Partition const& partition,
             typename Partition::value_type::index_type const& index,
             boost::mpl::false_)
{
  return std::make_tuple(partition.find(index), 0, LQ_DONTCARE);
}

template <typename Partition>
std::tuple<typename Partition::index_type, location_type, loc_qual>
get_cid_info(Partition const& partition,
             typename Partition::value_type::index_type const& index,
             boost::mpl::true_)
{
  return partition.find(index);
}

//////////////////////////////////////////////////////////////////////
/// @brief Compute the partial partition information of a subset of the
/// GIDs of a container domain.
///
/// @param partition Partition of the container being constructed.  Contains
/// the domain of the container and its find method maps a GID to a
/// partition id.
/// @param mapper Mapper of the container being constructed.  Its map method
/// maps a partition id to a location id.
/// @return vector of the partial information for the base containers that
/// will be constructed.
///
/// This function is the first step of a two-step process, and is called
/// during base container manager initialization. The computation of partial
/// information is separated from the butterfly computation to find the full
/// partition information to allow the multi-dimensional container manager to
/// specialize the computation of partial partition information without
/// replicating the code containing the butterfly skeleton.
///
/// @todo Remove the construction of @ref partitioned_domain when the domain
/// provided by the partition is distributed.
//////////////////////////////////////////////////////////////////////
template <typename Partition, typename Mapper>
std::vector<tuple<
  boost::icl::interval_set<typename Partition::value_type::index_type>,
  typename Mapper::cid_type, location_type> >
get_partial_partition_info(Partition const& partition, Mapper const& mapper)
{
  typedef typename Partition::value_type                    domain_type;
  typedef typename domain_type::index_type                  index_type;
  typedef boost::icl::interval_set<index_type>              interval_set_type;
  typedef typename Mapper::cid_type                         cid_type;
  typedef tuple<interval_set_type, cid_type, location_type> value_type;
  typedef std::vector<value_type>                           result_type;

  // Partition domain
  typedef partitioned_domain<domain_type> part_dom_type;
  part_dom_type part_dom(partition.global_domain());
  typename part_dom_type::subdomains_view_type subdomains =
    part_dom.local_subdomains();

  result_type subdomain_value;
  // Construct partial domains and store as tuple<partition, id, location>
  typename part_dom_type::size_type nsubdomains = subdomains.size();
  for (size_t i = 0; i != nsubdomains; ++i)
  {
    typename part_dom_type::subdomains_view_type::reference
      subdomain(subdomains[i]);
    if (!subdomain.empty())
    {
      index_type first = subdomain.first();
      index_type prev = subdomain.first();
      std::tuple<cid_type, location_type, loc_qual> first_cid_info =
        get_cid_info(partition, first, is_view_based<Partition>());
      cid_type first_cid = std::get<0>(first_cid_info);
      stapl_assert(first_cid != index_bounds<cid_type>::invalid(),
        "get_partial_partition_info(partition, mapper) given invalid cid");
      location_type first_loc = mapper.map(first_cid);
      index_type curr  = subdomain.first();
      for (; curr != subdomain.last(); curr  = subdomain.advance(curr, 1))
      {
        std::tuple<cid_type, location_type, loc_qual> curr_cid_info =
          get_cid_info(partition, curr, is_view_based<Partition>());
        cid_type curr_cid = std::get<0>(curr_cid_info);
        stapl_assert(curr_cid != index_bounds<cid_type>::invalid(),
          "get_partial_partition_info(partition, mapper) given invalid cid");

        if (first_cid != curr_cid)
        {
          stapl_assert(first <= prev, "Creating invalid domain");
          subdomain_value.push_back(
            value_type(
              interval_set_type(
                boost::icl::interval<index_type>::closed(first, prev)),
              first_cid, first_loc));
          first = curr;
          first_cid = curr_cid;
          first_loc = mapper.map(first_cid);
        }
        prev = curr;
      }
      // curr is the last element in the subdomain at this point.
      std::tuple<cid_type, location_type, loc_qual> curr_cid_info =
        get_cid_info(partition, curr, is_view_based<Partition>());
      cid_type curr_cid = std::get<0>(curr_cid_info);
      stapl_assert(curr_cid != index_bounds<cid_type>::invalid(),
        "get_partial_partition_info(partition, mapper) given invalid cid");
      if (first_cid == curr_cid)
      {
        stapl_assert(first <= curr, "Creating invalid domain");
        subdomain_value.push_back(
          value_type(
            interval_set_type(
              boost::icl::interval<index_type>::closed(first, curr)),
            first_cid, first_loc));
      }
      else
      {
        stapl_assert(first <= prev, "Creating invalid domain");
        subdomain_value.push_back(
          value_type(
            interval_set_type(
              boost::icl::interval<index_type>::closed(first, prev)),
            first_cid, first_loc));
        first_loc = mapper.map(curr_cid);
        subdomain_value.push_back(
          value_type(
            interval_set_type(
              boost::icl::interval<index_type>::closed(curr, curr)),
            curr_cid, first_loc));
      }
    }
  }

  return subdomain_value;
}

//////////////////////////////////////////////////////////////////////
/// @brief Extract the partial partition information of a subset of the GIDs
/// of a container domain from the explicit specification of an
/// arbitrary distribution.
///
/// @param part_cont Container of @ref arb_partition_info elements that
/// specifies an arbitrary distribution
/// @return vector of the partial information for the base containers that
/// will be constructed.
///
/// This function is the first step of a two-step process, and is called
/// during base container manager initialization. The computation of partial
/// information is separated from the butterfly computation to find the full
/// partition information to allow the multi-dimensional container manager to
/// specialize the computation of partial partition information without
/// replicating the code containing the butterfly skeleton.
//////////////////////////////////////////////////////////////////////
template <typename PartitionContainer>
std::vector<tuple<
  boost::icl::interval_set<unsigned long int>, unsigned long int, location_type
>>
get_partial_partition_info(PartitionContainer const* const part_cont)
{
  typedef unsigned long int index_type;
  typedef boost::icl::interval_set<index_type>            interval_set_type;
  typedef typename boost::icl::interval<index_type>::type interval_type;

  typedef tuple<
    boost::icl::interval_set<index_type>, index_type, location_type
  >  bc_info_type;

  // Create storage for the partial information stored in part_cont
  std::vector<bc_info_type> partial_bc_info;

  // Used to traverse the local base containers of part_cont
  auto lcl_bc_it =
    part_cont->container().distribution().container_manager().begin();
  auto lcl_bc_end =
    part_cont->container().distribution().container_manager().end();

  if (lcl_bc_it != lcl_bc_end)
    partial_bc_info.reserve(lcl_bc_it->size());

  // Extract partition information from part_cont
  // Iterate over the local base containers of part_cont
  for (; lcl_bc_it != lcl_bc_end; ++lcl_bc_it)
  {
    auto part_dom = lcl_bc_it->domain();
    index_type pid = part_dom.first();

    // Iterate over the elements of the current part_cont base container.
    // Each element represents a partition.
    auto part_it = lcl_bc_it->begin();
    auto part_end = lcl_bc_it->end();
    for (; part_it != part_end; ++part_it, pid = part_dom.advance(pid, 1))
    {
      // Construct the information for this partition
      auto gid_dom = part_it->domain();
      partial_bc_info.push_back(bc_info_type(
        interval_set_type(boost::icl::interval<index_type>::closed(
          gid_dom.first, gid_dom.second)), pid, part_it->location()));
    }
  }

  return partial_bc_info;
}

//////////////////////////////////////////////////////////////////////
/// @brief Compute the partitions of a domain that will be stored on
/// each location.
///
/// @tparam PartInfo vector of tuple<interval_set, cid, location>
/// @param  partial_info vector of partial component information for some
/// unknown subset of the base containers to be constructed.
///
/// @return vector of the component information necessary to construct the
/// base containers needed on this location.
///
/// This function is called during base container manager initialization
/// to determine which base containers to construct on each location.
///
/// @todo Remove the construction of @ref partitioned_domain when the domain
/// provided by the partition is distributed.
//////////////////////////////////////////////////////////////////////
template <typename PartInfo>
PartInfo
get_local_partition_info(PartInfo const& partial_info)
{
  typedef typename PartInfo::value_type value_type;
  typedef PartInfo                      result_type;

  const size_t n_locs = get_num_locations();

  // Avoid the construction of a temporary container and call to butterfly if
  // the information we need is already available.
  if (n_locs == 1)
    return partial_info;

  // Information on the elements in the local subdomain
  typedef static_array<result_type>               array_type;
  typedef array_view<array_type>                  view_type;

  array_type subdomain_values(n_locs, result_type());
  view_type  subdomain_view(subdomain_values);

  array_type final_subdomain_values(n_locs, result_type());
  view_type  final_subdomain_view(final_subdomain_values);

  const location_type id = get_location_id();

  // sort the partial domains in order to simplify merging
  subdomain_values[id] = partial_info;

  // Replace with sort of subdomain[id].begin()/end() when gForge Bug 778 fixed.

  typedef typename array_type::distribution_type               dist_type;
  typedef typename dist_type::container_manager_type::iterator iter_type;

  iter_type bc_it = subdomain_values.distribution().container_manager().begin();

  std::sort(bc_it->begin()->begin(),
            bc_it->begin()->end(),
            loc_and_part_id_less<value_type>());

  // Pass partial domains through butterfly to get final partition information
  // for this location and return it.
  // reduce + butterfly + broadcast used to handle the case where the number of
  // locations is not a power of two
  typedef skeletons::spans::nearest_pow_two<
            skeletons::spans::balanced<>> bfly_span_t;
  skeletons::execute(
    skeletons::default_execution_params(),
    skeletons::sink<result_type>(
      skeletons::compose(
        skeletons::reduce_to_pow_two(combine_partition_info<value_type>()),
        skeletons::butterfly<true, use_default, bfly_span_t>
          (merge_partition_info<value_type>(subdomain_view.size())),
        skeletons::expand_from_pow_two<use_default,true>
          (split_partition_info<value_type>())
      )
    ),
    subdomain_view,
    final_subdomain_view);

  // Ensure all locations have passed this point before receiving messages from
  // other locations that are inserting into the base container ordering.
  subdomain_values.advance_epoch();
  return final_subdomain_values[id];
}

} // namespace cm_impl

} // namespace stapl

#endif
