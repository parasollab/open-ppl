/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_METADATA_COARSEN_ALIGN_GUIDED_OFFSET_HPP
#define STAPL_VIEWS_METADATA_COARSEN_ALIGN_GUIDED_OFFSET_HPP

#include <stapl/runtime.hpp>
#include <numeric>
#include <functional>

#include <stapl/algorithms/functional.hpp>

#include <stapl/utility/tuple.hpp>
#include <stapl/views/metadata/coarsen_utility.hpp>
#include <stapl/views/metadata/transform.hpp>
#include <stapl/views/metadata/utility/convert_to_md_vec_array.hpp>

#include <stapl/paragraph/paragraph_fwd.h>
#include <stapl/utility/vs_map.hpp>
#include <stapl/utility/do_once.hpp>

namespace stapl {

// Forward declarations of types used to disable static metadata optimization
// in guided_offset_alignment::apply below.
template<typename Ptr, typename BC>
struct list_gid;

template <typename Functor, int n, typename... Distribution>
struct functor_container;


namespace metadata {

//////////////////////////////////////////////////////////////////////
/// @brief Helper functor to determine the local storage type for the
///        metadata information.
//////////////////////////////////////////////////////////////////////
template <typename Part>
struct get_copy_md_entry_type
{
  using type = std::map<size_t, typename Part::second_type::value_type>;
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper functor to compute the minimum size and offset from
///        the given pairs of size and offset.
//////////////////////////////////////////////////////////////////////
struct min_size_offset
{
  using result_type = std::pair<size_t, size_t>;

  result_type operator()(result_type const& a, result_type const& b) const
  {
    return std::make_pair(std::min(a.first,b.first),
                          std::min(a.second,b.second));
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper functor to determine if the given pair <size, offset>
///        @p p, has the same specified offset.
//////////////////////////////////////////////////////////////////////
class equal_second
{
private:
  size_t m_offset;

public:
  equal_second(size_t offset)
    : m_offset(offset)
  { }

  bool operator()(std::pair<size_t,size_t> const& p) const
  {
    return (m_offset==p.second || p.first==index_bounds<size_t>::highest());
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Distributed object used to push metadata to neighboring locations
/// in cases where static metadata is used. This eliminates blocking
/// reads of the partition data in guided_offset_alignment::apply.
//////////////////////////////////////////////////////////////////////
struct push_static_metadata
  : public p_object
{
private:
  std::map<std::size_t, void*> m_prev_md;
  std::map<std::size_t, bool>  m_set;
  bool                         m_static_metadata;

public:
  push_static_metadata(bool static_metadata)
    : p_object(allow_try_rmi | no_aggregation),
      m_static_metadata(static_metadata)
  { }

  template<typename LocalMD>
  void register_md(std::size_t id, LocalMD* local_md)
  { m_prev_md.insert(std::make_pair(id, (void*)local_md)); }

  template <typename MD>
  void send(std::size_t id, size_t gidx, MD&& md)
  {
    if (m_static_metadata && this->get_location_id() != 0)
    {
      try_rmi(this->get_location_id()-1, this->get_rmi_handle(),
        &push_static_metadata::template receive<MD>, id, gidx, std::move(md));
    }
  }

  template <typename MD>
  void receive(std::size_t id, size_t gidx, MD md)
  {
    auto local_md_ptr = m_prev_md.find(id);
    if (local_md_ptr != m_prev_md.end())
    {
      (*reinterpret_cast<std::map<size_t, MD>*>(local_md_ptr->second))[gidx] =
        std::move(md);
    }
    m_set[id] = true;
  }

  void yield(long long i, std::size_t id)
  {
    // If this location is accessing the second metadata entry, static metadata
    // is available, and there is a neighboring location, wait for the metadata
    // to arrive
    if (m_static_metadata && i == 1 &&
        this->get_location_id() != this->get_num_locations()-1)
      runtime::yield_until(
        [this, id]{ return this->m_set.find(id) != this->m_set.end(); });
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper functor to compute the size and offset from the
///        metadata at position @p i.
//////////////////////////////////////////////////////////////////////
struct size_offset
{
private:
  push_static_metadata* m_push_static_metadata;

public:
  using result_type = std::pair<size_t, size_t>;

  size_offset(push_static_metadata& b)
    : m_push_static_metadata(&b)
  { }

  template <typename View, typename Part, typename LocalMD>
  std::pair<size_t,size_t>
  operator()(View const& view, Part const& part, LocalMD& local_md,
             long long i) const
  {
    using partition_t    = typename LocalMD::mapped_type;
    using local_domain_t = typename partition_t::domain_type;

    m_push_static_metadata->register_md(
       view.get_rmi_handle().internal_handle().abstract(), &local_md);

    // get the global partition id of the entry at i
    size_t gidx = part.second.get_local_vid(0) + i;

    // push the metadata for the first entry on the location to the neighbor
    // location to avoid a synchronous read from part.second below.
    if (i == 0 && gidx != std::numeric_limits<size_t>::max())
    {
      m_push_static_metadata->send(
        view.get_rmi_handle().internal_handle().abstract(), gidx,
        part.second[gidx]);
    }

    // if there aren't any local partitions, set the current
    // partition to be the last global partition
    if (part.second.local_size()==0 ||
        part.second.get_local_vid(0)==index_bounds<size_t>::invalid())
    {
      gidx = part.second.size()-1 + i;
    }

    // Receive neighboring information
    m_push_static_metadata->yield(i,
      view.get_rmi_handle().internal_handle().abstract());

    // if the partition is not in our local cache, grab it
    // from the global partition
    if (local_md.find(gidx) == local_md.end())
    {
      local_md[gidx] = part.second[gidx];
    }

    // get the domain for the partition
    local_domain_t ldom = local_md[gidx].domain();

    // domain of the global view
    typename View::domain_type gdom = view.domain();

    // return the size of the partition and how far the start
    // of that partition's domain is from the global domain
    // of the view
    size_t sz = ldom.size();

    // figure out where the next gid would be in the local domain
    size_t off = gdom.distance(gdom.first(), ldom.first());

    return std::make_pair(sz, off);
  }
};


namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Helper functor to initialize the temporary local information
///   required for the alignment.
//////////////////////////////////////////////////////////////////////
struct initialize_guided_offset_alignment
{
private:
  size_t m_size;
  size_t m_idx;
  bool   m_use_fixed;

public:
  using result_type = void;

  //////////////////////////////////////////////////////////////////////
  /// @brief Captures state needed for intialization of metadata containers
  ///   for each view being aligned
  /// @param use_fixed indicates if fixed-size metadata container can be used
  /// @param size size of the metadata container to be allocated if fixed-sized
  ///   container is being used
  /// @param idx first index in the metadata container to be initialized by this
  ///   location
  /// @return What is the type, name, and meaning of the return value?
  //////////////////////////////////////////////////////////////////////
  initialize_guided_offset_alignment(bool use_fixed, size_t size, size_t idx)
    : m_size(size), m_idx(idx), m_use_fixed(use_fixed)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @param x Position of the first local metadata entry.
  /// @param p The resulting metadata alignment.
  /// @param pos Position in the aligned metadata where this location will
  ///   start writing.
  /// @param part Initial locality metadata partition.
  /// @param lmd Temporary local storage for metadata information.
  /// @param lsz Number of local metadata entries to align.
  //////////////////////////////////////////////////////////////////////
  template <typename AlignMD, typename AlignPos, typename Part,
            typename LocalMD>
  void operator()(long long& x, AlignMD& p, AlignPos& pos, Part const& part,
                  LocalMD& lmd, size_t& lsz) const
  {
    using part_t = typename AlignMD::view_container_type;
    using md_t   = typename Part::second_type::value_type;

    x = 0;

    // If the metadata of the views being aligned is static and the overlap in
    // domains is such that we can predict the number of elements in the aligned
    // metadata container (i.e., the delta between the first index in each view
    // is less than n/p) allocate a fixed size container.  The update function
    // in this case will not perform the scan and reduce that dynamic metadata
    // containers require.
    if (m_use_fixed)
    {
      p   = AlignMD(new part_t(m_size));
      pos = m_idx;
    }
    else
    { p = AlignMD(new part_t()); }

    // When the size is the max size_t it indicates a repeated view,
    // and there's only one entry in the container.
    if (part.second.size() != index_bounds<size_t>::highest())
    {
      size_t tmpsz = 0;
      for (size_t i=0; i < part.second.local_size(); ++i)
      {
        const size_t gpos = part.second.get_local_vid(0) + i;

        md_t md_entry = part.second[gpos];

        if (!md_entry.domain().empty())
        {
          lmd[gpos] = md_entry;
          ++tmpsz;
        }
      }
      lsz = tmpsz;
    }
    else
    {
      lmd[1] = part.second[0];
      lsz    = 1;
    }
  }
}; // struct initialize_guided_offset_alignment


//////////////////////////////////////////////////////////////////////
/// @brief Helper functor to align metadata entries that have the
///        same offset (are left aligned).
//////////////////////////////////////////////////////////////////////
class proc_equal_offset
{
private:
  const size_t m_mblk;
  const size_t m_fixed_size;

public:
  using result_type = void;

  proc_equal_offset(size_t mblk, bool fixed_size)
    : m_mblk(mblk), m_fixed_size(fixed_size)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Aligns metadata that have the same offset.
  /// @param sz_off Information about size and offset of the
  ///               metadata to be aligned.
  /// @param part Partitioned metadata locality.
  /// @param i Index of the metadata that is going to be aligned.
  /// @param local_md Temporary local storage for metadata entries.
  /// @param alignpart Container for metadata locality that contains the
  ///                  resulting coarsened partitioned metadata locality.
  /// @param idx Index in the aligned metadata to write.
  /// @param k Returns the increment that needs to be applied to
  ///          advance to the next metadata locality.
  //////////////////////////////////////////////////////////////////////
  template <typename SzOff, typename Part,
            typename LocalMD, typename AlignPart>
  void operator()(SzOff sz_off, Part const& part, long long i,
                  LocalMD& local_md, AlignPart& alignpart, size_t& idx,
                  long long& k) const
  {
    using md_entry_t  = typename LocalMD::mapped_type;
    using md_domain_t = typename md_entry_t::domain_type;

    // get the i'th local partition id
    size_t gpos = part.second.get_local_vid(0) + i;

    // if there aren't any local partitions, use the last
    // global partition + i
    if (part.second.local_size() == 0
        || part.second.get_local_vid(0)==index_bounds<size_t>::invalid())
    {
      gpos = part.second.size()-1 + i;
    }

    stapl_assert(local_md.find(gpos) != local_md.end(),
                 "Failed to find local_md entry");

    // get the metadata for that partition
    md_entry_t md_entry = local_md[gpos];
    md_domain_t ldom    = md_entry.domain();

    // if that partition's domain is empty, then
    // return that we should skip this one
    if (ldom.size() == 0)
    {
      k = 1;
      return;
    }

    // if the size of the domain is equal to the size of the
    // domain that we are trying to create, then return that
    // we should skip to the next base container
    k = sz_off.first == m_mblk ? 1 : 0;

    // current offset in the partition (the GID that we should start
    // with for the new domain)

    // create a new domain for the realigned view
    md_domain_t ndom(ldom.first(), ldom.advance(ldom.first(), m_mblk-1), ldom);

    // create a new metadata with this entry and add it to the aligned partition
    md_entry_t md(typename md_entry_t::cid_type(), ndom, md_entry.component(),
                  md_entry.location_qualifier(), md_entry.affinity(),
                  md_entry.handle(), md_entry.location());

    if (m_fixed_size)
    {
      alignpart.set_element(idx, md);
      ++idx;
    }
    else
      alignpart.push_back_here(md);

    // if we are not skipping this partition, we need to update
    // that when we next use it, we have to skip the GIDs that we just added
    // to the new realigned view
    if (k != 1)
    {
      md_domain_t udom(ldom.advance(ldom.first(), m_mblk), ldom.last(), ldom);
      md_entry_t upd_md(typename md_entry_t::cid_type(),
                  udom, md_entry.component(),
                  md_entry.location_qualifier(), md_entry.affinity(),
                  md_entry.handle(), md_entry.location());
      local_md[gpos] = upd_md;
    }
  }
}; // class proc_equal_offset


//////////////////////////////////////////////////////////////////////
/// @brief Helper functor to check if this location has all the
///        information which is required for doing local computation
///        of coarsening.
//////////////////////////////////////////////////////////////////////
class proc_unequal_offset
{
private:
  /// The distance from the beginning of the global view's domain
  /// to the first local element of the guiding view
  const size_t m_global_guide_offset;

public:
  using result_type = void;

  proc_unequal_offset(size_t moff)
    : m_global_guide_offset(moff)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Helper function to verify and obtain all the metadata
  ///        information required for aligning the metadata at given
  ///        position.
  ///
  /// @param view View that is aligned.
  /// @param part Partitioned metadata locality.
  /// @param i Index of the metadata that is going to be computed.
  /// @param local_md Container for metadata locality that contains the
  ///                 resulting coarsened partitioned metadata locality.
  /// @param k Returns the increment that needs to be applied to
  ///          advance to the next metadata locality.
  //////////////////////////////////////////////////////////////////////
  template <typename View, typename Part, typename LocalMD>
  void operator()(View const& view, Part const& part, long long i,
                  LocalMD& local_md, long long& k) const
  {
    using md_entry_t  = typename LocalMD::mapped_type;
    using md_domain_t = typename md_entry_t::domain_type;

    // get the id of the i'th local partition
    size_t pidx = part.second.get_local_vid(0) + i;

    // if there aren't any local partitions then set the
    // current partition id to the last partition + i
    if (part.second.local_size()==0 ||
        part.second.get_local_vid(0)==index_bounds<size_t>::invalid())
    {
      pidx = part.second.size()-1 + i;
    }

    stapl_assert(local_md.find(pidx) != local_md.end(),
                 "Failed to find local_md entry");

    // get the domain of the partition
    md_domain_t ldom = local_md[pidx].domain();
    const size_t sz = ldom.size();

    // get the global domain of original view
    typename View::domain_type gdom = view.domain();

    // the index in the global view that has the first element
    // of the guiding view
    typename md_domain_t::index_type guiding_first =
      gdom.advance(gdom.first(), m_global_guide_offset);

    // if the first element of the local domain is the same as the
    // first element of the guiding view's local domain then we
    // don't need to skip any local partitions
    if (guiding_first == ldom.first() || sz == index_bounds<size_t>::highest())
    {
      k = 0;

      return;
    }

    // the current running offset
    long long j = 0;

    // if the domain of the partition is empty, set the local
    // partition of the previous partition to be the last partition
    if (sz == 0)
    {
      --j;
      local_md[pidx+j] = part.second[part.second.size()-1];
    }
    else
    {
      // try to find the local domain that contains
      // the first local element of the guiding view
      if (!ldom.contains(guiding_first))
      {
        // distance between the beginning of global domain and the
        // local domain
        const size_t first_off = gdom.distance(gdom.first(), ldom.first());

        do {
          // the guiding view's first domain is before this view's,
          // so we should look at the next partition
          if (m_global_guide_offset < first_off)
            --j;
          else
            ++j;

          // if the next or prev partition is not on this location,
          // then get it from the partition
          if (local_md.find(pidx+j) == local_md.end()) {
            local_md[pidx+j] = part.second[pidx+j];
          }
        } while (!local_md[pidx+j].domain().contains(guiding_first));
      }
    }

    // we should now have the metadata entry for the local partition that
    // has the first element of the guiding view's local dom
    md_entry_t const& md_entry = local_md[pidx+j];

    // the distance between the found domain's first
    // and the guiding view's first
    size_t tmp_off = gdom.distance(md_entry.domain().first(), guiding_first);

    // if the found partition doesn't start with the same element as the
    // guiding view's start, add a new metadata entry that has the
    // correct subdomain
    if (tmp_off > 0)
    {
      md_domain_t udom(md_entry.domain().advance(
                         md_entry.domain().first(), tmp_off),
                       md_entry.domain().last(),
                       md_entry.domain());

      local_md[pidx+j] = md_entry_t(
        typename md_entry_t::cid_type(), udom, md_entry.component(),
        md_entry.location_qualifier(), md_entry.affinity(),
        md_entry.handle(), md_entry.location()
      );
    }

    k = j;
  }
}; // proc_unequal_offset


// Template alias that detects if the underlying container is a STAPL
// p_container.
template<typename View>
using view_of_container_type =
  typename is_container<
    typename underlying_container<View>::type>::type;


// Compare references of two views over non-STAPL containers.
struct compare_references
{
  // Specialization for Views of the same type.
  template<typename View>
  static bool compare(View&& view0, View&& view1)
  {
    return view0.get_container() == view1.get_container();
  }

  // Specialization for Views of different type.
  template<typename V0, typename V1>
  static bool compare(V0&& view0, V1&& view1)
  {
    return false;
  }
};

// Specialization for STAPL p_containers.
template <typename V0, typename V1>
bool is_same_container_impl(V0&& view0, V1&& view1, std::true_type)
{
  return view0.container().distribution().get_rmi_handle() ==
         view1.container().distribution().get_rmi_handle();
}

// Specialization for non-STAPL containers.
template <typename V0, typename V1>
bool is_same_container_impl(V0&& view0, V1&& view1, std::false_type)
{
  return compare_references::compare(view0, view1);
}

// Function that detects if the underlying containers of two views are the
// same.
template <typename V0, typename V1>
bool is_same_container(V0&& view0, V1&& view1)
{
  return is_same_container_impl<V0, V1>(view0, view1,
           std::integral_constant<bool,
             view_of_container_type<V0>::value &&
             view_of_container_type<V1>::value
           >{}
         );
}


//////////////////////////////////////////////////////////////////////
/// @brief Returns the difference between two view indices
//////////////////////////////////////////////////////////////////////
template<typename Index>
static size_t compute_delta(Index&& i, Index&& j)
{ return i > j ? i - j : j - i; }


//////////////////////////////////////////////////////////////////////
/// @brief Returns the difference between two view indices
///
/// Specialization for list_gid returns a size that will disable the
/// static metadata optimization
//////////////////////////////////////////////////////////////////////
template<typename Ptr, typename BC>
static size_t compute_delta(list_gid<Ptr, BC>&& i, list_gid<Ptr, BC>&& j)
{ return std::numeric_limits<size_t>::max(); }


//////////////////////////////////////////////////////////////////////
/// @brief Returns the difference between two view indices
///
/// Specialization for two different index types returns a size that
/// will disable the static metadata optimization
//////////////////////////////////////////////////////////////////////
template<typename Index0, typename Index1>
static size_t compute_delta(Index0&&, Index1&&)
{ return std::numeric_limits<size_t>::max(); }


//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to identify generator views such as
///   @ref counting_view
//////////////////////////////////////////////////////////////////////
template<typename Container>
struct is_functor_container
  : public std::false_type
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to identify generator views such as
///   @ref counting_view
///
/// Specialization for the case the view is backed by a functor
//////////////////////////////////////////////////////////////////////
template <typename Generator, int n, typename... Distribution>
struct is_functor_container<functor_container<Generator, n, Distribution...>>
  : public std::true_type
{ };

} // namespace detail


//////////////////////////////////////////////////////////////////////
/// @brief Generates a metadata alignment for the given view set, using
///        the view at position @p k to guide the alignment.
//////////////////////////////////////////////////////////////////////
template<typename ViewSet,
          typename = make_index_sequence<tuple_size<ViewSet>::value>>
struct guided_offset_alignment;


template<typename ViewSet, std::size_t ...Indices>
struct guided_offset_alignment<ViewSet, index_sequence<Indices...>>
{
private:
  using md_extractor_t  = metadata_from_container<ViewSet>;
  using partitions_type = typename md_extractor_t::type;

  using coarsen_part_type =
    tuple<
      typename convert_to_md_vec_array<
        typename tuple_element<Indices, partitions_type>::type>::type...>;

public:
  using type =
    tuple<
      partitioned_mix_view<
        typename tuple_element<Indices, ViewSet>::type,
        convert_to_wrap_partition<
          typename tuple_element<Indices, partitions_type>::type>
      >...
    >;

private:
  static constexpr size_t vs_size = tuple_size<ViewSet>::value;

  using position_t      = std::array<long long, vs_size>;
  using sizes_t         = std::array<size_t, vs_size>;
  using size_offset_t   = std::array<std::pair<size_t, size_t>, vs_size>;

  using md_local_copy_t =
    tuple<
      typename get_copy_md_entry_type<
        typename tuple_element<Indices, partitions_type>::type>::type...>;


  //////////////////////////////////////////////////////////////////////
  /// @brief Determines the largest portion of data that can be aligned
  ///   based on the current local metadata entries.
  ///
  /// @param views The original views
  /// @param parts The partitions generated by the metadata extraction
  /// @param align_parts The new partitions that we are currently generating
  /// @param align_pos Position at which to write new partitions
  /// @param pos The partition IDs for each view that we are currently
  ///   trying to align
  /// @param local_mds A cache of local md information (local partitions)
  /// @param fixed_size Indicate if static alignment metadata can be used
  /// @param psm @ref p_object used to push metadata to neighboring locations
  /// @param GuideIndex The index of the guiding view
  ///
  /// @return The number of partitions for each view that need to be skipped
  ///         for the next round. Note that align_parts is also populated
  ///         with new partition information.
  //////////////////////////////////////////////////////////////////////
  template<int GuideIndex>
  static position_t
  process_local_md(ViewSet const& views, partitions_type const& parts,
                   coarsen_part_type const& align_parts, sizes_t& align_pos,
                   position_t const& pos, md_local_copy_t& local_mds,
                   bool fixed_size, push_static_metadata& psm)
  {
    // compute the size of each partition and the distance from
    // the partition's first GID to the global view's first GID
    //
    size_offset offset_wf(psm);

    size_offset_t szs_offs{{
       offset_wf(get<Indices>(views), get<Indices>(parts),
                 get<Indices>(local_mds), get<Indices>(pos))...
    }};

    // the smallest distance that we have to traverse from the source
    // is the distance of the guiding view's partition
    const size_t minimum_offset = get<GuideIndex>(szs_offs).second;

    // find amongst all of the views, which has the smallest partition
    // size and which has the smallest offset from the beginning
    std::pair<size_t, size_t> sz_off = vs_map_reduce(
      element_map(), min_size_offset(), std::make_pair(-1,-1), szs_offs);

    // if the guiding view's partition has the smallest distance from the
    // beginning of the global view
    bool equal_offset = vs_map_reduce(
      equal_second(minimum_offset), stapl::logical_and<bool>(), true, szs_offs);

    if (equal_offset)
    {
      // the block size should be the smallest partition size of all
      // of the views
      size_t mblk = sz_off.first;
      position_t ks;
      vs_map(detail::proc_equal_offset(mblk, fixed_size),
             szs_offs, parts, pos, local_mds, align_parts, align_pos, ks);

      return ks;
    }

    // at this point, a view besides the guiding view has a partition
    // whose beginning is furthest from the global view's beginning

    position_t ks;
    vs_map(detail::proc_unequal_offset(minimum_offset),
           views, parts, pos, local_mds, ks);

    return ks;
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Align the views based on the view at GuideIndex.
  ///
  /// @param views The set of original views
  /// @param part_loc The metadata of the views that was generated through
  ///                 the metadata extraction process
  /// @param use_fixed_size Indicates if the metadata of the views is static
  /// @tparam GuideIndex A compile time integral constant describing which
  ///                    view in the view set is guiding the alignment process
  ///
  /// @return A set of new views that are partitioned in such a way that
  ///         they can be referenced in tandem and be aligned
  //////////////////////////////////////////////////////////////////////
  template<int GuideIndex>
  static type
  apply(ViewSet const& views, partitions_type& part_loc, bool use_fixed_size)
  {
    // a tuple of partition ids for the current local partition
    // that is being aligned
    position_t          pos;

    // the number of local partitions in part_loc
    sizes_t             local_sz;

    // the final aligned partition that is being computed
    coarsen_part_type   alignpart;

    // the position in the fixed size alignment metadata container
    sizes_t             alignpos;

    // Enable fixed size alignment only in case of overlapped views off by one.
    const size_t delta =
      detail::compute_delta(std::get<0>(views).domain().first(),
                            std::get<1>(views).domain().first());

    // Disable fixed size alignment if the first view is a generated view,
    // because the lack of locality information results in more metadata entries
    // than the optimization expects.
    use_fixed_size = use_fixed_size && delta == 1 &&
      stapl::metadata::detail::is_same_container<
        decltype(std::get<0>(views)),decltype(std::get<1>(views))>
        (std::get<0>(views), std::get<1>(views)) &&
      !detail::is_functor_container<
        typename tuple_element<0, ViewSet>::type::view_container_type>::value;

    // If we can use fixed size alignment each location except the last will
    // have two metadata entries, one for the all local elements, and one for
    // the chunk with an element in one view on this location and the other on
    // another location.
    const size_t md_sz  = 2*std::get<0>(views).get_num_locations() - 1;
    const size_t md_idx = 2*std::get<0>(views).get_location_id();

    // a local map of metadata information.
    // essentially a cache of part_loc that contains partition
    // information for local partitions
    md_local_copy_t local_md;

    // populate local_md with metadata information for local
    // partitions found in part_loc
    // also, set local_sz to be the number of local partitions
    // and pos to be be all zeroes
    vs_map(
      detail::initialize_guided_offset_alignment(use_fixed_size, md_sz, md_idx),
      pos, alignpart, alignpos, part_loc, local_md, local_sz);

    // we only need to do alignment if there are actually any local partitions
    // in the guiding view
    const bool need_alignment =
      get<GuideIndex>(part_loc).second.local_size() > 0;

    // declare the object used to send metadata to neighboring locations here
    // because the number of iterations of the while loop varies across
    // locations
    push_static_metadata psm(use_fixed_size);

    if (need_alignment)
    {
      // if the next partition id to look at is larger than the number of local
      // partitions, then we are done
      bool res = (size_t(get<GuideIndex>(pos)) < get<GuideIndex>(local_sz));

      while (res)
      {
        // for each view, we look at its partition at the index given by pos
        // and determine what is the largest portion of that partition that
        // is contained within the partition of the guiding view at index pos.
        const position_t incpos =
          process_local_md<GuideIndex>(
            views, part_loc, alignpart, alignpos, pos, local_md, use_fixed_size,
            psm
          );

        pos = position_t{{(get<Indices>(pos) + get<Indices>(incpos))...}};

        // we're done if the guiding view does not have
        // any more local partitions
        res = (size_t(get<GuideIndex>(pos)) < get<GuideIndex>(local_sz));
      }
    }

    // create the new aligned views based on the partitions created
    // in the previous steps
    return metadata::transform(views, alignpart);
  }
}; // struct guided_offset_alignment

} // namespace metadata

} // namespace stapl

#endif // STAPL_VIEWS_METADATA_COARSEN_ALIGN_GUIDED_HPP
