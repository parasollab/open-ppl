/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_METADATA_COARSEN_ALIGN_VOLUMETRIC_HPP
#define STAPL_VIEWS_METADATA_COARSEN_ALIGN_VOLUMETRIC_HPP

#include <stapl/runtime.hpp>
#include <numeric>
#include <functional>

#include <stapl/algorithms/functional.hpp>

#include <stapl/utility/tuple.hpp>
#include <stapl/utility/integer_sequence.hpp>
#include <stapl/views/metadata/coarsen_utility.hpp>
#include <stapl/views/metadata/transform.hpp>
#include <stapl/views/metadata/infinite_helpers.hpp>
#include <stapl/views/metadata/container/multiarray.hpp>

#include <stapl/paragraph/paragraph_fwd.h>
#include <stapl/utility/vs_map.hpp>
#include <stapl/utility/do_once.hpp>

#include <stapl/containers/partitions/block_partition.hpp>
#include <stapl/containers/type_traits/dimension_traits.hpp>
#include <stapl/containers/type_traits/default_traversal.hpp>

#include <stapl/views/metadata/alignment/renumbering.hpp>

namespace stapl {

namespace metadata {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Helper functor to initialize the alignment metadata container
//////////////////////////////////////////////////////////////////////
template<typename Dims>
struct initialize_metadata_view
{
  Dims m_dims;

  initialize_metadata_view(Dims const& dims) : m_dims(dims) { }

  typedef void result_type;

  template<typename OutputCont, typename InputCont>
  void operator()(OutputCont& out, InputCont& in) const
  {
    using md_container_t = typename OutputCont::view_container_type;

    out = OutputCont(new md_container_t(m_dims));

    for (auto&& md : in[get_location_id()])
      out.container().set_element(md.id(), md);
  }
};


template<typename GID, typename = make_index_sequence<tuple_size<GID>::value>>
struct element_wise_min;

//////////////////////////////////////////////////////////////////////
/// @brief Reduction operator that finds the minimum of a tuple in
///        every dimension.
//////////////////////////////////////////////////////////////////////
template<typename GID, std::size_t... Indices>
struct element_wise_min<GID, index_sequence<Indices...>>
{
  typedef GID result_type;

  template<int I>
  typename tuple_element<I, GID>::type
  min_of(GID const& x, GID const& y) const
  {
    return std::min(std::get<I>(x), std::get<I>(y));
  }

  template<typename... T>
  result_type operator()(tuple<T...> const& xs, tuple<T...> const& ys) const
  {
    return GID(min_of<Indices>(xs, ys)...);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Functor that finds the metadata entry that contains a given
///        point in the original domain (GID).
///
/// @tparam GID Type for the GID we are searching for
//////////////////////////////////////////////////////////////////////
template<typename GID>
class find_entry_and_intersect
{
  GID m_top_left;

  //////////////////////////////////////////////////////////////////////
  /// @brief Search through the set of metadata entries and find the entry
  ///        that contains a given GID.
  ///
  /// @param g    GID to search for
  /// @param part Vector of metadata entries.
  //////////////////////////////////////////////////////////////////////
  template<typename Part>
  typename Part::value_type which(GID const& g, Part& part) const
  {
    for (std::size_t i = 0; i < part.size(); ++i)
      if (part[i].domain().contains(g))
        return part[i];

    // it should be in there
    stapl::abort("GID is not found in alignment metadata");
    return part[0];
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @param top_left The gid that we are searching for.
  //////////////////////////////////////////////////////////////////////
  find_entry_and_intersect(GID const& top_left)
    : m_top_left(top_left)
  { }

  typedef GID result_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Search through the locally known partitions and find the
  ///        partition that contains the gid. Once found, set the MD
  ///        entry and return the last gid in its domain.
  ///
  /// @param global_md Array of metadata that contains locally known metadata
  ///                  entries
  /// @param md        The metadata entry that is to be set once it is found
  //////////////////////////////////////////////////////////////////////
  template<typename MDCache, typename MD>
  result_type operator()(MDCache& global_md, MD& md) const
  {
    // set of all MD entries this location knows about
    typename MDCache::value_type parts = global_md[get_location_id()];

    // find which metadata contains the gid
    MD found = which(m_top_left, parts);
    auto dom = found.domain();

    md = found;

    // reduce on the domain's last point
    return dom.last();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Add a new metadata entry to the alignment partition that restricts
///        the domain to the intersection that was just computed.
///
///        Note that it is possible for this metadata entry to be added
///        to a location that does not have locality in any of the views,
///        as it is added based on an arbitrary spatial decomposition
///        of the original domain.
///
/// @tparam CID CID for the metadata entry
/// @tparam Domain Domain of the intersection
//////////////////////////////////////////////////////////////////////
template<typename CID, typename Domain>
struct add_md_entry
{
  CID m_cid;
  Domain m_new_dom;

  add_md_entry(CID const& cid, Domain const& new_dom)
    : m_cid(cid), m_new_dom(new_dom)
  { }

  typedef void result_type;

  template<typename Part, typename MD>
  void operator()(Part& part, MD& md) const
  {
    MD new_md(m_cid, m_new_dom, md.component(),
                md.location_qualifier(), md.affinity(),
                md.handle(), md.location());

    part[part.get_location_id()].push_back(new_md);
  }
};


template<typename GID, typename CID,
         typename Indices = make_index_sequence<tuple_size<GID>::value>>
class add_new_points;


//////////////////////////////////////////////////////////////////////
/// @brief Add new points to the set of points to start looking at
///        based on the boundaries of an intersected domain.
//////////////////////////////////////////////////////////////////////
template<typename GID, typename CID, std::size_t... Indices>
class add_new_points<GID, CID, index_sequence<Indices...>>
{
  typedef std::integral_constant<size_t, tuple_size<GID>::value> dimensionality;
  typedef std::array<std::pair<GID, CID>, dimensionality::value> points_type;

  template<typename... T>
  static void no_op(T...) { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Push a point to the queue if it hasn't been seen yet and
  ///        it is still within the location's domain
  //////////////////////////////////////////////////////////////////////
  template<typename Points, typename Seen, typename Domain>
  static void push_queue(Points& points, GID const& gid, CID const& cid,
                         Seen& seen, Domain const& dom)
  {
    if (dom.contains(gid) && seen.count(gid) == 0)
    {
      points[gid] = cid;
      seen.insert(gid);
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Compute the I'th position of the i'th point to add.
  //////////////////////////////////////////////////////////////////////
  template<int I>
  static bool new_point(GID& g, CID& cid, int i, GID const& q)
  {
    // use the last + 1 if this is the correct dimension
    get<I>(g) = I == i ? get<I>(q)+1 : get<I>(g);

    // increment the CID in the dimension of the point that we're adding
    get<I>(cid) = I == i ? get<I>(cid)+1 : get<I>(cid);

    // dummy return
    return true;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Compute d points that should be added to the set of points.
  ///
  ///        The points are computed based on the following pattern:
  ///          x_0 = (q_0+1, p_1, p_2, p_3, ..., p_{d-1})
  ///          x_1 = (p_0, q_1+1, p_2, p_3, ..., p_{d-1})
  ///          x_2 = (p_0, p_1, q_2+1, p_3, ..., p_{d-1})
  ///          x_{d-1} = (p_0, p_1, p_2, p_3, ..., q_{d-1}+1)
  ///
  ///        Essentially, the i'th point has the corresponding entry from
  ///        point p, but is one past the end (q) for the i'th dimension.
  //////////////////////////////////////////////////////////////////////
  static points_type
  new_points(GID const& p, GID const& q, CID const& cid)
  {
    // the new points will all be p, with each one only slightly changed
    // in one dimension by q. additionally, all of the CIDs will be based
    // on the previous CID, with only one of the dimensions changed
    points_type pts;
    std::fill(std::begin(pts), std::end(pts), std::make_pair(p, cid));

    constexpr std::size_t size = tuple_size<points_type>::value;

    // for each dimension, generate new points and CIDs
    for (std::size_t i = 0; i < size; ++i)
      no_op(new_point<Indices>(pts[i].first, pts[i].second, i, q)...);

    return pts;
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Add new points to the set of points to start looking at, bounded
  ///        by the domain.
  ///
  /// @param p The extreme first point (top left in 2D)
  /// @param q The extreme second point (bottom right in 2D)
  /// @param points Set of points to add new points to
  /// @param seen Set of all points seen thus far
  /// @param dom Domain that bounds addition of new points
  //////////////////////////////////////////////////////////////////////
  template<typename Points, typename Seen, typename Domain>
  static void apply(GID const& p, GID const& q, CID const& cid, Points& points,
                    Seen& seen, Domain const& dom)
  {
    auto&& pts = new_points(p, q, cid);

    for (auto&& x : pts)
      push_queue(points, x.first, x.second, seen, dom);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Function object that is applied to each view's metadata container
///        that determines which location needs information about each
///        metadata entry and pushes that information to it.
///
/// @tparam Decomposed Partition that is used to decompose the work
///                    of alignment.
//////////////////////////////////////////////////////////////////////
template<typename Decomposed>
class push_alignment_md
{
  Decomposed m_decomposed;

  typedef typename Decomposed::traversal_type traversal_t;

public:
  push_alignment_md(Decomposed const& decomposed)
    : m_decomposed(decomposed)
  { }

  typedef void                                result_type;

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Determine the set of locations that require information
  ///        about a given MD entry's domain.
  ///
  /// @param dom Domain of the MD entry
  /// @return Set of locations to send the info to
  //////////////////////////////////////////////////////////////////////
  template<typename Domain>
  std::vector<std::size_t>
  responsible(Domain const& dom) const
  {
    typedef typename Domain::index_type        index_t;
    typedef nd_linearize<index_t, traversal_t> linearize_func_t;

    std::vector<std::size_t> ranks;

    if (!m_decomposed.domain().empty())
    {
      linearize_func_t linearizer(m_decomposed.dimensions());

      ranks.push_back(linearizer(m_decomposed.find(dom.first())));
      ranks.push_back(linearizer(m_decomposed.find(dom.last())));
    }

    return ranks;

  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Push local metadata entries to locations that need to know
  ///        about them for the alignment process. This is views whose
  ///        metadata entries are not infinite
  ///
  /// @param part_loc  Initial partition information (MD entries)
  /// @param global_md Array of vectors representing needed MD entries
  ///                  for each location
  //////////////////////////////////////////////////////////////////////
  template<typename Partitions, typename MDCache>
  void push_entries(Partitions& part_loc,
                  MDCache& global_md,
                  std::integral_constant<bool, true>) const
  {
    using index_type = typename Partitions::second_type::index_type;

    auto const domain = part_loc.second.domain();

    // go through each local md entry
    domain_map(domain, [&](index_type const& idx)
    {
      auto const& md = part_loc.second[idx];

      // figure out which locations need to know about this entry
      const std::vector<std::size_t> locs = this->responsible(md.domain());

      // push the entry to the locations
      for (auto&& loc : locs)
        global_md[loc].push_back(md);

    });
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Store a single copy of an infinite metadata entry per
  ///        location. This is for views with infinite domains.
  ///
  /// @param part_loc  Initial partition information (MD entries)
  /// @param global_md Array of vectors representing needed MD entries
  ///                  for each location
  //////////////////////////////////////////////////////////////////////
  template<typename Partitions, typename MDCache>
  void push_entries(Partitions& part_loc,
                  MDCache& global_md,
                  std::integral_constant<bool, false>) const
  {
    const std::size_t loc = global_md.get_location_id();

    auto const& md = *part_loc.second.begin();

    global_md[loc].push_back(md);
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Push local metadata entries to locations that need to know
  ///        about them for the alignment process.
  ///
  /// @param part_loc  Initial partition information (MD entries)
  /// @param global_md Array of vectors representing needed MD entries
  ///                  for each location
  //////////////////////////////////////////////////////////////////////
  template<typename Partitions, typename MDCache>
  void operator()(Partitions& part_loc,
                  MDCache& global_md) const
  {
    constexpr bool is_infinite =
      has_finite_domain<typename Partitions::second_type::value_type>::value;

    this->push_entries(
      part_loc, global_md, std::integral_constant<bool, is_infinite>()
    );
  }
};

} // namespace detail

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to compute the global metadata storage type
//////////////////////////////////////////////////////////////////////
template <typename Part>
struct get_copy_md_entry_vector_type
{
  typedef typename std::remove_pointer<typename Part::second_type>
    ::type::value_type value_type;
  typedef stapl::static_array<std::vector<value_type> > type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to extract a single MD entry from a partition
//////////////////////////////////////////////////////////////////////
template <typename Part>
struct get_copy_md_entry_single_type
{
  typedef typename std::remove_pointer<typename Part::second_type>
    ::type::value_type type;
};


template<typename MDCont>
struct convert_to_multidimensional_container
{
  typedef typename std::remove_pointer<typename MDCont::second_type>::type
    md_cont_type;
  typedef typename md_cont_type::value_type                     value_type;
  typedef metadata::multidimensional_container<value_type>      container_type;
  typedef typename container_type::domain_type                  domain_type;
  typedef metadata::view<container_type, domain_type>           type;
};


template<typename MDCont>
struct convert_to_isomorphic_wrap_partition
{
  typedef MDCont md_cont_type;
  typedef typename md_cont_type::value_type                     value_type;
  typedef metadata::view_wrapper<
    value_type,
    typename md_cont_type::domain_type,
    true
  >                                                     type;
};

//////////////////////////////////////////////////////////////////////
/// @brief Generates a metadata alignment for the given view set, based
///        on a volumetric decomposition of the space of the views.
///
///        This alignment policy has the requirement that all of the
///        views have the same type of domain, and that they are
///        multidimensional domains.
///
/// @todo This alignmnent policy places the metadata based on an arbitrary
///       volumetric decomposition of the view's domain. Ideally, metadata
///       should be inserted on a location where at least one of the
///       input views has locality of the data.
///
/// @todo Currently assumes first view has finite domain.  Need to insert
///       logic to inspect view domain types and select first finite domain.
///
/// @tparam ViewSet Tuple of views to align
//////////////////////////////////////////////////////////////////////
template<typename ViewSet,
          typename = make_index_sequence<tuple_size<ViewSet>::value>>
struct volumetric_alignment;


template<typename ViewSet, std::size_t ...Indices>
struct volumetric_alignment<ViewSet, index_sequence<Indices...>>
{
private:
  typedef metadata_from_container<ViewSet>  md_extractor_t;
  typedef typename md_extractor_t::type     partitions_type;

  typedef typename result_of::transform1<
    partitions_type, convert_to_multidimensional_container
  >::type md_containers_type;

  typedef typename result_of::transform1<
    //partitions_type, convert_to_isomorphic_wrap_partition
    md_containers_type, convert_to_isomorphic_wrap_partition
  >::type coarsen_wrap_part_type;

public:
  using type =
    tuple<
      partitioned_mix_view<
        typename tuple_element<Indices, ViewSet>::type,
        typename tuple_element<Indices, coarsen_wrap_part_type>::type
      >...
    >;

private:
  typedef typename result_of::transform1<
    partitions_type, get_copy_md_entry_single_type
  >::type  md_local_single_t;

  typedef typename result_of::transform1<
    partitions_type, get_copy_md_entry_vector_type
  >::type  global_md_storage_t;

  typedef typename tuple_element<0, ViewSet>::type             first_view_t;
  typedef typename first_view_t::index_type                    index_type;
  typedef typename first_view_t::domain_type                   domain_type;
  typedef typename std::remove_pointer<
    typename tuple_element<0, partitions_type>::type::second_type>::type
      ::value_type::cid_type cid_type;
  typedef typename dimension_traits<first_view_t>::type        dimension_t;
  typedef typename default_traversal<dimension_t::value>::type traversal_t;

  typedef multiarray_impl::block_partition<traversal_t> decomposed_partition_t;
  typedef typename decomposed_partition_t::value_type   decomposed_domain_t;


  //////////////////////////////////////////////////////////////////////
  /// @brief Perform a spatial decomposition of the domain of the
  ///        views that represents a partitioning of work for alignment.
  ///
  /// @param views The set of views that we are aligning
  //////////////////////////////////////////////////////////////////////
  static decomposed_partition_t
  decomposed_partition(ViewSet const& views)
  {
    return decomposed_partition_t(
      get<0>(views).domain(),
      multiarray_impl::make_multiarray_size<dimension_t::value>()(
        get_num_locations()
      )
    );
  }

  static auto
  multidimensional_rank(decomposed_partition_t const& part,
                        std::size_t const rank)
   -> decltype(nd_reverse_linearize<index_type, traversal_t>(
                 part.dimensions())(rank)
               )
  {
    return nd_reverse_linearize<index_type, traversal_t>(
             part.dimensions())(rank);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Domain of the original space that represents the part of
  ///        the space that the location @p rank is responsible for
  ///        aligning the views in.
  ///
  /// @param part Spatial decomposition of original space
  /// @param rank The location
  //////////////////////////////////////////////////////////////////////
  static decomposed_domain_t
  decomposed_domain(decomposed_partition_t const& part, std::size_t const rank)
  {
    auto mdr = multidimensional_rank(part, rank);

    return part.domain().contains(mdr) ? part[mdr] : decomposed_domain_t();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Each location exchanges metadata entries to each other based
  ///        off of the spatial decomposition represented by decomposed.
  ///
  /// @param part_loc   Container of initial metadata entries
  /// @param decomposed Spatial decompsition representing the partitioning
  ///                   of the work done for alignment.
  /// @param global_md  Container representing the local metadata entries
  ///                   needed for alignment.
  ///
  /// @todo This can be optimized and the fence can be removed if each location
  ///       performs bookkeeping about the domains of the metadata entries
  ///       that it receives. There needs to be an ICL set in each dimension
  ///       that is updated when a domain is received, and there will be
  ///       a block_until the ICL set contains the entire domain of the
  ///       space that this location is responsible for aligning.
  //////////////////////////////////////////////////////////////////////
  static void exchange_md(partitions_type& part_loc,
                   decomposed_partition_t& decomposed,
                   global_md_storage_t& global_md)
  {
    vs_map(
      detail::push_alignment_md<decomposed_partition_t>(decomposed),
      part_loc, global_md
    );

    stapl::rmi_fence();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief The main logic for traversing a volumetric grid from the
  ///        "top-left" corner of each view and intersecting the boundaries
  ///        of the coarsened chunks.
  ///
  /// @param local_dom Domain of the local portion of the space that this
  ///                  location is responsible for aligning
  /// @param aligned_entries Output chunks that will be populated as a result
  ///                        of this process
  /// @param global_md Storage for the chunks that this location is responsible
  ///                  for aligning
  ///
  /// @return A set of new views that are partitioned in such a way that
  ///         they can be referenced in tandem and be aligned
  //////////////////////////////////////////////////////////////////////
  static void traverse_grid(decomposed_domain_t const& local_dom,
                            global_md_storage_t& aligned_entries,
                            global_md_storage_t& global_md)
  {
    using set_type = std::unordered_set<index_type, stapl::hash<index_type>>;
    using map_type = std::unordered_map<
      index_type, cid_type, stapl::hash<index_type>
    >;

    // set of starting points to look for intersections
    map_type points;

    // set of seen points so they aren't re-added to points
    set_type seen;

    // start searching from the beginning of the domain that this location is
    // responsible for and set its CID to all zeroes
    points[local_dom.first()] = cid_type();

    while (!points.empty())
    {
      // pop a point to look at
      auto&& it = points.begin();
      index_type p = it->first;
      cid_type cid = it->second;
      points.erase(it);

      // tuple of md entries corresponding to the entry containing point p
      md_local_single_t mds;
      // for each view, find the entry that contains point p, place it
      // in mds and find the smallest last of those entries, effectively
      // performing an intersection
      auto q = vs_map_reduce(
        detail::find_entry_and_intersect<index_type>(p),
        detail::element_wise_min<index_type>(),
        index_bounds<index_type>::invalid(), global_md, mds
      );

      // domain of the intersection of md entries containing p
      domain_type intersection(p, q);

      // add each md entry to the aligned partition with the new domain
      vs_map(
        detail::add_md_entry<cid_type, domain_type>(cid, intersection),
        aligned_entries, mds
      );

      // add new points to start searching at
      detail::add_new_points<index_type, cid_type>::apply(
        p, q, cid, points, seen, local_dom
      );
    }
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Align the views.
  ///
  /// @tparam GuideIndex Ignored. Alignment is guided by the spatial
  ///                    decomposition of the original domain.
  /// @param views The set of original views
  /// @param part_loc The metadata of the views that was generated through
  ///                 the metadata extraction process
  /// @param use_fixed_size Indicates if the metadata of the views is static
  ///
  /// @return A set of new views that are partitioned in such a way that
  ///         they can be referenced in tandem and be aligned
  //////////////////////////////////////////////////////////////////////
  template<int GuideIndex>
  static type
  apply(ViewSet const& views, partitions_type& part_loc, bool use_fixed_size)
  {
    // initialize the final aligned  that is being computed
    global_md_storage_t aligned_entries =
      make_homogeneous_tuple<global_md_storage_t>::apply(get_num_locations());

    // create a multidimensional partition to balance the work of alignment
    auto decomposed = decomposed_partition(views);

    // figure out which portion of the global space this location is
    // responsible for aligning
    auto local_dom = decomposed_domain(decomposed, get_location_id());

    // storage to store md entries that this locations is responsible
    // for aligning
    global_md_storage_t global_md =
      make_homogeneous_tuple<global_md_storage_t>::apply(get_num_locations());

    // exchange local md with other locations that need it
    exchange_md(part_loc, decomposed, global_md);

    // if this location has work to do for alignment, traverse the grid
    // starting from the "top-left" corner and store the result in
    // aligned_entries
    if (!local_dom.empty())
      traverse_grid(local_dom, aligned_entries, global_md);

    // renumber the metadata entries to have IDs that represent its position
    // in the global space
    cid_type result_size =
      entry_renumbering<global_md_storage_t>::apply(aligned_entries);

    // create the multidimensional metadata container and populate its entries
    md_containers_type md_conts;
    vs_map(
      detail::initialize_metadata_view<cid_type>(result_size),
      md_conts,
      aligned_entries
    );

    // create the new aligned views based on the partitions created
    // in the previous steps
    return metadata::transform(views, md_conts);
  }
}; // struct volumetric_alignment

} // namespace metadata

} // namespace stapl

#endif // STAPL_VIEWS_METADATA_COARSEN_ALIGN_VOLUMETRIC_HPP
