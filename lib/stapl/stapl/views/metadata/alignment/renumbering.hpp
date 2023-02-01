/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_METADATA_ALIGNMENT_RENUMBERING_HPP
#define STAPL_VIEWS_METADATA_ALIGNMENT_RENUMBERING_HPP


#include <stapl/containers/multiarray/multiarray.hpp>
#include <stapl/containers/type_traits/dimension_traits.hpp>
#include <stapl/views/multiarray_view.hpp>
#include <stapl/views/functor_view.hpp>
#include <stapl/utility/integer_sequence.hpp>

#include <stapl/skeletons/executors/execute.hpp>
#include <stapl/skeletons/functional/wavefront.hpp>
#include <stapl/skeletons/functional/map_reduce.hpp>
#include <stapl/skeletons/functional/broadcast_to_locs.hpp>

#include <stapl/containers/array/static_array.hpp>
#include <stapl/views/array_view.hpp>

namespace stapl {

namespace metadata {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Helper function to make a boundary view of all zeroes.
//////////////////////////////////////////////////////////////////////
template<typename T>
struct zero_generator
{
  using index_type = T;
  using result_type = T;

  result_type operator()(index_type const&) const
  {
    return T{};
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Make a boundary view of all zeroes.
///
/// @param size A tuple of the sizes in each dimension.
//////////////////////////////////////////////////////////////////////
template<int I, typename Size>
auto make_boundary_view(Size&& size) -> decltype(
  functor_view(size, zero_generator<Size>())
 )
{
  return functor_view(size, zero_generator<Size>());
}


//////////////////////////////////////////////////////////////////////
/// @brief Increment a d-dimensional tuple with d d-dimensional tuples
///        by adding the i'th dimension with the i'th dimension of the i'th
///        tuple.
///
///        For example, if the inputs are:
///          (1,2), (3,4), (5,6)
///        The expected output is (1+3, 2+6) = (4,8)
///
///        Also assigns the input tuple to be the i'th element of the i'th
///        tuple. Using the same example, we will assign the input to be (3,6).
///
/// @tparam T Tuple type
//////////////////////////////////////////////////////////////////////
template<typename T>
struct piecewise_sum
{
  using result_type = T;

  //////////////////////////////////////////////////////////////////////
  /// @brief Assign the i'th value of x to be the i'th value of y.
  ///        That is, perform x[i] = y[i].
  ///
  /// @tparam I Compile time index
  /// @param x Output tuple
  /// @param y Input tuple
  //////////////////////////////////////////////////////////////////////
  template<int I, typename X, typename Y>
  void assign(X& x, Y&& y)
  {
    T y_copy = y;

    get<I>(x) = get<I>(y_copy);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a new tuple based on the i'th value of the i'th input tuple
  ///
  /// @param t The tuple to write to
  /// @param us A composed tuple of the other tuples.
  //////////////////////////////////////////////////////////////////////
  template<std::size_t... Indices, typename X, typename Ys>
  void extract(index_sequence<Indices...>, X&& t, Ys&& us)
  {
    using expander = int[];

    (void) expander{0, (
    assign<Indices>(t, get<Indices>(us))
    , 0)...};
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Add two tuples together piecewise (x[i] += y[i])
  ///
  /// @param lhs The tuple that is being incremented
  /// @param rhs The tuple that is being read from
  //////////////////////////////////////////////////////////////////////
  template<std::size_t... Indices, typename LHS, typename RHS>
  void add(index_sequence<Indices...>, LHS& lhs, RHS&& rhs)
  {
    using expander = int[];

    (void) expander{0, (
    get<Indices>(lhs) += get<Indices>(rhs)
    , 0)...};
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Compute a piecewise sum of a tuple with d other tuples.
  ///        Mutates the first tuple to contain the i'th element of
  //         the i'th tuple (excluding itself).
  ///
  /// @param t The original tuple and the one that will be modified
  /// @param us Pack of tuples on which to extract and add
  //////////////////////////////////////////////////////////////////////
  template<typename X, typename... Ys>
  result_type operator()(X&& t, Ys&&... us)
  {
    T t_copy = t;
    T rhs{};

    // take the i'th value of the i'th tuple and store it in rhs
    extract(make_index_sequence<sizeof...(Ys)>(),
      rhs, std::forward_as_tuple(us...)
    );

    // mutate the input tuple
    t = rhs;

    // do a piecewise sum of the input and the dimension-wise extraction of the
    // other tuples
    add(make_index_sequence<sizeof...(Ys)>(), t_copy, rhs);

    return t_copy;
  }
};

template<typename GID,
         typename Indices = make_index_sequence<tuple_size<GID>::value>>
struct element_wise_max;

//////////////////////////////////////////////////////////////////////
/// @brief Reduction operator that finds the maximum of a tuple in
///        every dimension.
///
/// @tparam GID The d-dimensional ID type
/// @tparam Indices Integral sequence from 0..d-1
//////////////////////////////////////////////////////////////////////
template<typename GID, std::size_t... Indices>
struct element_wise_max<GID, index_sequence<Indices...>>
{
  using result_type = GID;

  //////////////////////////////////////////////////////////////////////
  /// @brief Compute the max of two tuples only in the I'th dimension
  //////////////////////////////////////////////////////////////////////
  template<int I>
  typename tuple_element<I, GID>::type
  max_of(GID const& x, GID const& y) const
  {
    return std::max(std::get<I>(x), std::get<I>(y));
  }

  template<typename... T>
  result_type operator()(tuple<T...> const& xs, tuple<T...> const& ys) const
  {
    return result_type(max_of<Indices>(xs, ys)...);
  }

  template<typename... T, typename A1, typename A2, typename... U>
  result_type operator()(proxy<tuple<T...>, A1> const& t,
                         proxy<tuple<U...>, A2> const& u) const
  {
    result_type x = t;
    result_type y = u;

    return this->operator()(x,y);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Add an offset to all of the metadata entry IDs generated
///        by this location
///
/// @tparam CID Metadata entry ID type
//////////////////////////////////////////////////////////////////////
template<typename CID>
class increment_entry_cids
{
  CID m_offset;

public:
  using result_type = void;

  increment_entry_cids(CID const& cid)
   : m_offset(cid)
  { }

  template<typename MDCont>
  void operator()(MDCont& md_cont) const
  {
    for (auto&& md : md_cont[md_cont.get_location_id()])
    {
      CID id = vs_map(stapl::plus<std::size_t>(), md.id(), m_offset);

      md.set_id(id);
    }
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Compute the max CID in a collection of metadata entries
///
/// @tparam CID Metadata entry ID
//////////////////////////////////////////////////////////////////////
template<typename CID>
struct md_collection_max_cid
{
  using cid_type = CID;
  using result_tye = cid_type;

  template<typename Collection>
  cid_type operator()(Collection&& c)
  {
    // compute the max CID in each dimension
    detail::element_wise_max<cid_type> maxer{};
    cid_type largest{};

    for (auto&& md : c)
      largest = maxer(largest, md.id());

    return largest;
  }
};

} // namespace detail

//////////////////////////////////////////////////////////////////////
/// @brief Function object that renumbers the IDs of all metadata entries
///        to represent its ID in the global space.
///
/// @tparam MDConts The tuple of MD containers that have been created as
///                 a result of the alignment process.
//////////////////////////////////////////////////////////////////////
template<typename MDConts>
class entry_renumbering
{
  /// Type of one of the metadata containers.
  using md_container_type = typename tuple_element<0, MDConts>::type;

  /// Base container ID
  using cid_type = typename md_container_type::value_type::value_type::cid_type;

  /// The dimensionality of the views
  using dimension_t = typename dimension_traits<cid_type>::type;

  template<typename Offsets>
  static cid_type
  multidimensional_rank(Offsets&& offsets)
  {
    using traversal_t =
      typename std::decay<Offsets>::type::view_container_type::traversal_type;
    using project_func_t = nd_reverse_linearize<cid_type, traversal_t>;

    std::size_t const rank = get_location_id();
    cid_type const dims = offsets.dimensions();

    return project_func_t(dims)(rank);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Execute the wavefront skeleton by first creating d boundary views
  ///        and invoking the non-coarsened executor.
  ///
  /// @param offsets The multidimensional container of offsets for each
  ///                location. This will be populated to be the global offsets.
  /// @param skeleton The wavefront skeleton
  //////////////////////////////////////////////////////////////////////
  template<typename Offsets, typename Skeleton, std::size_t... Dims>
  static void execute_wavefront(Offsets&& offsets, Skeleton&& skeleton,
                                index_sequence<Dims...>)
  {
    skeletons::execute(
      skeletons::default_execution_params(),
      skeleton,
      offsets, detail::make_boundary_view<Dims>(offsets.dimensions())...
    );
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Perform a multidimensional wavefront to compute the offsets
  ///        of the metadata entries in each dimension per location.
  ///
  ///        If the sizes on each location look like the following:
  ///
  ///        <code>
  ///          (1,4)  -->  (6, 2)
  ///            |           |
  ///            v           v
  ///          (3,5)  -->  (1,1)
  ///        </code>
  ///
  ///        The expected offsets that need to be added to each ID would be:
  ///
  ///        <code>
  ///          (0,0)  -->  (0, 4)
  ///            |           |
  ///            v           v
  ///          (1,0)  -->  (6,5)
  ///        </code>
  ///
  /// @param offsets The multidimensional container of offsets for each
  ///                location. This will be populated to be the global offsets.
  //////////////////////////////////////////////////////////////////////
  template<typename Offsets>
  static void wavefront_renumbering(Offsets& offsets)
  {
    using dims_tuple_t = make_index_sequence<dimension_t::value>;

    using wp = stapl::skeletons::position;

    // set up the corners to be the "top-left" (first in all dimensions)
    std::array<wp, dimension_t::value> corners;
    std::fill(std::begin(corners), std::end(corners), wp::first);

    auto wavefront_skeleton = skeletons::wavefront(
      detail::piecewise_sum<cid_type>(), corners
    );

    execute_wavefront(offsets, wavefront_skeleton, dims_tuple_t());
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Increment a base container ID by 1 in every dimension
  //////////////////////////////////////////////////////////////////////
  static cid_type increment_cid(cid_type const& cid)
  {
    return vs_map(boost::bind(stapl::plus<std::size_t>(), _1, 1), cid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Compute the number of metadata entries in each dimension.
  ///
  /// @param md_cont Container of per-location metadata collections
  //////////////////////////////////////////////////////////////////////
  template<typename MDCont>
  static cid_type size(MDCont& md_cont)
  {
    auto vw = make_array_view(md_cont);

    // get the max cid from the local md collection and globally reduce
    auto mr_skeleton = skeletons::map_reduce(
      detail::md_collection_max_cid<cid_type>(),
      detail::element_wise_max<cid_type>()
    );

    // broadcast the max value to all locations
    auto mr_broadcast_skeleton =
      skeletons::compose(mr_skeleton, skeletons::broadcast_to_locs<true>());

    // execute the skeleton to get the max cid globally
    cid_type largest = skeletons::execute(
        skeletons::execution_params<cid_type>(),
        mr_broadcast_skeleton,
        vw);

    // add 1 to each dimension to get the sizes in each dimension
    return increment_cid(largest);
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Renumber the IDs of all metadata entries to represent its ID
  ///        in the global space.
  ///
  ///        This is performed after the alignment has created all of the
  ///        metadata entries, but has not given them the correct globally
  ///        unique ID.
  ///
  /// @param md_conts The tuple of MD containers that have been created as
  ///                 a result of the alignment process. The element of each
  ///                 container is assumed to be a collection of metadata
  ///                 representing all of the metadata generated from a single
  ///                 location.
  //////////////////////////////////////////////////////////////////////
  static cid_type apply(MDConts& md_conts)
  {
    // we can use only the first metadata container since at this
    // point, they should all have entries with identical domains and IDs
    auto& md_cont = get<0>(md_conts);

    // compute the max CID in each dimension
    detail::md_collection_max_cid<cid_type> maxer{};
    cid_type largest = maxer(md_cont[get_location_id()]);

    // add 1 to each dimension to get the sizes in each dimension
    cid_type sizes = increment_cid(largest);

    // now we know the sizes in each dimension on this location

    using offsets_type = multiarray<dimension_t::value, cid_type>;
    using offsets_view_type = multiarray_view<offsets_type>;

    // create a multiarray to store the offsets
    offsets_type offsets_ct{
      multiarray_impl::make_multiarray_size<dimension_t::value>()(
        get_num_locations()
      )
    };

    offsets_view_type offsets(offsets_ct);

    // get the tuple representing this location in an n-d space
    auto const rank = multidimensional_rank(offsets);

    // set the offset for this location in the multiarray
    offsets[rank] = sizes;

    // perform a wavefront starting from the top left location to
    // globally renumber
    wavefront_renumbering(offsets);

    // use the result of the wavefront to get this location's offset as a tuple
    cid_type offset = offsets[rank];

    // increment entries for all views with the global offset
    vs_map(detail::increment_entry_cids<cid_type>(offset), md_conts);

    // return the number of total metadata entries in each dimension
    return size(md_cont);
  }
}; // struct entry_renumbering

} // namespace metadata

} // namespace stapl

#endif // STAPL_VIEWS_METADATA_ALIGNMENT_RENUMBERING_HPP
