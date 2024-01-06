/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_SLICES_VIEW_HPP
#define STAPL_VIEWS_SLICES_VIEW_HPP

#include <stapl/containers/partitions/sliced.hpp>
#include <stapl/containers/type_traits/is_base_container.hpp>
#include <stapl/containers/multiarray/deep_slice.hpp>
#include <stapl/views/segmented_view.hpp>
#include <stapl/views/sliced_view.hpp>
#include <stapl/views/type_traits/underlying_container.hpp>
#include <stapl/views/type_traits/is_identity.hpp>
#include <stapl/views/type_traits/remap_slices.hpp>
#include <stapl/views/metadata/coarsening_traits.hpp>
#include <stapl/views/metadata/projection/slices.hpp>
#include <stapl/utility/integer_sequence.hpp>
#include <stapl/utility/tuple/to_index.hpp>
#include <stapl/utility/tuple/homogeneous_tuple.hpp>
#include <stapl/utility/tuple/discard.hpp>
#include <boost/mpl/has_xxx.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Wrapper class around segmented_view template instantiation
///  used to implement slices view concept.  Exists for clarity and to
///  reduce type's symbol size.
/// @todo Propagate use of index_sequence into @ref SLICED_view
//////////////////////////////////////////////////////////////////////
template <typename Slices, typename Container, typename... OptParams>
class slices_segmented_view;

template<typename C, typename ...OptionalParams>
class multiarray_view;

namespace view_operations {

template <typename View>
struct compute_local_domain;

template <typename Slices, typename C, typename... Args, typename... OptParams>
struct compute_local_domain<stapl::slices_segmented_view<
  Slices, stapl::multiarray_view<C, Args...>, OptParams...>>
{
  using underlying_view_t = stapl::multiarray_view<C, Args...>;
  using view_t =
    stapl::slices_segmented_view<Slices, underlying_view_t, OptParams...>;
  using domain_type    = typename view_t::domain_type;
  using dimension_type = typename domain_type::dimension_type;
  using slices_type    = typename view_t::slices_type;
  static std::vector<domain_type> apply(view_t const& view,
                                        runtime::location_id loc_id,
                                        std::size_t num_locs)
  {
    std::vector<domain_type> v;
    auto&& doms =
      compute_local_domain<underlying_view_t>::apply(
        view.container().view(), loc_id, num_locs);

    if (doms.size() == 0)
    {
      v.push_back(domain_type());
      return v;
    }

    for (auto const& dom : doms)
    {
      // current partition of the sliced_view's domain
      const domain_type slice_dom(
        tuple_ops::filter<slices_type>(dom.first()),
        tuple_ops::filter<slices_type>(dom.last())
      );

      v.push_back(slice_dom);
    }

    if (v.empty())
      v.push_back(domain_type());

    return v;
  }
};

} // namespace view_operations

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Subview creator that creates deep slices whenever the
///        traditional subview is over a base container.
///
/// @tparam Slices Tuple of integral constants
/// @tparam Container Container that the slices view is over
//////////////////////////////////////////////////////////////////////
template<typename Slices, typename Container>
struct deep_slice_subview_creator
{
  static constexpr int num_slices = tuple_size<Slices>::value;

  using fixed_type = typename std::conditional<
    num_slices == 1,
    std::size_t,
    typename homogeneous_tuple_type<num_slices, std::size_t>::type
  >::type;

  using deep_slice_type = typename Container::template slice_type<
    Slices, fixed_type
  >::type;

  /// Return type of the functor
  using type = multiarray_view<deep_slice_type>;

  deep_slice_subview_creator(void)
  { }

  template<typename Container2, typename ViewDom, typename ViewMF,
           typename Index, typename Partitioner, typename MFG>
  type operator()(Container2* cnt, ViewDom const&, ViewMF&&,
                  Index const& index, Partitioner const&, MFG&&) const
  {
    auto deep_slice = cnt->template slice<Slices>(index);
    auto deep_slice_heap = new decltype(deep_slice)(deep_slice);

    return type(deep_slice_heap);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Subview creator that creates deep slices over an existing
///        slices view.
///
/// @tparam Slices Tuple of the new slices (i.e., the slices that
///   came from the slice() call.)
/// @tparam SlicesView The type of the inner slices view.
//////////////////////////////////////////////////////////////////////
template<typename Slices, typename SlicesView>
struct nested_deep_slice_subview_creator
{
  static constexpr int num_slices = tuple_size<Slices>::value;
  using fixed_type = typename std::conditional<
    num_slices == 1,
    std::size_t,
    typename homogeneous_tuple_type<num_slices, std::size_t>::type
  >::type;

  using type =
    typename SlicesView::template slice_type<Slices, fixed_type>::type;

  nested_deep_slice_subview_creator(void)
  { }

  template<typename Container, typename ViewDom, typename ViewMF,
           typename Index, typename Partitioner, typename MFG>
  type operator()(Container* cnt, ViewDom const&, ViewMF&&,
                  Index const& index, Partitioner const&, MFG&&) const
  {
    SlicesView sv(*cnt);
    return sv.template slice_as_slices<Slices>(index);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Subview creator that creates shallow slices (SLICED_view)
///        when it is not possible to create deep slices.
///
/// @tparam Slices Tuple of integral constants
/// @tparam Container Container that the slices view is over
/// @tparam Domain New domain of the SLICED_view
/// @tparam MF New mapping function for the SLICED_view
//////////////////////////////////////////////////////////////////////
template<typename Slices, typename Container, typename Domain, typename MF>
struct shallow_slice_subview_creator
 : public view_impl::default_subview_creator<
     SLICED_view<Slices, Container, Domain, MF>
   >
{
  shallow_slice_subview_creator(void)
  { }
};

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction that projects the given sliced dimensions
///   down the hierarchy of composed slices views, stopping at
///   the second level.
///
/// Provides a member @p type which is set to a tuple of integral constants
/// representing the projected dimensions, or to @p void in case of more
/// than two levels of composition.
///
/// @tparam Slices  Tuple of @p integral_constants specifying the
///   sliced dimensions at the actual level of the composition.
/// @tparam Container  The container at the next level of the composition.
/// @tparam Lvl  Actual level of the composition (starting at 0).
/// @tparam unnamed  Boolean indicating whether the next level container is
///    again a segmented view or the final level of the composition has been
///    reached.
//////////////////////////////////////////////////////////////////////
template<typename Slices, typename Container, int Lvl,
  bool = is_segmented_view<Container>::value>
struct project_slices
{
  using projected_slices = typename tuple_ops::result_of::heterogeneous_filter<
    Slices, typename Container::slices_type
  >::type;

  using type = typename project_slices<
    projected_slices,
    typename Container::view_container_type::view_container_type,
    Lvl+1
  >::type;
};


template<typename Slices, typename Container, int Lvl>
struct project_slices<Slices, Container, Lvl, false>
{
  using type = Slices;
};

template<typename Slices, typename Container>
struct project_slices<Slices, Container, 2, true>
{
  using type = void;
};

template<typename Slices, typename Container>
struct project_slices<Slices, Container, 2, false>
{
  using type = void;
};

BOOST_MPL_HAS_XXX_TRAIT_DEF(traversal_type)

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to determine whether a slices specification is
///   valid for deep slicing of given container.
///
/// Provides a boolean @p value which is true iff the sliced dimensions
/// are the last of the  UnderlyingContainer's traversal order. This is
/// ascertained by testing whether @ref indices_of_most_significant will
/// return a tuple of integral constants in the [0, |Slices|-1] range
/// (which will eventually be used to @ref rearrange the @p Slices tuple).
///
/// @tparam UnderlyingContainer  The bottom-most container that is being
///   sliced (a multi-dimensional container type providing @p traversal_type).
/// @tparam Slices  Tuple of @p integral_constants specifying the final
///   sliced dimensions (projected through all levels of an eventual composition
///   of slices views, which is no more than two levels deep).
/// @tparam unnamed Because clang doesn't short-circuit constexpr boolean,
///   conditions, we need to test that the underlying container has
///   @p traversal_type here. GCC instantiates valid_slicing only in the case
///   @p UnderlyingContainer is not a deep slice and is multi-dimensional (in
///   which case it should have @p traversal_type), thanks to the appropriate
///   tests performed when setting slices_view_types::is_deep_sliceable.
//////////////////////////////////////////////////////////////////////
template<typename UnderlyingContainer, typename Slices,
  bool = has_traversal_type<UnderlyingContainer>::value>
struct valid_slicing
{
private:
  using traversal = typename UnderlyingContainer::traversal_type;
  using order = typename indices_of_most_significant<Slices, traversal>::type;

  static constexpr bool valid_order(std::size_t max)
  {
    return true;
  }

  template <typename Head, typename... Tail>
  static constexpr bool valid_order(std::size_t max, Head x, Tail... y)
  {
    return 0 <= x && x < max && valid_order(max, y...);
  }

  template <typename... OrderICs>
  static constexpr bool valid_order(std::size_t max, tuple<OrderICs...> const&)
  {
    return valid_order(max, OrderICs::value...);
  }

public:
  static constexpr bool value =
    valid_slicing::template valid_order(tuple_size<Slices>::value, order{});
};

template<typename Container, typename Slices>
struct valid_slicing<Container, Slices, false>
{
  static constexpr bool value = false;
};

//////////////////////////////////////////////////////////////////////
/// brief Specialization for the case of a composition of three or more
///   @ref slices_segmented_view types.
///
/// This case is indicated by passing @p void as the @p Slices type in
/// the generic case. Deep slicing is currently unconditionally disabled
/// in this case, resulting in a fall-back to the general shallow slicing.
///
/// @todo: Enable deep slicing for more than two levels of slices view
///   nesting.
//////////////////////////////////////////////////////////////////////
template<typename Container>
struct valid_slicing<Container, void, true>
{
  static constexpr bool value = false;
};

template<typename Container>
struct valid_slicing<Container, void, false>
{
  static constexpr bool value = false;
};


//////////////////////////////////////////////////////////////////////
/// @brief Defines intermediate types used to define instantiation
///  of a segmented view which underlies the slices view concept.
//////////////////////////////////////////////////////////////////////
template <typename Slices, typename Container, typename... OptParams>
struct slices_view_types
{
  typedef typename view_traits<
    SLICED_view<Slices, Container, OptParams...>
   >::map_function                                       map_function;

  typedef map_fun_gen1<map_function>                     mfg_type;

  typedef typename view_traits<
    SLICED_view<Slices, Container, OptParams...>
   >::domain_type                                        domain_type;

  typedef sliced_partition<
    Slices, typename Container::domain_type
  >                                                      partition_type;

  using underlying_container_type = underlying_container_t<Container>;

  //////////////////////////////////////////////////////////////////////
  /// The underlying container is deep sliceable if it's a deep slice
  /// itself or it is a multidimensional base container, the sliced
  /// dimensions are the last of its traversal order and the eventual
  /// nesting of the slices views over it is no more than two levels deep.
  //////////////////////////////////////////////////////////////////////
  static constexpr bool is_deep_sliceable =
    is_identity<typename view_traits<Container>::map_function>::value && (
      is_deep_slice<underlying_container_type>::value || (
        is_base_container<underlying_container_type>::value &&
        dimension_traits<underlying_container_type>::type::value > 1 &&
        valid_slicing<
          underlying_container_type,
          typename project_slices<Slices, Container, 0>::type
        >::value
      )
    );

  // Decide what the subview creator should be based on whether or not the
  // container is deep sliceable
  using subview_creator_type = typename std::conditional<
    is_deep_sliceable, /* if */
    typename std::conditional< /* then */
      is_segmented_view<Container>::value, /* if */
      nested_deep_slice_subview_creator<Slices, Container>, /* then */
      deep_slice_subview_creator<Slices, Container> /* else */
    >::type,
    shallow_slice_subview_creator< /* else */
      Slices, Container, domain_type, map_function
    >
  >::type;

  using type = segmented_view<
    Container, partition_type, mfg_type, subview_creator_type>;
};


} // namespace detail

template <std::size_t... Indices, typename Container, typename... OptParams>
class slices_segmented_view<index_sequence<Indices...>, Container, OptParams...>
  : public detail::slices_view_types<
      tuple<std::integral_constant<std::size_t, Indices>...>, Container,
      OptParams...>::type
{
public:
  using slices_type = tuple<std::integral_constant<std::size_t, Indices>...>;
  using underlying_view_type = Container;

private:
  using base_type =
    typename detail::slices_view_types<
      slices_type,
      Container,
      OptParams...>::type;

public:
  using domain_type = typename base_type::domain_type;

  template<typename... Args>
  slices_segmented_view(Args&&... args)
    : base_type(std::forward<Args>(args)...)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Metafunction to compute the result of calling slice<Slices>
  ///        on this view.
  ///
  /// @tparam Slices The indices to slice on this view
  /// @tparam Fixed The type of element that will be used to specify the
  ///               fixed values. Typically a tuple of size |Slices|.
  //////////////////////////////////////////////////////////////////////
  template<typename Slices, typename Fixed>
  struct slice_type
  {
    static constexpr std::size_t underlying_dims =
      dimension_traits<Container>::type::value;

    // Compute what Slices should be based on the current view's slice indices
    // and the dimensionality of the container that it's over. This will be used
    // as the new slices for the return view
    using remapped = typename remap_slices<
      index_sequence<Indices...>,
      typename tuple_ops::to_index_sequence<Slices>::type,
      underlying_dims
    >::type;

    // Since Slices really refers to the indices of this view, use Slices as
    // an index array into this view's actual slices. This will be used to
    // call slice on the underlying container.
    using projected_slices =
      typename tuple_ops::result_of::heterogeneous_filter<Slices,
                                                          slices_type>::type;

    using map_func_t =
      typename Container::map_func_type::template slice_type<Slices,
                                                             Fixed>::type;

    using underlying_cont_t =
      typename Container::template slice_type<projected_slices, Fixed>::type;

    using underlying_dom_t  = typename underlying_cont_t::domain_type;

    using underlying_view_t =
      multiarray_view<underlying_cont_t, underlying_dom_t, map_func_t>;

    // The type of the new slices view
    using type = slices_segmented_view<
      typename tuple_ops::to_index_sequence<remapped>::type,
      underlying_view_t
    >;
  };

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a deep slice of this view. Returns a new
  ///        slices_segmented_view over a deep slice.
  ///
  /// @tparam Slices The indices to slice on this view
  //////////////////////////////////////////////////////////////////////
  template<typename Slices, typename Fixed>
  typename slice_type<Slices, Fixed>::type
  slice_as_slices(Fixed const& fixed)
  {
    using projected_slices =
      typename slice_type<Slices, Fixed>::projected_slices;

    using underlying_dom_t =
      typename slice_type<Slices, Fixed>::underlying_dom_t;

    // Invoke slice on the container that this segmented view is over.
    auto deep = this->get_container()-> template slice<projected_slices>(fixed);

    auto deep_heap = new decltype(deep)(deep);

    using underlying_view_t =
      typename slice_type<Slices, Fixed>::underlying_view_t;

    auto underlying_dimension = tuple_ops::discard<projected_slices>(
      this->get_container()->get_view()->dimensions());

    return {
      view_impl::store_in_frame{},
      underlying_view_t(deep_heap, underlying_dom_t(underlying_dimension))};
  }

  template <typename Slices, typename Fixed>
  typename slice_type<Slices, Fixed>::type
  slice(Fixed const& fixed)
  {
    return slice_as_slices<Slices>(fixed);
  }

  void define_type(typer& t)
  {
    t.base<base_type>(*this);
  }
};  // class slices_segmented_view


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for changing the container used for a
///        slices segmented view
//////////////////////////////////////////////////////////////////////
template<typename Slices,
         typename Container,
         typename... OptParams,
         typename NewC >
struct cast_container_view<
  slices_segmented_view<Slices, Container, OptParams...>, NewC
>
{
  using type =
    slices_segmented_view<Slices,
                          typename cast_container_view<Container, NewC>::type,
                          OptParams...>;
};

namespace metadata {

template <typename Slices, typename Container,
          typename... OptParams>
struct coarsening_traits<
  slices_segmented_view<Slices, Container, OptParams...>>
{
  typedef slices_segmented_view<Slices, Container, OptParams...>
    view_type;

  template<typename P>
  struct construct_projection
  {
    typedef slices_projection<const view_type, P> type;
  };
};

} // namespace metadata

//////////////////////////////////////////////////////////////////////
/// @brief Create a slices view over a d-dimensional view, which
///        is a view of dimension |Slices| that produces subviews
///        of dimension d-|Slices|. This essentially produces a
///        @ref SLICED_view for each possible value of the dimension
///        that is being sliced along.
///
///        For example, a slices view slicing off dimension 1 of a 3D
///        view will create a 1D view whose domain is all of the values
///        of the 1st dimension and whose elements are 2D views.
///
/// @tparam Slice A variadic list of dimensions to slice along
/// @param ct The view to create the slices view on top of
//////////////////////////////////////////////////////////////////////
template<std::size_t... Slice, typename Container>
slices_segmented_view<index_sequence<Slice...>, Container>
make_slices_view(Container const& ct)
{
  using view_type =
    slices_segmented_view<index_sequence<Slice...>, Container>;

  return view_type(ct, typename view_type::partition_type(ct.domain()));
}

//////////////////////////////////////////////////////////////////////
/// @brief Alternate version where Slices are passed as an index_sequence.
//////////////////////////////////////////////////////////////////////
template<typename Slices, typename Container>
slices_segmented_view<Slices, Container>
make_slices_view(Container const& ct)
{
  static_assert(is_index_sequence<Slices>::value, "Slices must be specified as "
    "an index_sequence");

  typedef slices_segmented_view<Slices, Container> view_type;

  return view_type(ct, typename view_type::partition_type(ct.domain()));
}

//////////////////////////////////////////////////////////////////////
/// @brief Create a slices view over a d-dimensional view,
///        where the underlying container is constructed in
///        place and whose lifetime is managed by the slices view
///
/// @param tag  the tag to specify the life time
///              management of underlying container
/// @param args the underlying view/container which the slices view is
///              created over.
//////////////////////////////////////////////////////////////////////
template<typename Container, std::size_t... Slice, typename... Args>
slices_segmented_view<index_sequence<Slice...>, Container>
make_slices_view(view_impl::store_in_frame tag, Args&&... args)
{
  using view_type =
    slices_segmented_view<index_sequence<Slice...>, Container>;

  return view_type(tag, std::forward<Args>(args)...);
}

//////////////////////////////////////////////////////////////////////
/// @brief Alternate version where Slices are passed as an index_sequence.
//////////////////////////////////////////////////////////////////////
template<typename Container, typename Slices, typename... Args>
slices_segmented_view<Slices, Container>
make_slices_view(view_impl::store_in_frame tag, Args&&... args)
{
  using view_type = slices_segmented_view<Slices, Container>;

  return view_type(tag, std::forward<Args>(args)...);
}

} // namespace stapl

#endif
