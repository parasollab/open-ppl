/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_MULTIARRAY_VIEW_HPP
#define STAPL_VIEWS_MULTIARRAY_VIEW_HPP

#include <stapl/utility/tuple.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/containers/type_traits/dimension_traits.hpp>
#include <stapl/views/mapping_functions/mapping_functions.hpp>
#include <stapl/views/mapping_functions/linearization.hpp>
#include <stapl/containers/type_traits/default_traversal.hpp>
#include <stapl/utility/use_default.hpp>
#include <stapl/views/operations/multi_dimensional_subscript.hpp>
#include <stapl/domains/indexed.hpp>
#include <stapl/views/type_traits/underlying_container.hpp>
#include <stapl/views/type_traits/is_segmented_view.hpp>
#include <stapl/views/metadata/coarsening_traits.hpp>
#include <stapl/views/metadata/projection/multiarray_view_over_array_view.hpp>
#include <stapl/containers/array/array_fwd.hpp>

#include <iostream>

namespace stapl {

#ifdef STAPL_DOCUMENTATION_ONLY

//////////////////////////////////////////////////////////////////////
/// @brief Provide a one dimensional, random access view over a
///   multi-dimensional view.
/// @tparam View The underlying multi-dimensional view to linearize.
/// @tparam Traversal The traversal method to use for linearization
/// @todo Remove linear_view freestanding function idiom and rename
///  this class to linear_view.
//////////////////////////////////////////////////////////////////////
template<typename View,
         typename Traversal = typename default_traversal<
             tuple_size<typename view_traits<View>::index_type>::value
           >::type>
class linear_view_impl;

#else

template<typename View, typename ...OptionalParams>
class linear_view_impl;

#endif


//////////////////////////////////////////////////////////////////////
/// @brief Type metafunction to select a default traversal if a
///   @ref linear_view has not been passed an optional traversal
///   type parameter.  Otherwise, return the passed type.
//////////////////////////////////////////////////////////////////////
template<int N, typename ...OptionalTraversal>
struct compute_traversal_type;


template<int N>
struct compute_traversal_type<N>
{ typedef typename default_traversal<N>::type type; };


template<int N, typename Traversal>
struct compute_traversal_type<N, Traversal>
{ typedef Traversal type; };


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for when all four type parameters are specified.
//////////////////////////////////////////////////////////////////////
template<typename View, typename ...OptionalParams>
struct view_traits<linear_view_impl<View, OptionalParams...>>
  : default_view_traits<
      typename view_traits<View>::container,
      indexed_domain<size_t>,
      typename compose_func<
        typename view_traits<View>::map_function,
        nd_reverse_linearize<
          typename view_traits<View>::index_type,
          typename compute_traversal_type<
           tuple_size<typename view_traits<View>::index_type>::value,
           OptionalParams...>::type>>::type,
      linear_view_impl<View, OptionalParams...>
    >
{ };

template <typename View>
struct is_SLICED_view;

//////////////////////////////////////////////////////////////////////
/// @brief Meta-function providing a public member @p value which is
///   true if given view composition starting with @ref linear_view_impl
///   should be preserved.
///
/// This specialization is needed because the default implementation of
/// @p preserve_composition uses @p view_container_type to recurse down
/// the composition, but @p linear_view_impl::view_container_type doesn't
/// refer to the view being linearized, but rather to its underlying
/// container.
///
/// @todo We should be able to flatten linear_view_impl<SLICED_view<...>>,
///   but there is currently a mapping function type mis-match when
///   constructing the corresponding localized_view.
//////////////////////////////////////////////////////////////////////
template<typename View, typename... OptionalParams>
struct preserve_composition<linear_view_impl<View, OptionalParams...>>
{
  static constexpr bool value = should_preserve<View>::value;
};

//////////////////////////////////////////////////////////////////////
/// @brief Transform a linear view to one over a base container.
///
/// For linear views over views other than @ref segmented_view (and views
/// derived from it) or @ref SLICED_view, the variadic
/// @ref new_cast_container_view metafunction is used (returning a linear view
/// over the base container with a mapping function composed of all the
/// underlying views). For linear views over a @p segmented_view or
/// @p SLICED_view, a linear view over the casted @p segmented_view or
/// @p SLICED_view is returned (with its original mapping function).
//////////////////////////////////////////////////////////////////////
template<typename OldC, typename ...OptionalParams, typename NewC>
struct cast_container_view<linear_view_impl<OldC, OptionalParams...>, NewC>
  : std::conditional<
      is_segmented_view<OldC>::value || is_SLICED_view<OldC>::value,
      cast_container_view_preserve_mapfunc<
        linear_view_impl<OldC, OptionalParams...>,
        typename cast_container_view<OldC, NewC>::type
      >,
      new_cast_container_view<
        linear_view_impl<OldC, OptionalParams...>, NewC
      >
    >::type
{ };

//////////////////////////////////////////////////////////////////////
/// @brief Specialization ensures container, domain and mapping function
///   transform for variadic based optionals is used.
//////////////////////////////////////////////////////////////////////
template<typename C, typename ...OptionalParams, typename Dom, typename MF>
struct upcast_view<linear_view_impl<C, OptionalParams...>, Dom, MF>
  : new_upcast_view<linear_view_impl<C, OptionalParams...>, Dom, MF>
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Implementation of the linear view concept (a 1D view providing
///        access to all elements of given multidimensional view by using
///        linearized indices).
///
/// @tparam View  original multidimensional view to be linearized
/// @tparam OptionalParams  optional parameter specifying the traversal
///                         order used for linearization
//////////////////////////////////////////////////////////////////////
template<typename View, typename ...OptionalParams>
class linear_view_impl
  : public array_view<
      typename view_traits<View>::container,
      indexed_domain<size_t>,
      typename compose_func<
        typename view_traits<View>::map_function,
        nd_reverse_linearize<
         typename view_traits<View>::index_type,
         typename compute_traversal_type<
           tuple_size<typename view_traits<View>::index_type>::value,
           OptionalParams...>::type>>::type,
      linear_view_impl<View, OptionalParams...>
  >
{
public:
  STAPL_VIEW_REFLECT_TRAITS(linear_view_impl)

private:
  typedef typename view_traits<linear_view_impl>::derived_type derived_type;

  typedef array_view<view_container_type, domain_type,
                     map_function, linear_view_impl>           base_type;

  typedef view_operations::sequence<derived_type>              sequence_op_type;

public:
  typedef typename sequence_op_type::iterator                  iterator;
  typedef typename sequence_op_type::const_iterator            const_iterator;

  linear_view_impl(void) = default;
  linear_view_impl(linear_view_impl const&) = default;
  linear_view_impl(linear_view_impl&&) = default;

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor used to pass ownership of the container to the view.
  ///
  /// @param vcont Pointer to the container used to forward the operations.
  /// @param dom Domain to be used by the view.
  /// @param mfunc Mapping function to transform view indices to container
  ///              gids.
  //////////////////////////////////////////////////////////////////////
  linear_view_impl(view_container_type* vcont,
             domain_type const& dom,
             map_func_type const& mfunc = map_function())
    : base_type(vcont, dom, mfunc)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor used to pass ownership of the container to the view.
  ///
  /// @param vcont Pointer to the container used to forward the operations.
  /// @param dom Domain to be used by the view.
  /// @param mfunc Mapping function to transform view indices to container
  ///              gids.
  /// @param other View to copy from.
  //////////////////////////////////////////////////////////////////////
  linear_view_impl(view_container_type* vcont,
             domain_type const& dom,
             map_func_type const& mfunc,
             linear_view_impl const&)
    : base_type(vcont, dom, mfunc)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor that does not takes ownership over the passed
  ///        container.
  ///
  /// @param vcont Reference to the container used to forward the operations.
  /// @param dom Domain to be used by the view.
  /// @param mfunc Mapping function to transform view indices to container
  ///              gids.
  //////////////////////////////////////////////////////////////////////
  linear_view_impl(view_container_type const& vcont,
             domain_type const& dom,
             map_func_type const& mfunc = map_function())
    : base_type(vcont, dom, mfunc)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor that does not takes ownership over the passed
  ///        container.
  ///
  /// @param vcont Reference to the container used to forward the operations.
  /// @param dom Domain to be used by the view.
  /// @param mfunc Mapping function to transform view indices to container
  ///              gids.
  /// @param other View to copy from (ignored for linear_view_impl)
  //////////////////////////////////////////////////////////////////////
  linear_view_impl(view_container_type const& vcont,
             domain_type const& dom,
             map_func_type const& mfunc,
             linear_view_impl const&)
    : base_type(vcont, dom, mfunc)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a view that can reference all the elements of
  ///        the passed container. The view takes ownership of the container.
  ///
  /// @param vcont Pointer to the container used to forward the operations.
  //////////////////////////////////////////////////////////////////////
  linear_view_impl(view_container_type* vcont)
    : base_type(vcont, stapl::get_domain(*vcont))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a view that can reference all the elements of
  ///        the passed container.
  ///
  /// @param vcont Reference to the container used to forward the operations.
  //////////////////////////////////////////////////////////////////////
  linear_view_impl(view_container_type& vcont)
    : base_type(vcont, stapl::get_domain(vcont))
  { }
}; // class linear_view_impl


namespace result_of {

//////////////////////////////////////////////////////////////////////
/// @brief Helper struct to determine the one dimensional view type
///        over the given multidimensional @p View based on the
///        specified @p Traversal.
//////////////////////////////////////////////////////////////////////
template<typename View, typename... OptionalParams>
struct linear_view
{
  typedef linear_view_impl<View, OptionalParams...> type;
};

} // namespace result_of


namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Get the container for the linear view over given multidimensional
///        view.
///
/// A linear view is implemented as an @ref array_view with special
/// mapping function and container type obtained from @ref view_traits
/// for the original view (@see linear_view_impl).
///
/// If the container type of the linear view is the same as the container
/// type of the original view, then we may use it directly to construct
/// the linear array view. If not (like when the original view does not
/// conform to the <tt>V<C,D,F,Derived></tt> convention and view_traits
/// reflect the view type itself as the container), we need to construct
/// the container for the linear view anew from the original view, in
/// order to prevent temporary implicit conversion when the container is
/// passed to the linear view constructor.
///
/// This primary template is used in the first case, where we just return
/// a reference to the container of the provided view.
///
/// @tparam OriginalView  original multidimensional view to be linearized
/// @tparam LinearView    result of the linearization
//////////////////////////////////////////////////////////////////////
template<typename OriginalView, typename LinearView,
  bool = std::is_same<
    typename OriginalView::view_container_type,
    typename LinearView::view_container_type
  >::value
>
struct linear_view_container
{
  using container_type = typename LinearView::view_container_type;

  static container_type const& get(OriginalView const& view)
  {
    return view.container();
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Get the container for the linear view over given multidimensional
///        view.
///
/// Specialization for the case when the container type of the linear view
/// doesn't match the container type of the original view. In this case,
/// we need to heap-allocate the container and return a pointer to it, so
/// that it is not lost when the linear view is constructed and the linear
/// view takes responsibility for its deallocation.
///
/// @tparam OriginalView  original multidimensional view to be linearized
/// @tparam LinearView    result of the linearization
//////////////////////////////////////////////////////////////////////
template<typename OriginalView, typename LinearView>
struct linear_view_container<OriginalView, LinearView, false>
{
  using container_type = typename LinearView::view_container_type;

  static container_type* get(OriginalView const& view)
  {
    static_assert(is_view<OriginalView>::value, "View type expected.");
    return get_impl(view,
      typename std::is_same<container_type, OriginalView>::type{});
  }

private:
  static container_type* get_impl(OriginalView const& view, std::true_type)
  {
    return new container_type(view.container(), view.domain(), view.mapfunc());
  }

  static container_type* get_impl(OriginalView const& view, std::false_type)
  {
    return new container_type(view.container());
  }
};

}

//////////////////////////////////////////////////////////////////////
/// @brief Helper function to return a one dimensional view
///        over the given multidimensional @p view based on the
///        specified @p Traversal type.
//////////////////////////////////////////////////////////////////////
template<typename View, typename ...OptionalTraversal>
typename std::enable_if<dimension_traits<View>::type::value != 1,
  typename result_of::linear_view<View, OptionalTraversal...>::type
>::type
linear_view(View const& view, OptionalTraversal const&...)
{
  typedef typename compute_traversal_type<
    tuple_size<typename view_traits<View>::index_type>::value,
    OptionalTraversal...
  >::type                                                   traversal_type;

  typedef typename View::domain_type::size_type             size_type;
  typedef typename view_traits<View>::index_type            index_type;
  typedef typename view_traits<View>::map_function          map_func_type;

  typedef typename result_of::linear_view<
    View, OptionalTraversal...>::type                       result_t;
  typedef nd_reverse_linearize<index_type, traversal_type>  reverse_lin_t;
  typedef compose_func<map_func_type, reverse_lin_t>        comp_mapfunc_t;

  const size_type size = view.domain().dimensions();

  nd_linearize<index_type, traversal_type> mf(size);

  const size_t first = mf(view.domain().first());
  const size_t last  = mf(view.domain().last());

  return result_t(
    detail::linear_view_container<View, result_t>::get(view),
    indexed_domain<size_t>(first, last),
    comp_mapfunc_t::apply(view.mapfunc(), reverse_lin_t(size)));
}

//////////////////////////////////////////////////////////////////////
/// @brief Definition of @p linear_view helper function for the case
///        when the view being linearized is already one-dimensional.
///
/// Just returns the input view unchanged.
//////////////////////////////////////////////////////////////////////
template<typename View, typename ...OptionalTraversal>
typename std::enable_if<dimension_traits<View>::type::value == 1,
  View
>::type
linear_view(View const& view, OptionalTraversal const&...)
{
  return view;
}

#ifdef STAPL_DOCUMENTATION_ONLY

//////////////////////////////////////////////////////////////////////
/// @brief Defines a multi-dimensional array view.
///
/// Provides the operations that are commonly present in an array
/// (random access, iteration).
/// @tparam C Container type.
/// @tparam Dom Domain type. By default uses the same domain type
///             provided for the container.
/// @tparam Mapping function type
///    (default: identity mapping function)
/// @tparam Derived Type of the most derived class (default: itself)
/// @ingroup multi_array_view
//////////////////////////////////////////////////////////////////////
template<typename C,
         typename Dom = typename container_traits<C>::domain_type,
         typename MapFunc = f_ident<typename Dom::index_type>,
         typename Derived = use_default>
class multiarray_view;

#else

template<typename C, typename ...OptionalParams>
class multiarray_view;

#endif

namespace view_operations {

template <typename View>
struct compute_local_domain;

//////////////////////////////////////////////////////////////////////
/// @brief Helper function for computing the local domain that by default
/// is partition agnostic of the underlying container distribution.
/// Specializations of the class template addresses cases where we do better
/// given what we know about the distribution (ideally we would like the
/// local domain to be that of the elements stored on this affinity).
///
/// Specialization for the case where the multiarray_view over the multiarray.
/// In this case we ask the underlying base container about existing elements
/// in this affinity.
//////////////////////////////////////////////////////////////////////
template<int N, typename T, typename ...CArgs, typename ...Args>
struct compute_local_domain<
  multiarray_view<multiarray<N, T, CArgs...>, Args...>>
{
  using view_t      = multiarray_view<multiarray<N, T, CArgs...>, Args...>;
  using domain_type = typename view_t::domain_type;

  static std::vector<domain_type> apply(view_t const& view,
                                        runtime::location_id loc_id,
                                        std::size_t num_locs)
  {
    std::vector<domain_type> v;

    if (!domain_equal<N>::apply(view.domain(), view.container().domain()) ||
        num_locs != view.container().get_num_locations())
    {
      v.push_back(
        default_local_domain(std::true_type(), view, loc_id, num_locs));
      return v;
    }

    auto const& cm = view.container().distribution().container_manager();

    if (cm.size() == 0)
      v.push_back(domain_type());
    else
    {
      v.reserve(cm.size());

      for (auto&& bc : cm)
      {
        v.push_back(bc.domain());
      }
    }

    return v;
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Helper function for computing the local domain that by default
/// is partition agnostic of the underlying container distribution.
/// Specializations of the class template addresses cases where we do better
/// given what we know about the distribution (ideally we would like the
/// local domain to be that of the elements stored on this affinity).
///
/// Specialization for the case where the multiarray_view over the multiarray.
/// In this case we ask the underlying base container about existing elements
/// in this affinity.
//////////////////////////////////////////////////////////////////////
template<int N, typename T, typename ...CArgs, typename ...Args,
         typename Accessor>
struct compute_local_domain<
  multiarray_view<proxy<multiarray<N, T, CArgs...>, Accessor>, Args...>>
{
  using view_t      =
    multiarray_view<proxy<multiarray<N, T, CArgs...>, Accessor>, Args...>;

  using domain_type = typename view_t::domain_type;

  static std::vector<domain_type> apply(view_t const& view,
                                        runtime::location_id loc_id,
                                        std::size_t num_locs)
  {
    std::vector<domain_type> v;

    if (!domain_equal<N>::apply(view.domain(), view.container().domain()) ||
        num_locs != view.container().get_num_locations())
    {
      v.push_back(
        default_local_domain(std::true_type(), view, loc_id, num_locs));
      return v;
    }

    auto const& cm = view.container().distribution().container_manager();

    if (cm.size() == 0)
      v.push_back(domain_type());
    else
    {
      v.reserve(cm.size());

      for (auto&& bc : cm)
      {
        v.push_back(bc.domain());
      }
    }

    return v;
  }
};

} // namespace view_operations


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for when only container type parameter is specified.
//////////////////////////////////////////////////////////////////////
template<typename C>
struct view_traits<multiarray_view<C>>
  : default_view_traits<C,
      typename container_traits<C>::domain_type,
      f_ident<typename container_traits<C>::domain_type::index_type>,
      multiarray_view<C>>
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for when container and domain type parameters
///    are specified.
//////////////////////////////////////////////////////////////////////
template<typename C, typename D>
struct view_traits<multiarray_view<C, D>>
  : default_view_traits<C, D, f_ident<typename D::index_type>,
                        multiarray_view<C,D>>
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for when container, domain, and mapping function
///  type parameters are specified.
//////////////////////////////////////////////////////////////////////
template<typename C, typename D, typename F>
struct view_traits<multiarray_view<C, D, F>>
  : default_view_traits<C, D, F, multiarray_view<C, D, F>>
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for when all four type parameters are specified.
//////////////////////////////////////////////////////////////////////
template<typename C, typename D, typename F, typename Derived>
struct view_traits<multiarray_view<C, D, F, Derived>>
  : default_view_traits<C, D, F, Derived>
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Specialization ensures container transform for variadic
///  based optionals is used.
//////////////////////////////////////////////////////////////////////
template<typename OldC, typename ...OptionalParams, typename NewC>
struct cast_container_view<multiarray_view<OldC, OptionalParams...>, NewC>
  : new_cast_container_view<multiarray_view<OldC, OptionalParams...>, NewC>
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Specialization ensures container domain and mapping function
///   transform for variadic based optionals is used.
//////////////////////////////////////////////////////////////////////
template<typename C, typename ...OptionalParams, typename Dom, typename MF>
struct upcast_view<multiarray_view<C, OptionalParams...>, Dom, MF>
  : new_upcast_view<multiarray_view<C, OptionalParams...>, Dom, MF>
{ };


template<typename C, typename ...OptionalParams>
class multiarray_view
  : public core_view<
      C,
      typename view_traits<multiarray_view<C, OptionalParams...>>::domain_type,
      typename view_traits<multiarray_view<C, OptionalParams...>>::map_function
    >,
    public view_operations::readwrite<multiarray_view<C, OptionalParams...>>,
    public view_operations::multi_dimensional_subscript<
      typename view_traits<multiarray_view<C, OptionalParams...>>::derived_type
    >
{
public:
  STAPL_VIEW_REFLECT_TRAITS(multiarray_view)

  typedef index_type                                    dimensions_type;
  typedef typename dimension_traits<index_type>::type   dimension_type;
private:
  typedef core_view<C, domain_type, map_function> base_type;

public:
  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::array_view::array_view(view_container_type&,domain_type const&,map_func_type)
  //////////////////////////////////////////////////////////////////////
  multiarray_view(view_container_type& vcont,
                  domain_type const& dom,
                  map_function mfunc = map_function())
    : base_type(vcont,dom,mfunc)
  { }

  multiarray_view(view_container_type const& vcont,
                  domain_type const& dom,
                  map_function mfunc = map_function())
    : base_type(vcont,dom,mfunc)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::array_view::array_view(view_container_type*)
  //////////////////////////////////////////////////////////////////////
  multiarray_view(view_container_type* vcont)
    : base_type(vcont, stapl::get_domain(*vcont))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::array_view::array_view(view_container_type&)
  //////////////////////////////////////////////////////////////////////
  multiarray_view(view_container_type& vcont)
    : base_type(vcont, stapl::get_domain(vcont))
  { }

  multiarray_view(view_container_type const& vcont)
    : base_type(vcont, stapl::get_domain(vcont))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::array_view::array_view(view_container_type*,domain_type const&,map_func_type)
  //////////////////////////////////////////////////////////////////////
  multiarray_view(view_container_type* vcont,
                  domain_type const& dom,
                  map_function mfunc = map_function())
    : base_type(vcont,dom,mfunc)
  { }

  /////////////////////////////////////////////////////////////////
  /// @brief Copy constructor when the passed view is not the most
  ///        derived view.
  //////////////////////////////////////////////////////////////////////
  template<typename ...OtherOptionalParams>
  multiarray_view(multiarray_view<C, OtherOptionalParams...> const& other)
    : base_type(other.container(), other.domain(), other.mapfunc())
  { }

  multiarray_view(view_container_type& vcont,
                  domain_type const& dom,
                  map_function mfunc,
                  multiarray_view const&)
    : base_type(vcont,dom,mfunc)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief The length of each of the dimensions of the multiarray.
  //////////////////////////////////////////////////////////////////////
  dimensions_type dimensions(void) const
  {
    return this->domain().dimensions();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Update the underlying container to hold the specified
  ///        number of elements.  All previous information is lost.
  //////////////////////////////////////////////////////////////////////
  void resize(typename domain_type::size_type size)
  {
    this->incr_version();
    this->container().resize(size);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Update the underlying container to have the distribution
  /// specified.
  ///
  /// This function is primarily called when container instances in composed
  /// container instances are redistributed.
  ///
  /// @param dist_view Specification of the desired container distribution,
  /// including the distribution of any nested containers in the case of a
  /// composed container instance.
  //////////////////////////////////////////////////////////////////////
  template <typename DistSpecView>
  typename std::enable_if<
    is_distribution_view<DistSpecView>::value ||
    detail::has_is_composed_dist_spec<DistSpecView>::value>::type
  redistribute(DistSpecView const& dist_view)
  {
    this->incr_version();
    this->container().redistribute(dist_view);
  }


  //////////////////////////////////////////////////////////////////////
  /// @internal
  /// @brief use to examine this class
  /// @param msg your message (to provide context)
  //////////////////////////////////////////////////////////////////////
  void debug(char *msg=0)
  {
    std::cerr << "MULTI_ARRAY_VIEW " << this << " : ";
    if (msg) {
      std::cerr << msg;
    }
    std::cerr << "\n";
    base_type::debug();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Metafunction to compute the type of a deep slice on this view
  ///        container.
  ///
  /// @tparam Slices The indices to slice on this deep slice
  /// @tparam Fixed The type of element that will be used to specify the
  ///               fixed values. Typically a tuple of size |Slices|.
  //////////////////////////////////////////////////////////////////////
  template<typename Slices, typename Fixed>
  struct slice_type
  {
    using type =
      typename view_container_type::template slice_type<Slices, Fixed>::type;
  };

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a deep slice by slicing off dimensions specified
  ///        in Slices.
  ///
  /// @tparam Slices The indices to slice on this deep slice
  /// @tparam Fixed The type of element that will be used to specify the
  ///               fixed values. Typically a tuple of size |Slices|.
  //////////////////////////////////////////////////////////////////////
  template<typename Slices, typename Fixed>
  typename slice_type<Slices, Fixed>::type slice(Fixed const& fixed)
  {
    return this->container().template slice<Slices>(fixed);
  }

  std::vector<domain_type>
  local_domain(runtime::location_id loc_id, std::size_t num_locs) const
  {
    return view_operations::compute_local_domain<multiarray_view>::apply(
      *this, loc_id, num_locs);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief internal
  //////////////////////////////////////////////////////////////////////
  void define_type(typer& t)
  {
    t.base<base_type>(*this);
  }
}; // class multiarray_view

//////////////////////////////////////////////////////////////////////
/// @brief Helper function to construct a multiarray_view over the passed
///        container
///
/// @param ct Underlying container used for the multiarray_view.
/// @return A multiarray_view over the Container.
//////////////////////////////////////////////////////////////////////
template<typename Container>
multiarray_view<Container>
make_multiarray_view(Container const& ct)
{
  return multiarray_view<Container>(const_cast<Container&>(ct));
}

//////////////////////////////////////////////////////////////////////
/// @brief A linearization mapping function which accepts an initial
///        offset. This mapping function is used in the cases where a
///        @c multiarray_view is defined over an existing 1D view.
///
/// @tparam GID the GID type of the n-dimensional view.
//////////////////////////////////////////////////////////////////////
template <typename GID>
struct nd_linearize_with_offset
  : public nd_linearize<
      GID, typename default_traversal<tuple_size<GID>::value>::type>
{
  typedef nd_linearize<
    GID, typename default_traversal<tuple_size<GID>::value>::type> base_t;

  STAPL_IMPORT_DTYPE(base_t, size_type)
  STAPL_IMPORT_DTYPE(base_t, index_type)
  STAPL_IMPORT_DTYPE(base_t, gid_type)

  gid_type m_offset;

  nd_linearize_with_offset(gid_type offset, size_type const& sizes)
    : base_t(sizes),
      m_offset(offset)
  { }

  gid_type operator()(index_type const& gid) const
  {
    return base_t::operator()(gid) + m_offset;
  }

  template<typename RefGetter, typename... Indices>
  typename std::result_of<RefGetter(Indices...)>::type
  apply_get(RefGetter const& get_reference, Indices... indices) const
  {
    return get_reference(base_t::operator()(indices...) + m_offset);
  }

  void define_type(typer& t)
  {
    t.member(m_offset);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Class that provides basic types used by a multidimensional
///        view over a 1D view.
///
/// Provides the domain type, mapping function type, type of the
/// multidimensional view itself and its localized version where
/// the underlying container has been swapped out for the base
/// container. Also provides free-standing functions that return the
/// domain and mapping function instances associated with such a
/// multidimensional view.
///
/// @tparam View the original 1D view
/// @tparam Dims dimensions of the multidimensional view
//////////////////////////////////////////////////////////////////////
template<typename View, typename... Dims>
struct make_multiarray_view_traits
{
  static size_t constexpr dim = sizeof...(Dims);

  using domain_type = indexed_domain<std::size_t, dim,
    typename default_traversal<dim>::type>;
  using index_type = typename domain_type::index_type;
  using map_function = nd_linearize_with_offset<index_type>;

  using type = multiarray_view<View, domain_type, map_function>;

private:
  using view_container_type = typename underlying_container<View>::type;
  using base_container_type =
    typename view_container_type::distribution_type::base_container_type;

public:
  using localized_type =
    typename cast_container_view<type, base_container_type>::type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Given n dimensions as variadic arguments, creates an
  ///        n-dimensional domain.
  ///
  /// @param dims dimensions of the new domain
  ///
  /// @return an @ref indexed_domain
  //////////////////////////////////////////////////////////////////////
  static domain_type domain(Dims... dims)
  {
    return { std::forward_as_tuple(dims...) };
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Given a one-dimensional view and n dimensions as variadic
  ///        arguments, creates a mapping function that maps an
  ///        n-dimensional index to one-dimensional GID of the view
  ///        (starting at the first GID in the domain of the view).
  ///
  /// @param view the original one-dimensional view
  /// @param dims dimensions of the domain from which indices for
  ///             the mapping function can be drawn
  ///
  /// @return an instance of @ref nd_linearize_with_offset
  //////////////////////////////////////////////////////////////////////
  static map_function mapfunc(View const& view, Dims... dims)
  {
    return { view.domain().first(), std::forward_as_tuple(dims...) };
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Given a 1D view and n dimensions as variadic arguments,
///        creates an n-dimensional view that can be used to access
///        elements of the 1D view using nD indices
///        (using default linearization).
///
/// @param view original one-dimensional view
/// @param dim0 first dimension of the returned view
/// @param dims other dimensions of the returned view
///
/// @return a @ref multiarray_view over the provided 1D view
//////////////////////////////////////////////////////////////////////
template <typename View, typename D, typename... Dims>
typename make_multiarray_view_traits<View, D, Dims...>::type
make_multiarray_view(View const& view, D dim0, Dims... dims)
{
  static_assert(dimension_traits<View>::type::value == 1,
    "make_multiarray_view(view, dim0, dim1, ...) requires 1D view.");

  using traits = make_multiarray_view_traits<View, D, Dims...>;

  return {
    view, traits::domain(dim0, dims...), traits::mapfunc(view, dim0, dims...)
  };
}

//////////////////////////////////////////////////////////////////////
/// @brief Linearization of the multidimensional view over a 1D view.
/// @return the original 1D view
//////////////////////////////////////////////////////////////////////
template <typename View, typename D, typename... Dims>
View linear_view(
  typename make_multiarray_view_traits<View, D, Dims...>::type const& view)
{
  return view.container();
}

namespace metadata {

//////////////////////////////////////////////////////////////////////
/// @brief Traits class that specializes functionality of the coarsening
///        process for multidimensional views over 1D array_views.
///
/// The projection algorithm (@ref multiarray_view_over_array_view_projection)
/// relies on the fact that these views are logically partitioned along the
/// first dimension only.
///
/// @see multiarray_view_over_array_view_projection
//////////////////////////////////////////////////////////////////////
template <typename View, typename Domain>
struct coarsening_traits<
  multiarray_view<
    View, Domain, nd_linearize_with_offset<typename Domain::index_type>
  >
>
{
  using view_type = multiarray_view<
    View, Domain, nd_linearize_with_offset<typename Domain::index_type>
  >;

  template<typename P>
  struct construct_projection
  {
    using type = multiarray_view_over_array_view_projection<const view_type, P>;
  };
};

//////////////////////////////////////////////////////////////////////
/// @brief Traits class that specializes functionality of the coarsening
///        process for linear views over multidimensional views.
///
/// The default metadata projection is used for most types of
/// @p linear_view<multiarray_view<...>>, based on the underlying container
/// type. If the underlying container is 1D @ref array, we assume that
/// a @p multiarray_view<array_view<array>> has been created
/// (using <tt>make_multiarray_view(view, dim0, dim1, ...)</tt>), possibly with
/// additional multidimensional views on top of it (e.g. the slices view).
/// This means that @ref multiarray_view_over_array_view_projection has been
/// invoked in the metadata projection phase and a multidimensional metadata
/// container partitioned along the first dimension only is being passed to the
/// projection algorithm for the linear view. This allows us to use
/// @ref linearized_multiarray_view_over_array_view_projection for this
/// algorithm.
///
/// @see linearized_multiarray_view_over_array_view_projection
/// @see multiarray_view_over_array_view_projection
//////////////////////////////////////////////////////////////////////
template<typename View, typename... OptionalParams>
struct coarsening_traits<linear_view_impl<View, OptionalParams...>>
{
private:
  using underlying_container_type =
    typename underlying_container<View>::type;

  using view_type = linear_view_impl<View, OptionalParams...>;

  /// Base template - same implementation as in coarsening_traits.hpp
  template<typename P, typename = underlying_container_type>
  struct based_on_underlying_container_type
  {
    using view_container_type = typename view_type::view_container_type;

    using type = typename default_metadata_projection<
                    view_type, P,
                    is_container<view_container_type>::value,
                    is_segmented_view<view_type>::value,
                    is_invertible_view<view_type>::value
                  >::type;
  };

  /// Specialization for the case when the underlying container is a 1D array
  template<typename P, typename T, typename... OptParams>
  struct based_on_underlying_container_type<P, array<T, OptParams...>>
  {
    using type = linearized_multiarray_view_over_array_view_projection<
      const view_type, P>;
  };

public:
  template<typename P>
  struct construct_projection
  {
    using type = typename based_on_underlying_container_type<P>::type;
  };
};

} // namespace metadata

} // namespace stapl

#endif // STAPL_VIEWS_MULTIARRAY_VIEW_HPP
