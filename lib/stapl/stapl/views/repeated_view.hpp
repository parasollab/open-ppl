/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_REPEATED_VIEW_HPP
#define STAPL_VIEWS_REPEATED_VIEW_HPP

#include <stapl/domains/infinite.hpp>
#include <stapl/views/array_ro_view.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/views/metadata/metadata_entry.hpp>
#include <stapl/views/metadata/projection/generic.hpp>
#include <stapl/views/metadata/coarsening_traits.hpp>
#include <stapl/utility/use_default.hpp>
#include <stapl/views/type_traits/upcast.hpp>
#include <stapl/views/operations/multi_dimensional_subscript.hpp>
#include <stapl/skeletons/map_reduce.hpp>

namespace stapl {

namespace metadata {

template<typename C, typename MD>
class infinite_container;

} // namespace metadata


namespace view_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Functor to return a locality metadata that knows how to
///        return metadata for an infinite set of elements.
/// @tparam C container that represents an infinite collection of elements
//////////////////////////////////////////////////////////////////////
template<typename C, int N = 1>
struct infinite_locality_metadata
{
  using domain_type   =
    typename std::conditional< N == 1, infinite, infinite_nd<N>>::type;

  using dom_info_type = metadata_entry<domain_type, C*>;
  using md_cont_type  = metadata::infinite_container<C, dom_info_type>;
  using index_type    = typename domain_type::index_type;
  using return_type   = std::pair<bool, md_cont_type*>;

  return_type operator()(C* cont) const
  {
    return std::make_pair(false, new md_cont_type(cont));
  }
}; // struct infinite_locality_metadata


template<typename T, int N = 1, bool ReadOnly = false,
         bool IsView = is_view<T>::value>
struct repeat_container;


template<typename Domain, typename T, int N, bool IsReadOnly, bool IsView>
struct repeat_container_distribution
  : p_object
{
  using domain_type    = Domain;
  using component_type = repeat_container<T, N, IsReadOnly, IsView>*;
  using dom_info_type  = metadata_entry<domain_type, component_type>;

  future<dom_info_type> metadata_at(size_t)
  {
    return make_ready_future(dom_info_type(
             typename dom_info_type::cid_type(), domain_type(), 0,
             LQ_DONTCARE, invalid_affinity_tag, this->get_rmi_handle(), 0));
  }
}; // struct repeat_container_distribution


//////////////////////////////////////////////////////////////////////
/// @brief Defines the container to represent an infinite collection
///        of elements
/// @tparam T element type to repeat
//////////////////////////////////////////////////////////////////////
template<typename T, int N, bool IsReadOnly, bool IsView>
struct repeat_container
{
  using value_type        = T;
  using reference         = T&;
  using const_reference   = T const&;
  using domain_type       =
    typename std::conditional<N == 1, infinite, infinite_nd<N>>::type;
  using dimensions_type   = typename dimension_traits<domain_type>::type;
  using dimension_type    = dimensions_type;
  using index_type        = typename domain_type::index_type;
  using gid_type          = index_type;
  using cid_type          = index_type;
  using size_type         = typename domain_type::size_type;
  using loc_dist_metadata = infinite_locality_metadata<repeat_container, N>;
  using distribution_type =
    repeat_container_distribution<domain_type, T, N, IsReadOnly, IsView>;

  value_type         m_data;
  distribution_type  m_dist;

  repeat_container(void)
    : m_data()
  { }

  repeat_container(value_type val)
    : m_data(std::move(val))
  { }

  distribution_type& distribution(void)
  { return m_dist; }

  reference operator[](index_type)
  {
    return m_data;
  }

  value_type get_element(index_type) const
  {
    return m_data;
  }

  //////////////////////////////////////////////////////////////////////
  /// @internal
  //////////////////////////////////////////////////////////////////////
  size_t version(void) const
  {
    return 0;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies a function f to the element specified by the GID,
  /// and returns the result.
  /// @param gid The GID of the element.
  /// @param f The Functor to apply on the element.
  /// @return The result of applying the functor to the element.
  //////////////////////////////////////////////////////////////////////
  template<typename F>
  typename F::result_type
  apply_get(gid_type const& gid, F const& f)
  {
    return f(m_data);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the number of elements associated with this
  ///        container, in this case is infinite (maximum value of \c
  ///        size_t)
  //////////////////////////////////////////////////////////////////////
  size_type size(void) const
  {
    return domain_type().size();
  }

  domain_type domain(void) const
  {
    return domain_type();
  }

  using task_placement_dontcare = std::true_type;

  constexpr locality_info locality(size_t) const
  {
    return LQ_DONTCARE;
  }

  //////////////////////////////////////////////////////////////////////
  /// @internal
  //////////////////////////////////////////////////////////////////////
  void define_type(typer& t)
  {
    abort("repeat_container packing attempted");
  }
}; // struct repeat_container


//////////////////////////////////////////////////////////////////////
/// @brief Specialization when repeated element is a view.
///
/// Fetches a local copy of the distributed when size is below a given
/// threshold.  If this local copy is populated, use it during fast view
/// transformations during algorithm execution.
///
/// @tparam Q element type to repeat
/// @todo Generalize threshold function and add intelligence / adaptivity.
/// @todo Remove mutable keyword from @p m_local_copy when views over
/// const vectors are supported.
//////////////////////////////////////////////////////////////////////
template<typename Q>
struct repeat_container<Q, 1, true, true>
{
private:
  using val_t               = typename Q::value_type;

public:
  using fast_container_type = std::vector<val_t>;
  using value_type          = Q;
  using reference           = Q&;
  using const_reference     = Q const&;
  using domain_type         = infinite;
  using dimensions_type     = typename dimension_traits<domain_type>::type;
  using dimension_type      = dimensions_type;
  using index_type          = typename domain_type::index_type;
  using gid_type            = index_type;
  using cid_type            = index_type;
  using size_type           = typename domain_type::size_type;
  using loc_dist_metadata   = infinite_locality_metadata<repeat_container, 1>;
  using distribution_type   =
    repeat_container_distribution<domain_type, Q, 1, true, true>;

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Extract all elements of sub range of view to be repeated
  ///   and copy them in a std vector, to be concatenated with other
  ///   ranges' values before being broadcast to all locations.
  //////////////////////////////////////////////////////////////////////
  struct ro_zipper
  {
    using result_type = fast_container_type;

    template<typename View>
    fast_container_type
    operator()(View const& vw) const
    {
      fast_container_type result(vw.size());
      std::copy(vw.begin(), vw.end(), result.begin());
      return result;
    }
  };


  //////////////////////////////////////////////////////////////////////
  /// @brief Concatenates two vectors together, gathering all elements
  ///   of the view to be repeated together.
  //////////////////////////////////////////////////////////////////////
  struct ro_reducer
  {
    template<typename R1, typename R2>
    fast_container_type
    operator()(R1&& r1, R2&& r2) const
    {
      fast_container_type result = r1;
      result.insert(result.end(), r2.begin(), r2.end());
      return result;
    }
  };


  using pmg_t =
     decltype(skeletons::make_paragraph_skeleton_manager(
       skeletons::sink_value<fast_container_type>(
         skeletons::compose(
           skeletons::zip_reduce<1>(ro_zipper(), ro_reducer()),
           skeletons::broadcast_to_locs())),
       skeletons::execution_params<fast_container_type>(
         default_coarsener())));

  using tg_t     = paragraph<typename pmg_t::scheduler_type, pmg_t, Q>;
  using handle_t = decltype(std::declval<tg_t>()((int) 0));

  mutable bool                      m_b_local_copy_arrived;
  mutable fast_container_type       m_local_copy;
  value_type                        m_data;
  distribution_type                 m_dist;
  mutable boost::optional<handle_t> m_local_copy_handle;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Return a reference to the local copy, if initialized.
  //////////////////////////////////////////////////////////////////////
  fast_container_type& fast_container(void) const
  {
    stapl_assert(m_local_copy_handle, "Attempted to use invalid local copy");

    if (!m_b_local_copy_arrived)
    {
      m_b_local_copy_arrived = true;
      m_local_copy           = *m_local_copy_handle;
    }

    return m_local_copy;
  }


public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Initializes copy of view to repeated and based on a threshold
  /// predicate (for now fundamental element type and view.size() < 10000),
  /// fire off an asynchronous @ref paragraph to populate a local copy of
  /// the view.  Avoids communications for reads later.
  ///
  /// @todo Remove the call to @ref fast_container, which forces fulfillment
  /// of the local copy handle from the paragraph, allowing it to lazily be
  /// called during the fast view transformation in the paragraph using this
  /// repeated view instance.  Doing so now, causes hang / assertion in the
  /// runtime.
  //////////////////////////////////////////////////////////////////////
  repeat_container(Q const& view)
    : m_b_local_copy_arrived(false),
      m_data(view)

  {
    if (std::is_fundamental<typename Q::value_type>::value
        && view.size() < 10000)
    {
      auto pmg =
        skeletons::make_paragraph_skeleton_manager(
          skeletons::sink_value<fast_container_type>(
            skeletons::compose(
              skeletons::zip_reduce<1>(ro_zipper(), ro_reducer()),
              skeletons::broadcast_to_locs())),
          skeletons::execution_params<fast_container_type>(
            default_coarsener()));

      using tg_t =
        paragraph<typename decltype(pmg)::scheduler_type, decltype(pmg), Q>;

      m_local_copy_handle =
        boost::in_place<handle_t>(
          proxy_core_access::accessor((tg_t(pmg, view))()));

      fast_container();
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Queried by localization process during @ref paragraph execution.
  /// If the local_copy is will be used, then the handle (proxy) to the value
  /// to be fulfilled by a @ref paragraph has been initialized.
  //////////////////////////////////////////////////////////////////////
  bool is_local(void) const
  { return m_local_copy_handle.is_initialized(); }

  distribution_type& distribution(void)
  { return m_dist; }

  reference operator[](index_type)
  { return m_data; }

  value_type get_element(index_type) const
  { return m_data; }

  //////////////////////////////////////////////////////////////////////
  /// @internal
  //////////////////////////////////////////////////////////////////////
  size_t version(void) const
  { return 0; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies a function f to the element specified by the GID,
  /// and returns the result.
  /// @param gid The GID of the element.
  /// @param f The Functor to apply on the element.
  /// @return The result of applying the functor to the element.
  //////////////////////////////////////////////////////////////////////
  template<typename F>
  typename F::result_type
  apply_get(gid_type const& gid, F const& f)
  { return f(m_data); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the number of elements associated with this
  ///        container, in this case is infinite (maximum value of \c
  ///        size_t)
  //////////////////////////////////////////////////////////////////////
  size_type size(void) const
  { return domain_type().size(); }

  domain_type domain(void) const
  { return domain_type(); }

  using task_placement_dontcare = std::true_type;

  constexpr locality_info locality(size_t) const
  { return LQ_DONTCARE; }

  //////////////////////////////////////////////////////////////////////
  /// @internal
  //////////////////////////////////////////////////////////////////////
  void define_type(typer& t)
  { abort("repeat_container packing attempted"); }
}; // struct repeat_container

} // namespace view_impl


template <typename C>
class repeat_view;


template<typename C>
struct view_traits<repeat_view<C>>
  : default_view_traits<C,
      typename container_traits<C>::domain_type,
      f_ident<typename container_traits<C>::domain_type::index_type>,
      repeat_view<C>>
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Specialization ensures container transform for variadic
///  based optionals is used.
//////////////////////////////////////////////////////////////////////
template<typename OldC, typename NewC>
struct cast_container_view<repeat_view<OldC>, NewC>
  : new_cast_container_view<repeat_view<OldC>, NewC>
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Specialization ensures container domain and mapping function
///   transform for variadic based optionals is used.
//////////////////////////////////////////////////////////////////////
template<typename C, typename Dom, typename MF>
struct upcast_view<repeat_view<C>, Dom, MF>
  : new_upcast_view<repeat_view<C>, Dom, MF>
{ };


////////////////////////////////////////////////////////////////////////
/// @brief A view that can provide infinite copies of the same element.
/// @tparam T The type of element to be repeated.
////////////////////////////////////////////////////////////////////////
template<typename C>
class repeat_view
  : public core_view<
      C,
      typename view_traits<repeat_view<C>>::domain_type,
      typename view_traits<repeat_view<C>>::map_function>,
    public view_operations::read<repeat_view<C>>,
    public std::conditional<
      dimension_traits<C>::type::value == 1,
      view_operations::subscript<repeat_view<C>>,
      view_operations::multi_dimensional_subscript<repeat_view<C>>
    >::type,
    public view_operations::sequence<
      repeat_view<C>, detail::view_iterator<repeat_view<C>>>
{
public:
  STAPL_VIEW_REFLECT_TRAITS(repeat_view)

private:
  using sequence_op_type =
    view_operations::sequence<repeat_view, detail::view_iterator<repeat_view>>;

  using base_type        = core_view<C, domain_type, map_function>;

public:
  using dimensions_type  = typename dimension_traits<C>::type;
  using dimension_type   = dimensions_type;

  using iterator         = typename sequence_op_type::iterator;
  using const_iterator   = typename sequence_op_type::const_iterator;

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor used to pass ownership of the container to the view.
  ///
  /// @param vcont pointer to the container used to forward the operations.
  /// @param dom domain to be used by the view.
  /// @param mfunc mapping function to transform view indices to container
  ///        gids.
  //////////////////////////////////////////////////////////////////////
  repeat_view(view_container_type* vcont,
              domain_type dom     = domain_type(),
              map_func_type mfunc = map_func_type())
    : base_type(vcont, std::move(dom), std::move(mfunc))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor that does not takes ownership over the passed
  ///        container.
  ///
  /// @param vcont reference to the container used to forward the operations.
  /// @param dom domain to be used by the view.
  /// @param mfunc mapping function to transform view indices to container
  ///        gids.
  //////////////////////////////////////////////////////////////////////
  repeat_view(view_container_type const& vcont,
              domain_type dom,
              map_func_type mfunc = map_func_type())
    : base_type(vcont, std::move(dom), std::move(mfunc))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor that does not takes ownership over the passed
  ///        container.
  ///
  /// @param vcont reference to the container used to forward the operations.
  /// @param dom domain to be used by the view.
  /// @param mfunc mapping function to transform view indices to container
  ///        gids.
  //////////////////////////////////////////////////////////////////////
  repeat_view(view_container_type const& vcont,
              domain_type dom,
              map_func_type mfunc,
              repeat_view const&)
    : base_type(vcont, std::move(dom), std::move(mfunc))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @internal
  /// @brief use to examine this class
  /// @param msg your message (to provide context)
  //////////////////////////////////////////////////////////////////////
  void debug(char *msg=0)
  {
    std::cerr << "REPEAT_VIEW " << this << " : ";
    if (msg) {
      std::cerr << msg;
    }
    std::cerr << std::endl;
    base_type::debug();
  }

  //////////////////////////////////////////////////////////////////////
  /// @internal
  //////////////////////////////////////////////////////////////////////
  void define_type(typer& t)
  {
    stapl_assert(false, "repeat_view is not meant to be shipped");
  }
}; // class repeat_view


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for @ref mix_view of repeat_view when a local
/// copy might be fetched.  Customized localization to be based on whether
/// the local copy was fetched.  Localize to a repeat view over this local
/// copy.
//////////////////////////////////////////////////////////////////////
template<typename T, typename Info, typename CID>
struct localized_view<
  mix_view<
    repeat_view<view_impl::repeat_container<T, 1, true, true>>, Info, CID>>
  : public repeat_view<
      view_impl::repeat_container<
        array_view<typename view_impl::repeat_container<
          T, 1, true, true>::fast_container_type>>>
{
  using custom_is_local_check = std::true_type;

  using base_container_type = view_impl::repeat_container<T, 1, true, true>;

  using base_view_type = mix_view<repeat_view<base_container_type>, Info, CID>;

  using fast_container_type = typename base_container_type::fast_container_type;

  using new_base_container_type =
    view_impl::repeat_container<array_view<fast_container_type>>;

  using base_type = repeat_view<new_base_container_type>;

  static bool is_local(base_view_type const& v)
  { return v.container().is_local(); }

  localized_view(base_view_type const& v)
   : base_type(new new_base_container_type(array_view<fast_container_type>(
       v.container().fast_container())))
  { }
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper function to construct a @ref repeat_view
/// @tparam data The object provide access to, regardless of provided index.
//////////////////////////////////////////////////////////////////////
template<typename T>
repeat_view<view_impl::repeat_container<T>>
make_repeat_view(T const& data)
{
  return repeat_view<view_impl::repeat_container<T>>(
    new view_impl::repeat_container<T>(data));
}


template<typename T>
repeat_view<view_impl::repeat_container<T, 1, true>>
make_repeat_ro_view(T const& data)
{
  return repeat_view<view_impl::repeat_container<T, 1, true>>(
    new view_impl::repeat_container<T, 1, true>(data));
}


//////////////////////////////////////////////////////////////////////
/// @brief Helper function to construct a @ref repeat_view
/// @tparam N dimensionality to create repeat view with.
/// @tparam T element type to repeat
/// @return a repeat view with @p N dimensions
//////////////////////////////////////////////////////////////////////
template<int N, typename T>
repeat_view<view_impl::repeat_container<T, N>>
make_repeat_view_nd(T const& data)
{
  return repeat_view<view_impl::repeat_container<T, N>>(
    new view_impl::repeat_container<T, N>(data));
}

} // namespace stapl

#endif // STAPL_VIEWS_REPEATED_VIEW_HPP
