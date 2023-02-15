/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_MIX_VIEW_HPP
#define STAPL_VIEWS_MIX_VIEW_HPP

#include <type_traits>

#include <stapl/views/type_traits/upcast.hpp>
#include <stapl/views/segmented_view_base.hpp>
#include <stapl/runtime.hpp>
#include <stapl/utility/loc_qual.hpp>
#include <stapl/containers/base/bc_base.hpp>
#include <stapl/views/type_traits/has_vertex_property.hpp>

namespace stapl {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Helper class used to cast given view instance to one over
///   the provided base container instance.
///
/// This primary template is used when the view is derived from a
/// @ref segmented_view. It constructs the cast view instance by
/// re-creating the underlying view (hierarchy) on heap, using reference
/// to the base container at the bottom-most level, and passing the
/// pointer to the result alongside the reference to the
/// original segmented_view to the segmented_view constructor that takes
/// ownership of the reconstructed view (hierarchy).
///
/// @tparam CastView  Type of the cast view to be instantiated.
/// @sa segmented_view(Other const& other, Container* view)
//////////////////////////////////////////////////////////////////////
template<typename CastView,
  bool = is_view<CastView>::value, bool = is_segmented_view<CastView>::value>
class fast_view_builder
{
  using component_type = underlying_container_t<CastView>;

public:
  /// Creates the cast view by recursively heap-allocating the underlying
  /// views until the bottom-most container is hit, in which case
  /// the reference to the provided component (base container) is used.
  template<typename View>
  static CastView create(View const& v, component_type* c)
  {
    static_assert(is_segmented_view<View>::value,
      "Mismatched view/cast view types.");

    using next_level_builder = fast_view_builder<
      typename CastView::view_container_type::view_container_type
    >;

    return CastView{
      v,
      next_level_builder::create_new_or_get_component(v.container().view(), c)
    };
  }

  template<typename View>
  static CastView* create_new_or_get_component(View const& v, component_type* c)
  {
    static_assert(is_segmented_view<View>::value,
      "Mismatched view/cast view types.");

    using next_level_builder = fast_view_builder<
      typename CastView::view_container_type::view_container_type
    >;

    return new CastView{
      v,
      next_level_builder::create_new_or_get_component(v.container().view(), c)
    };
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper class used to cast given view instance to one over
///   the provided base container instance.
///
/// This specialization is used when the view is not derived from a
/// @ref segmented_view. It dispatches the cast view creation based
/// on whether the underlying views form a view composition that
/// has to be preserved (in which case the fast_view_builder is called
/// recursively as in the primary template) or not (in which case the
/// view is created directly with appropriately flattened mapping
/// function).
///
/// @tparam CastView  Type of the cast view to be instantiated.
/// @sa preserve_composition
//////////////////////////////////////////////////////////////////////
template<typename CastView>
class fast_view_builder<CastView, true, false>
{
  using component_type = underlying_container_t<CastView>;

  /// @brief Construct on stack the view over the heap-allocated container at
  /// the next level of a composition that is to be preserved (or over the base
  /// container), using the original domain and mapping function.
  template<typename View>
  static CastView
  create_impl(View const& v, component_type* c, std::true_type)
  {
    using next_level_builder = fast_view_builder<
      typename CastView::view_container_type
    >;

    return CastView{
      next_level_builder::create_new_or_get_component(v.container(), c),
      v.domain(), v.mapfunc()
    };
  }

  /// @brief Heap allocate the view over the heap-allocated container at the
  /// next level of a composition that is to be preserved (or over the base
  /// container), using the original domain and mapping function.
  template<typename View>
  static CastView*
  create_new_impl(View const& v, component_type* c, std::true_type)
  {
    using next_level_builder = fast_view_builder<
      typename CastView::view_container_type
    >;

    return new CastView{
      next_level_builder::create_new_or_get_component(v.container(), c),
      v.domain(), v.mapfunc()
    };
  }

  /// @brief Construct on stack the flattened view over the base
  /// container, using the original domain and appropriate composition
  /// of the mapping functions of intermediate views.
  template<typename View>
  static CastView
  create_impl(View const& v, component_type* c, std::false_type)
  {
    static_assert(std::is_same<
                    typename CastView::view_container_type, component_type
                  >::value, "Mismatched underlying container types.");

    using view_caster_t = cast_container_view<View, component_type>;

    return CastView{*c, v.domain(), view_caster_t().mapfunc(v)};
  }

  /// @brief Heap allocate the flattened view over the base
  /// container, using the original domain and appropriate composition
  /// of the mapping functions of intermediate views.
  template<typename View>
  static CastView*
  create_new_impl(View const& v, component_type* c, std::false_type)
  {
    static_assert(std::is_same<
                    typename CastView::view_container_type, component_type
                  >::value, "Mismatched underlying container types.");

    using view_caster_t = cast_container_view<View, component_type>;

    return new CastView{*c, v.domain(), view_caster_t().mapfunc(v)};
  }

public:
  template<typename View>
  static CastView create(View const& v, component_type* c)
  {
    return create_impl(v, c, typename preserve_composition<View>::type{});
  }

  template<typename View>
  static CastView* create_new_or_get_component(View const& v, component_type* c)
  {
    return create_new_impl(v, c, typename preserve_composition<View>::type{});
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Terminator of the recursive fast_view_builder instantiation
///   when the bottom-most container of the original view has been hit.
///
/// Returns the reference to the base container (which has been passed
/// through via the component pointer), so that the created cast view
/// doesn't take ownership of it.
//////////////////////////////////////////////////////////////////////
template<typename BaseContainer>
class fast_view_builder<BaseContainer, false, false>
{
public:
  template<typename Container>
  static BaseContainer&
  create_new_or_get_component(Container const&, BaseContainer* comp)
  {
    return *comp;
  }
};

} // namespace detail


//////////////////////////////////////////////////////////////////////
/// @brief A view that has the same behavior as the given @c View type,
///        but uses a base container as underlying container to provide
///        fast access over the data.
///
/// Such a view is constructed when the task is going to be executed in
/// this location and all the views required for the task access their
/// data locally.
///
/// @tparam View  @ref mix_view over the view that is to be transformed
///         to one over the underlying base container (component pointer
///         of the @p mix_view)
//////////////////////////////////////////////////////////////////////
template<typename View>
struct localized_view
  : public cast_container_view<
      typename View::base_type, typename View::component_type
    >::type
{
private:
  /// @brief Cast the actual view wrapped by the @ref mix_view to one over a
  /// base container
  using base_t = typename cast_container_view<
    typename View::base_type, typename View::component_type
  >::type;

public:
  localized_view(View const& v)
    : base_t(detail::fast_view_builder<base_t>::create(v, v.get_component()))
  { }
}; // struct localized_view


template<typename View, typename Info, typename CID>
class mix_view;

namespace view_impl {

template <typename C, typename PS, typename MFG, typename SVC>
class view_container;

}

// disable fast views for now
template<typename V, typename... Params>
class multiarray_view;

template<typename C, typename PS, typename MFG, typename SVC, typename Info,
         typename CID, typename... MParams>
struct localized_view<
         mix_view<
           multiarray_view<
             view_impl::view_container<C,PS,MFG,SVC>,
             MParams...
           >,
           Info,CID
         >
       >
  : public multiarray_view<view_impl::view_container<C,PS,MFG,SVC>,MParams...>
{
  typedef multiarray_view<view_impl::view_container<
    C,PS,MFG,SVC
  >,MParams...> base_type;

  template<typename V>
  localized_view(V const& v)
    : base_type(v)
  { }
};


namespace view_impl {

template <typename T>
struct helper_get_container
{
  static
  typename T::view_container_type&
  apply(T const& v)
  {
    return v.container();
  }
};


template <template <typename,typename,typename,typename> class V,
          typename C, typename A, typename D, typename F, typename Derived>
struct helper_get_container<V<proxy<C, A>, D, F, Derived> >
{
  typedef proxy<C, A> container_type;

  static
  container_type* apply(V<proxy<C, A>, D, F, Derived> const& v)
  {
    return new container_type(v.container());
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Determine location id of the data referenced by given view.
///
/// This overload is invoked for nested view types (view over another view or a
/// @ref segmented_view).
//////////////////////////////////////////////////////////////////////
template<typename View>
location_type get_container_location(View const* vw,
  typename std::enable_if<
    is_view<View>::value
    and (
      is_view<typename View::view_container_type>::value
      or std::is_base_of<
           view_container_base, typename View::view_container_type
         >::value
    )
  >::type* = 0)
{
  return get_container_location(vw->get_container());
}

//////////////////////////////////////////////////////////////////////
/// @brief Determine location id of the data referenced by given view.
///
/// This overload is invoked for a view over a p_object (that the underlying
/// container is a p_object with the method @p get_location_id() is
/// guaranteed by this method being called only from either
/// @ref mix_view::is_local_helper(std::true_type) or the final recursion step
/// of the previous @p get_container_location overload).
//////////////////////////////////////////////////////////////////////
template<typename View>
location_type get_container_location(View const* vw,
  typename std::enable_if<
    is_view<View>::value
    and not is_view<typename View::view_container_type>::value
    and not std::is_base_of<
              view_container_base, typename View::view_container_type
            >::value
  >::type* = 0)
{
  return vw->container().get_location_id();
}

//////////////////////////////////////////////////////////////////////
/// @brief Determine location id of given container.
//////////////////////////////////////////////////////////////////////
template<typename Cont>
location_type get_container_location(Cont const* ct,
  typename std::enable_if<not is_view<Cont>::value>::type* = 0)
{
  return ct->get_location_id();
}

//////////////////////////////////////////////////////////////////////
/// @brief Functor that checks if the underlying container of given view
///        is a p_object.
///
/// This version is instantiated for nested view types (view over another view
/// or a @ref segmented_view).
//////////////////////////////////////////////////////////////////////
template<typename View,
         bool = is_view<typename View::view_container_type>::value>
struct view_container_has_gethandle_type
{
  using type = typename view_container_has_gethandle_type<
                 typename View::view_container_type
               >::type;
};

//////////////////////////////////////////////////////////////////////
/// @brief Functor that checks if given container is a p_object.
///
/// This version is instantiated for the @ref view_container of the
/// @ref segmented_view.
//////////////////////////////////////////////////////////////////////
template<typename Container,
         bool = std::is_base_of<view_container_base, Container>::value>
struct container_has_gethandle_type
{
  using type = typename view_container_has_gethandle_type<Container>::type;
};

//////////////////////////////////////////////////////////////////////
/// @brief Functor that checks if the underlying container of given
///        non-nested view is a p_object.
//////////////////////////////////////////////////////////////////////
template<typename View>
struct view_container_has_gethandle_type<View, false>
{
  using view_container = typename View::view_container_type;
  using type = typename container_has_gethandle_type<view_container>::type;
};

//////////////////////////////////////////////////////////////////////
/// @brief Functor that checks if given container is a p_object.
///
/// This version is instantiated for containers other than the
/// @ref view_container.
//////////////////////////////////////////////////////////////////////
template<typename Container>
struct container_has_gethandle_type<Container, false>
{
  using type = typename is_p_object<Container>::type;
};


BOOST_MPL_HAS_XXX_TRAIT_DEF(custom_is_local_check)


//////////////////////////////////////////////////////////////////////
/// @brief Checks whether the value type of the the container or the
/// vertex property type (if it is defined) is a stapl container,
/// disabling localization attempts so as to avoid unneeded overhead
/// both at compile and runtime.
//////////////////////////////////////////////////////////////////////
template<typename View,
         bool b1 = is_container<typename View::value_type>::value,
         bool b2 = has_vertex_property<View>::value>
struct check_disable_localization
  : boost::mpl::false_
{ };


template<typename View, bool B2>
struct check_disable_localization<View, true, B2>
  : boost::mpl::true_
{ };


template<typename View>
struct check_disable_localization<View, false, true>
  : is_container<typename View::vertex_property>::type
{ };

} // namespace view_impl


//////////////////////////////////////////////////////////////////////
/// @brief Defines a view that behaves as the given @c View type but
///        has information about data locality specified by the given
///        locality information (@c Info).
///
/// @tparam View Type of view that this view is based on.
/// @tparam Info Locality information type.
/// @tparam CID Type of the subview index
//////////////////////////////////////////////////////////////////////
template<typename View, typename Info, typename CID>
class mix_view
  : public View
{
public:
  typedef typename View::domain_type                       domain_type;
  typedef typename View::map_func_type                     map_func_type;

  typedef typename std::remove_pointer<
    typename Info::component_type
  >::type                                                  component_type;

  typedef View                                             base_type;
  typedef CID                                              index_type;

  /// @brief Defines the type of the resulting view when the base
  ///        container is used instead of the @c View's underlying
  ///        container.
  typedef localized_view<mix_view>                             fast_view_type;

  using disable_localization = view_impl::check_disable_localization<View>;

private:
public:
  index_type      m_id;
  Info            m_md_info;

public:
  mix_view(void)
    : m_id(),
      m_md_info()
  { }

  mix_view(mix_view const& other)
    : View(other),
      m_id(),
      m_md_info(other.m_md_info)
  { }

  mix_view(mix_view const& other, typename View::view_container_type* vcont)
    : View(vcont, other.domain(), other.mapfunc()),
      m_id(other.m_id),
      m_md_info(other.m_md_info)
  { }

  // Constructor used for reconstructing a mix_view over another view during
  // view unpacking. The mix_view needs to take ownership of the wrapped view,
  // so that it can be disposed of properly from the unpacking call site.
  mix_view(typename View::view_container_type* vcont,
           domain_type const& dom,
           map_func_type mfunc,
           index_type const& id,
           Info const& md_info,
           std::true_type /*take_ownership*/)
    : View(vcont, dom, mfunc),
      m_id(id),
      m_md_info(md_info)
  { }

  mix_view(typename View::view_container_type* vcont,
           domain_type const& dom,
           map_func_type mfunc,
           index_type const& id,
           Info const& md_info,
           std::false_type take_ownership = std::false_type())
    : View(*vcont, dom, mfunc),
      m_id(id),
      m_md_info(md_info)
  { }

  mix_view(View const& vcont,
           domain_type const& dom,
           map_func_type mfunc,
           index_type const& vid,
           Info const& md_info)
    : View(view_impl::helper_get_container<View>::apply(vcont), dom, mfunc),
      m_id(vid),
      m_md_info(md_info)
  { }

  mix_view(base_type const& base_view,
           index_type const& vid,
           Info const& md_info)
    : View(base_view),
      m_id(vid),
      m_md_info(md_info)
  { }

  component_type* get_component(void) const
  {
    return m_md_info.component();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Determine if the data that this view references is local
  ///        (if the methods invoked have the the same location as
  ///        the referenced data).
  ///
  /// @todo Currently there is not specialization for comparing
  ///       locations on different gangs.
  //////////////////////////////////////////////////////////////////////
  bool is_local(void) const
  {
    return is_local_helper(
      std::integral_constant<
        bool, view_impl::has_custom_is_local_check<fast_view_type>::value>(),
      typename view_impl::view_container_has_gethandle_type<View>::type()
    );
  }

  index_type get_id(void) const
  {
    return m_id;
  }

  Info const& get_metadata(void) const
  {
    return m_md_info;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the location where the associated data is stored,
  ///        qualified based on the validity of the component.
  ///
  /// @todo Remapping of locations on different gangs is needed here.
  //////////////////////////////////////////////////////////////////////
  std::pair<location_type, loc_qual>
  get_preferred_location(void) const
  {
    if (m_md_info.get_component() != nullptr)
      return std::make_pair(m_md_info.location(), LQ_CERTAIN);
    else
      return std::make_pair(m_md_info.location(), LQ_DONTCARE);
  }

  void define_type(typer& t)
  {
    t.base<base_type>(*this);
    t.member(m_id);
    t.member(m_md_info);
  }

private:
  /////////////////////////////////////////////////////////////////////
  /// @brief Specialization when the @ref localized_view specialization of the
  /// view has defined a custom local check.
  /////////////////////////////////////////////////////////////////////
  template<typename B>
  bool is_local_helper(std::true_type, B) const
  {
    return fast_view_type::is_local(*this);
  }

  /////////////////////////////////////////////////////////////////////
  /// @brief Specialization when the container is a p_object and no custom
  /// local check has been defined.
  /// @todo Check of container as the view element can be statically resolved.
  //////////////////////////////////////////////////////////////////////
  bool is_local_helper(std::false_type, std::true_type) const
  {
    if (is_nested_container<typename View::value_type>::value)
      return false;

    return ( ( view_impl::get_container_location(static_cast<View const*>(this))
                 == m_md_info.location() )
             && (m_md_info.component() != nullptr)
             && (get_affinity() == m_md_info.affinity()) );
  }

  /////////////////////////////////////////////////////////////////////
  /// @brief General case, based on the validity of the component reference.
  //////////////////////////////////////////////////////////////////////
  bool is_local_helper(std::false_type, std::false_type) const
  {
    if (is_container<typename View::value_type>::value)
      return false;

    return (m_md_info.component() != nullptr)
            && (get_affinity() == m_md_info.affinity());
  }
}; // class mix_view

template<typename View, typename Info, typename CID, typename NewC>
struct cast_container_view<mix_view<View, Info, CID>, NewC>
  : public cast_container_view<View, NewC>
{ };

} // namespace stapl

#endif // STAPL_VIEWS_MIX_VIEW_HPP
