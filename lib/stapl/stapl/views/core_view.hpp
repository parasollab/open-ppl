/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_CORE_VIEW_HPP
#define STAPL_VIEWS_CORE_VIEW_HPP

#ifdef _STAPL
 #include <stapl/utility/has_member_function.hpp>
 #include <stapl/runtime.hpp>
 #include <stapl/utility/loc_qual.hpp>
 #include <stapl/utility/tuple.hpp>
 #include <stapl/views/view_traits.hpp>
 #include <stapl/views/segmented_view_base.hpp>
 #include <stapl/views/type_traits/is_segmented_view.hpp>
#endif

#include <stapl/views/base_view.hpp>
#include <stapl/views/operations/local_domain.hpp>

#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

#include <functional>
#include <iostream>
#include <type_traits>

namespace stapl {

namespace view_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Class to store the container* and provide reference counting
///   via shared_ptr, and determine when a container should be destroyed
///   for views which have taken ownership of the container.
///   boost::shared_ptr is being used in place of std::shared_ptr in order to
///   minimize the number of conflicts caused by ADL. Performance was shown to
///   be comparable.
//////////////////////////////////////////////////////////////////////
template<typename Container>
class object_holder
{
  using sptr_type = boost::shared_ptr<Container>;

  union
  {
    Container* m_ptr;
    sptr_type  m_sptr;
  };

  bool m_b_own_container;

protected:
  object_holder(void)
   : m_ptr(nullptr),
     m_b_own_container(false)
  { }

  object_holder(object_holder const& other)
  {
    m_b_own_container = other.m_b_own_container;

    if (m_b_own_container)
      new (&m_sptr) sptr_type(other.m_sptr);
    else
      m_ptr  = other.m_ptr;
  }

  explicit object_holder(Container* ct)
    : m_sptr(boost::shared_ptr<Container>(ct)),
      m_b_own_container(true)
  { }

  explicit object_holder(Container const& ct)
    : m_ptr(std::addressof(const_cast<Container&>(ct))),
      m_b_own_container(false)
  { }

  object_holder& operator=(object_holder const& rhs)
  {
    if (&rhs != this)
    {
      if (m_b_own_container)
        m_sptr.~sptr_type();

      m_b_own_container = rhs.m_b_own_container;

      if (m_b_own_container)
        new (&m_sptr) sptr_type(rhs.m_sptr);
      else
        m_ptr = rhs.m_ptr;
    }

    return *this;
  }

  ~object_holder()
  {
    if (m_b_own_container)
      m_sptr.~sptr_type();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the container's pointer
  //////////////////////////////////////////////////////////////////////
  Container* container_ptr(void) const
  {
    stapl_assert((m_b_own_container && m_sptr) || m_ptr,
                 "object_holder::container_ptr found NULL pointer");

    return m_b_own_container ? m_sptr.get() : m_ptr;
  }

#ifdef _STAPL
public:
  bool owned(void) const noexcept
  {
    return m_b_own_container && m_sptr.unique();
  }

  void define_type(typer& t)
  {
    if (m_b_own_container)
      t.member(m_sptr);
    else
      t.member(m_ptr);

    t.member(m_b_own_container);
  }
#endif
}; // class object_holder

template <typename MapFunc>
struct mapfunc_types_helper
{
  typedef typename MapFunc::index_type                index_type;
  typedef typename MapFunc::gid_type                  gid_type;
};

template <typename Signature>
struct mapfunc_types_helper<std::function<Signature> >
{
  typedef typename std::function<Signature>::argument_type index_type;
  typedef typename std::function<Signature>::result_type   gid_type;
};


//////////////////////////////////////////////////////////////////////
/// @name Validators for versioning.
///
/// Templates allow p_objects to be versioned and non_p_objects,
/// like stl objects, to ignore versioning
//////////////////////////////////////////////////////////////////////
///@{

//////////////////////////////////////////////////////////////////////
/// @brief Primary validator template.
/// @tparam T Type of the object to be validated.
//////////////////////////////////////////////////////////////////////
template<typename T,
  bool = is_p_object<T>::value,
  bool = is_view<T>::value,
  bool = std::is_base_of<view_container_base, T>::value >
struct validator
{
  static bool apply(T const& t, size_t version)
  {
    return t.version() == version;
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Validator for versioning of STAPL views.
///
/// In case of nested views, checks if versions of all containers down
/// the hierarchy match the top-most version passed in the argument.
///
/// @tparam T Type of the view to be validated.
//////////////////////////////////////////////////////////////////////
template<typename T>
struct validator <T, true, true, false>
{
  static constexpr bool apply(T const& view, size_t version) noexcept
  {
    return validator<typename T::view_container_type>::apply(view.container(),
                                                             version);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Validator for versioning of STAPL @ref view_impl::view_container.
///
/// Checks if the view whose segments are managed by the @p view_container
/// is valid. In cases when the view pointer is not assigned in view_container,
/// we just compare against the view_container's version.
///
/// @tparam T Type of the @p view_container to be validated.
//////////////////////////////////////////////////////////////////////
template<typename T, bool is_view>
struct validator <T, true, is_view, true>
{
  static constexpr bool apply(T const& view_cont, size_t version) noexcept
  {
    return view_cont.get_view() == nullptr ?
      version == view_cont.version() :
      validator<typename T::view_view_container_type>::apply(
        view_cont.container(), view_cont.get_view()->version());
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Validator for versioning of objects that are not p_objects.
///
/// Always returns true (valid).
///
/// @tparam T Type of the object to be validated.
//////////////////////////////////////////////////////////////////////
template<typename T, bool is_view_container>
struct validator <T, false, false, is_view_container>
{
  static constexpr bool apply(T const&, size_t) noexcept
  {
    return true;
  }
};
///@}

//////////////////////////////////////////////////////////////////////
/// @brief Versioner for versioning. Templates allow p_objects to be
///   versioned and non_p_objects, like stl objects, to ignore versioning
/// @tparam T Container type.
//////////////////////////////////////////////////////////////////////
template<typename T, bool = is_p_object<T>::value >
struct versioner
{
  static size_t apply(T const& t)
  {
    return t.version();
  }

  static size_t apply(T* t)
  {
    return t->version();
  }
};

template<typename T>
struct versioner <T, false>
{
  static constexpr size_t apply(T const&) noexcept
  {
    return 0;
  }

  static constexpr size_t apply(T*) noexcept
  {
    return 0;
  }
};

} // namespace view_impl


#ifdef _STAPL

namespace view_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Boolean type metafunction checking whether template is both
///  a view and whether its container type is a proxy.
///
///  Reflects index type to use as return value for @p index_of().
///  gcc evaluates the return type even if @p enable_if disables the
///  function signature.
//////////////////////////////////////////////////////////////////////
template<typename View, bool = is_view<View>::value>
struct is_view_of_proxy
  : std::false_type
{ };

template<typename View>
struct is_view_of_proxy<View, true>
  : std::integral_constant<
      bool, is_proxy<typename view_traits<View>::container>::value>
{
  typedef decltype(proxy_core_access::accessor(
    std::declval<View>().container()).index())       index_type;
};

//////////////////////////////////////////////////////////////////////
/// @brief Type metafunction returning true_type if the provided type
///  is either @ref p_object or a proxy to a @ref p_object.
//////////////////////////////////////////////////////////////////////
template<typename Container>
struct is_p_object_backed
  : is_p_object<Container>
{ };


template<typename Container, typename Accessor>
struct is_p_object_backed<proxy<Container, Accessor>>
  : is_p_object<Container>
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Static functor redirecting locality queries to container
///  if it is a @ref p_object.  Otherwise return LQ_DONTCARE.
//////////////////////////////////////////////////////////////////////
template<typename Container, bool = is_p_object_backed<Container>::value>
struct query_locality
{
  template<typename Index>
  static
  locality_info apply(Container& c, Index const& index)
  { return c.locality(index); }
};


template<typename Container>
struct query_locality<Container, false>
{
  template<typename Index>
  static
  locality_info apply(Container const&, Index const&)
  { return LQ_DONTCARE; }
};

} // namespace view_impl


//////////////////////////////////////////////////////////////////////
/// @brief Alias template defining @p enable_if condition for proxy
///  related operations on views of nested containers.
//////////////////////////////////////////////////////////////////////
template<typename View>
using ViewProxyEnabler =
  typename std::enable_if<view_impl::is_view_of_proxy<View>::value>::type*;

//////////////////////////////////////////////////////////////////////
/// @brief Returns the associated index of the element referenced by
///        the given proxy.  Specialization for views of nested
///        containers.
//////////////////////////////////////////////////////////////////////
template<typename View>
typename view_impl::is_view_of_proxy<View>::index_type
index_of(View const& view, ViewProxyEnabler<View> = nullptr)
{
  return proxy_core_access::accessor(view.container()).index();
}


template <typename View, bool isSegmentedView = is_segmented_view<View>::value>
struct get_proxy_index
{
  auto operator()(View const& view) const
    ->decltype(index_of(view))
  {
    return index_of(view);
  }
};


template <typename View>
struct get_proxy_index<View, true>
{
  auto operator()(View const& view) const -> decltype(
    get_proxy_index<typename View::view_container_type::view_container_type>()(
      view.container().view()))
  {
    using view_t = typename View::view_container_type::view_container_type;
    return get_proxy_index<view_t>()(view.container().view());
  }
};


template<typename View>
bool is_null_reference(View const& view, ViewProxyEnabler<View> = nullptr)
{
  return proxy_core_access::accessor(view.container()).is_null();
}



#endif

//////////////////////////////////////////////////////////////////////
/// @brief Main class to define a pView, composed of a reference to a
///        collection of elements, a domain and a mapping function.
///
/// @tparam C Container type.
/// @tparam Dom Domain type.
/// @tparam MapFunc Mapping function type.
//////////////////////////////////////////////////////////////////////
template <typename C, typename Dom, typename MapFunc >
class core_view
  : public base_view,
    public view_impl::object_holder<C>
{
  typedef view_impl::mapfunc_types_helper<MapFunc>    mf_type_helper;
public:
  typedef C                                           view_container_type;
  typedef Dom                                         domain_type;
  typedef MapFunc                                     map_func_type;
  typedef typename mf_type_helper::index_type         index_type;
  typedef typename mf_type_helper::gid_type           gid_type;
  typedef std::size_t                                 size_type;

private:
  typedef view_impl::object_holder<C>                 ct_holder_t;

  /// Domain of the view.
  domain_type            m_domain;

  /// Mapping function used to transform indices to container's gids.
  map_func_type          m_mapfunc;
  size_t                 m_version;

public:
  core_view(void)
    : m_version(0)
  { }

  core_view& operator=(core_view const&) = default;
  core_view(core_view&&)                 = default;

  //////////////////////////////////////////////////////////////////////
  /// @brief core_view constructor based on another view.
  ///
  /// This constructor creates a view from another view.
  /// When we coarsen to base containers there is no versioning.
  ///
  /// @param other The other view to copy from.
  //////////////////////////////////////////////////////////////////////
  core_view(core_view const& other)
    : ct_holder_t(other),
      m_domain(other.m_domain),
      m_mapfunc(other.m_mapfunc),
      m_version(other.version())
  { }


  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor based on a container pointer. The views takes
  ///        ownership over the passed underlying container.
  ///
  /// @param vcont  pointer to the container used to forward the operations.
  /// @param dom  domain to be used by the view.
  /// @param mfunc mapping function to transform view indices to
  ///              container gids.
  //////////////////////////////////////////////////////////////////////
  core_view(view_container_type* vcont,
            domain_type const& dom,
            map_func_type mfunc = MapFunc())
    : ct_holder_t(vcont),
      m_domain(dom),
      m_mapfunc(mfunc)
  {
    m_version = view_impl::versioner<view_container_type>::apply(vcont);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief core_view constructor based on a container reference.
  ///
  /// This constructor creates a view that doesn't take ownership over
  /// the container.
  ///
  /// @param vcont reference to the container used to forward the operations.
  /// @param dom domain to be used by the view.
  /// @param mfunc mapping function to transform view indices to container
  ///              gids.
  //////////////////////////////////////////////////////////////////////
  core_view(view_container_type const& vcont,
            domain_type const& dom,
            map_func_type mfunc = MapFunc())
    : ct_holder_t(vcont),
      m_domain(dom),
      m_mapfunc(mfunc)
  {
    m_version = view_impl::versioner<view_container_type>::apply(vcont);
  }

  //////////////////////////////////////////////////////////////////////
  /// @internal
  /// @brief  Validates view and aborts if view is invalid. Only
  ///         pobject containers are validated (When we coarsen to base
  ///         containers there is no validation.)
  //////////////////////////////////////////////////////////////////////
  bool validate(void) const
  {
    if (view_impl::validator<C>::apply(this->container(), m_version))
    {
      return true;
    }
    else
    {
      abort("View validation failed. View/container out of sync.");
      return false;
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @internal
  /// @brief  Returns whether the view is valid or not.
  /// The difference from validate() is that this method doesn't abort
  /// if the view is invalid.
  //////////////////////////////////////////////////////////////////////
  bool is_valid(void) const
  {
    return view_impl::validator<C>::apply(this->container(), m_version);
  }

  //////////////////////////////////////////////////////////////////////
  /// @internal
  /// @brief  Increment the version number
  //////////////////////////////////////////////////////////////////////
  void incr_version(void)
  {
    ++m_version;
  }

  //////////////////////////////////////////////////////////////////////
  /// @internal
  /// @brief Returns the current version number
  //////////////////////////////////////////////////////////////////////
  size_t version(void) const
  {
    return m_version;
  }

  //////////////////////////////////////////////////////////////////////
  /// @todo Const qualifying the returned reference produces compiler errors
  //////////////////////////////////////////////////////////////////////
  view_container_type /*const*/* get_container(void) const
  {
    return this->container_ptr();
  }

  view_container_type& container(void)
  {
    return *this->container_ptr();
  }

  //////////////////////////////////////////////////////////////////////
  /// @todo Const qualifying the returned reference produces compiler errors
  //////////////////////////////////////////////////////////////////////
  view_container_type /*const*/& container(void) const
  {
    return *this->container_ptr();
  }

  domain_type const& domain(void) const
  {
    return m_domain;
  }

  domain_type& domain(void)
  {
    return m_domain;
  }

  domain_type get_domain(void)
  {
    return m_domain;
  }

  void set_domain(domain_type const& dom)
  {
    m_domain = dom;
  }

  map_func_type const& mapfunc(void) const
  {
    return m_mapfunc;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of elements referenced for the view.
  //////////////////////////////////////////////////////////////////////
  size_type size(void) const
  {
    return m_domain.size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns true if the view does not reference any element.
  //////////////////////////////////////////////////////////////////////
  bool empty(void) const
  {
    return m_domain.empty();
  }

#ifdef _STAPL
public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Determine the locality of the given index.  Common use is
  ///  in task placement within a @ref paragraph.
  /// @return A @ref locality_info object describing the locality of the
  ///   given element.
  //////////////////////////////////////////////////////////////////////
  locality_info locality(index_type const& index) const
  {
    return view_impl::query_locality<view_container_type>::apply(
      this->container(), this->mapfunc()(index));
  }

  rmi_handle::reference nested_locality(index_type const& index)
  {
    typedef typename container_traits<C>::value_type v_t;
    return this->container().apply_get(this->mapfunc()(index),
      boost::bind<rmi_handle::reference>(
        [](v_t const& v) {
         /// @todo this const cast should be removed.
         return const_cast<v_t&>(v).get_rmi_handle();
       } , _1));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns true if all the elements referenced for the view are
  ///        local.
  //////////////////////////////////////////////////////////////////////
  bool is_local(void) const
  {
    return false;
  }

  std::vector<domain_type>
  local_domain(runtime::location_id loc_id, std::size_t num_locs) const
  {
    return view_operations::compute_local_domain<core_view>::apply(
      *this, loc_id, num_locs);
  }

  //////////////////////////////////////////////////////////////////////
  /// @internal
  //////////////////////////////////////////////////////////////////////
  void define_type(typer& t)
  {
    t.base<ct_holder_t>(*this);
    t.member(m_domain);
    t.member(m_mapfunc);
    t.member(m_version);
  }

  //////////////////////////////////////////////////////////////////////
  /// @internal
  /// @brief use to examine this class
  /// @param msg your message (to provide context)
  //////////////////////////////////////////////////////////////////////
  void debug(char *msg=0)
  {
    std::cerr << "CORE_VIEW " << this << " : ";
    if (msg) {
      std::cerr << msg;
    }
    std::cerr << std::endl;
    m_domain.debug();
  }

  //////////////////////////////////////////////////////////////////////
  /// @internal
  /// @brief used by paragraph
  //////////////////////////////////////////////////////////////////////
  void pre_execute(void)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @internal
  /// @brief used by paragraph
  //////////////////////////////////////////////////////////////////////
  void post_execute(void)
  { }

#endif // ifdef _STAPL

}; // class core_view

} // namespace stapl

#endif // STAPL_VIEWS_CORE_VIEW_HPP
