/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_PACKING_HPP
#define STAPL_VIEWS_PACKING_HPP

#include <stapl/views/type_traits/is_view.hpp>
#include <stapl/views/store_in_frame.hpp>
#include <stapl/runtime.hpp>
#include <type_traits>
#include <utility>

namespace stapl {

template<typename View, typename Info, typename CID>
class mix_view;

template <typename C>
class repeat_view;

template <typename Slices, typename Container, typename... OptParams>
class slices_segmented_view;

template<typename View>
class package;

namespace view_impl {

template <typename T, int N, typename Policy, typename... OptionalDistribution>
struct counting_container;

} // namespace view_impl

template <typename Func, int n, typename... Distribution>
struct functor_container;

//////////////////////////////////////////////////////////////////////
/// @brief Class which abstract storage of container for @ref package,
///   storing a pointer for a @ref p_object container type and by value
///   for other container types (i.e., proxy).
//////////////////////////////////////////////////////////////////////
template<typename Container, bool = is_p_object<Container>::value,
         bool = is_view<Container>::value>
class package_container_storage
{
private:
  p_object_pointer_wrapper<Container> m_ct_ptr;

public:
  package_container_storage(Container* const ct_ptr)
    : m_ct_ptr(ct_ptr)
  { }

  Container* container_ptr(void) const
  {
    return m_ct_ptr;
  }

  void define_type(typer& t)
  {
    t.member(m_ct_ptr);
  }
};


template<typename Container>
class package_container_storage<Container, false, false>
{
private:
  Container m_ct;

public:
  package_container_storage(Container* const ct_ptr)
    : m_ct(*ct_ptr)
  { }

  Container* container_ptr(void) const
  {
    return new Container(m_ct);
  }

  void define_type(typer& t)
  {
    t.member(m_ct);
  }
};


//////////////////////////////////////////////////////////////////////
/// @name Specialization of package_container_storage for the
/// @ref counting_container / @ref functor_container, which are the containers
/// underlying a @ref counting_view / functor_view.
///
/// This specialization is required in order for the package specialization
/// for mix_view<View, Info, CID> to use @ref package_container_storage
/// instead of a raw pointer to the underlying container. This is because
/// the counting_container / functor_container itself isn't a p_object, but
/// one of its data members is currently a p_object.
//////////////////////////////////////////////////////////////////////
///@{
template<typename T, int N, typename Policy, typename... OptionalDistribution>
class package_container_storage<
  view_impl::counting_container<T, N, Policy, OptionalDistribution...>,
    false, false>
{
private:
  using ct_t = view_impl::counting_container<T, N, Policy,
    OptionalDistribution...>;

  ct_t* m_ct;

public:
  package_container_storage(ct_t* const ct_ptr)
    : m_ct(ct_ptr)
  { }

  ct_t* container_ptr(void) const
  {
    return m_ct;
  }

  void define_type(typer& t)
  {
    t.member(m_ct);
  }
};

template <typename Func, int n, typename... Distribution>
class package_container_storage<functor_container<Func, n, Distribution...>,
                                true, false>
{
private:
  using ct_t = functor_container<Func, n, Distribution...>;

  ct_t* m_ct;

public:
  package_container_storage(ct_t* const ct_ptr)
    : m_ct(ct_ptr)
  { }

  ct_t* container_ptr(void) const
  {
    return m_ct;
  }

  void define_type(typer& t)
  {
    t.member(m_ct);
  }
};
///@}

template<typename Container>
struct take_ownership
{
  using type = std::integral_constant<bool,
    is_view<Container>::value || !is_p_object<Container>::value
  >;
};

template <typename T, int N, typename Policy, typename... OptionalDistribution>
struct take_ownership<
  view_impl::counting_container<T,N,Policy,OptionalDistribution...>
>
  : std::false_type
{ };

template <typename Func, int n, typename... Distribution>
struct take_ownership<functor_container<Func, n, Distribution...>>
  : std::false_type
{ };

template<typename View>
class package_container_storage<View, true, true>
  : public package<View>
{
  using container_t = typename View::view_container_type;

  View* container_ptr_impl(std::true_type /*take_ownership*/) const
  {
    return new View(this->m_ct_storage.container_ptr(), this->m_dom,
                    this->m_mf);
  }

  View* container_ptr_impl(std::false_type /*take_ownership*/) const
  {
    return new View(*this->m_ct_storage.container_ptr(), this->m_dom,
                    this->m_mf);
  }

public:
  package_container_storage(View* const vw_ptr)
    : package<View>(*vw_ptr)
  { }

  View* container_ptr() const
  {
    // If the container is a view itself or is not a p_object, pass the pointer
    // to the constructor so the view being returned takes ownership of the new
    // object being created in the package_container_storage specialization for
    // views or non-p_objects.
    return container_ptr_impl(typename take_ownership<container_t>::type());
  }

  void define_type(typer& t)
  {
    t.base<package<View>>(*this);
  }
};


template <typename PG, typename Dom, typename MF, typename Derived>
class hierarchical_graph_view;

template <typename PG, typename Dom, typename MF, typename Derived>
class package_container_storage<hierarchical_graph_view<PG, Dom, MF, Derived>,
                                true, true>
  : package<hierarchical_graph_view<PG, Dom, MF, Derived>>
{
private:
  using View = hierarchical_graph_view<PG, Dom, MF, Derived>;
  using container_t = typename View::view_container_type;

  size_t m_level;

  View* container_ptr_impl(std::true_type /*take_ownership*/) const
  {
    return new View(m_level, this->m_ct_storage.container_ptr(), this->m_dom,
                    this->m_mf);
  }

  View* container_ptr_impl(std::false_type /*take_ownership*/) const
  {
    return new View(m_level, *this->m_ct_storage.container_ptr(), this->m_dom,
                    this->m_mf);
  }

public:
  package_container_storage(View* const vw_ptr)
    : package<View>(*vw_ptr), m_level(vw_ptr->level())
  { }

  View* container_ptr() const
  {
    // If the container is a view itself or is not a p_object, pass the pointer
    // to the constructor so the view being returned takes ownership of the new
    // object being created in the package_container_storage specialization for
    // views or non-p_objects.
    return container_ptr_impl(typename take_ownership<container_t>::type());
  }

  void define_type(typer& t)
  {
    t.base<package<hierarchical_graph_view<PG, Dom, MF, Derived>>>(*this);
    t.member(m_level);
  }
};


template <typename Slices, typename Container, typename... OptParams>
class package_container_storage<
  slices_segmented_view<Slices, Container, OptParams...>, true, true>
  : public package<
      slices_segmented_view<Slices, Container, OptParams...>>
{
  using view_t =
    slices_segmented_view<Slices, Container, OptParams...>;

public:
  package_container_storage(view_t* const vw_ptr)
    : package<view_t>(*vw_ptr)
  { }

  view_t* container_ptr() const
  {
    return new view_t(view_impl::store_in_frame(), this->m_ct_storage());
  }

  void define_type(typer& t)
  {
    t.base<package<view_t>>(*this);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Class which deconstructs a view into its constituent parts so that
///   it can be serialized. For general views, extract and store the
///   container pointer, domain, and mapping function. If it's a mix view,
///   also extract and store the id and metadata info members.
//////////////////////////////////////////////////////////////////////
template<typename View>
class package
{
protected:
  typedef typename View::view_container_type container_t;
  typedef typename View::domain_type         domain_t;
  typedef typename View::map_func_type       mapfunc_t;

  package_container_storage<container_t>    m_ct_storage;
  domain_t                                  m_dom;
  mapfunc_t                                 m_mf;

  package& operator=(package const&) = delete;

  View unpack(std::true_type /*take_ownership*/) const
  {
    return View(m_ct_storage.container_ptr(), m_dom, m_mf);
  }

  View unpack(std::false_type /*take_ownership*/) const
  {
    return View(*(m_ct_storage.container_ptr()), m_dom, m_mf);
  }

public:
  typedef View                               result_type;

  package(View& v)
    : m_ct_storage(v.get_container()),
      m_dom(v.domain()),
      m_mf(v.mapfunc())
  { }

  View operator()(void) const
  {
    // If the container is a view itself or is not a p_object, pass the pointer
    // to the constructor so the view being returned takes ownership of the new
    // object being created in the package_container_storage specialization for
    // views or non-p_objects.
    return unpack(typename take_ownership<container_t>::type());
  }

  void define_type(typer& t)
  {
    t.member(m_ct_storage);
    t.member(m_dom);
    t.member(m_mf);
  }
};

template <typename Slices, typename Container, typename... OptParams>
class package<
  slices_segmented_view<Slices, Container, OptParams...>>
{
protected:
  using view_t =
    slices_segmented_view<Slices, Container, OptParams...>;

  using container_t = Container;

  package_container_storage<container_t>    m_ct_storage;

  package& operator=(package const&) = delete;

  view_t unpack(std::true_type take_ownership) const
  {
    return view_t(view_impl::store_in_frame(), m_ct_storage());
  }

  view_t unpack(std::false_type take_ownership) const
  {
    return view_t(view_impl::store_in_frame(), m_ct_storage());
  }

public:
  using result_type = view_t;

  package(view_t& v)
    : m_ct_storage(v.container().get_view())
  { }

  view_t operator()(void) const
  {
    // If the container is a view itself or is not a p_object, pass the pointer
    // to the constructor so the view being returned takes ownership of the new
    // object being created in the package_container_storage specialization for
    // views or non-p_objects.
    using take_container_ownership_t = std::integral_constant<bool,
      is_view<container_t>::value || !is_p_object<container_t>::value
    >;

    return unpack(take_container_ownership_t());
  }

  void define_type(typer& t)
  {
    t.member(m_ct_storage);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of package for mix_view.
//////////////////////////////////////////////////////////////////////
template<typename View, typename Info, typename CID>
class package<mix_view<View, Info, CID>>
{
private:
  typedef typename View::view_container_type container_t;
  typedef typename View::domain_type         domain_t;
  typedef typename View::map_func_type       mapfunc_t;
  typedef CID                                index_t;

  package_container_storage<container_t>     m_ct_ptr;
  domain_t                                   m_dom;
  mapfunc_t                                  m_mf;
  index_t                                    m_id;
  Info                                       m_md;

public:
  typedef mix_view<View, Info, CID> result_type;

  package(result_type& v)
    : m_ct_ptr(v.get_container()), m_dom(v.domain()), m_mf(v.mapfunc()),
      m_id(v.get_id()), m_md(v.get_metadata())
  { }

  result_type operator()(void) const
  {
    return result_type(m_ct_ptr.container_ptr(),
                       m_dom, m_mf, m_id, m_md,
                       typename take_ownership<container_t>::type());
  }

  void define_type(typer& t)
  {
    t.member(m_ct_ptr);
    t.member(m_dom);
    t.member(m_mf);
    t.member(m_id);
    t.member(m_md);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of package for mix_view over @ref repeat_view
///   Must be treated differently due to fake container underlying that
///   view.
//////////////////////////////////////////////////////////////////////
template<typename Container, typename Info, typename CID>
class package<mix_view<repeat_view<Container>, Info, CID>>
{
private:
  typedef repeat_view<Container>     View;

  package<repeat_view<Container>>    m_vw_pkg;
  CID                                m_id;
  Info                               m_md;

public:
  typedef mix_view<repeat_view<Container>, Info, CID> result_type;

  package(result_type& v)
    : m_vw_pkg(static_cast<View&>(v)), m_id(v.get_id()), m_md(v.get_metadata())
  { }

  result_type operator()(void) const
  {
    return result_type(m_vw_pkg(), m_id, m_md);
  }

  void define_type(typer& t)
  {
    t.member(m_vw_pkg);
    t.member(m_id);
    t.member(m_md);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of package for repeated view.  Only store
///  the underlying, repeatedly referenced value.  Then, reconstruct
///  using this value on destination.
//////////////////////////////////////////////////////////////////////
template<typename Container>
class package<repeat_view<Container>>
{
private:
  using value_type  = typename Container::value_type;

  package_container_storage<value_type> m_value;

public:
  using result_type = repeat_view<Container>;

  package(result_type& v)
    : m_value(std::make_shared<value_type>(value_type(
       v.get_element(typename result_type::index_type()))).get())
  { }

  result_type operator()(void) const
  {
    return result_type(new Container(*(m_value.container_ptr())));
  }

  void define_type(typer& t)
  {
    t.member(m_value);
  }
}; // class package<repeat_view<Container>>


//////////////////////////////////////////////////////////////////////
/// @brief Support class of @ref transport_packager polymorphic functor. If
///   the specified argument is a view, store its constituent parts in
///   @ref package and return the package. Otherwise, just return the argument.
//////////////////////////////////////////////////////////////////////
template<typename View, bool = is_view<View>::value>
struct transporter_packager_impl
{
  typedef package<View> type;

  static type apply(View& view)
  {
    return package<View>(view);
  }
};


template<typename View>
struct transporter_packager_impl<View, false>
{
  typedef View type;

  static View& apply(View& view)
  {
    return view;
  }
};


/////////////////////////////////////////////////////////////////////
/// @brief Functor used by to create serialized views for an out of
///   PARAGRAPH task placement via @ref executor_rmi.
/////////////////////////////////////////////////////////////////////
struct transporter_packager
{
  template<typename Signature>
  struct result;

  template<typename VS>
  struct result<transporter_packager(VS)>
    : transporter_packager_impl<
        typename std::remove_reference<VS>::type
      >
  { };

  template<typename VS>
  typename result<transporter_packager(VS)>::type
  operator()(VS& vs) const
  {
    return transporter_packager_impl<
      typename std::remove_reference<VS>::type
    >::apply(vs);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Support class of @ref transporter_unpackager polymorphic functor.
/// If the specified argument is a @ref package, invoke the function operator.
/// Otherwise, just return the argument
//////////////////////////////////////////////////////////////////////
template<typename View>
struct transporter_unpackager_impl
{
  typedef View type;

  static type& apply(View& view)
  {
    return view;
  }

  static type& apply(View&& view)
  {
    return view;
  }
};


template<typename View>
struct transporter_unpackager_impl<package<View> >
{
  typedef typename package<View>::result_type type;

  static type apply(package<View>& pkg)
  {
    return pkg();
  }

  static type apply(package<View>&& pkg)
  {
    return pkg();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor used by @ref transporter to unpack views sent out
///   of PARAGRAPH gang by the task placement policy.
//////////////////////////////////////////////////////////////////////
struct transporter_unpackager
{
  template<typename VS>
  typename transporter_unpackager_impl<
             typename std::remove_reference<VS>::type
           >::type
  operator()(VS&& vs) const
  {
    return transporter_unpackager_impl<
             typename std::remove_reference<VS>::type
           >::apply(std::forward<VS>(vs));
  }
};

} // namespace stapl

#endif
