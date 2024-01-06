/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_TRANSFORM_VIEW_HPP
#define STAPL_VIEWS_TRANSFORM_VIEW_HPP

#include <boost/utility/result_of.hpp>

#include <stapl/views/metadata/mix_view.hpp>
#include <stapl/views/type_traits/retype.hpp>
#include <stapl/views/common_view.hpp>

#include <stapl/utility/invoke_arg.hpp>

#include <iostream>

namespace stapl {

template<typename View, typename Functor>
class transform_view;

//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref view_traits for transform_view.
/// @see transform_view.
//////////////////////////////////////////////////////////////////////
template<typename View, typename Functor>
struct view_traits<transform_view<View, Functor> >
  : public view_traits<View>
{
private:
  typedef typename inherit_retype<View,
             transform_view<View,Functor> >::type                base_t;
  typedef typename view_traits<base_t>::reference                base_ref_t;

public:
  typedef typename boost::result_of<Functor(base_ref_t)>::type   reference;
  typedef reference                                              value_type;
};


namespace view_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Specialization of gid_accessor over transform_view to allow
///        application of the transformation functor over the element
///        referenced.
///
/// @todo Verify if this specialization can be implemented by
/// inheritance instead of copy the whole code.
//////////////////////////////////////////////////////////////////////
template<typename View, typename Functor>
class gid_accessor<transform_view<View, Functor> >
{
public:
  typedef transform_view<View, Functor>            view_t;
  typedef typename view_traits<view_t>::index_type index_type;
  typedef typename view_traits<view_t>::container  container;
  typedef typename view_traits<view_t>::value_type value_type;
  typedef typename container::gid_type             gid_type;

protected:
  template <typename Derived, typename A, typename C, typename R, typename D>
  friend class iterator_facade;

  container* m_container;
  gid_type   m_gid;
  Functor    m_func;

public:
  bool is_null(void) const
  {
    return m_container == NULL;
  };

  void define_type(typer& t)
  {
    t.member(m_container);
    t.member(m_gid);
  }

  explicit gid_accessor(void)
    : m_container(NULL),
      m_gid()
  { }

  gid_accessor(null_reference const&)
    : m_container(NULL),
      m_gid()
  { }

  gid_accessor(gid_accessor const& other)
    : m_container(other.m_container), m_gid(other.m_gid)
  { }

  gid_accessor(view_t* view, index_type gid)
    : m_container(&view->container()),
      m_gid(view->mapfunc()(gid)),
      m_func(view->func())
  { }

  gid_accessor(const view_t* view, index_type const& gid)
    : m_container(&const_cast<view_t*>(view)->container()),
      m_gid(view->mapfunc()(gid)),
      m_func(view->func())
  { }

  gid_accessor(view_t const& view, index_type const& gid)
    : m_container(&const_cast<view_t&>(view).container()),
      m_gid(const_cast<view_t&>(view).mapfunc()(gid)),
      m_func(view.func())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @todo PARAGRAPH doesn't have get_element.
  //////////////////////////////////////////////////////////////////////
  value_type read(void) const
  {
    // return m_container->get_element(m_gid);
    return m_func(m_container->operator[](m_gid));
  }

  template<typename T>
  void write(const T& val) const
  {
    m_container->set_element(m_gid,val);
  }

  template<typename Class, typename Rtn>
  Rtn invoke(Rtn (Class::* /*const*/ memberFuncPtr)(void) /*const*/) const
  {
    return m_container->apply_get(m_gid, boost::bind(memberFuncPtr, _1));
  }

  template<typename Class>
  void invoke(void (Class::* memberFuncPtr)(void)) const
  {
    m_container->apply_set(m_gid, boost::bind(memberFuncPtr, _1));
  }

  template<typename Class, typename Arg1>
  void invoke(void (Class::* memberFuncPtr)(Arg1),
                    typename invoke_arg<Arg1>::type const& arg1) const
  {
    m_container->apply_set(m_gid, boost::bind(memberFuncPtr,
                                              _1, static_cast<Arg1>(arg1)));
  }

  template<typename Class, typename Arg1, typename Arg2>
  void invoke(void (Class::* memberFuncPtr)(Arg1, Arg2),
                    typename invoke_arg<Arg1>::type const& arg1,
                    typename invoke_arg<Arg2>::type const& arg2) const
  {
    return m_container->apply_set(m_gid, boost::bind(memberFuncPtr,
                      _1, static_cast<Arg1>(arg1),static_cast<Arg2>(arg2)));
  }

  template<typename Class, typename Rtn>
  Rtn const_invoke(Rtn (Class::* const memberFuncPtr)(void) const) const
  {
    return m_container->apply_get(m_gid, boost::bind(memberFuncPtr, _1));
  }

  template<typename Class, typename Rtn, typename Arg1>
  Rtn const_invoke(Rtn (Class::* const memberFuncPtr)(Arg1) const,
                        typename invoke_arg<Arg1>::type const& arg1) const
  {
    return m_container->apply_get(m_gid, boost::bind(memberFuncPtr,
                                             _1, static_cast<Arg1>(arg1)));
  }

  template<typename Class, typename Rtn>
  Rtn* invoke(Rtn* (Class::* /*const*/ memberFuncPtr)(void) /*const*/,
          typename boost::disable_if<is_p_object<Rtn> >::type* dummy = 0 ) const
  {
    return m_container->apply_get_local(m_gid, boost::bind(memberFuncPtr, _1));
  }
}; //class gid_accessor

} // namespace view_impl


//////////////////////////////////////////////////////////////////////
/// @brief Defines a view that behaves as the given @c View but
///        applies the given functor to the elements when they are
///        returned by get data methods.
/// @ingroup transform_view
//////////////////////////////////////////////////////////////////////
template<typename View, typename Functor>
class transform_view :
    public common_view,
    public inherit_retype<View, transform_view<View, Functor> >::type
{
private:
  typedef typename inherit_retype<View, transform_view>::type   base_t;

  //Use container, domain, and map_func types from initial view for
  //constructor params
  typedef typename view_traits<View>::container                 container_t;
  typedef typename view_traits<View>::domain_type               domain_t;
  typedef typename view_traits<View>::map_function              map_func_t;
  Functor m_func;
  typedef decltype(std::declval<base_t>().begin())        base_iterator_t;
  typedef decltype(std::declval<base_t const>().begin())  const_base_iterator_t;

public:

  using iterator = boost::transform_iterator<Functor, base_iterator_t>;
  using const_iterator =
    boost::transform_iterator<Functor, const_base_iterator_t>;

  //////////////////////////////////////////////////////////////////////
  /// @brief Non-const begin iterator
  /// @return Iterator to the start of the transformed view
  //////////////////////////////////////////////////////////////////////
  iterator begin()
  {
    return boost::make_transform_iterator(base_t::begin(), m_func);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Non-const past end iterator
  /// @return Iterator to one past the end of the transformed view
  //////////////////////////////////////////////////////////////////////
  iterator end()
  {
    return boost::make_transform_iterator(base_t::end(), m_func);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Const begin iterator
  /// @return Iterator to the start of the transformed view
  //////////////////////////////////////////////////////////////////////
  const_iterator begin() const
  {
    return boost::make_transform_iterator(base_t::begin(), m_func);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Const past end iterator
  /// @return Iterator to one past the end of the transformed view
  //////////////////////////////////////////////////////////////////////
  const_iterator end() const
  {
    return boost::make_transform_iterator(base_t::end(), m_func);
  }

  STAPL_VIEW_REFLECT_TRAITS(transform_view)

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::array_view::array_view(container_t const&,domain_t const&,map_func_t)
  //////////////////////////////////////////////////////////////////////
  transform_view(container_t& vcont, domain_t const& dom,
                 map_func_t mfunc,
                 transform_view const& other)
    : base_t(vcont, dom, mfunc), m_func(other.m_func)
  { }

  transform_view(container_t& vcont, domain_t const& dom,
                 map_func_t mfunc = map_func_t())
    : base_t(vcont, dom, mfunc), m_func()
  { }

  transform_view(View const& view, Functor const& func)
    : base_t(view), m_func(func)
  { }

  reference
  operator[](index_type g) const
  {
    return m_func(base_t::operator[](g));
  }

  Functor const& func(void) const
  {
    return m_func;
  }

  //////////////////////////////////////////////////////////////////////
  /// @internal
  /// @brief use to examine this class
  /// @param msg your message (to provide context)
  //////////////////////////////////////////////////////////////////////
  void debug(char *msg=0)
  {
    std::cerr << "TRANSFORM_VIEW " << this << " : ";
    if (msg) {
      std::cerr << msg;
    }
    std::cerr << std::endl;
  }

  //////////////////////////////////////////////////////////////////////
  /// @internal
  //////////////////////////////////////////////////////////////////////
  size_t version(void) const
  {
    return 0;
  }

  //////////////////////////////////////////////////////////////////////
  /// @internal
  //////////////////////////////////////////////////////////////////////
  bool validate(void) const
  {
    return true;
  }
}; //class transform_view


//////////////////////////////////////////////////////////////////////
/// @brief Specialization to construct a fast_view over a
///        transform_view.
///
/// @todo Current implementation disables fast transform view
///       creation.
//////////////////////////////////////////////////////////////////////
template<typename View, typename Functor, typename Info, typename CID>
struct localized_view<mix_view<transform_view<View, Functor>, Info, CID>>
  : public transform_view<View, Functor>
{
  localized_view(transform_view<View, Functor> const& v)
    : transform_view<View, Functor>(v)
  { }
};

} // namespace stapl

#endif
