/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_PROXY_INDEX_ACCESSOR_HPP
#define STAPL_VIEWS_PROXY_INDEX_ACCESSOR_HPP

#include <stapl/views/proxy/accessor.hpp>
#include <stapl/views/proxy/accessor_base.hpp>
#include <stapl/views/view_traits.hpp>

namespace stapl {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Accessor used to access the element referenced for the
///        given @p index in the specified @p view.
///
/// The internal representation keeps a reference to the view's
/// container and the referenced index.
//////////////////////////////////////////////////////////////////////
template<typename View>
class index_accessor
{
public:
  typedef typename view_traits<View>::index_type             index_type;
  typedef typename view_traits<View>::container              container;
  typedef typename view_traits<View>::value_type             value_type;
  typedef typename view_traits<View>::map_function::gid_type gid_type;

protected:
  friend class accessor_core_access;

  container* m_container;
  gid_type   m_gid;
  index_type m_index;

public:
  bool is_null(void) const
  {
    return m_container == NULL;
  };

  void define_type(typer& t)
  {
    t.member(m_container);
    t.member(m_gid);
    t.member(m_index);
  }

  index_accessor(void)
    : m_container(NULL),
      m_gid()
  { }

  explicit index_accessor(null_reference const&)
    : m_container(NULL),
      m_gid(),
      m_index()
  { }

  index_accessor(index_accessor const& other)
    : m_container(other.m_container), m_gid(other.m_gid),
      m_index(other.m_index)
  { }

  index_accessor(View* view, index_type index)
    : m_container(&(view->container())), m_gid(view->mapfunc()(index)),
      m_index(index)
  { }

  index_accessor(const View* view, index_type const& index)
    : m_container(const_cast<View*>(view)->get_container()),
      m_gid((const_cast<View*>(view)->mapfunc())(index)),
      m_index(index)
  { }

  index_accessor(View const& view, index_type const& index)
    : m_container(&const_cast<View&>(view).container()),
      m_gid(const_cast<View&>(view).mapfunc()(index)),
      m_index(index)
  { }

  template<typename F>
  typename F::result_type apply_get(F const& f) const
  {
    return m_container->apply_get(m_gid,f);
  }

  template<typename F>
  void apply_set(F const& f) const
  {
    m_container->apply_set(m_gid,f);
  }

  template<typename T>
  void write(const T& val) const
  {
    m_container->set_element(m_gid, val);
  }

  value_type read() const
  {
    return m_container->get_element(m_gid);
  }

  template<typename Class, typename... Args>
  void invoke(void (Class::* const pmf)(Args...),
              typename std::decay<Args>::type const&... args) const

  {
    m_container->apply_set(m_gid,
                           detail::apply_set_helper<
                             void (Class::*)(Args...),
                             typename std::decay<Args>::type...
                           >(pmf, args...));
  }

  template<typename Class, typename Rtn, typename... Args>
  Rtn invoke(Rtn (Class::* const pmf)(Args...),
             typename std::decay<Args>::type const&... args) const
  {
    return m_container->apply_get(m_gid,
                                  detail::apply_get_helper<
                                    Rtn (Class::*)(Args...),
                                    typename std::decay<Args>::type...
                                  >(pmf, args...));
  }

  template<typename Class, typename Rtn, typename... Args>
  Rtn const_invoke(Rtn (Class::* const pmf)(Args...) const,
                   typename std::decay<Args>::type const&... args) const

  {
    return m_container->apply_get(m_gid,
                                  detail::const_apply_get_helper<
                                    Rtn (Class::*)(Args...) const,
                                    typename std::decay<Args>::type...
                                  >(pmf, args...));
  }

  index_type index(void) const
  {
    return m_index;
  }
}; // class index_accessor

} } // namespace stapl::detail

#endif // STAPL_VIEWS_PROXY_INDEX_ACCESSOR_HPP
