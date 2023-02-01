/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_PROXY_CONST_INDEX_ACCESSOR_HPP
#define STAPL_VIEWS_PROXY_CONST_INDEX_ACCESSOR_HPP

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
class const_index_accessor
{
public:
  typedef typename view_traits<View>::index_type             index_type;
  typedef typename view_traits<View>::container              container;
  typedef typename view_traits<View>::value_type             value_type;
  typedef typename view_traits<View>::map_function::gid_type gid_type;

protected:
  friend class accessor_core_access;

  container const* m_container;
  gid_type         m_index;

public:
  bool is_null(void) const
  {
    return m_container == NULL;
  };

  void define_type(typer& t)
  {
    t.member(m_container);
    t.member(m_index);
  }

  const_index_accessor(void)
    : m_container(NULL),
      m_index()
  { }

  explicit const_index_accessor(null_reference const&)
    : m_container(NULL),
      m_index()
  { }

  const_index_accessor(const_index_accessor const& other)
    : m_container(other.m_container), m_index(other.m_index)
  { }

  const_index_accessor(View* view, index_type index)
    : m_container(&(view->container())), m_index(view->mapfunc()(index))
  { }

  const_index_accessor(const View* view, index_type const& index)
    : m_container(view->get_container()),
      m_index((const_cast<View*>(view)->mapfunc())(index))
  { }

  const_index_accessor(View const& view, index_type const& gid)
    : m_container(view.container()),
      m_index(const_cast<View&>(view).mapfunc()(gid))
  { }

  template<typename F>
  typename F::result_type apply_get(F const& f) const
  {
    return m_container->apply_get(m_index,f);
  }

  value_type read() const
  {
    return m_container->get_element(m_index);
  }

  template<typename Class, typename Rtn, typename... Args>
  Rtn const_invoke(Rtn (Class::* const pmf)(Args...) const,
                   typename std::decay<Args>::type const&... args) const

  {
    return m_container->apply_get(m_index,
                                  detail::const_apply_get_helper<
                                    Rtn (Class::*)(Args...) const,
                                    typename std::decay<Args>::type...
                                  >(pmf, args...));
  }

  index_type index(void) const
  {
    return m_index;
  }
}; // class const_index_accessor

} } // namespace stapl::detail

#endif // STAPL_VIEWS_PROXY_CONST_INDEX_ACCESSOR_HPP
