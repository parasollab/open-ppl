/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_GRAPH_ACCESSOR_HPP
#define STAPL_CONTAINERS_GRAPH_GRAPH_ACCESSOR_HPP

#include <stapl/containers/graph/const_graph_accessor.hpp>
#include <stapl/containers/iterators/container_accessor.hpp>
#include <stapl/containers/iterators/property_accessor.hpp>
#include <stapl/containers/iterators/const_property_accessor.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>

#include <stapl/views/proxy/index_accessor.hpp>
#include <stapl/views/proxy/const_index_accessor.hpp>
#include <stapl/utility/invoke_arg.hpp>

#include <boost/bind.hpp>
#include <boost/utility/result_of.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Functor creates a reference either as a proxy or a view, based
///   on @ref use_view_as_reference metafunction.
/// @tparam T The value type of the outer container, possibly another container.
/// @tparam The Outer container type.
/// @tparam GID Gid type of the outer container.
///
/// Primary template constructs a proxy and returns it as the reference for the
/// iterator dereference. The sole specialization instead heap allocates the
/// proxy and uses it as the container for a view (the type of which is
/// determined by @ref container_traits.
//////////////////////////////////////////////////////////////////////
template<typename T, typename Container, typename GID,
         bool = is_container<T>::value>
struct property_reference_constructor
{
private:
  typedef property_accessor<Container>         accessor_t;

public:
  typedef proxy<T, accessor_t>                 result_type;

  result_type operator()(Container* ct_ptr, GID const& gid) const
  {
    return result_type(accessor_t(ct_ptr, gid));
  }

  result_type operator()(Container const* ct_ptr, GID const& gid) const
  {
    return result_type(accessor_t(ct_ptr, gid));
  }
};

//////////////////////////////////////////////////////////////////////
/// @todo Determine if the function operator can be made static to
/// avoid creating an object for each access.
//////////////////////////////////////////////////////////////////////
template<typename T, typename Container, typename GID>
struct property_reference_constructor<T, Container, GID, true>
{
private:
  typedef property_accessor<Container>         accessor_t;
  typedef proxy<T, accessor_t>                 proxy_t;

public:
  typedef typename container_traits<T>::
    template construct_view<proxy_t>::type     result_type;

  result_type operator()(Container* ct_ptr, GID const& gid) const
  {
    return result_type(new proxy_t(accessor_t(ct_ptr, gid)));
  }

  result_type operator()(Container const* ct_ptr, GID const& gid) const
  {
    return result_type(new proxy_t(accessor_t(ct_ptr, gid)));
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief accessor for vertices of a graph.
/// @tparam Container Type of the pGraph container.
/// @ingroup pgraphDistObj
//////////////////////////////////////////////////////////////////////
template <typename Container>
class graph_accessor
  : public container_accessor<Container>
{
  typedef container_accessor<Container>                     base_type;
public:
  typedef Container     container_type;
  typedef typename container_traits<Container>::gid_type    index_type;
  typedef typename container_traits<Container>::value_type  value_type;

  typedef property_reference_constructor<
    typename container_type::vertex_property,
    container_type, index_type>                             referencer_t;
  typedef typename referencer_t::result_type                property_reference;

  typedef const_property_reference_constructor<
    typename container_type::vertex_property,
    container_type, index_type>                             const_referencer_t;
  typedef typename const_referencer_t::result_type    const_property_reference;

protected:
  template <typename Derived, typename A, typename C, typename D>
  friend class iterator_facade;

  friend class accessor_core_access;

public:
  graph_accessor(void) = default;

  graph_accessor(null_reference const&)
  { }

  graph_accessor(graph_accessor const& other)
    : base_type(other)
  { }

  graph_accessor(container_type* container, index_type const& index)
    : base_type(container, index)
  { }

  property_reference property(void)
  {
    return referencer_t()(this->m_container, this->m_index);
  }

  property_reference property(void) const
  {
    return referencer_t()(this->m_container, this->m_index);
  }

  const_property_reference const_property(void) const
  {
    return const_referencer_t()(this->m_container, this->m_index);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Provide a copy of the underlying variable this accessor
  ///   provides access for, to apply functors below.
  /// @todo Return by copy is likely incorrect if mutating underlying
  ///   is desired.
  /// @todo Remove this method.  Requests should be redirected to underlying
  ///   accessor.
  //////////////////////////////////////////////////////////////////////
  value_type ref(void) const
  {
    return this->read();
  }

  bool is_local(void) const
  {
    return this->m_container->is_local(this->m_index);
  }
}; // class graph_accessor


template<typename PG, typename Dom, typename MapFunc, typename Derived>
class graph_view;


namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref index_accessor for @ref graph_view.
/// @tparam PG Type of the pGraph container.
/// @tparam Dom Type of the pGraph Domain.
/// @tparam MapFunc Type of the Mapping function.
/// @tparam Derived The most derived type.
/// @ingroup pgraphDistObj
//////////////////////////////////////////////////////////////////////
template<typename PG, typename Dom, typename MapFunc, typename Derived>
class index_accessor<graph_view<PG, Dom, MapFunc, Derived>>
{
public:
  typedef graph_view<PG, Dom, MapFunc, Derived> View;
  typedef typename view_traits<View>::index_type index_type;
  typedef typename view_traits<View>::container  container;
  typedef typename view_traits<View>::value_type value_type;

  typedef typename container::gid_type           gid_type;

  using referencer_t
    = property_reference_constructor<
       typename container::vertex_property, container, index_type>;

  using property_reference = typename referencer_t::result_type;

  using const_referencer_t
    = const_property_reference_constructor<
        typename container::vertex_property, container, index_type>;

  using const_property_reference = typename const_referencer_t::result_type;

protected:
  template <typename Derived1, typename A, typename C, typename D>
  friend class iterator_facade;

  friend class ::stapl::accessor_core_access;

  container* m_container;
  gid_type   m_gid;

public:
  value_type ref(void) const
  {
    return read();
  }

  property_reference property(void)
  {
    return referencer_t()(this->m_container, this->m_gid);
  }

  const_property_reference property(void) const
  {
    return const_referencer_t()(this->m_container, this->m_gid);
  }


  const_property_reference const_property(void)
  {
    return const_property_reference(
             property_accessor<container>(this->m_container, this->m_gid));
  }

  bool is_null(void) const
  {
    return m_container == NULL;
  };

public:
  void define_type(typer& t)
  {
    t.member(m_container);
    t.member(m_gid);
  }

  index_accessor(void)
    : m_container(NULL)
  { }

  index_accessor(null_reference const&)
    : m_container(NULL)
  { }

  index_accessor(index_accessor const& other)
    : m_container(other.m_container), m_gid(other.m_gid)
  { }

  index_accessor(View* view, index_type gid)
    : m_container(&view->container()), m_gid(view->mapfunc()(gid))
  { }

  index_accessor(const View* view, index_type const& gid)
    : m_container(const_cast<View*>(view)->get_container()),
       m_gid((const_cast<View*>(view)->mapfunc())(gid))
  { }

  index_accessor(View const& view, index_type const& gid)
    : m_container(&const_cast<View&>(view).container()),
      m_gid(const_cast<View&>(view).mapfunc()(gid))
  { }

  value_type read(void) const
  {
    return m_container->get_element(m_gid);
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
              const typename invoke_arg<Arg1>::type& arg1) const
  {
    m_container->apply_set(m_gid, boost::bind(memberFuncPtr, _1,
                                              static_cast<Arg1>(arg1)));
  }

  template<typename Class, typename Arg1, typename Arg2>
  void invoke(void (Class::* memberFuncPtr)(Arg1, Arg2),
                  const typename invoke_arg<Arg1>::type& arg1,
                  const typename invoke_arg<Arg2>::type& arg2) const
  {
    return m_container->apply_set(m_gid, boost::bind(memberFuncPtr, _1,
                                                     static_cast<Arg1>(arg1),
                                                     static_cast<Arg2>(arg2)));
  }

  template<typename Class, typename Rtn>
  Rtn const_invoke(Rtn (Class::* const memberFuncPtr)(void) const) const
  {
    return m_container->apply_get(m_gid, boost::bind(memberFuncPtr, _1));
  }

  template<typename Class, typename Rtn, typename Arg1>
  Rtn const_invoke(Rtn (Class::* const memberFuncPtr)(Arg1) const,
                   const typename invoke_arg<Arg1>::type& arg1) const
  {
    return m_container->apply_get(m_gid, boost::bind(memberFuncPtr, _1,
                                                     static_cast<Arg1>(arg1)));
  }

  template<typename Class, typename Rtn>
  Rtn* invoke(Rtn* (Class::* /*const*/ memberFuncPtr)(void) /*const*/,
                    typename boost::disable_if<is_p_object<Rtn> >::type* dummy
              = 0 ) const
  {
    return m_container->apply_get_local(m_gid, boost::bind(memberFuncPtr, _1));
  }

  index_type index(void) const
  {
    return m_gid;
  }

private:
  template<typename F>
  typename boost::result_of<F(value_type)>::type apply_get(F const& f) const
  {
    return m_container->apply_get(m_gid,f);
  }

  template<typename F>
  void apply_set(const F& f) const
  {
    m_container->apply_set(m_gid, f);
  }

}; //class index_accessor, specialized for graph.

} // detail namespace

} // stapl namespace

#endif /* STAPL_CONTAINERS_GRAPH_GRAPH_ACCESSOR_HPP */
