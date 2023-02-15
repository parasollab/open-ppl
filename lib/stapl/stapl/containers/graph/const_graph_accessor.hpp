/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_CONST_GRAPH_ACCESSOR_HPP
#define STAPL_CONTAINERS_GRAPH_CONST_GRAPH_ACCESSOR_HPP

#include <stapl/containers/iterators/const_container_accessor.hpp>
#include <stapl/containers/iterators/const_property_accessor.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>

#include <stapl/views/proxy/const_index_accessor.hpp>
#include <stapl/utility/invoke_arg.hpp>

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
struct const_property_reference_constructor
{
private:
  typedef const_property_accessor<Container>   accessor_t;

public:
  typedef proxy<T, accessor_t>                 result_type;

  result_type operator()(Container* ct_ptr, GID const& gid) const
  {
    return result_type(accessor_t(ct_ptr, gid));
  }
};

//////////////////////////////////////////////////////////////////////
/// @todo Determine if the function operator can be made static to
/// avoid creating an object for each access.
//////////////////////////////////////////////////////////////////////
template<typename T, typename Container, typename GID>
struct const_property_reference_constructor<T, Container, GID, true>
{
private:
  typedef const_property_accessor<Container>   accessor_t;
  typedef proxy<T, accessor_t>                 proxy_t;

public:
  typedef typename container_traits<T>::
    template construct_view<proxy_t>::type     result_type;

  result_type operator()(Container* ct_ptr, GID const& gid) const
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
class const_graph_accessor
  : public const_container_accessor<Container>
{
  typedef const_container_accessor<Container>               base_type;
public:
  typedef Container     container_type;
  typedef typename container_traits<Container>::gid_type    index_type;
  typedef typename container_traits<Container>::value_type  value_type;

  typedef const_property_reference_constructor<
    typename container_type::vertex_property,
    container_type, index_type>                             referencer_t;
  typedef typename referencer_t::result_type                property_reference;
  typedef property_reference                          const_property_reference;

protected:
  template <typename Derived, typename A, typename C, typename D>
  friend class iterator_facade;

  friend class accessor_core_access;

public:
  const_graph_accessor(void) = default;

  const_graph_accessor(null_reference const&)
  { }

  const_graph_accessor(const_graph_accessor const& other)
    : base_type(other)
  { }

  const_graph_accessor(container_type* container, index_type const& index)
    : base_type(container, index)
  { }

  const_graph_accessor(container_type const* container, index_type const& index)
    : base_type(container, index)
  { }

  property_reference property()
  {
    return referencer_t()(this->m_container, this->m_index);
  }

  property_reference property() const
  {
    return referencer_t()(this->m_container, this->m_index);
  }

  const_property_reference const_property() const
  {
    return referencer_t()(this->m_container, this->m_index);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Provide a copy of the underlying variable this accessor
  ///   provides access for, to apply functors below.
  /// @todo Return by copy is likely incorrect if mutating underlying
  ///   is desired.
  /// @todo Remove this method.  Requests should be redirected to underlying
  ///   accessor.
  //////////////////////////////////////////////////////////////////////
  const value_type ref(void) const
  {
    return this->read();
  }

  bool is_local(void) const
  {
    return this->m_container->is_local(this->m_index);
  }
}; // class const_graph_accessor


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
class const_index_accessor<graph_view<PG, Dom, MapFunc, Derived> >
{
public:
  typedef graph_view<PG, Dom, MapFunc, Derived> View;
  typedef typename view_traits<View>::index_type index_type;
  typedef typename view_traits<View>::container  container;
  typedef typename view_traits<View>::value_type value_type;

  typedef typename container::gid_type           gid_type;

  typedef proxy<typename value_type::property_type,
                const_property_accessor<container> > property_reference;
  typedef property_reference                         const_property_reference;

protected:
  template <typename Derived1, typename A, typename C, typename D>
  friend class iterator_facade;

  friend class ::stapl::accessor_core_access;

  container const* m_container;
  gid_type         m_gid;

public:
  const value_type ref(void) const
  {
    return read();
  }

  property_reference property()
  {
    return property_reference(
             const_property_accessor<container>(this->m_container,
                                                this->m_gid));
  }

  property_reference property() const
  {
    return property_reference(
             const_property_accessor<container>(this->m_container,
                                                this->m_gid));
  }

  const_property_reference const_property() const
  {
    return const_property_reference(
             const_property_accessor<container>(this->m_container,
                                                this->m_gid));
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

  const_index_accessor(void)
    : m_container(NULL)
  { }

  const_index_accessor(null_reference const&)
    : m_container(NULL)
  { }

  const_index_accessor(const_index_accessor const& other)
    : m_container(other.m_container), m_gid(other.m_gid)
  { }

  const_index_accessor(View const* view, index_type gid)
    : m_container(&view->container()), m_gid(view->mapfunc()(gid))
  { }

  const_index_accessor(View const& view, index_type const& gid)
    : m_container(&view.container()),
      m_gid(view.mapfunc()(gid))
  { }

  value_type read() const
  {
    return m_container->get_element(m_gid);
  }

  template<typename Class, typename Rtn>
  Rtn invoke(Rtn (Class::* const memberFuncPtr)(void) const) const
  {
    return m_container->apply_get(m_gid,
      boost::bind((Rtn (Class::* const )(void) const)memberFuncPtr, _1));
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
}; //class const_index_accessor, specialized for graph.

} // detail namespace

} // stapl namespace

#endif // STAPL_CONTAINERS_GRAPH_CONST_GRAPH_ACCESSOR_HPP
