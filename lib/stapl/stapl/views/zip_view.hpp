/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_ZIP_VIEW_HPP
#define STAPL_ZIP_VIEW_HPP

#include <stapl/utility/tuple.hpp>
#include <stapl/views/mapping_functions/identity.hpp>
#include <stapl/views/core_view.hpp>
#include <stapl/views/operations/subscript.hpp>
#include <stapl/views/operations/read_write.hpp>
#include <stapl/views/operations/sequence.hpp>

#include <boost/mpl/transform.hpp>

#include <iostream>

namespace stapl {

namespace view_impl {

template <typename T>
struct get_value_type
{
  typedef typename T::value_type type;
};


struct get_element_at
{
  size_t m_idx;

  get_element_at(size_t idx)
    : m_idx(idx)
  { }

  template <typename V>
  typename V::value_type operator()(V const& v) const
  {
    return *(v.begin() + m_idx);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Defines a container to zip together a set of views.
///
/// @tparam Vn nth View type.
/// @todo Determine if const_reference should be a tuple of const views.
//////////////////////////////////////////////////////////////////////
template <typename... Views>
struct zip_container
  : public p_object
{
  typedef tuple<Views...>                                       views_type;
  typedef tuple<typename Views::value_type...>                  value_type;

  typedef value_type                                            reference;
  typedef const value_type                                      const_reference;

  typedef indexed_domain<size_t>                                domain_type;

  typedef size_t                                                gid_type;

private:
  views_type  m_views;
  size_t      m_size;

public:

  // Constructors

  zip_container(Views const&... vs)
    : m_views(vs...),
      m_size(get<0>(m_views).size())
  { }


  value_type get_element(gid_type const& index) const
  {
    return vs_map(get_element_at(index), m_views);
  }

  value_type operator[](gid_type const& index) const
  {
    return get_element(index);
  }

  template <typename F>
  value_type apply_get(gid_type const& index, F const& f)
  {
    return f(this->get_element(index));
  }

  //////////////////////////////////////////////////////////////////////
  /// @internal
  //////////////////////////////////////////////////////////////////////
  size_t version(void) const
  {
    return 0;
  }

  size_t size(void) const
  {
    return m_size;
  }

  domain_type domain(void) const
  {
    return domain_type(m_size);
  }

  //////////////////////////////////////////////////////////////////////
  /// @internal
  //////////////////////////////////////////////////////////////////////
  void define_type(typer& t)
  {
    t.member(m_views);
    t.member(m_size);
  }
}; // struct zip_container

} // view_impl namespace

//////////////////////////////////////////////////////////////////////
/// @brief Defines a one dimensional view over a zipped set of views.
/// @ingroup zip_view
//////////////////////////////////////////////////////////////////////
template <typename... Views>
class zip_view :
    public core_view<view_impl::zip_container<Views...>,
            indexed_domain<size_t>, f_ident<size_t> >,
    public view_operations::readwrite<
                zip_view<Views...> >,
    public view_operations::subscript<
                zip_view<Views...> >,
    public view_operations::sequence<
                zip_view<Views...> >
{
  typedef core_view<view_impl::zip_container<Views...>,
                    indexed_domain<size_t>, f_ident<size_t> >  base_type;

  typedef view_operations::sequence<zip_view>                  sequence_op_type;

public:
  typedef view_impl::zip_container<Views...>                   container_t;

  STAPL_VIEW_REFLECT_TRAITS(zip_view)

  typedef typename sequence_op_type::iterator                  iterator;
  typedef typename sequence_op_type::const_iterator            const_iterator;


  zip_view(Views... vs)
    : base_type(new container_t(std::forward<Views>(vs)...),
                domain_type( std::get<0>(make_tuple(vs...)).size() ) )
  { }

  zip_view(container_t* vcont, domain_type const& dom,
           map_func_type mfunc=map_func_type())
    : base_type(vcont, dom, mfunc)
  { }

  zip_view(container_t const& vcont,
           domain_type const& dom,
           map_func_type mfunc = map_func_type())
    : base_type(vcont, dom, mfunc)
  { }

  zip_view(container_t const& vcont,
           domain_type const& dom,
           map_func_type mfunc,
           zip_view const&)
    : base_type(vcont, dom, mfunc)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @internal
  /// @brief use to examine this class
  /// @param msg your message (to provide context)
  //////////////////////////////////////////////////////////////////////
  void debug(char *msg=0)
  {
    std::cerr << "ZIP_VIEW " << this << " : ";
    if (msg) {
      std::cerr << msg;
    }
    std::cerr << std::endl;
    base_type::debug();
  }
}; // class zip_view


//////////////////////////////////////////////////////////////////////
/// Helper function to construct zip views.
//////////////////////////////////////////////////////////////////////
template <typename... Views>
zip_view<Views...> zip(Views... vs)
{
  typedef zip_view<Views...>  view_t;
  return view_t(vs...);
}


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref view_traits for @c zip_view.
/// @see zip_view.
//////////////////////////////////////////////////////////////////////
template<typename... Views>
struct view_traits<zip_view<Views...> >
{
  typedef zip_view<Views...>                    view_t;
  typedef view_impl::zip_container<Views...>    container;
  typedef typename container::value_type        value_type;
  typedef typename container::reference         reference;
  typedef typename container::const_reference   const_reference;
  typedef f_ident<size_t>                       map_function;
  typedef indexed_domain<size_t>                domain_type;
  typedef typename domain_type::index_type      index_type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for @ref fast_view for zip_view.
/// @todo The fast view transformation is not enabled.
//////////////////////////////////////////////////////////////////////
template <typename... Views, typename Info, typename CID>
struct localized_view<mix_view<zip_view<Views...>,Info,CID>>
  : public zip_view<Views...>
{
  typedef mix_view<zip_view<Views...>,Info,CID> view_base_type;

  typedef zip_view<Views...>                    base_type;


  localized_view(view_base_type const& v)
    : base_type(v)
  { }
};



//////////////////////////////////////////////////////////////////////
/// @brief Temporary function to get an element from the
///        @c zip_view::value_type
//////////////////////////////////////////////////////////////////////
template <int I, typename T, typename  A>
typename tuple_element<I,T>::type
proxy_get(stapl::proxy<T,A> const& val)
{
  T t = val;
  return get<I>(t);
}

} // namespace stapl

#endif // STAPL_ZIP_VIEW_HPP
