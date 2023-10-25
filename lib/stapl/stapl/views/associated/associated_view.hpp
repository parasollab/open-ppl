/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_ASSOCIATED_VIEW_HPP
#define STAPL_VIEWS_ASSOCIATED_VIEW_HPP

#include <stapl/domains/interval.hpp>
#include <stapl/views/metadata/mix_view.hpp>
#include <stapl/views/type_traits/retype.hpp>
#include <stapl/utility/use_default.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Defines a view that behaves as V but modifications to V's
///        container distribution influences the U's containers
///        distribution.
///
/// @tparam V Main view
/// @tparam U Follower view
/// @tparam Mapping Function object that maps index of V to collection
///         of indexes of U
///         @f$( i\rightarrow \{j_0, j_1, ...\}
///            \ |\ i \in \textsf{Dom} \wedge
///                 j_k \in {\tt domain}(\textsf{U}) )@f$.
/// @tparam Dom Domain type.
/// @tparam MapFunc Mapping function type.
/// @tparam Derived Type of the most derived class (default: itself).
//////////////////////////////////////////////////////////////////////
template <typename V,
          typename U,
          typename Mapping,
          typename Dom = typename view_traits<V>::domain_type,
          typename MapFunc = typename view_traits<V>::map_function,
          typename Derived = use_default>
class associated_view
  : public inherit_retype<
             V, associated_view<V, U, Mapping, Dom, MapFunc, Derived>
           >::type
{
  typedef typename inherit_retype<V, associated_view>::type     base_type;

  Mapping  m_mapping;
  U        m_follower;

public:
  STAPL_VIEW_REFLECT_TRAITS(associated_view)

  typedef typename Mapping::value_type              associated_value_type;

  typedef typename associated_value_type::iterator  associated_iterator;

  typedef typename view_traits<U>::index_type       follower_index_type;

  typedef domset1D<follower_index_type>             follower_domain_type;

  typedef typename domain_retype<
    U, follower_domain_type>::type                  follower_type;

 //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::array_view::array_view(view_container_type*,domain_type const&,map_func_type)
  //////////////////////////////////////////////////////////////////////
  associated_view(view_container_type* vcont, domain_type const& dom,
                  map_func_type mfunc=MapFunc())
    : base_type(vcont, dom, mfunc)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::array_view::array_view(view_container_type const&,domain_type const&,map_func_type)
  //////////////////////////////////////////////////////////////////////
  associated_view(view_container_type const& vcont, domain_type const& dom,
                  map_func_type mfunc=MapFunc())
    : base_type(vcont, dom, mfunc)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs an associated view based on the given @c view,
  ///        with the @c follower view and using the given @c mapping
  ///        to represent the relation between @c view and @c
  ///        follower.
  ///
  /// @param view Main view.
  /// @param follower Follower view.
  /// @param mapping Mapping between @c view and @c follower.
  //////////////////////////////////////////////////////////////////////
  associated_view(V const& view, U const& follower, Mapping const& mapping)
    : base_type(view.container()), m_mapping(mapping), m_follower(follower)
  { }

  associated_view()
  { }

  associated_view(associated_view const& other)
    : base_type(other), m_mapping(other.m_mapping), m_follower(other.m_follower)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Migrate the element @c i and its associated elements to
  ///        the new location @c loc.
  ///
  /// @param i Index of element to migrate.
  /// @param loc Destination location.
  //////////////////////////////////////////////////////////////////////
  void migrate(index_type const& i, location_type const& loc)
  {
    associated_value_type associated = m_mapping(this->make_reference(i), i);

    associated_iterator it = associated.begin();
    associated_iterator eit = associated.end();

    for (; it != eit; ++it)
        m_follower.container().migrate(*it, loc);

    this->container().migrate(i, loc);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a view over the elements that are associated with
  ///        the given element @c i.
  ///
  /// @param i Index of element.
  //////////////////////////////////////////////////////////////////////
  follower_type associated(index_type const& i)
  {
    follower_domain_type dom;

    associated_value_type associated = m_mapping(this->make_reference(i), i);

    associated_iterator it = associated.begin();
    associated_iterator eit = associated.end();

    for (; it != eit; ++it)
      dom += *it;

    return follower_type(m_follower.container(), dom);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Associates the index j to the index i.
  ///
  /// @param i Index from the main view.
  /// @param j Index from the follower view.
  //////////////////////////////////////////////////////////////////////
  void track(index_type const& i, follower_index_type const& j)
  {
    m_mapping.track(i, j);
  }

  void define_type(typer &t)
  {
    t.base<base_type>(*this);
    t.member(m_mapping);
    t.member(m_follower);
  }
}; //class associated_view


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for view_traits over an associated_view
//////////////////////////////////////////////////////////////////////
template<typename V, typename U, typename M,
         typename D, typename F, typename De>
struct view_traits<associated_view<V, U, M, D,F,De> >
{
  typedef typename V::value_type                             value_type;
  typedef typename V::value_type                             reference;
  typedef typename V::view_container_type                    container;
  typedef F                                                  map_function;
  typedef D                                                  domain_type;
  typedef typename D::index_type                             index_type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization to convert an associated_view into a fast
///        access associated_view.
//////////////////////////////////////////////////////////////////////
template<typename V, typename U, typename M,
         typename D, typename F, typename De, typename Info, typename CID>
struct localized_view<mix_view<associated_view<V, U, M, D, F, De>, Info, CID>>
  : public associated_view<V, U, M, D, F, De>
{
  localized_view(associated_view<V, U, M, D, F, De> const& v)
    : associated_view<V, U, M, D, F, De>(v)
  { }
};

} // stapl namespace

#endif
