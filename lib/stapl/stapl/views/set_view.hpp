/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_SET_VIEW_HPP
#define STAPL_VIEWS_SET_VIEW_HPP

#include <stapl/views/core_view.hpp>

#include <stapl/domains/interval.hpp>
#include <stapl/views/mapping_functions/identity.hpp>
#include <stapl/views/operations/subscript.hpp>
#include <stapl/views/operations/read_write.hpp>
#include <stapl/views/operations/sequence.hpp>
#include <stapl/views/type_traits/select_derive.hpp>
#include <stapl/utility/use_default.hpp>

#include <iostream>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Defines a set view
///
/// Provides the operations that are commonly present in a set.
///
/// @tparam C Container type.
/// @tparam Dom Domain type. By default uses the same domain type
///             provided for the container.
/// @tparam MapFunc Mapping function type. (default: identity mapping
///                 function)
/// @tparam Derived Type of the most derived class (default: itself)
/// @todo Change C::domain_type to container_traits<C>::domain_type.
/// @ingroup set_view
//////////////////////////////////////////////////////////////////////
template <typename C,
          typename Dom = typename C::domain_type,
          typename MapFunc = f_ident<typename C::gid_type>,
          typename Derived = use_default>
class set_view :
    public core_view<C,Dom,MapFunc>,
    public view_operations::readwrite<set_view<C,Dom,MapFunc,Derived> >,
    public view_operations::subscript<set_view<C,Dom,MapFunc,Derived> >,
    public view_operations::sequence<
                   typename select_derived<
                     Derived, set_view<C, Dom, MapFunc, Derived> >::type>
{
  typedef core_view<C,Dom,MapFunc>                     base_type;

  typedef typename select_derived<Derived,
                                  set_view>::type      derived_type;

  typedef view_operations::sequence<derived_type>      sequence_op_type;

public:

  STAPL_VIEW_REFLECT_TRAITS(set_view)

  typedef typename C::key_type                         key_type;
  typedef typename C::gid_type                         gid_type;
  typedef typename sequence_op_type::iterator          iterator;
  typedef typename sequence_op_type::const_iterator    const_iterator;

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::array_view::array_view(view_container_type*,domain_type const&,map_func_type)
  //////////////////////////////////////////////////////////////////////
  set_view(view_container_type* vcont, domain_type const& dom,
           map_func_type mfunc=MapFunc())
    : base_type(vcont, dom, mfunc)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::array_view::array_view(view_container_type*,domain_type const&,map_func_type, array_view const&)
  //////////////////////////////////////////////////////////////////////
  set_view(view_container_type* vcont,
           domain_type const& dom,
           map_func_type mfunc,
           set_view const&)
    : base_type(vcont, dom, mfunc)
  { }


  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::array_view::array_view(view_container_type const&,domain_type const&,map_func_type)
  //////////////////////////////////////////////////////////////////////
  set_view(view_container_type const& vcont, domain_type const& dom,
           map_func_type mfunc=MapFunc())
    : base_type(vcont, dom, mfunc)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::array_view::array_view(view_container_type const&,domain_type const&,map_func_type)
  //////////////////////////////////////////////////////////////////////
  set_view(view_container_type const& vcont, domain_type const& dom,
           map_func_type mfunc, set_view const&)
    : base_type(vcont, dom, mfunc)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::array_view::array_view(view_container_type*)
  //////////////////////////////////////////////////////////////////////
  set_view(view_container_type* vcont)
    : base_type(vcont, domain_type(vcont->domain()))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::array_view::array_view(view_container_type&)
  //////////////////////////////////////////////////////////////////////
  set_view(view_container_type& vcont)
    : base_type(vcont, vcont.domain())
  { }

  template<typename Derived1>
  set_view(set_view<C, Dom, MapFunc, Derived1> const& other)
    : base_type(other.container(), other.domain(), other.mapfunc())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Inserts the given @p key if the key
  ///        does not exist in the container
  /// @todo update domain to reflect change in size
  //////////////////////////////////////////////////////////////////////
  void insert(key_type const& key)
  {
    this->incr_version();
    this->container().insert(key);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Removes the value associated with the specified @p key
  ///   asynchronously.
  ///
  /// @todo update domain to reflect change in size
  //////////////////////////////////////////////////////////////////////
  void erase(key_type const& key)
  {
    this->incr_version();
    this->container().erase(key);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Removes the value associated with the specified @p key.
  /// @return The number of elements erased.
  /// @todo update domain to reflect change in size
  //////////////////////////////////////////////////////////////////////
  size_t erase_sync(key_type const& key)
  {
    this->incr_version();
    return this->container().erase_sync(key);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator pointing to the element associated with
  ///        the given @p key.
  ///
  /// @todo Verify this implementation.
  //////////////////////////////////////////////////////////////////////
  iterator find(key_type const& key)
  {
    return this->container().make_iterator(key);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Counts the number of elements with the key @p key.
  /// @return The number of elements.
  //////////////////////////////////////////////////////////////////////
  size_t count(key_type const& key) const
  {
    return this->container().count(key);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Removes all the elements stored in the container.
  ///
  /// @todo Verify this implementation.
  /// @todo update domain to reflect change in size
  //////////////////////////////////////////////////////////////////////
   void clear(void)
  {
    this->incr_version();
    this->container().clear();
  }

  //////////////////////////////////////////////////////////////////////
  /// @internal
  /// @brief use to examine this class
  //////////////////////////////////////////////////////////////////////
  void debug(char *msg=0)
  {
    std::cerr << "SET_VIEW %p: " << this;
    if (msg) {
      std::cerr << msg;
    }
    std::cerr << "\n";
    (static_cast<base_type *>(this))->debug();
  }

  //////////////////////////////////////////////////////////////////////
  /// @internal
  //////////////////////////////////////////////////////////////////////
  void define_type(typer& t)
  {
    t.base<base_type>(*this);
  }

};

} // stapl namespace

#endif /* STAPL_SET_VIEW_HPP */
