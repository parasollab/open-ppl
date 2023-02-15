/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_MAP_VIEW_HPP
#define STAPL_MAP_VIEW_HPP

#include <stapl/views/core_view.hpp>

#include <stapl/domains/interval.hpp>
#include <stapl/views/mapping_functions/identity.hpp>
#include <stapl/views/operations/subscript.hpp>
#include <stapl/views/operations/read_write.hpp>
#include <stapl/views/operations/sequence.hpp>
#include <stapl/views/type_traits/select_derive.hpp>
#include <stapl/views/type_traits/upcast.hpp>
#include <stapl/utility/use_default.hpp>

#include <iostream>

namespace stapl {

#ifdef STAPL_DOCUMENTATION_ONLY

//////////////////////////////////////////////////////////////////////
/// @brief Defines a map view
///
/// Provides the operations that are commonly present in a map.
///
/// @tparam C Container type.
/// @tparam Dom Domain type. By default uses the same domain type
///             provided for the container.
/// @tparam MapFunc Mapping function type. (default: identity mapping
///                 function)
/// @tparam Derived Type of the most derived class (default: itself)
/// @todo Change C::domain_type to container_traits<C>::domain_type.
/// @ingroup map_view
//////////////////////////////////////////////////////////////////////
template <typename C,
          typename Dom     = typename C::domain_type,
          typename MapFunc = f_ident<typename C::gid_type>,
          typename Derived = use_default>
class map_view;

#else

template<typename C, typename ...OptionalParams>
class map_view;

#endif


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for when only container type parameter is specified.
//////////////////////////////////////////////////////////////////////
template<typename C>
struct view_traits<map_view<C>>
  : default_view_traits<C,
     typename C::domain_type,
     f_ident<typename C::gid_type>,
     map_view<C>>
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for when container and domain type parameters
///    are specified.
//////////////////////////////////////////////////////////////////////
template<typename C, typename D>
struct view_traits<map_view<C, D>>
  : default_view_traits<
      C, D, f_ident<typename C::gid_type>, map_view<C, D>
    >
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for when container, domain, and mapping function
///  type parameters are specified.
//////////////////////////////////////////////////////////////////////
template<typename C, typename D, typename F>
struct view_traits<map_view<C, D, F>>
  : default_view_traits<C, D, F, map_view<C, D, F>>
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for when all four type parameters are specified.
//////////////////////////////////////////////////////////////////////
template<typename C, typename D, typename F, typename Derived>
struct view_traits<map_view<C, D, F, Derived>>
  : default_view_traits<C, D, F, Derived>
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for map_view which must keep the old view's
///   iterator domain and cannot use the new one.  Case when @p NewMF
///   varies from old mapping function
//////////////////////////////////////////////////////////////////////
template<typename OldC, typename NewC, typename NewMF>
struct retype_ct_mf_impl<map_view<OldC>, NewC, NewMF, false, 0>
{ typedef map_view<NewC, typename OldC::domain_type, NewMF> type; };


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for map_view which must keep the old view's
///   iterator domain and cannot use the new one.  Case when @p NewMF
///   does not vary from old mapping function
//////////////////////////////////////////////////////////////////////
template<typename OldC, typename NewC, typename NewMF>
struct retype_ct_mf_impl<map_view<OldC>, NewC, NewMF, true, 0>
{ typedef map_view<NewC, typename OldC::domain_type> type; };


//////////////////////////////////////////////////////////////////////
/// @brief Specialization ensures container transform for variadic
///  based optionals is used.
//////////////////////////////////////////////////////////////////////
template<typename OldC, typename NewC, typename ...Params>
struct cast_container_view<map_view<OldC, Params...>, NewC>
  : new_cast_container_view<map_view<OldC, Params...>, NewC>
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Specialization ensures container domain and mapping function
///   transform for variadic based optionals is used.
//////////////////////////////////////////////////////////////////////
template<typename C, typename Dom, typename MF, typename ...Params>
struct upcast_view<map_view<C, Params...>, Dom, MF>
  : new_upcast_view<map_view<C, Params...>, Dom, MF>
{ };


template <typename C, typename ...OptionalParams>
class map_view
  : public core_view<
       C,
       typename view_traits<map_view<C, OptionalParams...>>::domain_type,
       typename view_traits<map_view<C, OptionalParams...>>::map_function
    >,
    public view_operations::readwrite<map_view<C, OptionalParams...>>,
    public view_operations::subscript<map_view<C, OptionalParams...>>,
    public view_operations::sequence<
      typename view_traits<map_view<C, OptionalParams...>>::derived_type
    >
{
public:
  STAPL_VIEW_REFLECT_TRAITS(map_view)

private:
  typedef typename view_traits<map_view>::derived_type   derived_type;

  typedef core_view<C, domain_type, map_function>          base_type;

  typedef view_operations::sequence<derived_type>          sequence_op_type;

public:
  typedef typename C::key_type                         key_type;
  typedef typename C::mapped_type                      mapped_type;
  typedef typename C::second_reference                 second_reference;
  typedef typename sequence_op_type::iterator          iterator;
  typedef typename sequence_op_type::const_iterator    const_iterator;

  map_view(void) = delete;

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::array_view::array_view(view_container_type*,domain_type const&,map_func_type)
  //////////////////////////////////////////////////////////////////////
  map_view(view_container_type* vcont,
           domain_type const& dom,
           map_func_type mfunc = map_function())
    : base_type(vcont, dom, mfunc)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::array_view::array_view(view_container_type*,domain_type const&,map_func_type, array_view)
  //////////////////////////////////////////////////////////////////////
  map_view(view_container_type* vcont,
           domain_type const& dom,
           map_func_type mfunc,
           map_view const& other)
    : base_type(vcont, dom, mfunc)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::array_view::array_view(view_container_type const&,domain_type const&,map_func_type)
  //////////////////////////////////////////////////////////////////////
  map_view(view_container_type const& vcont, domain_type const& dom,
           map_func_type mfunc=map_function())
    : base_type(vcont, dom, mfunc)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::array_view::array_view(view_container_type const&,domain_type const&,map_func_type)
  //////////////////////////////////////////////////////////////////////
  map_view(view_container_type const& vcont, domain_type const& dom,
           map_func_type mfunc, map_view const&)
    : base_type(vcont, dom, mfunc)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::array_view::array_view(view_container_type*)
  //////////////////////////////////////////////////////////////////////
  map_view(view_container_type* vcont)
    : base_type(vcont, domain_type(vcont->domain()))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::array_view::array_view(view_container_type&)
  //////////////////////////////////////////////////////////////////////
  map_view(view_container_type& vcont)
    : base_type(vcont, vcont.domain())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Allow map_view bases class  other class hierarchies to be used
  ///  for copy construction.
  //////////////////////////////////////////////////////////////////////
  template<typename ...OtherOptionalParams>
  map_view(map_view<C, OtherOptionalParams...> const& other)
    : base_type(other.container(), other.domain(), other.mapfunc())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Inserts the given key-value pair if the value
  ///        does not exist in the container.
  /// @todo update domain to reflect change in size
  //////////////////////////////////////////////////////////////////////
  bool insert(value_type const& value)
  {
    this->incr_version();
    return this->container().insert(value);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Inserts the given <key,value> pair (@p val) if the key
  ///        does not exist in the container.
  /// @todo update domain to reflect change in size
  //////////////////////////////////////////////////////////////////////
  void insert(key_type const& key, mapped_type const& value)
  {
    this->incr_version();
    this->container().insert(key, value);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Inserts the given <key,value> pair (@p val) if the key
  ///        does not exist in the container, otherwise mutates the
  ///        stored value by applying the given functor @p func.
  /// @todo update domain to reflect change in size
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  void insert(value_type const& val, Functor const& func)
  {
    this->incr_version();
    this->container().insert(val, func);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Inserts the given @p value associated with the given @p
  ///        key if the @p key does not exist in the container,
  ///        otherwise mutates the stored value by applying the given
  ///        functor @p func.
  ///
  /// @todo There is no uniform interface across the containers for
  ///       invoking the forwarded method.  E.g., @ref map provides
  ///       the method while @ref unordered_map does not.
  /// @todo update domain to reflect change in size
  //////////////////////////////////////////////////////////////////////
  template <typename Functor>
  void insert(key_type const& key, mapped_type const& value,
                    Functor const& on_failure)
  {
    this->incr_version();
    this->container().insert(key, value, on_failure);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Fetch the value associated with the specified @p key.
  //////////////////////////////////////////////////////////////////////
  mapped_type get(key_type const& key)
  {
    return this->container()[key];
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Removes the value associated with the specified @p key.
  ///
  /// @todo update domain to reflect change in size
  //////////////////////////////////////////////////////////////////////
  size_t erase(key_type const& key)
  {
    if (this->domain().contains(key))
    {
      this->incr_version();
      return this->container().erase(key);
    }
    //else
    return 0;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator pointing to the element associated with
  ///        the given @p key.
  ///
  /// @todo Verify this implementation.
  //////////////////////////////////////////////////////////////////////
  iterator find(key_type const& key)
  {
    if (this->domain().contains(key))
      return this->container().find(key);

    //else
    return this->end();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns 1 if the specified key exists, and otherwise 0
  /// @param key key to find
  /// @return 0 or 1 if the unique key is in the map
  //////////////////////////////////////////////////////////////////////
  int count(key_type const& key)
  {
    return static_cast<int> (this->find(key)!=this->end());
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
  /// @brief Construct a reference to the mapped value for a specific key.
  /// @param index The GID of the element to access.
  /// @return Proxy over the mapped value
  //////////////////////////////////////////////////////////////////////
  second_reference operator[](index_type index) const
  {
    return this->container()[this->mapfunc()(index)];
  }

  //////////////////////////////////////////////////////////////////////
  /// @internal
  /// @brief use to examine this class
  //////////////////////////////////////////////////////////////////////
  void debug(char *msg=0)
  {
    std::cerr << "MAP_VIEW " << this << " : ";
    if (msg) {
      std::cerr << msg;
    }
    std::cerr << std::endl;
    base_type::debug();
  }

  //////////////////////////////////////////////////////////////////////
  /// @internal
  //////////////////////////////////////////////////////////////////////
  void define_type(typer& t)
  {
    t.base<base_type>(*this);
  }

}; // class map_view

} // namespace stapl

#endif // STAPL_MAP_VIEW_HPP
