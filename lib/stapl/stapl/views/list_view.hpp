/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_LIST_VIEW_HPP
#define STAPL_VIEWS_LIST_VIEW_HPP

#include <stapl/views/core_view.hpp>
#include <stapl/views/mapping_functions/identity.hpp>
#include <stapl/views/operations/sequence.hpp>
#include <stapl/containers/iterators/local_iterator.hpp>
#include <stapl/views/type_traits/select_derive.hpp>
#include <stapl/containers/type_traits/container_traits.hpp>
#include <stapl/utility/use_default.hpp>

#include <iostream>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Defines a view that provides the interface of a list
///        abstract data type. (@see stapl::list)
///
/// Provides the operations that are commonly present in a list
/// (random access, iteration, insert, push_back, etc.).
/// @tparam C Container type.
/// @tparam Dom Domain type. By default uses the same domain type
///             provided for the container.
/// @tparam MapFunc Mapping function type. (default: identity mapping function)
/// @tparam Derived Type of the most derived class (default: itself)
/// @ingroup list_view
//////////////////////////////////////////////////////////////////////
template <typename C,
          typename Dom     = typename container_traits<C>::domain_type,
          typename MapFunc = f_ident<typename Dom::index_type>,
          typename Derived = use_default>
class list_view
  : public core_view<C,Dom,MapFunc>,
    public view_operations::sequence<
      typename select_derived<
        Derived, list_view<C, Dom, MapFunc, Derived>
      >::type
    >
{
private:
  using base_type        = core_view<C, Dom, MapFunc>;
  using derived_type     = typename select_derived<Derived, list_view>::type;
  using sequence_op_type = view_operations::sequence<derived_type>;

public:
  STAPL_VIEW_REFLECT_TRAITS(list_view)

  using iterator       = typename sequence_op_type::iterator;
  using const_iterator = typename sequence_op_type::const_iterator;

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::array_view::array_view(view_container_type*,domain_type const&,map_func_type)
  //////////////////////////////////////////////////////////////////////
  list_view(view_container_type* vcont,
            domain_type const& dom,
            map_func_type mfunc = MapFunc())
    : base_type(vcont, dom, std::move(mfunc))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::array_view::array_view(view_container_type const&,domain_type const&,map_func_type)
  //////////////////////////////////////////////////////////////////////
  list_view(view_container_type const& vcont,
            domain_type const& dom,
            map_func_type mfunc = MapFunc())
    : base_type(vcont, dom, std::move(mfunc))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::array_view::array_view(view_container_type const&,domain_type const&,map_func_type)
  //////////////////////////////////////////////////////////////////////
  list_view(view_container_type const& vcont,
            domain_type const& dom,
            map_func_type mfunc,
            list_view const&)
    : base_type(vcont, dom, mfunc, std::move(mfunc))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::array_view::array_view(view_container_type*)
  //////////////////////////////////////////////////////////////////////
  list_view(view_container_type* vcont)
    : base_type(vcont, vcont->domain())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc stapl::array_view::array_view(view_container_type&)
  //////////////////////////////////////////////////////////////////////
  list_view(view_container_type& vcont)
    : base_type(vcont, vcont.domain())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Copy constructor when the passed view is not the most
  ///        derived view.
  //////////////////////////////////////////////////////////////////////
  template<typename Derived1>
  list_view(list_view<C, Dom, MapFunc, Derived1> const& other)
    : base_type(other.get_container(), other.domain(), other.mapfunc())
  { }

  list_view(list_view const&) = default;

  //////////////////////////////////////////////////////////////////////
  /// @brief Inserts the given @p value at the position @p index.
  /// @todo update domain to reflect change in size
  //////////////////////////////////////////////////////////////////////
  iterator insert(iterator const& pos, value_type const& value)
  {
    this->incr_version();
    return this->container().insert(pos, value);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Inserts the given @p value an the front of the underlying
  ///         container.
  //////////////////////////////////////////////////////////////////////
  void push_front(value_type const& value)
  {
    this->incr_version();
    this->container().push_front(value);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Removes the first element in the underlying container.
  //////////////////////////////////////////////////////////////////////
  void pop_front(void)
  {
    this->incr_version();
    this->container().pop_front();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Inserts the given @p value an the end of the underlying
  ///         container.
  /// @todo update domain to reflect change in size
  //////////////////////////////////////////////////////////////////////
  void push_back(value_type const& value)
  {
    this->incr_version();
    this->container().push_back(value);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Removes the last element in the underlying container.
  /// @todo update domain to reflect change in size
  //////////////////////////////////////////////////////////////////////
  void pop_back(void)
  {
    this->incr_version();
    this->container().pop_back();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Removes the element at the position @p index.
  /// @todo update domain to reflect change in size
  //////////////////////////////////////////////////////////////////////
  void erase(index_type const& index)
  {
    this->incr_version();
    this->container().erase(this->mapfunc()(index));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns and iterator pointing to the element at position
  ///        @p index.
  //////////////////////////////////////////////////////////////////////
  iterator find(index_type const& index)
  {
    return this->container().make_iterator(this->mapfunc()(index));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a reference to the element at the beginning of
  ///        the underlying container.
  //////////////////////////////////////////////////////////////////////
  reference front(void)
  {
    return this->container().front();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a reference to the element at the end of the
  ///        underlying container.
  //////////////////////////////////////////////////////////////////////
  reference back(void)
  {
    return this->container().back();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a reference to the element at the given @c index.
  //////////////////////////////////////////////////////////////////////
  reference make_reference(index_type const& index) const
  {
    stapl_assert(this->domain().contains(index),
                 "index out of view domain boundary\n");

    return this->container().make_reference(this->mapfunc()(index));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Insert the given @p value into the underlying container.
  ///
  /// The value is inserted in the container on the location where add
  /// is invoked.
  /// @todo update domain to reflect change in size
  //////////////////////////////////////////////////////////////////////
  void add(value_type const& value)
  {
    this->incr_version();
    this->container().add(value);
  }

  //////////////////////////////////////////////////////////////////////
  /// @internal
  /// @brief use to examine this class
  //////////////////////////////////////////////////////////////////////
  void debug(char *msg=0)
  {
    std::cerr << "LIST_VIEW " << this << " : ";
    if (msg) {
      std::cerr << msg;
    }
    std::cerr << std::endl;
    base_type::debug();
  }
}; // class list_view


namespace detail {

template <typename C, typename Dom, typename MF, typename Der>
struct make_iterator<list_view<C,Dom,MF,Der>, typename C::iterator,
                     typename boost::disable_if<boost::is_same<C,
                       std::list<typename C::value_type> > >::type>
{
  typedef typename C::value_type  value_t;
  typedef typename C::iterator    iterator;
  typedef list_view<C,Dom,MF,Der> view_t;

  template <typename View, typename Iterator>
  static
  iterator iterator_helper(View const& view,
                           typename View::index_type const& index,
                           Iterator)
  {
    return iterator(view.container().get_distribution(),
                    view.domain(),
                    view.mapfunc()(index));
  }

  template <typename View, typename ...Args>
  static
  iterator iterator_helper(View const& view,
                           typename View::index_type const& index,
                           local_iterator<C, Args...>)
  {
    return view.container().make_iterator(view.mapfunc()(index));
  }

  template <typename T>
  static
  iterator iterator_helper(list_view<std::list<T>,Dom,MF,Der> const& view,
                           typename Dom::index_type const& index, iterator)
  {
    return index;
  }

  template <typename View>
  static
  iterator apply(View const& view, typename View::index_type const& index)
  {
    return iterator_helper(view, index, iterator());
  }
}; // struct make_iterator<list_view>

} // namespace detail

} // namespace stapl

#endif
