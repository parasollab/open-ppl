/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_ARRAY_RO_VIEW_HPP
#define STAPL_VIEWS_ARRAY_RO_VIEW_HPP

#include <stapl/views/core_view.hpp>

#include <stapl/views/mapping_functions/identity.hpp>
#include <stapl/views/operations/subscript.hpp>
#include <stapl/views/operations/read_write.hpp>
#include <stapl/views/operations/sequence.hpp>
#include <stapl/views/type_traits/select_derive.hpp>
#include <stapl/views/type_traits/upcast.hpp>
#include <stapl/views/operations/view_iterator.hpp>
#include <stapl/containers/type_traits/container_traits.hpp>
#include <stapl/utility/use_default.hpp>

#include <iostream>

namespace stapl {

#ifdef STAPL_DOCUMENTATION_ONLY

//////////////////////////////////////////////////////////////////////
/// @brief One dimensional read only array view
///
/// Provides the operations that are commonly present in an array
/// (random access, iteration). Write operation is not supported.
/// @tparam C Container type.
/// @tparam Dom Domain type. By default uses the same domain type
///             provided for the container.
/// @tparam MapFunc Mapping function type. (default: identity mapping function)
/// @tparam Derived Type of the most derived class (default: itself)
/// @ingroup array_ro_view
/// @todo Remove default constructor
//////////////////////////////////////////////////////////////////////





template<typename C,
         typename Dom = typename container_traits<C>::domain_type,
         typename MapFunc = f_ident<typename Dom::index_type>,
         typename Derived = use_default>
class array_ro_view;

#else

template<typename C, typename ...OptionalParams>
class array_ro_view;

#endif


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for when only container type parameter is specified.
//////////////////////////////////////////////////////////////////////
template<typename C>
struct view_traits<array_ro_view<C>>
  : default_view_traits<C,
      typename container_traits<C>::domain_type,
      f_ident<typename container_traits<C>::domain_type::index_type>,
      array_ro_view<C>>
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for when container and domain type parameters
///    are specified.
//////////////////////////////////////////////////////////////////////
template<typename C, typename D>
struct view_traits<array_ro_view<C, D>>
  : default_view_traits<
      C, D, f_ident<typename D::index_type>, array_ro_view<C,D>>
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for when container, domain, and mapping function
///  type parameters are specified.
//////////////////////////////////////////////////////////////////////
template<typename C, typename D, typename F>
struct view_traits<array_ro_view<C, D, F>>
  : default_view_traits<C, D, F, array_ro_view<C, D, F>>
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for when all four type parameters are specified.
//////////////////////////////////////////////////////////////////////
template<typename C, typename D, typename F, typename Derived>
struct view_traits<array_ro_view<C, D, F, Derived>>
  : default_view_traits<C, D, F, Derived>
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Specialization ensures container transform for variadic
///  based optionals is used.
//////////////////////////////////////////////////////////////////////
template<typename OldC, typename NewC, typename ...Params>
struct cast_container_view<array_ro_view<OldC, Params...>, NewC>
  : new_cast_container_view<array_ro_view<OldC, Params...>, NewC>
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Specialization ensures container domain and mapping function
///   transform for variadic based optionals is used.
//////////////////////////////////////////////////////////////////////
template<typename C, typename Dom, typename MF, typename ...Params>
struct upcast_view<array_ro_view<C, Params...>, Dom, MF>
  : new_upcast_view<array_ro_view<C, Params...>, Dom, MF>
{ };


template <typename C, typename ...OptionalParams>
class array_ro_view
  : public core_view<
      C,
      typename view_traits<array_ro_view<C, OptionalParams...>>::domain_type,
      typename view_traits<array_ro_view<C, OptionalParams...>>::map_function
    >,
    public view_operations::read<array_ro_view<C, OptionalParams...>>,
    public view_operations::subscript<array_ro_view<C, OptionalParams...>>,
    public view_operations::sequence<
      typename view_traits<array_ro_view<C, OptionalParams...>>::derived_type
    >
{
public:
  STAPL_VIEW_REFLECT_TRAITS(array_ro_view)

private:
  typedef typename view_traits<array_ro_view>::derived_type    derived_type;

  typedef core_view<C, domain_type, map_function>              base_type;

  typedef view_operations::sequence<derived_type>              sequence_op_type;

public:
  typedef typename sequence_op_type::iterator                  iterator;
  typedef typename sequence_op_type::const_iterator            const_iterator;

  // Default constructor is currently required by cast_container_view.
  array_ro_view(void)                       = default;
  array_ro_view(array_ro_view const& other) = default;

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a view that can reference all the elements of
  ///        the container provided. The view takes ownership of the
  ///        container.
  ///
  /// @param vcont pointer to the container used to forward the operations.
  //////////////////////////////////////////////////////////////////////
  array_ro_view(view_container_type* vcont)
    : base_type(vcont, stapl::get_domain(*vcont))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a view with a restricted domain of the elements
  ///  of the container provided. The view takes ownership of the
  ///  container.
  ///
  /// @param vcont pointer to the container used to forward the operations.
  /// @param dom domain to be used by the view.
  //////////////////////////////////////////////////////////////////////
  array_ro_view(view_container_type* vcont, domain_type const& dom)
    : base_type(vcont, dom)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a view with a potentially restricted domain of
  ///  the elements of the container provided and a non-identity mapping
  ///  function. The view takes ownership of the container.
  ///
  /// @param vcont pointer to the container used to forward the operations.
  /// @param dom domain to be used by the view.
  /// @param mfunc mapping function to transform view indices to container
  ///        gids.
  //////////////////////////////////////////////////////////////////////
  array_ro_view(view_container_type* vcont, domain_type const& dom,
                map_func_type mfunc)
    : base_type(vcont, dom, mfunc)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a view that can reference all the elements of
  ///        the container provided.  The view does not take ownership
  ///        of the container.
  ///
  /// @param vcont reference to the container used to forward the operations.
  //////////////////////////////////////////////////////////////////////
  array_ro_view(view_container_type const& vcont)
    : base_type(vcont,stapl::get_domain(vcont))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a view with a restricted domain of the elements
  ///  of the container provided. The view does not take ownership of
  ///  the container.
  ///
  /// @param vcont reference to the container used to forward the operations.
  /// @param dom domain to be used by the view.
  //////////////////////////////////////////////////////////////////////
  array_ro_view(view_container_type const& vcont, domain_type const& dom)
    : base_type(vcont, dom)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a view with a potentially restricted domain of
  ///  the elements of the container provided and a non-identity mapping
  ///  function. The view does not take ownership of the container.
  ///
  /// @param vcont reference to the container used to forward the operations.
  /// @param dom domain to be used by the view.
  /// @param mfunc mapping function to transform view indices to container
  ///        gids.
  //////////////////////////////////////////////////////////////////////
  array_ro_view(view_container_type const& vcont, domain_type const& dom,
                map_func_type mfunc)
    : base_type(vcont, dom, mfunc)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a view with a potentially restricted domain of the
  /// elements of the container provided and a non-identity mapping function.
  /// The constructor accepts another @ref array_ro_view instance from which
  /// it could copy additional state. The view does not take ownership of
  /// the container.
  ///
  /// This constructor is provided to match the interfaces of other view
  /// classes. There is no additional state that needs to be copied from the
  /// reference view provided.
  ///
  /// @param vcont reference to the container used to forward the operations.
  /// @param dom domain to be used by the view.
  /// @param mfunc mapping function to transform view indices to container
  ///        gids.
  //////////////////////////////////////////////////////////////////////
  array_ro_view(view_container_type const& vcont, domain_type const& dom,
                map_func_type mfunc, array_ro_view const&)
    : base_type(vcont, dom, mfunc)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Copy constructor when the view provided is not the most
  ///        derived view.
  //////////////////////////////////////////////////////////////////////
  template<typename ...OtherOptionalParams>
  array_ro_view(array_ro_view<C, OtherOptionalParams...> const& other)
    : base_type(other.container(), other.domain(), other.mapfunc())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @internal
  //////////////////////////////////////////////////////////////////////
  void define_type(typer &t)
  {
    t.base<base_type>(*this);
  }

  //////////////////////////////////////////////////////////////////////
  /// @internal
  /// @brief use to examine this class
  //////////////////////////////////////////////////////////////////////
  void debug(char *msg=0)
  {
    std::cerr << "ARRAY_RO_VIEW " << this << " : ";
    if (msg) {
      std::cerr << msg;
    }
    std::cerr << std::endl;
    base_type::debug();
  }

}; //class array_ro_view


namespace functional {

//////////////////////////////////////////////////////////////////////
/// @brief Functor to define a functional construction of array_ro_view.
///
/// @return an array_ro_view over the passed referenced container
/// @note This functor is used to construct array_ro_view over proxy
//////////////////////////////////////////////////////////////////////
struct array_ro_view
{
  template<typename Signature>
  struct result;

  template<typename Ref>
  struct result<array_ro_view(Ref)>
  {
    typedef stapl::array_ro_view<Ref> type;
  };

  template<typename Ref>
  stapl::array_ro_view<Ref>
  operator()(Ref elem) const
  {
    return stapl::array_ro_view<Ref>(new Ref(elem));
  }
};

} // namespace functional


template<typename C, typename ...OptionalParams>
struct container_traits<array_ro_view<C, OptionalParams...>>
  : public view_traits<array_ro_view<C, OptionalParams...>>
{ };

} // namespace stapl

#endif // STAPL_VIEWS_ARRAY_RO_VIEW_HPP
