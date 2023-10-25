/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_TYPE_TRAITS_POLYMORPHIC_HPP
#define STAPL_RUNTIME_TYPE_TRAITS_POLYMORPHIC_HPP

#include "type_id.hpp"
#include "../exception.hpp"
#include <tuple>
#include <type_traits>
#include <utility>
#include <boost/mpl/has_xxx.hpp>

namespace stapl {

////////////////////////////////////////////////////////////////////
/// @brief Provide traits for RMIs with arbitrary types.
///
/// @ingroup ARMITypeTraits
////////////////////////////////////////////////////////////////////
template<typename T>
struct derived_types;


namespace runtime {

////////////////////////////////////////////////////////////////////
/// @brief Returns if a type @p T has @c polymorphic_callable defined.
///
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
BOOST_MPL_HAS_XXX_TRAIT_DEF(typelist_type)


////////////////////////////////////////////////////////////////////
/// @brief Returns if a type is a base class of a polymorphic hierarchy.
///
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
template<typename T>
struct is_known_polymorphic
: public has_typelist_type<derived_types<typename std::remove_cv<T>::type>>
{ };


namespace polymorphic_impl {

////////////////////////////////////////////////////////////////////
/// @brief Scans a @c std::tuple of types (@p T) and calls @c f.apply<U>() if
///        and only if the @ref type_id matches a type in @p T.
///
/// @ingroup runtimeTypeTraitsImpl
////////////////////////////////////////////////////////////////////
template<typename T>
struct apply_impl
{
  template<typename R, typename F>
  static R apply(F&&, const type_id) noexcept
  {
    STAPL_RUNTIME_ERROR("Type not found in typelist.");
    return R();
  }
};


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref apply_impl for @c std::tuple.
///
/// @ingroup runtimeTypeTraitsImpl
////////////////////////////////////////////////////////////////////
template<typename T, typename... Types>
struct apply_impl<std::tuple<T, Types...>>
{
  template<typename R, typename F>
  static R apply(F&& f, const type_id tid)
  {
    const auto id = T::polymorphic_type_id();

    if (tid==id)
      return std::forward<F>(f).template operator()<T>(); // found the type

    // else invoke for the next type
    return apply_impl<std::tuple<Types...>>::template apply<R>(
             std::forward<F>(f), tid);
  }
};

} // namespace polymorphic_impl


////////////////////////////////////////////////////////////////////
/// @brief Scans the tuple of types @p Tuple and calls @c f() for the type that
///        matches the type id @p tid.
///
/// @tparam R Return type of @p f.
///
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
template<typename R, typename Tuple, typename F>
R polymorphic_apply(F&& f, const type_id tid)
{
  return polymorphic_impl::apply_impl<Tuple>::template apply<R>(
           std::forward<F>(f), tid);
}

} // namespace runtime

} // namespace stapl


////////////////////////////////////////////////////////////////////
/// @brief Declares the current type as a type in a polymorphic hierarchy.
///
/// Usage example:
/// @code
/// class B
/// {
/// public:
///   virtual ~B();
///
///   STAPL_POLYMORPHIC_TYPE()
/// };
///
/// class D1
/// : public B
/// {
/// public:
///   STAPL_POLYMORPHIC_TYPE()
/// };
///
/// class D2
/// : public B
/// {
/// public:
///   STAPL_POLYMORPHIC_TYPE()
/// }
///
/// namespace stapl {
///
/// template<>
/// struct derived_types<B>
/// {
///   typedef std::tuple<D1, D2> typelist_type;
/// };
///
/// } // namespace stapl
/// @endcode
///
/// @warning This macro has to be placed in the body of the class or struct
///          after a @c public access specifier.
///
/// @ingroup ARMITypeTraits
////////////////////////////////////////////////////////////////////
#define STAPL_POLYMORPHIC_TYPE()                                      \
  static stapl::type_id polymorphic_type_id(void) noexcept            \
  { return stapl::type_id(&polymorphic_type_id); }                    \
  virtual stapl::type_id get_polymorphic_type_id(void) const noexcept \
  { return polymorphic_type_id(); }

#endif
