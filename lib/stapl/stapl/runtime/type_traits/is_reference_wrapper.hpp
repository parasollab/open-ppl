/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_TYPE_TRAITS_IS_REFERENCE_WRAPPER_HPP
#define STAPL_RUNTIME_TYPE_TRAITS_IS_REFERENCE_WRAPPER_HPP

#include <type_traits>

// forward declarations

namespace std {

template<typename T>
class reference_wrapper;

} // namespace std


namespace boost {

template<typename T>
class reference_wrapper;

} // namespace boost


namespace stapl {

namespace runtime {

////////////////////////////////////////////////////////////////////
/// @brief Returns if @p T is a @c std::reference_wrapper<> or
///        @c boost::reference_wrapper.
///
/// @ingroup runtimeTypeTraitsImpl
////////////////////////////////////////////////////////////////////
template<typename T>
struct is_reference_wrapper_impl
: public std::false_type
{ };


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref is_reference_wrapper_impl for
///        @c std::reference_wrapper.
///
/// @ingroup runtimeTypeTraitsImpl
////////////////////////////////////////////////////////////////////
template<typename T>
struct is_reference_wrapper_impl<std::reference_wrapper<T>>
: public std::true_type
{ };


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref is_reference_wrapper_impl for
///        @c boost::reference_wrapper.
///
/// @ingroup runtimeTypeTraitsImpl
////////////////////////////////////////////////////////////////////
template<typename T>
struct is_reference_wrapper_impl<boost::reference_wrapper<T>>
: public std::true_type
{ };


////////////////////////////////////////////////////////////////////
/// @brief Returns if @p T is a @c std::reference_wrapper<> or
///        @c boost::reference_wrapper.
///
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
template<typename T>
struct is_reference_wrapper
: public is_reference_wrapper_impl<typename std::remove_cv<T>::type>
{ };

} // namespace runtime

} // namespace stapl

#endif
