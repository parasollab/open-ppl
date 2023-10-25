/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_TYPE_TRAITS_IS_SHARED_PTR_HPP
#define STAPL_RUNTIME_TYPE_TRAITS_IS_SHARED_PTR_HPP

#include <type_traits>

// forward declarations

namespace std {

template<typename T>
class shared_ptr;

} // namespace std


namespace boost {

template<typename T>
class shared_ptr;

} // namespace boost


namespace stapl {

namespace runtime {

////////////////////////////////////////////////////////////////////
/// @brief Returns if @p T is a @c std::shared_ptr or @c boost::shared_ptr.
///
/// @ingroup runtimeTypeTraitsImpl
////////////////////////////////////////////////////////////////////
template<typename T>
struct is_shared_ptr_impl
: public std::false_type
{ };


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref is_shared_ptr_impl for @c std::shared_ptr.
///
/// @ingroup runtimeTypeTraitsImpl
////////////////////////////////////////////////////////////////////
template<typename T>
struct is_shared_ptr_impl<std::shared_ptr<T>>
: public std::true_type
{ };


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref is_shared_ptr_impl for @c boost::shared_ptr.
///
/// @ingroup runtimeTypeTraitsImpl
////////////////////////////////////////////////////////////////////
template<typename T>
struct is_shared_ptr_impl<boost::shared_ptr<T>>
: public std::true_type
{ };


////////////////////////////////////////////////////////////////////
/// @brief Returns if @p T is a @c std::shared_ptr<> or
///        @c boost::shared_ptr.
///
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
template<typename T>
struct is_shared_ptr
: public is_shared_ptr_impl<typename std::remove_cv<T>::type>
{ };

} // namespace runtime

} // namespace stapl

#endif
