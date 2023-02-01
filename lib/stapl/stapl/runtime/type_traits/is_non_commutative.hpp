/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_TYPE_TRAITS_IS_NON_COMMUTATIVE_HPP
#define STAPL_RUNTIME_TYPE_TRAITS_IS_NON_COMMUTATIVE_HPP

#include <type_traits>
#include <utility>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Wrapper class for a non-commutative binary operator.
///
/// @related non_commutative()
/// @see non_commutative
/// @ingroup ARMITags
//////////////////////////////////////////////////////////////////////
template<typename BinaryOperation>
class non_commutative_wrapper
: public BinaryOperation
{
public:
  template<typename T>
  explicit non_commutative_wrapper(T&& op) noexcept
  : BinaryOperation(std::forward<T>(op))
  { }
};


//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref non_commutative_wrapper for non-commutative
///        functions.
///
/// @related non_commutative()
/// @see non_commutative
/// @ingroup ARMITags
//////////////////////////////////////////////////////////////////////
template<typename R, typename T1, typename T2>
class non_commutative_wrapper<R(T1, T2)>
{
private:
  using fun_ptr_type = R(*)(T1, T2);

  fun_ptr_type m_f;

public:
  constexpr explicit non_commutative_wrapper(R (*f)(T1, T2)) noexcept
  : m_f(f)
  { }

  operator fun_ptr_type(void) const noexcept
  { return m_f; }
};


//////////////////////////////////////////////////////////////////////
/// @brief Declares binary operator @p op as non-commutative.
///
/// @see is_non_commutative
/// @related non_commutative_wrapper
/// @ingroup ARMITags
//////////////////////////////////////////////////////////////////////
template<typename BinaryOperation>
constexpr non_commutative_wrapper<typename std::decay<BinaryOperation>::type>
non_commutative(BinaryOperation&& op) noexcept
{
  return non_commutative_wrapper<
          typename std::decay<BinaryOperation>::type
         >{std::forward<BinaryOperation>(op)};
}


//////////////////////////////////////////////////////////////////////
/// @brief Declares function @p f as non-commutative.
///
/// @see is_non_commutative
/// @related non_commutative_wrapper
/// @ingroup ARMITags
//////////////////////////////////////////////////////////////////////
template<typename R, typename T1, typename T2>
constexpr non_commutative_wrapper<R(T1, T2)>
non_commutative(R (*f)(T1, T2)) noexcept
{
  return non_commutative_wrapper<R(T1, T2)>{f};
}


//////////////////////////////////////////////////////////////////////
/// @brief Detects if @p BinaryOperation is non-commutative.
///
/// @ingroup ARMITypeTraitsImpl
//////////////////////////////////////////////////////////////////////
template<typename BinaryOperation>
struct is_non_commutative_impl
: public std::false_type
{ };


//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref is_non_commutative_impl for non-commutative
///        operators.
///
/// @ingroup ARMITypeTraitsImpl
//////////////////////////////////////////////////////////////////////
template<typename... T>
struct is_non_commutative_impl<non_commutative_wrapper<T...>>
: public std::true_type
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Detects if @p BinaryOperation is a non-commutative operation.
///
/// @ingroup ARMITypeTraits
//////////////////////////////////////////////////////////////////////
template<typename BinaryOperation>
struct is_non_commutative
: public is_non_commutative_impl<
           typename std::remove_cv<BinaryOperation>::type
         >
{ };

} // namespace stapl

#endif
