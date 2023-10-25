/*
 // Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
 // component of the Texas A&M University System.

 // All rights reserved.

 // The information and source code contained herein is the exclusive
 // property of TEES and may not be disclosed, examined or reproduced
 // in whole or in part without explicit written authorization from TEES.
 */

#ifndef STAPL_UTILITY_HAS_MEMBER_FUNCTION_HPP
#define STAPL_UTILITY_HAS_MEMBER_FUNCTION_HPP

#include <boost/mpl/bool.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// \brief Macro which creates a type metafunction that deduces
/// whether a given struct/class defines a member function with a specified
/// name and signature.
///
/// @param func Name of member function to detect.
/// @param name Name of created metafunction.
///
/// @tparam T    Class type to inspect for existence of @p func.
/// @tparam Sign The type signature of @p func.
///
/// @ingroup utility

/// @note This approach does not detect the member function if it is defined in
/// a base class of T.
///
/// \note This uses esoteric functionality of the standard wrt to SFINAE. The
/// approach is taken from: http://stackoverflow.com/questions/257288/
////////////////////////////////////////////////////////////////////////
#define STAPL_HAS_MEMBER_FUNCTION(func, name)                       \
  template<typename T, typename Signature>                          \
  struct name                                                       \
  {                                                                 \
    typedef char yes[1];                                            \
    typedef char no[2];                                             \
                                                                    \
    template <typename U, U>                                        \
    struct type_check;                                              \
                                                                    \
    template <typename _1>                                          \
    static yes &chk(type_check<Signature, &_1::func> *);            \
                                                                    \
    template <typename   >                                          \
    static no &chk(...);                                            \
                                                                    \
    static bool const value = sizeof(chk<T>(0)) == sizeof(yes);     \
                                                                    \
    typedef boost::mpl::bool_<value> type;                          \
  }
} // namespace stapl

#endif // STAPL_UTILITY_HAS_MEMBER_FUNCTION_HPP
