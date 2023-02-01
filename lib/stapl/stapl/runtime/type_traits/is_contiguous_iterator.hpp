/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_TYPE_TRAITS_IS_CONTIGUOUS_ITERATOR_HPP
#define STAPL_RUNTIME_TYPE_TRAITS_IS_CONTIGUOUS_ITERATOR_HPP

#include <cstddef>
#include <iterator>
#include <type_traits>

#include <string>
#include <valarray>
#include <vector>

namespace stapl {

namespace runtime {

////////////////////////////////////////////////////////////////////
/// @brief If @p T is an iterator to a container that its elements are in
///        contiguous space it is equivalent to @c std::true_type, for any other
///        iterator it is @c std::false_type.
///
/// @ingroup runtimeTypeTraits
///
/// @todo @c std::array iterator recognition depends on the fact that it is
///       probably a pointer.
/// @todo @c std::vector iterator recognition will fail for vectors with a non
///       default allocator.
////////////////////////////////////////////////////////////////////
template<typename T>
struct is_contiguous_iterator
: public std::integral_constant<
           bool,
           (std::is_pointer<T>::value ||
            std::is_same<
              T,
              decltype(
                std::begin(
                  std::declval<
                    std::valarray<typename std::iterator_traits<T>::value_type>
                  >()
                )
              )
            >::value                  ||
            std::is_same<
              T,
              typename std::vector<
                typename std::iterator_traits<T>::value_type
              >::const_iterator
            >::value                  ||
            std::is_same<
              T,
              typename std::vector<
                typename std::iterator_traits<T>::value_type
              >::iterator
            >::value                  ||
         std::is_same<
              T,
              std::string::const_iterator
            >::value                  ||
            std::is_same<
              T,
              std::string::iterator
            >::value)
         >
{ };

} // namespace runtime

} // namespace stapl

#endif
