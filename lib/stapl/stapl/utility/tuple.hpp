/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

//////////////////////////////////////////////////////////////////////
/// @file stapl/utility/tuple.hpp This file wraps the use of std::tuple,
///  and implements several of Boost.Fusion's algorithm calls using
///  std::tuple.  See Boost.Fusion's documentation for the full interface.
///  There are a few additional metafunctions that we implement,
///   documented below.
//////////////////////////////////////////////////////////////////////

#ifndef STAPL_UTILITY_TUPLE_HPP
#define STAPL_UTILITY_TUPLE_HPP

#include <type_traits>
#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/utility/tuple_mpl_adapt.hpp>
#include <boost/type_traits/integral_constant.hpp>

#include <stapl/utility/utility.hpp>
#include <stapl/utility/integer_sequence.hpp>
#include <stapl/utility/tuple/tuple_element.hpp>
#include <stapl/utility/tuple/tuple_size.hpp>
#include <stapl/utility/tuple/apply.hpp>
#include <stapl/utility/tuple/back.hpp>
#include <stapl/utility/tuple/ensure_tuple.hpp>
#include <stapl/utility/tuple/extract_1D.hpp>
#include <stapl/utility/tuple/difference.hpp>
#include <stapl/utility/tuple/discard.hpp>
#include <stapl/utility/tuple/expand_and_copy.hpp>
#include <stapl/utility/tuple/filter.hpp>
#include <stapl/utility/tuple/find_first_index.hpp>
#include <stapl/utility/tuple/fold.hpp>
#include <stapl/utility/tuple/for_each.hpp>
#include <stapl/utility/tuple/from_array.hpp>
#include <stapl/utility/tuple/from_index.hpp>
#include <stapl/utility/tuple/front.hpp>
#include <stapl/utility/tuple/ignore_index.hpp>
#include <stapl/utility/tuple/homogeneous_tuple.hpp>
#include <stapl/utility/tuple/pad_tuple.hpp>
#include <stapl/utility/tuple/pop_back.hpp>
#include <stapl/utility/tuple/pop_front.hpp>
#include <stapl/utility/tuple/print.hpp>
#include <stapl/utility/tuple/push_back.hpp>
#include <stapl/utility/tuple/push_front.hpp>
#include <stapl/utility/tuple/rearrange.hpp>
#include <stapl/utility/tuple/reverse.hpp>
#include <stapl/utility/tuple/transform.hpp>
#include <stapl/utility/tuple/tuple_contains.hpp>
#include <stapl/utility/tuple/to_index.hpp>
#include <stapl/utility/tuple/zip3.hpp>

#ifdef _STAPL
#include <stapl/runtime/serialization/tuple.hpp>
#endif

#endif // STAPL_UTILITY_TUPLE_HPP
