/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_STATIC_MATCH_HPP
#define STAPL_UTILITY_STATIC_MATCH_HPP

#include <stapl/utility/tuple.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Tag to represent that a particular item in an option will not
///        change the overall choice.
//////////////////////////////////////////////////////////////////////
struct dont_care {};


//////////////////////////////////////////////////////////////////////
/// @brief Tag class that is returned if a choice is not found for
///        a @c static_match.
//////////////////////////////////////////////////////////////////////
struct not_matched {};

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction that determines if a a choice (tuple of types)
///        conforms to an option, which may or may not contain don't cares
//////////////////////////////////////////////////////////////////////
template <typename Choice, typename Option>
struct conforms_to;

template <>
struct conforms_to<tuple<>, tuple<>>
  : public std::true_type
{ };

template <typename C, typename... Cs, typename O, typename... Os>
struct conforms_to<tuple<C, Cs...>, tuple<O, Os...>>
  : public std::integral_constant<bool,
      (std::is_same<dont_care, O>::value || std::is_same<C, O>::value) &&
      conforms_to<tuple<Cs...>, tuple<Os...>>::type::value>
{ };

}

//////////////////////////////////////////////////////////////////////
/// @brief Statically choose a type from a list of options.
///
///        A choice is a tuple of types (either true_type or false_type)
///        and the options are a tuple of pairs, where the first element
///        of the pair is a possible choice (tuple of true_type, false_type
///        or dont_care) and the second element is the final type that
///        should be emitted.
///
///        For example, options can be the following:
///          tuple<
///            pair<tuple<true_type , false_type>, foo>,
///            pair<tuple<true_type , true_type >, bar>,
///            pair<tuple<false_type, dont_care >, baz>
///          >
///
///        Which means if given (T, F) emit the type foo, if given (T, T),
///        emit the type bar and if given (F, ?), emit the type baz.
///
///        Example with the above options:
///
///        static_match<tuple<false_type, true_type>, Opts>::type == baz
///
///        Reflects the trait "type" that corresponds to the matched
///        choice, or @c not_matched if no options are matched. Note
///        that the choices are matched sequentially and if there are
///        several possible matches, only the first is returned.
//////////////////////////////////////////////////////////////////////
template <typename Choice, typename Options>
struct static_match;

template <typename Choice>
struct static_match<Choice, tuple<>>
{
  typedef not_matched type;
};

template <typename Choice, typename Option, typename... Options>
struct static_match<Choice, tuple<Option, Options...>>
{
  typedef typename std::conditional<
    // if we found the option that matches
    detail::conforms_to<Choice, typename Option::first_type>::value,

    // choose this and emit its value
    typename Option::second_type,

    // otherwise, recurse on the rest of the options
    typename static_match<Choice, tuple<Options...>>::type
  >::type type;
};

} // namespace stapl

#endif
