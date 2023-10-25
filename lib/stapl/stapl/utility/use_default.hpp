/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_USE_DEFAULT_HPP
#define STAPL_UTILITY_USE_DEFAULT_HPP

#include <boost/mpl/eval_if.hpp>

#include "tuple.hpp"

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Tag type detected by select_parameter metafunction to detect if a
/// class template is specified by user or if default param is used.
//////////////////////////////////////////////////////////////////////
struct use_default
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Type metafunction used by class templates with default template
/// parameters that might be lengthy in type / name.  Shortens symbol names
/// for better debug output and compile times.
///
/// @tparam Passed parameter passed to the user's class template
/// @tparam Default the type which should be used if Passed is use_default
///
/// usage:
/// template<typename T, typename Q = use_default)
/// class foo
/// {
///   // use int as default type
///   typedef typename select_parameter<Q, really_long_type>::type Q_t;
///   ...
/// };
///
/// foo<float>       // Q_t = really_long_type
/// foo<<float, int> // Q_t = int
///
/// primary template reflects Passed (Default != use_default).
//////////////////////////////////////////////////////////////////////
template<typename Passed, typename Default>
struct select_parameter
{
  using type = Passed;
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization when Passed parameter to metafunction is use_default.
///
/// reflect Default
//////////////////////////////////////////////////////////////////////
template<typename Default>
struct select_parameter<use_default, Default>
{
  using type = Default;
};


//////////////////////////////////////////////////////////////////////
/// @brief Intercept optional parameters passed with an object of type
/// @ref use_default.  If this occurs, return a default constructed
/// instance of type @ref Q.  Otherwise, forward on the passed parameter.
//////////////////////////////////////////////////////////////////////
template<typename T, typename Q,
         bool = std::is_same<use_default, typename std::decay<T>::type>::value>
struct initialize_parameter
{
  static T&& apply(T&& t)
  { return std::forward<T>(t); }
};


template<typename T, typename Q>
struct initialize_parameter<T, Q, true>
{
  static Q apply(T&&)
  { return Q(); }
};


namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Helper metafunction used by @ref compute_type_parameters to delay
///   the invocation of the type transformation metafunction call until
///   OptionalParams padded with @ref use_default types at the end if needed.
//////////////////////////////////////////////////////////////////////
template<typename DefaultParamsTuple, typename ...OptionalParams>
struct compare_type_parameters
{ };


template<typename ...DefaultParams, typename ...OptionalParams>
struct compare_type_parameters<tuple<DefaultParams...>, OptionalParams...>
  : result_of::transform2<
      tuple<OptionalParams...>, tuple<DefaultParams...>, select_parameter
    >
{ };

} // namespace detail


//////////////////////////////////////////////////////////////////////
/// @brief Metafunction that returns a tuple of types to a template
///  instantiation that should be used for a set of optional template
///  parameters. This allows writers to avoid use of default template
///  parameters, reducing error message and symbol table sizes.
///
/// @tparam DefaultParamsTuple A tuple containing a list of default values
///   for optional template parameters.
///
/// @tparam OptionalParams Any optional parameters explicitly passed to a
///   template instantiation. The size of @p DefaultParams defines the
///   maximum size of this list of parameters.
///
///  For example, given this class template with two optional parameter:
///
///  template<typename T1, typename T2, typename O1 = foo, typename O2 = bar>
///  class baz
///  {
///    typedef O1 optional1_type;
///    typedef O2 optional2_type;
///  };
///
///  Could be transformed to the following:
///
///  template<typename T1, typename T2, typename ...OptionalParams>
///  class baz
///  {
///    typedef typename compute_type_parameters<
///      tuple<O1, O2> OptionalParams...
///    >::type                                              param_types;
///
///    typedef typename tuple_element<0, param_types>::type optional1_type;
///    typedef typename tuple_element<1, param_types>::type optional2_type;
///  };
///////////////////////////////////////////////////////////////////////
template<typename DefaultParamsTuple, typename ...OptionalParams>
struct compute_type_parameters
{ };


template<typename ...DefaultParams, typename ...OptionalParams>
struct compute_type_parameters<tuple<DefaultParams...>, OptionalParams...>
{
  static_assert(
    sizeof...(OptionalParams) <= sizeof...(DefaultParams),
    "Excess Specified Type Parameters Detected"
  );

  using type =
    typename boost::mpl::eval_if_c<
      sizeof...(OptionalParams) < sizeof...(DefaultParams),
      compute_type_parameters<
        tuple<DefaultParams...>, OptionalParams..., use_default
      >,
      detail::compare_type_parameters<
        tuple<DefaultParams...>, OptionalParams...>
    >::type;
};


template <typename T, typename Default>
using default_type = typename std::conditional<
                       std::is_same<T, stapl::use_default>::value,
                       Default,
                       T
                     >::type;

} // namespace stapl

#endif // STAPL_UTILITY_USE_DEFAULT_HPP
