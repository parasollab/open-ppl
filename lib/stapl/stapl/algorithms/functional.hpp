/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_ALGORITHMS_FUNCTIONAL_HPP
#define STAPL_ALGORITHMS_FUNCTIONAL_HPP

#include <stapl/runtime/serialization_fwd.hpp>
#include <stapl/runtime/stapl_assert.hpp>

#include <stapl/paragraph/edge_container/views/edge_accessor_fwd.hpp>

#include <boost/mpl/has_xxx.hpp>

#include <limits>
#include <stapl/algorithms/identity_value.hpp>

#include <stapl/utility/tuple.hpp>
#include <stapl/skeletons/utility/wf_iter_compare.hpp>

//////////////////////////////////////////////////////////////////////
/// @file
/// @brief Contains function objects equivalent to the function objects
/// provided by STL, and specializations of @ref stapl::identity_value for the
/// function object when applicable.
///
/// The STAPL implementations of functions that have a direct STL equivalent
/// (e.g., std::plus) differ from the STL implementation in that the function
/// operator is templated to allow proxies over the type to be passed to the
/// functor.
//////////////////////////////////////////////////////////////////////

namespace stapl {

template<typename T1, typename T2>
class proxy;

template<typename T>
class cref_accessor;

template<typename T>
proxy<T, cref_accessor<T> > make_cref(T const &t);


//////////////////////////////////////////////////////////////////////
/// @brief Represent a read-only unary functor, which provides typedefs for the
///   argument and return types. Mimics the equivalent STL construct to allow
///   for compatibility.
/// @ingroup baseFunctionObjects
/// @tparam Arg Argument type of the functor.
/// @tparam Result Return type of the functor.
//////////////////////////////////////////////////////////////////////
template<typename Arg, typename Result>
struct ro_unary_function
{
  using argument_type = Arg;
  using result_type   = Result;
};


//////////////////////////////////////////////////////////////////////
/// @brief Represent a read-only binary functor, which provides typedefs for the
///   argument and return types. Mimics the equivalent STL construct to allow
///   for compatibility.
/// @ingroup baseFunctionObjects
/// @tparam Arg1 First argument type of the functor.
/// @tparam Arg2 Second argument type of the functor.
/// @tparam Result Return type of the functor.
//////////////////////////////////////////////////////////////////////
template<typename Arg1, typename Arg2, typename Result>
struct ro_binary_function
{
  using first_argument_type  = Arg1;
  using second_argument_type = Arg2;
  using result_type          = Result;
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function which returns the minimum of two values.
/// @tparam T Type of values being compared.
/// @ingroup comparatorFunctionObjects
//////////////////////////////////////////////////////////////////////
template<typename T>
struct min
  : public ro_binary_function<T, T, T>
{
  template<typename Reference1, typename Reference2>
  T operator()(Reference1 x, Reference2 y) const
  {
    if (x < y)
     return x;
    else
     return y;
  }
};


STAPL_DEFINE_IDENTITY_VALUE(min<char>, char, std::numeric_limits<char>::max())
STAPL_DEFINE_IDENTITY_VALUE(min<unsigned char>, unsigned char,
  std::numeric_limits<unsigned char>::max())
STAPL_DEFINE_IDENTITY_VALUE(min<int>, int, std::numeric_limits<int>::max())
STAPL_DEFINE_IDENTITY_VALUE(min<unsigned int>, unsigned int,
  std::numeric_limits<unsigned int>::max())
STAPL_DEFINE_IDENTITY_VALUE(min<long int>, long int,
  std::numeric_limits<long int>::max())
STAPL_DEFINE_IDENTITY_VALUE(min<long unsigned int>, long unsigned int,
  std::numeric_limits<long unsigned int>::max())
STAPL_DEFINE_IDENTITY_VALUE(min<float>, float,
  std::numeric_limits<float>::max())
STAPL_DEFINE_IDENTITY_VALUE(min<double>, double,
  std::numeric_limits<double>::max())


//////////////////////////////////////////////////////////////////////
/// @brief Work function which returns the maximum of two values.
/// @tparam T Type of values being compared.
/// @ingroup comparatorFunctionObjects
//////////////////////////////////////////////////////////////////////
template<typename T>
struct max
  : public ro_binary_function<T, T, T>
{
  template<typename Reference1, typename Reference2>
  T operator()(Reference1 x, Reference2 y) const
  {
    if (x < y)
     return y;
    else
     return x;
  }
};


STAPL_DEFINE_IDENTITY_VALUE(max<char>, char, std::numeric_limits<char>::min())
STAPL_DEFINE_IDENTITY_VALUE(max<unsigned char>, unsigned char,
  std::numeric_limits<unsigned char>::min())
STAPL_DEFINE_IDENTITY_VALUE(max<int>, int, std::numeric_limits<int>::min())
STAPL_DEFINE_IDENTITY_VALUE(max<unsigned int>, unsigned int,
                            std::numeric_limits<unsigned int>::min())
STAPL_DEFINE_IDENTITY_VALUE(max<long int>, long int,
                            std::numeric_limits<long int>::min())
STAPL_DEFINE_IDENTITY_VALUE(max<long unsigned int>, long unsigned int,
                            std::numeric_limits<long unsigned int>::min())
STAPL_DEFINE_IDENTITY_VALUE(max<float>, float,
                            std::numeric_limits<float>::min())
STAPL_DEFINE_IDENTITY_VALUE(max<double>, double,
                            std::numeric_limits<double>::min())


//////////////////////////////////////////////////////////////////////
/// @brief Work function implementing identity (returns the function argument).
/// @todo Check if uses of this can be replaced with identity.
/// @ingroup comparatorFunctionObjects
//////////////////////////////////////////////////////////////////////
struct element_map
{
  template<typename Reference>
  Reference operator()(Reference elem) const
  {
    return elem;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function which assigns the value of the first argument to the
///   second.
/// @tparam T Type of the elements being assigned.
/// @ingroup mutatingFunctionObjects
//////////////////////////////////////////////////////////////////////
template<typename T>
struct assign
{
  //TODO - factor these typedefs to an appropriate base_class
  typedef T                  first_argument_type;
  typedef T                  second_argument_type;
  typedef void               result_type;

  template<typename Reference1, typename Reference2>
  void operator()(Reference1&& x, Reference2&& y) const
  {
    y = x;
  }
};


namespace detail {

BOOST_MPL_HAS_XXX_TRAIT_DEF(set_elements_type)

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to check whether given view parameter has a
/// set_elements() method, by inspecting the reflected type
//// @p set_elements_type if one exists.
//////////////////////////////////////////////////////////////////////
template<typename View,
         bool = has_set_elements_type<typename std::decay<View>::type>::value>
struct has_set_elements
  : std::false_type
{ };


template<typename View>
struct has_set_elements<View, true>
  : View::set_elements_type
{ };

} // namespace detail


//////////////////////////////////////////////////////////////////////
/// @brief Based on boolean parameter, dispatch logical invocation of
/// the assignment operator on two view ranges either to a simple for
/// loop or the destination view's @p set_elements member function.
//////////////////////////////////////////////////////////////////////
template<bool HasSetElements>
struct coarse_assign_helper
{
  template<typename View1, typename View2>
  static void apply(View1&& x, View2&& y)
  {
    y.set_elements(std::forward<View1>(x));
  }
};


template<>
struct coarse_assign_helper<false>
{
  template<typename View0, typename View1>
  static void apply(View0&& x, View1&& y)
  {
    using iter_compare_t =
      wf_iter_compare<
        typename std::decay<View0>::type, typename std::decay<View1>::type
      >;

    iter_compare_t iter_compare(std::forward<View0>(x), std::forward<View1>(y));

    auto iter0 = x.begin();
    auto iter1 = y.begin();

    for (; iter_compare(iter0, iter1); ++iter0, ++iter1)
      *iter1 = *iter0;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Performs assignment of all values from one view to the
/// elements of another.
///
/// If the destination view (@p y), defines @p set_elements,
/// use that for assignment of the range of values from @p x.
/// Otherwise, just use a for loop.
//////////////////////////////////////////////////////////////////////
struct coarse_assign
{
  using result_type = void;

  template<typename View1, typename View2>
  void operator()(View1&& x, View2&& y) const
  {
    coarse_assign_helper<
      detail::has_set_elements<typename std::decay<View2>::type>::value
    >::apply(std::forward<View1>(x), std::forward<View2>(y));
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function that swaps the values between two views.
/// @ingroup mutatingFunctionObjects
//////////////////////////////////////////////////////////////////////
template<typename T>
struct coarse_swap_ranges
{
  template<typename View0, typename View1>
  void operator()(View0 &&x, View1 &&y)
  {
    //Note: std::swap_ranges calls std::iter_swap that has been overloaded
    //      in proxy_ops.hpp to support proxies.
    std::swap_ranges(x.begin(), x.end(), y.begin());
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function which applies the given functor to its argument.
/// @tparam Functor Functor to apply to the argument.
/// @ingroup mutatingFunctionObjects
//////////////////////////////////////////////////////////////////////
template<typename Functor>
struct apply_functor
  : private Functor
{
  typedef void result_type;

  apply_functor(Functor const& f)
    : Functor(f)
  { }

  void define_type(typer& t)
  {
    t.base<Functor>(*this);
  }

  template<typename Ref1>
  void operator()(Ref1 x) /*const*/
  {
    static_cast<Functor&>(*this)(x);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Function which applies given functor to its first argument(s),
///   and assigns the result to its last argument.
/// @tparam Functor Functor to apply to the arguments.
/// @ingroup mutatingFunctionObjects
//////////////////////////////////////////////////////////////////////
template<typename Functor>
struct transform_assign
  : private Functor
{
  typedef void result_type;

  transform_assign(Functor const& f)
    : Functor(f)
  { }

  void define_type(typer& t)
  {
    t.base<Functor>(*this);
  }

  template<typename Ref1, typename Ref2>
  void operator()(Ref1 x, Ref2 y) /*const*/
  {
    y = static_cast<Functor&>(*this)(x);
  }

  template<typename Ref1, typename Ref2, typename Ref3>
  void operator()(Ref1 x, Ref2 y, Ref3 z) /*const*/
  {
    z = static_cast<Functor&>(*this)(x, y);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Function which returns the sum of its arguments.
/// @tparam T Type of the arguments.
/// @ingroup numericFunctionObjects
//////////////////////////////////////////////////////////////////////
template<typename T>
struct plus
  : public ro_binary_function<T, T, T>
{
  template<typename Reference1, typename Reference2>
  T operator()(Reference1 x, Reference2 y) const
  { return x + y; }
};


STAPL_DEFINE_IDENTITY_VALUE(plus<char>, char, 0)
STAPL_DEFINE_IDENTITY_VALUE(plus<unsigned char>, unsigned char, 0)
STAPL_DEFINE_IDENTITY_VALUE(plus<int>, int, 0)
STAPL_DEFINE_IDENTITY_VALUE(plus<unsigned int>, unsigned int, 0)
STAPL_DEFINE_IDENTITY_VALUE(plus<long int>, long int, 0)
STAPL_DEFINE_IDENTITY_VALUE(plus<long long int>, long long int, 0)
STAPL_DEFINE_IDENTITY_VALUE(plus<long unsigned int>, long unsigned int, 0)
STAPL_DEFINE_IDENTITY_VALUE(plus<float>, float, 0.f)
STAPL_DEFINE_IDENTITY_VALUE(plus<double>, double, 0.0)

template<>                                               \
struct identity_value<plus<std::tuple<long unsigned int, long unsigned int>>,
                      std::tuple<long unsigned int, long unsigned int>>
{
  static std::tuple<long unsigned int, long unsigned int>
  value(void)
  { return std::tuple<long unsigned int, long unsigned int>(0,0); }
};

//////////////////////////////////////////////////////////////////////
/// @brief Function which returns the difference of its arguments.
/// @tparam T Type of the arguments.
/// @ingroup numericFunctionObjects
//////////////////////////////////////////////////////////////////////
template<typename T>
struct minus
  : public ro_binary_function<T, T, T>
{
  template<typename Reference1, typename Reference2>
  T operator()(Reference1 x, Reference2 y) const
  { return x - y; }
};


STAPL_DEFINE_IDENTITY_VALUE(minus<char>, char, 0)
STAPL_DEFINE_IDENTITY_VALUE(minus<unsigned char>, unsigned char, 0)
STAPL_DEFINE_IDENTITY_VALUE(minus<int>, int, 0)
STAPL_DEFINE_IDENTITY_VALUE(minus<unsigned int>, unsigned int, 0)
STAPL_DEFINE_IDENTITY_VALUE(minus<long int>, long int, 0)
STAPL_DEFINE_IDENTITY_VALUE(minus<long unsigned int>, long unsigned int, 0)
STAPL_DEFINE_IDENTITY_VALUE(minus<float>, float, 0.f)
STAPL_DEFINE_IDENTITY_VALUE(minus<double>, double, 0.0)


//////////////////////////////////////////////////////////////////////
/// @brief Function which returns the product of its arguments.
/// @tparam T Type of the arguments.
/// @ingroup numericFunctionObjects
//////////////////////////////////////////////////////////////////////
template<typename T>
struct multiplies
  : public ro_binary_function<T, T, T>
{
  template<typename Reference1, typename Reference2>
  T operator()(Reference1 x, Reference2 y) const
  { return x * y; }
};


STAPL_DEFINE_IDENTITY_VALUE(multiplies<char>, char, 1)
STAPL_DEFINE_IDENTITY_VALUE(multiplies<unsigned char>, unsigned char, 1)
STAPL_DEFINE_IDENTITY_VALUE(multiplies<int>, int, 1)
STAPL_DEFINE_IDENTITY_VALUE(multiplies<unsigned int>, unsigned int, 1)
STAPL_DEFINE_IDENTITY_VALUE(multiplies<long int>, long int, 1)
STAPL_DEFINE_IDENTITY_VALUE(multiplies<long unsigned int>, long unsigned int, 1)
STAPL_DEFINE_IDENTITY_VALUE(multiplies<float>, float, 1.0f)
STAPL_DEFINE_IDENTITY_VALUE(multiplies<double>, double, 1.0)


//////////////////////////////////////////////////////////////////////
/// @brief Function which returns the result of dividing the first argument by
///   the second.
/// @tparam T Type of the arguments.
/// @ingroup numericFunctionObjects
///
/// When called with integral types, returns the quotient. When called with
/// real-valued types, returns the exact result of the division.
//////////////////////////////////////////////////////////////////////
template<typename T>
struct divides
  : public ro_binary_function<T, T, T>
{
  template<typename Reference1, typename Reference2>
  T operator()(Reference1 x, Reference2 y) const
  { return x / y; }
};


STAPL_DEFINE_IDENTITY_VALUE(divides<char>, char, 1)
STAPL_DEFINE_IDENTITY_VALUE(divides<unsigned char>, unsigned char, 1)
STAPL_DEFINE_IDENTITY_VALUE(divides<int>, int, 1)
STAPL_DEFINE_IDENTITY_VALUE(divides<unsigned int>, unsigned int, 1)
STAPL_DEFINE_IDENTITY_VALUE(divides<long int>, long int, 1)
STAPL_DEFINE_IDENTITY_VALUE(divides<long unsigned int>, long unsigned int, 1)
STAPL_DEFINE_IDENTITY_VALUE(divides<float>, float, 1.0f)
STAPL_DEFINE_IDENTITY_VALUE(divides<double>, double, 1.0)


//////////////////////////////////////////////////////////////////////
/// @brief Function which returns the modulus of its arguments.
/// @tparam T Type of the arguments (expected to be integral).
/// @ingroup numericFunctionObjects
//////////////////////////////////////////////////////////////////////
template<typename T>
struct modulus
  : public ro_binary_function<T, T, T>
{
  template<typename Reference1, typename Reference2>
  T operator()(Reference1 x, Reference2 y) const
  { return x % y; }
};


//////////////////////////////////////////////////////////////////////
/// @brief Function which returns the negation of its argument.
/// @tparam T Type of the argument.
/// @ingroup numericFunctionObjects
//////////////////////////////////////////////////////////////////////
template<typename T>
struct negate
  : public ro_unary_function<T, T>
{
  template<typename Reference>
  T operator()(Reference x)
  { return -x; }
};


//////////////////////////////////////////////////////////////////////
/// @brief Function which returns whether its arguments are equal.
/// @tparam T Type of the arguments.
/// @ingroup comparatorFunctionObjects
//////////////////////////////////////////////////////////////////////
template<typename T>
struct equal_to
  : public ro_binary_function<T, T, bool>
{
  template<typename Reference1, typename Reference2>
  bool operator()(Reference1&& x, Reference2&& y) const
  { return x == y; }
};


//////////////////////////////////////////////////////////////////////
/// @brief Function which returns whether its arguments are not equal.
/// @tparam T Type of the arguments.
/// @ingroup comparatorFunctionObjects
//////////////////////////////////////////////////////////////////////
template<typename T>
struct not_equal_to
  : public ro_binary_function<T, T, bool>
{
  template<typename Reference1, typename Reference2>
  bool operator()(Reference1 x, Reference2 y) const
  { return x != y; }
};


//////////////////////////////////////////////////////////////////////
/// @brief Function which returns whether the first argument is greater than
///   the second.
/// @tparam T Type of the arguments.
/// @ingroup comparatorFunctionObjects
//////////////////////////////////////////////////////////////////////
template<typename T>
struct greater
  : public ro_binary_function<T, T, bool>
{
  template<typename Reference1, typename Reference2>
  bool operator()(Reference1 x, Reference2 y) const
  { return x > y; }
};

//////////////////////////////////////////////////////////////////////
/// @brief Function which returns whether the first argument is less than
///   the second.
/// @tparam T Type of the arguments.
/// @ingroup comparatorFunctionObjects
//////////////////////////////////////////////////////////////////////
template<typename T>
struct less
  : public ro_binary_function<T, T, bool>
{
  template<typename Reference1, typename Reference2>
  bool operator()(Reference1&& x, Reference2&& y) const
  { return x < y; }
};

//////////////////////////////////////////////////////////////////////
/// @brief Function which returns whether the first argument is greater than
///   or equal to the second.
/// @tparam T Type of the arguments.
/// @ingroup comparatorFunctionObjects
//////////////////////////////////////////////////////////////////////
template<typename T>
struct greater_equal
  : public ro_binary_function<T, T, bool>
{
  template<typename Reference1, typename Reference2>
  bool operator()(Reference1 x, Reference2 y) const
  { return x >= y; }
};


//////////////////////////////////////////////////////////////////////
/// @brief Function which returns whether the first argument is less than
///   or equal to the second.
/// @tparam T Type of the arguments.
/// @ingroup comparatorFunctionObjects
//////////////////////////////////////////////////////////////////////
template<typename T>
struct less_equal
  : public ro_binary_function<T, T, bool>
{
  template<typename Reference1, typename Reference2>
  bool operator()(Reference1 x, Reference2 y) const
  { return x <= y; }
};


//////////////////////////////////////////////////////////////////////
/// @brief Function which returns the logical and of its arguments.
/// @tparam T Type of the arguments.
/// @ingroup logicFunctionObjects
//////////////////////////////////////////////////////////////////////
template<typename T>
struct logical_and
  : public ro_binary_function<T, T, bool>
{
  template<typename Reference1, typename Reference2>
  bool operator()(Reference1&& x, Reference2&& y) const
  { return x && y; }
};


STAPL_DEFINE_IDENTITY_VALUE(logical_and<bool>, bool, true)
STAPL_DEFINE_IDENTITY_VALUE(logical_and<char>, char, 1)
STAPL_DEFINE_IDENTITY_VALUE(logical_and<unsigned char>, unsigned char, 1)
STAPL_DEFINE_IDENTITY_VALUE(logical_and<int>, int, 1)
STAPL_DEFINE_IDENTITY_VALUE(logical_and<unsigned int>, unsigned int, 1)
STAPL_DEFINE_IDENTITY_VALUE(logical_and<long int>, long int, 1)
STAPL_DEFINE_IDENTITY_VALUE(logical_and<long unsigned int>, long unsigned int,
  1)


//////////////////////////////////////////////////////////////////////
/// @brief Function which returns the logical or of its arguments.
/// @tparam T Type of the arguments.
/// @ingroup logicFunctionObjects
//////////////////////////////////////////////////////////////////////
template<typename T>
struct logical_or
  : public ro_binary_function<T, T, bool>
{
  template<typename Reference1, typename Reference2>
  bool operator()(Reference1 x, Reference2 y) const
  { return x || y; }
};


STAPL_DEFINE_IDENTITY_VALUE(logical_or<bool>, bool, false)
STAPL_DEFINE_IDENTITY_VALUE(logical_or<char>, char, 0)
STAPL_DEFINE_IDENTITY_VALUE(logical_or<unsigned char>, unsigned char, 0)
STAPL_DEFINE_IDENTITY_VALUE(logical_or<int>, int, 0)
STAPL_DEFINE_IDENTITY_VALUE(logical_or<unsigned int>, unsigned int, 0)
STAPL_DEFINE_IDENTITY_VALUE(logical_or<long int>, long int, 0)
STAPL_DEFINE_IDENTITY_VALUE(logical_or<long unsigned int>, long unsigned int, 0)


//////////////////////////////////////////////////////////////////////
/// @brief Function which returns the logical not of its argument.
/// @tparam T Type of the argument.
/// @ingroup logicFunctionObjects
//////////////////////////////////////////////////////////////////////
template<typename T>
struct logical_not
  : public ro_unary_function<T, bool>
{
  template<typename Reference>
  bool operator()(Reference x) const
  { return !x; }
};


//////////////////////////////////////////////////////////////////////
/// @brief Function which returns the bitwise and of its arguments.
/// @tparam T Type of the arguments.
/// @ingroup bitwiseFunctionObjects
//////////////////////////////////////////////////////////////////////
template<typename T>
struct bit_and
  : public ro_binary_function<T, T, bool>
{
  template<typename Reference1, typename Reference2>
  bool operator()(Reference1 x, Reference2 y) const
  { return x & y; }
};


//////////////////////////////////////////////////////////////////////
/// @brief Function which returns the bitwise or of its arguments.
/// @tparam T Type of the arguments.
/// @ingroup bitwiseFunctionObjects
//////////////////////////////////////////////////////////////////////
template<typename T>
struct bit_or
  : public ro_binary_function<T, T, bool>
{
  template<typename Reference1, typename Reference2>
  bool operator()(Reference1 x, Reference2 y) const
  { return x | y; }
};


//////////////////////////////////////////////////////////////////////
/// @brief Function which returns the bitwise xor of its arguments.
/// @tparam T Type of the arguments.
/// @ingroup bitwiseFunctionObjects
//////////////////////////////////////////////////////////////////////
template<typename T>
struct bit_xor
  : public ro_binary_function<T, T, bool>
{
  template<typename Reference1, typename Reference2>
  bool operator()(Reference1 x, Reference2 y) const
  { return x ^ y; }
};


namespace detail {

BOOST_MPL_HAS_XXX_TRAIT_DEF(argument_type)
BOOST_MPL_HAS_XXX_TRAIT_DEF(first_argument_type)

//////////////////////////////////////////////////////////////////////
/// @brief Base class for @ref unary_negate the reflects the type
///   @p argument_type if defined in the template.
/// @ingroup logicFunctionObjects
//////////////////////////////////////////////////////////////////////
template<typename Predicate, bool = has_argument_type<Predicate>::value>
struct unary_negate_helper
{ };


template<typename Predicate>
struct unary_negate_helper<Predicate, true>
{
  using argument_type = typename Predicate::argument_type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Base class for @ref binary_negate the reflects the type
///   @p first_argument_type and @p second_argument_type
///   if they are defined in the template.  It's assumed that the two
///   argument type parameters either both exist or both do not, so we
///   only check for first_argument_type.
/// @ingroup logicFunctionObjects
//////////////////////////////////////////////////////////////////////
template<typename Predicate, bool = has_first_argument_type<Predicate>::value>
struct binary_negate_helper
{ };


template<typename Predicate>
struct binary_negate_helper<Predicate, true>
{
  using first_argument_type  = typename Predicate::first_argument_type;
  using second_argument_type = typename Predicate::second_argument_type;
};

} // namespace detail


//////////////////////////////////////////////////////////////////////
/// @brief Function which returns whether its argument was not matched by the
///   given predicate.
/// @tparam Predicate Function which returns @c true for matched arguments.
/// @ingroup logicFunctionObjects
//////////////////////////////////////////////////////////////////////
template<typename Predicate>
struct unary_negate
  : public detail::unary_negate_helper<Predicate>,
    private Predicate
{
  typedef bool result_type;

  explicit
  unary_negate(Predicate predicate)
    : Predicate(std::move(predicate))
  { }

  void define_type(typer& t)
  {
    t.base<Predicate>(*this);
  }

  template<typename Reference>
  bool operator()(const Reference x) const
  {
    return !static_cast<Predicate const&>(*this)(x);
  }
}; // struct unary_negate


//////////////////////////////////////////////////////////////////////
/// @brief Construct a @ref unary_negate functor with the given predicate.
/// @param pred The predicate used to construct the functor.
/// @return An instance of unary_negate using the given predicate.
/// @ingroup logicFunctionObjects
//////////////////////////////////////////////////////////////////////
template<typename Predicate>
unary_negate<Predicate>
not1(Predicate const& pred)
{
  return unary_negate<Predicate>(pred);
}


//////////////////////////////////////////////////////////////////////
/// @brief Function which returns whether its arguments were not matched by the
///   given predicate.
/// @tparam Predicate Function which returns @c true for matched arguments.
/// @ingroup logicFunctionObjects
//////////////////////////////////////////////////////////////////////
template<typename Predicate>
struct binary_negate
  : public detail::binary_negate_helper<Predicate>,
    private Predicate
{
  typedef bool result_type;

  explicit
  binary_negate(Predicate const& pred)
    : Predicate(pred)
  { }

  void define_type(typer& t)
  {
    t.base<Predicate>(*this);
  }

  template<typename Reference1, typename Reference2>
  bool operator()(Reference1 x, Reference2 y) const
  {
    return !static_cast<Predicate const&>(*this)(x, y);
  }
}; // struct binary_negate


//////////////////////////////////////////////////////////////////////
/// @brief Construct a @ref binary_negate functor with the given predicate.
/// @param pred The predicate used to construct the functor.
/// @return An instance of binary_negate using the given predicate.
/// @ingroup logicFunctionObjects
//////////////////////////////////////////////////////////////////////
template<typename Predicate>
binary_negate<Predicate>
not2(Predicate const& pred)
{
  return binary_negate<Predicate>(pred);
}


//TODO
// adaptors pointers functions
//

//TODO
//(not standard but gcc helper), Identity, SelectFirst/Second(pair)
//

//////////////////////////////////////////////////////////////////////
/// @brief Function which returns its argument.
/// @tparam T Type of the argument.
/// @ingroup logicFunctionObjects
/// @todo Non-const function possibly not needed.
///
/// Instances of this struct with the same template parameter all compare equal.
//////////////////////////////////////////////////////////////////////
template<typename T>
struct identity
  : public ro_unary_function<T, T>
{
  template<typename Ref1>
  T operator()(Ref1 x)
  {
    return x;
  }

  template<typename Ref1>
  T operator()(Ref1 x) const
  {
    return x;
  }

  bool operator==(identity const& other) const
  {
    return true;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Function which returns its argument.
///
/// The difference between this workfunction and @ref identity is in
/// the fact that the type of the argument is not known a priori.
///
/// @note For the special case of @c identity_op over paragraph edge
/// values, the proxy is unwrapped to the value underneath.
///
/// @ingroup logicFunctionObjects
//////////////////////////////////////////////////////////////////////
struct identity_op
{
  template <typename T>
  struct result;

  template<typename Ref1>
  struct result<const identity_op(Ref1)>
  {
    typedef Ref1 type;
  };

  template<typename Ref1>
  struct result<identity_op(Ref1)>
  {
    typedef Ref1 type;
  };

  template<typename V, typename T>
  struct result<identity_op(proxy<V, edge_accessor<T> >)>
  {
    typedef V type;
  };

  template<typename V, typename T>
  struct result<const identity_op(proxy<V, edge_accessor<T> >)>
  {
    typedef V type;
  };

  template<typename Ref1>
  typename result<identity_op(Ref1)>::type
  operator()(Ref1 x)
  { return x; }

  template<typename Ref1>
  typename result<identity_op(Ref1)>::type
  operator()(Ref1 x) const
  { return x; }

  bool operator==(identity_op const&) const
  {
    return true;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor which returns its argument (which is a reference).
/// @tparam b True if this functor does not create a proxy.
/// @ingroup refFunctionObjects
///
/// This is the primary template for wrap_ref<false>.
//////////////////////////////////////////////////////////////////////
template<bool b>
struct wrap_ref
{
  template<typename T>
  T& operator()(T& t)
  {
    return t;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor which wraps its argument in a @c stapl::proxy using a
///   @c cref_accessor.
/// @ingroup refFunctionObjects
//////////////////////////////////////////////////////////////////////
template<>
struct wrap_ref<false>
{
  template<typename T>
  proxy<T, cref_accessor<T> >
  operator()(T const& t)
  {
    return make_cref(t);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Functor which serializes a reference.
/// @tparam b True if this functor is used.
/// @ingroup refFunctionObjects
///
/// This is the primary template for pack_ref<false>. Calling this functor
/// will cause a runtime assert.
//////////////////////////////////////////////////////////////////////
template<bool b>
struct pack_ref
{
  template<typename T>
  void operator()(T&, typer&)
  {
    stapl_assert(0, "proxy binder define_types not working, need proxy_holder");
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor which serializes a reference using a given @ref typer.
/// @ingroup refFunctionObjects
//////////////////////////////////////////////////////////////////////
template<>
struct pack_ref<false>
{
  template<typename T>
  void operator()(T& t, typer& mytyper)
  {
    mytyper.member(t);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor representing the result of binding the first argument of the
///   given operation to the stored value.
/// @tparam Operation Operation to bind.
/// @tparam Stored Type of the stored value.
/// @tparam b_is_stapl_ref True if the Stored type is a @ref stapl::proxy.
/// @ingroup bindFunctionObjects
///
/// @todo These are old-style binders, need to bring in line with std::bind.
//////////////////////////////////////////////////////////////////////
template<typename Operation, typename Stored, bool b_is_stapl_ref>
class binder1st
  : public ro_unary_function<typename Operation::second_argument_type,
                             typename Operation::result_type>,
    private Operation
{
private:
  typedef ro_unary_function<
    typename Operation::second_argument_type,
    typename Operation::result_type>               base_t;

  Stored m_value;

  Operation const& op(void) const
  {
    return *this;
  }

public:
  typedef typename base_t::argument_type   argument_type;
  typedef typename base_t::result_type     result_type;

  binder1st(Operation const& op, Stored const& val)
    : Operation(op), m_value(val)
  { }

  void define_type(typer& t)
  {
    t.base<Operation>(*this);

    pack_ref<b_is_stapl_ref>()(m_value, t);
  }

  template<typename Reference>
  result_type
  operator()(Reference x) const
  {
    return op()(wrap_ref<b_is_stapl_ref>()(m_value), x);
  }
}; // class binder1st


//////////////////////////////////////////////////////////////////////
/// @brief Functor representing the result of binding the second argument of the
///   given operation to the stored value.
/// @tparam Operation Operation to bind.
/// @tparam Stored Type of the stored value.
/// @tparam b_is_stapl_ref True if the Stored type is a @ref stapl::proxy.
/// @ingroup bindFunctionObjects
///
/// @todo These are old-style binders, need to bring in line with std::bind.
//////////////////////////////////////////////////////////////////////
template<typename Operation, typename Stored, bool b_is_stapl_ref>
class binder2nd
  : public ro_unary_function<typename Operation::first_argument_type,
                             typename Operation::result_type>,
    private Operation
{
private:
  typedef ro_unary_function<
    typename Operation::first_argument_type,
    typename Operation::result_type>               base_t;

  Stored m_value;

  Operation const& op(void) const
  {
    return *this;
  }

public:
  typedef typename base_t::argument_type  argument_type;
  typedef typename base_t::result_type    result_type;

  binder2nd(Operation const& op, Stored const& val)
    : Operation(op), m_value(val)
  { }

  void define_type(typer& t)
  {
    t.base<Operation>(*this);

    pack_ref<b_is_stapl_ref>()(m_value, t);
  }

  template<typename Reference>
  result_type
  operator()(Reference x) const
  {
    return op()(x, wrap_ref<b_is_stapl_ref>()(m_value));
  }
}; // class binder2nd


//////////////////////////////////////////////////////////////////////
/// @brief Functor representing the result of binding the third argument of the
///   given operation to the stored value.
/// @tparam Operation Operation to bind.
/// @tparam Stored Type of the stored value.
/// @tparam b_is_stapl_ref True if the Stored type is a @ref stapl::proxy.
/// @ingroup bindFunctionObjects
///
/// @todo These are old-style binders, need to bring in line with std::bind.
/// @todo Needs to inherit properly from ro_binary_function
//////////////////////////////////////////////////////////////////////
template<typename Operation, typename Stored, bool b_is_stapl_ref>
class binder3rd
  :  private Operation
{
private:
  Stored m_value;

  Operation const& op(void) const
  {
    return *this;
  }

public:
  typedef typename Operation::result_type result_type;

  binder3rd(Operation const& op, Stored const& val)
    : Operation(op), m_value(val)
  { }

  void define_type(typer& t)
  {
    t.base<Operation>(*this);

    pack_ref<b_is_stapl_ref>()(m_value, t);
  }

  template<typename Reference1, typename Reference2>
  result_type
  operator()(Reference1 x, Reference2 y) const
  {
    return op()(x, y, wrap_ref<b_is_stapl_ref>()(m_value));
  }
}; // class binder3rd


namespace result_of {

//////////////////////////////////////////////////////////////////////
/// @brief Defines the type of a @ref binder1st instantiated with the given
///   parameters.
/// @tparam Operation Operation to bind.
/// @tparam T Type of bound argument.
/// @ingroup bindFunctionObjects
//////////////////////////////////////////////////////////////////////
template<typename Operation, typename T>
struct bind1st
{
  typedef binder1st<Operation, T, false> type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Defines the type of a @ref binder1st instantiated with the given
///   parameters, where the bound argument is a \ref stapl::proxy.
/// @tparam Operation Operation to bind.
/// @tparam T Type of bound argument.
/// @tparam A Accessor for the proxy.
/// @ingroup bindFunctionObjects
//////////////////////////////////////////////////////////////////////
template<typename Operation, typename T, typename A>
struct bind1st<Operation, proxy<T, A> >
{
  typedef binder1st<Operation, proxy<T,A>, true> type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Defines the type of a @ref binder2nd instantiated with the given
///   parameters.
/// @tparam Operation Operation to bind.
/// @tparam T Type of bound argument.
/// @ingroup bindFunctionObjects
//////////////////////////////////////////////////////////////////////
template<typename Operation, typename T>
struct bind2nd
{
  typedef binder2nd<Operation, T, false> type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Defines the type of a @ref binder2nd instantiated with the given
///   parameters, where the bound argument is a \ref stapl::proxy.
/// @tparam Operation Operation to bind.
/// @tparam T Type of bound argument.
/// @tparam A Accessor for the proxy.
/// @ingroup bindFunctionObjects
//////////////////////////////////////////////////////////////////////
template<typename Operation, typename T, typename A>
struct bind2nd<Operation, proxy<T,A> >
{
  typedef binder2nd<Operation, proxy<T,A>, true> type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Defines the type of a @ref binder2nd instantiated with the given
///   parameters.
/// @tparam Operation Operation to bind.
/// @tparam T Type of bound argument.
/// @ingroup bindFunctionObjects
//////////////////////////////////////////////////////////////////////
template<typename Operation, typename T>
struct bind3rd
{
  typedef binder3rd<Operation, T, false> type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Defines the type of a @ref binder2nd instantiated with the given
///   parameters, where the bound argument is a \ref stapl::proxy.
/// @tparam Operation Operation to bind.
/// @tparam T Type of bound argument.
/// @tparam A Accessor for the proxy.
/// @ingroup bindFunctionObjects
//////////////////////////////////////////////////////////////////////
template<typename Operation, typename T, typename A>
struct bind3rd<Operation, proxy<T, A> >
{
  typedef binder3rd<Operation, proxy<T,A>, true> type;
};

} // namespace result_of


//////////////////////////////////////////////////////////////////////
/// @brief Bind the first argument of @c fn to @c x.
/// @param fn The function to bind.
/// @param x The value for the bound argument.
/// @return A function object representing the bound function.
/// @ingroup bindFunctionObjects
/// @bug The binder returned should hold the value by proxy.
//////////////////////////////////////////////////////////////////////
template<typename Operation, typename T>
binder1st<Operation, T, false>
bind1st(Operation const& fn, T const& x)
{
  return binder1st<Operation, T, false>(fn, x);
}


//////////////////////////////////////////////////////////////////////
/// @brief Bind the first argument of @c fn to @c x.
/// @param fn The function to bind.
/// @param p The value for the bound argument, which is a @ref stapl::proxy.
/// @return A function object representing the bound function.
/// @ingroup bindFunctionObjects
/// @bug The binder returned should hold the value by proxy.
//////////////////////////////////////////////////////////////////////
template<typename Operation, typename T, typename A>
binder1st<Operation, proxy<T,A>, true>
bind1st(Operation const& fn, proxy<T,A> p)
{
  return binder1st<Operation, proxy<T,A>, true>(fn, p);
}


//////////////////////////////////////////////////////////////////////
/// @brief Bind the second argument of @c fn to @c x.
/// @param fn The function to bind.
/// @param x The value for the bound argument.
/// @return A function object representing the bound function.
/// @ingroup bindFunctionObjects
/// @bug The binder returned should hold the value by proxy.
//////////////////////////////////////////////////////////////////////
template<typename Operation, typename T>
binder2nd<Operation, T, false>
bind2nd(Operation const& fn, T const& x)
{
  return binder2nd<Operation, T, false>(fn, x);
}


//////////////////////////////////////////////////////////////////////
/// @brief Bind the second argument of @c fn to @c x.
/// @param fn The function to bind.
/// @param p The value for the bound argument, which is a @ref stapl::proxy.
/// @return A function object representing the bound function.
/// @ingroup bindFunctionObjects
/// @bug The binder returned should hold the value by proxy.
//////////////////////////////////////////////////////////////////////
template<typename Operation, typename T, typename A>
binder2nd<Operation, proxy<T,A>, true>
bind2nd(Operation const& fn, proxy<T,A> p)
{
  return binder2nd<Operation, proxy<T,A>, true>(fn, p);
}


//////////////////////////////////////////////////////////////////////
/// @brief Bind the third argument of @c fn to @c x.
/// @param fn The function to bind.
/// @param x The value for the bound argument.
/// @return A function object representing the bound function.
/// @ingroup bindFunctionObjects
/// @bug The binder returned should hold the value by proxy.
/// @todo This returns a binary, not unary, function, and should be generalized.
//////////////////////////////////////////////////////////////////////
template<typename Operation, typename T>
binder3rd<Operation, T, false>
bind3rd(Operation const& fn, T const& x)
{
  return binder3rd<Operation, T, false>(fn, x);
}


//////////////////////////////////////////////////////////////////////
/// @brief Bind the third argument of @c fn to @c x.
/// @param fn The function to bind.
/// @param p The value for the bound argument, which is a @ref stapl::proxy.
/// @return A function object representing the bound function.
/// @ingroup bindFunctionObjects
/// @todo This returns a binary, not unary, function, and should be generalized.
//////////////////////////////////////////////////////////////////////
template<typename Operation, typename T, typename A>
binder3rd<Operation, proxy<T,A>, true>
bind3rd(Operation const& fn, proxy<T,A> p)
{
  return binder3rd<Operation, proxy<T,A>, true>(fn, p);
}

} // namespace stapl

#endif
