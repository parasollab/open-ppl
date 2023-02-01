/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef TEST_EXPECT_HPP
#define TEST_EXPECT_HPP

#include <iostream>
#include <functional>
#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/utility/tuple/print.hpp>
#include <stapl/utility/integer_sequence.hpp>

namespace stapl {

template <typename... T>
std::ostream& operator<<(std::ostream& os, stapl::tuple<T...> const& tuple)
{
  return stapl::print_tuple(os, tuple);
}

namespace tests {
namespace tests_impl {

//////////////////////////////////////////////////////////////////////
/// @brief A move-only wrapper around a stream-like object that prints
/// a '\n' when destroyed.
//////////////////////////////////////////////////////////////////////
template<class Stream>
struct test_stream
{
private:
  Stream* os;

  // The type of io manipulators like std::endl, etc.
  using manip_t = Stream& (&)(Stream&);

public:
  test_stream(Stream* stream)
    : os(stream)
  { }

  test_stream(Stream& stream)
    : os(stream)
  { }

  test_stream(test_stream const&) = delete;

  test_stream(test_stream&& other)
    : os(other.os)
  {
    other.os = nullptr;
  }

  test_stream& operator=(test_stream const&) = delete;
  test_stream& operator=(test_stream&& other) = delete;

  ~test_stream()
  {
    if (os != nullptr)
      *os << '\n';
  }

  template<class T>
  test_stream&& operator<<(T&& t) &&
  {
    return *os << std::forward<T>(t), std::move(*this);
  }

  test_stream&& operator<<(manip_t m) &&
  {
    return m(*os), std::move(*this);
  }

  void swap(test_stream& other)
  {
    std::swap(other.os, os);
  }
};

template<class Stream>
void swap(test_stream<Stream>& a, test_stream<Stream>& b)
{
  return a.swap(b);
}

using test_ostream = test_stream<std::ostream>;

/// The general case
template <typename Comp, typename T, typename U>
test_ostream expect(Comp&& comp,
                     std::string const& comp_str,
                     T&& computed,
                     U&& expected_value)
{
  if (comp(computed, expected_value)) {
    return {&(std::cout << "\x1b[;32m[PASSED]\x1b[;0m ")};
  } else {
    return {&(std::cout << "\x1b[;31m[FAILED]\x1b[;0m !(" << computed
                      << comp_str << expected_value << ")")};
  }
}

} // namespace tests_impl

using tests_impl::test_ostream;

template <typename T, typename U>
test_ostream expect_eq(T&& computed, U&& expected_value)
{
  return tests_impl::expect(std::equal_to<typename std::decay<T>::type>(),
                            "==",
                            std::forward<T>(computed),
                            std::forward<U>(expected_value));
}

template <typename T, typename U>
test_ostream expect_ne(T&& computed, U&& expected_value)
{
  return tests_impl::expect(std::not_equal_to<typename std::decay<T>::type>(),
                            "!=",
                            std::forward<T>(computed),
                            std::forward<U>(expected_value));
}

template <typename T, typename U>
test_ostream expect_lt(T&& computed, U&& expected_value)
{
  return tests_impl::expect(std::less<typename std::decay<T>::type>(),
                            "<",
                            std::forward<T>(computed),
                            std::forward<U>(expected_value));
}

template <typename T, typename U>
test_ostream expect_le(T&& computed, U&& expected_value)
{
  return tests_impl::expect(std::less_equal<typename std::decay<T>::type>(),
                            "<=",
                            std::forward<T>(computed),
                            std::forward<U>(expected_value));
}

template <typename T, typename U>
test_ostream expect_gt(T&& computed, U&& expected_value)
{
  return tests_impl::expect(std::greater<typename std::decay<T>::type>(),
                            ">",
                            std::forward<T>(computed),
                            std::forward<U>(expected_value));
}

template <typename T, typename U>
test_ostream expect_ge(T&& computed, U&& expected_value)
{
  return tests_impl::expect(std::greater_equal<typename std::decay<T>::type>(),
                            ">=",
                            std::forward<T>(computed),
                            std::forward<U>(expected_value));
}

test_ostream expect(bool computed)
{
  return expect_eq(computed, true);
}
} // namespace tests
} // namespace stapl

#endif // TEST_EXPECT_HPP
