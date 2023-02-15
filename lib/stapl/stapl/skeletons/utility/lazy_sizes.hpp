/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_UTILITY_LAZY_SIZES_HPP
#define STAPL_SKELETONS_UTILITY_LAZY_SIZES_HPP

namespace stapl {
namespace skeletons {

//////////////////////////////////////////////////////////////////////
/// @brief Log lazy size is used in skeletons where the size is
/// defined as the logarithm of the inputs. A good example is an
/// n-ary @c tree
///
/// @tparam i the arity of the tree
///
/// @see tree
/// @see repeat
///
/// @ingroup skeletonsUtilities
//////////////////////////////////////////////////////////////////////
class input_lazysize
{
public:
  std::size_t operator()(std::size_t a)
  {
    return a == 1 ? 1 : a-1;
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Log lazy size is used in skeletons where the size is
/// defined as the logarithm of the inputs. A good example is an
/// n-ary @c tree
///
/// @tparam i the arity of the tree
///
/// @see tree
/// @see repeat
///
/// @ingroup skeletonsUtilities
//////////////////////////////////////////////////////////////////////
template <int i>
class log_lazysize
{
public:
  std::size_t operator()(std::size_t a)
  {
    return a == 1 ? 1 : static_cast<std::size_t>(log(a) / log(i));
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Log ceiling lazy size is used in skeletons where the size is
/// defined as the ceiling of the logarithm of the input size.
/// Hillis Steele or any other pointer jumping algorithm use this lazy
/// size evaluator.
///
/// @tparam i the arity of the tree
///
/// @see log_lazysize
///
/// @ingroup skeletonsUtilities
//////////////////////////////////////////////////////////////////////
template <int i>
class log_ceil_lazysize
{
public:
  std::size_t operator()(std::size_t a)
  {
    return a == 1 ? 1 : static_cast<std::size_t>(ceil(log(a) / log(i)));
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief The basic lazy size evaluator which returns a predefined
/// size upon evaluation.
///
/// @ingroup skeletonsUtilities
//////////////////////////////////////////////////////////////////////
class lazysize
{
  const std::size_t m_const_size;

public:
  explicit lazysize(std::size_t const_size)
    : m_const_size(const_size)
  { }

  template <typename T>
  std::size_t operator()(T&&)
  {
    return m_const_size;
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief The lazy size evaluator for matrix which returns a predefined
/// size upon evaluation.
///
/// @ingroup skeletonsUtilities
//////////////////////////////////////////////////////////////////////
struct matrix_lazysize
{
  template <typename SizeF>
  std::size_t operator()(SizeF&& mx_size)
  {
    return stapl::get<1>(mx_size);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Lazy size evaluation for divide-and-conquer algorithms
///
/// @ingroup skeletonsUtilities
//////////////////////////////////////////////////////////////////////
class lazy_divide_and_conquer_size
{
public:
  std::size_t operator()(std::size_t a)
  {

    if (a == 1) {
      return 1;
    }
    else {
      std::size_t reps = static_cast<std::size_t>(ceil(log(a) / log(2)));
      std::size_t result = (reps*(reps+1))/2;
      return result;
    }
  }
};


}
}
#endif // STAPL_SKELETONS_UTILITY_LAZY_SIZES_HPP
