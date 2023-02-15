/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_FUNCTIONAL_SERIAL_HPP
#define STAPL_SKELETONS_FUNCTIONAL_SERIAL_HPP

#include <type_traits>
#include <utility>
#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/utility/skeleton.hpp>
#include <stapl/skeletons/operators/elem.hpp>
#include <stapl/skeletons/flows/elem_flow.hpp>
#include <stapl/skeletons/param_deps/serial_pd.hpp>

namespace stapl {
namespace skeletons {
namespace skeletons_impl {

////////////////////////////////////////////////////////////////////////
/// @brief This class abstracts the semantics of an serial skeleton
/// by exposing only the necessary information in its representation.
///
/// This abstraction not only makes the reconstruction of a
/// serial skeleton easier, but also provides access to the
/// underlying operation.
/// Furthermore, it reduces the symbol size for a serial skeleton,
/// hence, reducing the total compilation time.
///
/// @tparam Op the operation to be applied on each element of views
/// @tparam i  number of inputs to each parametric dependency element
///
/// @ingroup skeletonsFunctionalInternal
//////////////////////////////////////////////////////////////////////
template <typename Op, int i>
struct serial
  : public decltype(
             skeletons::elem<
               stapl::use_default,
               flows::elem_f::doacross>
             (skeletons::serial_pd<i, Op>(std::declval<Op>(),
                                          std::size_t()))
           )
{
  using skeleton_tag_type = tags::serial<i>;
  using base_type = decltype(
                      skeletons::elem<
                        stapl::use_default, flows::elem_f::doacross
                      >(skeletons::serial_pd<i, Op>(std::declval<Op>(),
                                                    std::size_t())));

  serial(Op const& op, std::size_t number_of_sets)
    : base_type(
        skeletons::elem<
          stapl::use_default,
          flows::elem_f::doacross
        >(skeletons::serial_pd<i, Op>(op, number_of_sets))
      )
  { }

  auto get_op(void) const ->
    decltype(
      std::declval<base_type>().nested_skeleton().get_op()
    )
  {
    return base_type::nested_skeleton().get_op();
  }

  std::size_t get_number_of_sets(void) const
  {
    return base_type::nested_skeleton().get_number_of_sets();
  }

  void define_type(typer& t)
  {
    t.base<base_type>(*this);
  }
};

} // namespace skeletons_impl


namespace result_of {

template <int i, typename Op>
using serial = skeletons_impl::serial<typename std::decay<Op>::type, i>;

} // namespace result_of


//////////////////////////////////////////////////////////////////////
/// @brief  Construct and execute a skeleton that performs a set of serial
/// computations
///
/// @param op the operator the should be applied on elements of sets.
/// @param number_of_sets   The number of sets to form. View elements
/// are balance distributed across the sets.
///
/// @ingroup skeletonsFunctional
//////////////////////////////////////////////////////////////////////
template <int i, typename Op>
result_of::serial<i, Op>
serial(Op&& op, std::size_t number_of_sets)
{
  return result_of::serial<i, Op>(std::forward<Op>(op), number_of_sets);
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_FUNCTIONAL_SERIAL_HPP
