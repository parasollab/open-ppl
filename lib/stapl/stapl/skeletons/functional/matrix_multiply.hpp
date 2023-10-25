/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_FUNCTIONAL_MATRIX_MULTIPLY_HPP
#define STAPL_SKELETONS_FUNCTIONAL_MATRIX_MULTIPLY_HPP

#include <type_traits>
#include <utility>
#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/operators/elem.hpp>
#include <stapl/skeletons/param_deps/matrix_multiply_pd.hpp>
#include <stapl/skeletons/utility/tags.hpp>

namespace stapl {
namespace skeletons {

namespace flows {

//////////////////////////////////////////////////////////////////////
/// @brief A @c SUMMA flow for @c SUMMA skeleton. The input matrices
/// (A,B) are passed in each iteration, along with the intermediate
/// result matrix (C').
//////////////////////////////////////////////////////////////////////
struct summa_flow
{
  template <typename Repeat>
  struct port_types
  {
  private:
    using nested_p_type = typename Repeat::nested_p_type;
  protected:
    Repeat const& m_repeat;
  public:
    static constexpr std::size_t in_port_size = nested_p_type::in_port_size;
    using in_port_type = typename nested_p_type::in_port_type;

    template <typename In>
    struct out_port_type
    {
      typedef typename nested_p_type::template out_port_type<In>::type type;
    };


    port_types(Repeat const& repeat)
      : m_repeat(repeat)
    { }

    //////////////////////////////////////////////////////////////////////
    /// @brief S<b>[first-iter]</b><sub>in-flow</sub> =
    ///        repeat<sub>in-flow</sub>
    //////////////////////////////////////////////////////////////////////
    template <typename In>
    In
    in_flow(std::size_t, In const& in, std::size_t, tags::repeat_first_iter)
    {
      return in;
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief S<b>[iter]</b><sub>in-flow</sub> =
    ///          [pop-back(repeat<sub>in-flow</sub>),
    ///           S<b>[iter-1]</b><sub>out-flow</sub>]
    //////////////////////////////////////////////////////////////////////
    template <typename In, typename Tag>
    skeletons::flows::result_of::concat<
      typename tuple_ops::result_of::pop_back<In>::type,
      typename nested_p_type::template out_port_type<In>::type
    >
    in_flow(std::size_t iter_num, In const& in, std::size_t lid_offset, Tag)
    {
      return skeletons::flows::concat(
               tuple_ops::pop_back(in),
               m_repeat.nested_out_port(
                 in, lid_offset, iter_num-1));
    }


    //////////////////////////////////////////////////////////////////////
    /// @brief S<b>[iter]</b><sub>out-flow</sub> =
    ///        back(S<b>[iter+1]</b><sub>in-flow</sub>)
    //////////////////////////////////////////////////////////////////////
    template <typename Out, typename Tag>
    stapl::tuple<
      typename tuple_ops::result_of::back<
        typename nested_p_type::in_port_type>::type>
    out_flow(std::size_t iter_num, Out const& out, std::size_t lid_offset, Tag)
    {
      return stapl::make_tuple(
               tuple_ops::back(
                 m_repeat.nested_in_port(lid_offset, iter_num+1)));
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief S<b>[last-iter]</b><sub>out-flow</sub> =
    ///        repeat<sub>out-flow</sub>
    //////////////////////////////////////////////////////////////////////
    template <typename Out>
    Out
    out_flow(std::size_t, Out const& out, std::size_t, tags::repeat_last_iter)
    {
      return out;
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief repeat<sub>in-flow</sub> =
    ///        S<b>[first-iter]</b><sub>in-flow</sub>
    ///
    /// @todo the in port is connected to all iterations, this needs to be
    /// corrected.
    //////////////////////////////////////////////////////////////////////
    typename port_types<Repeat>::in_port_type
    in_port(std::size_t lid_offset)
    {
      return m_repeat.template nested_in_port(lid_offset, 0);
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief repeat<sub>out-flow</sub> =
    ///        S<b>[last-iter]</b><sub>out-flow</sub>
    //////////////////////////////////////////////////////////////////////
    template <typename In>
    typename out_port_type<In>::type
    out_port(In const& in, std::size_t lid_offset)
    {
      return m_repeat.nested_out_port(
               in, lid_offset, m_repeat.dimensions()-1);
    }
  };
};


} // namespace flows

namespace skeletons_impl {

template <typename T>
struct summa_op
{
  using result_type = T;

  template <typename V1, typename V2, typename V3>
  result_type operator()(V1&& v1, V2&& v2, V3&& v3)
  {
    return v1*v2 + v3;
  }
};

template <typename ValueType, typename Tag>
struct matrix_multiply;

//////////////////////////////////////////////////////////////////////
/// @brief This class abstracts the semantics of the matrix multiply skeleton
/// by exposing only the necessary information in its representation.
///
/// Provides a SUMMA skeleton for computing matrix multiplication.
///
/// @param ValueType  the type of input elements
///
/// @ingroup skeletonsFunctionalInternal
//////////////////////////////////////////////////////////////////////
template <typename ValueType>
struct matrix_multiply<ValueType, tags::summa>
  : public decltype(
             skeletons::repeat<skeletons::flows::summa_flow>(
               skeletons::elem<skeletons::spans::summa<2>>(
                 skeletons::matrix_multiply_pd<tags::summa>(
                   summa_op<ValueType>())),
               std::declval<skeletons::matrix_lazysize>()
             )
           )
{
private:
  using base_type = decltype(
            skeletons::repeat<skeletons::flows::summa_flow>(
              skeletons::elem<skeletons::spans::summa<2>>(
                skeletons::matrix_multiply_pd<tags::summa>(
                  summa_op<ValueType>())),
              std::declval<skeletons::matrix_lazysize>()
            )
          );
public:
  using skeleton_tag_type = tags::matrix_multiply<tags::summa>;

  matrix_multiply()
    : base_type(
        skeletons::repeat<skeletons::flows::summa_flow>(
          skeletons::elem<skeletons::spans::summa<2>>(
            skeletons::matrix_multiply_pd<tags::summa>(
              summa_op<ValueType>())),
          skeletons::matrix_lazysize()
        )
      )
  { }

  summa_op<ValueType> get_op(void)
  {
    return base_type::nested_skeleton().nested_skeleton().get_op();
  }

  void define_type(typer& t)
  {
    t.base<base_type>(*this);
  }
};

} // skeletons_impl

namespace result_of {

template <typename ValueType, typename Tag>
using matrix_multiply = skeletons_impl::matrix_multiply<
                          ValueType,
                          stapl::default_type<Tag, tags::summa>>;

} // namespace result_of


//////////////////////////////////////////////////////////////////////
/// @brief Creates and returns a matrix multiply skeleton based on tag.
///
/// @tparam ValueType matrix elements value type
//////////////////////////////////////////////////////////////////////
template <typename ValueType, typename Tag = stapl::use_default>
result_of::matrix_multiply<ValueType, Tag>
matrix_multiply(void)
{
  static_assert(std::is_same<Tag, stapl::use_default>::value      ||
                std::is_same<Tag, tags::summa>::value,
                "The supported type of matrix multiplication with "
                "skeletons is summa.");

  return result_of::matrix_multiply<ValueType, Tag>();
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_FUNCTIONAL_MATRIX_MULTIPLY_HPP
