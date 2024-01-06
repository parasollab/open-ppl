/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_FUNCTIONAL_ALLTOALL_HPP
#define STAPL_SKELETONS_FUNCTIONAL_ALLTOALL_HPP

#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/utility/skeleton.hpp>
#include "butterfly.hpp"
#include <stapl/skeletons/param_deps/alltoall_flat_pd.hpp>
#include <stapl/skeletons/param_deps/alltoall_hybrid_pd.hpp>
#include <stapl/skeletons/param_deps/alltoall_pairwise_exchange_pd.hpp>
#include <stapl/skeletons/flows/repeat_flows.hpp>

namespace stapl {
namespace skeletons {
namespace skeletons_impl {

template<typename T, typename Span, typename Tag>
struct alltoall;

//////////////////////////////////////////////////////////////////////
/// @brief This class abstracts the semantics of an alltoall skeleton
/// by exposing only the necessary information in its representation.
///
/// A butterfly-based (recursive-doubling) alltoall reduces the load
/// of communications by reducing the distance of communication at each
/// level of the butterfly. This implementation of alltoall can be used
/// for medium size messages. For larger messages it is advised to use
/// a pair-based implementation (in which each nodes exchanges data with
/// \f$index = i xor level\f$), or a ring-based implementation.
///
/// This abstraction not only makes the reconstruction of an
/// alltoall skeleton easier, but also provides access to the
/// information of how long the message can be on each location.
/// Furthermore, it reduces the symbol size for an alltoall skeleton,
/// hence, reducing the total compilation time.
///
/// @tparam T       the type of elements to be communicated
/// @tparam Span    the iteration space for elements on each level of
///                 the butterfly
///
/// @note This implementation can only be used for power-of-two sizes
/// for other cases, one should use a ring-based, pair-wise, or any
/// other alltoall implementations that can handle non-power-of-two inputs.
///
/// @ingroup skeletonsFunctionalInternal
//////////////////////////////////////////////////////////////////////
template<typename T, typename Span>
struct alltoall<T, Span, skeletons::tags::butterfly<false>>
  : public decltype(
             skeletons::butterfly<true, stapl::use_default, Span>(
               alltoall_merge<T, tags::butterfly<false>>(),
               alltoall_filter<T, tags::butterfly<false>>()
             )
           )
{
  using skeleton_tag_type = tags::alltoall<tags::butterfly<false>>;
  using base_type = decltype(
                      skeletons::butterfly<true, stapl::use_default, Span>(
                        alltoall_merge<T, tags::butterfly<false>>(),
                        alltoall_filter<T, tags::butterfly<false>>()));


  alltoall(void)
    : base_type(
        skeletons::butterfly<true, stapl::use_default, Span>(
          alltoall_merge<T, tags::butterfly<false>>(),
          alltoall_filter<T, tags::butterfly<false>>()
        )
      )
  { }

  void define_type(typer& t)
  {
    t.base<base_type>(*this);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief This class abstracts the semantics of a flat alltoall skeleton
/// by exposing only the necessary information in its representation.
///
/// In a flat alltoall the global exchanges happen in such a way that
/// each participant communicates with every other participant (O(1)).
/// This method is useful for lower processor count and when the messages
/// are large.
///
/// This abstraction not only makes the reconstruction of an
/// alltoall skeleton easier, but also reduces the symbol size for an alltoall
/// skeleton, hence, reducing the total compilation time.
///
/// @tparam T       the type of elements to be communicated
/// @tparam Span    the iteration space for elements
///
/// @note This version of alltoall can be used for non-power-of-two sizes
///
/// @ingroup skeletonsFunctionalInternal
//////////////////////////////////////////////////////////////////////
template<typename T, typename Span>
struct alltoall<T, Span, tags::flat>
  : public decltype(
             skeletons::elem<Span>(skeletons_impl::alltoall_pd<T, tags::flat>())
           )
{
  std::size_t m_num_elems_per_node;
public:
  using skeleton_tag_type = tags::alltoall<tags::flat>;
  using base_type = decltype(
                      skeletons::elem<Span>(
                        skeletons_impl::alltoall_pd<T, tags::flat>()));

  alltoall(void)
    : base_type(
        skeletons::elem<Span>(
          skeletons_impl::alltoall_pd<T, tags::flat>())
      )
  { }

  void define_type(typer& t)
  {
    t.base<base_type>(*this);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief This class abstracts the semantics of a hybrid alltoall skeleton
/// by exposing only the necessary information in its representation.
///
/// A hybrid alltoall combines ideas from butterfly-based alltoall and
/// flat alltoall. Therefore, in a hybrid alltoall the global exchanges
/// happen in log2(n) levels, and at each level every participant in
/// the upper half of a butterfly communicates with every participant
/// in the lower half of the same butterfly. This method is useful for
/// lower processor count and when the messages are large.
///
/// This abstraction not only makes the reconstruction of an
/// alltoall skeleton easier, but also reduces the symbol size for an alltoall
/// skeleton, hence, reducing the total compilation time.
///
/// @tparam T       the type of elements to be communicated
/// @tparam Span    the iteration space for elements (@todo ignored for now)
///
/// @note This version of alltoall can be used for only power-of-two sizes
///
/// @ingroup skeletonsFunctionalInternal
//////////////////////////////////////////////////////////////////////
template<typename T, typename Span>
struct alltoall<T, Span, tags::hybrid>
  : public decltype(
             skeletons::repeat<
               flows::repeat_flows::input_wrapper<flows::repeat_flows::piped>
             >(skeletons::elem(
                 skeletons_impl::alltoall_pd<T, tags::hybrid>()),
               log_lazysize<2>()))
{
  using skeleton_tag_type = tags::alltoall<tags::hybrid>;
  using base_type = decltype(
                      skeletons::repeat<
                        flows::repeat_flows::input_wrapper<
                          flows::repeat_flows::piped>
                      >(skeletons::elem(
                          skeletons_impl::alltoall_pd<T, tags::hybrid>()),
                        log_lazysize<2>()));

  alltoall(void)
    : base_type(
        skeletons::repeat<
          flows::repeat_flows::input_wrapper<flows::repeat_flows::piped>
        >(skeletons::elem(
            skeletons_impl::alltoall_pd<T, tags::hybrid>()), log_lazysize<2>())
      )
  { }

  void define_type(typer& t)
  {
    t.base<base_type>(*this);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief This class abstracts the semantics of a hybrid alltoall skeleton
/// by exposing only the necessary information in its representation.
///
/// A hybrid alltoall combines ideas from butterfly-based alltoall and
/// flat alltoall. Therefore, in a hybrid alltoall the global exchanges
/// happen in log2(n) levels, and at each level every participant in
/// the upper half of a butterfly communicates with every participant
/// in the lower half of the same butterfly. This method is useful for
/// lower processor count and when the messages are large.
///
/// This abstraction not only makes the reconstruction of an
/// alltoall skeleton easier, but also reduces the symbol size for an alltoall
/// skeleton, hence, reducing the total compilation time.
///
/// @tparam T       the type of elements to be communicated
/// @tparam Span    the iteration space for elements (@todo ignored for now)
///
/// @note This version of alltoall can be used for only power-of-two sizes
///
/// @ingroup skeletonsFunctionalInternal
//////////////////////////////////////////////////////////////////////
template<typename T, typename Span>
struct alltoall<T, Span, tags::pairwise_exchange>
  : public decltype(
             skeletons::repeat(
               skeletons::elem(
                 skeletons_impl::alltoall_pd<T, tags::pairwise_exchange>()),
               input_lazysize()))
{
  using skeleton_tag_type = tags::alltoall<tags::pairwise_exchange>;
  using base_type = decltype(
                      skeletons::repeat(
                        skeletons::elem(
                          skeletons_impl::alltoall_pd<
                            T, tags::pairwise_exchange>()),
                        input_lazysize()));

  alltoall(void)
    : base_type(
            skeletons::repeat(
              skeletons::elem(
                skeletons_impl::alltoall_pd<T, tags::pairwise_exchange>()),
              input_lazysize()))
  { }

  void define_type(typer& t)
  {
    t.base<base_type>(*this);
  }
};

} // namespace skeletons_impl

namespace result_of {

template <typename T, typename Tag, typename Span>
using alltoall = skeletons_impl::alltoall<
                   T, Span, stapl::default_type<Tag, tags::hybrid>>;

} // namespace result_of


//////////////////////////////////////////////////////////////////////
/// @brief This alltoall skeleton is a recursive-doubling alltoall and
/// works only for power-of-two sizes.
///
/// In a recursive doubling alltoall each node sends its lower and upper
/// half chunks to its butterflied pair in the parametric dependency.
/// Each chunk is of size @c num_elem_per_node/2. The communication is
/// therefore:
///
/// \f$\theta(log_{2}(n).n.num\_elem\_per\_node)\f$
///
/// @note The current default implementation can only be used for
/// power-of-two sizes, since it uses a butterfly skeleton.
///
/// @tparam T            the type of elements to be communicated
/// @tparam Span         the span to be used for @c butterfly inside
/// @tparam Tag          determines which type of alltoall to use
///
/// @ingroup skeletonsFunctionalExchange
//////////////////////////////////////////////////////////////////////
template <typename T,
          typename Tag  = stapl::use_default,
          typename Span = stapl::use_default>
skeletons::result_of::alltoall<T, Tag, Span>
alltoall(void)
{
  static_assert(std::is_same<Tag, stapl::use_default>::value      ||
                std::is_same<Tag, tags::flat>::value              ||
                std::is_same<Tag, tags::butterfly<false>>::value  ||
                std::is_same<Tag, tags::pairwise_exchange>::value ||
                std::is_same<Tag, tags::hybrid>::value,
                "The supported types of alltoall are flat, "
                "butterfly, and hybrid");

  return skeletons::result_of::alltoall<T, Tag, Span>();
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_FUNCTIONAL_ALLTOALL_HPP
