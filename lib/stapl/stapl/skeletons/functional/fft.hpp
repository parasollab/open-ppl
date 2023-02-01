/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_FUNCTIONAL_FFT_HPP
#define STAPL_SKELETONS_FUNCTIONAL_FFT_HPP

#include <utility>
#include <type_traits>
#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/utility/skeleton.hpp>
#include <stapl/skeletons/operators/compose.hpp>
#include <stapl/skeletons/functional/butterfly.hpp>
#include <stapl/skeletons/functional/bitreversal.hpp>

namespace stapl {
namespace skeletons {
namespace skeletons_impl {

//////////////////////////////////////////////////////////////////////
/// @brief This workfunction is used for Decimation In Frequency FFT
///          (DIF) computation.
///
/// @tparam T the type of components of the complex number
///             used in FFT
/// @tparam is_forward if it is a forward FFT or not
//////////////////////////////////////////////////////////////////////
template <typename T, bool is_forward = true>
struct fft_dif_wf
{
  using value_type  = std::complex<T>;
  using result_type = std::complex<T>;
private:
  bool       m_is_downedge;
  value_type m_twiddle_factor;

public:
  fft_dif_wf(void)
    : m_is_downedge(false),
      m_twiddle_factor(0., 0.)
  { }

  void set_position(std::size_t butterfly_size,
                    std::size_t index1,
                    std::size_t index2, std::size_t /* ignored */)
  {
    m_is_downedge = index1 >= index2;
    // 2 * pi * i / 2 * butterfly_size = pi * i / butterfly_size
    T const e = (is_forward ? -1 : 1) * M_PI * (index1 % butterfly_size)
                / double(butterfly_size);

    m_twiddle_factor = result_type(cos(e), sin(e));
  }

  template <typename E>
  result_type operator()(E el1, E el2) const
  {
    if (m_is_downedge) {
      return (el2 - el1) * m_twiddle_factor;
    }
    else{
      return el1 + el2;
    }
  }

  void define_type(typer &t)
  {
    t.member(m_is_downedge);
    t.member(m_twiddle_factor);
  }

};

//////////////////////////////////////////////////////////////////////
/// @brief This workfunction is used for Decimation In Time
///   FFT (DIT) computation.
///
/// @tparam T the type of components of the complex number
///             used in FFT
/// @tparam is_forward if it is a forward FFT or not
//////////////////////////////////////////////////////////////////////
template <typename T, bool is_forward = true>
struct fft_dit_wf
{
  using value_type  = std::complex<T>;
  using result_type = std::complex<T>;
private:
  bool       m_is_downedge;
  value_type m_twiddle_factor;

public:
  fft_dit_wf(void)
    : m_is_downedge(false),
      m_twiddle_factor(0., 0.)
  { }

  void set_position(std::size_t butterfly_size,
                    std::size_t index1,
                    std::size_t index2, std::size_t /* ignored */)
  {
    m_is_downedge = index1 >= index2;
    // 2 * pi * i / 2 * butterfly_size = pi * i / butterfly_size
    T const e = (is_forward ? -1 : 1) * M_PI * (index1 % butterfly_size)
                / double(butterfly_size);

    m_twiddle_factor = result_type(cos(e), sin(e));
  }

  template <typename E>
  result_type operator()(E el1, E el2) const
  {
    if (m_is_downedge) {
      return el2 - el1 * m_twiddle_factor;
    }
    else {
      return el1 + el2 * m_twiddle_factor;
    }
  }

  void define_type(typer &t)
  {
    t.member(m_is_downedge);
    t.member(m_twiddle_factor);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief This class abstracts the semantics of an FFT skeleton
/// by exposing only the necessary information in its representation.
///
/// This abstraction not only makes the reconstruction of an
/// FFT skeleton easier, but also provides access to the
/// underlying operations of the enclosed reduction operation.
/// Furthermore, it reduces the symbol size for an FFT skeleton,
/// hence, reducing the total compilation time.
///
/// @tparam T       determines the type of input and output elements
/// @tparam Span    the iteration space for elements.
/// @tparam Tag     determines the type of the FFT skeleton
///
/// @ingroup skeletonsFunctionalInternal
//////////////////////////////////////////////////////////////////////
template <typename T, typename Span, typename Tag>
struct fft;

//////////////////////////////////////////////////////////////////////
/// @brief This class abstracts the semantics of a Decimation In
/// Frequency (DIF) FFT skeleton by exposing only the necessary
/// information in its representation.
///
/// @tparam Span the iteration space for elements on each level
///
/// @ingroup skeletonsFunctionalInternal
//////////////////////////////////////////////////////////////////////
template <typename T, typename Span>
struct fft<T, Span, tags::dif>
  : public decltype(
             skeletons::compose(
               skeletons::butterfly<true>(std::declval<fft_dif_wf<T>>()),
               skeletons::bitreversal()
             )
           )
{
  using skeleton_tag_type = tags::fft<tags::dif>;
  using base_type = decltype(
                      skeletons::compose(
                        skeletons::butterfly<true>(
                          std::declval<fft_dif_wf<T>>()),
                        skeletons::bitreversal()));

  fft(void)
    : base_type(
        skeletons::compose(
          skeletons::butterfly<true>(fft_dif_wf<T>()),
          skeletons::bitreversal()))
  { }

  fft_dif_wf<T> get_op(void) const
  {
    return base_type::template get_skeleton<0>().get_op();
  }

  void define_type(typer& t)
  {
    t.base<base_type>(*this);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief This class abstracts the semantics of a Decimation In
/// Time (DIT) FFT skeleton by exposing only the necessary
/// information in its representation.
///
/// @tparam Span the iteration space for elements on each level
///
/// @ingroup skeletonsFunctionalInternal
//////////////////////////////////////////////////////////////////////
template <typename T, typename Span>
struct fft<T, Span, tags::dit>
  : public decltype(
             skeletons::compose(
               skeletons::bitreversal(),
               skeletons::reverse_butterfly<true>(std::declval<fft_dit_wf<T>>())
             )
           )
{
  using skeleton_tag_type = tags::fft<tags::dit>;
  using base_type = decltype(
                      skeletons::compose(
                        skeletons::bitreversal(),
                        skeletons::reverse_butterfly<true>(
                          std::declval<fft_dit_wf<T>>())));

  fft()
    : base_type(
        skeletons::compose(
          skeletons::bitreversal(),
          skeletons::reverse_butterfly<true>(fft_dit_wf<T>())))
  { }

  fft_dif_wf<T> get_op(void) const
  {
    return base_type::template get_skeleton<0>().get_op();
  }

  void define_type(typer& t)
  {
    t.base<base_type>(*this);
  }
};

}

namespace result_of {

template <typename T,
          typename Span,
          typename Tag>
using fft = skeletons_impl::fft<
              T,
              stapl::default_type<Span, spans::balanced<>>,
              stapl::default_type<Tag, tags::dit>>;

} // namespace result_of

//////////////////////////////////////////////////////////////////////
/// @brief An FFT skeleton which its type is determined by the tag
/// passed to it. The possible choices for the tags are:
/// @li tags::dit
/// @li tags::dif
/// @li stapl::use_default - which will use the Decimation In Time
///                            (DIT) FFT
///
/// @tparam T    the type of input elements
/// @tparam tag  determines which type of FFT should be used
/// @tparam Span the span to be used for @c FFT skeleton
///
/// @ingroup skeletonsFunctionalFFT
//////////////////////////////////////////////////////////////////////
template <typename T,
          typename Tag = stapl::use_default,
          typename Span = stapl::use_default>
result_of::fft<T, Span, Tag>
fft(void)
{
  return result_of::fft<T, Span, Tag>();
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_FUNCTIONAL_FFT_HPP
