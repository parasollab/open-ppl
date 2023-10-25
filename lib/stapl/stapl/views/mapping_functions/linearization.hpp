/*
 // Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
 // component of the Texas A&M University System.

 // All rights reserved.

 // The information and source code contained herein is the exclusive
 // property of TEES and may not be disclosed, examined or reproduced
 // in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_MAPFUNCS_LINEARIZATION_HPP
#define STAPL_MAPFUNCS_LINEARIZATION_HPP

#include <stapl/algorithms/functional.hpp>
#include <stapl/views/mapping_functions/mapping_functions.hpp>
#include <stapl/utility/tuple.hpp>
#include <stapl/utility/tuple/to_index.hpp>
#include <stapl/utility/tuple/ensure_tuple.hpp>
#include <stapl/utility/tuple/rearrange.hpp>
#include <stapl/utility/integer_sequence.hpp>

#include <boost/mpl/find_if.hpp>
#include <boost/mpl/vector.hpp>
#include <boost/mpl/distance.hpp>

namespace stapl {

template<typename GID, typename Traversal>
struct nd_reverse_linearize;


template<typename GID, typename Traversal>
struct nd_linearize;


namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Rearrange the contents of a tuple to be in the correct
///        traversal order.
///
/// For example, given the tuple (x, y, z) and the traversal <1, 2, 0>,
/// output the tuple (z, x, y).
///
/// @tparam Tuple Type of the run-time tuple of std::size_ts
/// @tparam Traversal Traversal ordering.
//////////////////////////////////////////////////////////////////////
template<typename Tuple, typename Traversal,
         typename Indices = make_index_sequence<tuple_size<Traversal>::value>>
struct traversal_order;


template<typename Tuple, typename... IntTypes, std::size_t... Indices>
struct traversal_order<Tuple, tuple<IntTypes...>, index_sequence<Indices...>>
{
  typedef boost::mpl::vector<IntTypes...>                  vector_t;

private:
  typedef tuple<
    typename boost::mpl::distance<
      typename boost::mpl::begin<vector_t>::type,
      typename boost::mpl::find_if<
        vector_t, type_constant_equal<Indices>>::type
    >::type...>                                            indices_t;

public:
  static Tuple apply(Tuple const& src)
  {
    return Tuple(get<tuple_element<Indices, indices_t>::type::value>(src)...);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for size_t
//////////////////////////////////////////////////////////////////////
template<typename... IntTypes, std::size_t... Indices>
struct traversal_order<std::size_t, tuple<IntTypes...>,
                       index_sequence<Indices...>>
{
  static std::size_t apply(std::size_t src)
  {
    return src;
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Rearrange the contents of a tuple to be in the reverse
///        traversal order.
/// @tparam Tuple Type of tuple to rearrange.
/// @tparam Travesal Traversal ordering.
///
/// For example, given the tuple (x, y, z) and the traversal <1, 2, 0>,
/// output the tuple (y, z, x).
//////////////////////////////////////////////////////////////////////
template<typename Tuple, typename Traversal,
         typename Indices = make_index_sequence<tuple_size<Tuple>::value>>
struct reverse_traversal_order;


template<typename Tuple, typename Traversal, std::size_t... Indices>
struct reverse_traversal_order<Tuple, Traversal, index_sequence<Indices...>>
{
  static Tuple apply(Tuple const& src)
  {
    return Tuple(get<tuple_element<Indices, Traversal>::type::value>(src)...);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Computes the modulus of a 1-dimensional number with the
///        linear size of all of the dimensions before the current
///        dimension @p N.
//////////////////////////////////////////////////////////////////////
template<int N>
struct apply_modulo
{
  template<class Tuple>
  static size_t apply(Tuple const& sizes, size_t result)
  {
    return apply_modulo<N-1>::apply(
      tuple_ops::pop_back(sizes),
      result % tuple_ops::fold(sizes, 1, stapl::multiplies<size_t>()));
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Base case @ref apply_modulo.
//////////////////////////////////////////////////////////////////////
template<>
struct apply_modulo<0>
{
  template<class Tuple>
  static size_t apply(Tuple const&, size_t result)
  { return result; }
};


//////////////////////////////////////////////////////////////////////
/// @brief Divides a given number by all of the elements of a tuple.
///
/// Start with N = N-1 and sizes in traversal order
//////////////////////////////////////////////////////////////////////
template<int N>
struct apply_divide
{
  template<class Tuple>
  static size_t apply(Tuple const& sizes, size_t result)
  {
    return apply_divide<N-1>::apply(sizes, result / get<N>(sizes));
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Base case @ref apply_divide.
//////////////////////////////////////////////////////////////////////
template<>
struct apply_divide<-1>
{
  template<class Tuple>
  static size_t apply(Tuple const&, size_t result)
  { return result; }
};


//////////////////////////////////////////////////////////////////////
/// @brief Translates a 1-dimensional linearization back to its
///        n-dimensional form.
///
/// This functor assumes that the sizes are in the correct traversal
/// order.
//////////////////////////////////////////////////////////////////////
template<int I, typename GID>
struct reverse_linearize
{
  static GID apply(GID const& m_size, size_t linear, GID gid)
  {
    size_t result =
      apply_modulo<tuple_size<GID>::value-1-I>::apply(
        tuple_ops::pop_back(m_size), linear);

    result = apply_divide<I-1>::apply(m_size, result);

    get<I>(gid) = result;

    return reverse_linearize<I-1, GID>::apply(m_size, linear, gid);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Base case of recursion for @ref reverse_linearize.
/// @note Intel fails if function operator takes @p gid by reference.
//////////////////////////////////////////////////////////////////////
template<typename GID>
struct reverse_linearize<-1, GID>
{
  static GID apply(GID const&, size_t, GID gid)
  { return gid; }
};


//////////////////////////////////////////////////////////////////////
/// @brief Small function to optimize index linearization by not
/// performing multiplication on the first element.
//////////////////////////////////////////////////////////////////////
template<int Idx>
struct conditional_multiply
{
  template<typename Tuple>
  static size_t apply(Tuple const& plane_sizes, size_t lhs)
  {
    return lhs * get<Idx>(plane_sizes);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for first element of tuple.  Do not perform
/// multiplication and assert the first element in plane_sizes is 1.
//////////////////////////////////////////////////////////////////////
template<>
struct conditional_multiply<0>
{
  template<typename Tuple>
  static size_t apply(Tuple const& plane_sizes, size_t lhs)
  {
    stapl_assert(get<0>(plane_sizes) == 1, "found last idx != 1");
    return lhs;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Reorders input Indices according to provided traversal order,
/// decrements each by corresponding value in @p first and then multiplies
/// by corresponding value in @p plane_sizes, accumulating the results into
/// a linearized index.
//////////////////////////////////////////////////////////////////////
template <int Iterate, int LastIdx, typename Traversal>
struct reorder_localize_linearize
{
  template<typename Tuple, typename Index, typename... Indices>
  static size_t apply(Tuple const& first, Tuple const& plane_sizes,
                    Index&& i, Indices&&... is)

 {
   constexpr size_t idx = std::tuple_element<Iterate, Traversal>::type::value;

   return conditional_multiply<idx>::apply(
     plane_sizes, i - std::get<idx>(first))
     + reorder_localize_linearize<Iterate + 1, LastIdx, Traversal>::apply(
         first, plane_sizes, std::forward<Indices>(is)...);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Base case of the recursion.
/// @note Traversal of the tuple occurs from 0 to tuple_size-1 for performance
/// reasons.
//////////////////////////////////////////////////////////////////////
template<int LastIdx, typename Traversal>
struct reorder_localize_linearize<LastIdx, LastIdx, Traversal>
{
  template<typename Tuple, typename Index>
  static size_t apply(Tuple const& first, Tuple const& plane_sizes, Index&& i)
  {
    constexpr size_t idx = std::tuple_element<LastIdx, Traversal>::type::value;

    return conditional_multiply<idx>::apply(
      plane_sizes, i - std::get<idx>(first));
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Computes the factor by which a component of an n-dimensional GID
/// should be multiplied when linearizing the GID.
//////////////////////////////////////////////////////////////////////
template<int Iterate, int LastIdx>
struct compute_plane_sizes
{
  template <typename Tuple>
  static void apply(Tuple const& size, Tuple& plane_size)
  {
    // Other indices multiplied by size of all inner dimensions.
    std::get<Iterate>(plane_size) =
      std::get<Iterate-1>(size)*std::get<Iterate-1>(plane_size);
    compute_plane_sizes<Iterate+1, LastIdx>::apply(size, plane_size);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for first iteration of @ref compute_plane_sizes.
///
/// The first element represents the innermost dimension of the traversal,
/// and thus isn't multiplied by any of the other dimensions.
/// @note Traversal of the tuple occurs from 0 to tuple_size-1 for performance
/// reasons.
//////////////////////////////////////////////////////////////////////
template<int LastIdx>
struct compute_plane_sizes<0, LastIdx>
{
  template <typename Tuple>
  static void apply(Tuple const& size, Tuple& plane_size)
  {
    // Innermost dimension is multiplied by 1 when linearizing.
    std::get<0>(plane_size) = 1;
    compute_plane_sizes<1, LastIdx>::apply(size, plane_size);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for final iteration of @ref compute_plane_sizes.
///
/// The last element represents the outermost dimension of the traversal,
/// and thus is multiplied by the size of all inner dimensions when linearized.
/// @note Traversal of the tuple occurs from 0 to tuple_size-1 for performance
/// reasons.
//////////////////////////////////////////////////////////////////////
template<int LastIdx>
struct compute_plane_sizes<LastIdx, LastIdx>
{
  template <typename Tuple>
  static void apply(Tuple const& size, Tuple& plane_size)
  {
    // Plane size is size multiplied by size of all inner dimensions.
    std::get<LastIdx>(plane_size) =
      std::get<LastIdx-1>(size)*std::get<LastIdx-1>(plane_size);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for the 1-D case.
//////////////////////////////////////////////////////////////////////
template<>
struct compute_plane_sizes<0, 0>
{
  template <typename Tuple>
  static void apply(Tuple const& size, Tuple& plane_size)
  {
    // Innermost dimension is multiplied by 1 when linearizing.
    std::get<0>(plane_size) = 1;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Type metafunction reflecting the reduced rank linearizer
/// that a slice operation returns.
//////////////////////////////////////////////////////////////////////
template<std::size_t SliceIndex, typename Traversal, typename GID>
struct slice_result
{
  using slices_tuple_t =
    typename tuple_ops::from_index_sequence<index_sequence<SliceIndex>>::type;

  using new_gid_t =
    typename tuple_ops::result_of::heterogeneous_discard<
      slices_tuple_t, GID>::type;

  using new_traversal_t =
    typename tuple_ops::result_of::heterogeneous_discard<
      slices_tuple_t, Traversal>::type;

  using type = nd_linearize<new_gid_t, new_traversal_t>;
};

template<typename Slices, typename Traversal,
         typename = make_index_sequence<tuple_size<Slices>::value>>
struct indices_of_most_significant;


//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to compute the reordering of a Slices tuple (as a tuple
///        of indices) in descending order of Traversal's most significant
///        (slowest moving) dimensions.
///
///        When given multiple dimensions to slice at a time, we need to slice
///        them in the order of the largest to smallest (where largest and
///        smallest refers to their positions in the real traversal ordering).
///
///        For example, if Slices is <1,2,3> and Traversal is <0,2,4,3,1> then
///        the output would be <2,0,1>.
///
///        This is because we compute:
///          <4-Tr<Sl<0>>, 4-Tr<Sl<1>>, 4-Tr<Sl<2>>> =
///          <4-Tr<1>, 4-Tr<2>, 4-Tr<3>> =
///          <4-2, 4-4, 4-3> =
///          <2,0,1>
///
///        The meaning of <2,0,1> in this case means we need to slice position
///        1 first, then position 2, and then position 0. This boils down to
///        calling slice<2> and then slice<3> and then slice<1>. This is
///        necessary because in actuality, we are slicing the real dimensions of
///        4 then 3 then 2 of the traversal, which are the most significant
///        dimensions descending in order.
///
/// @note  The result of this metafunction only has meaning if the input slices
///        indeed refer to the last |Slices| dimensions of the linearizer and
///        it is only the order that is unknown.
//////////////////////////////////////////////////////////////////////
template<typename Slices, typename Traversal, std::size_t... Indices>
struct indices_of_most_significant<Slices, Traversal,
                                   index_sequence<Indices...>>
{
  static constexpr std::size_t traversal_size = tuple_size<Traversal>::value-1;

  using type = tuple<std::integral_constant<std::size_t,
    traversal_size - tuple_element<
      tuple_element<Indices, Slices>::type::value, Traversal>::type::value
  >...>;
};

//////////////////////////////////////////////////////////////////////
/// @brief Compute what the rest of the slices should be based on the current
///        slice. For example, if the incoming slices for slice() are
///        <0,1,2>, we would need to recursively call slice<0>.slice<0>.slice<0>
///        because we are now referring to the dimensions which are one less.
///        However, if the incoming slices are <2,1,0>, we would instead call
///        slice<2>.slice<1>.slice<0>.
//////////////////////////////////////////////////////////////////////
constexpr std::size_t adjusted_slice(std::size_t FirstSlice,
                                     std::size_t SecondSlice)
{
  return SecondSlice > FirstSlice ? SecondSlice-1 : SecondSlice;
}

} // namespace detail


//////////////////////////////////////////////////////////////////////
/// @brief Translates an n-dimensional @p gid to a 1-dimensional
///        linearization, based on the given @p Traversal order.
//////////////////////////////////////////////////////////////////////
template<typename GID, typename Traversal>
struct nd_linearize
{
public:
  using traversal_type = Traversal;
  using index_type     = GID;
  using gid_type       = size_t;
  using size_type      = index_type;
  using inverse        = nd_reverse_linearize<GID, Traversal>;
  using is_bijective   = std::true_type;

  static constexpr size_t last_idx = tuple_size<size_type>::value - 1;

  size_type         m_size;

  size_type         m_plane_sizes;
  size_type         m_first;

  /// @brief Original unreordered size kept to make slicing logic easier.
  size_type         m_original_size;

  /// @brief Original unreordered first index kept to make slicing logic easier.
  size_type         m_original_first;

public:
  nd_linearize(void) = default;

  explicit
  nd_linearize(size_type size)
    : m_size(detail::traversal_order<GID, Traversal>::apply(size)), m_first(),
      m_original_size(size), m_original_first()
  {
    detail::compute_plane_sizes<0, last_idx>::apply(m_size, m_plane_sizes);
  }

  explicit
  nd_linearize(size_type size, size_type first)
    : m_size(detail::traversal_order<GID, Traversal>::apply(size)),
      m_first(detail::traversal_order<GID, Traversal>::apply(first)),
      m_original_size(size), m_original_first(first)
  {
    detail::compute_plane_sizes<0, last_idx>::apply(m_size, m_plane_sizes);
  }

  explicit
  nd_linearize(nd_reverse_linearize<GID, Traversal> const& other)
    : m_size(other.m_size), m_first()
  {
    detail::compute_plane_sizes<0, last_idx>::apply(m_size, m_plane_sizes);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Slice a single index of the multidimensional space, fixing
  /// its value to that of the passed @p idx parameters. Return an offset
  /// representing the effect of the slicing operation and a reduced rank
  /// linearizer for the remaining indices.
  ///
  /// @todo Consider storing offset internally in @ref nd_linearize to
  /// better abstract behavior.
  //////////////////////////////////////////////////////////////////////
  template<std::size_t SliceIndex>
  std::pair<
    std::size_t, typename detail::slice_result<SliceIndex,Traversal,GID>::type
  >
  slice(size_t const& idx)
  {
    constexpr size_t mapped_index =
      tuple_element<SliceIndex, Traversal>::type::value;

    static_assert(mapped_index == last_idx,
                  "Slicing limited to last in traversal order");

    const size_t multiplier  = get<mapped_index>(m_plane_sizes);
    const std::size_t offset = (idx - get<mapped_index>(m_first)) * multiplier;

    using slice_result_t = detail::slice_result<SliceIndex,Traversal,GID>;
    using SlicesTuple    = typename slice_result_t::slices_tuple_t;
    using NewGID         = typename slice_result_t::new_gid_t;
    using NewTraversal   = typename slice_result_t::new_traversal_t;

    return std::make_pair(
      offset,
      nd_linearize<NewGID, NewTraversal>(
        tuple_ops::heterogeneous_discard<SlicesTuple>(m_original_size),
        tuple_ops::heterogeneous_discard<SlicesTuple>(m_original_first)));
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Slice multiple indices of the multidimensional space.
  /// Recursively called with single @p slice signature above as base case.
  //////////////////////////////////////////////////////////////////////
  template<std::size_t FirstSlice, std::size_t SecondSlice,
           std::size_t... Slices, typename Fixed>
    std::pair<std::size_t,
              decltype(
                std::declval<
                  typename detail::slice_result<
                    FirstSlice, Traversal, GID>::type
                >().template slice<
                    detail::adjusted_slice(FirstSlice, SecondSlice),
                    detail::adjusted_slice(FirstSlice, Slices)...
                  >(
                  tuple_ops::heterogeneous_discard<
                    tuple<std::integral_constant<size_t, 0>>
                  >(std::declval<Fixed>())).second)>
  slice(Fixed const& fixed)
  {
    using discard_tuple_t = tuple<std::integral_constant<size_t, 0>>;

    auto p = slice<FirstSlice>(get<0>(fixed));

    auto q = (p.second).template slice<
      detail::adjusted_slice(FirstSlice, SecondSlice),
      detail::adjusted_slice(FirstSlice, Slices)...>(
        tuple_ops::heterogeneous_discard<discard_tuple_t>(fixed)
      );

    return std::make_pair(p.first + q.first, q.second);
  }

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Slice multiple indices of the multidimensional space.
  ///    Specialization when Slices is an index_sequence
  //////////////////////////////////////////////////////////////////////
  template<std::size_t... Slices, typename Fixed>
  auto slice_impl(index_sequence<Slices...>, Fixed const& fixed)
    -> decltype(this->template slice<Slices...>(fixed))
  {
    return this->template slice<Slices...>(fixed);
  }

public:

  //////////////////////////////////////////////////////////////////////
  /// @brief Slice multiple indices of the multidimensional space.
  ///        Specialization when Slices is a tuple of integral constants
  ///
  ///        When slicing multiple indices at a time, we first need to rearrange
  ///        the slice dimensions to match the most significant dimensions of
  ///        the traversal.
  ///
  /// @tparam Slices Tuple of integral constants
  /// @param fixed The indices to fix in the slice
  //////////////////////////////////////////////////////////////////////
  template<typename Slices, typename Fixed>
  auto slice(Fixed const& fixed) ->
    decltype(this->slice_impl(typename tuple_ops::to_index_sequence<
      typename tuple_ops::result_of::rearrange<Slices,
        typename detail::indices_of_most_significant<Slices, Traversal>::type
      >::type
    >::type(), fixed))
  {
    // Rearrange the tuple of fixed values
    auto reordered_fixed = detail::traversal_order<Fixed,
      typename detail::indices_of_most_significant<Slices, Traversal>::type
    >::apply(fixed);

    // Rearrange the compile-time tuple of slices
    using reordered_slices = typename tuple_ops::to_index_sequence<
      typename tuple_ops::result_of::rearrange<
        Slices,
        typename detail::indices_of_most_significant<Slices, Traversal>::type
      >::type
    >::type;

    return this->slice_impl(reordered_slices(), reordered_fixed);
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Localize and then linearize the indices that are the components
  /// of an n-dimensional GID.
  //////////////////////////////////////////////////////////////////////
  template <typename... Indices>
  gid_type operator()(Indices const&... is) const
  {
    return detail::reorder_localize_linearize<0, last_idx, Traversal>::apply(
      m_first, m_plane_sizes, is...);
  }

private:
  template<std::size_t... I>
  gid_type
  linearize_impl(index_type const& gid, index_sequence<I...> const&) const
  {
    return detail::reorder_localize_linearize<0, last_idx, Traversal>::apply(
      m_first, m_plane_sizes, std::get<I>(gid)...);
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// Localize and then linearize the n-dimensional GID provided.
  //////////////////////////////////////////////////////////////////////
  template<typename Indices =
             make_index_sequence<tuple_size<index_type>::value>>
  gid_type operator()(index_type const& gid) const
  {
    return linearize_impl(gid, Indices());
  }

  void define_type(typer& t)
  {
    t.member(m_size);
    t.member(m_plane_sizes);
    t.member(m_first);
    t.member(m_original_size);
    t.member(m_original_first);
  }
}; // struct nd_linearize


//////////////////////////////////////////////////////////////////////
/// @brief Base case for @ref nd_linearize for 1-dimensional GID.
///
/// @todo Possibly use of the f_ident mapping function for this.
//////////////////////////////////////////////////////////////////////
template<typename Traversal>
struct nd_linearize<size_t, Traversal>
{
  using index_type = std::size_t;
  using traversal_type = Traversal;

  size_t m_size;
  size_t m_original_size;
  size_t m_original_first;
  size_t m_first;

  nd_linearize(void)
    : m_first(0)
  { }

  explicit
  nd_linearize(size_t)
    : m_first(0)
  { }

  explicit
  nd_linearize(size_t size, size_t first)
    : m_size(size), m_original_size(size), m_original_first(first),
      m_first(first)
  { }

  size_t operator()(size_t gid) const
  { return gid - m_first; }

  void define_type(typer& t)
  {
    t.member(m_size);
    t.member(m_original_size);
    t.member(m_original_first);
    t.member(m_first);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Translates a 1-dimensional linearized GID to its
///        n-dimensional GID, based on the given @p Traversal order.
//////////////////////////////////////////////////////////////////////
template<typename GID, typename Traversal>
struct nd_reverse_linearize
{
  typedef GID                               gid_type;
  typedef gid_type                          result_type;
  typedef std::size_t                       index_type;
  typedef gid_type                          size_type;
  typedef nd_linearize<GID, Traversal>      inverse;
  typedef std::true_type is_bijective;

  enum { last_idx = tuple_size<size_type>::value - 1 };

  size_type m_size;

  //////////////////////////////////////////////////////////////////////
  /// @todo Is the default constructor needed? if it is size should be
  ///       correctly initialized (default constructed in domain.hpp).
  //////////////////////////////////////////////////////////////////////
  nd_reverse_linearize(void) = default;

  explicit
  nd_reverse_linearize(size_type const& size)
    : m_size(detail::traversal_order<gid_type, Traversal>::apply(size))
  { }

  explicit
  nd_reverse_linearize(nd_linearize<GID, Traversal> const& other)
    : m_size(other.m_size)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @todo merge calls to inverse_linearizer_t::operator() and
  /// inverse_index_order_t::operator() into a single call
  /// to minimize the number of traversals of the tuple.
  //////////////////////////////////////////////////////////////////////
  gid_type operator()(index_type linear) const
  {
    return detail::reverse_traversal_order<gid_type, Traversal>::apply(
      detail::reverse_linearize<last_idx, gid_type>::apply(
        m_size, linear, gid_type())
    );
  }

  void define_type(typer& t)
  {
    t.member(m_size);
  }
}; // struct nd_reverse_linearize


//////////////////////////////////////////////////////////////////////
/// @brief Base case for @ref nd_reverse_linearize for 1-dimensional GID.
///
/// @todo Possible use of the f_ident mapping function for this.
//////////////////////////////////////////////////////////////////////
template<typename Traversal>
struct nd_reverse_linearize<size_t, Traversal>
{
  nd_reverse_linearize(void) = default;

  explicit
  nd_reverse_linearize(size_t)
  { }

  size_t operator()(size_t linear) const
  { return linear; }
};

} // namespace stapl

#endif // STAPL_MAPFUNCS_LINEARIZATION_HPP
