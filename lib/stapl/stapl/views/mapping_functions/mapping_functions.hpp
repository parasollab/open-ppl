/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_MAPPING_FUNCTIONS_HPP
#define STAPL_VIEWS_MAPPING_FUNCTIONS_HPP

#include <stapl/views/mapping_functions/identity.hpp>
#include <stapl/views/mapping_functions/traits.hpp>
#include <stapl/utility/utility.hpp>
#include <stapl/utility/tuple/tuple_size.hpp>
#include <stapl/utility/tuple/tuple_element.hpp>
#include <stapl/utility/tuple/ensure_tuple.hpp>
#include <stapl/utility/integer_sequence.hpp>

namespace stapl {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Generic composition of two mapping functions.
//////////////////////////////////////////////////////////////////////
template<typename F, typename G,
         typename = make_index_sequence<
                      tuple_size<
                        tuple_ops::ensure_tuple_t<typename F::gid_type>
                      >::value>>
class mapfun_comp;

template<typename F, typename G, std::size_t... GIDIndices>
class mapfun_comp<F, G, index_sequence<GIDIndices...>>
  : private tuple<F,G>
{
  //////////////////////////////////////////////////////////////////////
  /// @brief  Calls a functor that returns a reference to the element at the
  ///   final position (mapped by the outer mapping of the composition).
  ///
  /// @tparam RefGetter   Functor specified by an underlying container, that
  ///   returns a reference to the container's element at given position.
  /// @tparam OuterMapFun Functor that realizes the outer mapping of the
  ///   composition.
  //////////////////////////////////////////////////////////////////////
  template<typename RefGetter, typename OuterMapFun>
  struct outer_mapping
  {
  private:
    RefGetter const& m_ref_getter;
    OuterMapFun const& m_outer_mapfun;

  public:
    outer_mapping(RefGetter const& ref_getter, OuterMapFun const& outer)
      : m_ref_getter(ref_getter), m_outer_mapfun(outer)
    {}

    template<typename... Indices>
    auto operator() (Indices... indices) const
    STAPL_AUTO_RETURN (
      m_outer_mapfun.apply_get(m_ref_getter, indices...)
    )
  };

  using base_type = tuple<F,G>;

public:
  using index_type = typename G::index_type;
  using gid_type   = typename F::gid_type;

  mapfun_comp(void) = default;

  template<typename FParam, typename GParam>
  mapfun_comp(FParam&& f, GParam&& g)
    : base_type(std::forward<FParam>(f), std::forward<GParam>(g))
  { }

  gid_type operator()(index_type const& x) const
  {
    return get<0>(*this)(get<1>(*this)(x));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief  Applies the composed mapping on the input indices
  ///   (<tt>i1,i2,... -> F(G(i1,i2,...))</tt>) and calls the @c get_reference
  ///   functor with the mapped indices.
  ///
  ///   It calls @c G::apply_get with the outer mapping functor and indices to
  ///   be mapped. @c G::apply_get performs the inner mapping of the indices.
  ///   After the indices are mapped, @c G::apply_get applies the outer mapping
  ///   functor (@c F::apply_get) on them, which performs the outer mapping and
  ///   calls @c get_reference with the final (twice mapped) indices.
  ///
  /// @tparam RefGetter   Functor specified by an underlying container, that
  ///   returns a reference to the container's element at given position.
  /// @tparam Indices     Indices to be mapped.
  /// @return Reference to the element at the mapped position.
  //////////////////////////////////////////////////////////////////////
  template<typename RefGetter, typename... Indices>
  typename std::result_of<
    RefGetter(
      typename tuple_element<
        GIDIndices, tuple_ops::ensure_tuple_t<gid_type>
      >::type...
    )
  >::type
  apply_get(RefGetter const& get_reference, Indices... indices) const
  {
    return get<1>(*this).apply_get(
      outer_mapping<RefGetter, F>(get_reference, get<0>(*this)),
      indices...);
  }

  void define_type(typer& t)
  {
    t.base<base_type>(*this);
  }
}; // class mapfun_comp

} // namespace detail

//////////////////////////////////////////////////////////////////////
/// @brief Mapping functions composition. Compose two given mapping
///        functions f(g(x)).
///
/// This class has specializations based on template parameters.
/// @param F First mapping function type.
/// @param G Second mapping function type.
//////////////////////////////////////////////////////////////////////
template<typename F, typename G>
struct compose_func
{
  using type = detail::mapfun_comp<F, G>;

  static type apply(F const& f, G const& g)
  {
    return type(f, g);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization when the two mapping functions are identity.
//////////////////////////////////////////////////////////////////////
template<typename T>
struct compose_func<f_ident<T>, f_ident<T>>
{
  using type = f_ident<T>;

  static type apply(type const& f, type const&)
  {
    return f;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization when only the outer mapping function is
///        identity.
//////////////////////////////////////////////////////////////////////
template<typename T, typename G>
struct compose_func<f_ident<T>, G>
{
  using type = G;

  static type apply(f_ident<T> const&, G const& g)
  {
    return g;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization when the inner mapping function is identity.
//////////////////////////////////////////////////////////////////////
template<typename F, typename T>
struct compose_func<F, f_ident<T>>
{
  using type = F;

  static type apply(F const& f, f_ident<T> const&)
  {
    return f;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Mapping function generator without state.
///
/// This generator represents an infinite collection of the same
/// mapping function.
/// @tparam MF Type of the mapping function to return.
//////////////////////////////////////////////////////////////////////
template<typename MF>
struct map_fun_gen
{
  using mapfunc_type = MF;

  template<typename... Args>
  MF operator[](Args const&...) const
  {
    return MF();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Template alias for making a identity mapping function
/// generator given the specified GID type.
//////////////////////////////////////////////////////////////////////
template<typename GID>
using identity_map_func_gen = map_fun_gen<f_ident<GID>>;


//////////////////////////////////////////////////////////////////////
/// @brief Mapping function generator where every mapping function
///        generated is constructed passing the index as input
///        argument.
///
/// @tparam MF Type of the mapping function to return.
//////////////////////////////////////////////////////////////////////
template<typename MF>
struct map_fun_gen1
{
  using mapfunc_type = MF;

  template<typename... Args>
  MF operator[](Args&&... args) const
  {
    return mapfunc_type(std::forward<Args>(args)...);
  }
};

} // namespace stapl

#endif // STAPL_VIEWS_MAPPING_FUNCTIONS_HPP
