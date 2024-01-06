#ifndef ND_TRAITS_HPP
#define ND_TRAITS_HPP

#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/utility/tuple/homogeneous_tuple.hpp>
#include <stapl/containers/type_traits/dimension_traits.hpp>

namespace stapl {

template<size_t N>
struct nd_traits {

  using indices = make_index_sequence<N>;

  template<class C>
  using index_type = typename C::index_type;

  template<class C>
  using dimensions_type = typename C::dimensions_type;

  template<class T>
  using make_nd = homogeneous_tuple_type_t<N, T>;

  template<class T>
  static auto dimensions(T&& t) STAPL_AUTO_RETURN((
    t.dimensions()
  ))
  template<size_t I, class T>
  static auto nd_get(T&& t) STAPL_AUTO_RETURN((
    get<I>(std::forward<T>(t))
  ))
};

template<>
struct nd_traits<size_t{1}> {

  using indices = make_index_sequence<1>;

  template<class C>
  using index_type = typename C::size_type;

  template<class C>
  using dimensions_type = typename C::size_type;

  template<class T>
  using make_nd = T;

  template<class T>
  static auto dimensions(T&& t) STAPL_AUTO_RETURN((
    t.size()
  ))
  template<class T>
  static auto dimensions(T const& t) STAPL_AUTO_RETURN((
    t.size()
  ))

  template<size_t I, class T>
  static T nd_get(T&& t) {
    static_assert(I == 0, "invalid nd_get on scalar");
    return t;
  }
};

template<size_t I, class T>
auto nd_get(T&& t) STAPL_AUTO_RETURN((
  nd_traits<dimension_traits<typename std::decay<T>::type>::type::value>
    ::template nd_get<I>(std::forward<T>(t))
))

} /* stapl */

#endif /* end of include guard: ND_TRAITS_HPP */
