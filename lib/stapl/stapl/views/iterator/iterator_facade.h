/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_ITERATOR_ITERATOR_FACADE_HPP
#define STAPL_VIEWS_ITERATOR_ITERATOR_FACADE_HPP

#include <iterator>
#include <boost/mpl/int.hpp>
#include <boost/type_traits/is_convertible.hpp>

#include <stapl/runtime/stapl_assert.hpp>
#include <stapl/containers/type_traits/container_traits.hpp>
#include <stapl/views/proxy/stub.h>
#include <stapl/views/iterator/iterator_facade_fwd.h>
#include <stapl/views/iterator/iterator_core_access.h>
#include <type_traits>

namespace stapl {

template <typename T, typename A>
class proxy;


// Since STL doesn't define it, we do to be able to dispatch
struct trivial_iterator_tag
{ };




//////////////////////////////////////////////////////////////////////
/// @brief Functor creates a reference either as a proxy or a view, based
///   on @ref use_view_as_reference metafunction.
/// @tparam Accessor The accessor created by the derived iterator class.
///
/// Primary template constructs a proxy with the accessor and returns it
/// as the reference for the iterator dereference. The sole specialization
/// instead heap allocates the proxy and uses it as the container for a
/// view (the type of which is determined by @ref container_traits.
//////////////////////////////////////////////////////////////////////
template<typename Accessor,
         bool = view_impl::use_view_as_reference<
           typename std::remove_const<typename Accessor::value_type>::type
         >::value>
struct referencer
{
private:
  typedef typename std::remove_const<
    typename Accessor::value_type
  >::type                                                value_t;

public:
  typedef proxy<value_t, Accessor>                       result_type;

  result_type operator()(Accessor const& acc) const
  {
    return result_type(acc);
  }
};


template<typename Accessor>
struct referencer<Accessor, true>
{
private:
  typedef typename std::remove_const<
    typename Accessor::value_type
  >::type                                                value_t;

  typedef proxy<value_t, Accessor>                       proxy_t;

public:
  typedef typename container_traits<value_t>::
    template construct_view<proxy_t>::type               result_type;

  result_type operator()(Accessor const& acc) const
  {
    return result_type(new proxy_t(acc));
  }
};


template <typename Derived,
          typename Accessor,
          typename Category,
          typename Difference>
class iterator_facade
{
  friend class iterator_core_access;

private:
  typedef Accessor                            accessor;

protected:
  ~iterator_facade() = default;

public:
  typedef Category                            iterator_category;

  typedef typename std::remove_const<
    typename Accessor::value_type
  >::type                                     value_type;

  typedef Difference                          difference_type;

  typedef stub<value_type, Accessor>          pointer;

private:
  typedef referencer<Accessor>                referencer_t;

public:
  typedef typename referencer_t::result_type  reference;

  Derived& derived()
  {
    return *static_cast<Derived*>(this);
  }

  Derived const& derived() const
  {
    return *static_cast<Derived const*>(this);
  }

  reference operator*() const
  {
    return referencer_t()(iterator_core_access::access(this->derived()));
  }

  pointer operator->()
  {
    stapl_assert(0, "-> not currently supported in stapl iterators");
    return pointer();
  }

  reference operator[](int n) const
  {
    return referencer_t()(iterator_core_access::access((*this + n).derived()));
  }

  Derived& operator++()
  {
    iterator_core_access::increment(this->derived());
    return this->derived();
  }

  Derived operator++(int)
  {
    Derived tmp(this->derived());
    operator++();
    return tmp;
  }

  Derived& operator--()
  {
    iterator_core_access::decrement(this->derived());
    return this->derived();
  }

  Derived operator--(int)
  {
    Derived tmp(this->derived());
    operator--();
    return tmp;
  }

  Derived& operator+=(difference_type n)
  {
    iterator_core_access::advance(this->derived(), n);
    return this->derived();
  }

  Derived& operator-=(difference_type n)
  {
    iterator_core_access::advance(this->derived(), -n);
    return this->derived();
  }

  bool less_than(iterator_facade const& rhs) const
  {
    return iterator_core_access::distance_from(
        static_cast<Derived const&>(*this),
        static_cast<Derived const&>(rhs),boost::mpl::true_()) < 0;
  }
  friend Derived
  operator+(iterator_facade const& src, difference_type n)
  {
    Derived tmp(static_cast<Derived const&>(src));
    return tmp += n;
  }

  friend Derived
  operator+(difference_type n, iterator_facade const& src)
  {
    return src+n;
  }

  friend Derived
  operator-(iterator_facade const& src, difference_type n)
  {
    Derived tmp(static_cast<Derived const&>(src));
    return tmp -= n;
  }

  friend Derived
  operator-(difference_type n, iterator_facade const& src)
  {
    return src-n;
  }

private:
  void define_type(typer&);

}; // class iterator_facade


// NOTE - Below we define binary operators for iterators using iterator_facade.
// There are three versions that functionally do the same but that catch various
// types of lhs / rhs pairs.
// (1) Both lhs and rhs are exactly the same type and use default Category
//     and Difference template parameter values of iterator_facade.
//
// (2) Both lhs and rhs are exactly the same type and use non-default values
//     of Category and Difference.
//
// (3) The General Case where lhs and rhs are different types and possibly
//     use non-default values of Category and Difference.
//
// FIXME - return could be Derived2::difference_type, boost does mpl::if switch
//

//
// operator-
//
template<typename Derived, typename Accessor>
typename Derived::difference_type
operator-(iterator_facade<Derived, Accessor> const& lhs,
          iterator_facade<Derived, Accessor> const& rhs)
{
  return iterator_core_access::distance_from(
    static_cast<Derived const&>(rhs),
    static_cast<Derived const&>(lhs),
    boost::mpl::true_()
  );
}


template<typename Derived, typename Accessor,
         typename Category, typename Difference>
typename Derived::difference_type
operator-(iterator_facade<Derived, Accessor, Category, Difference> const& lhs,
          iterator_facade<Derived, Accessor, Category, Difference> const& rhs)
{
  return iterator_core_access::distance_from(
    static_cast<Derived const&>(rhs),
    static_cast<Derived const&>(lhs),
    boost::mpl::true_()
  );
}


template<typename Derived1, typename Accessor1,
         typename Category1, typename Difference1,
         typename Derived2, typename Accessor2,
         typename Category2, typename Difference2>
typename Derived2::difference_type
operator-(iterator_facade<Derived1, Accessor1,
                          Category1, Difference1> const& lhs,
          iterator_facade<Derived2, Accessor2,
                          Category2, Difference2> const& rhs)
{
  return iterator_core_access::distance_from(
    static_cast<Derived2 const&>(rhs),
    static_cast<Derived1 const&>(lhs),
    boost::is_convertible<Derived1, Derived2>()
  );
}


//
// operator==
//
template<typename Derived, typename Accessor>
typename Derived::difference_type
operator==(iterator_facade<Derived, Accessor> const& lhs,
           iterator_facade<Derived, Accessor> const& rhs)
{
  return iterator_core_access::equal(
    static_cast<Derived const&>(lhs),
    static_cast<Derived const&>(rhs),
    boost::mpl::true_()
  );
}


template<typename Derived, typename Accessor,
         typename Category, typename Difference>
typename Derived::difference_type
operator==(iterator_facade<Derived, Accessor, Category, Difference> const& lhs,
           iterator_facade<Derived, Accessor, Category, Difference> const& rhs)
{
  return iterator_core_access::equal(
    static_cast<Derived const&>(lhs),
    static_cast<Derived const&>(rhs),
    boost::mpl::true_()
  );
}


template<typename Derived1, typename Accessor1,
         typename Category1, typename Difference1,
         typename Derived2, typename Accessor2,
         typename Category2, typename Difference2>
typename Derived2::difference_type
operator==(iterator_facade<Derived1, Accessor1,
                           Category1, Difference1> const& lhs,
           iterator_facade<Derived2, Accessor2,
                           Category2, Difference2> const& rhs)
{
  return iterator_core_access::equal(
    static_cast<Derived1 const&>(lhs),
    static_cast<Derived2 const&>(rhs),
    boost::is_convertible<Derived2, Derived1>()
  );
}


//
// operator!=
//
template<typename Derived, typename Accessor>
bool operator!=(iterator_facade<Derived, Accessor> const& lhs,
                iterator_facade<Derived, Accessor> const& rhs)
{
  return !(lhs == rhs);
}


template<typename Derived, typename Accessor,
         typename Category, typename Difference>
bool operator!=(iterator_facade<Derived, Accessor,
                                Category, Difference> const& lhs,
                iterator_facade<Derived, Accessor,
                                Category, Difference> const& rhs)
{
  return !(lhs == rhs);
}


template <typename Derived1, typename Accessor1,
          typename Category1, typename Difference1,
          typename Derived2, typename Accessor2,
          typename Category2, typename Difference2>
bool operator!=(iterator_facade<Derived1, Accessor1,
                                Category1, Difference1> const& lhs,
                iterator_facade<Derived2, Accessor2,
                                Category2, Difference2> const& rhs)
{
  return !(lhs == rhs);
}


//
// operator<
//
template<typename Derived, typename Accessor>
bool operator<(iterator_facade<Derived, Accessor> const& lhs,
               iterator_facade<Derived, Accessor> const& rhs)
{
  return iterator_core_access::less_than(
    static_cast<Derived const&>(lhs),
    static_cast<Derived const&>(rhs),
    boost::mpl::true_()
  );
}


template<typename Derived, typename Accessor,
         typename Category, typename Difference>
bool operator<(iterator_facade<Derived, Accessor,
                               Category, Difference> const& lhs,
               iterator_facade<Derived, Accessor,
                               Category, Difference> const& rhs)
{
  return iterator_core_access::less_than(
    static_cast<Derived const&>(lhs),
    static_cast<Derived const&>(rhs),
    boost::mpl::true_()
  );
}


template<typename Derived1, typename Accessor1,
         typename Category1, typename Difference1,
         typename Derived2, typename Accessor2,
         typename Category2, typename Difference2>
bool operator<(iterator_facade<Derived1, Accessor1,
                               Category1, Difference1> const& lhs,
               iterator_facade<Derived2, Accessor2,
                               Category2, Difference2> const& rhs)
{
  return iterator_core_access::less_than(
    static_cast<Derived1 const&>(lhs),
    static_cast<Derived2 const&>(rhs),
    boost::is_convertible<Derived2, Derived1>()
  );
}


//
// operator>
//
template<typename Derived, typename Accessor>
bool operator>(iterator_facade<Derived, Accessor> const& lhs,
               iterator_facade<Derived, Accessor> const& rhs)
{
  return iterator_core_access::less_than(
    static_cast<Derived const&>(rhs),
    static_cast<Derived const&>(lhs),
    boost::mpl::true_()
  );
}


template<typename Derived, typename Accessor,
         typename Category, typename Difference>
bool operator>(iterator_facade<Derived, Accessor,
                               Category, Difference> const& lhs,
               iterator_facade<Derived, Accessor,
                               Category, Difference> const& rhs)
{
  return iterator_core_access::less_than(
    static_cast<Derived const&>(rhs),
    static_cast<Derived const&>(lhs),
    boost::mpl::true_()
  );
}


template<typename Derived1, typename Accessor1,
         typename Category1, typename Difference1,
         typename Derived2, typename Accessor2,
         typename Category2, typename Difference2>
bool operator>(iterator_facade<Derived1, Accessor1,
                               Category1, Difference1> const& lhs,
               iterator_facade<Derived2, Accessor2,
                               Category2, Difference2> const& rhs)
{
  return iterator_core_access::less_than(
    static_cast<Derived2 const&>(rhs),
    static_cast<Derived1 const&>(lhs),
    boost::is_convertible<Derived1, Derived2>()
  );
}


//
// operator<=
//
template<typename Derived, typename Accessor>
bool operator<=(iterator_facade<Derived, Accessor> const& lhs,
                iterator_facade<Derived, Accessor> const& rhs)
{
  return iterator_core_access::less_than_equal(
    static_cast<Derived const&>(lhs),
    static_cast<Derived const&>(rhs),
    boost::mpl::true_()
  );
}


template<typename Derived, typename Accessor,
         typename Category, typename Difference>
bool operator<=(iterator_facade<Derived, Accessor,
                                Category, Difference> const& lhs,
                iterator_facade<Derived, Accessor,
                                Category, Difference> const& rhs)
{
  return iterator_core_access::less_than_equal(
    static_cast<Derived const&>(lhs),
    static_cast<Derived const&>(rhs),
    boost::mpl::true_()
  );
}


template<typename Derived1, typename Accessor1,
         typename Category1, typename Difference1,
         typename Derived2, typename Accessor2,
         typename Category2, typename Difference2>
bool operator<=(iterator_facade<Derived1, Accessor1,
                                Category1, Difference1> const& lhs,
                iterator_facade<Derived2, Accessor2,
                                Category2, Difference2> const& rhs)
{
  return iterator_core_access::less_than_equal(
    static_cast<Derived1 const&>(lhs),
    static_cast<Derived2 const&>(rhs),
    boost::is_convertible<Derived2, Derived1>()
  );
}


//
// operator>=
//
template<typename Derived, typename Accessor>
bool operator>=(iterator_facade<Derived, Accessor> const& lhs,
                iterator_facade<Derived, Accessor> const& rhs)
{
  return iterator_core_access::less_than_equal(
    static_cast<Derived const&>(rhs),
    static_cast<Derived const&>(lhs),
    boost::mpl::true_()
  );
}


template<typename Derived, typename Accessor,
         typename Category, typename Difference>
bool operator>=(iterator_facade<Derived, Accessor,
                                Category, Difference> const& lhs,
                iterator_facade<Derived, Accessor,
                                Category, Difference> const& rhs)
{
  return iterator_core_access::less_than_equal(
    static_cast<Derived const&>(rhs),
    static_cast<Derived const&>(lhs),
    boost::mpl::true_()
  );
}


template<typename Derived1, typename Accessor1,
         typename Category1, typename Difference1,
         typename Derived2, typename Accessor2,
         typename Category2, typename Difference2>
bool operator>=(iterator_facade<Derived1, Accessor1,
                                Category1, Difference1> const& lhs,
                iterator_facade<Derived2, Accessor2,
                                Category2, Difference2> const& rhs)
{
  return iterator_core_access::less_than_equal(
    static_cast<Derived2 const&>(rhs),
    static_cast<Derived1 const&>(lhs),
    boost::is_convertible<Derived1, Derived2>()
  );
}

} // namespace stapl

#endif // STAPL_VIEWS_ITERATOR_ITERATOR_FACADE_HPP
