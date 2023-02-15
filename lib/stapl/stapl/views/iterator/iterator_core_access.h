#ifndef STAPL_VIEWS_ITERATOR_CORE_ACCESS
#define STAPL_VIEWS_ITERATOR_CORE_ACCESS

#include <boost/mpl/bool.hpp>
#include <stapl/views/iterator/iterator_facade_fwd.h>

namespace stapl {



//////////////////////////////////////////////////////////////////////
/// @brief Helper class to access the core members functions that are
///        derived from iterator_facade.
//////////////////////////////////////////////////////////////////////
class iterator_core_access
{
  template <typename Derived, typename A,
    typename C, typename D>

  friend class iterator_facade;

private:
  template <typename Iterator>
  static typename Iterator::accessor
  access(Iterator const& i)
  {
    return i.access();
  }

  template <typename Iterator>
  static void
  increment(Iterator& i)
  {
    i.increment();
  }

  template <typename Iterator>
  static void
  decrement(Iterator& i)
  {
    i.decrement();
  }

public:
  template <typename Iterator>
  static void advance(Iterator& it, typename Iterator::difference_type n)
  {
    it.advance(n);
  }

  template <typename Iterator1, typename Iterator2>
  static typename Iterator1::difference_type
  distance_from(Iterator1 const& it1, Iterator2 const& it2, boost::mpl::true_)
  {
    return it1.distance_to(it2);
  }


  template <typename Iterator1, typename Iterator2>
  static typename Iterator2::difference_type
  distance_from(Iterator1 const& it1, Iterator2 const& it2, boost::mpl::false_)
  {
    //FIXME: the value here will need to be negated when the domain returns a
    //signed value
    return it2.distance_to(it1);
  }

  template <typename Iterator1, typename Iterator2>
  static bool less_than(Iterator1 const& lhs,
                        Iterator2 const& rhs, boost::mpl::true_)
  {
    return lhs.less_than(rhs);
  }

  template <typename Iterator1, typename Iterator2>
  static bool less_than(Iterator1 const& lhs,
                        Iterator2 const& rhs, boost::mpl::false_)
  {
    return !(rhs.less_than(lhs) || rhs.equal(lhs));
  }

  template <typename Iterator1, typename Iterator2>
  static bool equal(Iterator1 const& lhs,
                    Iterator2 const& rhs, boost::mpl::true_)
  {
    return lhs.equal(rhs);
  }

  template <typename Iterator1, typename Iterator2>
  static bool equal(Iterator1 const& lhs,
                    Iterator2 const& rhs, boost::mpl::false_)
  {
    return rhs.equal(lhs);
  }

  template <typename Iterator1, typename Iterator2>
  static bool less_than_equal(Iterator1 const& lhs,
                              Iterator2 const& rhs, boost::mpl::true_)
  {
    return lhs.equal(rhs) || lhs.less_than(rhs);
  }

  template <typename Iterator1, typename Iterator2>
  static bool less_than_equal(Iterator1 const& lhs,
                              Iterator2 const& rhs, boost::mpl::false_)
  {
    return rhs.equal(lhs) || !rhs.less(lhs);
  }

  template <typename Iterator>
  static auto base(Iterator iter) -> decltype(iter.base())
  {
    return iter.base();
  }

private:
  iterator_core_access();
}; //class iterator_core_access

} // namespace STAPL

#endif
