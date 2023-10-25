#ifndef STAPL_VIEWS_ITERATOR_FACADE_FWD
#define STAPL_VIEWS_ITERATOR_FACADE_FWD

namespace stapl {

template <typename Derived,
          typename Accessor,
          typename Category   = std::random_access_iterator_tag,
          typename Difference = ptrdiff_t>
class iterator_facade;

} // namespace STAPL

#endif
