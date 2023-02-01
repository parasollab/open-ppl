/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_ALGORITHMS_ALGO_DETAIL_HPP
#define STAPL_ALGORITHMS_ALGO_DETAIL_HPP

namespace stapl {

namespace algo_details {


//////////////////////////////////////////////////////////////////////
/// @brief Work function invokes sequential (i.e., STL) @p adjacent_find.
///
/// The function operator will be called for each local set of overlapping
/// intervals (for range_adjacent_find, the overlap is just one element and the
/// intervals are simply pairs of adjacent elements) and for each set of
/// overlapping intervals that span across locations (and need to fetch remote
/// elements):
///
///      LOC 1      ||   LOC 2
///  --*----*----*--||--*----*--
///  __|____|____|      |    |
///     CALL 1   |______|    |
///               CALL 2|____|__
///                       CALL 3
///
/// The domain of the coarsened @ref overlap_view passed in the input argument
/// will contain the first elements of (the domains of) all the intervals in
/// the current set; hence, in the calls for the internal subdomains (CALL 1/3),
/// we extend the domain by the size of the right overlap (= 1 in this case) in
/// order to cover the whole local chunk.
///
/// At the boundary of the global domain, there is a degenerate interval with
/// size smaller than the overlap - it has been clipped so that the overlap
/// doesn't fall beyound the domain (see @ref overlap_partition):
///
///   LOC 1
///  --*----*----*
///  __|____|____|
///             |_|
///     CALL 1
///
/// Therefore, there is no need to extend the domain. If, however, the
/// degenerate interval is on a different location:
///
///    LOC 1       || LOC 2
///  --*------*----||----*
///  __|______|
///   CALL 1  |__________|
///            CALL 2  |___|
///                    CALL 3
///
/// we skip it (returning a @ref null_reference in CALL 3), as it has been
/// covered by the previous call.
///
/// Note that in other range-based algorithms, the size and number of the
/// degenerate intervals to be skipped may be greater than one (cf. the
/// algorithms in the "See also" section below).
///
/// @sa range_find_end, range_search
///
/// @ingroup nonModifyingAlgorithms
//////////////////////////////////////////////////////////////////////
template<typename Predicate>
class range_adjacent_find
{
private:
  Predicate m_predicate;

public:
  range_adjacent_find(Predicate predicate)
    : m_predicate(std::move(predicate))
  { }

  template<typename View>
  typename View::reference::reference
  operator()(View const& vw) const
  {
    auto source_vw = vw.container().view();

    using domain_type = typename decltype(source_vw)::domain_type;
    using result_type = typename View::reference::reference;

    if (vw.domain().last() == source_vw.domain().last()) {
      if (vw.domain().size() == 1)
        return result_type(null_reference());

      source_vw.set_domain(
        domain_type(vw.domain().first(), vw.domain().last()));
    }
    else {
      source_vw.set_domain(
        domain_type(vw.domain().first(), vw.domain().last() + 1));
    }

    auto iter =
      std::adjacent_find(source_vw.begin(), source_vw.end(), m_predicate);

    return iter != source_vw.end() ?
      *iter : result_type(null_reference());
  }

  void define_type(typer& t)
  { t.member(m_predicate); }
}; // class range_adjacent_find


//////////////////////////////////////////////////////////////////////
/// @brief Work function which returns its first argument if
///   non-null, and the second otherwise.
//////////////////////////////////////////////////////////////////////
struct find_reduce
{
  template<typename Reference1, typename Reference2>
  Reference1 operator()(Reference1 lhs, Reference2 rhs) const
  { return is_null_reference(lhs) ? rhs : lhs; }
};



//////////////////////////////////////////////////////////////////////
/// @brief Work function which returns its second argument if
///   non-null, and the first otherwise.
/// @ingroup nonModifyingAlgorithms
//////////////////////////////////////////////////////////////////////
struct find_end_reduce
{
  template<typename Reference1, typename Reference2>
  Reference1 operator()(Reference1&& lhs, Reference2&& rhs) const
  {
    return is_null_reference(rhs) ? lhs : rhs;
  }
};



} // namespace algo_detail

} // namespace stapl
#endif // STAPL_ALGORITHMS_ALGO_DETAIL_HPP
