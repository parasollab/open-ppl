/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in dom without explicit written authorization from TEES.
*/

// Per-view localization needed for the cases when there are fewer overlapped
// subdomains than number of locations. See the comment at the first map_reduce
// call in stapl_main.
#define STAPL_PER_VIEW_LOCALIZATION

// Set to 1 to see the original domain and the overlapepd subdomains (and for
// the contiguous case also whether the subdomains are fully included in blocks
// of given size).
#define PRINT_DOMAINS  0

// The default projection performs poorly when the domain consists of multiple
// intervals. The invertible_metadata_projection has good performance if the
// number of intervals is less than or equal to the number of locations. In
// the extreme case of n intervals of size 1, while much slower than the
// default array_view of same size, it is still significantly faster than
// the default projection (see also test_domset1D.cc).
#define USE_INVERTIBLE_MD_PROJECTION  0

#include <stapl/array.hpp>
#include <stapl/views/overlap_view.hpp>
#include <stapl/views/counting_view.hpp>
#include <stapl/views/domain_view.hpp>
#include <stapl/containers/type_traits/is_base_container.hpp>
#include <stapl/algorithms/functional.hpp>
#include <stapl/algorithms/numeric_fwd.hpp>
#include <stapl/algorithms/algorithm_fwd.hpp>

#include "../test_report.hpp"

#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>

using namespace stapl;

using cont_subdomain_bounds_t = std::vector<std::tuple<size_t, size_t, bool>>;

using sparse_dom_t = domset1D<size_t>;
using sparse_subdomains_t = std::vector<sparse_dom_t>;

using cont_t = array<int>;
using sparse_view_t = array_view<cont_t, sparse_dom_t>;

#if USE_INVERTIBLE_MD_PROJECTION
#include <stapl/views/metadata/coarsening_traits.hpp>

namespace stapl {
namespace metadata {

//////////////////////////////////////////////////////////////////////
/// @brief Specialization of metadata projection for the view with
///        @ref domset1D domain.
//////////////////////////////////////////////////////////////////////
template<>
struct coarsening_traits<sparse_view_t>
{
  template<typename P>
  struct construct_projection
  { using type = invertible_metadata_projection<const sparse_view_t, P>; };
};

} // namespace metadata
} // namespace stapl
#endif

//////////////////////////////////////////////////////////////////////
/// @brief Get the expected bounds for each overlapping subdomain of the
///   contiguous domain of size @p n.
///
/// @param c   Size of the "core" of each subdomain (number of elements that
///   are not overlapped).
/// @param l   Number of elements that are overlapped to the left.
/// @param r   Number of elements that are overlapped to the right.
/// @param n   Size of the original domain.
/// @param bs  Block size used to determine whether each overlapping subdomain
///   is fully contained in a block of that size (and hence the accesses to its
///   elements should be local). Should be set to zero for overlap_views over
///   a container distributed by uneven blocks -- this simple localization test
///   will then be disabled.
///
/// @returns Vector of tuples
///   {left_boundary, right_boundary, overlapping_subdomain_is_in_one_block}
//////////////////////////////////////////////////////////////////////
cont_subdomain_bounds_t
get_cont_subdomain_bounds(size_t c, size_t l, size_t r, size_t n, size_t bs)
{
  size_t n_subdom = n/c;
  if (n_subdom * c != n)
    ++n_subdom;

  cont_subdomain_bounds_t ret;
  ret.reserve(n_subdom);

  for (size_t i = 0; i < n_subdom; ++i)
  {
    size_t dom_sz = l + c + r;
    std::ptrdiff_t lbound = i*c - l;
    size_t rbound = lbound + dom_sz - 1;

    if (lbound < 0)   lbound = 0;
    if (rbound > n-1) rbound = n-1;

    bool fully_within_block = (bs>0) ? (lbound/bs == rbound/bs) : false;
    ret.push_back( std::make_tuple(lbound, rbound, fully_within_block) );
  }

  return ret;
}

//////////////////////////////////////////////////////////////////////
/// @brief Get the expected overlapping subdomains of the sparse domain of
///   size @p n.
///
/// @param c    Size of the "core" of each subdomain (number of elements
///   that are not overlapped).
/// @param l    Number of elements that are overlapped to the left.
/// @param r    Number of elements that are overlapped to the right.
/// @param odom The original domain (@ref domset1D or @ref domainset1D).
///
/// @returns std::vector of subdomains of the same type as the original domain
///   that form the overlapping partition of the original domain as specified
///   by the parameters @p c, @p l and @p r.
//////////////////////////////////////////////////////////////////////
sparse_subdomains_t
get_sparse_subdomains(size_t c, size_t l, size_t r, sparse_dom_t const& odom)
{
  const size_t n = odom.size();

  size_t n_subdom = n/c;
  if (n_subdom * c != n)
    ++n_subdom;

  sparse_subdomains_t ret;
  ret.reserve(n_subdom);

  for (size_t i = 0; i < n_subdom; ++i)
  {
    size_t dom_sz = l + c + r;
    std::ptrdiff_t lbound = i*c - l;
    size_t rbound = lbound + dom_sz - 1;

    if (lbound < 0)   lbound = 0;
    if (rbound > n-1) rbound = n-1;

    sparse_dom_t subdom(odom.advance(odom.first(), lbound),
                        odom.advance(odom.first(), rbound),
                        odom);

    if (!subdom.empty())
      ret.push_back(std::move(subdom));
  }

  return ret;
}

//////////////////////////////////////////////////////////////////////
/// @brief Print the subdomain bounds information obtained from
///   @ref get_cont_subdomain_bounds.
//////////////////////////////////////////////////////////////////////
void print_subdomains(cont_subdomain_bounds_t const& subdom_bounds,
  size_t c, size_t l, size_t r, size_t n, size_t bs)
{
#if PRINT_DOMAINS
  std::cout << std::string(70, '-') << "\n";

  std::cout << boost::format(
    "sz = %1%, bs = %2%, c = %3%, l = %4%, r = %5%\n")
      % n % bs % c % l % r;

  std::cout << "\n";

  for (auto const& sb : subdom_bounds)
    std::cout << boost::format("  [%1%, %2%] : %3%\n") %
      get<0>(sb) % get<1>(sb) % get<2>(sb);

  std::cout << std::endl;
#endif
}

//////////////////////////////////////////////////////////////////////
/// @brief Print the subdomains obtained from @ref get_sparse_subdomains.
//////////////////////////////////////////////////////////////////////
void print_subdomains(sparse_subdomains_t const& subdomains,
  size_t c, size_t l, size_t r, size_t n, size_t rf)
{
#if PRINT_DOMAINS
  std::cout << std::string(70, '-') << "\n";

  std::cout << boost::format(
    "sz = %1%, c = %2%, l = %3%, r = %4%, rf = %5%\n")
      % n % c % l % r % rf;

  std::cout << "\n";

  for (auto const& sb : subdomains)
    std::cout << sb << std::endl;

  std::cout << std::endl;
#endif
}

//////////////////////////////////////////////////////////////////////
/// @brief Workfunction that checks if the subview generated by the
///   @ref overlap_view references correct elements of the underlying
///   container and is localized in the cases when it can be.
///
/// Used for an overlap view over a view with contiguous domain.
//////////////////////////////////////////////////////////////////////
struct compare_cont_doms_wf
{
  using result_type = bool;

  compare_cont_doms_wf(bool test_localization)
    : m_test_localization(test_localization)
  { }

  template<typename SubView>
  bool operator()(SubView&& subview,
    cont_subdomain_bounds_t const& exp_subdom_bounds, size_t idx) const
  {
    size_t l, r;
    bool localizable;
    std::tie(l,r,localizable) = exp_subdom_bounds[idx];

    bool loc_test = true;
    if (m_test_localization) {
      loc_test = localizable ?
          is_base_container<underlying_container_t<SubView>>::value
        : !is_base_container<underlying_container_t<SubView>>::value;
    }

    return stapl::equal(subview, counting_view(subview.size(), l)) && loc_test;
  }

  void define_type(typer& t)
  { t.member(m_test_localization); }

private:
  bool m_test_localization;
};

//////////////////////////////////////////////////////////////////////
/// @brief Workfunction that checks if the subview generated by the
///   @ref overlap_view references correct elements of the underlying
///   container.
///
/// Used for an overlap view over a view with a sparse domain.
//////////////////////////////////////////////////////////////////////
struct compare_sparse_doms_wf
{
  compare_sparse_doms_wf(sparse_subdomains_t exp_subdoms)
    : m_exp_subdoms(std::move(exp_subdoms))
  { }

  template<typename SubView>
  bool operator()(SubView&& subview, size_t idx) const
  {
    return map_reduce(check_value_wf(m_exp_subdoms[idx]), logical_and<bool>(),
      subview, counting_view(subview.size(), 0ul));
  }

  void define_type(typer& t)
  { t.member(m_exp_subdoms); }

private:
  sparse_subdomains_t m_exp_subdoms;

  struct check_value_wf
  {
    using result_type = bool;

    check_value_wf(sparse_dom_t exp_subdom)
      : m_exp_subdom(std::move(exp_subdom))
    { }

    template<typename Ref>
    bool operator()(Ref&& elem, size_t idx) const
    { return elem == m_exp_subdom.advance(m_exp_subdom.first(), idx); }

    void define_type(typer& t)
    { t.member(m_exp_subdom); }

  private:
    sparse_dom_t m_exp_subdom;
  };
};

stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 2)
  {
    do_once([] {
      std::cout << "Usage: exec_cmd ./test_overlap_view n [rf]\n"
                   "n is the size of the underlying array and rf is the"
                   " optional number enabling the non-contiguous test that will"
                   " use a domain [0,n) with every rf-th element removed.\n";
    });

    return EXIT_FAILURE;
  }

  const size_t n = boost::lexical_cast<size_t>(argv[1]);
  const size_t nloc = get_num_locations();

  // Determine the block size of the balanced distribution, used to
  // test the localizability of the overlapped subviews.
  const size_t bs = std::max(1ul, n/nloc);

  // Perform the simple localizability test only in the case of a perfectly
  // balanced distribution.
  const bool loc_test = (bs * nloc == n);

  // Determine the frequency at which elements of the original domain should
  // be removed in order to create a sparse domain, used for testing the
  // non-contiguous case.
  size_t rf = 0;
  if (argc == 3) {
    rf = boost::lexical_cast<size_t>(argv[2]);

    if (rf == 0)
      do_once([] {
        std::cout << "WARNING: Zero removal frequency specified: only the "
          "contiguous tests will be performed\n";
      });
    else if (rf == 1) {
      do_once([] {
        std::cout << "WARNING: Removal frequency of 1 specified: only the "
          "contiguous tests will be performed as the sparse domain used for "
          "the non-contigous tests would be empty.\n";
      });
    }
  }

  // Create the basic container used as an underlying container for both tested
  // overlap_views (with contiguous and sparse domain, respectively).
  cont_t a(n);
  auto avw = make_array_view(a);
  iota(avw, 0);

  // Create the sparse domain used in the non-contigous test.
  sparse_dom_t sparse_dom(n);
  if (rf > 1)
    for (size_t i = 0; i < n; ++i)
      if ((i+1) % rf == 0)
        sparse_dom -= i;

#if PRINT_DOMAINS
  stapl::do_once([&sparse_dom, &avw, rf]() {
    std::cout << "Original contiguous domain: " << avw.domain() << std::endl;
    if (rf > 1)
      std::cout << "Original sparse domain: " << sparse_dom << std::endl;
  });
#endif

  //
  // Define and run the individual test cases (different overlaps)
  //

  const auto lrparams = (bs > 1 && nloc > 1) ?
    std::vector<size_t> { 0ul, 1ul, 2ul, bs, size_t(1.5*bs), 2*bs } :
    std::vector<size_t> { 0ul, 1ul, 2ul };
  const auto cparams = (bs > 1 && nloc > 1) ?
    std::vector<size_t> { 1ul, 2ul, bs, size_t(1.5*bs), 2*bs } :
    std::vector<size_t> { 1ul, 2ul };

  for (size_t c : cparams)
  {
    for (size_t l : lrparams)
    {
      for (size_t r : lrparams)
      {
        //
        // Test the view with a contiguous domain
        //

        auto cont_subdom_bnds =
          get_cont_subdomain_bounds(c, l, r, n, loc_test ? bs : 0);

        do_once([&] {
          print_subdomains(cont_subdom_bnds, c, l, r, n, bs);
        });

        auto a_ovw = make_overlap_view(avw, c, l, r);
        bool passed = a_ovw.size() == cont_subdom_bnds.size();

        // When there are fewer overlapped subdomains than the number of
        // locations, the overlap_view may be distributed over a possibly
        // non-contiguous subset of locations and the localization may fail for
        // the counting_view which is distributed over the contiguous subset.
        // Without per-view localization, the fast task creation would then be
        // avoided for both views and the test for localizability of the
        // overlapped subviews would fail.
        passed &= map_reduce(compare_cont_doms_wf(loc_test),logical_and<bool>(),
          a_ovw, make_repeat_view(cont_subdom_bnds),
          counting_view(a_ovw.size(), 0ul));

        STAPL_TEST_REPORT(passed, boost::str(boost::format(
          "Testing overlap_view<avw(ar<int>(%1%), balanced, indexed_domain), "
            "%2%, %3%, %4%>") % n % c % l % r));

        //
        // If valid removal frequency has been specified, test the view with a
        // contiguous domain
        //
        if (rf > 1) {
          auto sparse_subdoms = get_sparse_subdomains(c, l, r, sparse_dom);

          do_once([&] {
            print_subdomains(sparse_subdoms, c, l, r, sparse_dom.size(), rf);
          });

          sparse_view_t svw(a, sparse_dom);
          auto s_ovw = make_overlap_view(svw, c, l, r);

          passed = s_ovw.size() == sparse_subdoms.size();

          passed &= map_reduce(
            compare_sparse_doms_wf(sparse_subdoms), logical_and<bool>(),
            s_ovw, counting_view(s_ovw.size(), 0ul));

          STAPL_TEST_REPORT(passed, boost::str(boost::format(
          "Testing overlap_view<avw(ar<int>(%1%), balanced, domset1D(rf=%5%)), "
            "%2%, %3%, %4%>") % n % c % l % r % rf));
        }
      }
    }
  }

  return EXIT_SUCCESS;
}
