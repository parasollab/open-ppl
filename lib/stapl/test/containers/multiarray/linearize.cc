/*
 // Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
 // component of the Texas A&M University System.

 // All rights reserved.

 // The information and source code contained herein is the exclusive
 // property of TEES and may not be disclosed, examined or reproduced
 // in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/runtime.hpp>
#include <stapl/utility/tuple.hpp>
#include <stapl/views/mapping_functions/linearization.hpp>
#include <stapl/containers/type_traits/default_traversal.hpp>

#include <test/algorithms/test_utils.h>
#include "../../test_report.hpp"

#include <algorithm>
#include <sstream>
#include <boost/iterator/counting_iterator.hpp>

#include <boost/fusion/include/copy.hpp>
#include <boost/fusion/include/transform.hpp>

#include <boost/preprocessor/seq/for_each.hpp>

using namespace stapl;

template<int N, typename T, typename Traversal>
struct linear_equality
{
  T m_size;
  T m_index;

  linear_equality(T const& size, T const& index)
    : m_size(size), m_index(index)
  { }

  bool operator()(bool b, size_t const& x)
  {
    get<N>(m_index) = x;

    return b && std::accumulate(
                  boost::make_counting_iterator<size_t>(0),
                  boost::make_counting_iterator<size_t>(get<N>(m_size)), b,
                  linear_equality<N-1, T, Traversal>(m_size, m_index));
  }
};

template<typename T, typename Traversal>
struct linear_equality<0, T, Traversal>
{
  T m_size;
  T m_index;

  linear_equality(T const& size, T const& index)
    : m_size(size), m_index(index)
  { }

  bool operator()(bool b, size_t const& x)
  {
    get<0>(m_index) = x;
    nd_linearize<T, Traversal> lin(m_size);
    nd_reverse_linearize<T, Traversal> r_lin(m_size);

    return b && r_lin(lin(m_index)) == m_index;
  }
};

struct random_size
{
  typedef size_t result_type;

  size_t operator()(size_t const& x) const
  {
    return rand() % x;
  }
};


// FIXME - use general identity...
//
template<typename T>
class my_identity
{
private:
  T const& m_t;

public:
  typedef T result_type;

  my_identity(T const& t)
   : m_t(t)
  { }

  template<typename Q>
  T operator()(Q const&) const
  {
    return m_t;
  }
};


#define TEST_LINEARIZATION_EQUALITY(r, n, dim)                                 \
{                                                                              \
  typedef default_traversal<dim>::type              traversal_type;            \
  typedef indexed_domain<size_t, dim, traversal_type>::size_type index_type;   \
  typedef result_of::reverse<traversal_type>::type  alternate_traversal_type;  \
  typedef index_type                                size_type;                 \
                                                                               \
  typedef stapl::result_of::transform<                                         \
            size_type,                                                         \
            my_identity<size_t>                                                \
          >::type transf_t;                                                    \
  transf_t size = tuple_ops::transform(size_type(), my_identity<size_t>(n));   \
                                                                               \
  boost::counting_iterator<size_t> b =                                         \
    boost::make_counting_iterator<size_t>(0);                                  \
  boost::counting_iterator<size_t> e =                                         \
    boost::make_counting_iterator<size_t>(n);                                  \
                                                                               \
  linear_equality<dim-1, transf_t, traversal_type> dole(size, index_type());   \
  bool passed = std::accumulate(b, e, true, dole);                             \
                                                                               \
  stapl_bool res(passed);                                                      \
  bool p = res.reduce();                                                       \
  std::stringstream str;                                                       \
  str << "Testing " << dim                                                     \
      <<  "d linearize (default traversal & homogeneous size)";                \
  STAPL_TEST_REPORT(p, str.str());                                             \
                                                                               \
  linear_equality<dim-1,                                                       \
                  transf_t,                                                    \
                  alternate_traversal_type> aole(size, index_type());          \
  passed = std::accumulate(b, e, true, aole);                                  \
                                                                               \
  res = stapl_bool(passed);                                                    \
  p = res.reduce();                                                            \
  str.str(std::string());                                                      \
  str << "Testing " << dim                                                     \
      <<  "d linearize (alt. traversal & homogeneous size)";                   \
  STAPL_TEST_REPORT(p, str.str());                                             \
/*                                                                             \
  sizev = boost::fusion::transform(sizev, random_size());                      \
  size_type hetero_size;                                                       \
  boost::fusion::copy(sizev, hetero_size);                                     \
  std::cout << sizev << std::endl;                                             \
                                                                               \
  linear_equality<dim-1, size_type, alternate_traversal_type>                  \
    aele(hetero_size, index_type());                                           \
  passed = std::accumulate(b, e, true, aele);                                  \
                                                                               \
  res = stapl_bool(passed);                                                    \
  p = res.reduce();                                                            \
  str.str(std::string());                                                      \
  str << "Testing " << dim                                                     \
      <<  "d linearize (default traversal & heterogeneous size)";              \
  STAPL_TEST_REPORT(p, str.str());                                          \*/\
}


stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 2) {
    std::cout << "usage: " << argv[0] << " n" << std::endl;
    exit(1);
  }

  const size_t n = atoi(argv[1]);

  #define DIMENSIONS (2)(3)(4)(5)
  BOOST_PP_SEQ_FOR_EACH(TEST_LINEARIZATION_EQUALITY, n, DIMENSIONS)
  #undef DIMENSIONS

  return EXIT_SUCCESS;
}

