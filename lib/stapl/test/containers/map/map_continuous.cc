/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/map/map.hpp>
#include "../../test_report.hpp"
#include <stapl/domains/continuous.hpp>
#include <stapl/containers/partitions/explicit.hpp>

using namespace stapl;

class point
{
public:
  explicit point() : m_x(0), m_y(0) { }

  explicit point (size_t const& i, size_t const& j) : m_x(i), m_y(j) { }

  void define_type(stapl::typer &t)
  {
    t.member(m_x);
    t.member(m_y);
  }

  size_t m_x;
  size_t m_y;
};

constexpr bool operator< (point const& id1, point const& id2)
{
  return id1.m_x < id2.m_x || (!(id2.m_x < id1.m_x) && id1.m_y < id2.m_y);
}

constexpr bool operator> (point const& id1, point const& id2)
{
  return  operator< (id2,id1);
}

constexpr bool operator<=(point const& id1, point const& id2)
{
  return !operator> (id1,id2);
}

constexpr bool operator>=(point const& id1, point const& id2)
{
  return !operator< (id1,id2);
}

constexpr bool operator==(point const& id1, point const& id2)
{
    return id1.m_x == id2.m_x && id1.m_y == id2.m_y;
}

constexpr bool operator!=(point const& id1, point const& id2)
{
  return !(id1==id2);
}

namespace stapl {

template<>
struct index_bounds<point>
{
  static point invalid ()
  {
    return point(stapl::index_bounds<size_t>::invalid(),
                 stapl::index_bounds<size_t>::invalid());
  }

};

template<>
struct hash<point>
{
  std::size_t operator()(point const& t) const
  {
    return 0;
  }
};

}

stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 2) {
    std::cerr << "usage: exe n" << std::endl;
    exit(1);
  }

  const size_t n = atoi(argv[1]);

  typedef point key_type;
  typedef continuous_domain<key_type> domain_type;
  typedef explicit_partition<domain_type> partition_type;
  typedef stapl::map<key_type, char,
                     std::less<key_type>, partition_type> container_type;

  domain_type dom(point(0,0), point(n-1, n-1));
  std::vector<domain_type> doms;
  size_t block = n / get_num_locations();
  for (size_t i = 0; i < get_num_locations(); ++i)
    doms.push_back(domain_type(point(i*block, i*block),
                               point(i*block + block, i*block + block)));
  partition_type part(dom, doms);
  container_type m(part);


  if (get_location_id() == 0) {
    m.insert(std::make_pair(point(0,0), 'a'));
    m.insert(std::make_pair(point(0,1), 'c'));
  } else if (get_location_id() == 1) {
    m.insert(std::make_pair(point(n-1,n-1), 'b'));
  }

  // ensure assignments are complete before reading data.
  rmi_fence();

  char x = m[point(0,1)];
  char y = m[point(n-1,n-1)];

  bool passed = x == 'c' && y == 'b';

  STAPL_TEST_REPORT(passed, "Testing insert and operator[]");

  return EXIT_SUCCESS;
}
