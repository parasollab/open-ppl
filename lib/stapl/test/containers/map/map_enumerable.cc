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

class group
{
public:
  explicit group() : m_x(0), m_y(0) { }

  explicit group (size_t const& i, char const& j) : m_x(i), m_y(j) { }

  explicit group (size_t const& i) : m_x(i), m_y(0) { }

  bool operator==(const group& p) const
  {
    return p.m_x == this->m_x && p.m_y == this->m_y;
  }

  group operator+(group const& other) const
  {
    return group(m_x + other.m_x, m_y + other.m_y);
  }

  size_t operator-(group const& other) const
  {
    return m_x - other.m_x;
  }

  inline bool operator< (group const& other) const
  {
    return m_x < other.m_x || (!(other.m_x < m_x) && m_y < other.m_y);
  }

  inline bool operator> (group const& other) const
  {
    return !(*this == other) && !(*this < other);
  }

  inline bool operator<= (group const& other) const
  {
    return *this < other || *this == other;
  }

  inline bool operator>=(group const& other) const
  {
    return !operator< (other);
  }

  group operator+(int s) const
  {
    return group(m_x+s, m_y);
  }

  group operator-(int s) const
  {
    return group(m_x-s, m_y);
  }

  void define_type(stapl::typer &t)
  {
    t.member(m_x);
    t.member(m_y);
  }

  size_t m_x;
  char m_y;
};

namespace stapl {

template<>
struct index_bounds<group>
{
  static group invalid ()
  {
    return group(stapl::index_bounds<size_t>::invalid(),
                 stapl::index_bounds<char>::invalid());
  }

};

template<>
struct hash<group>
{
  std::size_t operator()(const group& t) const
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

  typedef indexed_domain<group>  domain_type;
  typedef stapl::map<group, int> container_type;

  domain_type dom(group(0,0), group(n-1, 'z'));

  container_type m(dom);

  if (get_location_id() == 0)
    m.insert(std::make_pair(group(0, 'a'), 2));
  else if (get_location_id() == get_num_locations() - 1)
    m.insert(std::make_pair(group(n-1, 'd'), 3));

  int x = m[group(0, 'a')];
  int y = m[group(n-1, 'd')];

  bool passed = x == 2 && y == 3;

  STAPL_TEST_REPORT(passed, "Testing insert and operator[]");

  return EXIT_SUCCESS;
}
