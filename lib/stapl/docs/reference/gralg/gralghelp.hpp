#include <iostream>
#include <fstream>
#include <stapl/utility/do_once.hpp>
#include <stapl/stream.hpp>
#include <stapl/runtime.hpp>
#include <stapl/skeletons/serial.hpp>

#include <stapl/containers/generators/single.hpp>
#include <stapl/containers/graph/generators/binary_tree.hpp>
#include <stapl/containers/graph/generators/complete.hpp>
#include <stapl/containers/graph/generators/cycles.hpp>
#include <stapl/containers/graph/generators/list.hpp>
#include <stapl/containers/graph/generators/mesh.hpp>
#include <stapl/containers/graph/generators/random_neighborhood.hpp>
#include <stapl/containers/graph/generators/star.hpp>
#include <stapl/containers/graph/generators/torus.hpp>

using namespace std;

/*-------------------------------------------------------------------------*/

class msg {
private:
  string m_txt;
  stapl::stream<ofstream> m_zout;
public:
  msg(stapl::stream<ofstream> const& zout, const char *text)
    :  m_txt(text), m_zout(zout)
  { }
  typedef void result_type;
  result_type operator() () {
    m_zout << m_txt << endl;
  }
  void define_type(stapl::typer &t)
  {
    t.member(m_txt);
    t.member(m_zout);
  }
};

template<typename Value>
class msg_val {
private:
  string m_txt;
  Value m_val;
  stapl::stream<ofstream> m_zout;
public:
  msg_val(stapl::stream<ofstream> const & zout, const char *text, Value val)
    : m_txt(text), m_val(val), m_zout(zout)
  { }
  typedef void result_type;
  result_type operator() () {
    m_zout << m_txt << " " << m_val << endl;
  }
  void define_type(stapl::typer &t)
  {
    t.member(m_txt);
    t.member(m_val);
    t.member(m_zout);
  }
};

