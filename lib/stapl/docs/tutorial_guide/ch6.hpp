#include <iostream>
#include <sstream>
#include <string>

#include <stapl/utility/do_once.hpp>
#include <stapl/vector.hpp>
#include <stapl/array.hpp>
#include <stapl/graph.hpp>
#include <stapl/utility/tuple.hpp>
#include <stapl/domains/indexed.hpp>
#include <stapl/algorithm.hpp>
#include <stapl/stream.hpp>
#include <stapl/runtime.hpp>
#include <stapl/skeletons/serial.hpp>

/*-------------------------------------------------------------------------*/

using namespace std;

typedef stapl::vector<int> vec_int_tp;
typedef stapl::vector_view< vec_int_tp > vec_int_vw_tp;

typedef stapl::array<int> ary_int_tp;
typedef stapl::array_view<ary_int_tp> ary_int_vw_tp;

typedef stapl::array<size_t> ary_sz_tp;
typedef stapl::array_view<ary_sz_tp> ary_sz_vw_tp;

typedef stapl::indexed_domain<int> ndx_dom_tp;

typedef stapl::identity<int> id_int_wf;
typedef stapl::identity<size_t> id_un_wf;
typedef stapl::plus<int> add_int_wf;
typedef stapl::minus<int> sub_int_wf;
typedef stapl::min<int> min_int_wf;
typedef stapl::max<int> max_int_wf;
typedef stapl::bit_xor<size_t> xor_un_wf;
typedef stapl::bit_or<size_t> ior_un_wf;
typedef stapl::bit_and<size_t> and_un_wf;

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
  void define_type(stapl::typer& t)
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
  void define_type(stapl::typer& t)
  {
    t.member(m_txt);
    t.member(m_val);
    t.member(m_zout);
  }
};

/*-------------------------------------------------------------------------*/

class put_ndx_val_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  put_ndx_val_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename Ref1, typename Ref2>
  result_type operator()(Ref1 pos, Ref2 val) {
    m_zout << "[" << pos << "]= " << val <<  "\n";
  }

  void define_type(stapl::typer& t) {
    t.member(m_zout);
  }
};

class put_val_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  put_val_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename Ref>
  result_type operator()(Ref val) {
    m_zout << val << " ";
  }

  void define_type(stapl::typer& t) {
    t.member(m_zout);
  }
};

class get_pair_wf
{
private:
  stapl::stream<ifstream> m_zin;
public:
  typedef void result_type;

  get_pair_wf(stapl::stream<ifstream> const& zin)
    : m_zin(zin)
  { }

  template <typename Ref1, typename Ref2>
  result_type operator()(Ref1 first, Ref2 second)
  {
    m_zin >> first >> second;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zin);
  }
};

class get_triple_wf
{
private:
  stapl::stream<ifstream> m_zin;
public:
  typedef void result_type;

  get_triple_wf(stapl::stream<ifstream> const& zin)
    : m_zin(zin)
  { }

  template <typename Ref1, typename Ref2, typename Ref3>
  result_type operator()(Ref1 first, Ref2 second, Ref3 third)
  {
    m_zin >> first >> second >> third;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zin);
  }
};

/*-------------------------------------------------------------------------*/

struct roll_wf {
  typedef void result_type;
  template <typename View1, typename View2>
  void operator()(View1 length, View2 val) const {
    length = 1 + (rand() % val);
  }
};

/*-------------------------------------------------------------------------*/

struct inner_graf_wf {
  typedef int result_type;
  template<typename Vertex>
  int operator()(Vertex v) const {
    return v.property();
  }
};

struct outer_graf_wf {
  typedef int result_type;
  template<typename Vertex>
  int operator()(Vertex v) const {
    return stapl::map_reduce(inner_graf_wf(), stapl::plus<int>(), v.property());
  }
};
