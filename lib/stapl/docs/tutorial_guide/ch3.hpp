#include <iostream>
#include <string>

#include <stapl/utility/do_once.hpp> //
#include <stapl/views/vector_view.hpp> //
#include <stapl/containers/vector/vector.hpp> //

#include <stapl/views/array_view.hpp>
#include <stapl/containers/array/array.hpp>

#include <stapl/utility/tuple.hpp>
#include <stapl/domains/indexed.hpp>
#include <stapl/views/map_view.hpp>
#include <stapl/containers/map/map.hpp>

#include <stapl/algorithms/algorithm.hpp> //
#include <stapl/algorithms/functional.hpp> //
#include <stapl/stream.hpp> //
#include <stapl/runtime.hpp> //
#include <stapl/skeletons/serial.hpp> //

/*-------------------------------------------------------------------------*/

using namespace std;

extern int prime_nums[], rand_nums[], fibo20[];
extern unsigned int data_cnt;

using namespace std;

typedef stapl::vector<int> vec_int_tp;
typedef stapl::vector_view< vec_int_tp > vec_int_vw_tp;

typedef stapl::array<int> ary_int_tp;
typedef stapl::array_view<ary_int_tp> ary_int_vw_tp;

typedef stapl::map<int,int> map_int_tp;
typedef stapl::map_view<map_int_tp> map_int_vw_tp;

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

/*-------------------------------------------------------------------------*/

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

class put_map_val_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  put_map_val_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename Ref>
  result_type operator()(Ref pair) {
    typename Ref::first_reference left = pair.first;
    typename Ref::second_reference right = pair.second;
    m_zout << "{" << left << "}= " << right << "\n";
  }

  void define_type(stapl::typer& t) {
    t.member(m_zout);
  }
};

class get_val_wf
{
private:
  stapl::stream<ifstream> m_zin;
public:
  get_val_wf(stapl::stream<ifstream> const& zin)
    : m_zin(zin)
  { }

  typedef void result_type;
  template <typename Ref>
  result_type operator()(Ref val) {
    m_zin >> val;
  }

  void define_type(stapl::typer& t) {
    t.member(m_zin);
  }
};

class get_triple_wf
{
private:
  stapl::stream<ifstream> m_zin;
public:
  get_triple_wf(stapl::stream<ifstream> const& zin)
    : m_zin(zin)
  { }

  typedef void result_type;
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

class show_map_indexable_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  typedef void result_type;

  show_map_indexable_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  template <typename Element>
  result_type operator()(Element elem)
  {
    typename Element::first_reference first = elem.first;
    typename Element::second_reference second = elem.second;
    m_zout << "{" << first << "} #" << second.size() << "\n";
    stapl::serial_io(put_val_wf(m_zout), second);
    stapl::do_once( msg(m_zout, "\n" ) );
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

class show_indexable_map_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  typedef void result_type;

  show_indexable_map_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  template <typename Element>
  result_type operator()(Element elem)
  {
    stapl::serial_io(put_map_val_wf(m_zout), elem);
    stapl::do_once( msg(m_zout, "\n" ) );
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

class show_inner_map_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  typedef void result_type;

  show_inner_map_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  template <typename Pair>
  result_type operator()(Pair pair)
  {
    typename Pair::first_reference left = pair.first;
    typename Pair::second_reference right = pair.second;
    m_zout << "{" << left << "}= " << right << "\n";
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

class show_outer_map_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  typedef void result_type;

  show_outer_map_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  template <typename Pair>
  result_type operator()(Pair pair)
  {
    typename Pair::first_reference left = pair.first;
    typename Pair::second_reference right = pair.second;
    m_zout << "{" << left << "} #" << right.size() << "\n";
    stapl::serial_io(show_inner_map_wf(m_zout), pair.second);
    stapl::do_once( msg(m_zout, "\n" ) );
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

/*-------------------------------------------------------------------------*/

struct nested_cksum_wf {
  typedef size_t result_type;
  template<typename Ref1>
  size_t operator()(Ref1 v1) {
    return stapl::map_reduce(id_un_wf(), xor_un_wf(), v1);
  }
};

struct roll_wf {
  typedef void result_type;
  template <typename View1, typename View2>
  void operator()(View1 length, View2 val) const {
    length = 1 + (rand() % val);
  }
};

// map check sums

struct inner_cksum_wf {
  typedef int result_type;
  template<typename Element>
  int operator()(Element elem) const {
    return elem;
  }
};

struct outer_map_cksum_wf {
  typedef int result_type;
  template<typename Element>
  int operator()(Element elem) const {
    return stapl::map_reduce(inner_cksum_wf(), xor_un_wf(), elem.second);
  }
};

struct inner_map_wf {
  typedef int result_type;
  template<typename Element>
  int operator()(Element elem) const {
    typename Element::second_reference map_arg = elem.second;
    return map_arg;
  }
};

struct inner_map_cksum_wf {
  typedef int result_type;
  template<typename Element>
  int operator()(Element elem) const {
    return stapl::map_reduce(inner_map_wf(), xor_un_wf(), elem);
  }
};

struct multi_map_cksum_wf {
  typedef int result_type;
  template<typename Element>
  int operator()(Element elem) const {
    return stapl::map_reduce(inner_map_wf(), xor_un_wf(), elem.second);
  }
};

