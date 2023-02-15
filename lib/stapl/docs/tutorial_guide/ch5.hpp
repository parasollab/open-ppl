#include <iostream>
#include <string>

#include <stapl/utility/do_once.hpp>
#include <stapl/utility/tuple.hpp>
#include <stapl/domains/indexed.hpp>

#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>
#include <stapl/runtime.hpp>
#include <stapl/skeletons/serial.hpp>
#include <stapl/stream.hpp>

using namespace std;

/*-------------------------------------------------------------------------*/

extern int prime_nums[], rand_nums[], fibo20[];
extern unsigned int data_cnt;

typedef stapl::indexed_domain<int>                    ndx_dom_tp;

typedef stapl::negate<int> neg_int_wf;
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
    : m_txt(text), m_zout(zout)
  { }
  typedef void result_type;
  result_type operator() () {
    m_zout << m_txt << endl;
  }
  void define_type(stapl::typer &t) {
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
  void define_type(stapl::typer &t) {
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

struct show_log_ndx_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  typedef void result_type;

  show_log_ndx_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  template <typename Ref1, typename Ref2>
  result_type operator()(Ref1 val, Ref2 pos)
  {
    switch( pos ) {
    case 0: case 1:
    case 2: case 3: case 4: case 5:
    case 6: case 7: case 8: case 9:
    case 20: case 30: case 40: case 50:
    case 60: case 70: case 80: case 90:
    case 200: case 300: case 400: case 500:
    case 600: case 700: case 800: case 900:
    case 2000: case 3000: case 4000: case 5000:
    case 6000: case 7000: case 8000: case 9000:
    case 20000: case 30000: case 40000: case 50000:
    case 60000: case 70000: case 80000: case 90000:
    case 200000: case 300000: case 400000: case 500000:
    case 600000: case 700000: case 800000: case 900000:
      m_zout << "[" << pos << "]= " << val << " ";
      break;
    case 10: case 100: case 1000:
    case 10000: case 100000: case 1000000:
      m_zout << "[" << pos << "]= " << val << endl;
      break;
    default: ;
      // no display
      break;
    }
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

struct show_log_map_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  typedef void result_type;

  show_log_map_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  template <typename Ref1, typename Ref2>
  result_type operator()(Ref1 val, Ref2 pos)
  {
    switch( pos ) {
    case 0: case 1:
    case 2: case 3: case 4: case 5:
    case 6: case 7: case 8: case 9: case 10:
    case 20: case 30: case 40: case 50:
    case 60: case 70: case 80: case 90: case 100:
    case 200: case 300: case 400: case 500:
    case 600: case 700: case 800: case 900: case 1000:
    case 2000: case 3000: case 4000: case 5000:
    case 6000: case 7000: case 8000: case 9000: case 10000:
    case 20000: case 30000: case 40000: case 50000:
    case 60000: case 70000: case 80000: case 90000: case 100000:
    case 200000: case 300000: case 400000: case 500000:
    case 600000: case 700000: case 800000: case 900000: case 1000000:
      m_zout << "{" << val.first << "}= " << val.second << endl;
      break;
    default: ;
      // no display
      break;
    }
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

