#include <iostream>
#include <fstream>
#include <stapl/utility/do_once.hpp>
#include <stapl/stream.hpp>
#include <stapl/skeletons/serial.hpp>
#include <boost/bind.hpp>
#include <random>

#include "confint.hpp"
#include "timing.h"

extern int prime_nums[];
extern int rand_nums[];
extern int fib20[];
extern int init_count;

using namespace std;

struct roll_wf
{
  typedef void result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 length, View2 val) const
  {
    length = 1 + (rand()%val);
  }
};

typedef stapl::array<size_t> ary_sz_tp;
typedef stapl::array_view<ary_sz_tp> ary_sz_vw_tp;

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

/*-------------------------------------------------------------------------*/

template<typename Diff>
class rand_num_gen
{
  typedef Diff result_type;
  typedef boost::random::uniform_int_distribution<Diff> distrib_tp;
  boost::random::mt19937 m_gen;
public:
  rand_num_gen(unsigned int seed) : m_gen(seed)
  { }
  Diff operator()(void)
  { return distrib_tp()(m_gen); }
  Diff operator()(Diff n) 
  { return distrib_tp(0, n)(m_gen); }
  void define_type(stapl::typer& t)
  { t.member(m_gen); }
};

/*-------------------------------------------------------------------------*/

class init_iota_vec_wf
{
private:
  int m_base;
public:
  init_iota_vec_wf(int base)
    : m_base(base)
  { }
  typedef void result_type;
  template <typename Arg1, typename Arg2>
  result_type operator()(Arg1 vec, Arg2 size) {
    vec.resize(size);
    iota( vec.begin(), vec.end(), m_base );
  }
  void define_type(stapl::typer &t)
  {
    t.member(m_base);
  }
};

class init_seq_vec_wf
{
private:
  int m_base;
  int m_step;
public:
  init_seq_vec_wf(int base, int step)
    : m_base(base)
  { }
  typedef void result_type;
  template <typename Arg1, typename Arg2>
  result_type operator()(Arg1 vec, Arg2 size) {
    vec.resize(size);
    for (int i = 0; i < size; i++ ) {
      vec[i] = i + m_base;
    }
  }
  void define_type(stapl::typer &t)
  {
    t.member(m_base);
    t.member(m_step);
  }
};

class init_block_vec_wf
{
private:
  int m_base;
  int m_rep;
public:
  init_block_vec_wf(int base, int rep)
    : m_base(base), m_rep(rep)
  { }
  typedef void result_type;
  template <typename Arg1, typename Arg2>
  result_type operator()(Arg1 vec, Arg2 size) {
    vec.resize(size);
    int len = m_rep+1;
    for (int i = m_base; i < size/len; i++) {
      int start = i * len;
      for (int j = 0; j < len; j++ ) {
        vec[start+j] = j;
      }
    }
  }
  void define_type(stapl::typer &t)
  {
    t.member(m_base);
    t.member(m_rep);
  }
};

class init_rand_vec_wf
{
public:
  typedef void result_type;
  template <typename Arg1, typename Arg2>
  result_type operator()(Arg1 vec, Arg2 size) {
    vec.resize(size);
    std::default_random_engine eng;
    std::uniform_int_distribution<int> dist(0,size-1);
    for( int i=0; i<size; i++ ) {
      vec[i] = dist(eng);
    }

    //rand_num_gen<atom_tp>(16807);
    //generate(vec.begin(), vec.end(), rand);
  }
};

///////////////////////////////////////////////////////////////////////////

template <typename atom_tp>
struct odd_atom_pred
{
  typedef atom_tp argument_type;
  typedef bool result_type;
  result_type operator() (const atom_tp& x) const
  {
    return 1 == (x % 2);
  }
};

template <typename atom_tp>
struct even_atom_pred
{
  typedef atom_tp argument_type;
  typedef bool result_type;
  result_type operator() (const atom_tp& x) const
  {
    return 0 == (x % 2);
  }
};

template <typename atom_tp>
struct neg_atom_pred
{
  typedef atom_tp argument_type;
  typedef bool result_type;
  result_type operator() (const atom_tp& x) const
  {
    return x < 0;
  }
};

template <typename atom_tp>
struct eq_atom_pred
{
  typedef atom_tp argument_type;
  typedef bool result_type;
  result_type operator() (const atom_tp& x, const atom_tp& y) const
  {
    return x == y;
  }
};

template <typename atom_tp>
struct less_atom_pred
{
  typedef atom_tp argument_type;
  typedef bool result_type;
  result_type operator() (const atom_tp& x, const atom_tp& y) const
  {
    return x < y;
  }
};

template <typename atom_tp>
struct half_atom_pred
{
private:
  atom_tp m_mid;
public:
  typedef atom_tp argument_type;
  half_atom_pred(atom_tp mid) :
    m_mid(mid)
  { }
  typedef bool result_type;
  result_type operator() (const atom_tp& x) const
  {
    return x < m_mid;
  }
  void define_type(stapl::typer& t )
  {
    t.member(m_mid);
  }
};

template <typename atom_tp>
struct pivot_atom_pred
{
private:
  atom_tp m_piv;
public:
  typedef atom_tp    argument_type;
  pivot_atom_pred(atom_tp piv) :
    m_piv(piv)
  { }
  typedef bool result_type;
  result_type operator() (const atom_tp& x) const
  {
    return x < m_piv;
  }
  void define_type(stapl::typer& t )
  {
    t.member(m_piv);
  }
};

template <typename atom_tp>
struct cut_atom_pred
{
private:
  atom_tp m_cut;
public:
  cut_atom_pred( atom_tp cut ) :
    m_cut(cut)
  { }
  typedef atom_tp argument_type;
  typedef bool result_type;
  result_type operator() (const atom_tp& x) const
  {
    return x < m_cut;
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_cut);
  }
};

///////////////////////////////////////////////////////////////////////////

template <typename stl_tp>
struct odd_stl_pred
{
  typedef stl_tp argument_type;
  typedef bool result_type;
  result_type operator() (const stl_tp& x) const
  {
    bool res = true;
    for( auto it = x.begin(); it != x.end(); it++ ) {
      res &= 1 == (*it % 2);
    }
    return res;
  }
};

template <typename stl_tp>
struct even_stl_pred
{
  typedef stl_tp argument_type;
  typedef bool result_type;
  result_type operator() (const stl_tp& x) const
  {
    bool res = true;
    for( auto it = x.begin(); it != x.end(); it++ ) {
      res &= 0 == (*it % 2);
    }
    return res;
  }
};

template <typename stl_tp>
struct neg_stl_pred
{
  typedef stl_tp argument_type;
  typedef bool result_type;
  result_type operator() (const stl_tp& x) const
  {
    bool res = true;
    for( auto it = x.begin(); it != x.end(); it++ ) {
      res &= *it < 0;
    }
    return res;
  }
};

template <typename stl_tp>
struct eq_stl_pred
{
  typedef stl_tp argument_type;
  typedef bool result_type;
  result_type operator() (const stl_tp& x, const stl_tp& y) const
  {
    if( x.size() != y.size() ) {
      return false;
    }
    bool res = true;
    auto x_it = x.begin();
    auto y_it = y.begin();
    for( ; x_it != x.end(); x_it++, y_it++ ) {
      res &= *x_it == *y_it;
    }
    return res;
  }
};

template <typename stl_tp>
struct neq_stl_pred
{
  typedef stl_tp argument_type;
  typedef bool result_type;
  result_type operator() (const stl_tp& x, const stl_tp& y) const
  {
    if( x.size() != y.size() ) {
      return true;
    }
    bool res = true;
    auto x_it = x.begin();
    auto y_it = y.begin();
    for( ; x_it != x.end(); x_it++, y_it++ ) {
      res &= *x_it != *y_it;
    }
    return res;
  }
};

template <typename stl_tp>
struct less_stl_pred
{
  typedef stl_tp argument_type;
  typedef bool result_type;
  result_type operator() (const stl_tp& x, const stl_tp& y) const
  {
    if( x.size() < y.size() ) {
      return true;
    } else if( x.size() > y.size() ) {
      return false;
    }
    bool res = true;
    auto x_it = x.begin();
    auto y_it = y.begin();
    for( ; x_it != x.end(); x_it++, y_it++ ) {
      res &= *x_it < *y_it;
    }
    return res;
  }
};

template <typename stl_tp>
struct gt_stl_pred
{
  typedef stl_tp argument_type;
  typedef bool result_type;
  result_type operator() (const stl_tp& x, const stl_tp& y) const
  {
    if( x.size() > y.size() ) {
      return true;
    } else if( x.size() < y.size() ) {
      return false;
    }
    bool res = true;
    auto x_it = x.begin();
    auto y_it = y.begin();
    for( ; x_it != x.end(); x_it++, y_it++ ) {
      res &= *x_it > *y_it;
    }
    return res;
  }
};

template <typename stl_tp>
struct half_stl_pred
{
private:
  stl_tp m_mid;
public:
  typedef stl_tp argument_type;
  half_stl_pred(stl_tp mid) :
    m_mid(mid)
  { }
  typedef bool result_type;
  result_type operator() (const stl_tp& x) const
  {
    //return x < m_mid;
    return true;
  }
  void define_type(stapl::typer& t )
  {
    t.member(m_mid);
  }
};

template <typename stl_tp>
struct pivot_stl_pred
{
private:
  stl_tp m_piv;
public:
  typedef stl_tp argument_type;
  pivot_stl_pred(stl_tp piv) :
    m_piv(piv)
  { }
  typedef bool result_type;
  result_type operator() (const stl_tp& x) const
  {
    //return x < m_piv;
    return true;
  }
  void define_type(stapl::typer& t )
  {
    t.member(m_piv);
  }
};

template <typename stl_tp>
struct cut_stl_pred
{
private:
  stl_tp m_cut;
public:
  cut_stl_pred( stl_tp cut ) :
    m_cut(cut)
  { }
  typedef stl_tp argument_type;
  typedef bool result_type;
  result_type operator() (const stl_tp& x) const
  {
    //return x < m_cut;
    return true;
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_cut);
  }
};
