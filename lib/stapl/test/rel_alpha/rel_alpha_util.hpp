/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <iostream>
#include <stapl/algorithms/functional.hpp>
using namespace std;

typedef stapl::identity<int>    id_int_wf;
typedef stapl::identity<size_t> id_un_wf;
typedef stapl::negate<int>      neg_int_wf;

typedef stapl::plus<int>        add_int_wf;
typedef stapl::minus<int>       sub_int_wf;
typedef stapl::multiplies<int>  mul_int_wf;
typedef stapl::min<int>         min_int_wf;
typedef stapl::max<int>         max_int_wf;

typedef stapl::bit_xor<size_t>  xor_un_wf;
typedef stapl::bit_or<size_t>   ior_un_wf;
typedef stapl::bit_and<size_t>  and_un_wf;

#if 0
template<typename T>
struct abs_val
  : public ro_unary_function<T, T>
{
  template<typename Ref>
  T operator()(Ref x)
  { return ( x < 0 ) ? (-x) : (x); }
};
typedef abs_val<int> abs_int_wf;
#else
struct abs_int_wf
{
  typedef int result_type;
  template<typename Ref>
  result_type operator()(Ref x)
  { return ( x < 0 ) ? (-x) : (x); }
};
#endif


/*-------------------------------------------------------------------------*/

struct msg
{
private:
  string m_txt;
public:
  typedef void result_type;
  msg(string text)
    : m_txt(text)
  { }
  void operator() () {
    cout << m_txt << endl;
  }
};

template<typename Value>
struct msg_val
{
private:
  string m_txt;
  Value m_val;
public:
  typedef void result_type;
  msg_val(string text, Value val)
    : m_txt(text), m_val(val)
  { }
  void operator() () {
    cout << m_txt << " " << m_val << endl;
  }
};

/*-------------------------------------------------------------------------*/

struct put_val_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  put_val_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename Ref>
  result_type operator()(Ref val)
  {
    m_zout << val << " ";
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

struct put_ndx_val_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  put_ndx_val_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename Ref1, typename Ref2>
  result_type operator()(Ref1 pos, Ref2 val)
  {
    m_zout << "[" << pos << "]= " << val << endl;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

struct get_val_wf
{
private:
  stapl::stream<ifstream> m_zin;
public:
  get_val_wf(stapl::stream<ifstream> const& zin)
    : m_zin(zin)
  { }

  typedef void result_type;
  template <typename Ref>
  result_type operator()(Ref val)
  {
    m_zin >> val;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zin);
  }
};

struct put_map_val_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  put_map_val_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename Ref>
  result_type operator()(Ref pair)
  {
    m_zout << "{" << pair.first << "}= " << pair.second << endl;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

struct put_graph_val_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  put_graph_val_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename Vertex>
  result_type operator()(Vertex v)
  {
    m_zout << "{" << v.property() << "} " << endl;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

struct get_pair_wf
{
private:
  stapl::stream<ifstream> m_zin;
public:
  get_pair_wf(stapl::stream<ifstream> const& zin)
    : m_zin(zin)
  { }

  typedef void result_type;
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

struct get_triple_wf
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

struct show_map_indexable_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  show_map_indexable_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename Element>
  result_type operator()(Element elem)
  {
    typename Element::first_reference first = elem.first;
    typename Element::second_reference second = elem.second;
    m_zout << "{" << first << "} #" << second.size() << endl;
    stapl::serial_io(put_val_wf(m_zout), second);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

struct show_indexable_map_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  show_indexable_map_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename Element>
  result_type operator()(Element elem)
  {
    stapl::serial_io(put_map_val_wf(m_zout), elem);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

struct show_inner_map_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  show_inner_map_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename Elem>
  result_type operator()(Elem elem)
  {
    m_zout << "  {" << elem.first << "}= " << elem.second << endl;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

struct show_outer_map_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  show_outer_map_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename Elem>
  result_type operator()(Elem elem)
  {
    m_zout << "{" << elem.first << "} #" << elem.second.size() << endl;
    stapl::serial_io(show_inner_map_wf(m_zout), elem.second);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};

#if 0
struct print_graph_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  print_graph_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

#if 0
  struct print_vertex_wf
  {
  private:
    stapl::stream<ofstream> m_zout;
  public:
    typedef void result_type;

    print_vertex_wf(stapl::stream<ofstream> const& zout)
      : m_zout(zout)
    { }

    typedef void result_type;
    template <typename Vertex>
    result_type operator()(Vertex v)
    {
#if 0
      m_zout << "> " << v.descriptor() << " / " << v.size() << "  {";
      for (auto it = v.begin(); it != v.end(); ++it)
        m_zout << "  " << (*it).target() << ", ";
      m_zout << " }" << std::endl;
#endif
    }
    void define_type(stapl::typer& t)
    {
      t.member(m_zout);
    }
  };
#endif

  template <typename View1>
  result_type operator()(View1 vw1)
  {
    m_zout << "element: " << vw1.size() << std::endl;
    //stapl::map_func(print_vertex_wf(m_zout), vw1);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_zout);
  }
};
#endif

struct show_log_ndx_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  show_log_ndx_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
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
  show_log_map_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
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
      m_zout << "[" << val.first << "]= " << val.second << endl;
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

struct show_log_triple_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  show_log_triple_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename Tuple, typename Pos>
  result_type operator()(Tuple val, Pos pos)
  {
    tuple<int,int,int> tup = val;
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

      m_zout << "[" << pos << "]= ";
      m_zout << "( " << get<0>(tup);
      m_zout << ", " << get<1>(tup);
      m_zout << ", " << get<2>(tup);
      m_zout << ")" << endl;;
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

struct show_log_graf_wf
{
  stapl::stream<ofstream> m_zout;
public:
  show_log_graf_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename Vertex, typename Ref>
  result_type operator()(Vertex vtx, Ref pos)
  {
    typename Vertex::adj_edge_iterator it;
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
      m_zout << ">  " << vtx.descriptor() << " / " << vtx.size() << " {";
      for ( it= vtx.begin(); it != vtx.end(); ++it) {
        m_zout << " " << (*it).target() << ", ";
      }
      m_zout << " }" << endl;
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

void show_time(const char *name, const char *version, double time,
  double io=0.0)
{
  if( 0 == strcmp("STAPL", version) ) {
    if( stapl::get_location_id() == 0 ) {
      cerr << "Test : " << name << endl;
      cerr << "Version : " << version << endl;
      cerr << "Time : " << time << endl;
      if( io != 0.0 ) {
        cerr << "Note : IO time= " << io << endl;
      }
    }
  }
}

/*-------------------------------------------------------------------------*/

struct inner_seq_elem_wf
{
  typedef int result_type;
  template<typename Element>
  result_type operator()(Element elem) const
  {
    return elem;
  }
};

struct inner_map_elem_wf
{
  typedef int result_type;
  template<typename Element>
  result_type operator()(Element elem) const
  {
    typename Element::second_reference map_arg = elem.second;
    return map_arg;
  }
};

struct inner_graph_elem_wf
{
  typedef int result_type;
  template<typename Vertex>
  result_type operator()(Vertex v) const
  {
    return v.property();
  }
};

struct l1_map_l0_map_cksum_wf
{
  typedef int result_type;
  template<typename Element>
  result_type operator()(Element elem) const
  {
    return stapl::map_reduce(inner_map_elem_wf(), xor_un_wf(), elem.second);
  }
};

struct l1_graph_l0_graph_cksum_wf
{
  typedef int result_type;
  template<typename Vertex>
  result_type operator()(Vertex v) const
  {
    return stapl::map_reduce(inner_graph_elem_wf(), xor_un_wf(), v.property());
  }
};

struct l1_map_l0_seq_cksum_wf
{
  typedef int result_type;
  template<typename Element>
  result_type operator()(Element elem) const
  {
    return stapl::map_reduce(inner_seq_elem_wf(), xor_un_wf(), elem.second);
  }
};

struct l1_graph_l0_seq_cksum_wf
{
  typedef int result_type;
  template<typename Vertex>
  result_type operator()(Vertex v) const
  {
    return stapl::map_reduce(inner_seq_elem_wf(), xor_un_wf(), v.property());
  }
};

struct l1_seq_l0_map_cksum_wf
{
  typedef int result_type;
  template<typename Element>
  result_type operator()(Element elem) const
  {
    return stapl::map_reduce(inner_map_elem_wf(), xor_un_wf(), elem);
  }
};

struct l1_seq_l0_graph_cksum_wf
{
  typedef int result_type;
  template<typename Element>
  result_type operator()(Element elem) const
  {
    return stapl::map_reduce(inner_graph_elem_wf(), xor_un_wf(), elem);
  }
};

struct l1_seq_l0_seq_cksum_wf
{
  typedef int result_type;
  template<typename Element>
  result_type operator()(Element elem) const
  {
    return stapl::map_reduce(inner_seq_elem_wf(), xor_un_wf(), elem);
  }
};

struct l2_seq_l1_seq_l0_seq_cksum_wf
{
  typedef int result_type;
  template<typename Element>
  result_type operator()(Element elem) const
  {
    return stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(), elem);
  }
};

struct l2_seq_l1_seq_l0_map_cksum_wf
{
  typedef int result_type;
  template<typename Element>
  result_type operator()(Element elem) const
  {
    return stapl::map_reduce(l1_seq_l0_map_cksum_wf(), xor_un_wf(), elem);
  }
};

struct l2_seq_l1_seq_l0_graph_cksum_wf
{
  typedef int result_type;
  template<typename View>
  result_type operator()(View vw) const
  {
    return stapl::map_reduce(l1_seq_l0_graph_cksum_wf(), xor_un_wf(), vw);
  }
};

struct l2_seq_l1_map_l0_seq_cksum_wf
{
  typedef int result_type;
  template<typename Element>
  result_type operator()(Element elem) const
  {
    return stapl::map_reduce(l1_map_l0_seq_cksum_wf(), xor_un_wf(), elem);
  }
};

struct l2_seq_l1_graph_l0_seq_cksum_wf
{
  typedef int result_type;
  template<typename Element>
  result_type operator()(Element e) const
  {
    return stapl::map_reduce(l1_graph_l0_seq_cksum_wf(), xor_un_wf(), e);

  }
};

struct l2_seq_l1_map_l0_map_cksum_wf
{
  typedef int result_type;
  template<typename Element>
  result_type operator()(Element elem) const
  {
    return stapl::map_reduce(l1_map_l0_map_cksum_wf(), xor_un_wf(), elem);
  }
};

struct l2_seq_l1_graph_l0_graph_cksum_wf
{
  typedef int result_type;
  template<typename Element>
  result_type operator()(Element e) const
  {
    return stapl::map_reduce(l1_graph_l0_graph_cksum_wf(), xor_un_wf(), e);
  }
};

struct l2_map_l1_seq_l0_seq_cksum_wf
{
  typedef int result_type;
  template<typename Element>
  result_type operator()(Element elem) const
  {
    return stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(),
      elem.second);
  }
};

struct l2_graph_l1_seq_l0_seq_cksum_wf
{
  typedef int result_type;
  template<typename Vertex>
  result_type operator()(Vertex v) const
  {
    return stapl::map_reduce(l1_seq_l0_seq_cksum_wf(), xor_un_wf(),
      v.property());
  }
};

struct l2_map_l1_seq_l0_map_cksum_wf
{
  typedef int result_type;
  template<typename Element>
  result_type operator()(Element elem) const
  {
    return stapl::map_reduce(l1_seq_l0_map_cksum_wf(), xor_un_wf(),
      elem.second);
  }
};

struct l2_graph_l1_seq_l0_graph_cksum_wf
{
  typedef int result_type;
  template<typename Vertex>
  result_type operator()(Vertex v) const
  {
    return stapl::map_reduce(l1_seq_l0_graph_cksum_wf(), xor_un_wf(),
      v.property());
  }
};

struct l2_map_l1_map_l0_seq_cksum_wf
{
  typedef int result_type;
  template<typename Element>
  result_type operator()(Element elem) const
  {
    return stapl::map_reduce(l1_map_l0_seq_cksum_wf(), xor_un_wf(),
      elem.second);
  }
};

struct l2_map_l1_map_l0_map_cksum_wf
{
  typedef int result_type;
  template<typename Element>
  result_type operator()(Element elem) const
  {
    return stapl::map_reduce(l1_map_l0_map_cksum_wf(), xor_un_wf(),
      elem.second);
  }
};

//-----

struct outer_graph_cksum_wf
{
  typedef int result_type;
  template<typename Element>
  result_type operator()(Element elem) const
  {
    return stapl::map_reduce(inner_graph_elem_wf(), xor_un_wf(),
                             elem.property());
  }
};

struct outer_graph_cont_cksum_wf
{
  typedef int result_type;
  template<typename Element>
  result_type operator()(Element elem) const
  {
    return stapl::map_reduce(inner_seq_elem_wf(), xor_un_wf(),
                             elem.property());
  }
};

/*-------------------------------------------------------------------------*/

struct roll_wf
{
  typedef void result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 length, View2 val) const
  {
    length = 1 + (rand() % val);
  }
};

struct inner_roll_wf
{
  typedef void result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 length, View2 val) const
  {
    length = 1 + (rand() % val);
  }
};

struct outer_roll_wf
{
  typedef void result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 v1, View2 v2) const
  {
    stapl::map_func(inner_roll_wf(), v1, stapl::make_repeat_view(v2));
  }
};

struct tuple2_rand_init_wf
{
  typedef void result_type;
  template<typename TupleRef, typename Elem>
  result_type operator()(TupleRef ref, Elem val) const
  {
    const size_t x = 1 + (rand() % val);
    const size_t y = 1 + (rand() % val);
    ref = stapl::make_tuple(x,y);
  }
};

struct tuple3_rand_init_wf
{
  typedef void result_type;
  template<typename TupleRef, typename Elem>
  result_type operator()(TupleRef ref, Elem val) const
  {
    const size_t x = 1 + (rand() % val);
    const size_t y = 1 + (rand() % val);
    const size_t z = 1 + (rand() % val);
    ref = stapl::make_tuple(x,y,z);
  }
};

