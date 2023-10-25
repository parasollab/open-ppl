/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <fstream>
#include <stapl/views/repeated_view.hpp>
#include <stapl/stream.hpp>
#include <stapl/runtime.hpp>
#include <stapl/algorithm.hpp>
#include <stapl/skeletons/serial.hpp>
#include <stapl/utility/do_once.hpp>
#include <stapl/vector.hpp>

using namespace std;

typedef stapl::vector<double> vec_type;
typedef stapl::vector_view<vec_type> vec_view_type;

template<typename Value>
struct msg_val
{
  private:
  const char* m_txt;
  Value m_val;
  public:
    msg_val(const char* text,Value val)
      : m_txt(text),m_val(val)
    { }

  typedef void result_type;
  result_type operator()()
  {
    cout << m_txt<< " " <<m_val << endl;
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_txt);
  }
};

struct get_val_wf
{
  private:
    stapl::stream<ifstream> m_zin;
  public:
    get_val_wf(stapl::stream<ifstream> const& zin)
      :m_zin(zin)
    {}

    typedef void result_type;
    template <typename Ref>
    result_type operator()(Ref val)
    {
      m_zin>>val;
    }

    void define_type(stapl::typer& t)
    {
      t.member(m_zin);
    }
};

struct exp_wf
{
  typedef double result_type;
  template <typename View, typename Mean>
  result_type operator()(View v, Mean m)
  {
    return pow(v-m,2);
  }
};

stapl::exit_code stapl_main(int argc, char **argv)
{
  vec_type vec(boost::lexical_cast<long int>(argv[1]));
  vec_view_type vec_view(vec);
  stapl::stream<ifstream> zin;
  zin.open(argv[2]);
  stapl::serial_io(get_val_wf(zin),vec_view);
  stapl::generate( vec_view, stapl::sequence<double>(0,1));
  double n = vec.size();
  double mean = stapl::accumulate(vec_view,0) / n;
  double variance=stapl::map_reduce(exp_wf(), stapl::plus<double>(),
    vec_view, stapl::make_repeat_view(mean));
  variance /= n-1;
  double s_dev = sqrt(variance);
  stapl::do_once( msg_val<double>( "Mean:", mean ) );
  stapl::do_once( msg_val<double>( "Variance:", variance ) );
  stapl::do_once( msg_val<double>( "Standard deviation:", s_dev ) );
  return EXIT_SUCCESS;
}

