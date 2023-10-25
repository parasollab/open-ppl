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
  template <typename View>
  result_type operator () (View v)
  {
    return pow(v,2);
  }
};

stapl::exit_code stapl_main(int argc, char **argv)
{
  double n = boost::lexical_cast<double>(argv[1]);
  double result = 0;
  boost::lexical_cast<long int>(argv[1]);

  vec_type vec_x(boost::lexical_cast<long int>(argv[1]));
  vec_view_type vec_x_view(vec_x);
  vec_type vec_y(boost::lexical_cast<long int>(argv[1]));
  vec_view_type vec_y_view(vec_y);

  stapl::stream<ifstream> zin;
  zin.open(argv[2]);

  stapl::serial_io(get_val_wf(zin),vec_x_view);
  stapl::serial_io(get_val_wf(zin),vec_y_view);

  result=n*stapl::inner_product(vec_x_view,vec_y_view,result);

  double temp_x=stapl::accumulate(vec_x_view,0);
  double temp_y=stapl::accumulate(vec_y_view,0);

  result=result-(temp_x*temp_y);
  double temp_x_sq=stapl::map_reduce(exp_wf(),stapl::plus<double>(),vec_x_view);
  double temp_y_sq=stapl::map_reduce(exp_wf(),stapl::plus<double>(),vec_y_view);

  double temp_res=sqrt(n*temp_x_sq-pow(temp_x,2));
  temp_res*=sqrt(n*temp_y_sq-pow(temp_y,2));

  result/=temp_res;
  stapl::do_once( msg_val<double>( "Correlation Coefficient:", result ) );
  return EXIT_SUCCESS;
}
