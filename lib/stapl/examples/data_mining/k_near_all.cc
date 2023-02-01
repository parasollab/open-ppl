/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/stream.hpp>
#include <stapl/runtime.hpp>
#include <stapl/algorithm.hpp>
#include <stapl/skeletons/serial.hpp>
#include <stapl/utility/do_once.hpp>
#include <stapl/domains/indexed.hpp>
#include <stapl/map.hpp>
#include <stapl/vector.hpp>
#include <stapl/views/counting_view.hpp>
#include <stapl/runtime/counter/default_timer.hpp>
#include <stapl/runtime/counter/counter.hpp>
#include <stapl/views/repeated_view.hpp>

using namespace std;

struct point
{
  double x, y, z;

  point()
   : x(0.), y(0.), z(0.)
  { }

  point(double xx, double yy, double zz)
   : x(xx), y(yy), z(zz)
  { }

  void plus(point const& other)
  {
    x += other.x;
    y += other.y;
    z += other.z;
  }

  bool operator==(point const& other)
  {
    return std::abs(x-other.x) < FLT_EPSILON &&
           std::abs(y-other.y) < FLT_EPSILON &&
           std::abs(z-other.z) < FLT_EPSILON;
  }

  void define_type(stapl::typer& t)
  {
    t.member(x);
    t.member(y);
    t.member(z);
  }
};

namespace stapl{
STAPL_PROXY_HEADER(point)
{
  STAPL_PROXY_TYPES(point, Accessor)
  STAPL_PROXY_METHODS(point, Accessor)

  STAPL_PROXY_MEMBER(x, double)
  STAPL_PROXY_MEMBER(y, double)
  STAPL_PROXY_MEMBER(z, double)

  explicit proxy(Accessor const& acc)
   : Accessor(acc),
     x(member_referencer<x_accessor>()(acc)),
     y(member_referencer<y_accessor>()(acc)),
     z(member_referencer<z_accessor>()(acc))
  { }
};
}

typedef stapl::vector<point> vec_type;
typedef stapl::vector_view<vec_type> vec_view_type;
typedef std::multimap<double, point> mp_type;
typedef stapl::vector<std::multimap<double, point>> vec_mp_type;
typedef stapl::vector_view<vec_mp_type> vec_mp_view;

struct insert_wf
{
  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 v1)
  {
    stapl::random_sequence r;
    v1=point(r(), r(), r());
  }
};

//read from multimap into another multimap with distance as key
template <typename K>
struct reduce_wf
{
  private:
    K k;
  public:
    typedef mp_type result_type;
    reduce_wf(K pt)
      :k(pt)
    {}

    template <typename Value1, typename Value2>
    result_type operator()(Value1 v1, Value2 v2)
    {
      mp_type temp_v1=v1;
      mp_type temp_v2=v2;
      mp_type r1;
      mp_type::iterator iter1;
      int count=0;
      mp_type results;
      mp_type::iterator x =temp_v1.begin();
      mp_type::iterator y =temp_v2.begin();

      //iterate over the inputs, inserting the closer point into the multimap
      while(x!=temp_v1.end() && y!=temp_v2.end() && count!=k)
      {
        if(x->first < y->first)
        {
          results.insert(*x);
          ++x;
          ++count;
        }
        else if(y->first < x->first)
        {
          results.insert(*y);
          ++y;
          ++count;
        }
        else
        {
          results.insert(*x);
          results.insert(*y);
          ++x;
          ++y;
          count+=2;
        }
      }

      /*if the end of one of the maps is reached before k
      elements have been acquired, insert remaining needed elements
      from the other map*/
      while(x==temp_v1.end() && y!=temp_v2.end() && count<k)
      {
        results.insert(*y);
        ++y;
        ++count;
      }
      while(y==temp_v2.end() && x!=temp_v1.end() && count<k)
      {
        results.insert(*x);
        ++x;
        ++count;
      }

      return results;
    }
    void define_type(stapl::typer& t)
    {
      t.member(k);
    }
};

struct find_near_wf
{
  public:
   typedef mp_type result_type;
   template <typename Value1, typename Value2>

  result_type operator()(Value1 val1/*comparing pt*/, Value2 m_target)
   {
     mp_type pts;
     if(val1==m_target)
     {
        return pts;
     }
     pts.insert(std::make_pair(sqrt((pow(m_target.x-
      static_cast<double>(val1.x),2))+(pow(m_target.y-
      static_cast<double>(val1.y),2))+(pow(m_target.z-
      static_cast<double>(val1.z),2))),static_cast<point>(val1)));
     return pts;
   }
};

struct k_near_wf
{
  private:
   int pts;
  public:
    k_near_wf(int near)
      :pts(near)
    {}
    typedef void result_type;
    template <typename Value1, typename Value2, typename Value3>
    void operator()(Value1 val1/*comparing pt*/, Value2 val2, Value3 val3)
    {
     point temp=val1;
     val3=stapl::map_reduce(find_near_wf(),reduce_wf<int>(pts), val2,
      stapl::make_repeat_view(temp));
    }
    void define_type(stapl::typer& t)
    {
     t.member(pts);
    }
};

//defining output operator for std::pair
template <typename T1, typename T2>
std::ostream& operator<<(std::ostream& os, std::pair<T1, T2> const& pair)
{
  os<<pair.first<<", "<<pair.second<<'\n';
  return os;
}

//defining output operator for point
std::ostream& operator<<(std::ostream& os, point const& p)
{
  os<<p.x<<", "<<p.y<<", "<<p.z<<'\n';
  return os;
}

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

struct output_wf
{
  typedef double result_type;
  template <typename Type, typename Index>
  double operator()(Type temp, Index i)
  {
    mp_type t=temp;
    mp_type::iterator iter;
    double max=0;
    int num_pts=0;

    for (iter =t.begin(); iter!=t.end(); ++iter)
    {
      if((*iter).first > max)
      {
        max=(*iter).first;
      }
    }
    if(i % 100 == 99)
    {
      num_pts=t.size();
      msg_val<int>( "Number of Points: ", num_pts )();
    }

    return max;
  }
};

stapl::exit_code stapl_main(int argc, char **argv)
{
  //ask for total points and desired nearest points
  int total_points=boost::lexical_cast<int>(argv[1]);
  int near_points=boost::lexical_cast<int>(argv[2]);

  vec_type pts_vec_1(total_points);
  vec_view_type pts_view_1(pts_vec_1);

  vec_mp_type results(total_points);
  vec_mp_view result_view(results);

  stapl::map_func(insert_wf(), pts_view_1);

  map_func(k_near_wf(near_points), pts_view_1,
  stapl::make_repeat_view(pts_view_1), result_view);

  double max_distance = stapl::map_reduce(output_wf(),
  stapl::max<double>(), result_view,
  stapl::counting_view<int>(results.size()));

  stapl::do_once( msg_val <double>( "Max Distance: ", max_distance ) );

  return EXIT_SUCCESS;
}
