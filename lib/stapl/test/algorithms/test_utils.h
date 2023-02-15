/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef TEST_UTILS_H
#define TEST_UTILS_H

#include <iostream>
#include <string>
#ifdef USE_PROFILER
# include <typeinfo>
# include <stapl/runtime/system.hpp>
#endif
#include <stapl/runtime.hpp>

//#define SHORT_TEST  // define this before this #include for the quickest,
// most basic, test
// full p_transform builds taking 700+MB! of RAM+swap!...
//   30minute+ build on AMD64 3000 with 512MB (aborted)
//   4minute build on Dual-Athlon 2.8Ghz with 4GB RAM
// highly recommended you use "_debug" targets during development
// or #define SHORT_TEST before #include "test_utils.h"

#ifndef LONG_TEST      // if user, doesn't add USER_CXXFLAGS="-DLONG_TEST"
# define SHORT_TEST 1  // or #define LONG_TEST then default to array<int>
#endif

#if defined(SHORT_TEST)
# include <stapl/containers/array/array.hpp>
#elif defined(P_LIST_ONLY)
# include <p_list.h>
#else
# include <stapl/containers/array/array.hpp>
# include <p_vector.h>
# ifndef NO_P_LIST
#  include <p_list.h>
# endif
#endif

#ifdef USE_PROFILER  // define this to wrap stapl_main in the BaseProfiler
# include <profile/BaseProfiler.h>
#endif

#define DEFAULT_OUTPUT_STREAM std::cerr

#ifdef NO_POINT
# define Point char
#else
# include "Point.h"  // use our test Point class
#endif

// not the prettiest but functional and eliminates need for a
// seperate file defining stapl_main()
//
// This could be expanded to mix container types.
// I'd propose a 3-container template... something like:
//
//  stapl_test<stapl::p_list<int>, stapl::pArray<float>,
//   stapl::p_vector<double> >(argc, argv);
//
// with the corresponding:
//
//  template <typename PContainer1, PContainer2, PContainer3>
//   void stapl_test(int argc, char** argv) { /* stuff */ }
//
// that uses as many as they choose.  Three containers should cover
// a lot of possibilities.
#ifdef SHORT_TEST

#define STAPL_MAIN()                                \
stapl::exit_code stapl_main(int argc, char* argv[]) \
{                                                   \
  stapl_run<stapl::array<int> >(argc, argv);        \
  return EXIT_SUCCESS;                              \
}

#elif defined(P_LIST_ONLY)

#define STAPL_MAIN()                                \
stapl::exit_code stapl_main(int argc, char* argv[]) \
{                                                   \
  stapl_run<stapl::p_list<int> >(argc, argv);       \
  stapl_run<stapl::p_list<float> >(argc, argv);     \
  stapl_run<stapl::p_list<double> >(argc, argv);    \
  stapl_run<stapl::p_list<Point> >(argc, argv);     \
  return EXIT_SUCCESS;                              \
}

#else

# ifdef NO_P_LIST

#define STAPL_MAIN()                                \
stapl::exit_code stapl_main(int argc, char* argv[]) \
{                                                   \
  stapl_run<stapl::array<int> >(argc, argv);        \
  stapl_run<stapl::array<float> >(argc, argv);      \
  stapl_run<stapl::array<double> >(argc, argv);     \
  stapl_run<stapl::array<Point> >(argc, argv);      \
  stapl_run<stapl::p_vector<int> >(argc, argv);     \
  stapl_run<stapl::p_vector<float> >(argc, argv);   \
  stapl_run<stapl::p_vector<double> >(argc, argv);  \
  stapl_run<stapl::p_vector<Point> >(argc, argv);   \
  return EXIT_SUCCESS;                              \
}
# else

#define STAPL_MAIN()                                \
stapl::exit_code stapl_main(int argc, char* argv[]) \
{                                                   \
  stapl_run<stapl::array<int> >(argc, argv);        \
  stapl_run<stapl::array<float> >(argc, argv);      \
  stapl_run<stapl::array<double> >(argc, argv);     \
  stapl_run<stapl::array<Point> >(argc, argv);      \
  stapl_run<stapl::p_list<int> >(argc, argv);       \
  stapl_run<stapl::p_list<float> >(argc, argv);     \
  stapl_run<stapl::p_list<double> >(argc, argv);    \
  stapl_run<stapl::p_list<Point> >(argc, argv);     \
  stapl_run<stapl::p_vector<int> >(argc, argv);     \
  stapl_run<stapl::p_vector<float> >(argc, argv);   \
  stapl_run<stapl::p_vector<double> >(argc, argv);  \
  stapl_run<stapl::p_vector<Point> >(argc, argv);   \
  return EXIT_SUCCESS;                              \
}

# endif

#endif

template <typename Container> void stapl_test(int argc, char **argv);

namespace stapl {

class stapl_bool
{
private:
  bool       val;
  rmi_handle handle;

public:
  stapl_bool(bool v = false)
    : val(v),
      handle(this)
  { }

  bool value(void) const
  { return val; }

  bool reduce(void)
  {
    val = allreduce_rmi(std::logical_and<bool>(),
                        handle, &stapl_bool::value).get();
    return val;
  }

  stapl_bool& operator=(stapl_bool const& x)
  {
    val = x.val;
    return *this;
  }

  stapl_bool& operator=(bool x)
  {
    val = x;
    return *this;
  }

  stapl_bool& operator&=(stapl_bool const& x)
  {
    val &= x.val;
    return *this;
  }

  stapl_bool& operator&=(bool x)
  {
    val &= x;
    return *this;
  }
}; // class stapl_bool

#ifdef USE_PROFILER

template <typename Container>
class testprofiler : public BaseProfiler
{
  int initialize(int argc, char **argv) {
    BaseProfiler::initialize(argc, argv);

    // record algo and container
    Container for_type_string;
    std::string p_a_name =
      (strrchr(argv[0],'/') ? strrchr(argv[0],'/')+1 : argv[0]);

    std::string p_c_type(
      stapl::runtime::demangle(typeid(for_type_string).name()).c_str()
    );

    db.set_attribute("p_algorithm", p_a_name);
    db.set_attribute("p_container", p_c_type);

    // change filename prefix to include container
    std::string app_name  = p_a_name + "-" + p_c_type;
    if ( app_name.find("<") != app_name.npos) {
      app_name.replace(app_name.find("<"), 1, "_");
    }
    if ( app_name.find(">") != app_name.npos) {
      app_name.replace(app_name.find(">"), 1, "");
    }
    db.set_attribute("DB:applications_name", app_name);
    return 1;
  }
  int run(int argc, char **argv) {
    stapl_test<Container>(argc, argv);
    return 1;
  }
};

#endif

// dump a view to an output stream
template <typename View>
void dump_view(View const& v, std::string const& title = "View",
               std::ostream& o_str = DEFAULT_OUTPUT_STREAM)
{
  if (stapl::get_location_id() == 0) {
    o_str << title;
    const typename View::const_iterator v_begin = v.begin();
    const typename View::const_iterator v_end = v.end();
    for (typename View::const_iterator i = v_begin; i != v_end; i++) {
      o_str << *i << " ";
    }
    o_str << std::endl;
  }
}

// dump single source paragraph data to an output stream
template <typename Paragraph>
void dump_paragraph(Paragraph& pr, std::string const& title = "PARAGRAPH",
                    std::ostream& o_str = DEFAULT_OUTPUT_STREAM)
{
  dump_view(pr.get_view0(), title, o_str);
}

template <typename Paragraph>
inline void dump_paragraph(Paragraph& pr, char* title,
                           std::ostream const& o_str = DEFAULT_OUTPUT_STREAM)
{
  std::string stitle(title);
  dump_paragraph(pr, stitle, o_str);
}

// dump pContainer data to an output stream
// NOTE: since this calls create_view() it must be done on all locations
//       it is probably just better to avoid using it
template <typename PContainer>
void dump_p_container(std::string const& title, PContainer& pc,
                      std::ostream& o_str = DEFAULT_OUTPUT_STREAM)
{
  typename PContainer::view_type pc_view = pc.create_view();
  dump_view<PContainer>(pc_view, title, o_str);
}

template <typename PContainer>
inline void dump_p_container(char* title, PContainer const& pc,
                             std::ostream& o_str = DEFAULT_OUTPUT_STREAM)
{
  std::string stitle(title);
  dump_p_container(stitle, pc, o_str);
}

// verify contents of two containers are the same
// this is primarily available for checking STAPL p_algo
// implementations against the sequential STL algorithms.
// See p_transform() for an example of its use.
template <typename PView, typename View>
void verify_results(PView& p_v, View& v, std::string const& title,
                    std::ostream& o_str = DEFAULT_OUTPUT_STREAM)
{
  std::pair<typename PView::iterator, typename View::const_iterator>
    result = std::mismatch( p_v.begin(), p_v.end(), v.begin());
  stapl::rmi_fence(); // make sure all locations finish
  if ( result.first == p_v.end()) {
    if ( stapl::get_location_id() == 0) {
      o_str << "PASS: " << title << std::endl;
    }
  } else {
    if ( stapl::get_location_id() == 0) {
      o_str << "ERROR: " << title
                << " - Difference at position "
                << std::distance(p_v.begin(),result.first) << " -- "
                << *(result.first) << " != " << *(result.second) << std::endl;
    }
    dump_view(p_v, "  p_version: ", o_str);
    dump_view(v,  "STL version: ", o_str);
  }
  stapl::rmi_fence(); // don't proceed while outputting
}

// verify the return values of two methods and dump the containers
// if they don't match
template <typename Paragraph, typename PContainer, typename T>
bool verify_results(Paragraph& pr, PContainer& pc, T p_result, T std_result,
                    std::string const& title,
                    std::ostream& o_str = DEFAULT_OUTPUT_STREAM)
{
  bool success = true;
  typename PContainer::view_type pc_view = pc.create_view();
  if (std_result != p_result) {
    if (stapl::get_location_id() == 0) {
      o_str << "ERROR: RESULTS DON'T MATCH - " << title
                << " - stapl::p_version()=" << p_result
                << " != std::version()=" << std_result
                << std::endl;
    }
    success = false;
    dump_paragraph(pr, "pr: ", o_str);
    dump_view(pc_view, "pc: ", o_str);
  }
  return success;
}

template <typename Paragraph, typename PContainer, typename T>
inline bool verify_results(Paragraph& pr, PContainer& pc, T p_result,
                           T std_result, const char *title)
{
  std::string stitle(title);
  return verify_results(pr, pc, p_result, std_result, title);
}

} // namespace stapl

#ifdef USE_PROFILER
template <typename Container>
void stapl_run(int argc, char **argv)
{
  stapl::testprofiler<Container> tp;
  if (tp.collectProfile(argc, argv)) {
    tp.outputProfile();
  }
}
#else
template <typename Container>
void stapl_run(int argc, char **argv)
{
  stapl_test<Container>(argc, argv);
}
#endif

#endif
