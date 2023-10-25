/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


//////////////////////////////////////////////////////////////////////
/// @file
/// Benchmark that compares STAPL serialization against @c memcpy and
/// Boost.Serialization.
//////////////////////////////////////////////////////////////////////

#define BENCHMARK "# %s Latency Benchmark\n"

#include <stapl/runtime.hpp>
#include <cstdio>
#include <iostream>
#include <string>
#include <unistd.h>
#include <boost/optional.hpp>
#include <boost/program_options.hpp>
#include <boost/utility/in_place_factory.hpp>
#include "common.hpp"

#define MAX_SIZE (1<<22)

#ifndef FIELD_WIDTH
# define FIELD_WIDTH 20
#endif

#ifndef FLOAT_PRECISION
# define FLOAT_PRECISION 2
#endif


using namespace stapl;


// Prints the achieved latency
void print(int size, int loop, double t)
{
  const double latency = (t * 1.0e6 / loop);
  std::fprintf(stdout, "%-*d%*.*f\n",
               10, size, FIELD_WIDTH, FLOAT_PRECISION, latency);
  std::fflush(stdout);
}

enum class serialization_type
{
  MEMCPY,
  STAPL_TYPER,
  BOOST_SERIALIZATION
};

//////////////////////////////////////////////////////////////////////
/// @brief Returns the serialization type as a <tt>const char*</tt>.
//////////////////////////////////////////////////////////////////////
inline const char* get_serialization_name(serialization_type s)
{
  switch (s) {
    case serialization_type::MEMCPY:
      return "memcpy()";
    case serialization_type::STAPL_TYPER:
      return "stapl::type()";
    case serialization_type::BOOST_SERIALIZATION:
      return "boost::serialization()";
    default:
      error("%s(): Incorrect primitive.\n", __func__);
      return "";
  }
}


//////////////////////////////////////////////////////////////////////
/// @brief @c std::vector like class with STAPL serialization support.
//////////////////////////////////////////////////////////////////////
template<typename T>
class stapl_vector
{
private:
  typedef std::vector<T>                  vector_type;
public:
  typedef typename vector_type::size_type size_type;

private:
  vector_type m_data;

public:
  stapl_vector(size_type n, T const& t = T())
  : m_data(n, t)
  { }

  void define_type(typer& t)
  { t.member(m_data); }
};


//////////////////////////////////////////////////////////////////////
/// @brief @c std::vector like class with STAPL serialization support.
//////////////////////////////////////////////////////////////////////
template<typename T>
class boost_vector
{
private:
  typedef std::vector<T>                  vector_type;
public:
  typedef typename vector_type::size_type size_type;

private:
  vector_type m_data;

  friend class boost::serialization::access;

  template<typename Archive>
  void serialize(Archive& ar, const unsigned int)
  { ar & m_data; }

public:
  boost_vector(void) = default;

  boost_vector(size_type n, T const& t = T())
  : m_data(n, t)
  { }
};


exit_code stapl_main(int argc, char *argv[])
{
  int skip = 1000;
  int loop = 10000;

  // options
  serialization_type sn = serialization_type::STAPL_TYPER;

  // parse options
  try {
    namespace po = boost::program_options;

    po::options_description desc("Allowed options");
    desc.add_options()
      ("help",
       "Print this help message")
      ("stapl",
       "Use STAPL serialization (default)")
      ("memcpy",
       "Use memcpy serialization")
      ("boost",
       "Use Boost.Serialization");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
      if (get_location_id()==0)
        std::cout << desc << std::endl;
      return EXIT_SUCCESS;
    }

    if (vm.count("stapl"))
      sn = serialization_type::STAPL_TYPER;

    if (vm.count("memcpy"))
      sn = serialization_type::MEMCPY;

    if (vm.count("boost"))
      sn = serialization_type::BOOST_SERIALIZATION;
  }
  catch (boost::program_options::unknown_option& ex) {
    error("%s: %s.\n", argv[0], ex.what());
  }

  if (get_location_id() == 0) {
    std::fprintf(stdout, BENCHMARK, get_serialization_name(sn));
    std::fflush(stdout);
  }

  // latency benchmark
  for (int size = 0; size <= MAX_SIZE; size = (size ? size * 2 : size + 1)) {
    counter<default_timer> counter;
    double t = 0.0;
    switch (sn) {
      case serialization_type::MEMCPY: {
        char* const target_buf = new char[size];
        char* const buf        = new char[size];
        std::memset(buf, 'm', size * sizeof(*buf));

        for (int i = 0; i < skip + loop; i++) {
          if (i == skip)
            counter.start();
          std::memcpy(target_buf, buf, size * sizeof(*buf));
        }
        t = counter.stop();
        delete[] buf;
        delete[] target_buf;
      } break;
      case serialization_type::STAPL_TYPER: {
        typedef stapl_vector<char>                                vec_type;
        typedef runtime::arg_storage_t<vec_type, vec_type const&> arg_type;

        const vec_type v(size, 's');
        const std::size_t static_size = sizeof(arg_type);
        const std::size_t s           = static_size + arg_type::packed_size(v);
        char* const target_buf        = new char[s];

        for (int i = 0; i < skip + loop; i++) {
          if (i == skip)
            counter.start();
          std::size_t sz = static_size;
          arg_type* a = new(target_buf) arg_type(v, target_buf, sz);
          a->get(target_buf);
          a->~arg_type();
        }
        t = counter.stop();
        delete[] target_buf;
      } break;
      case serialization_type::BOOST_SERIALIZATION: {
        typedef boost_vector<char>                                vec_type;
        typedef runtime::arg_storage_t<vec_type, vec_type const&> arg_type;

        const vec_type v(size, 'b');
        const std::size_t static_size = sizeof(arg_type);
        const std::size_t s           = static_size + arg_type::packed_size(v);
        char* const target_buf        = new char[s];

        for (int i = 0; i < skip + loop; i++) {
          if (i == skip)
            counter.start();
          std::size_t sz = static_size;
          arg_type* a = new(target_buf) arg_type(v, target_buf, sz);
          a->get(target_buf);
          a->~arg_type();
        }
        t = counter.stop();
        delete[] target_buf;
      } break;
      default:
        error("%s: Incorrect serialization.\n", argv[0]);
        break;
    }

    print(size, loop, t);
  }

  return EXIT_SUCCESS;
}
