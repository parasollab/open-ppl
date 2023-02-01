/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <string>
#include <fstream>
#include <stdexcept>
#include <limits>
#include <cmath>
#include <iomanip>
#include <stapl/array.hpp>
#include <stapl/views/counting_view.hpp>
#include <stapl/algorithm.hpp>
#include "../utilities/confint.hpp"

using floating = float; //define precision of the floating point type
floating constexpr SQRT2 = 1.4142135623730950488;

typedef stapl::counter<stapl::default_timer> counter_t;

enum option {PUT, CALL};

struct asset
{
  floating s;  //spot price of asset
  floating k;  //strike price of asset
  floating r;  //risk free rate
  floating v;  //volatility of returns of asset
  floating t;  //time to maturity of asset (1 = 1 year, 0.25 = 3 months, ...)
  option type; //whether to calculate put or call

  asset(floating S, floating K, floating R, floating V,
        floating T, option TYPE)
    : s(S), k(K), r(R), v(V), t(T), type(TYPE)
  {}

  //not recommened, but allowed
  asset()
    : s(0), k(0), r(0), v(0), t(0), type(PUT)
  {}

  //needed for STAPL to be able to move instances of this
  //struct to different locations
  void define_type(stapl::typer& t2)
  {
   t2.member(s);
   t2.member(k);
   t2.member(r);
   t2.member(v);
   t2.member(t);
   t2.member(type);
  }
};


class parse_assets
{
  std::string filename;
  unsigned int total_num_locations;

  void skip_lines (std::istream& stream, size_t to_skip)
  {
    for(; to_skip > 0; --to_skip)
    {
      stream.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      if(!stream.good())
      {
        stapl::abort("There was a problem skipping to appropriate line!");
      }
    }
  }

  asset generate_asset(std::ifstream& file)
  {
    floating s;
    floating k;
    floating r;
    floating v;
    floating t;
    std::string temp; //stores temporary data

    option type;
    file >> s;
    file >> k;
    file >> r;

    //here we recieve dividend rate, which is unused
    file >> temp;
    temp.clear();
    file >> v;
    file >> t;
    file >> temp;
    if(temp == "C")
    {
      type = CALL;
    }
    else if(temp == "P")
    {
      type = PUT;
    }
    else
    {
      stapl::abort("Failed to read Put or Call!");
      type = PUT; //needed for gcc to compile
    }

    if(!file.good())
    {
      stapl::abort("A general read error has occured!");
      type = PUT; //needed for gcc to compile
    }

    return asset(s, k, r, v, t, type);
  }

public:

  parse_assets(std::string const& FILENAME, unsigned int t)
    : filename(FILENAME), total_num_locations(t)
  {}

  void operator()(unsigned int id,
                  stapl::array_view<stapl::array<asset>> const& data)
  {
    std::ifstream file(filename);
    int total_lines = amount_to_parse();
    int to_do = total_lines/total_num_locations;
    int i = id * to_do;
    int stop = (id + 1) * to_do;

    //ensures that last location picks up any leftover work
    if(id == total_num_locations - 1)
    {
      stop += total_lines % total_num_locations;
    }

    //we always need to skip at least the first line
    skip_lines(file, i + 1);
    for(; i < stop; ++i)
    {
      data[i] = generate_asset(file);
      skip_lines(file, 1); //skip to next line
    }
  }

  int amount_to_parse()
  {
    std::ifstream file(filename);
    int amount = -1;
    file >> amount;
    if(amount == -1)
    {
      stapl::abort("Failed to read number of lines!");
    }
    return amount;
  }

  //needed for STAPL to be able to move instances of
  //this struct to different locations
  void define_type(stapl::typer& t)
  {
    t.member(filename);
    t.member(total_num_locations);
  }
};


class black_scholes
{
  floating d1(asset const& a)
  {
    return 1/(a.v*std::sqrt(a.t))*(std::log(a.s/a.k)+(a.r+(a.v*a.v)/2)*(a.t));
  }

  floating d2(asset const& a)
  {
    return 1/(a.v*std::sqrt(a.t))*(std::log(a.s/a.k)+(a.r-(a.v*a.v)/2)*(a.t));
  }

  floating N(floating x)
  {
    return 0.5*(1+std::erf(x/SQRT2));
  }

  floating calculate_call(asset const& a)
  {
    return N(d1(a))*a.s-N(d2(a))*a.k*std::exp(-a.r*a.t);
  }

  floating calculate_put(asset const& a)
  {
    return N(-d2(a))*a.k*std::exp(-a.r*a.t)-N(-d1(a))*a.s;
  }

public:
  floating operator()(asset const& a)
  {
    if(a.type == CALL)
    {
      return calculate_call(a);
    }
    else if(a.type == PUT)
    {
      return calculate_put(a);
    }
    else
    {
      stapl::abort("The asset class did not specify PUT or CALL!");
      return 0; //needed for gcc to allow compilation
    }
  }
};


stapl::exit_code stapl_main(int argc, char **argv)
{
  std::string in_file;
  std::string out_file;

  const int samples = 64;
  confidence_interval_controller controller(samples, samples, 0.05);
  counter_t count;
  std::string report;
  report += "Test: Parsec Black_Scholes\n";
  report += "Version: STAPL\n";

  //input arguments
  if(argc > 1)
  {
    in_file = std::string(argv[1]);
    if(argc > 2)
    {
      out_file = std::string(argv[2]);
    }
    else
    {
      stapl::do_once([&]()
      {
        std::cerr<<"No output provided. Assuming output is \"output.txt\"\n";
      });
      out_file = "output.txt";
    }
  }
  else
  {
    stapl::do_once([&]()
    {
      std::cerr<<"No arguments provided! Assuming input is \"input.txt\""<<
        " and output is \"output.txt\"\n";
    });
    in_file = "input.txt";
    out_file = "output.txt";
  }

  //create parser
  parse_assets parser(in_file, stapl::get_num_locations());

  //create arrays
  stapl::array<asset> assets(parser.amount_to_parse());
  stapl::array<floating> asset_calculated(assets.size());

  //create views
  auto asset_view = stapl::make_array_view(assets);
  auto asset_calculated_view = stapl::make_array_view(asset_calculated);

  //parse input values in parallel
  stapl::map_func(parser,
    stapl::counting_view<unsigned int>(stapl::get_num_locations()),
    stapl::make_repeat_view(asset_view));

  while(controller.iterate())
  {
    count.reset();
    count.start();
    //do the calculations
    stapl::transform(asset_view, asset_calculated_view, black_scholes());
    count.stop();
    controller.push_back(count.value());
  }

  //print the calculated values to file
  stapl::do_once([&]()
  {
    std::ofstream file(out_file);
    file << asset_calculated_view.size()<<std::endl;
    for(unsigned int i = 0; i < asset_calculated_view.size(); ++i)
    {
      floating a = asset_calculated_view[i];
      file << std::setprecision(std::numeric_limits<floating>::digits10+2);
      file << a << std::endl;
    }
    std::cout << "Input was: " << in_file << std::endl;
    controller.report(report);
  });

  return EXIT_SUCCESS;
}

