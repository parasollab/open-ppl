/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#include <stapl/runtime/config.hpp>

#ifdef STAPL_RUNTIME_ENABLE_INSTRUMENTATION

# include <stapl/runtime/instrumentation.hpp>
# include <stapl/runtime/system.hpp>
# include <algorithm>
# include <cstdlib>
# include <fstream>
# include <mutex>
# include <sstream>
# include <string>
# include <unordered_map>
# include <vector>
# include <boost/accumulators/accumulators.hpp>
# include <boost/accumulators/statistics.hpp>

# warning "Instrumentation is on. Performance may degrade."

namespace stapl {

namespace runtime {

/// Output filename prefix.
static std::string filename_prefix;

/// Accumulated data.
static std::unordered_map<std::string, unsigned int> acc_data;
static std::mutex                                    acc_data_mtx;

/// Statistical data.
static std::unordered_map<std::string, std::vector<int>> stat_data;
static std::mutex                                        stat_data_mtx;


// Initializes instrumentation
void instrument::initialize(option const& opts)
{
  filename_prefix = opts.get<std::string>("STAPL_INSTRUMENTATION_FILE",
                                          "stapl_instrumentation");
}


// Finalizes instrumentation
void instrument::finalize(void)
{
  if (acc_data.empty() && stat_data.empty())
    return;

  // put all data in a stringstream
  std::ostringstream oss;

  // print accumulated data
  for (auto&& it : acc_data) {
    oss << it->first << ": " << it->second << '\n';
  }

  // print statistical data
  oss << std::endl << "Statistical data:\n";
  for (auto&& it : stat_data) {
    using namespace boost::accumulators;

    oss << it->first << ":\n";

    std::vector<int> const& v = it->second;
    accumulator_set<double, features<tag::mean,tag::min,tag::max>> acc;

    acc = std::for_each(v.begin(), v.end(), acc);

    oss << "\tdata points: " << v.size()                       << '\n'
        << "\tavg: "         << boost::accumulators::mean(acc) << '\n'
        << "\tmin: "         << boost::accumulators::min(acc)  << '\n'
        << "\tmax: "         << boost::accumulators::max(acc)  << '\n';
  }

  // write stringstream to file
  const int pid = getpid();
  std::ostringstream oss_fname;
  oss_fname << filename_prefix << '.' << pid;
  std::ofstream os{oss_fname.str().c_str(), std::ios_base::out};
  os << "# STAPL Instrumentation for pid " << pid << '\n';
  os << oss.str() << std::endl;

  // clear everything
  acc_data.clear();
  stat_data.clear();
}


// Increases the call counter for the given string
void instrument::accumulate(const char* s)
{
  const std::string key{s};

  std::lock_guard<std::mutex> lock{acc_data_mtx};
  ++acc_data[key];
}


// Pushes back the data for the given string
void instrument::push_back(const char* s, int n)
{
  const std::string key{s};

  std::lock_guard<std::mutex> lock{stat_data_mtx};
  std::vector<int>& v = stat_data[key];
  v.push_back(n);
}


// Clears all gathered data
void instrument::clear(void)
{
  std::unique_lock<std::mutex> lock1{acc_data_mtx, std::defer_lock};
  std::unique_lock<std::mutex> lock2{stat_data_mtx, std::defer_lock};
  std::lock(lock1, lock2);
  acc_data.clear();
  stat_data.clear();
}

} // namespace runtime

} // namespace stapl

#endif
