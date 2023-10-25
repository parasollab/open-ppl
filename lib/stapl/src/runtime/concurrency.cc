/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#include <stapl/runtime/concurrency/config.hpp>
#include <stapl/runtime/concurrency/concurrency.hpp>
#include <stapl/runtime/concurrency/thread_local_storage.hpp>
#include <stapl/runtime/exception.hpp>
#include <stapl/runtime/utility/algorithm.hpp>
#include <stapl/runtime/utility/string.hpp>
#include <climits>
#include <functional>
#include <iostream>
#include <numeric>
#include <string>
#include <sstream>
#include <utility>
#include <boost/lexical_cast.hpp>
#if defined(STAPL_RUNTIME_USE_OMP)
# include <omp.h>
#elif defined(STAPL_RUNTIME_USE_THREAD)
#else
# error "Multi-threading backend not defined."
#endif

namespace stapl {

// Maximum number of bits supported through the affinity tag.
const std::size_t affinity_tag::num_bits =
  (sizeof(affinity_tag::tag_type) * CHAR_BIT);


std::ostream& operator<<(std::ostream& os, const affinity_tag affinity)
{
  return os << '{' << affinity.tag << '}';
}


namespace runtime {

/// Master thread id.
static std::thread::id master_thread_id;

/// Processing element hierarchy.
static std::vector<unsigned int> hierarchy;

/// Number of available levels.
static std::vector<unsigned int>::size_type max_num_levels = 0;

/// Maximum number of threads.
static unsigned int max_num_threads = 0;


/// Bits reserved for process affinity.
static unsigned int process_affinity_width = 0;

/// Bits reserved for each hierarchy level.
static std::vector<unsigned int> affinity_width;


/// Master mutex.
static hierarchical_mutex process_mutex;


////////////////////////////////////////////////////////////////////
/// @brief Thread information.
///
/// @ingroup concurrency
////////////////////////////////////////////////////////////////////
struct thread_info_t
{
  /// Next available level.
  std::vector<unsigned int>::size_type level;
  /// Current affinity tag.
  affinity_tag                         affinity;
  /// Shared mutex.
  hierarchical_mutex*                  mutex;
};


#if defined(STAPL_RUNTIME_USE_OMP)

static thread_info_t thread_info = { 0, invalid_affinity_tag, nullptr };
#pragma omp threadprivate(thread_info)

#elif defined(STAPL_RUNTIME_USE_THREAD)

static STAPL_RUNTIME_THREAD_LOCAL_SPEC thread_info_t thread_info =
  { 0, invalid_affinity_tag, nullptr };

#endif


////////////////////////////////////////////////////////////////////
/// @brief Recursive forking for implementing @ref concurrency::fork().
///
/// @param parent_tid  Parent thread id.
/// @param parent_info Parent thread information.
/// @param end_level   Last required level.
/// @param af_offset   Affinity tag offset for the newly created threads.
/// @param f           Function to invoke at the end of the recursion.
///
/// @ingroup concurrency
////////////////////////////////////////////////////////////////////
static void fork_impl(const unsigned int parent_tid,
                      thread_info_t const& parent_info,
                      const unsigned int end_level,
                      const unsigned int af_offset,
                      std::function<void(unsigned int)> f)
{
  const auto n_level = (parent_info.level + 1u);
  hierarchical_mutex n_mutex{parent_info.mutex};
  const auto nth = hierarchy[parent_info.level];

#if defined(STAPL_RUNTIME_USE_OMP)

#pragma omp parallel default(none)                                \
                     shared(affinity_width, parent_info, n_mutex) \
                     firstprivate(f)                              \
                     num_threads(nth)
  {
    const auto tid   = omp_get_thread_num();
    const auto n_tid = (parent_tid * nth + tid);
    const auto n_af  =
      affinity_tag::make_tag(parent_info.affinity.tag | (tid << af_offset));
    const thread_info_t n_info = { n_level, n_af, &n_mutex };
    if (n_level==end_level) {
      // reached end of forking, invoke function
      thread_info = n_info;
      f(n_tid);
    }
    else {
      // fork recursively
      const auto n_offset = (af_offset + affinity_width[parent_info.level]);
      fork_impl(n_tid, n_info, end_level, n_offset, std::move(f));
    }
  }

#elif defined(STAPL_RUNTIME_USE_THREAD)

  std::vector<std::thread> threads;
  threads.reserve(nth - 1);

  if (n_level==end_level) {
    // reached end of forking, invoke function
    auto wf = [parent_tid, &parent_info, nth, af_offset, n_level, &n_mutex](
                const unsigned int tid,
                std::function<void(unsigned int)> f)
      {
        const auto n_tid = (parent_tid * nth + tid);
        const auto n_af  =
          affinity_tag::make_tag(parent_info.affinity.tag | (tid << af_offset));
        thread_info = { n_level, n_af, &n_mutex };
        f(n_tid);
      };

    for (auto i = 1u; i < nth; ++i)
      threads.emplace_back(wf, i, f);
    wf(0, std::move(f));
  }
  else {
    // invoke forking recursively
    auto wf =
      [parent_tid, &parent_info, nth, af_offset, n_level, end_level, &n_mutex](
        const unsigned int tid,
        std::function<void(unsigned int)> f)
    {
      const auto n_tid = (parent_tid * nth + tid);
      const auto n_af  =
        affinity_tag::make_tag(parent_info.affinity.tag | (tid << af_offset));
      const thread_info_t n_info = { n_level, n_af, &n_mutex };
      const auto n_offset = (af_offset + affinity_width[parent_info.level]);
      fork_impl(n_tid, n_info, end_level, n_offset, std::move(f));
    };

    for (auto i = 1u; i < nth; ++i)
      threads.emplace_back(wf, i, f);
    wf(0, std::move(f));
  }

  // wait for all threads
  for (auto& t : threads)
    t.join();

#endif
}


// Initializes the concurrency layer
void concurrency::initialize(option const& opts,
                             const process_id pid,
                             const unsigned int npids,
                             const unsigned int nppn)
{
  master_thread_id = std::this_thread::get_id();

  const auto hier_s = opts.get<std::string>("STAPL_PROC_HIERARCHY", "");
  const auto nth_s  = opts.get<std::string>("STAPL_NUM_THREADS", "");

  if (hier_s!="" && nth_s!="")
    STAPL_RUNTIME_ERROR("Cannot define both STAPL_NUM_THREADS and "
                        "STAPL_PROC_HIERARCHY.");

  if (hier_s=="auto" || nth_s=="auto") {
    // automatically detect number hierarchy information
#if defined(STAPL_RUNTIME_USE_OMP)
    const unsigned int nth = omp_get_max_threads();
    hierarchy = { nth };
#elif defined(STAPL_RUNTIME_USE_THREAD)
    hierarchy = { (std::thread::hardware_concurrency() / nppn) };
#endif
  }
  else if (hier_s!="") {
    // parse CPU hierarchy information
    hierarchy = split_string_to_vector<unsigned int>(hier_s, ", \t");
    if (hierarchy.empty())
      STAPL_RUNTIME_ERROR("STAPL_PROC_HIERARCHY can only contain unsigned "
                          "integers delimited by spaces or commas or the "
                          "word \"auto\".");
  }
  else if (nth_s!="") {
    // STAPL_NUM_THREADS defined
    try {
      hierarchy = { boost::lexical_cast<unsigned int>(nth_s) };
    }
    catch (boost::bad_lexical_cast&) {
      STAPL_RUNTIME_ERROR("Define STAPL_NUM_THREADS with a positive number of "
                          "threads.");
    }
  }
  else {
    // nothing defined
    STAPL_RUNTIME_ERROR("STAPL_NUM_THREADS or STAPL_PROC_HIERARCHY required.");
  }

  // maximum level allowed
  max_num_levels = opts.get("STAPL_PROC_MAX_LEVELS", hierarchy.size());
  if (max_num_levels>hierarchy.size())
    max_num_levels = hierarchy.size();

  // maximum number of threads
  max_num_threads = 1u;
  for (auto i = 0u; i < max_num_levels; ++i)
    max_num_threads *= hierarchy[i];

  // calculate affinity tag widths
  process_affinity_width = integral_ceil_log2(npids);
  affinity_width.reserve(hierarchy.size());
  for (auto threads_per_level : hierarchy)
    affinity_width.push_back(integral_ceil_log2(threads_per_level));
  const auto total_affinity_width =
    (process_affinity_width +
     std::accumulate(std::begin(affinity_width), std::end(affinity_width), 0u));
  if (total_affinity_width > affinity_tag::num_bits)
    STAPL_RUNTIME_ERROR("Increase the affinity_tag size and recompile.");

  // process affinity
  const auto process_affinity = affinity_tag::make_tag(pid);
  if (process_affinity==invalid_affinity_tag)
    STAPL_RUNTIME_ERROR("Invalid affinity tag.");

  // process information
  thread_info = { 0, process_affinity, &process_mutex };

  // check for oversubscription
  const bool oversubscribed =
    ((float(max_num_threads)*nppn) > std::thread::hardware_concurrency());
  if (oversubscribed)
    STAPL_RUNTIME_WARNING("Node is oversubscribed.");

  // output verbose information
  if (opts.count("STAPL_RUNTIME_VERBOSE")>0) {
    std::ostringstream os;
    os << std::boolalpha;

    os << '[' << pid << "] concurrency module:\n"
       << "\tbackend = "
       <<
#if defined(STAPL_RUNTIME_USE_OMP)
          "openmp"
#elif defined(STAPL_RUNTIME_USE_THREAD)
          "std::thread"
#endif
       << '\n'
       << "\tnum_threads = "    << max_num_threads << '\n'
       << "\toversubscribed = " << oversubscribed  << '\n';

    // hierarchy
    std::ostringstream tmp;
    for (auto i : hierarchy)
      tmp << i << ' ';
    os << "\thierarchy = [ "            << nppn << ' ' << tmp.str() << "]\n";
    os << "\tprocess_hierarchy = [ "    << tmp.str()                << "]\n";
    os << "\tprocess_max_num_levels = " << max_num_levels           << '\n';

    // affinity tag
    tmp.str("");
    for (auto i : affinity_width)
      tmp << i << ' ';
    os << "\taffinity_widths = [ "
       << process_affinity_width << ' '
       << tmp.str()              << "]\n";
    os << "\tprocess_affinity_widths = [ " << tmp.str() << "]\n";

    std::cout << os.str();
  }
}


// Finalizes the concurrency layer
void concurrency::finalize(void)
{
  auto& info = thread_info;
  if (info.affinity==invalid_affinity_tag)
    STAPL_RUNTIME_ERROR("Already finalized threading backend.");
  if (info.level!=0)
    STAPL_RUNTIME_ERROR("Cannot shutdown threading backend while threads are "
                        "active.");
  info = { 0, invalid_affinity_tag, nullptr };
}


// Returns number of concurrent threads provided to the runtime.
unsigned int concurrency::hardware_concurrency(void) noexcept
{
  return max_num_threads;
}


// Returns the master thread id
std::thread::id concurrency::get_master_thread_id(void) noexcept
{
  return master_thread_id;
}


// Returns the parallelism level widths.
std::vector<unsigned int> concurrency::get_level_widths(void)
{
  return std::vector<unsigned int>(std::begin(hierarchy),
                                   (std::begin(hierarchy) + max_num_levels));
}


// Returns the available nested parallelism levels
unsigned int concurrency::available_levels(void) noexcept
{
  return (max_num_levels - thread_info.level);
}


// Returns the affinity of the current thread
affinity_tag concurrency::get_affinity(void) noexcept
{
  return thread_info.affinity;
}


// Returns the hierarchical mutex
hierarchical_mutex& concurrency::get_mutex(void) noexcept
{
  return *(thread_info.mutex);
}


// Forks and executes the given function, consuming n levels of parallelism
void concurrency::fork(std::function<void(unsigned int)> init,
                       std::function<void(unsigned int)> f,
                       const unsigned int n)
{
  if (n==0)
    STAPL_RUNTIME_ERROR("Number of levels has to be a positive number.");

  auto& info = thread_info;

  const auto saved_info = info;

  // if the mutex is the process mutex, then this is first invocation of fork()
  // set mutex to nullptr to avoid having the process_mutex as the parent mutex
  if (info.mutex==&process_mutex)
    info.mutex = nullptr;

  // begin and end levels
  const auto begin_lvl = info.level;
  const auto end_lvl   = (((begin_lvl + n) < max_num_levels) ? (begin_lvl + n)
                                                             : max_num_levels);

  // total number of threads participating in [begin_lvl, end_lvl)
  const auto nth = std::accumulate((std::begin(hierarchy) + begin_lvl),
                                   (std::begin(hierarchy) + end_lvl),
                                   1u, std::multiplies<unsigned int>());
  init(nth);

  if (nth==1) {
    // no levels left or number of threads 1, execute in place
    info.level = end_lvl;
    if (!info.mutex)
      info.mutex = &process_mutex;
    f(0);
  }
  else {
#if defined(STAPL_RUNTIME_USE_OMP)
    const bool omp_nesting_enabled = (omp_get_nested()!=0);
    if (!omp_nesting_enabled)
      omp_set_nested(1);
#endif

    // affinity offset for the current level
    const auto offset =
      (process_affinity_width +
       std::accumulate(std::begin(affinity_width),
                       std::begin(affinity_width) + begin_lvl,
                       0u));

    fork_impl(0, info, end_lvl, offset, std::move(f));

#if defined(STAPL_RUNTIME_USE_OMP)
    if (!omp_nesting_enabled)
      omp_set_nested(0);
#endif
  }

  info = saved_info;
}

} // namespace runtime

} // namespace stapl
