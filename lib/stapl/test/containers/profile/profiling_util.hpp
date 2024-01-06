/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_P_CONTAINER_PROFILING_UTIL_HPP
#define STAPL_P_CONTAINER_PROFILING_UTIL_HPP

#include <random>
#include <algorithm>

#include <boost/lexical_cast.hpp>

#ifdef INDEX_STATS
  #include <sstream>
#endif

////////////////////////////////////////////////////////////////////////////////
/// @brief Macro intended to prevent optimizing-out the unused variable.
///
/// Used to profile the get-methods without storing the retrieved value (e.g.,
/// in the scenario when a method will be called on a retrieved object). The
/// results from GCC 4.9 indicate that it is probably not needed though (for
/// std containers, the get method gets optimized anyway, for STAPL it does
/// not even without the macro in place).
////////////////////////////////////////////////////////////////////////////////
#define GET_AND_KEEP(x) { decltype(x) y = x; (void)y; }

////////////////////////////////////////////////////////////////////////////////
/// @brief Simple print function that limits output to location 0
/// @param s A string of characters to print
////////////////////////////////////////////////////////////////////////////////
void stapl_print(const char* s)
{
  if (stapl::get_location_id() == 0)
    std::cout << s << std::flush;
}

/// Generator for random integers uniformly distributed in <tt>[a, b]</tt>.
struct rand_gen
{
  rand_gen(size_t seed_base,
           size_t a = 0, size_t b = std::numeric_limits<size_t>::max())
    : m_gen_engine(seed_base + stapl::get_location_id()), m_dist(a,b)
  { }

  size_t operator()()
  { return m_dist(m_gen_engine); }

private:
  std::mt19937 m_gen_engine;
  std::uniform_int_distribution<size_t> m_dist;
};

void print_index_generator_cmdline_help(std::ostream& os)
{
#ifdef _STAPL
  if (stapl::get_location_id()!=0)
    return;
#endif
  os << "Displaying the index generator options.\n"
     << "--premote #num   The percentage of indices that are remote.\n"
     << "--next_only  Restricts remote indices to adjacent locations.\n\n";
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Obtain the percentage of remote indices and whether the remote
///   indices are restricted to adjacent locations from command line arguments.
////////////////////////////////////////////////////////////////////////////////
std::tuple<size_t, bool> determine_index_generator_param(int argc, char** argv)
{
  size_t premote = 0;
  bool next_only = false;

  for (int i = 1; i < argc; i++) {
    if (!strcmp("--premote", argv[i])) {
      premote = boost::lexical_cast<size_t>(argv[++i]);
    }
    else if (!strcmp("--next_only", argv[i])) {
      next_only = true;
    }
  }

  return std::make_tuple(premote, next_only);
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Helper function to shift given index from current location domain to
///   the domain of the left neighbor (if possible).
///
/// In case of current domain being the first, shifts to right instead.
/// In case of single location, returns the index unchanged.
////////////////////////////////////////////////////////////////////////////////
void shift_to_left(size_t& idx, size_t first, size_t sz, size_t num_locs)
{
  if ((first + sz + idx) < num_locs * sz)
    idx = (first + sz + idx) % (num_locs * sz);
  else if (first >= idx)
    idx = first - idx;
  // else: single location
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Helper function to shift given index from current location domain to
///   the domain of the right neighbor (if possible).
///
/// In case of current domain being the last, shifts to left instead.
/// In case of single location, returns the index unchanged.
////////////////////////////////////////////////////////////////////////////////
void shift_to_right(size_t& idx, size_t first, size_t sz, size_t num_locs)
{
  if (first >= idx)
    idx = first - idx;
  else
    idx = (first + sz + idx) % (num_locs * sz); // wraps around in case of 1 loc
}

//////////////////////////////////////////////////////////////////////////////
/// @brief Generates indices to a 1d block-distributed container with a
/// specified percentage being remote.
///
/// @todo For views whose domain does not map 1on1 to the container's domain,
///   the indices generated must be properly adjusted so as not to fall out
///   of the view's domain. One possiblity is to extract local domains from
///   the coarsened version of such a view and make sure the generated indices
///   fall into that domain (and possibly to subdomains on other locations
///   for non-zero per_remote).
///
/// @tparam Idx Type of the generated indices
///
/// @param rnd Random numbers generator
/// @param first The first index value for the location
/// @param sz The number of indices to generate (size of the block)
/// @param per_remote The percentage of indices that are remote
/// @param next_only Restricts remote indices to neighbors
//////////////////////////////////////////////////////////////////////////////
template <typename Idx>
std::vector<Idx> generate_indices(rand_gen& rnd, size_t first, size_t sz,
  size_t per_remote, bool next_only)
{
  const size_t num_locs = stapl::get_num_locations();
  std::vector<Idx> indices(sz);

  // generate first as if all are local
  for ( size_t i = 0; i < sz; ++i)
    indices[i] = first + (rnd() % sz);

  if (per_remote != 0)
  {
    // there is a remote percentage
    assert(per_remote < 100);
    size_t nremote = (per_remote*sz) / 100;

    if (nremote != 0)
    {
      size_t bs = sz/nremote;

      if (next_only)
      {
        // remote only on neighbor (distributed evenly between the left and
        // right neighbor except for the first and last locations, where all
        // remote indices are on the single neighboring location)
        bool left = false;
        for ( size_t i = 0; i < sz; i+=bs)
        {
          if (!left)
            shift_to_left(indices[i], first, sz, num_locs);
          else
            shift_to_right(indices[i], first, sz, num_locs);

          left = !left;
        }
      }
      else
      {
        // all over the place
        for (size_t i = 0; i < sz; i+=bs)
          indices[i] = rnd() % (num_locs * sz);
      }
    }
  }

#ifdef INDEX_STATS
  // Check how many duplicate values have been generated for the set
  auto ind_tmp = indices;
  std::sort(ind_tmp.begin(), ind_tmp.end());
  size_t unique_cnt =
    std::unique(ind_tmp.begin(), ind_tmp.end()) - ind_tmp.begin();
  float dup_ratio = 1. - float(unique_cnt) / float(sz);

  std::stringstream ss;
  ss << "Loc " << stapl::get_location_id() << ": "
     << dup_ratio * 100 << "\% duplicates in the generated index set."
     << std::endl;

  std::cout << ss.str();
#endif // INDEX_STATS

  return indices;
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Generates indices to a volumetric-distributed two-dimensional
/// container with a specified percentage being remote.
///
/// @todo Generalize to n-d.
///
/// @todo Generation of remote indices does not follow the volumetric
///   distribution and needs to be fixed to generate sensible n-d indices on
///   neighboring locations.
///
/// @todo For views whose domain does not map 1on1 to the container's domain,
///   the indices generated must be properly adjusted so as not to fall out
///   of the view's domain. One possiblity is to extract local domains from
///   the coarsened version of such a view and make sure the generated indices
///   fall into that domain (and possibly to subdomains on other locations
///   for non-zero per_remote).
///
/// @tparam Idx Type of the generated indices
///
/// @param rnd Random numbers generator
/// @param firstn Starting index of n dimension
/// @param firstm Starting index of m dimension
/// @param n Final number of n indices
/// @param m Final number of m indices
/// @param p Percentage of remote indices
/// @param next_only If true, remote indices are only on direct neighbor
////////////////////////////////////////////////////////////////////////////////
template <typename Idx>
std::vector<Idx> generate_indices(rand_gen& rnd,
  size_t firstn, size_t firstm, size_t n, size_t m, size_t p,
  bool next_only)
{
  using namespace stapl;

  typedef tuple<size_t, size_t> gid_type;

  const size_t num_locs = get_num_locations();

  std::vector<Idx> indices;
  indices.reserve(n*m);

  // generate first as if all are local
  for (size_t i=0; i<n*m; ++i)
    indices.emplace_back(firstn + (rnd() % n), firstm + (rnd() % m));

  if (p != 0) // some indices non-local
  {
    assert(p < 100);
    size_t nremote = (p*n*m) / 100;

    if (nremote != 0)
    {
      size_t bs = (n*m)/nremote;
      if (next_only) // remote only on neighbor
      {
        bool left=false;
        for (size_t i=0; i<n*m; i+=bs)
        {
          if (!left)
          {
            shift_to_left(get<0>(indices[i]));
            shift_to_left(get<1>(indices[i]));
          }
          else
          {
            shift_to_right(get<0>(indices[i]));
            shift_to_right(get<1>(indices[i]));
          }

          left = !left;
        }
      }
      else
      {
        // all over the place
        for (size_t i=0; i<n*m; i+=bs)
          indices[i] = make_tuple( rnd() % (num_locs*n), rnd() % (num_locs*m));
      }
    }
  }

  return indices;
}

#endif
