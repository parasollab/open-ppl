/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <iostream>
#include <vector>
#include <cstdlib>
#include <iomanip>

// The choices for alltoall versions are currently hybrid, flat, and butterfly
#ifndef ALLTOALL_TAG
#define ALLTOALL_TAG hybrid
#endif

// There are two optimizations which improve IS performance which you can
// uncomment:
// 1. Using a buffer for the bucketing phase
//#define USE_BUFFER
// 2. Using persistent task graphs
//#define USE_PERSISTENT

#include <stapl/runtime.hpp>
#include <stapl/containers/array/array.hpp>
#include <stapl/algorithms/functional.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/views/functor_view.hpp>
#include <stapl/runtime/location_specific_storage.hpp>
#include <stapl/views/repeated_view.hpp>
#include <stapl/views/metadata/coarseners/all_but_last.hpp>

//includes from skeletons
#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/executors/execute.hpp>
#include <stapl/skeletons/functional/alltoall.hpp>
#include <stapl/skeletons/functional/allreduce.hpp>
#include <stapl/skeletons/functional/map.hpp>
#include <stapl/skeletons/functional/zip.hpp>
#include <stapl/skeletons/operators/compose.hpp>

#include <boost/shared_ptr.hpp>
#include <stapl/skeletons/utility/lightweight_vector.hpp>

#ifdef USE_NEW_NOTATION
#include <stapl/skeletons/operators/define_dag.hpp>
#endif

using namespace stapl;

typedef unsigned int            val_t;
typedef unsigned int            int_t;

#ifdef USE_BUFFER
std::vector<lightweight_vector<int_t>> buffer;
#endif

const int_t test_array_size =  5;

const long S_test_index_array[test_array_size] = {48427,17148,23627,62548,4431};
const long S_test_rank_array[test_array_size]  = {0,18,346,64917,65463};

const long W_test_index_array[test_array_size] = {357773, 934767, 875723,
                                                  898999, 404505};
const long W_test_rank_array[test_array_size]  = {1249, 11698, 1039987,
                                                  1043896, 1048018};

const long A_test_index_array[test_array_size] = {2112377, 662041, 5336171,
                                                  3642833, 4250760};
const long A_test_rank_array[test_array_size]  = {104, 17523, 123928,
                                                  8288932, 8388264};

const long B_test_index_array[test_array_size] = {41869, 812306, 5102857,
                                                  18232239, 26860214};
const long B_test_rank_array[test_array_size]  = {33422937, 10244, 59149,
                                                  33135281,99};

const long C_test_index_array[test_array_size] = {44172927, 72999161, 74326391,
                                                  129606274, 21736814};
const long C_test_rank_array[test_array_size]  = {61147, 882988, 266290,
                                                  133997595, 133525895};

const long D_test_index_array[test_array_size] = {1317351170, 995930646,
                                                  1157283250, 1503301535,
                                                  1453734525};
const long D_test_rank_array[test_array_size]  = {1,36538729,
                                                  1978098519, 2145192618,
                                                  2147425337};

long test_index_array[test_array_size];
long test_rank_array[test_array_size];

double randlc(double *X, double *A)
{
  static int        KS=0;
  static double     R23, R46, T23, T46;
  double            T1, T2, T3, T4;
  double            A1;
  double            A2;
  double            X1;
  double            X2;
  double            Z;
  int               i, j;

  // ??? KS fixed to 0 above...
  if (KS == 0)
  {
    R23 = 1.0;
    R46 = 1.0;
    T23 = 1.0;
    T46 = 1.0;

    for (i=1; i<=23; i++)
    {
      R23 = 0.50 * R23;
      T23 = 2.0 * T23;
    }

   for (i=1; i<=46; i++)
    {
      R46 = 0.50 * R46;
      T46 = 2.0 * T46;
    }

    KS = 1;
  }

  //
  // Break A into two parts such that A = 2^23 * A1 + A2 and set X = N.
  //
  T1 = R23 * *A;
  j  = T1;
  A1 = j;
  A2 = *A - T23 * A1;

  // Break X into two parts such that X = 2^23 * X1 + X2, compute
  // Z = A1 * X2 + A2 * X1  (mod 2^23), and then
  // X = 2^23 * Z + A2 * X2  (mod 2^46).

  T1 = R23 * *X;
  j  = T1;
  X1 = j;
  X2 = *X - T23 * X1;
  T1 = A1 * X2 + A2 * X1;

  j  = R23 * T1;
  T2 = j;
  Z  = T1 - T23 * T2;
  T3 = T23 * Z + A2 * X2;
  j  = R46 * T3;
  T4 = j;
  *X = T3 - T46 * T4;

   return R46 * *X;
}

void initialize_test_arrays(std::string problem_class)
{
  long const* index_array;
  long const* rank_array;

  bool val_set = false;

  if (problem_class == "S")
  {
    index_array = S_test_index_array;
    rank_array  = S_test_rank_array;
    val_set     = true;
  }

  if (problem_class == "A")
  {
    index_array = A_test_index_array;
    rank_array  = A_test_rank_array;
    val_set     = true;
  }

  if (problem_class == "W")
  {
    index_array = W_test_index_array;
    rank_array  = W_test_rank_array;
    val_set     = true;
  }

  if (problem_class == "B")
  {
    index_array = B_test_index_array;
    rank_array  = B_test_rank_array;
    val_set     = true;
  }

  if (problem_class == "C")
  {
    index_array = C_test_index_array;
    rank_array  = C_test_rank_array;
    val_set     = true;
  }

  if (problem_class == "D")
  {
    index_array = D_test_index_array;
    rank_array  = D_test_rank_array;
    val_set     = true;
  }

  if (!val_set)
    stapl::abort("problem class must be s, a, w, b, c, or d\n");

  for (int_t i = 0; i < test_array_size; ++i)
  {
    test_index_array[i] = index_array[i];
    test_rank_array[i]  = rank_array[i];
  }
}


struct problem_traits
{
public:
  constexpr problem_traits(int_t a, int_t b, int_t c,
                 int_t min_locs, int_t max_locs)
    : total_keys_log_2(a),
      max_key_log_2(b),
      num_buckets_log_2(c),
      min_n_locs(min_locs),
      max_n_locs(max_locs)
  { }

  int_t const total_keys_log_2;
  int_t const max_key_log_2;
  int_t const num_buckets_log_2;

  int_t const min_n_locs;
  int_t const max_n_locs;

  constexpr int_t total_keys() const
  {
    return (1 << total_keys_log_2);
  }

  constexpr int_t max_key() const
  {
    return 1 << max_key_log_2;
  }

  constexpr int_t num_buckets() const
  {
    return 1 << num_buckets_log_2;
  }
};

problem_traits compute_problem_traits(std::string problem_class)
{
  if (problem_class == "S")
    return problem_traits(16, 11,  9, 1,  128);

  if (problem_class == "W")
    return problem_traits(20, 16, 10, 1, 1024);

  if (problem_class == "A")
    return problem_traits(23, 19, 10, 1, 1024);

  if (problem_class == "B")
    return problem_traits(25, 21, 10, 1, 1024);

  if (problem_class == "C")
    return problem_traits(27, 23, 10, 1, 1024);

  if (problem_class == "D")
    return problem_traits(29, 27, 10, 4, 1024);

  stapl::abort("problem class must be s, w, a, b, c, or d.\n");

  return problem_traits(0, 0, 0, 0, 0);
}

double find_my_seed(int kn,   // my processor rank, 0<=kn<=num procs
                    int np,   // np = num procs
                    long nn,  // total num of ran numbers, all procs
                    double s, // Ran num seed, for ex.: 314159265.00
                    double a) // Ran num gen mult, try 1220703125.00
{
  long i;
  double t1,t2,an;
  long mq,nq,kk,ik;

  nq = nn / np;

  for (mq=0; nq>1; mq++,nq/=2)
  { }

  t1 = a;

  for (i=1; i<=mq; i++)
  {
    t2 = randlc(&t1, &t1);
  }

  an = t1;
  kk = kn;
  t1 = s;
  t2 = an;

  for (i=1; i<=100; i++)
  {
    ik = kk / 2;

    if (2 * ik !=  kk)
    {
      randlc(&t1, &t2);
    }

    if ( ik == 0 )
    {
      break;
    }

    randlc(&t2, &t2);

    kk = ik;
  }

  return t1;
}

//////////////////////////////////////////////////////////////////////
/// @brief Generates random values specific to NAS benchmark.
///
/// @todo generate fix for static variables
//////////////////////////////////////////////////////////////////////
struct random_sequence_is_func
{
private:
  double       m_a;
  double       m_seed;
  const int_t  m_k;

public:
  typedef void   result_type;

  random_sequence_is_func(double seed, int_t max_key)
    : m_a(1220703125.00),
      m_seed(seed),
      m_k(max_key/4)
  { }

  template <typename V>
  void operator()(V v)
  {
    for (auto&& e : v) {
      double x = randlc(&m_seed, &m_a);

      x += randlc(&m_seed, &m_a);
      x += randlc(&m_seed, &m_a);
      x += randlc(&m_seed, &m_a);

      e = m_k * x;
    }
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_a);
    t.member(m_seed);
    t.member(m_k);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Counts the number of elements in each bucket for NAS IS
/// benchmark.
//////////////////////////////////////////////////////////////////////
struct bucket_cardinality
{
  typedef std::vector<int_t> result_type;

  int_t const m_num_buckets;
  int_t const m_shift;

  bucket_cardinality(int_t const num_buckets,
                     int_t const shift)
    : m_num_buckets(num_buckets),
      m_shift(shift)
  { }

  template <typename Keys>
  result_type operator()(Keys&& keys) const
  {
    result_type result(m_num_buckets);
    for (auto&& elem : keys)
    {
      ++result[elem >> m_shift];
    }
    return result;
  }

  void define_type(typer& t)
  {
    t.member(m_num_buckets);
    t.member(m_shift);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Adds two vector of elements of type @c T.
///
/// @tparam T element type
//////////////////////////////////////////////////////////////////////
template <typename T>
struct vec_plus
{
  typedef std::vector<T> result_type;

  template <typename V1, typename V2>
  result_type operator()(V1&& v1, V2&& v2) const
  {
    result_type result;
    result.reserve(v1.size());
    std::transform(v1.begin(), v1.end(), v2.begin(),
                   std::back_inserter(result), std::plus<T>());
    return result;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Determines the redistribution strategy for the partitions
/// in NAS IS pattern to obtain better load balance.
///
/// The goal in this redistribution strategy is to reduce load imbalance
/// between partitions by giving each partition at most
/// \f$\frac{#keys}{#partitions}\f$. The last partition is the only
/// one that can exceed this limit.
//////////////////////////////////////////////////////////////////////
struct bucket_and_element_offset
{
  typedef std::pair<int_t, int_t> val_t;
  typedef std::vector<val_t> result_type;

  int_t const m_max_elems_per_partition;
  int_t const m_num_partitions;

  bucket_and_element_offset(int_t const total_keys,
                            int_t const num_partitions)
    : m_max_elems_per_partition(total_keys/num_partitions),
      m_num_partitions(num_partitions)
  { }

  template <typename VSizes>
  result_type operator()(VSizes&& global_bucket_sizes) const
  {
    result_type result;

    result.reserve(m_num_partitions);
    if (m_num_partitions > 0) {
      int_t bucket_sum_accumulator = 0;
      int_t cur_bucket_offset = 0;
      int_t cur_key_offset = 0;

      int_t j = 0;
      int_t i = 0;

      for (auto&& global_bucket_size : global_bucket_sizes)
      {
        bucket_sum_accumulator += global_bucket_size;

        if (bucket_sum_accumulator >= (j+1) * m_max_elems_per_partition) {
          result.emplace_back(cur_bucket_offset, cur_key_offset);
          cur_bucket_offset = i + 1;
          cur_key_offset = bucket_sum_accumulator;
          ++j;
        }
        ++i;
      }
      if (result.size() < m_num_partitions)
        result.resize(m_num_partitions,
                      val_t(cur_bucket_offset, cur_key_offset));
    }
    return result;
  }

  void define_type(typer& t)
  {
    t.member(m_max_elems_per_partition);
    t.member(m_num_partitions);
  }
};



//////////////////////////////////////////////////////////////////////
/// @brief With the information obtained from @c bucket_and_element_offset
/// partition the input and make it ready for an @c alltoall data
/// transfer with other partitions.
///
/// @tparam T type of the elements
//////////////////////////////////////////////////////////////////////
template <typename T>
struct prepare_for_alltoall
{
  typedef std::vector<T> result_type;

  int_t const m_num_buckets;
  int_t const m_shift;

  prepare_for_alltoall(int_t const num_buckets,
                       int_t const shift)
    : m_num_buckets(num_buckets),
      m_shift(shift)
  { }

  template <typename BEOffset, typename BSizes, typename Keys>
  result_type operator()(BEOffset&& b_and_e_offsets,
                         BSizes&& local_bucket_sizes, Keys&& keys) const
  {
    std::size_t const num_partitions = b_and_e_offsets.size();

#ifdef USE_BUFFER
    buffer.resize(num_partitions);
#else
    result_type result;
    result.reserve(num_partitions);
#endif

    std::vector<typename T::iterator> bucket_iters;
    bucket_iters.reserve(m_num_buckets);

    //we need to reserve memory for buckets
    for (int_t cur_part = 0, cur_bucket = 0;
         cur_part < num_partitions; ++cur_part) {
      int_t next_offset =
        cur_part == num_partitions - 1 ?
          m_num_buckets :
          b_and_e_offsets[cur_part+1].first;

      int_t size = 0;
      int_t i = cur_bucket;
      for (; cur_bucket < next_offset; ++cur_bucket) {
        size += local_bucket_sizes[cur_bucket];
      }

#ifdef USE_BUFFER
      buffer[cur_part].resize(size);
#else
      result.emplace_back(size);
#endif

      cur_bucket = i;
#ifdef USE_BUFFER
      auto&& bucket_it = buffer[cur_part].begin();
#else
      auto&& bucket_it = result[cur_part].begin();
#endif

      for (; cur_bucket < next_offset; ++cur_bucket) {
        bucket_iters.push_back(bucket_it);
        std::advance(bucket_it, local_bucket_sizes[cur_bucket]);
      }
    }

    for (auto&& key : keys)
    {
      (*bucket_iters[key >> m_shift]++) = key;
    }

#ifdef USE_BUFFER
    return buffer;
#else
    return result;
#endif
  }

  void define_type(typer& t)
  {
    t.member(m_num_buckets);
    t.member(m_shift);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Puts elements of the sorted keys on this partition in their
/// correct position in the output.
///
/// @tparam type of the elements
//////////////////////////////////////////////////////////////////////
struct final_sort
{
  int_t const m_cur_partition_index;
  int_t const m_num_buckets;
  int_t const m_keys_per_bucket;

  typedef void result_type;

  final_sort(int_t const cur_partition_index,
             int_t const num_buckets,
             int_t const keys_per_bucket)
    : m_cur_partition_index(cur_partition_index),
      m_num_buckets(num_buckets),
      m_keys_per_bucket(keys_per_bucket)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief writes the sorted key sequence to its right position in the
  /// output.
  ///
  /// @param chunks      the set of keys in the input that fall into the
  ///                    range that that this partition is responsible for
  /// @param b_e_offsets the bucket and key offsets from which this
  ///                    partition writes to the output
  /// @li @c output_view the output view to which the sorted sequence
  ///                    will be written.
  //////////////////////////////////////////////////////////////////////
  template <typename Chunks, typename BEOffset, typename Output>
  void operator()(Chunks&&    chunks,
                  BEOffset&&  b_and_e_offsets,
                  Output&&    output_view) const
  {
    int_t const num_partitions = b_and_e_offsets.size();

    // compute min and max value that belongs to this partition
    int_t min_value = b_and_e_offsets[m_cur_partition_index].first
                                  * m_keys_per_bucket;
    int_t const max_value = m_keys_per_bucket *
      ((m_cur_partition_index == num_partitions - 1) ?
        (m_num_buckets) :
        b_and_e_offsets[m_cur_partition_index+1].first);

    // create a vector of counts of the elements that fall in the range
    // [min_value, max_value]
    int_t const size = max_value - min_value;
    std::vector<int_t> counts(size + 2);
    counts[0] = min_value;
    counts[1] = max_value;
    int_t* counts_shifted = counts.data() - min_value + 2;

    // iterate over the received chunks from other partitions and increase
    // the count in the counts vector
    for (auto const& chunk : chunks) {
      for (auto&& elem : chunk) {
       counts_shifted[elem]++;
      }
    }

    counts_shifted = counts.data() + 2;
    int_t offset = b_and_e_offsets[m_cur_partition_index].second;

    for (int_t i = 0; i < size; ++ i) {
      int_t new_offset = counts_shifted[i] + offset;
      counts_shifted[i] = offset;
      offset = new_offset;
    }
    output_view.set_element(m_cur_partition_index, std::move(counts));
  }

  void define_type(typer& t)
  {
    t.member(m_cur_partition_index);
    t.member(m_num_buckets);
    t.member(m_keys_per_bucket);
  }

};

struct partial_verification
{
  typedef bool result_type;
  int_t m_iteration;
  std::vector<int_t> m_verification_ranks;
  std::string m_problem_class;

  partial_verification(int_t iteration,
                       std::vector<int_t>&& verification_ranks,
                       std::string problem_class)
    : m_iteration(iteration),
      m_verification_ranks(verification_ranks),
      m_problem_class(problem_class)
  { }

  template <typename RanksView>
  bool operator()(RanksView&& ranks_view) const
  {
    int_t min_key_val = ranks_view[0];
    int_t max_key_val = ranks_view[1];
    int_t idx = 0;
    for (auto&& k : m_verification_ranks)
    {
      if (min_key_val <= k &&  k <= max_key_val)
      {
        int_t correct_rank;
        if (m_problem_class == "A") {
           correct_rank =
            idx <= 2
              ? test_rank_array[idx] + (m_iteration - 1)
              : test_rank_array[idx] - (m_iteration - 1);
        }
        else if (m_problem_class == "B") {
          correct_rank =
            idx == 1 || idx == 2 || idx ==4
              ? test_rank_array[idx]+ m_iteration
              : test_rank_array[idx]- m_iteration;
        }
        else if (m_problem_class == "C") {
              correct_rank =
               idx <= 2
                ? test_rank_array[idx]+m_iteration
                : test_rank_array[idx]-m_iteration;

        }
        else if (m_problem_class == "D") {
              correct_rank =
               idx < 2
                ? test_rank_array[idx]+m_iteration
                : test_rank_array[idx]-m_iteration;
        }
        else {
          correct_rank =
            idx <= 2 ? test_rank_array[idx]+m_iteration
                       : test_rank_array[idx]-m_iteration;
        }

        //now we have to shift the offset to match the local chunk and skip
        //the first two elements which represent min and max
        int_t rank_index = (k - min_key_val) + 2;
        if (ranks_view[rank_index] != correct_rank) {
          std::cerr << "Failed partial verification: iteration " << m_iteration
                    << ", test key " << idx
                    << " = " << k << " should have rank = "
                    << correct_rank << " but it is = "
                    << ranks_view[rank_index]
                    << std::endl;
          return false;
        }
      }
      ++idx;
    }
    return true;
  }

  void define_type(typer& t)
  {
    t.member(m_iteration);
    t.member(m_verification_ranks);
    t.member(m_problem_class);
  }
};


struct copy_to_sorted_view
{
  typedef void result_type;

  template<typename RanksView, typename OutputView>
  void operator()(RanksView&& ranks_view, OutputView&& output_view)
  {
    int_t size = ranks_view.size();
    //the first two elements are min and max
    int_t const min_key_val = ranks_view[0];

    int_t cur_val = min_key_val;
    if (size > 2) {
      int_t offset = ranks_view[2];
      for (int_t i = 2; i < size; ++i) {
        int_t count = ranks_view[i];
        for (int_t j = 0; j < count; ++j) {
          output_view[offset++] == cur_val;
        }
        ++cur_val;
      }
    }

  }
};

stapl::exit_code stapl_main(int argc, char* argv[])
{
  size_t n_iters = 10;
  const size_t n_locs  = stapl::get_num_locations();
  const size_t loc_id  = stapl::get_location_id();

  if (argc < 2) {
    std::cerr << "Usage: "<< argv[0] << " <problem-class>\n"
              << "Problem classes are a, b, c, d, s1 or s2.\n";
    return EXIT_FAILURE;
  }
  std::string problem_class = argv[1];

  if (argc > 2) {
    n_iters = atol(argv[2]);
  }
  const problem_traits traits = compute_problem_traits(problem_class);

  stapl_assert(
    n_locs >= traits.min_n_locs && n_locs <= traits.max_n_locs,
    "Invalid processor count for this IS problem class.\n"
  );

  initialize_test_arrays(argv[1]);
  const int shift = traits.max_key_log_2 - traits.num_buckets_log_2;

  typedef stapl::static_array<val_t>  container_t;
  typedef array_view<container_t>     view_t;

  int_t keys_array_size = traits.total_keys() * traits.min_n_locs;
  container_t  keys_array(keys_array_size);
  view_t       keys_view(keys_array);

#ifdef FULL_VERIFICATION
  container_t  sorted_keys_array(keys_array_size);
  view_t       sorted_keys_view(keys_array);
#endif


  typedef stapl::static_array<std::vector<val_t>>  ranks_container_t;
  typedef array_view<ranks_container_t>            ranks_view_t;

  ranks_container_t  ranks_array(n_locs);
  ranks_view_t       ranks_view(ranks_array);

  const double seed =
    find_my_seed(
      loc_id,
      n_locs,
      4 * (long) traits.total_keys() * traits.min_n_locs,
      314159265.00,
      1220703125.00
    );

  //initialize the input keys view
  map_func<stapl::skeletons::tags::with_coarsened_wf>(
    random_sequence_is_func(seed, traits.max_key()), keys_view);

  typedef lightweight_vector<val_t> alltoall_val_t;
  typedef vec_plus<val_t>           vec_plus_t;

  //defining the is in terms of skeletons
  using namespace skeletons;
  using skeletons::map;
#ifndef USE_NEW_NOTATION
  DECLARE_INLINE_PLACEHOLDERS(6, x);
  namespace ph = stapl::skeletons::flows::inline_flows::placeholders;
  ph::input<0> keys_in;
  ph::input<1> ranks_in;

  /// IS composition specification:
  ///
  /// input = [keys_view  make_repeat_view(ranks_view)]
  /// v0 = map(bucket_cardinality) [keys_view]
  /// v1 = allreduce(vec_plus_t) [v0]
  /// v2 = map(bucket_and_element_offset) [v1]
  /// v3 = zip<3>(prepare_for_alltoall) [v2 v0 keys_view]
  /// v4 = alltoall() [v3]
  /// v5 = zip<3>(final_sort) [v4 v2 make_repeat_view(ranks_view)]
  /// output = [v5]


  auto is_skeleton =
    skeletons::compose<skeletons::tags::inline_flow>(
      x0 << map(bucket_cardinality(traits.num_buckets(), shift)) | keys_in,
      x1 << allreduce<tags::right_aligned>(vec_plus_t()) | x0,
      x2 << map(bucket_and_element_offset(keys_array_size, n_locs)) | x1,
      x3 << zip<3>(prepare_for_alltoall<alltoall_val_t>(traits.num_buckets(),
                                                   shift)) | (x2, x0, keys_in),
      x4 << alltoall<alltoall_val_t, tags::ALLTOALL_TAG>() | x3,
      x5 << zip<3>(final_sort(
                    loc_id, traits.num_buckets(),
                    traits.max_key() / traits.num_buckets())) | (x4,x2,ranks_in)
    );
#else
  auto x0 = map(bucket_cardinality(traits.num_buckets(), shift));

  auto x12 = ser(
    allreduce<tags::right_aligned>(vec_plus_t()),
    map(bucket_and_element_offset(keys_array_size, n_locs))
  );

  auto x34 = ser(
    zip<3>(prepare_for_alltoall<alltoall_val_t>(traits.num_buckets(), shift)),
    alltoall<alltoall_val_t, tags::ALLTOALL_TAG>()
  );

  auto x5 = zip<3>(final_sort(
                loc_id, traits.num_buckets(),
                traits.max_key() / traits.num_buckets()));

  auto is = stapl::skeletons::shift(
    take<0>(x0),
    take<2>(x12),
    take<3,2,0>(x34),
    take<4,3,1>(x5)
  );

  auto keys_in  = skeletons::id_t{};
  auto ranks_in = skeletons::id_t{};
  auto inputs = par(keys_in, ranks_in);

  auto is_skeleton = to_skeleton(ser(inputs, is));

#endif


#ifdef PERSISTENT
  using pmg_t = paragraph_skeleton_manager<
                  decltype(is_skeleton),
                  decltype(execution_params(
                             coarsen_all_but_last<default_coarsener>()))>;
  auto&& p = paragraph<stapl::persistent_scheduler,
                       pmg_t,
                       decltype(keys_view),
                       decltype(make_repeat_view(ranks_view))>(
                pmg_t(is_skeleton), keys_view, make_repeat_view(ranks_view));
  p();
#else
  skeletons::execute(
    skeletons::execution_params(coarsen_all_but_last<default_coarsener>()),
    is_skeleton,
    keys_view, make_repeat_view(ranks_view));
#endif

  stapl::counter<stapl::default_timer> tmr;
  // barrier to get all the timers in sync.
  stapl::rmi_fence();
  tmr.start();

  int_t check_passed = 0;;
  for (int_t iteration=1; iteration <= n_iters; ++iteration, ++check_passed)
  {
    // print iteration header and mutate 2 keys, per the spec
    if (loc_id == 0)
    {
      keys_view[iteration]         = iteration;
      keys_view[iteration+n_iters] = traits.max_key() - iteration;
    }

    // assignments to keys_view should finish before the computation starts
    rmi_fence();

#ifdef PERSISTENT
    p();
#else
  skeletons::execute(
    skeletons::execution_params(coarsen_all_but_last<default_coarsener>()),
    is_skeleton,
    keys_view, make_repeat_view(ranks_view));
#endif

    std::vector<int_t> verification_ranks;
    verification_ranks.reserve(test_array_size);

    for (size_t idx=0; idx < test_array_size; ++idx) {
      verification_ranks.push_back(keys_view[test_index_array[idx]]);
        }

    bool passed = stapl::map_reduce(
                    partial_verification(
                      iteration, std::move(verification_ranks), problem_class),
                    stapl::logical_and<bool>(), ranks_view);

    if (!passed && loc_id == 0) {
          std::cerr << "Failed partial verification: iteration " << iteration
                    << std::endl;
      return EXIT_FAILURE;
    }
  }

  const double t = tmr.stop();

#ifdef FULL_VERIFICATION
  stapl::map_func(copy_to_sorted_view(),
                  ranks_view, make_repeat_view(sorted_keys_view));

  bool correctly_sorted = stapl::is_sorted(sorted_keys_view);

  if (loc_id == 0) {
    if (correctly_sorted) {
      std::cout << "Full verification succeeded\n";
    }
    else {
      std::cout << "Full verification failed\n";
    }
  }
#endif

  if (loc_id == 0)
  {
    std::cout << "Partial Verifications passed : " << check_passed
              << " out of " << n_iters << std::endl;
    std::cout << "IS Benchmark Results:\n"
              << "CPU Time " << std::setw(10) << std::setprecision(4)
              << t << "\n";
    std::cout << "Benchmark completed\n";
  }

  return EXIT_SUCCESS;
}
