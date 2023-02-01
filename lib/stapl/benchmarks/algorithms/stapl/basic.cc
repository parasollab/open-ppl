/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include "../utilities.hpp"
#include "../timer.hpp"
#include <stapl/algorithms/sorting.hpp>
#include <vector>
#include <iostream>

using container_t = stapl::array<data_t>;
using view_t      = stapl::array_view<container_t>;

// work function used to initialize element on a location with the location id
struct assign_id
{
private:
  data_t m_lid;

public:
  assign_id(unsigned int lid)
    : m_lid(lid)
  { }

  template<typename Ref>
  void operator()(Ref&& val) const
  { val = m_lid; }

  void define_type(stapl::typer& t)
  { t.member(m_lid); }
};


// work function used to initialize vector before butterfly test
// initializes the vector to p elements with values [0, p-1]
struct init_buckets
{
private:
  unsigned int m_nlocs;
  unsigned int m_lid;

public:
  init_buckets(unsigned int nlocs, unsigned int lid)
    : m_nlocs(nlocs), m_lid(lid)
  { }

  template <typename Buckets>
  void operator()(Buckets&& b)
  {
    // Create p buckets locally, each with single element
    stapl::lightweight_vector<stapl::lightweight_vector<double>>
      buckets(m_nlocs);

    // Initialize element in each bucket to a location id [0, nlocs-1]
    unsigned int lid = 0;
    for (auto bucket : buckets)
      bucket.push_back(lid++);

    b = buckets;
  }

  void define_type(stapl::typer& t)
  { t.member(m_nlocs); }
};


// check the result of the butterfly combining scalars
// sum should be the sum of all location ids
struct check_bfly_scalar
{
private:
  unsigned int m_nlocs;

public:
  using result_type = bool;

  check_bfly_scalar(unsigned int nlocs, unsigned int lid)
    : m_nlocs(nlocs)
  { }

  template <typename BucketRef>
  bool operator()(BucketRef&& bucket)
  { return bucket == m_nlocs * (m_nlocs-1) / 2; }

  void define_type(stapl::typer& t)
  { t.member(m_nlocs); }
};


// check the result of the butterfly combining vectors
// sum of elements should be p times the location id
struct check_bfly_vector
{
private:
  unsigned int m_nlocs;
  unsigned int m_lid;

public:
  using result_type = bool;

  check_bfly_vector(unsigned int nlocs, unsigned int lid)
    : m_nlocs(nlocs), m_lid(lid)
  { }

  template <typename BucketRef>
  bool operator()(BucketRef&& bucket)
  {
    return bucket.size() == 1 && bucket[0].size() == m_nlocs &&
      (unsigned int)std::accumulate(bucket[0].begin(), bucket[0].end(), 0) ==
      m_lid * m_nlocs;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_nlocs);
    t.member(m_lid);
  }
};


// work function used in the butterfly combining scalars
template<typename T>
struct scalar_bucket_merge
{
  using result_type = T;

  template<typename Ref0, typename Ref1>
  result_type operator()(Ref0&& r0, Ref1&& r1)
  { return r0 + r1; }

  void set_position(std::size_t, std::size_t, std::size_t, std::size_t)
  { }
};


stapl::exit_code stapl_main(int argc, char *argv[])
{
  const int num_samples = argc == 2 ? atoi(argv[1]) : 10;

  std::vector<double> map_samples(num_samples,0.),
    map_reduce_samples(num_samples,0.), reduce_samples(num_samples, 0.),
    scan_samples(num_samples,0.), bfly_scalar_samples(num_samples,0.),
    bfly_vector_samples(num_samples,0.);

  bool map_correct(true), map_reduce_correct(true), scan_correct(true),
    reduce_correct(true), bfly_scalar_correct(true), bfly_vector_correct(true);

  counter_t timer;

  unsigned int nlocs = stapl::get_num_locations();
  unsigned int lid   = stapl::get_location_id();

  // sum of [0, nlocs-1]
  unsigned int expected_sum = (nlocs-1)*nlocs/2;

  // sum of exclusive scan of [0, nlocs-1]
  unsigned int expected_scan_sum = 0;
  for (unsigned int i = 0; i != nlocs; i++)
    expected_scan_sum += i * (nlocs - (i + 1));


  for (int sample = 0; sample != 10; ++sample)
  {
    // Container for algorithm basic tests
    stapl::array<data_t> bcont(nlocs);
    stapl::array_view<stapl::array<data_t>> b(bcont);

    // map_func test fills elements
    timer.reset();
    timer.start();

    stapl::map_func(assign_id(lid), b);

    map_samples[sample] = timer.stop();

    // reduce call checks sum
    timer.reset();
    timer.start();

    data_t red_result = stapl::reduce(b, stapl::plus<data_t>());

    reduce_samples[sample] = timer.stop();

    // scan is the commonly used exclusive scan
    timer.reset();
    timer.start();

    stapl::scan(b, b, stapl::plus<data_t>(), true);

    scan_samples[sample] = timer.stop();

    // map_reduce checks the sum after scan
    timer.reset();
    timer.start();

    data_t mapred_result =
      stapl::map_reduce(stapl::identity<data_t>(), stapl::plus<data_t>(), b);

    map_reduce_samples[sample] = timer.stop();

    map_correct = map_correct && red_result == expected_sum;

    reduce_correct = reduce_correct && red_result == expected_sum;

    map_reduce_correct = map_reduce_correct &&
      mapred_result == expected_scan_sum;

    scan_correct = scan_correct &&
      mapred_result == expected_scan_sum;

    // Container for butterfly test represents locally partitioned elements
    // with each partition containing a single element
    using bfly_data_t =
      stapl::lightweight_vector<stapl::lightweight_vector<double>>;
    using vector_cont_t = stapl::array<bfly_data_t>;

    vector_cont_t bucketed_data_in(nlocs);;
    vector_cont_t bucketed_data_out(nlocs);
    stapl::array_view<vector_cont_t> bucketed_in(bucketed_data_in);
    stapl::array_view<vector_cont_t> bucketed_out(bucketed_data_out);

    stapl::map_func(init_buckets(nlocs, lid), bucketed_in);

    // Butterfly to exchange vectors between locations.  Each location goes from
    // holding p vectors of size 1 to holding a single vector of size p.
    timer.reset();
    timer.start();

    typedef stapl::skeletons::spans::nearest_pow_two<
      stapl::skeletons::spans::balanced<>> bfly_span_t;

    stapl::skeletons::execute(stapl::skeletons::default_execution_params(),
      stapl::skeletons::sink<bfly_data_t>(
        stapl::skeletons::butterfly<true, stapl::use_default, bfly_span_t>
          (stapl::algo_details::bucket_merge_wf<data_t>(nlocs))),
      bucketed_in, bucketed_out);

    bfly_vector_samples[sample] = timer.stop();

    bfly_vector_correct = bfly_vector_correct &&
      stapl::map_reduce(
        check_bfly_vector(nlocs, bucketed_out.get_location_id()),
        stapl::logical_and<bool>(), bucketed_out);


    // Butterfly to merge scalars across locations.  Each location goes from a
    // scalar equal to the location id to a scalar equal to the sum of all
    // location ids
    stapl::map_func(assign_id(lid), b);

    timer.reset();
    timer.start();

    typedef stapl::skeletons::spans::nearest_pow_two<
      stapl::skeletons::spans::balanced<>> bfly_span_t;

    stapl::skeletons::execute(stapl::skeletons::default_execution_params(),
      stapl::skeletons::sink<data_t>(
        stapl::skeletons::butterfly<true, stapl::use_default, bfly_span_t>
          (scalar_bucket_merge<data_t>())),
      b, b);

    bfly_scalar_samples[sample] = timer.stop();

    bfly_scalar_correct = bfly_scalar_correct &&
      stapl::map_reduce(
        check_bfly_scalar(nlocs, b.get_location_id()),
        stapl::logical_and<bool>(), b);

    stapl::rmi_fence();
  }

  report_result("b_map_func","STAPL",map_correct, map_samples);
  report_result("b_map_reduce","STAPL",map_reduce_correct, map_reduce_samples);
  report_result("b_reduce","STAPL",reduce_correct, reduce_samples);
  report_result("b_scan","STAPL",scan_correct, scan_samples);
  report_result("b_butterfly_scalar","STAPL",bfly_scalar_correct,
    bfly_scalar_samples);
  report_result("b_butterfly_vector","STAPL",bfly_vector_correct,
    bfly_vector_samples);

  return EXIT_SUCCESS;
}
