/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef BENCHMARKS_ALGORITHMS_UTILITIES_HPP
#define BENCHMARKS_ALGORITHMS_UTILITIES_HPP

#include <random>
#include "../utilities/confint.hpp"


#ifndef DATATYPE
using data_t = double;
const data_t data_min = 0.;
const data_t data_max = 1.;
const data_t data_val(3.14159265359);
#else
using data_t = DATATYPE;

#if defined(DATATYPE_MIN) && defined(DATATYPE_MAX) && defined(DATATYPE_FIND_VAL)
const data_t data_min = DATATYPE_MIN;
const data_t data_max = DATATYPE_MAX;
const data_t data_val(DATATYPE_FIND_VAL);
#else
#error "Must define DATATYPE_MIN, DATATYPE_MAX, DATATYPE_FIND_VAL"
#endif

#endif //DATATYPE

#ifdef _STAPL
#include <stapl/array.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/sorting.hpp>
#endif

#ifdef _GLIBCXX_PARALLEL
#include <vector>
#include <parallel/algorithm>
#else
#include <vector>
#include <algorithm>
#endif

#ifdef _STAPL
typedef stapl::array<data_t,
        stapl::view_based_partition<stapl::distribution_spec<>>,
        stapl::view_based_mapper<stapl::distribution_spec<>>
> array_type;

typedef stapl::array_view<array_type> array_type_vw;
#endif

template <typename T>
void fill(std::vector<T>& v, T&& value)
{
#ifdef _GLIBCXX_PARALLEL
  __gnu_parallel::for_each(v.begin(), v.end(), [value](T& n){ n = value; });
#else
  std::fill(v.begin(), v.end(), value);
#endif
}

#ifdef _STAPL
template <typename View>
void fill(View& v, typename View::value_type& value)
{
  stapl::fill(v, value);
}
#endif

struct rand_wf
{
private:
  using random_distribution_t = std::uniform_real_distribution<data_t>;

  std::mt19937 m_rng;
  double       m_duplicate_ratio;
  size_t       m_num_dupes;
  data_t       m_prev_value;
  bool         m_uniform;

  data_t gen_value(size_t index)
  {
    if (m_uniform)
    {
      // If duplicates are uniform, and the value is greater than the
      // duplicate fraction, get a new value
      if (random_distribution_t(data_min, data_max)(m_rng) > m_duplicate_ratio)
        m_prev_value = random_distribution_t(data_min, data_max)(m_rng);
    } else {
      // If duplicates are blocked, generate value for elements beyond num_dupes
      if (index >= m_num_dupes)
        m_prev_value = random_distribution_t(data_min, data_max)(m_rng);
    }
    return m_prev_value;
  }

public:
  rand_wf() = default;

  rand_wf(size_t seed)
    : m_rng(seed), m_duplicate_ratio(0.), m_num_dupes(0), m_uniform(true)
  { }

  rand_wf(double duplicate_ratio, size_t container_size, bool uniform)
#ifdef _STAPL
    : m_rng(stapl::get_location_id()*1234),
#else
    : m_rng(1234),
#endif
      m_duplicate_ratio(duplicate_ratio),
      m_num_dupes(duplicate_ratio * container_size), m_uniform(uniform)
  { m_prev_value = random_distribution_t(data_min, data_max)(m_rng); }

  // Interface used in most cases
  data_t operator()()
  { return random_distribution_t(data_min, data_max)(m_rng); }

  // Interface used to restore values
  template<typename NumRef>
#ifdef _STAPL
  typename std::enable_if<!stapl::is_view<typename std::decay<NumRef>::type>::value>::type
#else
  void
#endif
  operator()(NumRef&& num)
  { num = random_distribution_t(data_min, data_max)(m_rng); }

  // Interface used for weak-scaling experiments
  template<typename View, typename IndexView>
  void operator()(View&& v, IndexView&& index)
  {
    auto end_it = v.end();
    auto i      = index.begin();
    for (auto it = v.begin(); it != end_it; ++it, ++i)
      *it = gen_value(*i);
  }

  // Interface used for sequential initialization of gnu_parallel container
  template<typename T>
  void operator()(std::vector<T>& v)
  {
    size_t i = 0;
    for (auto& element : v)
    {
      element = gen_value(i);
      ++i;
    }
  }

  // Interface used for sequential initialization of STAPL view
  template<typename View>
#ifdef _STAPL
  typename std::enable_if<stapl::is_view<View>::value>::type
#else
  void
#endif
  operator()(View& v)
  {
    size_t j = 0;
    typename View::size_type last = v.domain().last();
    for (typename View::size_type i = v.domain().first(); i <= last; ++i)
    {
      v.set_element(i, gen_value(j));
      ++j;
    }
  }

#ifdef _STAPL
  void define_type(stapl::typer& t)
  {
    stapl::abort("Attempt to serialize rand_wf. std::mt19937 not serializable");
  }
#endif
};

//////////////////////////////////////////////////////////////////////
/// @brief Fill the vector provided with random values
/// @param v container to populate
/// @param duplicate_ratio specifies the fraction of duplicate elements
///   to insert in the container
/// @param uniform specifies whether the duplicates should be uniformly
///   distributed through the input (true), or blocked together (false)
///
/// Duplicate elements are not inserted into a single contiguous block.
/// Instead the previously generated value is reused if the new value
/// is lower than the ratio.  This gives us the desired effect because we
/// use a uniform distribution.
//////////////////////////////////////////////////////////////////////
template <typename T>
void fill_random(std::vector<T>& v, double duplicate_ratio = 0., bool uniform = true)
{
  rand_wf gen(duplicate_ratio, v.size(), uniform);
  gen(v);
}

#ifdef _STAPL
//////////////////////////////////////////////////////////////////////
/// @brief Fill the view provided with random values
/// @param v view to populate
/// @param weak_scaling specifies if scaling is a weak scaling experiment,
///   in which case the data is generated in parallel
/// @param duplicate_ratio specifies the fraction of duplicate elements
///   to insert in the container
/// @param uniform specifies whether the duplicates should be uniformly
///   distributed through the input (true), or blocked together (false)
///
/// Duplicate elements are not inserted into a single contiguous block.
/// Instead the previously generated value is reused if the new value
/// is lower than the ratio.  This gives us the desired effect because we
/// use a uniform distribution.
//////////////////////////////////////////////////////////////////////
template <typename View>
void fill_random(View& v, bool weak_scaling = false,
                 double duplicate_ratio = 0., bool uniform = true)
{
  if (!weak_scaling) {
    stapl::do_once([&v, duplicate_ratio, uniform]{
      rand_wf gen(duplicate_ratio, v.size(), uniform);
      gen(v);
    });
  } else {
    stapl::map_func<stapl::skeletons::tags::with_coarsened_wf>(
      rand_wf(duplicate_ratio, v.size(), uniform),
      v, stapl::counting_view<size_t>(v.size()));
  }
}
#endif

template <typename T>
void set_known_value(std::vector<T>& v, size_t idx)
{ v[idx] = data_val; }

template <typename T>
void clear_known_value(std::vector<T>& v, size_t idx, size_t num = 1)
{
  rand_wf rwf(idx);
  num += idx;
  for (; idx != num; ++idx)
    v[idx] = rwf();
}


struct equal_known
{
private:
  data_t m_known;

public:
  equal_known(data_t const& known)
    : m_known(known)
  { }

  template <typename T>
  bool operator()(T&& value) const
  { return value == m_known; }

#ifdef _STAPL
  void define_type(stapl::typer& t)
  { t.member(m_known); }
#endif
};


#ifdef _STAPL
template <typename View>
void set_known_value(View& v, size_t idx)
{ stapl::do_once([&v,idx]{ v.set_element(idx, data_val); }); }

template <typename View>
void clear_known_value(View& v, size_t idx, size_t num = 1)
{
  stapl::do_once([&v,&idx,&num]{
    rand_wf rwf(idx);
    num += idx;
    for (; idx != num; ++idx)
      v.set_element(idx, rwf());
  });
}
#endif

//////////////////////////////////////////////////////////////////////
/// @brief Check the result returned against the expected value given
///   the size of the container.
/// @param result Value returned from algorithm
/// @param size   Size of the container passed to the algorithm
/// @param expected_mean expected value of the mean given a set of
///   doubles from a uniform random distribution [0, 1.0].
/// @return Whether the result obtained is within epsilon of the
///   expected mean
///
/// If the max value of the distribution is greater than 1 then,
/// regardless of whether the algorithm called is accumulate or inner product,
/// the mean will be the midpoint of the range from min to max.  If the
/// range is between 0 and 1 then the mean will be around 0.3.
//////////////////////////////////////////////////////////////////////
template <typename T>
bool check_numeric_result(T& result, size_t size, double expected_mean)
{
  auto mean = result / size;

  auto range_mean = (data_max - data_min)/2;
  if (data_max > 1.0)
    return std::abs(mean - range_mean) < 0.001;
  else
    return std::abs(mean - (data_max - data_min)*expected_mean) < 0.001;
}

void report_result(std::string name, std::string version, bool correct,
                   std::vector<double> samples)
{
  std::string result = correct == true ? "Pass" : "FAIL";

  auto stats = compute_stats(samples);
#ifdef _STAPL
  stapl::do_once([&stats,&name,&version,&result]{
#endif
    std::cerr << "Test : " << name << "\n"
              << "Version : " << version << "\n"
              << "Status : " << result << "\n"
              << "Time : " << stats.avg << "\n"
              << "Note : conf int " << stats.conf_interval
              << " min " << stats.min << " max " << stats.max
              << " stddev " << stats.stddev << " samples " << stats.num_samples
              << "\n\n";
#ifdef _STAPL
  });
#endif
}
#endif //BENCHMARKS_ALGORITHMS_UTILITIES_HPP
