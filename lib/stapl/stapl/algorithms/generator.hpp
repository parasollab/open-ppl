/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_ALGORITHMS_GENERATOR_HPP
#define STAPL_ALGORITHMS_GENERATOR_HPP

#include <type_traits>
#include <random>

#include <boost/mpl/has_xxx.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int_distribution.hpp>

namespace stapl {

namespace generator_impl {

BOOST_MPL_HAS_XXX_TRAIT_DEF(state_type)

//////////////////////////////////////////////////////////////////////
/// @brief Static polymorphic functor that either returns a provided
/// generator or adjusts its state for a given starting offset through
/// the offsetting constructor interface.
//////////////////////////////////////////////////////////////////////
template<typename Index, typename Generator, bool HasState>
struct state_adjust_impl
{
  static Generator apply(Generator const& gen, Index const&)
  {
    return gen;
  };
};


template<typename Index, typename Generator>
struct state_adjust_impl<Index, Generator, true>
{
  static Generator apply(Generator const& gen, Index const& offset)
  {
    return Generator(gen, offset);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Wrapper around @ref state_adjust_impl that detects presence of
/// @p state_type nested typename and either forwards its corresponding boolean
/// value to @ref state_adjust_impl or @p false if the typename does not exist.
//////////////////////////////////////////////////////////////////////
template<typename Index, typename Generator,
         bool = has_state_type<Generator>::value>
struct state_adjust
  : state_adjust_impl<Index, Generator, false>
{ };


template<typename Index, typename Generator>
struct state_adjust<Index, Generator, true>
  : state_adjust_impl<Index, Generator, Generator::state_type::value>
{ };

} // namespace generator_impl


//////////////////////////////////////////////////////////////////////
/// @brief Wrapper around a generator functor that calls the generator's
///   constructor to update its state to a specified offset, if the generator
///   has implemented the method.
///
/// This is used in the @ref generate() pAlgorithm to support generators that
/// produce values that vary based on the number of times the generator's
/// function operator has been invoked.
//////////////////////////////////////////////////////////////////////
template<typename Index, typename Value, typename Generator>
class offset_gen
{
private:
  Generator m_gen;

public:
  using index_type  = Index;
  using result_type = Value;

  offset_gen(Generator gen)
    : m_gen(std::move(gen))
  { }

  result_type operator()(index_type const& i) const
  {
    return generator_impl::state_adjust<Index, Generator>::apply(m_gen, i)();
  }

  void define_type(typer& t)
  {
    t.member(m_gen);
  }
}; // struct offset_gen


//////////////////////////////////////////////////////////////////////
/// @brief Generates a sequence of consecutive values.
/// @tparam T The value type of the elements generated.
//////////////////////////////////////////////////////////////////////
template<typename T>
class sequence
{
private:
  T counter, oldCtr, step;

public:
  using state_type = std::true_type;

  sequence(T start = 0, T st = 1)
    : counter(start), oldCtr(start), step(st)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a functor that is initialized to generate elements
  ///   beginning at the offset position in the sequence.
  /// @param seq The sequence functor is passed in to allow offset-independent
  ///   state to be copied.
  /// @param offset The offset to be copied.
  //////////////////////////////////////////////////////////////////////
  sequence(sequence const& seq, std::size_t offset)
    : counter(seq.counter+offset * seq.step), oldCtr(counter), step(seq.step)
  { }

  void define_type(typer& t)
  {
    t.member(counter);
    t.member(oldCtr);
    t.member(step);
  }

  T operator()(void)
  {
    oldCtr = counter;
    counter += step;
    return oldCtr;
  }
}; // class sequence


//////////////////////////////////////////////////////////////////////
/// @brief Generates a sequence of random values.
///
/// @bug This is hacked to work deterministically with the testing framework
///   still doesn't work for parallel generation, but guarantees that two
///   successive sequentially generated sequences are the same.  Since functors
///   are both created before either is used.  call srand() once iteration
///   via operator() has begun.
//////////////////////////////////////////////////////////////////////
class random_sequence
{
private:
  int m_r;
  int m_offset;
  std::mt19937 m_gen;

public:
  using state_type = std::true_type;

  random_sequence(void)
    : m_r(RAND_MAX), m_offset(0), m_gen(std::random_device()())
  { }

  random_sequence(int i)
    : m_r(i), m_offset(0), m_gen(std::random_device()())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a functor that is initialized to generate elements
  ///   beginning at the offset position in the sequence.
  /// @param seq The sequence functor is passed in to allow offset-independent
  ///   state to be copied.
  /// @param offset The offset to be copied.
  //////////////////////////////////////////////////////////////////////
  random_sequence(random_sequence const& seq, std::size_t offset)
    : m_r(seq.m_r), m_offset(offset)
  { }

  void define_type(typer& t)
  {
    t.member(m_r);
    t.member(m_offset);
    t.member(m_gen);
  }

  int operator()(void)
  { return std::uniform_int_distribution<int>(0, m_r)(m_gen); }
}; // class random_sequence


//////////////////////////////////////////////////////////////////////
/// @brief Generates a sequence of std::pair<K, V> elements.
/// @tparam KeyGenerator    Key's generator type.
/// @tparam ValueGenerator  Value's generator type.
/// @tparam Key             Key's value type.
/// @tparam Value           Value's value type.
//////////////////////////////////////////////////////////////////////
template<typename KeyGenerator, typename ValueGenerator,
         typename Key, typename Value>
class associative_sequence
{
private:
  using result_type = std::pair<Key, Value>;

  KeyGenerator   m_key_gen;
  ValueGenerator m_value_gen;
  std::size_t    m_offset;

public:
  using state_type = std::true_type;

  associative_sequence(KeyGenerator& kg, ValueGenerator& vg)
    : m_key_gen(kg), m_value_gen(vg), m_offset(0)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a functor that is initialized to generate elements
  ///   beginning at the offset position in the sequence.
  /// @param seq The sequence functor is passed in to allow offset-independent
  ///   state to be copied.
  /// @param offset The offset to be copied.
  //////////////////////////////////////////////////////////////////////
  associative_sequence(associative_sequence const& seq, std::size_t offset)
    : m_key_gen(seq.m_key_gen, offset), m_value_gen(seq.m_value_gen, offset),
      m_offset(offset)
  { }

  result_type operator()(void)
  {
    return std::make_pair(m_key_gen(), m_value_gen());
  }

  void define_type(typer& t)
  {
    t.member(m_offset);
    t.member(m_key_gen);
    t.member(m_value_gen);
  }
}; // class associative_sequence


//////////////////////////////////////////////////////////////////////
/// @brief Generates a repeating sequence of values.
/// @tparam T Value type of the used counter for the sequence.
///
/// This differs from the repetitive_sequence by providing the start and end
/// values of the sequence to be repeated while repetitive_sequence accepts the
/// start value and the number of elements before the repeat occurs.
//////////////////////////////////////////////////////////////////////
template<typename T>
class block_sequence
{
private:
  T init, counter, end_counter, step;

public:
  using state_type = std::true_type;

  block_sequence(T start, T end, T st = 1)
    : init(start), counter(start), end_counter(end), step(st)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a functor that is initialized to generate elements
  ///   beginning at the offset position in the sequence.
  /// @param seq The sequence functor is passed in to allow offset-independent
  ///   state to be copied.
  /// @param offset The offset to be copied.
  //////////////////////////////////////////////////////////////////////
  block_sequence(block_sequence const& seq, std::size_t offset)
    : init(seq.init),
      counter(init + (offset % (seq.end_counter - init + 1))),
      end_counter(seq.end_counter), step(seq.step)
  { }

  void define_type(typer &t)
  {
    t.member(init);
    t.member(counter);
    t.member(end_counter);
    t.member(step);
  }

  T operator()(void)
  {
    T oldCtr = counter;
    counter  = (counter <= (end_counter-step) ? counter + step : init);
    return oldCtr;
  }
}; // class block_sequence


//////////////////////////////////////////////////////////////////////
/// @brief Generates a sequence of unique random sorted values.
//////////////////////////////////////////////////////////////////////
class unique_sorted_sequence
{
private:
  using result_type = int;

  std::vector<result_type> m_data;
  std::size_t              m_offset;

  //////////////////////////////////////////////////////////////////////
  /// @brief Functor that binds a random number generator and a
  /// distribution together.
  ///
  /// Boost.Random doesn't provide this interface, and it is needed to
  /// generate a sequence of random numbers using std::generate.
  //////////////////////////////////////////////////////////////////////
  struct rand_wrapper
  {
    boost::random::mt19937 m_gen;
    boost::random::uniform_int_distribution<int> m_dist;

    // the seed used for the generator is a literal to ensure all locations
    // generate the same sequence.
    rand_wrapper()
      : m_gen(0),
        m_dist(std::numeric_limits<int>::min(), std::numeric_limits<int>::max())
    { }

    int operator()()
    {
      return m_dist(m_gen);
    }
  };

public:
  using state_type = std::true_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Generates the values in its constructor.
  //////////////////////////////////////////////////////////////////////
  template<typename Compare>
  unique_sorted_sequence(std::size_t n, Compare comp)
    : m_data(n), m_offset(0)
  {
    rand_wrapper rand_wrap;
    std::generate(m_data.begin(), m_data.end(), rand_wrap);

    // generate the # of missing data until the array is full
    std::size_t nb_unique = 0;
    do {
      std::sort(m_data.begin(), m_data.end(), comp);

      nb_unique = std::distance(m_data.begin(),
                         std::unique(m_data.begin() + nb_unique, m_data.end()));
      std::generate(m_data.begin() + nb_unique, m_data.end(), rand);
    } while (nb_unique != n);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a functor that is initialized to generate elements
  ///   beginning at the offset position in the sequence.
  /// @param seq The sequence functor is passed in to allow offset-independent
  ///   state to be copied.
  /// @param offset The offset to be copied.
  //////////////////////////////////////////////////////////////////////
  unique_sorted_sequence(unique_sorted_sequence const& seq, std::size_t offset)
    : m_data(seq.m_data), m_offset(offset)
  { }

  result_type operator()(void)
  {
    return m_data[m_offset++];
  }

  void define_type(typer& t)
  {
    t.member(m_data);
    t.member(m_offset);
  }
}; // class unique_sorted_sequence


//////////////////////////////////////////////////////////////////////
/// @brief Functor to generate a sequence of default-constructed values.
/// @tparam T Returned value type for the sequence.
//////////////////////////////////////////////////////////////////////
template<typename T>
struct null_sequence
{
  T operator()(void) const { return T(); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor to generate a sequence of default-constructed values.
/// @tparam int Returned value type for the sequence.
//////////////////////////////////////////////////////////////////////
template<>
struct null_sequence<int>
{
  int operator()(void) const { return 0; }
};


//////////////////////////////////////////////////////////////////////
/// @brief A sequence that repeats itself every 'repeat' elements.
/// @tparam T value type of the used counter for the sequence.
///
/// This differs from the block_sequence by providing the start value and the
/// number of elements before the repeat occurs while block_sequence accepts the
/// start and end values of the sequence to be repeated.
//////////////////////////////////////////////////////////////////////
template<typename T>
class repetitive_sequence
{
private:
  T counter, step, init;
  size_t el, restart_el;

public:
  using state_type = std::true_type;

  repetitive_sequence(T start, T st, size_t repeat)
    : counter(start), step(st), init(start), el(0), restart_el(repeat)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a functor that is initialized to generate elements
  ///   beginning at the offset position in the sequence.
  /// @param seq The sequence functor is passed in to allow offset-independent
  ///   state to be copied.
  /// @param offset The offset to be copied.
  //////////////////////////////////////////////////////////////////////
  repetitive_sequence(repetitive_sequence const& seq, const std::size_t offset)
    : counter(seq.init + seq.step * offset % seq.restart_el),
      step(seq.step),
      init(seq.init),
      el(offset % seq.restart_el),
      restart_el(seq.restart_el)
  { }

  void define_type(typer& t)
  {
    t.member(counter);
    t.member(step);
    t.member(init);
    t.member(el);
    t.member(restart_el);
  }

  T operator()(void)
  {
    if (el==restart_el)
    {
      el = 0;
      counter = init;
    }

    T oldCtr = counter;
    counter += step;
    ++el;
    return oldCtr;
  }
}; // class repetitive_sequence

//////////////////////////////////////////////////////////////////////
/// @brief Generates a pattern of values that ranges from start to end by
/// step, then from end to start by -step. This pattern is repeated. The
/// start and end values can be repeated, or not, as specified.
/// Example sequence: 0 1 2 3 4 3 2 1 0 1 2 3 4 3 2 1 0...     (no repeat)
/// Example sequence: 0 1 2 3 4 4 3 2 1 0 1 2 3 4 4 3 2 1 0... (repeat end)
///
/// @tparam T Value type of the used counter for the sequence.
/// @param start The starting value to generate the sequence.
/// @param end The end value to generate the sequence.
/// @param step The increment to increase or decrease the value by while
/// moving from start to stop or stop to start, respectively.
/// @param repeat Whether to repeat the start or stop values when they are
/// reached. Provided as an enumeration wave_sequence::None, Start, Stop, Both
///
/// @example wave_sequence<int>(0, 4, 1, wave_sequence<int>::None)
/// @example wave_sequence<int>(0, 4, 1, wave_sequence<int>::Stop)
///
///
/// This differs from the repetitive_sequence by providing the start and end
/// values of the sequence to be repeated while repetitive_sequence accepts the
/// start value and the number of elements before the repeat occurs.
//////////////////////////////////////////////////////////////////////
template<typename T>
class wave_sequence
{
  public:
  // Used to specify that the start value, stop value, or both are repeated
  // when the step direction changes.
  enum Repeat
  {
    None, Start, Stop, Both
  };

  private:
  T init, counter, end_counter, step;
  bool repeatStart=false;
  bool repeatStop=false;
  bool repeated=true; // Have we repeated whatever end we are now at?

  public:
  using state_type = std::true_type;

  wave_sequence(T start, T end, T st = 1, Repeat r = Repeat::None)
    : init(start), counter(start+st), end_counter(end), step(-st)
  {
    if (r== Start || r==Both)
      repeatStart = true;
    if (r== Stop || r==Both)
      repeatStop  = true;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a functor that is initialized to generate elements
  ///   beginning at the offset position in the sequence.
  /// @param seq The sequence functor is passed in to allow offset-independent
  ///   state to be copied.
  /// @param offset The offset to be copied.
  //////////////////////////////////////////////////////////////////////
  wave_sequence(wave_sequence const& seq, std::size_t offset)
    : init(seq.init),
      counter(init + (offset % (seq.end_counter - init + 1))),
      end_counter(seq.end_counter), step(seq.step),
      repeatStart(seq.repeatStart), repeatStop(seq.repeatStop),
      repeated(seq.repeated)
  { }

  void define_type(typer &t)
  {
    t.member(init);
    t.member(counter);
    t.member(end_counter);
    t.member(step);
    t.member(repeatStart);
    t.member(repeatStop);
    t.member(repeated);
  }

  T operator()(void)
  {
    if (counter == end_counter)
    {
      if (repeatStop && !repeated)
      {
        repeated=true; // This value is a repeat.
      }
      else
      {
        step = -step; // Reverse our stepping direction.
        repeated = false; // Not a repeated value.
        counter += step;
      }
    }
    else if (counter == init)
    {
      if (repeatStart && !repeated)
      {
        repeated=true; // This value is a repeat.
      }
      else
      {
        step = -step; // Reverse our stepping direction.
        repeated = false; // Not a repeated value.
        counter += step;
      }
    }
    else
    {
      counter += step;
    }
    return counter;
  }
}; // class wave_sequence


} // stapl namespace

#endif

