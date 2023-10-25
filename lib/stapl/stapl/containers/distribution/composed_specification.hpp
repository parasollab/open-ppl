/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_DISTRIBUTION_COMPOSED_SPECIFICATION_HPP
#define STAPL_CONTAINERS_DISTRIBUTION_COMPOSED_SPECIFICATION_HPP

#include <stapl/containers/distribution/specifications_fwd.hpp>
#include <stapl/utility/integer_sequence.hpp>

namespace stapl {

class composed_dist_spec
  : public p_object
{
public:
  /// @brief used to differentiate between composed distribution specifications
  /// and composed size specifications in the @ref stapl::container
  /// constructors.
  typedef void is_composed_dist_spec;

  typedef stapl::distribution_spec<> distribution_spec;

  typedef std::function<distribution_spec (std::vector<size_t> const&)>
    generator_type;

private:
  std::vector<size_t> m_index;
  generator_type      m_gen;

public:
  /// @name Constructors
  /// @{

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor accepts a functor that generates the distribution
  /// specification of all elements in a nested container instance.
  ///
  /// The multi-dimensional index passed is a std::vector of size_t elements.
  /// An empty vector is used to indicate that the distribution specfication
  /// for the outer container is to be provided.  This allows the size of the
  /// vector to correspond to the nesting level being generated.
  ///
  /// @param gen_wf Functor that is passed the multi-dimensional index of
  /// the container for which it will generate a distribution specfication.
  //////////////////////////////////////////////////////////////////////
  composed_dist_spec(generator_type const& gen)
    : m_index(), m_gen(gen)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor invoked by operator[] to generate the composed
  /// specification for a nested container.
  /// @param other A reference to the parent composed specification.
  /// @param i The index of the element for which the composed specification
  /// is desired.
  //////////////////////////////////////////////////////////////////////
  composed_dist_spec(composed_dist_spec const& other, size_t i)
    : m_index(other.m_index), m_gen(other.m_gen)
  { m_index.push_back(i); }

  /// @}

  /// @name Query Methods
  /// @{

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the size of the container that would be constructed
  /// by the specification.
  //////////////////////////////////////////////////////////////////////
  size_t size(void) const
  { return m_gen(m_index).domain().size(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Conversion operator that allows extracting the specification
  /// of the outer container's distribution.
  ///
  /// The extraction is done by invoking the user-provided functor that
  /// generates distribution specifications given a vector that represents
  /// the multi-dimensional index of the nested container to be constructed.
  ///
  /// @return The distribution specification this composed specification
  /// represents.
  //////////////////////////////////////////////////////////////////////
  operator distribution_spec() const
  { return m_gen(m_index); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the specification of the outer container's distribution
  /// from the current composed specification.
  ///
  /// The extraction is done by invoking the user-provided functor that
  /// generates distribution specifications given a vector that represents
  /// the multi-dimensional index of the nested container to be constructed.
  ///
  /// @return The distribution specification this composed specification
  /// represents.
  //////////////////////////////////////////////////////////////////////
  distribution_spec spec(void) const
  { return m_gen(m_index); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the composed specification of the i-th element of the
  /// container.
  /// @param i The index of the element whose specification being requested.
  //////////////////////////////////////////////////////////////////////
  composed_dist_spec operator[](size_t i) const
  {
    return composed_dist_spec(*this, i);
  }

  /// @}
}; // class composed_dist_spec



//////////////////////////////////////////////////////////////////////
/// @brief Structure holding a composition of distribution specification
/// views, each of which whose specification type and gid type maybe
/// be different.
///
/// @tparam Generator The user supplied generator invoked with a
///   the current list of values from container nest traversal.
/// @tparam GIDList The types of GIDs so far in initializing traversal.
/// @tparam A list of specifications to be used in remaining levels
///   of current traversal.
//////////////////////////////////////////////////////////////////////
template<typename Generator, typename GIDList, typename SpecList,
        typename =
           make_index_sequence<tuple_size<GIDList>::value>,
         typename =
           make_index_sequence<
             tuple_size<SpecList>::value == 0 ?
               0 : tuple_size<SpecList>::value-1>>
class heterogeneous_composed_dist_spec
{ };


template<typename Generator,
         typename... GIDs, typename... DistributionSpecs,
         std::size_t... GIDIndices, std::size_t... SpecIndices>
class heterogeneous_composed_dist_spec<
  Generator, tuple<GIDs...>, tuple<DistributionSpecs...>,
  index_sequence<GIDIndices...>, index_sequence<SpecIndices...>>
  : public p_object
{
private:
  typedef tuple<GIDs...>              gids_list_t;
  typedef tuple<DistributionSpecs...> specs_list_t;

  Generator   m_gen;
  gids_list_t m_gids;

public:
  typedef void                                          is_composed_dist_spec;
  typedef typename tuple_element<0, specs_list_t>::type distribution_spec;

  STAPL_IMPORT_DTYPE(distribution_spec, gid_type)

  template<typename GeneratorParam, typename GIDsParam>
  heterogeneous_composed_dist_spec(GeneratorParam&& gen, GIDsParam&& gids)
    : m_gen(std::forward<GeneratorParam>(gen)),
      m_gids(std::forward<GIDsParam>(gids))
  { }


  //////////////////////////////////////////////////////////////////////
  /// @brief Return the distribution specification at the current level
  ///  of traversal by invoking the user generator and passing it the
  ///  the partial list of gids.
  //////////////////////////////////////////////////////////////////////
  distribution_spec spec(void) const
  {
    return m_gen(get<GIDIndices>(m_gids)...);
  }

private:
  typedef heterogeneous_composed_dist_spec<
    Generator,
    tuple<typename tuple_element<GIDIndices, gids_list_t>::type..., gid_type>,
    tuple<typename tuple_element<SpecIndices + 1, specs_list_t>::type...>
  > next_composed_dist_spec_type;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Return the composed distribution specification for the next
  ///  level of the container initialization for the given @p gid.
  //////////////////////////////////////////////////////////////////////
  next_composed_dist_spec_type
  operator[](gid_type const& gid) const
  {
    return next_composed_dist_spec_type(
      m_gen, std::make_tuple(get<GIDIndices>(m_gids)..., gid));
  }
}; // class heterogeneous_composed_dist_spec


//////////////////////////////////////////////////////////////////////
/// @brief Constructs a composed distribution specification that will use
/// the functor provided to generate the distribution specifications of
/// the elements of the composed container instance.
///
/// @param gen Functor whose function operator is passed the multi-dimensional
/// index of the container element for which a distribution specification
/// is needed.  Gid types must all be the same.  Hence the list is passed
/// as an std::vector<>.
//////////////////////////////////////////////////////////////////////
template<typename Generator>
composed_dist_spec make_composed_dist_spec(Generator const& gen)
{
  return composed_dist_spec(gen);
}


template<typename Generator>
composed_dist_spec* make_composed_dist_spec_ptr(Generator const& gen)
{
  return new composed_dist_spec(gen);
}


//////////////////////////////////////////////////////////////////////
/// @brief Constructs a composed distribution specification that will use
/// the functor provided to generate the distribution specifications of
/// the elements of the composed container instance.  The gid types of
/// the distribution need not be homogeneous.
///
/// @param gen Functor whose function operator is passed the series of
/// gids in the container nest as distinct parameters with (possibly)
/// differing types.
//////////////////////////////////////////////////////////////////////
template<typename Generator, typename ...DistributionSpecs>
heterogeneous_composed_dist_spec<
  Generator, tuple<>, tuple<DistributionSpecs...>>
make_heterogeneous_composed_dist_spec(Generator&& gen)
{
  return heterogeneous_composed_dist_spec<
    typename std::decay<Generator>::type,
    tuple<>, tuple<DistributionSpecs...>
  >(std::forward<Generator>(gen), std::make_tuple());
}

} // namespace stapl

#endif // STAPL_CONTAINERS_DISTRIBUTION_COMPOSED_SPECIFICATION_HPP
