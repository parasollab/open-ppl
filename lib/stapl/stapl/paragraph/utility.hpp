/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_PARAGRAPH_UTILITY_HPP
#define STAPL_PARAGRAPH_UTILITY_HPP

#include <type_traits>

#include <boost/mpl/has_xxx.hpp>
#include <boost/utility/result_of.hpp>

#include <stapl/paragraph/task_factory_base.hpp>
#include <stapl/utility/use_default.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief This code encapsulates a view index (previously call cid) used
///   in cases when computation has been transparently replicated on portions of
///   the system to avoid unnecessary communication.
/// @ingroup pgUtility
///
/// @todo This code isn't used in trunk.  It was used in experimental branch
/// for NAS CG.  Investigate whether we want this feature on trunk.  If not,
/// remove this class.
//////////////////////////////////////////////////////////////////////
struct replicated_cid
{
private:
  /// @brief The logical view index this class represents.
  size_t m_idx;

  /// @brief The portion of the system performing replicated computation that
  /// this cid is associated it.  Determines what location it should request
  /// data from.
  size_t m_replicate_set;

public:
  replicated_cid(size_t idx, size_t replicate_set)
    : m_idx(idx), m_replicate_set(replicate_set)
  { }

  size_t index(void) const
  {
    return m_idx;
  }

  size_t replicate_set(void) const
  {
    return m_replicate_set;
  }

  operator size_t() const
  {
    return m_idx;
  }

  void define_type(typer &t)
  {
    t.member(m_idx);
    t.member(m_replicate_set);
  }
};


namespace paragraph_impl {

using boost::result_of;


//////////////////////////////////////////////////////////////////////
/// @brief Trivially wraps a workfunction.
/// @ingroup pgUtility
///
/// @tparam WF Workfunction type to wrap, hold instance as private base.
///
/// This functor inherits the behavior of its template parameter and
/// exists solely to quiet gcc warning about ambiguous bases in the
/// presence of multiple inheritance (done for the empty base class
/// optimization.
///
/// @todo Verify that newer gcc still warns without this code.
//////////////////////////////////////////////////////////////////////
template<typename WF>
struct identity_wf
  : private WF // inherited to employ empty base optimization.
{

  explicit
  identity_wf(WF const& freduce)
    : WF(freduce)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Nested result struct to compute return type. Redirect to
  /// any definition in @p WF.
  //////////////////////////////////////////////////////////////////////
  template<typename Signature>
  struct result;

  template<typename Args>
  struct result<identity_wf(Args)>
    : public result_of<WF(Args)>
  { };


  //////////////////////////////////////////////////////////////////////
  /// @brief Return reference to wrapped workfunction instance.
  //////////////////////////////////////////////////////////////////////
  WF& wf()
  {
    return *this;
  }


  void define_type(typer& t)
  {
    t.base<WF>(*this);
  }
};


BOOST_MPL_HAS_XXX_TRAIT_DEF(result_type)
BOOST_MPL_HAS_XXX_TRAIT_DEF(task_id_mapper_type)
BOOST_MPL_HAS_XXX_TRAIT_DEF(coarsener_type)
BOOST_MPL_HAS_XXX_TRAIT_DEF(tag_type)
BOOST_MPL_HAS_XXX_TRAIT_DEF(enable_persistence)


//////////////////////////////////////////////////////////////////////
/// @brief Metafunction that checks if @c enable_migration is defined for @c T.
/// @ingroup pgUtility
///
/// Extends from @c false_type by default.
///
/// @todo Similar to Boost.MPL.has_xxx. Maybe replace it with that.
//////////////////////////////////////////////////////////////////////
template<typename T, typename Enable = void>
struct has_enable_migration
  : public std::false_type
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Metafunction that checks if @c enable_migration is defined for @c T.
/// @ingroup pgUtility
///
/// Extends from @c true_type for @c T that has @c T::enable_migration.
//////////////////////////////////////////////////////////////////////
template<typename T>
struct has_enable_migration<T, typename T::enable_migration>
  : public std::true_type
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Metafunction that computes the tag type for a given
/// @c Factory.
/// @ingroup pgUtility
//////////////////////////////////////////////////////////////////////
template <typename Factory, bool = has_tag_type<Factory>::value>
struct tag_type
{
  using type = typename Factory::tag_type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Metafunction that computes the tag type for a given
/// @c Factory for the case that tag_type is not defined by the Factory.
/// @ingroup pgUtility
//////////////////////////////////////////////////////////////////////
template <typename Factory>
struct tag_type<Factory, false>
{
  using type = stapl::use_default;
};

} // namespace paragraph_impl


//////////////////////////////////////////////////////////////////////
/// @brief Boolean type metafunction to detect if template parameter @p WF is
///   a factory.
/// @ingroup pgUtility
//////////////////////////////////////////////////////////////////////
template<typename WF>
struct is_factory
  : public std::is_base_of<factory_wf, WF>::type
{ };

} // namespace stapl

#endif // STAPL_PARAGRAPH_UTILITY_HPP

