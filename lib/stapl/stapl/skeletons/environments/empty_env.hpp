/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_ENVIRONMENTS_EMPTY_ENV_HPP
#define STAPL_SKELETONS_ENVIRONMENTS_EMPTY_ENV_HPP

namespace stapl {
namespace skeletons {

//////////////////////////////////////////////////////////////////////
/// @brief An @c empty_env is a dummy environment which ignores all
/// the requests sent to it.
///
/// This environment is used to simplify the default cases in the
/// @c skeleton_manager.
///
/// @ingroup skeletonsEnvironments
//////////////////////////////////////////////////////////////////////
class empty_env
{
  std::size_t                      m_num_PEs;
  runtime::location_id             m_PE_id;
public:
  std::size_t get_num_PEs() const
  {
    return m_num_PEs;
  }

  std::size_t get_PE_id() const
  {
    return m_PE_id;
  }

  template <typename... Args>
  void pre_spawn(Args&&... args) const
  { }

  template <typename... Args>
  void post_spawn(Args&&... args) const
  { }

  void init_location_info(std::size_t num_PEs, runtime::location_id PE_id)
  {
    m_num_PEs = num_PEs;
    m_PE_id = PE_id;
  }

  template <bool isResult, typename... Args>
  void spawn_element(Args&&... args) const
  { }

  template <typename... Args>
  void set_num_succs(Args&&... args) const
  { }

  void define_type(typer& t)
  {
    t.member(m_num_PEs);
    t.member(m_PE_id);
  }
};

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_ENVIRONMENTS_EMPTY_ENV_HPP
