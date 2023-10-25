/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_ENVIRONMENTS_DEBUG_ENV_HPP
#define STAPL_SKELETONS_ENVIRONMENTS_DEBUG_ENV_HPP

#include <ostream>
#include <stapl/skeletons/flows/producer_info.hpp>
#include <stapl/skeletons/utility/printers.hpp>

namespace stapl {
namespace skeletons {

//////////////////////////////////////////////////////////////////////
/// @brief This environment can be used to print the information
/// about the generated task graph on the screen using @ref std::cout.
////
/// Remember, in the cases that a dynamic (conditional ) dependence
/// graphs such as @c do_while is used, you have to use this
/// along with a real execution environment such as @c taskgraph_env
///
/// @see do_while
/// @see taskgraph_env
///
/// @ingroup skeletonsEnvironments
//////////////////////////////////////////////////////////////////////
class debug_env
{
  std::size_t                      m_num_PEs;
  runtime::location_id             m_PE_id;
public:
  debug_env()
    : m_num_PEs(-1),
      m_PE_id(-1)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Prints out the spawned node on the screen along with its
  /// corresponding dependencies information using @ref std::cout.
  ///
  /// @param tid       the unique id assigned to the node in the graph
  /// @param result_id result_id of this task if it has one.
  /// @param wf        the workfunction to be executed for this node
  /// @param mapper    the output to output mapper for mapping the results
  /// @param num_succs  the exact number of successors for this element.
  ///                  Remember that in some cases this value is set to
  ///                  @c stapl::defer_specs (when the @c spawner is in
  ///                  @c SET_HOLD mode). This number is shown in
  ///                  parenthesis in the output graph.
  /// @param in        the producer information for the node's input
  ///                  arguments that will be translated as edge in the
  ///                  output graph
  //////////////////////////////////////////////////////////////////////
  template <bool isResult, typename WF, typename Mapper, typename... In>
  void spawn_element(std::size_t tid, std::size_t result_id, WF&& wf,
                     Mapper&& mapper, std::size_t num_succs, In&&... in) const
  {
    spawn_element<isResult>(tid, result_id, std::vector<std::size_t>(),
                            std::forward<WF>(wf), std::forward<Mapper>(mapper),
                            num_succs, std::forward<In>(in)...);
  }

  template <bool isResult, typename WF, typename Mapper, typename... In>
  void spawn_element(std::size_t tid, std::size_t result_id,
                     std::vector<std::size_t> const& notifications, WF&& wf,
                     Mapper&&, std::size_t num_succs, In&&... in) const
  {
    std::cout << (isResult ? "Result-" : "");
    std::cout << "Task(" << tid << ") with " << num_succs << " successors "
              << " depends on [";
    print_dependencies(std::forward<In>(in)...);
    std::cout << "]";
    if (notifications.size()) {
      std::cout << " and is notified by Tasks[";
      bool is_first = true;
      for (auto&& v : notifications) {
        if (!is_first) {
          std::cout << ",";
        }
        else {
          is_first = false;
        }
        std::cout << v;
      }
      std::cout << "]";
    }
    std::cout << " with WF=";
    skeletons::show(wf, std::cout);
    std::cout << std::endl;
  }

  template <typename Skeleton, typename Coord, typename In, typename Out>
  void pre_spawn(Skeleton&& skeleton,
                 std::size_t lid_offset,
                 Coord&& skeleton_size, Coord&& coord,
                 In&& in, Out&& out,
                 std::size_t cur_stage = 0) const
  {
    std::cout << "...................................................";
    std::cout << "\nSpawning";
    std::cout << "\n|__ Skeleton    = ";
    show(skeleton, std::cout);
    std::cout << "\n|__ Input Flow  = ";
    show(in, std::cout);
    std::cout << "\n|__ Output Flow = ";
    show(out, std::cout);
    std::cout << "\n|__ Offset      = " << lid_offset;
    std::cout << "\n|__ Coord       = [";
    show_value(coord, std::cout);
    std::cout << "]";
    std::cout << "\n|__ Size        = [";
    show_value(skeleton_size, std::cout);
    std::cout << "]\n";
  }

  template <typename Skeleton, typename... Args>
  void post_spawn(Skeleton&& skeleton, Args&&... args) const
  {
    std::cout << "...................................................\n";
  }

  void set_num_succs(std::size_t tid, std::size_t num_succs) const
  {
    std::cout << "Task(" << tid << ") successors changed to " << num_succs
      << std::endl;
  }

  void init_location_info(std::size_t num_PEs, runtime::location_id PE_id)
  {
    m_num_PEs = num_PEs;
    m_PE_id = PE_id;
  }

  std::size_t get_num_PEs() const
  {
    return m_num_PEs;
  }

  runtime::location_id get_PE_id() const
  {
    return m_PE_id;
  }

private:
  template <typename View>
  void
  print_dependency(flows::view_element_producer<View> const& producer) const
  {
    skeletons::show(producer.get_element(), std::cout);
    std::cout << "[";
    show_value(producer.get_index());
    std::cout << "]";
  }

  template <typename Element>
  void
  print_dependency(flows::reflexive_producer<Element> const& producer) const
  {
    skeletons::show(producer.get_element(), std::cout);
  }

  template <typename T>
  void
  print_dependency(flows::constant_producer<T> const& producer) const
  {
    std::cout << "Constant(" << producer.get_element() << ")";
  }

  template <typename Producer>
  void
  print_dependency(Producer const& producer) const
  {
    std::cout << "Task(";
    show_value(producer.get_index());
    std::cout << ")";
  }

  template <typename In0>
  void print_dependencies(In0&& in0) const
  {
    print_dependency(std::forward<In0>(in0));
  }

  template <typename In0, typename ...In>
  void print_dependencies(In0&& in0, In&&... in) const
  {
    print_dependency(std::forward<In0>(in0));
    std::cout << ", ";
    print_dependencies(std::forward<In>(in)...);
  }
public:
  void define_type(typer& t)
  {
    t.member(m_num_PEs);
    t.member(m_PE_id);
  }
};


} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_ENVIRONMENTS_DEBUG_ENV_HPP
