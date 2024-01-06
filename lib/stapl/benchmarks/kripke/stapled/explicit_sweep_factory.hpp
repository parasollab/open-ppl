/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARKS_KRIPKE_EXPLICIT_SWEEP_FACTORY_HPP
#define STAPL_BENCHMARKS_KRIPKE_EXPLICIT_SWEEP_FACTORY_HPP

#include <map>
#include <memory>
#include <vector>
#include <stapl/paragraph/task_factory_base.hpp>
#include <stapl/runtime/runtime.hpp>
#include "Grid.h"
#include "Kripke/Directions.h"


// work function to generate boundary input
// The problem boundary is initialized to 1.0
struct gen_bdry_ones
{
private:
  // number of zones in the first dimension of plane
  int m_dim1;

  // number of zones in the second dimension of plane
  int m_dim2;

  // product of the number of groups and number of directions
  // in the groupset and direction set.
  int m_groups_dirs;

public:
  typedef std::vector<std::vector<double>> result_type;

  gen_bdry_ones(int dim1, int dim2, int groups_dirs)
    : m_dim1(dim1), m_dim2(dim2), m_groups_dirs(groups_dirs)
  { }

  result_type operator()(void)
  {
    return std::vector<std::vector<double>>(1,
             std::vector<double>(m_dim1 * m_dim2 * m_groups_dirs, 1.0));
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_dim1);
    t.member(m_dim2);
    t.member(m_groups_dirs);
  }
};


template <typename IncPsi>
struct filter_domain_result;

template <typename IncPsi>
inline bool operator==(filter_domain_result<IncPsi> const&,
                       filter_domain_result<IncPsi> const&);

template <typename IncPsi>
struct filter_domain_result
{
protected:
  std::size_t m_dimension;

  template <typename IncPsi1>
  friend bool operator==(filter_domain_result<IncPsi1> const&,
                         filter_domain_result<IncPsi1> const&);
public:
  typedef IncPsi result_type;

  filter_domain_result(int dimension)
    : m_dimension(dimension)
  {}

  void define_type(stapl::typer& t)
  { t.member(m_dimension); }

  IncPsi operator()(IncPsi const& domain_result) const
  {
    //Declaring explicit return variable to allow use of move semantics.
    IncPsi result(1);
    result[0] = (domain_result.size() != 1) ?
      std::move(const_cast<typename IncPsi::value_type&>(
        domain_result[m_dimension])) :
      std::move(const_cast<typename IncPsi::value_type&>(
        domain_result[0]));
    return result;
  }
};

template <typename IncPsi>
inline bool operator==(filter_domain_result<IncPsi> const& lh,
                       filter_domain_result<IncPsi> const& rh)
{ return lh.m_dimension == rh.m_dimension; }

struct sweep_tag { };


template <typename SweepMethod>
class explicit_sweep_factory
  : public stapl::task_factory_base
{
  Grid_Data*       m_grid;
  SweepMethod      m_kernel;

public:
  typedef sweep_tag tag_type;
  typedef stapl::false_ align_type;
  typedef void result_type;

  explicit_sweep_factory(Grid_Data* g, SweepMethod const& kernel)
    : m_grid(g), m_kernel(kernel)
  { }

  template <typename TaskGraphView, typename GridView>
  void operator()(TaskGraphView const& tgv, GridView& grid_view)
  {
    // Due to the lack of cellsets, each location will generate a task to
    // process its portion of the grid, which is represented by an element of
    // the STAPL native_view named grid_view.

    // A task processing a partition of the spatial domain on the problem
    // boundary will need to receive a face initialized to all ones for each
    // face on the boundary.  There could be up to four tasks then.  The task
    // id regime used is as follows
    // [0,p-1]   -- task creating i incoming boundary face
    // [p,2p-1]  -- task creating j incoming boundary face
    // [2p,3p-1] -- task creating k incoming boundary face
    // [3p,4p-1] -- task processing spatial domain

    typedef typename SweepMethod::result_type task_result;
    typedef filter_domain_result<task_result>  result_filter;

    // Get the sweep direction
    Directions sweep_dir =
      m_grid->gd_sets()[m_kernel.group_set][m_kernel.direction_set].directions[0];

    // The product of groups in the groupset and directions in the direction set
    int groups_dirs =
      m_grid->gd_sets()[m_kernel.group_set][m_kernel.direction_set].num_groups *
      m_grid->gd_sets()[m_kernel.group_set][m_kernel.direction_set].num_directions;

    // Get the number of zones in each dimension on this location.
    int local_imax = m_grid->nzones()[0];
    int local_jmax = m_grid->nzones()[1];
    int local_kmax = m_grid->nzones()[2];

    // Get the ids of the neighboring locations that will be predecessors in
    // the sweep.
    int ip = (sweep_dir.id>0) ? m_grid->mynbr()[0][0] : m_grid->mynbr()[0][1];
    int jp = (sweep_dir.jd>0) ? m_grid->mynbr()[1][0] : m_grid->mynbr()[1][1];
    int kp = (sweep_dir.kd>0) ? m_grid->mynbr()[2][0] : m_grid->mynbr()[2][1];

    // If upstream neighbor is problem boundary, add task to generate the face.
    if (ip == -1)
    {
      // i problem boundary predecessors in tid space [0,p-1]
      ip = grid_view.get_location_id();
      tgv.add_task(ip, gen_bdry_ones(local_jmax, local_kmax, groups_dirs), 1);
    }
    else
    {
      // i predecessor in tid space [3p,4p-1]
      ip += 3 * grid_view.get_num_locations();
    }

    if (jp == -1)
    {
      // j problem boundary predecessors in tid space [p,2p-1]
      jp = grid_view.get_location_id() + grid_view.get_num_locations();
      tgv.add_task(jp, gen_bdry_ones(local_imax, local_kmax, groups_dirs), 1);
    }
    else
    {
      // j predecessor in tid space [3p,4p-1]
      jp += 3 * grid_view.get_num_locations();
    }

    if (kp == -1)
    {
      // k problem boundary predecessors in tid space [2p,3p-1]
      kp = grid_view.get_location_id() + 2*grid_view.get_num_locations();
      tgv.add_task(kp, gen_bdry_ones(local_imax, local_jmax, groups_dirs), 1);
    }
    else
    {
      // k predecessor in tid space [3p,4p-1]
      kp += 3 * grid_view.get_num_locations();
    }

    // Get the successor locations in order to find the number of successors.
    int num_succs = 0;
    int is = (sweep_dir.id>0) ? m_grid->mynbr()[0][1] : m_grid->mynbr()[0][0];
    int js = (sweep_dir.jd>0) ? m_grid->mynbr()[1][1] : m_grid->mynbr()[1][0];
    int ks = (sweep_dir.kd>0) ? m_grid->mynbr()[2][1] : m_grid->mynbr()[2][0];

    num_succs += (is != -1) ? 1 : 0;
    num_succs += (js != -1) ? 1 : 0;
    num_succs += (ks != -1) ? 1 : 0;

    // Add the task to process the spatial domain.
    // We consume the results of the upstream tasks, filtering the appropriate
    // face out of the vector<vector<double>> result of each.
    // 0 selects i-dim, 1 selects j-dim, 2 selects k-dim.
    int tid = grid_view.get_location_id() + 3*grid_view.get_num_locations();
    tgv.add_task(tid, m_kernel, num_succs,
      stapl::localize_ref(grid_view, grid_view.get_location_id()),
      stapl::consume<task_result>(tgv, ip, result_filter(0)),
      stapl::consume<task_result>(tgv, jp, result_filter(1)),
      stapl::consume<task_result>(tgv, kp, result_filter(2)));
  }
};

#endif // STAPL_BENCHMARKS_KRIPKE_EXPLICIT_SWEEP_FACTORY_HPP
