/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_WF_H
#define STAPL_WF_H

#include "args.h"
#include "bound_box.h"
#include "build_tree.h"
#include "dataset.h"
#include "logger.h"
#include "traversal.h"
#include "sort.h"
#include "up_down_pass.h"
#include "verify.h"
#include "stapl_proxy.h"
using namespace stapl;

//////////////////////////////////////////////////////////////////////
/// @brief a class used for the reduction of min/max Bounds of the domain
//////////////////////////////////////////////////////////////////////
template <typename MinOp, typename MaxOp>
struct compute_global_bounds
{
  typedef Bounds result_type;

  MinOp m_min_op;
  MaxOp m_max_op;

  compute_global_bounds(MinOp const& min_op,
                        MaxOp const& max_op)
    : m_min_op(min_op),
      m_max_op(max_op)
  { }

  template <typename V1, typename V2>
  result_type operator()(V1 && v1, V2 && v2)
  {
     return find_extreme(v1,v2, m_min_op, m_max_op);
  }

  void define_type(typer& t)
  {
    t.member(m_min_op);
    t.member(m_max_op);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief a Wrapper class for finding the bounds at local partitions
//////////////////////////////////////////////////////////////////////
template<typename ContainerType>
struct find_bounds
{
  private:
    int nspawn;

  public:
    find_bounds(int a_nspawn)
      : nspawn(a_nspawn)
    { }

    typedef Bounds result_type;
    template <typename V>
    result_type operator()(V&& v)
    {
      ContainerType bodies = v;
      result_type bounds;
      BoundBox boundBox(nspawn);
      bounds = boundBox.getBounds(bodies);
      return bounds;
    }

    void define_type(typer& t)
    {
      t.member(nspawn);
    }
};


//////////////////////////////////////////////////////////////////////
/// @brief Class for placing the 1D data into 2D vector to be used for alltoall
//////////////////////////////////////////////////////////////////////
template <typename T>
struct prepare_for_sorted_all_to_all
{
private:
  int buffer_size;
  int rank;
public:
  prepare_for_sorted_all_to_all(int a_size, int a_rank)
    : buffer_size(a_size), rank(a_rank)
  { }

  typedef T result_type;
  template <typename VectorView, typename TreeView>
    result_type operator()(VectorView && view, TreeView& tree)
    {
      result_type v(buffer_size);
      size_t curr_rank = 0;
      size_t prev_rank = 0;
      size_t count = 0;
      auto&& it_end   = view.end();
      auto it_begin   = view.begin();
      for (auto it = it_begin; it != it_end; ++it)
      {
        ++count;
        curr_rank = (*it).IRANK;
        (*it).IRANK = rank;
        if (curr_rank != prev_rank)
        {
          tree.sendBodyCount[prev_rank] = count - 1;
          auto&& curr = v[prev_rank];
          curr.insert(curr.begin(),
                      make_move_iterator(it_begin), make_move_iterator(it));
          it_begin = it;
          count = 0;
          prev_rank = curr_rank;
        }
        else if (it + 1 == it_end)
        {
          tree.sendBodyCount[curr_rank] = count;
          auto&& curr = v[curr_rank];
          curr.insert(curr.begin(),
                      make_move_iterator(it_begin), make_move_iterator(it_end));
        }
      }
      logger::startTimer("Comm partition");
      return v;
    }

    void define_type(typer& t)
    {
      t.member(buffer_size);
      t.member(rank);
    }
};


//////////////////////////////////////////////////////////////////////
/// @brief Class for assigning each particle to corresponding partition
/// using geometric bisection
//////////////////////////////////////////////////////////////////////
struct repartition_bodies_wf
{
private:
  vec3 globalXmin;
  /// Size of partitions in each direction
  real_t* Xpartition;
  int* Npartition;

public:
  repartition_bodies_wf(vec3 a_Xmin, real_t* a_Xpartition, int* a_Npartition)
    : globalXmin(a_Xmin), Xpartition(a_Xpartition), Npartition(a_Npartition)
  { }

  typedef void result_type;
  template<typename Ref>
  result_type operator()(Ref&& vw){
    int iX[3];
    vec3 X = vw.X;
    // Loop over dimensions
    for (int d=0; d<3; d++) {
      // Index vector of partition
      iX[d] = int((X[d] - globalXmin[d]) / Xpartition[d]);
    }

    int rank = iX[0] + Npartition[0] * (iX[1] + iX[2] * Npartition[1]);
    vw.IRANK = rank;// Set send rank
  }
  void define_type(stapl::typer& t)
  {
    t.member(globalXmin);
    t.member(Xpartition, 3);
    t.member(Npartition, 3);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Class for adding leeway to boundaries for error correction
//////////////////////////////////////////////////////////////////////
template<typename BoundsType>
struct add_leeway
{
public:
  typedef BoundsType result_type;
  template <typename V>
  result_type operator()(V&& bounds)
  {
    result_type b;
    // Loop over dimensions
    for (int d=0; d<3; d++) {
      // Adding a bit of leeway to global domain
      real_t leeway = (bounds.Xmax[d] - bounds.Xmin[d]) * 1e-6;
      // Convert Xmin to real_t
      b.Xmin[d] = bounds.Xmin[d] - leeway;
      // Convert Xmax to real_t
      b.Xmax[d] = bounds.Xmax[d] + leeway;
    }
    return b;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Wrapper class for geometric bisection
//////////////////////////////////////////////////////////////////////
template <typename T>
struct octsection
{
private:
  size_t loc_id;
  size_t num_locs;

public:
  octsection(size_t a_loc, size_t a_locs)
    : loc_id(a_loc), num_locs(a_locs)
  { }

  typedef T result_type;
  template <typename PartitionedView, typename TreeView, typename B>
  result_type operator()(PartitionedView&& bodies, TreeView& tree,B&& g)
  {
    auto const& global = add_leeway<Bounds>()(g);
    // Start timer
    logger::startTimer("Partition");
    // Initialize MPI size counter
    int size = num_locs;
    // Number of partitions in each direction
    int Npartition[3] = {1, 1, 1};
    // Initialize dimension counter
    int d = 0;
    // Divide domain while counter is not one
    while (size != 1) {
      // Divide this dimension
      Npartition[d] <<= 1;
      // Increment dimension
      d = (d+1) % 3;
      // Right shift the bits of counter
      size >>= 1;
    }
    // Size of partitions in each direction
    real_t Xpartition[3];
    // Loop over dimensions
    for (d=0; d<3; d++) {
      // Size of partition in each direction
      Xpartition[d] = (global.Xmax[d] - global.Xmin[d]) / Npartition[d];
    }
    // Index vector
    int iX[3];
    // x index of partition
    iX[0] = loc_id % Npartition[0];
    // y index
    iX[1] = loc_id / Npartition[0] % Npartition[1];
    // z index
    iX[2] = loc_id / Npartition[0] / Npartition[1];
    // Local bounds
    Bounds local;
    // Loop over dimensions
    for (d=0; d<3; d++) {
      // Xmin of local domain at current rank
      local.Xmin[d] = global.Xmin[d] + iX[d] * Xpartition[d];
      // Xmax of local domain at current rank
      local.Xmax[d] = global.Xmin[d] + (iX[d] + 1) * Xpartition[d];
    }

    Bodies temp = bodies;
    for (auto&& b: temp)
      repartition_bodies_wf(global.Xmin,Xpartition,Npartition)(b);
    // Start timer
    logger::stopTimer("Partition");
    logger::startTimer("Sort");
    // Instantiate sort class
    Sort sort;
    // Sort bodies according to IRANK
    temp = sort.irank(temp);
    logger::stopTimer("Sort");
    return prepare_for_sorted_all_to_all<T>(num_locs,loc_id)(temp,tree);
  }

  void define_type(stapl::typer& t)
  {
    t.member(loc_id);
    t.member(num_locs);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Wrapper class for generating bodies
//////////////////////////////////////////////////////////////////////
struct generate_bodies_wf
{
private:
  int seed;
public:
  generate_bodies_wf(int a_seed)
    : seed(a_seed)
  {
    srand48(seed);
  }
  typedef void result_type;
  template <typename Ref>
  result_type operator()(Ref body)
  {
    vec3 x;
    for (int d=0; d<3; ++d)
      x[d] = drand48() * 2 * M_PI - M_PI;
    body.X = x;
  }

  void define_type(stapl::typer& t)
  {
    t.member(seed);
  }

};


//////////////////////////////////////////////////////////////////////
/// @brief Class for projection the output to an element in the tree structure
/// for alltoall.
/// @todo Should be removed if appropriate feature of port mapping to an
/// in structure is provided
//////////////////////////////////////////////////////////////////////
template <typename T>
struct prepare_send_bodies
{
private:
  int buffer_size;
  int rank;

public:
  prepare_send_bodies(int a_size, int a_rank)
    : buffer_size(a_size), rank(a_rank)
  { }

  typedef T result_type;
  template <typename NotifyType, typename TreeView>
  result_type operator()(NotifyType&& notify, TreeView&& tree)
  {
    logger::startTimer("Comm LET bodies");
    return tree.sendBodies;
  }
  void define_type(typer& t)
  {
    t.member(buffer_size);
    t.member(rank);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Class for projection the output to an element in the tree structure
/// for alltoall.
/// @todo Should be removed if appropriate feature of port mapping to an
/// in structure is provided
//////////////////////////////////////////////////////////////////////
template <typename T>
struct prepare_send_cells
{
private:
  int buffer_size;
  int rank;

public:
  prepare_send_cells(int a_size, int a_rank)
    : buffer_size(a_size), rank(a_rank)
  { }

  typedef T result_type;

  template <typename NotifyType, typename TreeView>
  result_type operator()(NotifyType&& notify, TreeView&& tree)
  {
    logger::stopTimer("Comm LET bodies");
    logger::startTimer("Comm LET cells");
    return tree.sendCells;
  }
  void define_type(typer& t)
  {
    t.member(buffer_size);
    t.member(rank);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Class keep some data alive to avoid destruction. This is
/// attributable to the fact that shared_pointer are used. Could be
/// eliminated iff multiple output are provided by the Skeleton composition
//////////////////////////////////////////////////////////////////////
struct keep_alive
{
  typedef void result_type;

  template<typename SinkType, typename BodyType,typename TreeView>
  result_type operator()(SinkType&& a, BodyType&& bodies, TreeView&& tree)
  {
    tree.nBodies = bodies;
  }
};

template <typename V1, typename V2, typename MinOp, typename MaxOp>
Bounds
find_extreme(V1 const& v1, V2 const& v2,
             MinOp const& min_op, MaxOp const& max_op)
{
  Bounds bounds;
  vec3 v1min = v1.Xmin;
  vec3 v1max = v1.Xmax;
  vec3 v2min = v2.Xmin;
  vec3 v2max = v2.Xmax;
  bounds.Xmin[0] = min_op(v1min[0], v2min[0]);
  bounds.Xmin[1] = min_op(v1min[1], v2min[1]);
  bounds.Xmin[2] = min_op(v1min[2], v2min[2]);

  bounds.Xmax[0] = max_op(v1max[0], v2max[0]);
  bounds.Xmax[1] = max_op(v1max[1], v2max[1]);
  bounds.Xmax[2] = max_op(v1max[2], v2max[2]);
  return bounds;
}
//////////////////////////////////////////////////////////////////////
/// @brief class for flattening data received from alltoall
//////////////////////////////////////////////////////////////////////
template<typename Sub>
struct timed_flatten_bodies
{
  typedef Sub result_type;
  template<typename Top>
  result_type operator()(Top && all)
  {
    // Start timer
    logger::stopTimer("Comm partition");
    logger::startTimer("Flatten bodies");
    using std::begin;
    using std::end;
    Sub accum;
    size_t size = 0;
    for (auto&& sub : all) {
      size+= sub.size();
    }
    accum.reserve(size);
    for (auto&& sub : all) {
      auto const& sub2 = sub;
      std::copy(sub2.begin(), sub2.end(), std::back_inserter(accum));
    }
    logger::stopTimer("Flatten bodies");
    return accum;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Wrapper class for building tree
//////////////////////////////////////////////////////////////////////
template<typename CellType>
struct build_tree_wf
{
private:
  int ncrit;
  int nspawn;

public:
  build_tree_wf(int a_ncrit, int a_nspawn)
    : ncrit(a_ncrit), nspawn(a_nspawn)
  { }

  typedef CellType result_type;
  template<typename Ref1, typename Ref2>
  result_type operator()(Ref1&& bodies, Ref2&& bounds)
  {
    Bodies bodies_ = bodies;
    BuildTree localTree(ncrit,nspawn);
    return localTree.buildTree(bodies_,bounds);
  }

  void define_type(typer& t)
  {
    t.member(ncrit);
    t.member(nspawn);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Wrapper class for generating random bodies
//////////////////////////////////////////////////////////////////////
template <typename T>
struct generate_bodies
{
private:
  int numBodies;
  char distribution;
  int rank;
  int size;

public:
  typedef T result_type;
  generate_bodies(int a_numBodies, char a_distribution,
                  int a_rank, int a_size)
    : numBodies(a_numBodies),
      distribution(a_distribution),
      rank(a_rank), size(a_size)
  { }

  template <typename BodyView>
  result_type operator()(BodyView&& bodies)
  {
    Dataset data;
    auto generated = data.initBodies(numBodies, &distribution, rank, size);
    logger::printTitle("FMM Profiling");
    logger::startTimer("Total FMM");
    logger::startPAPI();
    return generated;
  }

  void define_type(typer& t)
  {
    t.member(numBodies);
    t.member(distribution);
    t.member(rank);
    t.member(size);
  }
};


template<typename BoundsReturn>
struct prepare_for_all_gather_bounds
{
  typedef BoundsReturn result_type;
  template<typename BoundsType>
  result_type operator()(BoundsType&& bounds)
  {
    BoundsReturn boundsVal(1);
    boundsVal[0] = bounds;
    return boundsVal;
  }
};


struct set_received_bounds
{
private:
    size_t loc;

public:
  set_received_bounds(size_t a_loc)
    : loc(a_loc)
  { };

  typedef bool result_type;
  template<typename TreeView, typename BoundsVec>
  result_type operator()(TreeView&& tree, BoundsVec&& bounds)
  {
    tree.minMaxBounds = bounds;
    return true;
  }

  void define_type(typer& t)
  {
    t.member(loc);
  }
};


template <typename CellVec>
struct set_let_wf
{
private:
  real_t cycle;
  size_t rank;
public:
  typedef CellVec result_type;

  set_let_wf(real_t a_cycle, int a_rank)
    : cycle(a_cycle), rank(a_rank)
  { }

  template <typename NotifyType, typename CellType, typename TreeView>
  result_type operator()(NotifyType&& notify, CellType&& cells, TreeView&& tree)
  {
    CellVec temp = cells;
    return tree.setLET(temp,cycle);
  }

  void define_type(typer& t)
  {
    t.member(cycle);
    t.member(rank);
  }
};


struct prepare_cell_bounds
{
private:
  Bounds local_bounds;

public:
  prepare_cell_bounds(Bounds a_local)
    : local_bounds(a_local)
  { }

  typedef Bounds result_type;
  template<typename ViewType>
  result_type operator() (ViewType&& v)
  {
    Bounds bounds;
    bounds.Xmax = bounds.Xmin = local_bounds.Xmin;
    return bounds;
  }

  void define_type(typer& t)
  {
    t.member(local_bounds);
  }
};


template<typename ContainerType>
struct find_bounds_initialized
{
private:
  int nspawn;

public:
  find_bounds_initialized(int a_nspawn)
    : nspawn(a_nspawn)
  { }

  typedef std::vector<Bounds> result_type;

  template<typename ViewType, typename BoundsType>
  result_type operator()(ViewType&& container,BoundsType&& bounds)
  {
    BoundBox boundbox(nspawn);
    auto&& result = boundbox.getBounds(container, bounds);
    return prepare_for_all_gather_bounds<std::vector<Bounds>>()(result);
  }

  void define_type(typer& t)
  {
    t.member(nspawn);
  }
};


template <typename T>
struct upward_pass_wf
{
private:
  double theta;
  int useRmax;
  int useRopt;

public:
  typedef T result_type;

  upward_pass_wf(double a_theta, int a_useRmax, int a_useRpot)
    : theta(a_theta), useRmax(a_useRmax), useRopt(a_useRpot)
  { }

  template <typename CellView, typename BodyView>
  result_type operator()(CellView&& cells, BodyView&& bodies)
  {
    Cells temp = cells;
    UpDownPass upDownPass(theta,useRmax,useRopt);
    upDownPass.upwardPass(temp);
    return temp;
  }

  void define_type(typer& t)
  {
    t.member(theta);
    t.member(useRmax);
    t.member(useRopt);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Wrapper class for dualtreeraversal for the local partition
//////////////////////////////////////////////////////////////////////
template <typename CellType>
struct dual_tree_traversal_wf
{
private:
  int nspawn;
  int images;
  real_t cycle;
  real_t eps;
  bool mutual;

public:
  typedef CellType result_type;

  dual_tree_traversal_wf(int a_nspawn, int a_images,
                         real_t a_cycle, real_t a_eps,
                         bool a_mutual)
    : nspawn(a_nspawn), images(a_images),
      cycle(a_cycle), eps(a_eps), mutual(a_mutual)
  { }

  template<typename CellView>
  result_type operator()(CellView&& cellview)
  {
    Cells temp = cellview;
    Traversal traversal(nspawn, images, eps);
    traversal.initWeight(temp);
    traversal.dualTreeTraversal(temp, temp, cycle, mutual);
    return temp;
  }

  void define_type(typer& t)
  {
    t.member(nspawn);
    t.member(images);
    t.member(cycle);
    t.member(eps);
    t.member(mutual);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Wrapper class for dualtreeraversal of the received local
/// essential trees
//////////////////////////////////////////////////////////////////////
template <typename CellType>
struct get_let_traverse
{
private:
  int nspawn;
  int images;
  real_t cycle;
  real_t eps;
  bool mutual;
  size_t rank;
  size_t size;

public:
  typedef CellType result_type;

  get_let_traverse(int a_nspawn, int a_images, real_t a_cycle, real_t a_eps,
                   bool a_mutual, size_t a_rank, size_t a_size)
    : nspawn(a_nspawn), images(a_images),
      cycle(a_cycle), eps(a_eps),mutual(a_mutual), rank(a_rank), size(a_size)
  { }

  template<typename BodyT, typename CellT,  typename TreeView,
           typename CellView, typename BodyView>
  result_type operator()(BodyT&& let_bodies, CellT&& let_cells,
                         TreeView&& tree, CellView&& cell_view,
                         BodyView&& bodies)
  {
    logger::stopTimer("Comm LET cells");
    tree.recvBodies = let_bodies;
    tree.recvCells  = let_cells;

    CellType cells = cell_view;
    CellType jcells;

    Bodies temp = bodies;
    tree.jBodies.resize(temp.size());
    std::copy(temp.begin(),temp.end(),tree.jBodies.begin());
    Traversal traversal(nspawn, images, eps);
    for (size_t irank = 0; irank < size; ++irank)
    {
      tree.getLET(jcells, (rank+irank)%size);
      traversal.dualTreeTraversal(cells, jcells, cycle, false);
    }
    return cells;
  }
  void define_type(typer& t){
    t.member(nspawn);
    t.member(images);
    t.member(cycle);
    t.member(eps);
    t.member(mutual);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Wrapper class for downward pass of the FMM kernels
//////////////////////////////////////////////////////////////////////
template <typename CellType>
struct downward_pass
{
private:
  double theta;
  int useRmax;
  int useRopt;

public:
  typedef CellType result_type;
  downward_pass(double a_theta, int a_useRmax, int a_useRpot)
    : theta(a_theta), useRmax(a_useRmax), useRopt(a_useRpot)
  { }

 template <typename CellView>
 result_type operator()(CellView&& cell_view)
 {
    CellType cells = cell_view;
    UpDownPass upDownPass(theta,useRmax,useRopt);
    upDownPass.downwardPass(cells);
    logger::stopPAPI();
    logger::stopTimer("Total FMM", 0);
    return cells;
  }

  void define_type(typer& t)
  {
    t.member(theta);
    t.member(useRmax);
    t.member(useRopt);
  }
};

#endif
