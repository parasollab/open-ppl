/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARK_LONESTAR_BH_TREE_CONSTRUCT
#define STAPL_BENCHMARK_LONESTAR_BH_TREE_CONSTRUCT

//////////////////////////////////////////////////////////////////////
///  @file build.hpp
///  @brief Implementation of constructing an octree from a view of
///         particles.
///
/// First the input graph is cleared and a single root node is inserted
/// into the graph. The center position of the root cell is computed by
/// finding the bounds of the positions of the input particles. Afterwards,
/// every particle initiates a traversal from the root of the tree to
/// locate the position of the tree that it should be inserted into.
/// During the traversal, the particle examines the node and determines
/// what to do in the following manner:
///
///  - If the child that the particle should be inserted to does not exist,
///    insert the particle as the child and add the appropriate edges.
///  - Otherwise, add a task to traverse the child.
///    - If the child is a non-leaf, recursively spawn a task to traverse
///      from that node.
///    - Otherwise, split up the leaf and reinsert the particle into the
///      newly created nodes.
//////////////////////////////////////////////////////////////////////

#include <stapl/algorithms/algorithm.hpp>
#include <stapl/utility/do_once.hpp>
#include <stapl/views/repeated_view.hpp>


//////////////////////////////////////////////////////////////////////
/// @brief Workfunction that is invoked on the appropriate child of a
///        node to determine whether or not it's an internal node or
///        a leaf.
///
///        If this child is an internal node, we will spawn a task to
///        visit it. Otherwise, it is a leaf and it will be split into
///        three nodes: the new internal node, the old leaf node as a
///        child of the new internal node and the node for the particle
///        that is being inserted.
//////////////////////////////////////////////////////////////////////
class insert_traverse_child_wf
  : public stapl::dynamic_wf
{
  /// ID of the particle that started the traversal
  std::size_t  m_pid;
  /// Coordinate of the particle that started the traversal
  point        m_coord;
  /// Mass of the particle that started the traversal
  double       m_mass;
  /// Coordinate of the center of the cell that we are traversing
  point        m_center_coord;
  /// Level of the traversal
  std::size_t  m_level;

public:
  typedef void result_type;

  insert_traverse_child_wf(std::size_t pid, point const& coord, double mass,
                           point const& center_coord, std::size_t level)
   : m_pid(pid), m_coord(coord), m_mass(mass), m_center_coord(center_coord),
     m_level(level)
  { }

  template<typename TGV, typename CellNode, typename View>
  void operator()(TGV const& tgv, CellNode cell_node, View& cellgraph_view);

  void define_type(stapl::typer& t)
  {
    t.member(m_pid);
    t.member(m_coord);
    t.member(m_mass);
    t.member(m_center_coord);
    t.member(m_level);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Traversal workfunction that will traverse the tree to find the
///        appropriate spot for a particle. If that spot is empty,
///        the workfunction will add it directly. If not, it will spawn
///        a task to further traverse the tree along that branch.
//////////////////////////////////////////////////////////////////////
class insert_wf
  : public stapl::dynamic_wf
{
  /// ID of the particle that started the traversal
  std::size_t  m_pid;
  /// Coordinate of the particle that started the traversal
  point        m_coord;
  /// Coordinate of the center of the box
  point        m_center_coord;
  /// Mass of the particle that started the traversal
  double       m_mass;
  /// Level of the traversal
  std::size_t  m_level;

public:
  typedef void result_type;

  insert_wf(std::size_t pid, point coord, point center_coord, double mass,
            std::size_t level)
   :  m_pid(pid), m_coord(coord), m_center_coord(center_coord), m_mass(mass),
      m_level(level)
  { }

  template<typename TGV, typename CellNode, typename View>
  void operator()(TGV const& tgv, CellNode cell_node, View& cellgraph_view)
  {
    // determine which octant this particle needs to go into based on position
    std::size_t octant = which_child(m_center_coord, m_coord);

    // determine the id of the node that contains the octant
    boost::optional<std::size_t> child = retrieve_child_vertex(
      cell_node, octant
    );

    // if the child node where the particle should go does not exist
    if (!child)
    {
      // create a new octree node (leaf) for the cell and give it
      // an initial position and mass
      octree_node leaf_node(m_coord, m_mass, m_pid);

      // add the new cell to the tree
      std::size_t leaf_gid = cellgraph_view.add_vertex_uniform(leaf_node);

      // set the newly added cell as the octant'th child of the current cell
      cellgraph_view.add_edge_async(cell_node.descriptor(), leaf_gid,
        std::make_pair(true, octant)
      );

      cellgraph_view.add_edge_async(leaf_gid, cell_node.descriptor(),
        std::make_pair(false, octant)
      );

      return;
    }

    // the child node exists, so create a child traversal task to visit it

    // determine what the center coordinate of the child node is
    point center_coord = octant_coordinate(m_center_coord, octant, m_level+1);

    // add a task to visit the child
    tgv.add_task(
      insert_traverse_child_wf(m_pid, m_coord, m_mass, center_coord, m_level+1),
      localize_ref(tgv),
      localize_ref(cellgraph_view, *child),
      localize_ref(cellgraph_view)
    );
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_pid);
    t.member(m_coord);
    t.member(m_center_coord);
    t.member(m_mass);
    t.member(m_level);
  }
};


template<typename TGV, typename CellNode, typename View>
void insert_traverse_child_wf::operator()(TGV const& tgv, CellNode cell_node,
                                          View& cellgraph_view)
{
  // the node we are visiting is a leaf, so we need to split it up
  if (cell_node.property().is_leaf())
  {
    // determine which octant of itself the node will go into
    int new_child_octant = which_child(
      m_center_coord, cell_node.property().coord()
    );

    // determine which octant the particle that we're inserting will go into
    int particle_octant = which_child(m_center_coord, m_coord);

    // create an octree node for the new child
    point new_child_coord = octant_coordinate(
      m_center_coord, new_child_octant, m_level+1
    );

    octree_node new_child(cell_node.property().coord(),
      cell_node.property().mass(), cell_node.property().particle_id()
    );

    // add the node to the graph
    std::size_t new_child_id = cellgraph_view.add_vertex_uniform(new_child);

    // transform the current node (which was a leaf) into an internal node,
    // and add the new child to itself
    cell_node.property().internalize();
    cellgraph_view.add_edge_async(cell_node.descriptor(), new_child_id,
      std::make_pair(true, new_child_octant)
    );

    cellgraph_view.add_edge_async(new_child_id, cell_node.descriptor(),
      std::make_pair(false, new_child_octant)
    );

    // if the two particles are going into separate octants
    // we can just create a node for the particle and add it
    // directly
    if (new_child_octant != particle_octant)
    {
      // create an octree node for the particle
      octree_node particle_node(m_coord, m_mass, m_pid);

      // add the node to the graph
      std::size_t particle_id = cellgraph_view.add_vertex_uniform(
        particle_node
      );

      // set the particle to be the child of the this node
      cellgraph_view.add_edge_async(cell_node.descriptor(), particle_id,
        std::make_pair(true, particle_octant)
      );
      cellgraph_view.add_edge_async(particle_id, cell_node.descriptor(),
        std::make_pair(false, particle_octant)
      );
    }
    // the particle is going into the same octant
    else
    {
      // add a task to visit the new node we created so we can
      // further refine it
      tgv.add_task(
        insert_traverse_child_wf(
          m_pid, m_coord, m_mass, new_child_coord, m_level+1
        ),
        localize_ref(tgv),
        localize_ref(cellgraph_view, new_child_id),
        localize_ref(cellgraph_view)
      );
    }

    return;
  }

  // otherwise, the node we are visiting is an internal node so we'll keep
  // visiting it

  // visit this node
  tgv.add_task(insert_wf(m_pid, m_coord, m_center_coord, m_mass, m_level),
    localize_ref(tgv),
    localize_ref(cellgraph_view, cell_node.descriptor()),
    localize_ref(cellgraph_view)
  );
}


//////////////////////////////////////////////////////////////////////
/// @brief Workfunction that is invoked on every particle, which spawns
///        a task to the root node to start the tree construction
//////////////////////////////////////////////////////////////////////
class construct_octree
  : public stapl::dynamic_wf
{
  /// Vertex descriptor of the root of the tree
  std::size_t m_base_node;
  // Coordinate of the root cell
  point m_center;

public:
  typedef void result_type;

  construct_octree(std::size_t base_node, point const& center)
   : m_base_node(base_node), m_center(center)
  { }

  template <typename TGV, typename P, typename CellGraphV>
  void operator()(TGV const&  tgv, P particle, CellGraphV& cellgraph_view)
  {
    // add a task to start the traversal at the base node
    tgv.add_task(
      insert_wf(index_of(particle),
        particle.coord(), m_center, particle.mass(), 0
      ),
      localize_ref(tgv),
      localize_ref(cellgraph_view, m_base_node),
      localize_ref(cellgraph_view)
    );
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_base_node);
    t.member(m_center);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Extracts the position from a particle and creates a pair
///        of that position to feed to the @see minmax_point reduction
///        operation.
//////////////////////////////////////////////////////////////////////
struct particle_to_pos_pair
{
  typedef std::pair<point, point> result_type;

  template<typename Ref>
  result_type operator()(Ref x) const
  {
    return std::make_pair(x.coord(), x.coord());
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Computes the min and max x, y, z coordinates of all of the
///        points in the space to determine the center coordinate
///        of the root cell.
//////////////////////////////////////////////////////////////////////
struct minmax_point
{
  typedef std::pair<point, point> result_type;

  template<typename T, typename U>
  result_type operator()(T aa, U bb) const
  {
    const std::pair<point, point> a(aa);
    const std::pair<point, point> b(bb);

    const point min(
      a.first.x < b.first.x ? a.first.x : b.first.x,
      a.first.y < b.first.y ? a.first.y : b.first.y,
      a.first.z < b.first.z ? a.first.z : b.first.z
    );

    const point max(
      a.second.x > b.second.x ? a.second.x : b.second.x,
      a.second.y > b.second.y ? a.second.y : b.second.y,
      a.second.z > b.second.z ? a.second.z : b.second.z
    );

    return std::make_pair(min, max);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Constructs an octree from a view of particles.
///
/// @param oct The octree view that will be populated. Note that the
///            data in the view will be cleared prior to construction.
/// @param particle_view The view of the particles
/// @warning  Invalidates the octree view.
/// @return The descriptor of the root of the octree
//////////////////////////////////////////////////////////////////////
template<typename Octree, typename ParticleView>
std::size_t build_octree(Octree& oct, ParticleView& particle_view)
{
  // remove all of the nodes in the tree
  oct.clear();

  // the initial root will have the ID 0
  std::size_t root_id = 0;

  // find the bounds of the input particles
  std::pair<point, point> minmax = stapl::map_reduce(
    particle_to_pos_pair(), minmax_point(), particle_view
  );

  // the position of the root node is computed by taking
  // the average of the furthest away positions
  point root_position(
    (minmax.first.x + minmax.second.x) / 2,
    (minmax.first.y + minmax.second.y) / 2,
    (minmax.first.z + minmax.second.z) / 2
  );

  // add the root node on exactly one location
  stapl::do_once([&]() {
    octree_node root(root_position);
    root_id = oct.add_vertex(root);
  });

  // build the octree starting from the root
  map_func(construct_octree(root_id, root_position),
           particle_view,
           make_repeat_view(oct));

  return root_id;
}

#endif
