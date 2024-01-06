/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/array/static_array.hpp>
#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/algorithms/hierarchical_view.hpp>
#include <stapl/containers/graph/partitioners/graph_partitioner_utils.hpp>
#include <stapl/containers/graph/algorithms/graph_io.hpp>
#include <stapl/domains/interval.hpp>
#include <stapl/utility/do_once.hpp>
#include <stapl/runtime.hpp>
#include <stapl/paragraph/factory_wf.hpp>
#include <functional>

////////////////////////////////////////////////////////////////////
/// @file sweeps.cc
///
/// Parallel implementation of a sweep of a 3D spatial domain.  The spatial
/// domain is represented using a stapl::graph.  There are eight sweeps, one
/// beginning at each corner of the spatial domain.  After the cell in the
/// corner is processed the adjacent cells in each dimension are processed, and
/// after they are processed the next set of adjacent cells may be processed.
/// The resulting flow of execution across the graph is referred to as as
/// wave front. Each sweep is implemented as a PARAGRAPH, which allows the
/// individual tasks to be processed as soon as its adjacent predecessors have
/// been processed.
///
/// Each sweep represents the sweep of a collection of directions whose
/// dot product with the sweep direction is positive.  The implementation
/// creates ten random directions for each octant that will be swept.
///
/// The benchmark uses a @ref hierarchical_graph_view to create a multi-level
/// coarsening of the cells, which are referred to as cellsets.  The sweep
/// itself is defined recursively using nested parallelism as a sweep of sweeps,
/// with the work function at each level processing a cellset by creating a
/// nested PARAGRAPH to sweep the cellsets contained within it.
///
/// When the lowest level of the hierarchy is reached the sweep of a single cell
/// is the application of a work function that averages the results from the
/// preceding cells with the value stored in the cell.  The result of the work
/// function is a vector of values, one for each of the three outgoing faces.
/// The value of a face is the weighted product of the average computed with the
/// directions whose normals have a positive dot product with the cell face
/// normal.
///
/// This is benchmark structure mimics the core computation of the PDT
/// application.  See http://parasol.tamu.edu/asci for an explanation of the
/// full application.
//////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
/// @brief Represents a unit of the discretized spatial domain.
///
/// The cell is stored as the property of the vertices of the graph
/// representing the entire spatial domain.
//////////////////////////////////////////////////////////////////////
struct cell
{
private:
  double       m_value;

public:
  cell()
    : m_value(0.)
  {}

  cell(double const& value)
    : m_value(value)
  {}

  double value() const
  { return m_value; }

  void value(double const& val)
  { m_value = val; }

  void define_type(stapl::typer& t) const
  {
    t.member(m_value);
  }
};


namespace stapl
{
//////////////////////////////////////////////////////////////////////
/// @brief specialization of @ref proxy for the cell struct.
//////////////////////////////////////////////////////////////////////
template <typename Accessor>
class proxy<cell, Accessor>
  : public Accessor
{
  friend class proxy_core_access;

  typedef cell target_t;

public:
  explicit proxy(Accessor const& acc)
    : Accessor(acc)
  {}

  operator target_t() const
  { return Accessor::read(); }

  double value() const
  { return Accessor::const_invoke(&target_t::value); }

  void value(double const& v)
  { Accessor::invoke(&target_t::value, v); }
};
}


//////////////////////////////////////////////////////////////////////
/// @brief Three-dimensional vector implementation
///
/// This class is used to represent sweep directions and cell face normals.
//////////////////////////////////////////////////////////////////////
struct normal
{
  double m_x;
  double m_y;
  double m_z;

  normal()
    : m_x(0.), m_y(0.), m_z(0.)
  {}

  normal(double x, double y, double z)
    : m_x(x), m_y(y), m_z(z)
  {}

  void define_type(stapl::typer& t)
  {
    t.member(m_x);
    t.member(m_y);
    t.member(m_z);
  }
};


std::ostream& operator<<(std::ostream& str, normal const& n)
{
  str << "(" << n.m_x << ", " << n.m_y << ", " << n.m_z << ")";
  return str;
}


bool operator==(normal const& x, normal const& y)
{
  double epsilon(0.000001);
  return (x.m_x - y.m_x < epsilon) && (x.m_x - y.m_x > -epsilon) &&
         (x.m_y - y.m_y < epsilon) && (x.m_y - y.m_y > -epsilon) &&
         (x.m_z - y.m_z < epsilon) && (x.m_z - y.m_z > -epsilon);
}


//////////////////////////////////////////////////////////////////////
/// @brief Compute the dot product of two normals
//////////////////////////////////////////////////////////////////////
double operator*(normal const& x, normal const& y)
{
  return x.m_x*y.m_x + x.m_y*y.m_y + x.m_z*y.m_z;
}

//////////////////////////////////////////////////////////////////////
/// @brief Captures the input arguments for the benchmark and provides easy
///   access for all methods that need information about the spatial domain.
//////////////////////////////////////////////////////////////////////
struct problem_input
{
private:
  /// Number of cells in the x-dimension.
  unsigned int m_nx;
  /// Number of cells in the y-dimension.
  unsigned int m_ny;
  /// Number of cells in the z-dimension.
  unsigned int m_nz;
  /// Aggregation factor for each dimension that determines the cellset size.
  unsigned int m_agg;
  /// The number of levels of nesting to be exercised.
  unsigned int m_level;
  /// The number of locations in the x-dimension.
  unsigned int m_px;
  /// The number of locations in the y-dimension.
  unsigned int m_py;
  /// The total number of locations.
  unsigned int m_nlocs;
  /// The id of the location.
  unsigned int m_loc_id;
  /// The x-y coordinates of the location in the location space.
  std::pair<unsigned int, unsigned int> m_loc_coords;
public:
  problem_input()
    : m_nx(0), m_ny(0), m_nz(0), m_agg(0), m_px(0), m_py(0),
      m_nlocs(0), m_loc_id(0), m_loc_coords(0,0)
  {}

  problem_input(const unsigned int nx, const unsigned int ny,
                const unsigned int nz, const unsigned int agg,
                const unsigned int level,
                const unsigned int px, const unsigned int py)
    : m_nx(nx), m_ny(ny), m_nz(nz), m_agg(agg), m_level(level), m_px(px),
      m_py(py),
      m_nlocs(stapl::get_num_locations()), m_loc_id(stapl::get_location_id()),
      m_loc_coords(0,0)
  {
    stapl_assert(m_px*m_py == m_nlocs,
      "The product of the last two arguments must be the number of locations.");
    stapl_assert(m_nx/m_px >= m_agg,
      "Num cells on location must be greater than the aggregation factor.");
    stapl_assert(m_ny/m_py >= m_agg,
      "Num cells on location must be greater than the aggregation factor.");
    stapl_assert(m_nz >= m_agg,
      "Num cells on location must be greater than the aggregation factor.");
    m_loc_coords.first  = m_loc_id / m_py;
    m_loc_coords.second = m_loc_id % m_py;
  }

  unsigned int nx() const
  {
    return m_nx;
  }

  unsigned int ny() const
  {
    return m_ny;
  }

  unsigned int nz() const
  {
    return m_nz;
  }

  unsigned int agg() const
  {
    return m_agg;
  }

  unsigned int level() const
  {
    return m_level;
  }

  unsigned int px() const
  {
    return m_px;
  }

  unsigned int py() const
  {
    return m_py;
  }

  unsigned int pz() const
  {
    return 1;
  }

  unsigned int get_num_locations() const
  {
    return m_nlocs;
  }

  unsigned int get_location_id() const
  {
    return m_loc_id;
  }

  unsigned int location_x_coord() const
  {
    return m_loc_coords.first;
  }

  unsigned int location_y_coord() const
  {
    return m_loc_coords.second;
  }

  std::pair<unsigned int, unsigned int> location_coords() const
  {
    return m_loc_coords;
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Specifies the supervertex id and which cells/cellsets of the graph
///   view at the lower level of the hierarchy will be grouped into a
///   supervertex at the level of the @ref hierarchical_graph_view that is being
///   constructed.
///
///  The partitioner assumes the spatial domain is a regular three-dimension
///  brick mesh.
//////////////////////////////////////////////////////////////////////
class regular_grid_partitioner
{
public:
  typedef stapl::domset1D<size_t>          domain_type;

private:
  typedef stapl::static_array<size_t>         desc_cont_type;
  typedef stapl::static_array<domain_type>    dom_cont_type;

  problem_input m_input;

public:
  typedef stapl::array_view<desc_cont_type>   descriptor_view_type;
  typedef stapl::array_view<dom_cont_type>    view_type;

  regular_grid_partitioner(problem_input const& input)
    : m_input(input)
  {}

  //////////////////////////////////////////////////////////////////////
  /// @brief Group vertices into supervertices.
  /// @param graph_view View of the graph at the lower level of the hierarchy.
  /// @param level Level of the hierarchy being created.
  /// @return pair of stapl::array containers, the elements of the first are
  ///   the domains of the supervertices to be created, and the elements of
  ///   the second are the ids of the supervertices.
  //////////////////////////////////////////////////////////////////////
  template <typename GraphView>
  std::pair<view_type, descriptor_view_type>
  operator()(GraphView const& graph_view, std::size_t level) const
  {
    // number of vertices in graph at lower level is original size / 2^level-2
    // because level 1 doesn't reduce number of vertices.
    size_t two_lm2 = 1 << (level-2);

    size_t ax = m_input.agg();
    size_t ay = m_input.agg();
    size_t az = m_input.agg();

    if (level == 1)
    {
      two_lm2 = 1;
      ax = ay = az = 1;
    }

    size_t nx = m_input.nx() / two_lm2;
    size_t ny = m_input.ny() / two_lm2;
    size_t nz = m_input.nz() / two_lm2;

    // Convenient constants
    size_t yz_plane_size = ny * nz;
    size_t z_col_size = nz;
    size_t total_num_cellsets = (nx/ax) * (ny/ay) * (nz/az);

   //size_t loc_id = m_input.get_location_id();

    //Declare the containers and views
    desc_cont_type* cellset_ids_cont = new desc_cont_type(total_num_cellsets);
    dom_cont_type* cellset_doms_cont = new dom_cont_type(total_num_cellsets);

    // Compute the first cell id on this location.
    size_t start_cell_id =
      yz_plane_size * (nx/m_input.px()) * m_input.location_x_coord() +
      z_col_size * (ny/m_input.py()) * m_input.location_y_coord();

    size_t ncsx = nx / ax;
    size_t ncsy = ny / ay;
    size_t ncsz = nz / az;
    size_t yz_cs_plane_size = ncsy * ncsz;

    // Compute which stacks of cellsets we'll be computing.
    size_t start_id =
      yz_cs_plane_size*(ncsx/m_input.px())*m_input.location_x_coord() +
      ncsz * (ncsy/m_input.py()) * m_input.location_y_coord();

    size_t cellset_id = start_id;
    for (size_t csx = 0; csx != ncsx/m_input.px(); ++csx)
    {
      for (size_t csy = 0; csy != ncsy/m_input.py(); ++csy)
      {
        for (size_t csz = 0; csz != ncsz/m_input.pz(); ++csz)
        {
          size_t cs_start_cell_id = start_cell_id;
          cs_start_cell_id += csz*az + csy*ay*z_col_size + csx*ax*yz_plane_size;
          domain_type v;
          std::vector<size_t> dbg_info;
          for (size_t x = 0; x != ax; ++x)
          {
            for (size_t y = 0; y != ay; ++y)
            {
              for (size_t z = 0; z != az; ++z)
              {
                v += cs_start_cell_id + z + y*z_col_size + x*yz_plane_size;
                dbg_info.push_back(
                  cs_start_cell_id + z + y*z_col_size + x*yz_plane_size);
              }
            }
          }
          cellset_id = start_id + csz + csy*ncsz/m_input.pz() +
            csx*yz_cs_plane_size;
          cellset_ids_cont->operator[](cellset_id) = cellset_id;
          cellset_doms_cont->operator[](cellset_id) = v;
//          std::ostringstream info("Location ");
//          info << loc_id << " Cellset " << cellset_id << "(";
//          for (std::vector<size_t>::iterator i = dbg_info.begin();
//               i != dbg_info.end(); ++i)
//            info << *i << " ";
//          info << ")\n";
//          printf("%s",info.str().c_str());
        }
      }
    }
    // FIXME: I think a rmi_synchronize may be all we need here.
    stapl::rmi_fence();

    return std::make_pair(view_type(cellset_doms_cont),
                          descriptor_view_type(cellset_ids_cont));
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Add edges between the supervertices at the level of the hierarchical
///   graph view that is being created.
///
/// Assumes the spatial domain is a regular three-dimension brick mesh.  Results
/// in an undirected graph.
//////////////////////////////////////////////////////////////////////
class cellset_edge_functor
{
  problem_input m_input;

public:
  typedef normal value_type;

  cellset_edge_functor(problem_input const& input)
    : m_input(input)
  {}

  template <typename Grid>
  void operator()(Grid* g, std::size_t level) const
  {
    // dimensions of the grid at this level are n[x|y|z]/2^(level-1)
    // level-1 is divisor because level 1 contains a cellset for every cell.
    size_t two_l = 1 << (level-1);
    size_t nx = m_input.nx() / two_l;
    size_t ny = m_input.ny() / two_l;
    size_t nz = m_input.nz() / two_l;

    // dimensions of local portion of the grid
    size_t lx = nx / m_input.px();
    size_t ly = ny / m_input.py();
    size_t lz = nz / m_input.pz();

    size_t px = m_input.px();
    size_t py = m_input.py();
    size_t xi = m_input.location_x_coord();
    size_t yi = m_input.location_y_coord();

    size_t z_col_size = nz;
    size_t yz_plane_size = ny * z_col_size;

//    std::stringstream dbg;
//    dbg << "Location " << m_input.get_location_id() << "\n";

    size_t start_cellset_id = xi * lx * yz_plane_size + yi * ly * z_col_size;
    for (size_t i = 0; i != lx; ++i)
    {
      for (size_t j = 0; j != ly; ++j)
      {
        for (size_t k = 0; k != lz; ++k)
        {
          size_t cellset_id = start_cellset_id + k + j*lz + i*yz_plane_size;
//          dbg << "Cellset " << cellset_id << ": ";
          if ((k != 0) && (k != z_col_size-1))
          {
            g->add_edge_async(cellset_id, cellset_id-1, normal(0.,0.,-1.));
            g->add_edge_async(cellset_id, cellset_id+1, normal(0.,0.,1.));
//            dbg << cellset_id-1 << normal(0,0,-1) << "  ";
//            dbg << cellset_id+1 << normal(0,0,1) << "  ";
          }
          else
          {
            if (z_col_size != 1)
            {
              // No self edges in the cellset graph.
              if (k == 0)
              {
                g->add_edge_async(cellset_id, cellset_id+1, normal(0.,0.,1.));
//                dbg << cellset_id+1 << normal(0,0,1) << "  ";
              }
              else
              {
                g->add_edge_async(cellset_id, cellset_id-1, normal(0.,0.,-1.));
//                dbg << cellset_id-1 << normal(0,0,-1) << "  ";
              }
            }
          }
          if ((yi != 0) && (yi != py-1))
          {
            g->add_edge_async(cellset_id, cellset_id-z_col_size,
                              normal(0.,-1.,0.));
            g->add_edge_async(cellset_id, cellset_id+z_col_size,
                              normal(0.,1.,0.));
//            dbg << cellset_id-z_col_size << normal(0,-1,0) << "  ";
//            dbg << cellset_id+z_col_size << normal(0,1,0) << "  ";
          }
          else
          {
            if (py != 1)
            {
              // No self edges in the cellset graph.
              if (yi == 0)
              {
                if (j != 0)
                {
                  g->add_edge_async(cellset_id, cellset_id-z_col_size,
                                    normal(0.,-1.,0.));
                  g->add_edge_async(cellset_id, cellset_id+z_col_size,
                                    normal(0.,1.,0.));
//                  dbg << cellset_id-z_col_size << normal(0,-1,0) << "  ";
//                  dbg << cellset_id+z_col_size << normal(0,1,0) << "  ";
                }
                else
                {
                  g->add_edge_async(cellset_id, cellset_id+z_col_size,
                                    normal(0.,1.,0.));
//                  dbg << cellset_id+z_col_size << normal(0,1,0) << "  ";
                }
              }
              else
              {
                if (j != ly-1)
                {
                  g->add_edge_async(cellset_id, cellset_id-z_col_size,
                                    normal(0.,-1.,0.));
                  g->add_edge_async(cellset_id, cellset_id+z_col_size,
                                    normal(0.,1.,0.));
//                  dbg << cellset_id-z_col_size << normal(0,-1,0) << "  ";
//                  dbg << cellset_id+z_col_size << normal(0,1,0) << "  ";
                }
                else
                {
                  g->add_edge_async(cellset_id, cellset_id-z_col_size,
                                    normal(0.,-1.,0.));
//                  dbg << cellset_id-z_col_size << normal(0,-1,0) << "  ";
                }
              }
            }
          }
          if ((xi != 0) && (xi != px-1))
          {
            g->add_edge_async(cellset_id, cellset_id-yz_plane_size,
                              normal(-1.,0.,0.));
            g->add_edge_async(cellset_id, cellset_id+yz_plane_size,
                              normal(1.,0.,0.));
 //           dbg << cellset_id-yz_plane_size << normal(-1,0,0) << "  ";
 //           dbg << cellset_id+yz_plane_size << normal(1,0,0) << "  ";
          }
          else
          {
            if (px != 1)
            {
              // No self edges in the cellset graph.
              if (xi == 0)
              {
                if (i != 0)
                {
                  g->add_edge_async(cellset_id, cellset_id-yz_plane_size,
                                    normal(-1.,0.,0.));
                  g->add_edge_async(cellset_id, cellset_id+yz_plane_size,
                                    normal(1.,0.,0.));
//                  dbg << cellset_id-yz_plane_size << normal(-1,0,0) << "  ";
//                  dbg << cellset_id+yz_plane_size << normal(1,0,0) << "  ";
                }
                else
                {
                  g->add_edge_async(cellset_id, cellset_id+yz_plane_size,
                                    normal(1.,0.,0.));
//                  dbg << cellset_id+yz_plane_size << normal(1,0,0) << "  ";
                }
              }
              else
              {
                if (i != lx-1)
                {
                  g->add_edge_async(cellset_id, cellset_id-yz_plane_size,
                                    normal(-1.,0.,0.));
                  g->add_edge_async(cellset_id, cellset_id+yz_plane_size,
                                    normal(1.,0.,0.));
//                  dbg << cellset_id-yz_plane_size << normal(-1,0,0) << "  ";
//                  dbg << cellset_id+yz_plane_size << normal(1,0,0) << "  ";
                }
                else
                {
                  g->add_edge_async(cellset_id, cellset_id-yz_plane_size,
                                    normal(-1.,0.,0.));
//                  dbg << cellset_id-yz_plane_size << normal(-1,0,0) << "  ";
                }
              }
            }
          }
//          dbg << "\n";
        }
      }
    }
    stapl::rmi_fence();
//    if (level < 3)
//      printf("%s",dbg.str().c_str());
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Represents the discretized spatial domain.  The vertex
///   property is a cell and the edge property is the cell face normal.
//////////////////////////////////////////////////////////////////////
struct spatial_grid
  : public stapl::dynamic_graph<stapl::DIRECTED, stapl::MULTIEDGES, cell,
             normal>
{};


typedef spatial_grid                  grid_type;

typedef stapl::graph_view<grid_type>  grid_view_type;

//////////////////////////////////////////////////////////////////////
/// @brief Construct the discretized spatial domain
/// @param grid Empty graph that will be populated.
/// @param input Information on the size of grid to be created.
//////////////////////////////////////////////////////////////////////
int create_grid(grid_type& grid, problem_input const& input)
{
  unsigned int ax = input.nx()/input.px();
  unsigned int ay = input.ny()/input.py();

  unsigned int local_yz_plane_size = ay*input.nz();
  unsigned int yz_plane_size = input.ny()*input.nz();

  unsigned int start_cell_id =
    input.nz()*input.ny()*ax * input.location_x_coord() +
    input.nz()*ay * input.location_y_coord();
  unsigned int cell_id = start_cell_id;

  double init_value(0.);
  for (unsigned int x = ax*input.location_x_coord();
       x != ax*(input.location_x_coord()+1); ++x)
  {
    for (unsigned int y = ay*input.location_y_coord();
         y != ay*(input.location_y_coord()+1); ++y)
    {
      for (unsigned int z = 0; z != input.nz(); ++z)
      {
        if (cell_id == 0)
        {
          double nz_val = 1000000000000000000.;
          grid.add_vertex(cell_id, cell(nz_val));
        }
        else
          grid.add_vertex(cell_id, cell(init_value));
        ++cell_id;
      }
    }
    cell_id = cell_id - local_yz_plane_size + yz_plane_size;
  }

  // Fence required before adding edges.
  // The graph doesn't buffer add_edge calls.
  stapl::rmi_fence();

  cell_id = start_cell_id;
  for (unsigned int x = ax*input.location_x_coord();
       x != ax*(input.location_x_coord()+1); ++x)
  {
    for (unsigned int y = ay*input.location_y_coord();
         y != ay*(input.location_y_coord()+1); ++y)
    {
      for (unsigned int z = 0; z != input.nz(); ++z)
      {
        if ((z != 0) && (z != input.nz()-1))
        {
            grid.add_edge_async(cell_id, cell_id-1, normal(0.,0.,-1.));
            grid.add_edge_async(cell_id, cell_id+1, normal(0.,0.,1.));
        }
        else
        {
          if (z == 0)
          {
            grid.add_edge_async(cell_id, cell_id, normal()); // Z-neg boundary
            grid.add_edge_async(cell_id, cell_id+1, normal(0.,0.,1.)); // Z-pos
          }
          else
          {
            grid.add_edge_async(cell_id, cell_id-1, normal(0.,0.,-1.)); // Z-neg
            grid.add_edge_async(cell_id, cell_id, normal()); // Z-pos boundary
          }
        }
        if ((y != 0) && (y != input.ny()-1))
        {
          grid.add_edge_async(cell_id, cell_id-input.nz(), normal(0.,-1.,0.));
          grid.add_edge_async(cell_id, cell_id+input.nz(), normal(0.,1.,0.));
        }
        else
        {
          if (y == 0)
          {
            grid.add_edge_async(cell_id, cell_id, normal());
            grid.add_edge_async(cell_id, cell_id+input.nz(), normal(0.,1.,0.));
          }
          else
          {
            grid.add_edge_async(cell_id, cell_id-input.nz(), normal(0.,-1.,0.));
            grid.add_edge_async(cell_id, cell_id, normal());
          }
        }
        if ((x != 0) && (x != input.nx()-1))
        {
          grid.add_edge_async(cell_id, cell_id-yz_plane_size,
                              normal(-1.,0.,0.));
          grid.add_edge_async(cell_id, cell_id+yz_plane_size,
                              normal(1.,0.,0.));
        }
        else
        {
          if (x == 0)
          {
            grid.add_edge_async(cell_id, cell_id, normal());
            grid.add_edge_async(cell_id, cell_id+yz_plane_size,
                                normal(1.,0.,0.));
          }
          else
          {
            grid.add_edge_async(cell_id, cell_id-yz_plane_size,
                                normal(-1.,0.,0.));
            grid.add_edge_async(cell_id, cell_id, normal());
          }
        }
        ++cell_id;
      }
    }
    cell_id = cell_id - local_yz_plane_size + yz_plane_size;
  }
  stapl::rmi_fence();

//  stapl::write_graph(grid, std::cout);
  return 0;
}

//////////////////////////////////////////////////////////////////////
/// @brief Determine to which of the eight octants the provided normal belongs.
//////////////////////////////////////////////////////////////////////
unsigned int octant(normal const& n)
{
  unsigned int octant = 0;
  if (n.m_x < 0)
    octant += 4;
  if (n.m_y < 0)
    octant += 2;
  if (n.m_z < 0)
    ++octant;
  return octant;
}

//////////////////////////////////////////////////////////////////////
/// @brief Create a vector of random normals that represent directions.
/// @return vector of directions that are uniformly distributed across octants.
//////////////////////////////////////////////////////////////////////
struct create_directions
{
  typedef std::vector<normal> result_type;

  unsigned int m_num_dirs;

  create_directions(unsigned int num_dirs)
    : m_num_dirs(num_dirs)
  {}

  result_type operator()(void) const
  {
    result_type directions(m_num_dirs);

    srand48((unsigned int)(drand48()*1000000000));
    const double pi = 3.14159265358979323846;

    unsigned int angles_per_octant = m_num_dirs >> 3;
    std::vector<unsigned int> octant_counts(8, 0);
    unsigned int total_dir_count = 0;
    while (total_dir_count != m_num_dirs)
    {
      // Method for random points on unit sphere taken from
      // http://mathworld.wolfram.com/SpherePointPicking.html
      double theta = 2*pi*drand48(); // theta = 2*pi*u
      double phi = std::acos(2*drand48()-1); // phi = acos(2*(v-1))
      normal dir(std::cos(phi)*std::sin(theta),
                 std::sin(phi)*std::sin(theta),
                 std::cos(theta));
      unsigned int oct = octant(dir);
      if (octant_counts[oct] != angles_per_octant)
      {
        ++total_dir_count;
        // Storing sorted by octant to eliminate explicit angle aggregation.
        directions[oct*angles_per_octant + octant_counts[oct]] = dir;
        octant_counts[oct] += 1;
      }
    }
    return directions;
  }
};


template <typename IncomingPsi>
struct filter_sweep_result;

template <typename IncomingPsi>
inline bool operator==(filter_sweep_result<IncomingPsi> const&,
                       filter_sweep_result<IncomingPsi> const&);

//////////////////////////////////////////////////////////////////////
/// @brief The value transmitted across the face of a cell when it is processed
///   in the sweep.
//////////////////////////////////////////////////////////////////////
class cell_face_value
{
  /// Id of the cell that was processed.
  unsigned int m_source_id;
  /// Id of the cell that will receive the value.
  unsigned int m_id;
  /// Normal of the face of the processed cell.
  normal       m_normal;
  /// Value being transmitted across the face.
  double       m_value;

public:
  cell_face_value()
    : m_source_id(std::numeric_limits<unsigned int>::max()),
      m_id(std::numeric_limits<unsigned int>::max()),
      m_normal(), m_value(0.)
  {}

  cell_face_value(const unsigned int source, const unsigned int id, normal norm,
                  double const& value)
    : m_source_id(source), m_id(id), m_normal(norm), m_value(value)
  {}

  void define_type(stapl::typer& t)
  {
    t.member(m_source_id);
    t.member(m_id);
    t.member(m_normal);
    t.member(m_value);
  }

  unsigned int id() const
  { return m_id; }

  unsigned int source_id() const
  { return m_source_id; }

  normal get_normal() const
  { return m_normal; }

  double const& value() const
  { return m_value; }

  void value(double const& v)
  { m_value = v; }
};

//////////////////////////////////////////////////////////////////////
/// @brief The value transmitted across the face of a cellset when it is
///   processed by the sweep.
///
/// The cellset face is an aggregation of the faces of the components it
/// contains.  For example, at the lowest level the cellset face is a cell face.
/// For each level up in the hierarchical graph view the cellset face is an
/// aggregation of cellset faces from the lower level.
//////////////////////////////////////////////////////////////////////
template <typename ComponentFace>
class cellset_face_value
{
  /// Id of the cellset processed by the sweep.
  unsigned int m_source_id;
  /// Id of the cellset that will receive the value.
  unsigned int m_id;
  /// Normal of the face of the cellset processed by the sweep.
  normal       m_normal;
  /// Collection of the faces of the components that make up this cellset face.
  std::vector<ComponentFace> m_surface_values;

public:
  typedef std::vector<ComponentFace> subface_set_type;

  cellset_face_value()
    : m_source_id(std::numeric_limits<unsigned int>::max()),
      m_id(std::numeric_limits<unsigned int>::max()),
      m_normal(), m_surface_values()
  {}

  cellset_face_value(const unsigned int src_id, const unsigned int id,
                     normal const& norm)
    : m_source_id(src_id), m_id(id), m_normal(norm), m_surface_values()
  {}

  cellset_face_value(const unsigned int src_id, const unsigned int id,
                     normal const& norm,
                     std::vector<ComponentFace> const& surface_values)
    : m_source_id(src_id), m_id(id), m_normal(norm),
      m_surface_values(surface_values)
  {}

  cellset_face_value(ComponentFace const& face)
    : m_source_id(face.source_id()), m_id(face.id()),
      m_normal(face.get_normal()), m_surface_values(1, face)
  {}

  unsigned int source_id() const
  { return m_source_id; }

  unsigned int id() const
  { return m_id; }

  normal get_normal() const
  { return m_normal; }

  std::vector<ComponentFace> const& value() const
  { return m_surface_values; }

  void push_back(ComponentFace const& fv)
  { m_surface_values.push_back(fv); }

  void define_type(stapl::typer& t)
  {
    t.member(m_source_id);
    t.member(m_id);
    t.member(m_normal);
    t.member(m_surface_values);
  }
};

namespace stapl
{
//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref proxy for @ref cell_face_value.
//////////////////////////////////////////////////////////////////////
template <typename Accessor>
class proxy<cell_face_value, Accessor>
  : public Accessor
{
  friend class proxy_core_access;

  typedef cell_face_value target_t;

public:
  explicit proxy(Accessor const& acc)
    : Accessor(acc)
  {}

  operator target_t() const
  { return Accessor::read(); }

  unsigned int id() const
  { return Accessor::const_invoke(&target_t::id); }

  double value() const
  { return Accessor::const_invoke(&target_t::value); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref proxy for @ref cellset_face_value.
//////////////////////////////////////////////////////////////////////
template <typename ComponentFace, typename Accessor>
class proxy<cellset_face_value<ComponentFace>, Accessor>
  : public Accessor
{
  friend class proxy_core_access;

  typedef cellset_face_value<ComponentFace> target_t;

public:
  typedef typename target_t::subface_set_type subface_set_type;

  explicit proxy(Accessor const& acc)
    : Accessor(acc)
  {}

  operator target_t() const
  { return Accessor::read(); }

  unsigned int source_id() const
  { return Accessor::const_invoke(&target_t::source_id); }

  unsigned int id() const
  { return Accessor::const_invoke(&target_t::id); }

  normal get_normal() const
  { return Accessor::const_invoke(&target_t::get_normal); }

  std::vector<ComponentFace> const& value() const
  { return Accessor::const_invoke(&target_t::value); }
};
}

//////////////////////////////////////////////////////////////////////
/// @brief Work function used to filter the result of a sweep task to select
///   one of the cellset faces.
///
/// There is a filter functor for every consumer of a cellset task, because
/// each cellset that the next wave front of tasks will process only need one
/// face from the cellsets on which it depends.  This reduces the amount of data
/// transmitted between locations significantly.
///
/// @todo Re-enable filtering now that multiple filters from a given location
///   and filters for local consumer tasks are supported.
//////////////////////////////////////////////////////////////////////
template <typename IncomingPsi>
struct filter_sweep_result
{
protected:
  normal m_normal;

  friend bool operator==<>(filter_sweep_result<IncomingPsi> const&,
                         filter_sweep_result<IncomingPsi> const&);

public:
  typedef IncomingPsi result_type;

  filter_sweep_result(normal const& n)
    : m_normal(n)
  {}

  void define_type(stapl::typer& t)
  {
    t.member(m_normal);
  }

  result_type operator()(std::vector<result_type> const& sweep_result) const
  {
    typename result_type::const_iterator res = sweep_result.begin();
    for (; res != sweep_result.end(); ++res)
    {
      if ((*res).normal() * m_normal > 0)
        return *res;
    }
    printf("ERROR: result for cellset not found.\n");
    return sweep_result[0];
  }

  result_type operator()(result_type const& result) const
  {
    if (result.get_normal() * m_normal < 0)
      printf("ERROR: result for cellset not found.\n");

    return result;
  }
};

template <typename IncomingPsi>
inline bool operator==(filter_sweep_result<IncomingPsi> const& lhs,
                       filter_sweep_result<IncomingPsi> const& rhs)
{
  return lhs.m_normal == rhs.m_normal;
}

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to compute the type of a cellset face at a given level
///   of the nested sweep.
//////////////////////////////////////////////////////////////////////
template <int Level>
struct cellset_face_type
{
  typedef cellset_face_value<typename cellset_face_type<Level-1>::type> type;
  typedef std::vector<type> result_type;
};

//////////////////////////////////////////////////////////////////////
/// @brief Specialization of the metafunction to compute the type of a cellset
///   face of the most nested sweep.
//////////////////////////////////////////////////////////////////////
template <>
struct cellset_face_type<0>
{
  typedef cellset_face_value<cell_face_value> type;
  typedef std::vector<type> result_type;
};

//////////////////////////////////////////////////////////////////////
/// @brief Id and successor cellset information of the cellset on which a
///   nested sweep has been invoked.
///
/// This is needed to properly construct the resulting cellset faces of the
/// nested sweep.
//////////////////////////////////////////////////////////////////////
struct parent_cellset_info
{
  /// @brief Id of the cellset that is being processed by the nested sweep.
  /// Equivalent to source_id in @ref cell_face_value and
  /// @ref cellset_face_value.
  unsigned int parent_id;
  /// Id of the successor of the processed cellset along the x-axis.
  std::pair<unsigned int, normal> x_succ;
  /// Id of the successor of the processed cellset along the y-axis.
  std::pair<unsigned int, normal> y_succ;
  /// Id of the successor of the processed cellset along the z-axis.
  std::pair<unsigned int, normal> z_succ;

  parent_cellset_info(unsigned int id)
    : parent_id(id)
  {}

  void define_type(stapl::typer& t)
  {
    t.member(parent_id);
    t.member(x_succ);
    t.member(y_succ);
    t.member(z_succ);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Populate the PARAGRAPH of a sweep with tasks.
//////////////////////////////////////////////////////////////////////
template <typename SweepMethod>
class sweep_factory
  : public factory_wf
{
  //////////////////////////////////////////////////////////////////////
  /// @brief Information about the set of cellsets to be swept.
  //////////////////////////////////////////////////////////////////////
  struct cellsets_info
  {
    std::vector<std::size_t> m_descriptors;
    std::size_t              m_max_descriptor;
    std::vector<std::size_t> m_x_surface;
    std::vector<std::size_t> m_y_surface;
    std::vector<std::size_t> m_z_surface;

    //////////////////////////////////////////////////////////////////////
    /// @brief Extracts the cellset information from the set of graph vertices
    ///   provided.
    /// @param begin Iterator to the first cellset.
    /// @param end Iterator one past the last cellset to be processed.
    /// @param ncellsets Number of cellsets to be swept.  Provided to allow
    ///   vectors to reserve the necessary space.
    //////////////////////////////////////////////////////////////////////
    template <typename VertexIterator>
    cellsets_info(VertexIterator begin, VertexIterator end,
                  std::size_t ncellsets)
      : m_max_descriptor(0)
    {
      m_descriptors.reserve(ncellsets);
      for (VertexIterator i = begin; i != end; ++i)
      {
        std::size_t descriptor = (*i).descriptor();
        m_descriptors.push_back(descriptor);
        m_max_descriptor =
          m_max_descriptor < descriptor ? descriptor : m_max_descriptor;
      }

      // sorting gives me the cellsets in z-y-x order.
      std::sort(m_descriptors.begin(), m_descriptors.end());
    }

    std::vector<std::size_t> const& descriptors() const
    { return m_descriptors; }

    std::size_t max() const
    { return m_max_descriptor; }

    //////////////////////////////////////////////////////////////////////
    /// @brief Check if the specified cellset will be processed by the current
    ///   sweep. Used to identify which neighboring cellsets are on the
    ///   boundary of the higher level cellset.
    //////////////////////////////////////////////////////////////////////
    // check if the cellset will be processed by the current sweep.
    bool is_sibling(std::size_t desc) const
    {
      return
        std::binary_search(m_descriptors.begin(), m_descriptors.end(), desc);
    }
  };

  //////////////////////////////////////////////////////////////////////
  /// @brief Functor used to build predecessor list for a cellset.
  //////////////////////////////////////////////////////////////////////
  struct categorize_edge
  {
    double epsilon;
    normal sweep_direction;

    /// Cellset that will be processed by the task being constructed.
    cellsets_info const& m_cs_info;

    /// Ids of the cellsets whose values this cellset will receive.
    std::vector<unsigned int> predecessors;

    /// @brief Ids of the cellsets that are processed by the same nested sweep
    /// that this cellset is processed by.
    std::vector<unsigned int> sibling_preds;

    /// The number of cellsets that will receive this cellset's result.
    std::size_t               num_sibling_succs;

    /// @brief The number of cellsets that will receive this cellset's result,
    ///   and will be processed by another nested sweep.
    std::size_t               num_boundary_succs;

    /// @brief The number of faces this cellset has on the boundary of the
    ///   higher level cellset.
    std::size_t               boundary_faces;

    categorize_edge(normal const& dir, cellsets_info const& cs_info)
      : epsilon(0.000001), sweep_direction(dir), m_cs_info(cs_info),
        num_sibling_succs(0), num_boundary_succs(0), boundary_faces(0)
    {}

    categorize_edge const& operator=(categorize_edge const& other)
    {
      epsilon = other.epsilon;
      sweep_direction = other.sweep_direction;
      // m_cs_info is skipped as it is a reference both objects share.
      predecessors = other.predecessors;
      sibling_preds = other.sibling_preds;
      num_sibling_succs = other.num_sibling_succs;
      num_boundary_succs = other.num_boundary_succs;
      boundary_faces = other.boundary_faces;
      return *this;
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief Increment boundary faces depending on the normal.
    ///
    /// This determines the cellset boundary faces to which this cellset
    /// contributes.
    //////////////////////////////////////////////////////////////////////
    void update_boundary_face(normal const& n)
    {
      if (n.m_x < -epsilon || n.m_x > epsilon)
        boundary_faces += 4;
      else if (n.m_y < -epsilon || n.m_y > epsilon)
        boundary_faces += 2;
      else if (n.m_z < -epsilon || n.m_z > epsilon)
        boundary_faces += 1;
    }

    template <typename EdgeIterator>
    void operator()(EdgeIterator& ei)
    {
      // Given an edge we want to know:
      // 1. predecessor or successor
      // 2. For predecessors
      //    a. boundary input required?
      //    b. who are sibling predecessors?
      // 3. For successors
      //    a. boundary output?
      //       1. which boundary face?

      normal property = ei.property();
      double product = property * sweep_direction;
      if (product > epsilon)
      {
        // target vertex is a successor.
        if (m_cs_info.is_sibling(ei.target()))
          ++num_sibling_succs;
        else
        {
          ++num_boundary_succs;
          update_boundary_face(property);
        }
      }
      else if (product < 0-epsilon)
      {
        // target vertex is a predecessor.
        if (m_cs_info.is_sibling(ei.target()))
          sibling_preds.push_back(ei.target());
        else
          predecessors.push_back(ei.target());
      }
      // otherwise the target is independent of this vertex
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief Reset predecessor and successor information to allow the struct
    ///   to be reused.
    //////////////////////////////////////////////////////////////////////
    void reset()
    {
      predecessors.clear();
      sibling_preds.clear();
      num_sibling_succs = 0;
      num_boundary_succs = 0;
      boundary_faces = 0;
    }
  };

  /// Type information of the higher level cellset.
  typedef cellset_face_type<SweepMethod::level+1> type_comp;
public:
  typedef stapl::null_coarsener           coarsener_type;
  typedef typename type_comp::result_type result_type;

private:
  SweepMethod         m_sweep_wf;
  normal              m_sweep_direction;

  /// Information about the cellset that we are sweeping.
  parent_cellset_info m_parent_cellset;

  /// @todo I'd rather the inputs to the cellset come in to the function
  /// operator as views, but for now this is what we do.
  result_type         m_inc_faces;

  typedef stapl::result_of::localize_object<
    typename SweepMethod::result_type::value_type
  > localized_object_type;

public:
  sweep_factory(SweepMethod const& sweep_wf, normal const& sweep_direction,
                parent_cellset_info const& parent_cellset,
                result_type const& inc_faces)
    : m_sweep_wf(sweep_wf), m_sweep_direction(sweep_direction),
      m_parent_cellset(parent_cellset), m_inc_faces(inc_faces)
  {}

  coarsener_type get_coarsener() const
  { return coarsener_type(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Work function to collect the cellset faces of the tasks on each
  ///   location performing the nested sweep.
  //////////////////////////////////////////////////////////////////////
  template <typename Result>
  struct build_cellset_face_wf
  {
    typedef Result result_type;

    result_type m_result;

    build_cellset_face_wf(unsigned int const& source_id,
      std::pair<unsigned int, normal> const& succ_info)
      : m_result(source_id, succ_info.first, succ_info.second)
    {}

    template <typename AggregatedFaces>
    result_type operator()(AggregatedFaces& faces)
    {
      normal norm = m_result.get_normal();
      std::size_t num_faces = faces.size();
      for (std::size_t i = 0; i != num_faces; ++i)
      {
        const typename AggregatedFaces::reference face_vec = faces[i];
        // each face is a vector of component faces.  Search for correct normal.
        typename AggregatedFaces::reference::const_iterator face_it
          = face_vec.begin();
        typename AggregatedFaces::reference::const_iterator face_end
          = face_vec.end();
        for (; face_it != face_end; ++face_it)
        {
          if (norm * (*face_it).get_normal() > 0)
            m_result.push_back(*face_it);
        }
      }
      return m_result;
    }

    result_type operator()(void)
    {
      return m_result;
    }

    void define_type(stapl::typer& t)
    {
      t.member(m_result);
    }
  };

  //////////////////////////////////////////////////////////////////////
  /// @brief Work function to combine the cellset faces from each location
  ///   into a single aggregation.
  //////////////////////////////////////////////////////////////////////
  template <typename Result>
  struct merge_cellset_face_wf
  {
    typedef Result result_type;

    result_type m_result;

    merge_cellset_face_wf(unsigned int const& source_id,
      std::pair<unsigned int, normal> const& succ_info)
      : m_result(source_id, succ_info.first, succ_info.second)
    {}

    template <typename AggregatedFaces>
    result_type operator()(AggregatedFaces& faces)
    {
      // face_type is a cellset_face_value.
      std::size_t num_faces = faces.size();
      for (std::size_t i = 0; i != num_faces; ++i)
      {
        const typename AggregatedFaces::reference face_vec = faces[i];
        typedef typename
          AggregatedFaces::reference::subface_set_type::const_iterator face_it;
        face_it fit = face_vec.value().begin();
        face_it fend = face_vec.value().end();
        for (; fit != fend; ++fit)
        {
          m_result.push_back(*fit);
        }
      }
      return m_result;
    }

    void define_type(stapl::typer& t)
    {
      t.member(m_result);
    }
  };

  //////////////////////////////////////////////////////////////////////
  /// @brief Work function to combine the cellset face results into a single
  ///   value that is the result of the sweep on the higher level cellset.
  //////////////////////////////////////////////////////////////////////
  template <typename Result>
  struct build_cellset_result
  {
    typedef Result result_type;

    template <typename Face0, typename Face1, typename Face2>
    result_type operator()(Face0 const& f0, Face1 const& f1, Face2 const& f2)
    {
      result_type res;
      res.push_back(f0);
      res.push_back(f1);
      res.push_back(f2);
      return res;
    }
  };

  template <typename T>
  struct identity_filter
  {
    typedef T result_type;

    template <typename Ref>
    T operator()(Ref r) const
    { return r; }

    bool operator==(identity_filter const&) const
    { return true; }
  };

  template <typename Result>
  struct identity_wf
  {
    typedef Result result_type;

    template <typename Ref>
    result_type operator()(Ref val)
    {
      return val;
    }
  };

  //////////////////////////////////////////////////////////////////////
  /// @brief Build the set of inputs for a cellset that come in from the
  ///   boundary of the higher level cellset.
  //////////////////////////////////////////////////////////////////////
  std::vector<localized_object_type>
  extract_task_inputs(std::size_t csid)
  {
    std::vector<localized_object_type> cs_inputs;
    cs_inputs.reserve(3);
    // for each face in m_inc_faces
    typename result_type::iterator face_it = m_inc_faces.begin();
    typename result_type::iterator face_end = m_inc_faces.end();
    for (; face_it != face_end; ++face_it)
    {
      typedef typename result_type::value_type::subface_set_type subface_set_t;
      subface_set_t const& subfaces = (*face_it).value();

      //   for each subface in face.value()
      typename subface_set_t::const_iterator subface_it = subfaces.begin();
      typename subface_set_t::const_iterator subface_end = subfaces.end();
      for (; subface_it != subface_end; ++subface_it)
      {
        if ((*subface_it).id() == csid)
          cs_inputs.push_back(localize_object(cs_in_type(*subface_it)));
      }
    }
    return cs_inputs;
  }

  template <typename TaskGraphView, typename GridView>
  void operator()(TaskGraphView const& tgv, GridView& grid_view)
  {
    typedef typename SweepMethod::result_type task_result;
    typedef typename SweepMethod::result_type::value_type filter_result;
    typedef typename GridView::vertex_descriptor  vertex_descriptor;
    typedef typename GridView::vertex_iterator  vertex_iterator;

    std::size_t loc_id = stapl::get_location_id();
    std::size_t nlocs = stapl::get_num_locations();
    std::size_t ncellsets = grid_view.size();

    vertex_iterator gvi(grid_view.begin()), gvi_end(grid_view.end());
    cellsets_info   cs_info(gvi, gvi_end, ncellsets);
    categorize_edge edge_cat(m_sweep_direction, cs_info);

    // Naively splitting the task creation.  Would like to spatially divide
    // the cellset according to organization of locations in the gang.  This
    // requires constructing spatial layout of cellset from the grid_view.
    std::vector<std::size_t> const& descriptors = cs_info.descriptors();
    std::size_t ndescriptors = ncellsets / nlocs;

    // The number of tasks generated by the factory is
    // ncellsets + 3 faces + 1 aggregator + nlocs-1 broadcasts.
    // tasks generated
    std::size_t task_id = 0;
    std::size_t num_succs;

    std::vector<std::size_t>::const_iterator
      descriptor = descriptors.begin() + ndescriptors*loc_id;
    std::vector<std::size_t>::const_iterator
      descriptor_end = descriptor + ndescriptors;
    for (; gvi != gvi_end; ++gvi)
    {
      vertex_descriptor cellset_id = (*gvi).descriptor();

      //FIXME: I'd rather iterate descriptor and call grid_view.find_vertex(),
      //but that doesn't compile as the graph container iterator returned by
      //the find_vertex implementation isn't convertible to vertex_iterator.
      if (!std::binary_search(descriptor, descriptor_end, cellset_id))
        continue;

      if ((cellset_id == 8) || (cellset_id == 6) || (cellset_id == 5))
        cellset_id = cellset_id;

      // task_id == cellset_id because predecessor info is specified in terms
      // of cellset ids by edge categorization.
      task_id = cellset_id;

      //every task in the sweep is consumed by three tasks.
      num_succs = 3;

      edge_cat.reset();
      edge_cat = std::for_each((*gvi).begin(), (*gvi).end(), edge_cat);

      switch (edge_cat.boundary_faces)
      {
        case 1:
          cs_info.m_z_surface.push_back(cellset_id);
          break;
        case 2:
          cs_info.m_y_surface.push_back(cellset_id);
          break;
        case 3:
          cs_info.m_z_surface.push_back(cellset_id);
          cs_info.m_y_surface.push_back(cellset_id);
          break;
        case 4:
          cs_info.m_x_surface.push_back(cellset_id);
          break;
        case 5:
          cs_info.m_z_surface.push_back(cellset_id);
          cs_info.m_x_surface.push_back(cellset_id);
          break;
        case 6:
          cs_info.m_y_surface.push_back(cellset_id);
          cs_info.m_x_surface.push_back(cellset_id);
          break;
        case 7:
          cs_info.m_z_surface.push_back(cellset_id);
          cs_info.m_y_surface.push_back(cellset_id);
          cs_info.m_x_surface.push_back(cellset_id);
      }
      typedef std::vector<localized_object_type> task_inputs_t;
      task_inputs_t task_inputs;
      if (edge_cat.predecessors.size())
      {
        // There are boundary inputs to collect for this cellset.
        task_inputs = extract_task_inputs(cellset_id);
      }
      // Note: num_succs = sweep consumers + surface consumers
      stapl_assert(task_inputs.size() <= 3, "Too many boundary inputs.");
      switch (task_inputs.size() + edge_cat.sibling_preds.size())
      {
        case 0:
          tgv.add_task(task_id, m_sweep_wf, num_succs,
                       std::make_pair(&grid_view, cellset_id));
          break;
        case 1:
          if (task_inputs.size())
            tgv.add_task(task_id, m_sweep_wf, num_succs,
                         std::make_pair(&grid_view, cellset_id),
                         task_inputs[0]);
          else
            tgv.add_task(task_id, m_sweep_wf, num_succs,
                  std::make_pair(&grid_view, cellset_id),
                  stapl::consume<task_result>(tgv, edge_cat.sibling_preds[0]));
          break;
        case 2:
          switch (task_inputs.size())
          {
            case 0:
              tgv.add_task(
                task_id, m_sweep_wf, num_succs,
                std::make_pair(&grid_view, cellset_id),
                stapl::consume<task_result>(tgv, edge_cat.sibling_preds[0]),
                stapl::consume<task_result>(tgv, edge_cat.sibling_preds[1])
              );
              break;
            case 1:
              tgv.add_task(
                task_id, m_sweep_wf, num_succs,
                std::make_pair(&grid_view, cellset_id),
                task_inputs[0],
                stapl::consume<task_result>(tgv, edge_cat.sibling_preds[0])
              );
              break;
            case 2:
              tgv.add_task(task_id, m_sweep_wf, num_succs,
                    std::make_pair(&grid_view, cellset_id),
                    task_inputs[0],
                    task_inputs[1]);
          }
          break;
        case 3:
          switch (task_inputs.size())
          {
            case 0:
              tgv.add_task(
                task_id, m_sweep_wf, num_succs,
                std::make_pair(&grid_view, cellset_id),
                stapl::consume<task_result>(tgv, edge_cat.sibling_preds[0]),
                stapl::consume<task_result>(tgv, edge_cat.sibling_preds[1]),
                stapl::consume<task_result>(tgv, edge_cat.sibling_preds[2])
              );
              break;
            case 1:
              tgv.add_task(
                task_id, m_sweep_wf, num_succs,
                std::make_pair(&grid_view, cellset_id),
                task_inputs[0],
                stapl::consume<task_result>(tgv, edge_cat.sibling_preds[0]),
                stapl::consume<task_result>(tgv, edge_cat.sibling_preds[1])
              );
              break;
            case 2:
              tgv.add_task(
                task_id, m_sweep_wf, num_succs,
                std::make_pair(&grid_view, cellset_id),
                task_inputs[0],
                task_inputs[1],
                stapl::consume<task_result>(tgv, edge_cat.sibling_preds[0])
              );
              break;
            case 3:
              tgv.add_task(task_id, m_sweep_wf, num_succs,
                    std::make_pair(&grid_view, cellset_id),
                    task_inputs[0],
                    task_inputs[1],
                    task_inputs[2]);
          }
          break;
        default:
          printf("too many boundary inputs (%zu).\n",task_inputs.size());
      }
    }

    // used in aggregated consume for the boundary tasks.
    identity_filter<task_result> id_filt;

    // Each location builds its component of the cellset faces.
    task_id = cs_info.m_max_descriptor + 3*loc_id + 1;
    num_succs = 1;
    tgv.add_task(task_id,
      build_cellset_face_wf<typename type_comp::type>(
        m_parent_cellset.parent_id, m_parent_cellset.z_succ),
      num_succs,
      stapl::consume<task_result>(tgv, cs_info.m_z_surface, id_filt));

    ++task_id;  num_succs = 1;
    tgv.add_task(task_id,
      build_cellset_face_wf<typename type_comp::type>(
        m_parent_cellset.parent_id, m_parent_cellset.y_succ),
      num_succs,
      stapl::consume<task_result>(tgv, cs_info.m_y_surface, id_filt));

    ++task_id; num_succs = 1;
    tgv.add_task(task_id,
      build_cellset_face_wf<typename type_comp::type>(
        m_parent_cellset.parent_id, m_parent_cellset.x_succ),
      num_succs,
      stapl::consume<task_result>(tgv, cs_info.m_x_surface, id_filt));

    // Location 0 collects components of each face and merges them.
    if (loc_id == 0) {
      identity_filter<typename type_comp::type> id_filt2;

      std::vector<std::size_t> preds(nlocs);
      std::size_t pred_id = cs_info.m_max_descriptor + 1;
      for (std::size_t pred_cnt = 0; pred_cnt != nlocs; ++pred_cnt, pred_id+=3)
        preds[pred_cnt] = pred_id;
      task_id = cs_info.m_max_descriptor + 3*nlocs + 1;
      num_succs = 1;
      tgv.add_task(task_id,
        merge_cellset_face_wf<typename type_comp::type>(
          m_parent_cellset.parent_id, m_parent_cellset.z_succ),
        num_succs,
        stapl::consume<typename type_comp::type>(tgv, preds, id_filt2));

      ++task_id;  num_succs = 1;
      pred_id = cs_info.m_max_descriptor + 2;
      for (std::size_t pred_cnt = 0; pred_cnt != nlocs; ++pred_cnt, pred_id+=3)
        preds[pred_cnt] = pred_id;
      tgv.add_task(task_id,
        merge_cellset_face_wf<typename type_comp::type>(
          m_parent_cellset.parent_id, m_parent_cellset.y_succ),
        num_succs,
        stapl::consume<typename type_comp::type>(tgv, preds, id_filt2));

      ++task_id; num_succs = 1;
      pred_id = cs_info.m_max_descriptor + 3;
      for (std::size_t pred_cnt = 0; pred_cnt != nlocs; ++pred_cnt, pred_id+=3)
        preds[pred_cnt] = pred_id;
      tgv.add_task(task_id,
        merge_cellset_face_wf<typename type_comp::type>(
          m_parent_cellset.parent_id, m_parent_cellset.x_succ),
        num_succs,
        stapl::consume<typename type_comp::type>(tgv, preds, id_filt2));

      ++task_id; num_succs = nlocs;
      tgv.add_task(task_id, build_cellset_result<result_type>(), num_succs,
                   stapl::consume<typename type_comp::type>(tgv, task_id-3),
                   stapl::consume<typename type_comp::type>(tgv, task_id-2),
                   stapl::consume<typename type_comp::type>(tgv, task_id-1));
      tgv.set_result(task_id);
    } else {
      std::size_t res_producer = cs_info.m_max_descriptor + 3*nlocs + 4;
      task_id = res_producer + loc_id; num_succs = 1;
      tgv.add_task(task_id, identity_wf<result_type>(), num_succs,
                   stapl::consume<result_type>(tgv, res_producer));
      tgv.set_result(task_id);
    }
    return;
  }

  bool finished() const
  {
    return true;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_sweep_wf);
    t.member(m_sweep_direction);
  }
};

// Customized termination detection can be re-enabled when the other aspects of
// nested parallelism have been resolved.
// namespace stapl
// {
// namespace detail
// {
// template<typename SweepMethod>
// struct
// terminator_initializer<sweep_factory<SweepMethod> >
// {
//   typedef counter_based_terminator terminator_t;
//
//   template <typename Result, typename SVS>
//   terminator_t*
//   operator()(Result const& result, task_graph&, SVS&) const
//   {
//     terminator_t* t_ptr = new terminator_t(1);
//
//     find_accessor<Result>()(result).request_notify(
//       std::bind(&counter_based_terminator::receive_notify, t_ptr)
//     );
//
//     return t_ptr;
//   }
// };
// }
// }

//////////////////////////////////////////////////////////////////////
/// @brief Work function used in the lowest level sweep to process a single
///   cell.
//////////////////////////////////////////////////////////////////////
class cell_wf
{
  typedef std::vector<cell_face_value> inc_cell_faces;
  typedef inc_cell_faces::iterator     cell_input_iterator;

  normal                     m_sweep_direction;
  // FIXME - ARMI doesn't know how to pack refs like this.
  // Pass by value for now.
  //
  // std::vector<normal> const& m_directions;
  //
  /// The full set of directions.
  const std::vector<normal>  m_directions;
  /// The octant we are sweeping.
  unsigned int               m_sweep_octant;
  /// The set of incoming values for the cell.
  inc_cell_faces             m_cell_inputs;

public:
  cell_wf(normal const& sweep_direction, std::vector<normal> const& directions,
          unsigned int octant)
    : m_sweep_direction(sweep_direction), m_directions(directions),
      m_sweep_octant(octant)
  {}

  void define_type(stapl::typer& t)
  {
    t.member(m_sweep_direction);
    t.member(m_directions);
    t.member(m_sweep_octant);
    t.member(m_cell_inputs);
  }

  typedef std::vector<cell_face_value> result_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Performs the core computation of processing a cell.
  ///
  /// This operator is also used for the first cell in a sweep as it has no
  /// incoming face values.
  //////////////////////////////////////////////////////////////////////
  template <typename CellRef>
  result_type operator()(CellRef cell)
  {
    inc_cell_faces::iterator input_it = m_cell_inputs.begin();
    double x_in(0.), y_in(0.), z_in(0.), epsilon(0.000001);
    typename CellRef::adj_edge_iterator edge = cell.begin();
    typename CellRef::adj_edge_iterator edges_end = cell.end();
    for (; edge != edges_end; ++edge)
    {
      // Edge normals are outfacing. A negative product is an incoming value.
      normal face_norm = (*edge).property();
      if (face_norm * m_sweep_direction < -epsilon)
      {
        inc_cell_faces::iterator val_it = m_cell_inputs.begin();
        for (; val_it != m_cell_inputs.end(); ++val_it)
          if (val_it->source_id() == (*edge).target())
            break;
        stapl_assert(val_it != m_cell_inputs.end(),
                     "Didn't find cell face input");
        if ((face_norm.m_x > epsilon) || (face_norm.m_x < -epsilon))
          x_in = val_it->value();
        else if ((face_norm.m_y > epsilon) || (face_norm.m_y < -epsilon))
          y_in = val_it->value();
        else
          z_in = val_it->value();
      }
    }

    // Take the value coming in on the face and multiply it by product of
    // the face normal and each direction in the octant.
    // Add that to the cell value.
    double x_contrib(0.), y_contrib(0.), z_contrib(0.);
    int angle = m_sweep_octant*10;
    int angle_end = (m_sweep_octant+1)*10;
    for (; angle != angle_end; ++angle)
    {
      x_contrib += x_in * m_directions[angle].m_x;
      y_contrib += y_in * m_directions[angle].m_y;
      z_contrib += z_in * m_directions[angle].m_z;
    }
    // Normalize
    x_contrib /= 10.0;
    y_contrib /= 10.0;
    z_contrib /= 10.0;

    double value = cell.property().value();
    value += x_contrib + y_contrib + z_contrib;
    cell.property().value(value);

    // Adjust the value for each outgoing face.
    x_contrib = 0.;
    y_contrib = 0.;
    z_contrib = 0.;
    angle = m_sweep_octant*10;
    for (; angle != angle_end; ++angle)
    {
      x_contrib += value * m_directions[angle].m_x;
      y_contrib += value * m_directions[angle].m_y;
      z_contrib += value * m_directions[angle].m_z;
    }
    // Normalize
    x_contrib /= 10.0;
    y_contrib /= 10.0;
    z_contrib /= 10.0;

    result_type result;
    edge = cell.begin();
    for (; edge != edges_end; ++edge)
    {
      // Edge normals are outfacing. A positive product is an outgoing value.
      normal face_norm = (*edge).property();
      if (face_norm * m_directions[m_sweep_octant*10] > epsilon)
      {
        cell_face_value fv((*edge).source(), (*edge).target(),
            (*edge).property(), 0.0);
        if ((face_norm.m_x > epsilon) || (face_norm.m_x < -epsilon))
        {
          fv.value(x_contrib);
        }
        else if ((face_norm.m_y > epsilon) || (face_norm.m_y < -epsilon))
        {
          fv.value(y_contrib);
        }
        else
        {
          fv.value(z_contrib);
        }
        result.push_back(fv);
      }
    }
    return result;
  }

  template <typename CellRef, typename CellInc>
  result_type operator()(CellRef cell,
                         const CellInc inc_face_val0)
  {
    m_cell_inputs.push_back(inc_face_val0);
    return (*this)(cell);
  }

  template <typename CellRef, typename CellInc0, typename CellInc1>
  result_type operator()(CellRef cell,
                         const CellInc0 inc_face_val0,
                         const CellInc1 inc_face_val1)
  {
    m_cell_inputs.push_back(inc_face_val0);
    m_cell_inputs.push_back(inc_face_val1);
    return (*this)(cell);
  }

  template <typename CellRef, typename CellInc0,
            typename CellInc1, typename CellInc2>
  result_type operator()(CellRef cell,
                         const CellInc0 inc_face_val0,
                         const CellInc1 inc_face_val1,
                         const CellInc2 inc_face_val2)
  {
    m_cell_inputs.push_back(inc_face_val0);
    m_cell_inputs.push_back(inc_face_val1);
    m_cell_inputs.push_back(inc_face_val2);
    return (*this)(cell);
  }
};


template <int Level, typename Result>
struct make_sweep_paragraph;

//////////////////////////////////////////////////////////////////////
/// @brief Work function to process a cellset.
///
/// The cellset is processed by executing a nested sweep PARAGRAPH on the
/// cellsets that make up this cellset.
//////////////////////////////////////////////////////////////////////
template <int Level, typename ComponentFace>
class cellset_wf
{
  normal                     m_sweep_direction;
  std::vector<normal> const  m_directions;
  unsigned int               m_sweep_octant; // The octant we are sweeping.

public:
  static const int level = Level;

  cellset_wf(normal const& sweep_direction,
             std::vector<normal> const& directions, unsigned int octant)
    : m_sweep_direction(sweep_direction), m_directions(directions),
      m_sweep_octant(octant)
  {}

  void define_type(stapl::typer& t)
  {
    t.member(m_sweep_direction);
    t.member(m_directions);
    t.member(m_sweep_octant);
  }

  typedef typename cellset_face_type<Level>::result_type result_type;

  template <typename CellSetRef>
  parent_cellset_info construct_this_info(CellSetRef cellset)
  {
    double epsilon(0.000001);
    parent_cellset_info this_info(cellset.descriptor());

    // There are no edges on problem boundary, so we need defaults.
    std::pair<unsigned int, normal> default_succ(99, normal(0.,0.,0.));
    default_succ.second.m_x = m_sweep_direction.m_x;
    this_info.x_succ = default_succ;
    default_succ.second.m_x = 0.;
    default_succ.second.m_y = m_sweep_direction.m_y;
    this_info.y_succ = default_succ;
    default_succ.second.m_y = 0.;
    default_succ.second.m_z = m_sweep_direction.m_y;
    this_info.z_succ = default_succ;

    // Iterate through neighboring cellsets.
    typename CellSetRef::adj_edge_iterator edge = cellset.begin();
    typename CellSetRef::adj_edge_iterator edge_end = cellset.end();
    for (; edge != edge_end; ++edge)
    {
      normal face_norm = (*edge).property();
      if (m_sweep_direction * face_norm > 0)
      {
        if ((face_norm.m_x > epsilon) || (face_norm.m_x < -epsilon))
          this_info.x_succ = std::make_pair(((*edge).target()), face_norm);
        else if ((face_norm.m_y > epsilon) || (face_norm.m_y < -epsilon))
          this_info.y_succ = std::make_pair(((*edge).target()), face_norm);
        else
          this_info.z_succ = std::make_pair(((*edge).target()), face_norm);
      }
    }
    return this_info;
  }

  template <typename CellSetRef>
  result_type operator()(CellSetRef cellset)
  {
    std::stringstream out;
    std::size_t cellset_id = cellset.descriptor();

    // Get the inner graph view and its domain.
    typedef typename CellSetRef::property_type comp_graph_type;
    comp_graph_type p = cellset.property();

    out << "Loc " << stapl::get_location_id() << " cellset_wf<" << Level << ">"
        << cellset_id << " has size == " << p.size() << " ";

    // Sweep the cells.
    parent_cellset_info this_cellset = construct_this_info(cellset);
    return make_sweep_paragraph<Level, result_type>(out, this_cellset)
      (m_sweep_direction, m_directions, m_sweep_octant, p);
  }

  template <typename CellSetRef, typename CellSetInc>
  result_type operator()(CellSetRef cellset,
                         const CellSetInc inc_face_values)
  {
    std::stringstream out;
    std::size_t cellset_id = cellset.descriptor();

    // Get the inner graph view and its domain.
    typedef typename CellSetRef::property_type comp_graph_type;
    comp_graph_type p = cellset.property();

    out << "Loc " << stapl::get_location_id() << " cellset_wf<" << Level << ">"
        << cellset_id << " has size == " << p.size() << " ";

    // Sweep the cells.
    parent_cellset_info this_cellset = construct_this_info(cellset);
    return make_sweep_paragraph<Level, result_type>(out, this_cellset,
                                                    inc_face_values)
      (m_sweep_direction, m_directions, m_sweep_octant, p);
  }

  template <typename CellSetRef, typename CellSetInc0, typename CellSetInc1>
  result_type operator()(CellSetRef cellset,
                         const CellSetInc0 inc_face_vals0,
                         const CellSetInc1 inc_face_vals1)
  {
    std::stringstream out;
    std::size_t cellset_id = cellset.descriptor();

    // Get the inner graph view and its domain.
    typedef typename CellSetRef::property_type comp_graph_type;
    comp_graph_type p = cellset.property();

    out << "Loc " << stapl::get_location_id() << " cellset_wf<" << Level << ">"
        << cellset_id << " has size == " << p.size() << " ";

    // Sweep the cells.
    parent_cellset_info this_cellset = construct_this_info(cellset);
    return make_sweep_paragraph<Level, result_type>(out, this_cellset,
             inc_face_vals0, inc_face_vals1)
      (m_sweep_direction, m_directions, m_sweep_octant, p);
  }

  template <typename CellSetRef, typename CellSetInc0,
            typename CellSetInc1, typename CellSetInc2>
  result_type operator()(CellSetRef cellset,
                         const CellSetInc0 inc_face_vals0,
                         const CellSetInc1 inc_face_vals1,
                         const CellSetInc2 inc_face_vals2)
  {
    std::stringstream out;
    std::size_t cellset_id = cellset.descriptor();

    // Get the inner graph view and its domain.
    typedef typename CellSetRef::property_type comp_graph_type;
    comp_graph_type p = cellset.property();

    out << "Loc " << stapl::get_location_id() << " cellset_wf<" << Level << ">"
        << cellset_id << " has size == " << p.size() << " ";

    // Sweep the cells.
    parent_cellset_info this_cellset = construct_this_info(cellset);
    return make_sweep_paragraph<Level, result_type>(out, this_cellset,
             inc_face_vals0, inc_face_vals1, inc_face_vals2)
      (m_sweep_direction, m_directions, m_sweep_octant, p);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Helper struct used to create the nested PARAGRAPH needed to
///   process a cellset.
//////////////////////////////////////////////////////////////////////
template <int Level, typename Result>
struct make_sweep_paragraph
{
  std::stringstream& m_out;
  parent_cellset_info const& m_parent_cellset;
  Result m_inc_faces;

  typedef stapl::result_of::localize_object<
    typename Result::value_type
  > inc_face_t;


  make_sweep_paragraph(std::stringstream& out, parent_cellset_info const& pcs)
    : m_out(out), m_parent_cellset(pcs), m_inc_faces()
  {}

  // These constructors are called on tasks that consume from other tasks.
  make_sweep_paragraph(std::stringstream& out, parent_cellset_info const& pcs,
                       Result const& inc_face)
    : m_out(out), m_parent_cellset(pcs), m_inc_faces()
  {
    for (typename Result::const_iterator f  = inc_face.begin();
                                         f != inc_face.end();
                                       ++f)
    {
      if ((*f).id() == m_parent_cellset.parent_id)
        m_inc_faces.push_back(*f);
    }
  }

  make_sweep_paragraph(std::stringstream& out, parent_cellset_info const& pcs,
                       Result const& inc_face0, Result const& inc_face1)
    : m_out(out), m_parent_cellset(pcs), m_inc_faces()
  {
    m_inc_faces.reserve(2);
    for (typename Result::const_iterator f  = inc_face0.begin();
                                         f != inc_face0.end();
                                       ++f)
    {
      if ((*f).id() == m_parent_cellset.parent_id)
        m_inc_faces.push_back(*f);
    }
    for (typename Result::const_iterator f  = inc_face1.begin();
                                         f != inc_face1.end();
                                       ++f)
    {
      if ((*f).id() == m_parent_cellset.parent_id)
        m_inc_faces.push_back(*f);
    }
  }

  make_sweep_paragraph(std::stringstream& out, parent_cellset_info const& pcs,
                       Result const& inc_face0, Result const& inc_face1,
                       Result const& inc_face2)
    : m_out(out), m_parent_cellset(pcs), m_inc_faces()
  {
    m_inc_faces.reserve(3);
    for (typename Result::const_iterator f  = inc_face0.begin();
                                         f != inc_face0.end();
                                       ++f)
    {
      if ((*f).id() == m_parent_cellset.parent_id)
        m_inc_faces.push_back(*f);
    }
    for (typename Result::const_iterator f  = inc_face1.begin();
                                         f != inc_face1.end();
                                       ++f)
    {
      if ((*f).id() == m_parent_cellset.parent_id)
        m_inc_faces.push_back(*f);
    }
    for (typename Result::const_iterator f  = inc_face2.begin();
                                         f != inc_face2.end();
                                       ++f)
    {
      if ((*f).id() == m_parent_cellset.parent_id)
        m_inc_faces.push_back(*f);
    }
  }

  // Constructors for a single boundary input.
  make_sweep_paragraph(std::stringstream& out, parent_cellset_info const& pcs,
                       inc_face_t const& inc_face0)
    : m_out(out), m_parent_cellset(pcs), m_inc_faces()
  {
    m_inc_faces.reserve(1);
    if ((*inc_face0).id() == m_parent_cellset.parent_id)
      m_inc_faces.push_back(*inc_face0);
  }

  make_sweep_paragraph(std::stringstream& out, parent_cellset_info const& pcs,
                       inc_face_t const& inc_face0, Result const& inc_face1)
    : m_out(out), m_parent_cellset(pcs), m_inc_faces()
  {
    m_inc_faces.reserve(2);
    if ((*inc_face0).id() == m_parent_cellset.parent_id)
      m_inc_faces.push_back(*inc_face0);

    for (typename Result::const_iterator f  = inc_face1.begin();
                                         f != inc_face1.end();
                                       ++f)
    {
      if ((*f).id() == m_parent_cellset.parent_id)
        m_inc_faces.push_back(*f);
    }
  }

  make_sweep_paragraph(std::stringstream& out, parent_cellset_info const& pcs,
                       inc_face_t const& inc_face0, Result const& inc_face1,
                       Result const& inc_face2)
    : m_out(out), m_parent_cellset(pcs), m_inc_faces()
  {
    m_inc_faces.reserve(3);
    if ((*inc_face0).id() == m_parent_cellset.parent_id)
      m_inc_faces.push_back(*inc_face0);

    for (typename Result::const_iterator f  = inc_face1.begin();
                                         f != inc_face1.end();
                                       ++f)
    {
      if ((*f).id() == m_parent_cellset.parent_id)
        m_inc_faces.push_back(*f);
    }
    for (typename Result::const_iterator f  = inc_face2.begin();
                                         f != inc_face2.end();
                                       ++f)
    {
      if ((*f).id() == m_parent_cellset.parent_id)
        m_inc_faces.push_back(*f);
    }
  }

  // Constructors for two boundary inputs.
  make_sweep_paragraph(std::stringstream& out, parent_cellset_info const& pcs,
                       inc_face_t const& inc_face0, inc_face_t const& inc_face1)
    : m_out(out), m_parent_cellset(pcs), m_inc_faces()
  {
    m_inc_faces.reserve(2);
    if ((*inc_face0).id() == m_parent_cellset.parent_id)
      m_inc_faces.push_back(*inc_face0);
    if ((*inc_face1).id() == m_parent_cellset.parent_id)
      m_inc_faces.push_back(*inc_face1);
  }

  make_sweep_paragraph(std::stringstream& out, parent_cellset_info const& pcs,
                       inc_face_t const& inc_face0, inc_face_t const& inc_face1,
                       Result const& inc_face2)
    : m_out(out), m_parent_cellset(pcs), m_inc_faces()
  {
    m_inc_faces.reserve(3);
    if ((*inc_face0).id() == m_parent_cellset.parent_id)
      m_inc_faces.push_back(*inc_face0);

    if ((*inc_face1).id() == m_parent_cellset.parent_id)
      m_inc_faces.push_back(*inc_face1);

    for (typename Result::const_iterator f  = inc_face2.begin();
                                         f != inc_face2.end();
                                       ++f)
    {
      if ((*f).id() == m_parent_cellset.parent_id)
        m_inc_faces.push_back(*f);
    }
  }


  // Constructors for strictly boundary input.
  make_sweep_paragraph(std::stringstream& out, parent_cellset_info const& pcs,
                       inc_face_t const& inc_face0, inc_face_t const& inc_face1,
                       inc_face_t const& inc_face2)
    : m_out(out), m_parent_cellset(pcs), m_inc_faces()
  {
    m_inc_faces.reserve(3);
    if ((*inc_face0).id() == m_parent_cellset.parent_id)
      m_inc_faces.push_back(*inc_face0);
    if ((*inc_face1).id() == m_parent_cellset.parent_id)
      m_inc_faces.push_back(*inc_face1);
    if ((*inc_face2).id() == m_parent_cellset.parent_id)
      m_inc_faces.push_back(*inc_face2);
  }

  template <typename CellsetView>
  Result operator()(normal const& sweep_direction,
                    std::vector<normal> const& directions,
                    unsigned int octant, CellsetView& cellset_view)
  {
    /*
    m_out << "nested cellset paragraph on (";
    typename CellsetView::vertex_iterator it, it_end;
    it = cellset_view.begin();
    it_end = cellset_view.end();
    for (; it != it_end; ++it)
      m_out << (*it).descriptor() << " ";
    m_out << ")\n";
    printf("%s",m_out.str().c_str());
    */

    // pull the correct face out of each result
    typedef cellset_wf<Level-1, typename cellset_face_type<Level-1>::type>
              cellset_wf_type;

    return stapl::make_paragraph(
      sweep_factory<cellset_wf_type>(
        cellset_wf_type(sweep_direction, directions, octant),
        sweep_direction, m_parent_cellset, m_inc_faces),
      /*nested==*/true, cellset_view)();
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Specialization for the most nested sweep that invokes cell_wf on
///   the cell to be processed.
//////////////////////////////////////////////////////////////////////
template <typename Result>
struct make_sweep_paragraph<0, Result>
{
  std::stringstream& m_out;
  parent_cellset_info const& m_parent_cellset;
  Result m_inc_faces;

  typedef stapl::result_of::localize_object<
    typename SweepMethod::result_type::value_type
  > inc_face_t;

  make_sweep_paragraph(std::stringstream& out, parent_cellset_info const& pcs)
    : m_out(out), m_parent_cellset(pcs), m_inc_faces()
  {}

  // Called by tasks consuming from other tasks in the sweep.
  make_sweep_paragraph(std::stringstream& out, parent_cellset_info const& pcs,
                       Result const& inc_face)
    : m_out(out), m_parent_cellset(pcs)
  {
    for (typename Result::const_iterator f  = inc_face.begin();
                                         f != inc_face.end();
                                       ++f)
    {
      if ((*f).id() == m_parent_cellset.parent_id)
        m_inc_faces.push_back(*f);
    }
  }

  make_sweep_paragraph(std::stringstream& out, parent_cellset_info const& pcs,
                       Result const& inc_face0, Result const& inc_face1)
    : m_out(out), m_parent_cellset(pcs)
  {
    for (typename Result::const_iterator f  = inc_face0.begin();
                                         f != inc_face0.end();
                                       ++f)
    {
      if ((*f).id() == m_parent_cellset.parent_id)
        m_inc_faces.push_back(*f);
    }
    for (typename Result::const_iterator f  = inc_face1.begin();
                                         f != inc_face1.end();
                                       ++f)
    {
      if ((*f).id() == m_parent_cellset.parent_id)
        m_inc_faces.push_back(*f);
    }
  }

  make_sweep_paragraph(std::stringstream& out, parent_cellset_info const& pcs,
                       Result const& inc_face0, Result const& inc_face1,
                       Result const& inc_face2)
    : m_out(out), m_parent_cellset(pcs)
  {
    for (typename Result::const_iterator f  = inc_face0.begin();
                                         f != inc_face0.end();
                                       ++f)
    {
      if ((*f).id() == m_parent_cellset.parent_id)
        m_inc_faces.push_back(*f);
    }
    for (typename Result::const_iterator f  = inc_face1.begin();
                                         f != inc_face1.end();
                                       ++f)
    {
      if ((*f).id() == m_parent_cellset.parent_id)
        m_inc_faces.push_back(*f);
    }
    for (typename Result::const_iterator f  = inc_face2.begin();
                                         f != inc_face2.end();
                                       ++f)
    {
      if ((*f).id() == m_parent_cellset.parent_id)
        m_inc_faces.push_back(*f);
    }
  }

  // Constructors for tasks with a single boundary input.
  make_sweep_paragraph(std::stringstream& out, parent_cellset_info const& pcs,
                       inc_face_t const& inc_face)
    : m_out(out), m_parent_cellset(pcs)
  {
    m_inc_faces.reserve(1);
    if ((*inc_face).id() == m_parent_cellset.parent_id)
      m_inc_faces.push_back(*inc_face);
  }

  make_sweep_paragraph(std::stringstream& out, parent_cellset_info const& pcs,
                       inc_face_t const& inc_face0, Result const& inc_face1)
    : m_out(out), m_parent_cellset(pcs)
  {
    if ((*inc_face0).id() == m_parent_cellset.parent_id)
      m_inc_faces.push_back(*inc_face0);

    for (typename Result::const_iterator f  = inc_face1.begin();
                                         f != inc_face1.end();
                                       ++f)
    {
      if ((*f).id() == m_parent_cellset.parent_id)
        m_inc_faces.push_back(*f);
    }
  }

  make_sweep_paragraph(std::stringstream& out, parent_cellset_info const& pcs,
                       inc_face_t const& inc_face0, Result const& inc_face1,
                       Result const& inc_face2)
    : m_out(out), m_parent_cellset(pcs)
  {
    if ((*inc_face0).id() == m_parent_cellset.parent_id)
      m_inc_faces.push_back(*inc_face0);

    for (typename Result::const_iterator f  = inc_face1.begin();
                                         f != inc_face1.end();
                                       ++f)
    {
      if ((*f).id() == m_parent_cellset.parent_id)
        m_inc_faces.push_back(*f);
    }
    for (typename Result::const_iterator f  = inc_face2.begin();
                                         f != inc_face2.end();
                                       ++f)
    {
      if ((*f).id() == m_parent_cellset.parent_id)
        m_inc_faces.push_back(*f);
    }
  }

  // Constructors for tasks with two boundary inputs.
  make_sweep_paragraph(std::stringstream& out, parent_cellset_info const& pcs,
                       inc_face_t const& inc_face0, inc_face_t const& inc_face1)
    : m_out(out), m_parent_cellset(pcs)
  {
    m_inc_faces.reserve(2);
    if ((*inc_face0).id() == m_parent_cellset.parent_id)
      m_inc_faces.push_back(*inc_face0);
    if ((*inc_face1).id() == m_parent_cellset.parent_id)
      m_inc_faces.push_back(*inc_face1);
  }

  make_sweep_paragraph(std::stringstream& out, parent_cellset_info const& pcs,
                       inc_face_t const& inc_face0, inc_face_t const& inc_face1,
                       Result const& inc_face2)
    : m_out(out), m_parent_cellset(pcs)
  {
    if ((*inc_face0).id() == m_parent_cellset.parent_id)
      m_inc_faces.push_back(*inc_face0);

    if ((*inc_face1).id() == m_parent_cellset.parent_id)
      m_inc_faces.push_back(*inc_face1);

    for (typename Result::const_iterator f  = inc_face2.begin();
                                         f != inc_face2.end();
                                       ++f)
    {
      if ((*f).id() == m_parent_cellset.parent_id)
        m_inc_faces.push_back(*f);
    }
  }

  // Constructor for task with all three faces on boundary.
  make_sweep_paragraph(std::stringstream& out, parent_cellset_info const& pcs,
                       inc_face_t const& inc_face0, inc_face_t const& inc_face1,
                       inc_face_t const& inc_face2)
    : m_out(out), m_parent_cellset(pcs)
  {
    m_inc_faces.reserve(3);
    if ((*inc_face0).id() == m_parent_cellset.parent_id)
      m_inc_faces.push_back(*inc_face0);
    if ((*inc_face1).id() == m_parent_cellset.parent_id)
      m_inc_faces.push_back(*inc_face1);
    if ((*inc_face2).id() == m_parent_cellset.parent_id)
      m_inc_faces.push_back(*inc_face2);
  }

  Result promote_return(std::vector<cell_face_value> const& cell_result)
  {
    // When cellset is multi-cell this becomes scan to extract faces for
    // x/y/z succ in m_parent_cellset.
    Result cs_result;
    cs_result.reserve(cell_result.size());
    for (std::vector<cell_face_value>::const_iterator f  = cell_result.begin();
                                                      f != cell_result.end();
                                                    ++f)
      cs_result.push_back(cellset_face_value<cell_face_value>(*f));
    return cs_result;
  }

  template <typename CellsetView>
  Result operator()(normal const& sweep_direction,
                    std::vector<normal> const& directions,
                    unsigned int octant, CellsetView& cellset_view)
  {
//    m_out << "nested CELL paragraph\n";
//    printf("%s",m_out.str().c_str());
    // Sweep the cell.
    switch (m_inc_faces.size())
    {
      case 0:
        return promote_return(
                 cell_wf(sweep_direction, directions, octant)
                   (*cellset_view.begin()));
        break;
      case 1:
        return promote_return(
                 cell_wf(sweep_direction, directions, octant)
                   (*cellset_view.begin(), m_inc_faces[0].value()[0]));
        break;
      case 2:
        return promote_return(
                 cell_wf(sweep_direction, directions, octant)
                   (*cellset_view.begin(), m_inc_faces[0].value()[0],
                    m_inc_faces[1].value()[0]));
        break;
      case 3:
        return promote_return(
                 cell_wf(sweep_direction, directions, octant)
                   (*cellset_view.begin(), m_inc_faces[0].value()[0],
                    m_inc_faces[1].value()[0], m_inc_faces[2].value()[0]));
        break;
      default:
        printf("Too many incoming faces in make_paragraph<1>::operator()\n");
        return Result(); // kill compiler warning.
    }
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Invokes the sweeps of the spatial domain for each octant.
//////////////////////////////////////////////////////////////////////
template <int N, typename CellsetView>
void sweep(CellsetView& cs_view, std::vector<normal> const& directions)
{
  // Let's start from the inside and work our way out.
  // The first loop to compose is over the set of directions we sweep.
  typedef cellset_wf<N, typename cellset_face_type<N>::type> cellset_wf_type;
  typedef typename sweep_factory<cellset_wf_type>::result_type result_type;

  typedef stapl::proxy<result_type,
            stapl::edge_accessor<stapl::detail::result_view<result_type> > >
          sweep_result_ref_type;

  typedef stapl::paragraph<stapl::default_scheduler,
            sweep_factory<cellset_wf_type>, CellsetView> sweep_tg_type;

  std::vector<std::pair<sweep_result_ref_type*, sweep_tg_type *> > sweep_tgs;
  sweep_tgs.reserve(8);

  for (unsigned int octant = 0; octant != 8; ++octant)
  {
    // Construct PARAGRAPH for the sweep of this octant.
    parent_cellset_info sweep_info(0);
    sweep_info.z_succ = std::make_pair((unsigned int)1, normal(0,0,1));
    sweep_info.y_succ = std::make_pair((unsigned int)2, normal(0,1,0));
    sweep_info.z_succ = std::make_pair((unsigned int)4, normal(1,0,0));
    sweep_tgs.push_back(std::make_pair(
      (sweep_result_ref_type*)0,
      new sweep_tg_type(
            sweep_factory<cellset_wf_type>(
              cellset_wf_type(directions[octant*10], directions, octant),
              directions[octant*10], sweep_info, result_type()),
            cs_view)));
  }
  // The loop is split because prefix scan in graph view coarsening causes the
  // executor to hang.
  typename std::vector<
             std::pair<sweep_result_ref_type*, sweep_tg_type *> >::iterator
    sweep_it = sweep_tgs.begin();
  for (; sweep_it != sweep_tgs.end(); ++sweep_it)
  {
    // Start sweep using the non-blocking function operator to allow
    // sweeps to proceed concurrently.
    sweep_it->first =
      new sweep_result_ref_type(sweep_it->second->operator()(0));
  }

  // causes all sweep PARAGRAPHS to be processed.
  stapl::get_executor()(stapl::execute_all);

  // NOTE - paragraph executor does this itself when not in persistent mode.
  //
  // sweep_it = sweep_tgs.begin();
  // for (; sweep_it != sweep_tgs.end(); ++sweep_it)
  //   delete sweep_it->second;
}


stapl::exit_code stapl_main(int argc, char* argv[])
{
  problem_input input;
  if (argc == 8)
  {
    input =
      problem_input(atoi(argv[1]), atoi(argv[2]), atoi(argv[3]), atoi(argv[4]),
        atoi(argv[5]), atoi(argv[6]), atoi(argv[7]));
  }
  else
  {
    fprintf(stderr,"./sweeps nx ny nz agg level px py (4 4 4 2 1 2 2)\n");
    return EXIT_FAILURE;
  }


  // Build some random directions, 10 per octant.
  create_directions dir_creator(80);
  std::vector<normal> directions = stapl::do_once(dir_creator);

  // Build Grid and Grid View
  grid_type grid;
  create_grid(grid, input);
  grid_view_type grid_view(grid);


  // Output of the benchmark is buffered in a stream.
  std::stringstream out;
  unsigned int last_cell = input.nx() * input.ny() * input.nz() - 1;
  out << "cell 0 and " << last_cell << " value before sweep: ("
      << grid_view[0].property().value() << ", "
      << grid_view[last_cell].property().value() << ")\n";


  // Build Cellset View.
  typedef stapl::hierarchical_view_type<
            grid_view_type, regular_grid_partitioner, normal
          >::type                                               l0_cs_view_type;

  typedef stapl::hierarchical_view_type<
            l0_cs_view_type, regular_grid_partitioner, normal
          >::type                                               l1_cs_view_type;

  typedef stapl::hierarchical_view_type<
            l1_cs_view_type, regular_grid_partitioner, normal
          >::type                                               l2_cs_view_type;

  typedef stapl::hierarchical_view_type<
            l2_cs_view_type, regular_grid_partitioner, normal
          >::type                                               l3_cs_view_type;

  typedef stapl::hierarchical_view_type<
            l3_cs_view_type, regular_grid_partitioner, normal
          >::type                                               l4_cs_view_type;

  typedef stapl::hierarchical_view_type<
            l4_cs_view_type, regular_grid_partitioner, normal
          >::type                                               l5_cs_view_type;

  // Cellsets in this view each contain a single cell.
  l0_cs_view_type l0_cs_view =
    stapl::create_level(grid_view, regular_grid_partitioner(input),
                        cellset_edge_functor(input));

  // Cellsets in these views each contain multiple lower level cellsets.
  l1_cs_view_type* l1_cs_view(0);
  l2_cs_view_type* l2_cs_view(0);
  l3_cs_view_type* l3_cs_view(0);
  l4_cs_view_type* l4_cs_view(0);
  l5_cs_view_type* l5_cs_view(0);

  // Construct views, sweep, and cleanup for the desired level of nesting.
  switch (input.level())
  {
    case 1:
      l1_cs_view = new l1_cs_view_type(
        stapl::create_level(l0_cs_view, regular_grid_partitioner(input),
                            cellset_edge_functor(input)));

      sweep<1>(*l1_cs_view, directions);

      delete l1_cs_view;

      break;
    case 2:
      l1_cs_view = new l1_cs_view_type(
        stapl::create_level(l0_cs_view, regular_grid_partitioner(input),
                            cellset_edge_functor(input)));
      l2_cs_view = new l2_cs_view_type(
        stapl::create_level(*l1_cs_view, regular_grid_partitioner(input),
                            cellset_edge_functor(input)));

      sweep<2>(*l2_cs_view, directions);

      delete l1_cs_view;
      delete l2_cs_view;

      break;
    case 3:
      l1_cs_view = new l1_cs_view_type(
        stapl::create_level(l0_cs_view, regular_grid_partitioner(input),
                            cellset_edge_functor(input)));
      l2_cs_view = new l2_cs_view_type(
        stapl::create_level(*l1_cs_view, regular_grid_partitioner(input),
                            cellset_edge_functor(input)));
      l3_cs_view = new l3_cs_view_type(
        stapl::create_level(*l2_cs_view, regular_grid_partitioner(input),
                            cellset_edge_functor(input)));

      sweep<3>(*l3_cs_view, directions);

      delete l1_cs_view;
      delete l2_cs_view;
      delete l3_cs_view;

      break;
    case 4:
      l1_cs_view = new l1_cs_view_type(
        stapl::create_level(l0_cs_view, regular_grid_partitioner(input),
                            cellset_edge_functor(input)));
      l2_cs_view = new l2_cs_view_type(
        stapl::create_level(*l1_cs_view, regular_grid_partitioner(input),
                            cellset_edge_functor(input)));
      l3_cs_view = new l3_cs_view_type(
        stapl::create_level(*l2_cs_view, regular_grid_partitioner(input),
                            cellset_edge_functor(input)));
      l4_cs_view = new l4_cs_view_type(
        stapl::create_level(*l3_cs_view, regular_grid_partitioner(input),
                            cellset_edge_functor(input)));

      sweep<4>(*l4_cs_view, directions);

      delete l1_cs_view;
      delete l2_cs_view;
      delete l3_cs_view;
      delete l4_cs_view;

      break;
    case 5:
      l1_cs_view = new l1_cs_view_type(
        stapl::create_level(l0_cs_view, regular_grid_partitioner(input),
                            cellset_edge_functor(input)));
      l2_cs_view = new l2_cs_view_type(
        stapl::create_level(*l1_cs_view, regular_grid_partitioner(input),
                            cellset_edge_functor(input)));
      l3_cs_view = new l3_cs_view_type(
        stapl::create_level(*l2_cs_view, regular_grid_partitioner(input),
                            cellset_edge_functor(input)));
      l4_cs_view = new l4_cs_view_type(
        stapl::create_level(*l3_cs_view, regular_grid_partitioner(input),
                            cellset_edge_functor(input)));
      l5_cs_view = new l5_cs_view_type(
        stapl::create_level(*l4_cs_view, regular_grid_partitioner(input),
                            cellset_edge_functor(input)));

      sweep<5>(*l5_cs_view, directions);

      delete l1_cs_view;
      delete l2_cs_view;
      delete l3_cs_view;
      delete l4_cs_view;
      delete l5_cs_view;
  }

  out << "cell 0 and " << last_cell << " value after sweep: ("
      << grid_view[0].property().value() << ", "
      << grid_view[last_cell].property().value() << ")\n";
  if (stapl::get_location_id() == 0)
    printf("%s",out.str().c_str());

  return EXIT_SUCCESS;
}
