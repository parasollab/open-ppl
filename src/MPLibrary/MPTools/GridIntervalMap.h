#ifndef PMPL_GRID_INTERVAL_MAP_H_
#define PMPL_GRID_INTERVAL_MAP_H_

#include "MPLibrary/MPBaseObject.h"
#include "MPLibrary/ValidityCheckers/CollisionDetection/CDInfo.h"
#include "Utilities/Hash.h"
#include "Utilities/SSSP.h"
#include "Utilities/XMLNode.h"
#include "Workspace/GridOverlay.h"
#include "Workspace/WorkspaceDecomposition.h"

#include <algorithm>
#include <map>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>


#ifdef PMPL_USE_SIMULATOR
#include "Simulator/Simulation.h"
#include "Geometry/Boundaries/WorkspaceBoundingBox.h"
#endif


////////////////////////////////////////////////////////////////////////////////
/// This tool attempts to help in the grid-cbs implementation. This preliminary 
/// implementation is based on MPTool TopologicalMap. Inside this tool, there is 
/// also the implementation of the Reservation Table used in 
///
/// The documentation will frequently refer to VIDs or configurations that are
/// 'contained' within a workspace region or grid cell. In this context, a
/// configuration is contained by a region or cell if it's reference point (used
/// for distance measurement) lies within.
///
/// @WARNING The current implementation assumes that the workspace decomposition
///          will not change once created. If this occurs, the maps must be
///          reset.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class GridIntervalMap final : public MPBaseObject<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType                CfgType;
    typedef typename MPTraits::WeightType             WeightType;
    typedef typename MPTraits::RoadmapType            RoadmapType;
    typedef typename MPTraits::Path                   Path;
    typedef typename RoadmapType::VID                 VID;
    typedef typename RoadmapType::VI                  VI;
    typedef typename RoadmapType::VertexSet           VertexSet;
    typedef WorkspaceDecomposition::vertex_descriptor VD;
    typedef WorkspaceDecomposition::adj_edge_iterator EI;

    ///@}
    ///@name Types
    ///@{

    /// A conflict decribes an inter-robot collision between two or more different
    /// robots at a given time.
    struct CellConflict {
      std::vector<Robot*> robots;     ///< The conflicting robots
      size_t cellIndex;     ///< The index cell.
      size_t  timestep; ///< The timestep when the collision occurred.

      /// @todo This is to support old gcc v4.x, replace with
      ///       default-constructed class members after we upgrade.
      CellConflict(const std::vector<Robot*> _robots,
          const size_t _cellIndex = 0,
          const size_t _timestep = 0) :
          robots(_robots),
          cellIndex(_cellIndex),
          timestep(_timestep)
      {}

      /// @return True if this is an empty conflict.
      bool Empty() const noexcept {
        return robots.size() <= 1;
      }
    };

    ///@name Local Types
    ///@{

    typedef std::map<size_t,size_t> Intervals; ///< A map representing a set of intervals <low,high>

    typedef std::map<Robot*,Intervals> RobotIntervals; // Mapping a robot to a set of intervals

    typedef std::map<size_t,RobotIntervals> CellsIntervals; // Mapping a cell to robot intervals



    



    ///@}
    ///@name Construction
    ///@{

    GridIntervalMap();

    GridIntervalMap(XMLNode& _node);

    virtual ~GridIntervalMap();

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Initialize() override;

    ///@}
    ///@name Map Queries
    ///@{


    ///@}
    ///@name Region and Cell Location
    ///@{

    /// Find the grid cell index that holds a given configuration.
    /// @param _v The configuration VID.
    /// @param _bodyIndex The body to use.
    /// @return The index of the grid cell which contains _bodyIndex when
    ///         configured at _v.
    size_t LocateCell(const VID _v, const size_t _bodyIndex = 0) const;

    /// Find the grid cell index that holds a given configuration.
    /// @param _c The configuration.
    /// @param _bodyIndex The body to use.
    /// @return The index of the grid cell which contains _bodyIndex when
    ///         configured at _c.
    size_t LocateCell(const CfgType& _c, const size_t _bodyIndex = 0) const;

    /// Find the grid cell index that holds a workspace point.
    /// @param _p The workspace point.
    /// @return The index of the grid cell which contains _p.
    size_t LocateCell(const Point3d& _p) const;

    /// Find the grid cells that touch a given configuration.
    /// @param _c The configuration.
    /// @return A set of cell indexes that touch _v (either the
    ///         boundary or closure).
    std::unordered_set<size_t> LocateCells(const CfgType& _c);

    void UpdateReservationTable(const Path* _p);

    void AddTimestep(Intervals& _intervals, size_t _t);

    bool ContainsTimestep(Intervals& _intervals, size_t _t);

    CellConflict FindConflict(size_t _cellIndex, RobotIntervals _robotIntervals, size_t _timestep);

    ///@} 

  private:

    ///@name Internal State
    ///@{

    std::string m_decompositionLabel; ///< The workspace decomposition to use.

    /// A grid matrix overlaid on the workspace.
    std::unique_ptr<const GridOverlay> m_grid;

    double m_gridSize{.1}; ///< Length of each grid cell.

    CellsIntervals m_cellsIntervals;



    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
GridIntervalMap<MPTraits>::
GridIntervalMap() : MPBaseObject<MPTraits>("GridIntervalMap") {
  this->SetName("GridIntervalMap");
}


template <typename MPTraits>
GridIntervalMap<MPTraits>::
GridIntervalMap(XMLNode& _node) : MPBaseObject<MPTraits>(_node) {
  this->SetName("GridIntervalMap");

  m_gridSize = _node.Read("gridSize", true, 0.,
      std::numeric_limits<double>::min(), std::numeric_limits<double>::max(),
      "The grid cell length. Very small values will cause memory overflow "
      "as we try to map huge quantities of grid cell to workspace region "
      "relations. Over-large values will cause slow mappings as we will need "
      "to check many candidate regions.");

  // m_decompositionLabel = _node.Read("decompositionLabel", true, "",
  //     "The workspace decomposition to use.");

}


template <typename MPTraits>
GridIntervalMap<MPTraits>::
~GridIntervalMap() = default;

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
GridIntervalMap<MPTraits>::
Initialize() {
  // This object only works for single-robot problems right now.
  // if(this->GetGroupTask())
  //   throw NotImplementedException(WHERE) << "Topological map does not yet "
  //                                        << "support robot groups.";

  auto env = this->GetEnvironment();
  // auto decomposition = this->GetDecomposition();

  // // If we are debugging, write the decomposition file to OBJ for inspection.
  // /// @todo Move this to the decomposition class, which currently cannot do
  // ///       this job because it does not know its own label.
  // if(this->m_debug)
  //   decomposition->WriteObj(this->GetLabel() + ".obj");


  // Initialize the grid and decomposition map.
  const std::string gridLabel = this->GetNameAndLabel() + "::GridOverlay";
  if(!m_grid.get())
  {
    m_grid = std::unique_ptr<const GridOverlay>(new GridOverlay(
        env->GetBoundary(), m_gridSize));
    this->GetStatClass()->SetStat(gridLabel + "::Size", m_grid->Size());
  }
    

    /// @TODO This exception should probably be a check on whether the robot's
    ///       largest minimum body radius is larger than the grid resolution, I
    ///       think probably at least 3x?
    if(env->UsingBoundaryObstacle())
      throw RunTimeException(WHERE) << "I'm pretty sure that using a boundary "
                                    << "obstacle with this won't work right, "
                                    << "unless your cell resolution is "
                                    << "significantly finer than the minimum "
                                    << "robot radius. Uncomment this exception "
                                    << "at your own peril!";


  // // Install roadmap hooks.
  // auto g = this->GetRoadmap();
  // g->InstallHook(RoadmapType::HookType::AddVertex, this->GetNameAndLabel(),
  //     [this](const VI _vi){this->MapCfg(_vi);});
  // g->InstallHook(RoadmapType::HookType::DeleteVertex, this->GetNameAndLabel(),
  //     [this](const VI _vi){this->UnmapCfg(_vi);});

  // // If the graph has existing vertices, map them now.
  // for(auto iter = g->begin(); iter != g->end(); ++iter)
  //   MapCfg(iter);
}

/*---------------------------------- Queries ---------------------------------*/


template <typename MPTraits>
size_t
GridIntervalMap<MPTraits>::
LocateCell(const VID _v, const size_t _bodyIndex) const {
  return LocateCell(this->GetRoadmap()->GetVertex(_v), _bodyIndex);
}


template <typename MPTraits>
size_t
GridIntervalMap<MPTraits>::
LocateCell(const CfgType& _cfg, const size_t _bodyIndex) const {
  _cfg.ConfigureRobot();
  auto body = _cfg.GetMultiBody()->GetBody(_bodyIndex);

  return LocateCell(body->GetWorldTransformation().translation());
}


template <typename MPTraits>
size_t
GridIntervalMap<MPTraits>::
LocateCell(const Point3d& _p) const {
  return m_grid->LocateCell(_p);
}


template<typename MPTraits>
std::unordered_set<size_t> 
GridIntervalMap<MPTraits>::
LocateCells(const CfgType& _c) {

  // Configure _c's robot at _c.
  auto robotMultiBody = _c.GetRobot()->GetMultiBody();
  robotMultiBody->Configure(_c);

  std::unordered_set<size_t> multiBodyCells;

  for(auto& b : robotMultiBody->GetBodies()) {
        const auto& poly = b.GetWorldPolyhedron();
        const std::unordered_set<size_t> bodyCells = m_grid->LocateCells(poly);

        if(this->m_debug)
          std::cout << "\tFound " << bodyCells.size() << " cells " << bodyCells  << "for robot " 
                    << _c.GetRobot()->GetLabel() << ", body " << b.GetIndex() << "."
                    << std::endl;

        std::copy(bodyCells.begin(), bodyCells.end(),
            std::inserter(multiBodyCells, multiBodyCells.end()));
      }

  return multiBodyCells;
}


template<typename MPTraits>
void
GridIntervalMap<MPTraits>::
UpdateReservationTable(const Path* _p) {

  auto cfgs = _p->FullCfgs(this->GetMPLibrary());
  auto robot = cfgs[0].GetRobot();
  for(size_t i = 0; i < cfgs.size() ; ++i) {
    auto cells = LocateCells(cfgs[i]);
    for(auto& cell : cells ) {
      if(m_cellsIntervals.count(cell)) {
        //inside this we need to verify is two or more robots are 
        //ocuppying the same cell a the same timestep

        auto it = m_cellsIntervals.find(cell);

        //auto robotIntervals = m_cellsIntervals[cell];

        if(it->second.count(robot)) {
          //cell has been occupied before, current robot has been there before
          //auto intervals = robotIntervals[robot];
          auto it2 = it->second.find(robot);
          AddTimestep(it2->second, i);
        } else {
          //cell has been occupied before, but current robot has not been there before
          it->second[robot][i] = i;
          //it->second.insert(std::make_pair(robot, std::make_pair(i,i)));
        }
        // once we added the timestep we can check it for conflicts

        auto cellConflict =  FindConflict(cell, it->second, i );
        if(cellConflict.Empty()) {
          //std::cout << "No conflicts found becuase robots size is " << cellConflict.robots.size() << std::endl;
        }
        else {
          std::cout << "CONFLICT found at timestep " << cellConflict.timestep << " ,at cell " 
            << cellConflict.cellIndex << " involving robots " << std::endl;
            for(auto& robot : cellConflict.robots)
              std::cout << robot->GetLabel() << "\t";
            std::cout << std::endl; 
        }

      } else {
        // this cell has not been occupied yet, there is no need for checking it for conclicts
        m_cellsIntervals[cell][robot][i] = i;
      }
    }
  }
}


template<typename MPTraits>
void
GridIntervalMap<MPTraits>::
AddTimestep(Intervals& _intervals, size_t _t) {
  // We have 3 cases
  //  1.- _t is contained in an existing interval
  //  2.- _t is adjacent to an existining interval, then we need to modify that interval
  //  3.- _t is neither contained or adjacent to an existing interval, 
  //      so we need to add a new interval
  bool timestepAdded = false;
  for(auto& interval : _intervals) {
    //Check it _t is adjacent to lower bound pf the interval
    if(_t + 1 == interval.first ) {
      //we need to modify this current interval for including _t at the beggining
      _intervals[_t] = interval.second;
      _intervals.erase(interval.first);
      timestepAdded = true;
      break;
    } else if ( interval.first <= _t and _t <= interval.second) {
      // no action is required here, since _t is already contained
      timestepAdded = true;
      break;
    } else if ( interval.second + 1 == _t) {
      // we need to modify the current interval for including _t at the end
      _intervals[interval.first] = _t;
      timestepAdded = true; 
      break;
    } else {}
  }
  if(!timestepAdded)
    _intervals[_t] = _t;

}


template<typename MPTraits>
bool
GridIntervalMap<MPTraits>::
ContainsTimestep(Intervals& _intervals, size_t _t) {
  for(auto& interval : _intervals) {
    if ( interval.first <= _t and _t <= interval.second) 
      return true; 
  }
  return false;
}


template<typename MPTraits>
typename GridIntervalMap<MPTraits>::CellConflict
GridIntervalMap<MPTraits>::
FindConflict(size_t _cellIndex, RobotIntervals _robotIntervals, size_t _timestep) {
  std::vector<Robot*> robots;
  for(auto& intervals : _robotIntervals) {
    if(ContainsTimestep(intervals.second, _timestep))
      robots.push_back(intervals.first);
  }
  return CellConflict(robots, _cellIndex, _timestep);
} 
/*----------------------------------------------------------------------------*/

#endif
