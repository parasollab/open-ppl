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
      CellConflict(
          const size_t _cellIndex = 0,
          const size_t _timestep = 0,
          const std::vector<Robot*> _robots = std::vector<Robot*>() ) :
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

    typedef std::map<size_t,CellConflict> ConflictMap; // A map of all the detected conflicts


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

    // Updates the reservation table, 
    void UpdateReservationTable(Robot* _robot, std::unordered_set<size_t> _cells , size_t _minTime,
      size_t _maxTime);

    // Adds a timestep to a set of intervals.
    void AddTimestep(Intervals& _intervals, size_t _t);

    // Checks whether a timestep is contained on a given set fo intervals.
    bool ContainsTimestep(Intervals& _intervals, size_t _t);

    // Check if two or more robots have been at the same cell and at the same timestep.  
    CellConflict FindConflict(size_t _cellIndex, RobotIntervals _robotIntervals, size_t _timestep);

    // After checking all the paths and discovering all the conflicts, this function returns the 
    // earliest one.
    CellConflict FindEarliestConflict();

    // Clears all conflicts and all intervals.
    void ClearConflicts();

    // Returns the geometric center of a cell.
    Point3d CellCenter(const size_t _index) const noexcept;

    ///@} 

  private:

    ///@name Internal State
    ///@{

    std::string m_decompositionLabel; ///< The workspace decomposition to use.

    /// A grid matrix overlaid on the workspace.
    std::unique_ptr<const GridOverlay> m_grid;

    double m_gridSize{.1}; ///< Length of each grid cell.

    CellsIntervals m_cellsIntervals;

    ConflictMap m_conflictMap;

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

}

/*---------------------------------- Queries ---------------------------------*/

template <typename MPTraits>
Point3d
GridIntervalMap<MPTraits>::
CellCenter(const size_t _index) const noexcept {
  return m_grid->CellCenter(_index);
}


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
    MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::LocateCells");

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
UpdateReservationTable(Robot* _robot, std::unordered_set<size_t> _cells , size_t _minTime,
  size_t _maxTime) {

  for(auto& cell : _cells ) {
    if(m_cellsIntervals.count(cell)) {
      // Inside this we need to verify if two or more robots are ocuppying the same cell a the same timestep
      auto it = m_cellsIntervals.find(cell);
      if(it->second.count(_robot)) {
        // Case 1: cell has been occupied before, current robot has been there before
        auto it2 = it->second.find(_robot);
        for(size_t i = _minTime ; i <= _maxTime ; ++i )
          AddTimestep(it2->second, i);
      } else {
        // Case 2: cell has been occupied before, but current robot has not been there before
        it->second[_robot][_minTime] = _maxTime;
      }
      // Once we added the timesteps during the robot swepts the cells, we can check it 
      // other robot has been there at the sime time (conflict detection)
      for(size_t i = _minTime ; i <= _maxTime ; ++i ) {
        auto cellConflict =  FindConflict(cell, it->second, i );
        // If we identify a cell conflict, we stop there, since we need the earliset confloct time.
        if(!cellConflict.Empty()) {
          m_conflictMap[cellConflict.timestep] = cellConflict;
          if(this->m_debug) {  
            std::cout << "CONFLICT found at timestep " << cellConflict.timestep << " ,at cell " 
              << cellConflict.cellIndex << " involving robots " << std::endl;
            for(auto& robot : cellConflict.robots)
              std::cout << robot->GetLabel() << "\t";
            std::cout << std::endl; 
          }
          break;
        }
      } 
      
    } else {
      // Case 3: This cell has not been occupied yet, there is no need for checking it for conclicts
      m_cellsIntervals[cell][_robot][_minTime] = _maxTime;
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
  // _t is not added, the we have to add it.
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
  return CellConflict(_cellIndex, _timestep, robots);
}


template<typename MPTraits>
typename GridIntervalMap<MPTraits>::CellConflict
GridIntervalMap<MPTraits>::
FindEarliestConflict(){

  typename GridIntervalMap<MPTraits>::CellConflict cellConflict;
  if(!m_conflictMap.empty()) {
    cellConflict = m_conflictMap.begin()->second;
 
    std::cout << "CONFLICT found at timestep " << cellConflict.timestep << " ,at cell " 
      << cellConflict.cellIndex << " involving robots " << std::endl;
    for(auto& robot : cellConflict.robots)
      std::cout << robot->GetLabel() << "\t";
    std::cout << std::endl; 
          
  } else {
    std::cout << "NO CONFLICTS FOUND" << std::endl;
  }
  return cellConflict;

}


template<typename MPTraits>
void
GridIntervalMap<MPTraits>::
ClearConflicts(){
  m_conflictMap.clear();
  m_cellsIntervals.clear();
}


/*----------------------------------------------------------------------------*/

#endif
