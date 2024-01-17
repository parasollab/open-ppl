#ifndef LOCAL_OBSTACLE_MAP_H_
#define LOCAL_OBSTACLE_MAP_H_

#ifndef INVALID_VID
#define INVALID_VID (std::numeric_limits<size_t>::max())
#endif

#ifndef INVALID_EID
#define INVALID_EID (std::numeric_limits<size_t>::max())
#endif

#include "ConfigurationSpace/GenericStateGraph.h"
#include "ConfigurationSpace/Cfg.h"
#include "ConfigurationSpace/Weight.h"
#include "Utilities/MetricUtils.h"
#include "Utilities/PMPLExceptions.h"

#include <unordered_map>
#include <unordered_set>


////////////////////////////////////////////////////////////////////////////////
/// Maintains an associative mapping between free and obstacle configurations
/// to avoid running a neighborhood finder on the obstacle map (which is
/// typically huge).
////////////////////////////////////////////////////////////////////////////////
class LocalObstacleMap final {

  public:

    ///@name Motion Planning Types
    ///@{
    
    typedef GenericStateGraph<Cfg, DefaultWeight<Cfg>> RoadmapType;
    typedef size_t VID;

    ///@}
    ///@name Local Types
    ///@{

    typedef std::unordered_set<VID>            VertexSet;
    typedef std::unordered_map<VID, VertexSet> MapType;

    ///@}
    ///@name Construction
    ///@{

    LocalObstacleMap(StatClass* const _stats);

    ///@}
    ///@name Accessors
    ///@{

    /// Get the local obstacle map for a given free vertex.
    /// @param _vid A free vertex.
    /// @return The local obstacle map for _vid.
    const VertexSet& Get(const VID& _vid) const;

    /// Get the aggregate local obstacle map for a set of free vertices.
    /// @param _c A container of free vertices.
    /// @return The aggregate local obstacle map for all vertices in _c.
    template <typename ContainerType>
    const VertexSet Get(const ContainerType& _c) const;

    /// Get the inverse local obstacle map for a given obstacle vertex.
    /// @param _vid An obstacle vertex.
    /// @return The inverse obstacle map for _vid.
    const VertexSet& Inverse(const VID& _vid) const;

    /// Get the aggregate inverse local obstacle map for a set of obstacle
    /// vertices.
    /// @param _c A set of obstacle vertices.
    /// @return The aggregate inverse obstacle map for all vertices in _c.
    template <typename ContainerType>
    const VertexSet Inverse(const ContainerType& _c) const;

    ///@}
    ///@name Addition
    ///@{
    /// Add a new obstacle configuration to the local obstacle map of a given
    /// free node.
    /// @param _obst The new obstacle node to add.
    /// @param _free The free node to add it to.

    void Add(const VID& _obst, const VID& _free);

    template <typename ContainerType>
    void Add(const ContainerType& _obst, const VID& _free);

    template <typename ContainerType>
    void Add(const VID& _obst, const ContainerType& _free);

    template <typename ContainerType>
    void Add(const ContainerType& _obst, const ContainerType& _free);

    ///@}
    ///@name Deletion
    ///@{
    /// Delete an obstacle configuration from one or more local obstacle maps.
    /// @param _obst The obstacle VID(s) to delete, or INVALID_VID to delete all.
    /// @param _free The free VID(s) to delete from, or INVALID_VID to delete
    ///              from all nodes.

    void Delete(const VID& _obst = INVALID_VID, const VID& _free = INVALID_VID);

    template <typename ContainerType>
    void Delete(const VID& _obst, const ContainerType& _free);

    template <typename ContainerType>
    void Delete(const ContainerType& _obst, const VID& _free = INVALID_VID);

    template <typename ContainerType>
    void Delete(const ContainerType& _obst, const ContainerType& _free);

    ///@}

  private:

    ///@name Internal State
    ///@{

    MapType m_map;     ///< Maps free to obstacle cfgs.
    MapType m_inverse; ///< Maps obstacle to free cfgs.

    StatClass* m_stats{nullptr};          ///< The stat class.

    static constexpr bool s_debug{false}; ///< Enable debugging messages?

    ///@}
};

#endif
