#include "LocalObstacleMap.h"

#include <unordered_map>
#include <unordered_set>

/*------------------------------- Construction -------------------------------*/

LocalObstacleMap::
LocalObstacleMap(StatClass* const _stats) : m_stats(_stats) { }

/*-------------------------------- Accessors ---------------------------------*/

const typename LocalObstacleMap::VertexSet&
LocalObstacleMap::
Get(const VID& _vid) const {
  // If _vid isn't in the LOM, this call initializes its empty map.
  auto& map = const_cast<MapType&>(m_map);
  return map[_vid];
}


template <typename ContainerType>
const typename LocalObstacleMap::VertexSet
LocalObstacleMap::
Get(const ContainerType& _c) const {
  VertexSet aggregate;
  for(const auto& vid : _c) {
    const auto& lom = Get(vid);
    aggregate.insert(lom.begin(), lom.end());
  }
  return aggregate;
}


const typename LocalObstacleMap::VertexSet&
LocalObstacleMap::
Inverse(const VID& _vid) const {
  // If _vid isn't in the inverse LOM, this call initializes its empty map.
  auto& map = const_cast<MapType&>(m_inverse);
  return map[_vid];
}


template <typename ContainerType>
const typename LocalObstacleMap::VertexSet
LocalObstacleMap::
Inverse(const ContainerType& _c) const {
  VertexSet aggregate;
  for(const auto& vid : _c) {
    const auto& inv = Inverse(vid);
    aggregate.insert(inv.begin(), inv.end());
  }
  return aggregate;
}

/*-------------------------------- Addition ----------------------------------*/

void
LocalObstacleMap::
Add(const VID& _obst, const VID& _free) {
  MethodTimer mt(m_stats, "LocalObstacleMap::Add");

  // Try to add the obstacle node to the map
  auto result = m_map[_free].insert(_obst);

  // If success, update the inverse map as well.
  if(result.second) {
    m_inverse[_obst].insert(_free);
    if(s_debug)
      std::cout << "\tAdding obstacle node " << _obst
                << " to the LOM of free node " << _free << "."
                << std::endl;
  }
  else if(s_debug)
    std::cout << "\tObstacle node " << _obst << " already exists in the LOM of "
              << "free node " << _free << ", not adding."
              << std::endl;
}


template <typename ContainerType>
void
LocalObstacleMap::
Add(const ContainerType& _obst, const VID& _free) {
  for(const auto& vid : _obst)
    Add(vid, _free);
}


template <typename ContainerType>
void
LocalObstacleMap::
Add(const VID& _obst, const ContainerType& _free) {
  for(const auto& vid : _free)
    Add(_obst, vid);
}


template <typename ContainerType>
void
LocalObstacleMap::
Add(const ContainerType& _obst, const ContainerType& _free) {
  for(const auto& vid : _free)
    Add(_obst, vid);
}

/*------------------------------- Deletion -----------------------------------*/

void
LocalObstacleMap::
Delete(const VID& _obst, const VID& _free) {
  MethodTimer mt(m_stats, "LocalObstacleMap::Delete");

  const bool clearAll = _obst == INVALID_VID,
             fromAll  = _free == INVALID_VID;

  if(s_debug)
    std::cout << "\tAttempting to delete "
              << (clearAll ? "all obstacle nodes" :
                             "obstacle node " + std::to_string(_obst))
              << " from "
              << (fromAll ? "all local obstacle maps... " :
                            std::to_string(_free) + "'s local obstacle map...")
              << std::endl;

  // If not clearing all, check that the obstacle node exists
  if(!clearAll && !m_inverse.count(_obst))
    throw RunTimeException(WHERE,
        "The obstacle VID " + std::to_string(_obst) + " was not found!.");

  // If not clearing from all, check that the free node exists
  if(!fromAll && !m_map.count(_free))
    throw RunTimeException(WHERE,
        "The free VID " + std::to_string(_free) + " was not found!.");

  const unsigned short test = clearAll * 10 + fromAll;
  switch(test) {
    case 0:  // clear one obst VID from one free node
      m_map[_free].erase(_obst);
      m_inverse[_obst].erase(_free);
      break;
    case 1:  // clear one obst VID from all free nodes
      {
        auto iter = m_inverse.find(_obst);
        for(auto& free : iter->second)
          m_map[free].erase(_obst);
        m_inverse.erase(iter);
      }
      break;
    case 10: // clear all obst VIDs from one free node
      m_map.erase(_free);
      break;
    case 11: // clear all obst VIDs from all free nodes
      m_map.clear();
      m_inverse.clear();
  }

  if(s_debug)
    std::cout << "\t\tSuccess!" << std::endl;
}


template <typename ContainerType>
void
LocalObstacleMap::
Delete(const VID& _obst, const ContainerType& _free) {
  for(const auto& vid : _free)
    Delete(_obst, vid);
}


template <typename ContainerType>
void
LocalObstacleMap::
Delete(const ContainerType& _obst, const VID& _free) {
  for(const auto& vid : _obst)
    Delete(vid, _free);
}


template <typename ContainerType>
void
LocalObstacleMap::
Delete(const ContainerType& _obst, const ContainerType& _free) {
  for(const auto& vid : _free)
    Delete(_obst, vid);
}

/*----------------------------------------------------------------------------*/
