#include "ConnectorMethod.h"

#include "MPLibrary/MPLibrary.h"

#include <iterator>
#include <set>

/*------------------------------- Construction -------------------------------*/

ConnectorMethod::
ConnectorMethod()
{ }


ConnectorMethod::
ConnectorMethod(XMLNode& _node) : MPBaseObject(_node) {
  m_lpLabel = _node.Read("lpLabel", true, "", "Local Planner");

  m_skipIfSameCC = _node.Read("checkIfSameCC", false, m_skipIfSameCC,
      "Skip connections between nodes in the same CC?");

  m_maxFailures = _node.Read("maxFailures", false, m_maxFailures,
      size_t(0), std::numeric_limits<size_t>::max(),
      "Terminate Connect operations after this many failures (0 to disable).");

	m_selfEdges = _node.Read("selfEdges", false, m_selfEdges,
			"Indicates if the connector should allow self edges.");

  m_oneWay = _node.Read("unidirectional", false, m_oneWay,
      "Create unidirectional edges.");
}

/*--------------------------- MPBaseObject Overrides -------------------------*/

void
ConnectorMethod::
Print(std::ostream& _os) const {
  MPBaseObject::Print(_os);
  _os << "\n\tlpLabel: " << m_lpLabel
      << "\n\tskip if same cc: " << m_skipIfSameCC
      << "\n\tmax failures: " << m_maxFailures
      << "\n\tunidirectional: " << m_oneWay
      << std::endl;
}


void
ConnectorMethod::
Initialize() {
  ClearConnectionAttempts();
}

/*---------------------------- Connection Interface --------------------------*/

bool
ConnectorMethod::
IsRewiring() const noexcept {
  return m_rewiring;
}

/*--------------------------- Connection Helpers -----------------------------*/

void
ConnectorMethod::
ConnectImpl(RoadmapType* const _r, const VID _source,
    const VertexSet* const _targetSet,
    OutputIterator<RoadmapType>* const _collision) {
  throw NotImplementedException(WHERE);
}


void
ConnectorMethod::
ConnectImpl(GroupRoadmapType* const _r, const VID _source,
    const VertexSet* const _targetSet,
    OutputIterator<GroupRoadmapType>* const _collision) {
  throw NotImplementedException(WHERE);
}


bool
ConnectorMethod::
ConnectNodes(RoadmapType* const _r, const VID _source, const VID _target,
    OutputIterator<RoadmapType>* const _collision) {
  auto env = this->GetEnvironment();
  auto robot = this->GetTask()->GetRobot();
  auto lp = this->GetMPLibrary()->GetLocalPlanner(m_lpLabel);

  const Cfg& c1 = _r->GetVertex(_source),
               & c2 = _r->GetVertex(_target);

  Cfg collision(robot);
  LPOutput lpOutput;
  const bool connected = lp->IsConnected(c1, c2, collision, &lpOutput,
        env->GetPositionRes(), env->GetOrientationRes(), true);

  if(connected) {
    if(m_oneWay)
      _r->AddEdge(_source, _target, lpOutput.m_edge.first);
    else
      _r->AddEdge(_source, _target, lpOutput.m_edge);
  }
  else {
    CacheFailedConnection(_r, _source, _target);
    if(_collision)
      *_collision = collision;
  }

  return connected;
}


bool
ConnectorMethod::
ConnectNodes(GroupRoadmapType* const _r, const VID _source, const VID _target,
    OutputIterator<GroupRoadmapType>* const _collision) {
  auto env = this->GetEnvironment();
  auto lp = this->GetMPLibrary()->GetLocalPlanner(m_lpLabel);

  const GroupCfgType& c1 = _r->GetVertex(_source),
                    & c2 = _r->GetVertex(_target);

  GroupCfgType collision(_r);
  GroupLPOutput lpOutput(_r);
  const bool connected = lp->IsConnected(c1, c2, collision, &lpOutput,
        env->GetPositionRes(), env->GetOrientationRes(), true);

  if(connected) {
    if(m_oneWay)
      _r->AddEdge(_source, _target, lpOutput.m_edge.first);
    else
      _r->AddEdge(_source, _target, lpOutput.m_edge);
  }
  else {
    CacheFailedConnection(_r, _source, _target);
    if(_collision)
      *_collision = collision;
  }

  return connected;
}

/*------------------------------ Connection Caching --------------------------*/

void
ConnectorMethod::
CacheFailedConnection(void* const _map, const VID _source, const VID _target)
    noexcept {
  m_attemptsCache[_map].insert({_source, _target});
}


bool
ConnectorMethod::
IsCached(void* const _map, const VID _source, const VID _target) const noexcept {
  // Look up the cache for this map. If it hasn't been created yet, nothing is
  // cached.
  auto iter = m_attemptsCache.find(_map);
  const bool cached = iter == m_attemptsCache.end()
                    ? false
                    : iter->second.count({_source, _target});

  this->GetStatClass()->GetAverage(this->GetNameAndLabel() + "::CacheHitRatio")
      += cached;
  return cached;
}


void
ConnectorMethod::
ClearConnectionAttempts() {
  m_attemptsCache.clear();
}

/*----------------------------------------------------------------------------*/
