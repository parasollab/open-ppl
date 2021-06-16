#include "Formation.h"


/*-------------------------------- Construction ------------------------------*/

Formation::
Formation(std::vector<Robot*> _robots, Robot* _leader, 
    std::unordered_map<MultiBody*,FormationConstraint> _constraintMap) 
    : m_robots(_robots), m_leader(_leader), m_constraintMap(_constraintMap), 
      m_multibody(MultiBody::Type::Active), m_cspace(0) {

  BuildMultiBody(); 
  InitializePlanningSpace();

}

Formation::
Formation(const Formation& _formation) :
  m_robots(_formation.m_robots),
  m_leader(_formation.m_leader),
  m_constraintMap(_formation.m_constraintMap),
  m_multibody(_formation.m_multibody),
  m_cspace(_formation.m_cspace),
  m_jointMap(_formation.m_jointMap),
  m_reverseJointMap(_formation.m_reverseJointMap),
  m_bodyMap(_formation.m_bodyMap) 
{ 
  //BuildMultiBody(); 
  //InitializePlanningSpace();
}

Formation::
Formation(Formation&& _formation) :
  m_robots(std::move(_formation.m_robots)),
  m_leader(std::move(_formation.m_leader)),
  m_constraintMap(std::move(_formation.m_constraintMap)),
  m_multibody(std::move(_formation.m_multibody)),
  m_cspace(std::move(_formation.m_cspace)),
  m_jointMap(std::move(_formation.m_jointMap)),
  m_reverseJointMap(std::move(_formation.m_reverseJointMap)),
  m_bodyMap(std::move(_formation.m_bodyMap)) 
{
  //BuildMultiBody(); 
  //InitializePlanningSpace();
}

Formation::
~Formation() {};
/*--------------------------------- Assignment -------------------------------*/

Formation&
Formation::
operator=(const Formation& _formation) {
  m_robots.clear();
  m_robots = _formation.m_robots;

  m_leader = _formation.m_leader;

  m_multibody = _formation.m_multibody;

  m_cspace = _formation.m_cspace;

  m_constraintMap.clear();
  m_constraintMap = _formation.m_constraintMap;

  m_jointMap.clear();
  m_jointMap = _formation.m_jointMap;

  m_reverseJointMap.clear();
  m_reverseJointMap = _formation.m_reverseJointMap;

  m_bodyMap.clear();
  m_bodyMap = _formation.m_bodyMap;

  return *this;
}

Formation&
Formation::
operator=(Formation&& _formation) {
  m_robots.clear();
  m_robots = std::move(_formation.m_robots);

  m_leader = std::move(_formation.m_leader);

  m_multibody = std::move(_formation.m_multibody);

  m_cspace = std::move(_formation.m_cspace);

  m_constraintMap.clear();
  m_constraintMap = std::move(_formation.m_constraintMap);

  m_jointMap.clear();
  m_jointMap = std::move(_formation.m_jointMap);

  m_reverseJointMap.clear();
  m_reverseJointMap = std::move(_formation.m_reverseJointMap);

  m_bodyMap.clear();
  m_bodyMap = std::move(_formation.m_bodyMap);

  return *this;
}

/*---------------------------------- Interface -------------------------------*/

std::vector<Cfg>
Formation::
GetRandomFormationCfg(const Boundary* const _b) {

  std::vector<double> dofs(m_multibody.DOF());

  /// Currently only support workspace boundaries
  if(_b->Type() != Boundary::Space::Workspace) 
    throw RunTimeException(WHERE) << "Random Formation currently only supports "
                                     "workspace boundaries.";

  // Determine how many DOF value will be generated from _b.
  size_t numDof = std::min(_b->GetDimension(),dofs.size());

  // If _b is a workspace boundary, use no more values than boundary size.
  if(_b->Type() == Boundary::Space::Workspace) {
    numDof = std::min(numDof, m_multibody.PosDOF());
  }

  for(size_t tries = 1000; tries > 0; tries--) {
    // Generate point in the boundary.
    auto sample = _b->GetRandomPoint();

    // Copy generated DOF values to dofs
    for(size_t i = 0; i < numDof; i++) {
      dofs[i] = sample[i];
    }

    // For each DOF that was not generated from sampling the boundary,
    // generate a value from the formation CSpace.
    for(size_t i = numDof; i < dofs.size(); i++) {
      dofs[i] = m_cspace.GetRange(i).Sample();
    }

    // The result is valid if it satisfies all the boundaries.
    if(!m_cspace.InBoundary(dofs))
      continue;

    auto cfgs = ConvertToIndividualCfgs(dofs);
    bool valid = true;
  
    for(auto cfg : cfgs) {
      if(_b->InBoundary(cfg.GetData()))
        continue;
      valid = false;
      break;      
    }

    if(valid)
      return cfgs;
  }
  
  throw RunTimeException(WHERE) << "Could not generate a random formation."
                                << "\nFormation Cspace: " << m_cspace
                                << "\nSampling Boundary: " << *_b
                                << "\nFormationRadius: " 
                                << m_multibody.GetBoundingSphereRadius();

  return {};
}

std::vector<Cfg>
Formation::
FindIncrement(std::vector<Cfg> _start, std::vector<Cfg> _goal, 
    const size_t _nTicks) {
  return {};
}

/*------------------------------- Helper Functions ---------------------------*/

void
Formation::
BuildMultiBody() {

  // Build multibody tree  
  std::unordered_map<Body*,std::vector<std::pair<Connection,Body*>>> tree;

  // Add leader to the tree.
  auto mb = m_leader->GetMultiBody();
  for(size_t j = 0; j < mb->GetNumBodies(); j++) {
    auto body = mb->GetBody(j);

    for(size_t i = 0; i < body->ForwardConnectionCount(); i++) {
      auto connection = body->GetForwardConnection(i);
      tree[body].push_back(std::make_pair(connection,connection.GetNextBody()));
    }
  }

  for(auto robot : m_robots) {
    if(robot == m_leader)
      continue;

    mb = robot->GetMultiBody();

    // Find new robot base
    auto constraint = m_constraintMap[robot->GetMultiBody()];
    auto base = constraint.dependentBody;

    // Add base to tree. Empty connection with nullptr mb indicates 
    // it comes from constraint.
    Connection empty(nullptr);
    tree[constraint.referenceBody].push_back(std::make_pair(empty,base));

    // Trace new base back to original
    auto body = base;
    std::set<std::pair<Body*,Body*>> addedConnections;
    while(body != mb->GetBase()) {
      auto oldBody = body;
      // Assuming only one backward connection.
      auto connection = body->GetBackwardConnection(0);
      body = connection.GetPreviousBody();

      // Add connection to tree. Will identify inverted connection later.
      tree[oldBody].push_back(std::make_pair(connection,body));

      addedConnections.insert(std::make_pair(body,oldBody));
    }

    // Fill in rest of bodies and connections
    for(auto b : mb->GetBodies()) {
      for(size_t i = 0; i < body->ForwardConnectionCount(); i++) {

        auto connection = body->GetForwardConnection(i);
        
        // Check that this was not added on the backtrace.
        auto added = std::make_pair(body,connection.GetNextBody());
        if(addedConnections.count(added))
          continue;

        addedConnections.insert(added);

        tree[body].push_back(std::make_pair(connection,connection.GetNextBody()));
      }
    }
  }

  // Convert tree into multibody
  m_multibody = MultiBody(MultiBody::Type::Active);
  auto body = m_leader->GetMultiBody()->GetBase();
  AddBody(body,true); 

  std::set<Body*> addedBodies;
  addedBodies.insert(body);

  // Add all bodies
  for(auto kv : tree) {
    auto source = kv.first;

    if(!addedBodies.count(source)) {
      AddBody(source);
    }

    for(auto p : kv.second) { 
      auto target = p.second;
      if(!addedBodies.count(target)) {
        AddBody(target);
      }
    }
  } 

  // Add all connections
  for(auto kv : tree) {
    auto source = kv.first;

    for(auto p : kv.second) {
      auto target = p.second;
      auto connection = p.first;

      AddConnection(source, target, connection);
    }
  }
}

void
Formation::
AddBody(Body* _body, bool _base) {
  Body body = *_body;

  // Check if base information is correct.
  if(_base != body.IsBase()) {
    if(!_base) {
      body.SetBodyType(Body::Type::Joint);
      body.SetMovementType(Body::MovementType::Joint);
    }
    else {
      throw RunTimeException(WHERE) << "Assumed leader base would be formation base.";
    }
  }

  m_bodyMap[_body] = m_multibody.AddBody(std::move(body));
}

void
Formation::
AddConnection(Body* _first, Body* _second, Connection& _connection) {

  Connection connection(&m_multibody);
  bool sameDirection = true;

  // Check if connection is to be copied or generated from constraint
  if(!connection.GetMultiBody()) {
    // Create connection from constraint
    auto constraint = m_constraintMap[_first->GetMultiBody()];
    Transformation toDHFrame = constraint.transformation;
    DHParameters dh;
    Transformation identity;
    Connection connection(&m_multibody,identity,toDHFrame,
                          dh,Connection::JointType::NonActuated);
  }
  else {
    // Copy connection
    Connection connection = _connection;

    // Check if connection is in the proper order;
    if(connection.GetPreviousBody() != _first) {
      connection.InvertConnection();
      sameDirection = false;
    }
  }

  // Update body pointers
  auto first = m_bodyMap[_first];
  auto second = m_bodyMap[_second];

  connection.SetBodies(&m_multibody,first,second);

  auto c = _first->GetConnectionTo(_second);

  auto index = m_multibody.AddJoint(std::move(connection));
  m_jointMap[c] = std::make_pair(sameDirection,index);
  m_reverseJointMap[index] = c;
}

void
Formation::
InitializePlanningSpace() {

  // Intialize the configuration space of the formation
  m_cspace = CSpaceBoundingBox(m_multibody.DOF());
  
  const auto& dofInfo = m_multibody.GetDofInfo();

  for(size_t i = 0; i < m_multibody.DOF(); i++) {
    m_cspace.SetRange(i, dofInfo[i].range);
  }
}
    
std::vector<double>
Formation::
ConvertToFormationDOF(std::vector<Cfg> _cfgs) {
  std::vector<double> dofs(m_multibody.DOF());

  std::unordered_map<size_t,double> indexedValues;

  for(auto cfg : _cfgs) {
    auto mb = cfg.GetRobot()->GetMultiBody();

    if(cfg.GetRobot() == m_leader) {
      // Extract base values
    }

    size_t counter = 0;
    for(const auto& joint : mb->GetJoints()) {
      auto j = joint.get();

      if(j->GetConnectionType() == Connection::JointType::NonActuated) 
        continue;
      
      auto index = m_jointMap[j].second;
      indexedValues[index] = cfg[counter];

      counter++;
    }
  }

  size_t counter = 0;
  
  for(size_t i = 0; i < m_multibody.GetJoints().size(); i++) {

    auto joint = m_multibody.GetJoint(i);

    if(joint->GetConnectionType() == Connection::JointType::NonActuated)
      continue;

    auto value = indexedValues[i];

    dofs[counter] = value;
    counter++;
  }

  return dofs;
}
    
std::vector<Cfg>
Formation::
ConvertToIndividualCfgs(std::vector<double> _dofs) {

  std::vector<Cfg> cfgs;

  std::unordered_map<Connection*,double> valueMap;

  size_t counter = 0;

  for(size_t i = 0; i < m_multibody.GetJoints().size(); i++) {
    auto joint = m_multibody.GetJoint(i);

    if(joint->GetConnectionType() == Connection::JointType::NonActuated)
      continue;

    auto connection = m_reverseJointMap[i];

    //TODO::Modify value if inverted.
    auto value = _dofs[counter];
    valueMap[connection] = value;

    counter++;
  }

  for(auto robot : m_robots) {
    auto mb = robot->GetMultiBody();
    std::vector<double> dofs(mb->DOF());

    counter = 0;
    for(const auto& joint : mb->GetJoints()) {
      if(joint->GetConnectionType() == Connection::JointType::NonActuated)
        continue;

      auto value = valueMap[joint.get()];
      dofs[counter] = value;

      counter++;
    }

    Cfg cfg(robot);
    cfg.SetData(dofs);

    cfgs.push_back(cfg);
  }

  return cfgs;
}

/*----------------------------------------------------------------------------*/
