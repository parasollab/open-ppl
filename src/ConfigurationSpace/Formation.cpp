#include "Formation.h"

/*-------------------------------- Construction ------------------------------*/

Formation::
Formation(std::vector<Robot*> _robots, Robot* _leader, 
    std::unordered_map<Robot*,FormationConstraint> _constraintMap) 
    : m_robots(_robots), m_leader(_leader), m_constraintMap(_constraintMap) {

  BuildMultiBody(); 
  InitializePlanningSpace();

}

Formation::
~Formation() { }

/*---------------------------------- Interface -------------------------------*/

std::vector<Cfg>
Formation::
RandomFormationCfg(const Boundary* const _b) {

  std::vector<double> dofs(m_multibody->DOF());

  /// Currently only support workspace boundaries
  if(_b->Type() != Boundary::Space::Workspace) 
    throw RunTimeException(WHERE) << "Random Formation currently only supports "
                                     "workspace boundaries.";

  // Determine how many DOF value will be generated from _b.
  size_t numDof = std::min(_b->GetDimension(),dofs.size());

  // If _b is a workspace boundary, use no more values than boundary size.
  if(_b->Type() == Boundary::Space::Workspace) {
    numDof = std::min(numDof, m_multibody->PosDOF());
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
      dofs[i] = m_cspace->GetRange(i).Sample();
    }

    // The result is valid if it satisfies all the boundaries.
    if(!m_cspace->InBoundary(dofs))
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
                                << "\nFormation Cspace: " << *(m_cspace.get())
                                << "\nSampling Boundary: " << *_b
                                << "\nFormationRadius: " 
                                << m_multibody->GetBoundingSphereRadius();

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
  std::unordered_map<Body*,std::vector<std::pair<Connection*,Body*>> tree;

  // Add leader to the tree.
  auto mb = m_leader->GetMultiBody();
  for(auto body : mb->GetBodies()) {
    for(size_t i = 0; i < body->ForwardConnectionCount(); i++) {
      auto connection = body->GetForwardConnection(i);
      tree[body].push_back(std::make_pair(connection,connection->GetNextBody()));
    }
  }

  for(auto robot : m_robots) {
    if(robot == m_leader)
      continue;

    mb = robot->GetMultiBody();

    // Find new robot base
    auto constraint = m_constraintMap[robot];
    auto base = constraint.dependentBody;

    // Add base to tree. Nullptr indicates it comes from constraint.
    tree[constraint.referenceBody] = std::make_pair(nullptr,base);

    // Trace new base back to original
    auto body = base;
    std::set<Connection*> addedConnections;
    while(body != mb->GetBase()) {
      auto oldBody = body;
      // Assuming only one backward connection.
      auto connection = body->GetBackwardConnection(0);
      body = connection->GetPreviousBody();

      // Add connection to tree. Will identify inverted connection later.
      tree[oldBody].push_back(std::make_pair(connection,body));

      addedConnections.insert(connection);
    }

    // Fill in rest of bodies and connections
    for(auto b : mb->GetBodies()) {
      for(size_t i = 0; i < body->ForwardConnectionCount(); i++) {

        auto connection = body->GetForwardConnection(i);
        
        // Check that this was not added on the backtrace.
        if(addedConnections.count(connection))
          continue;

        tree[body] = std::make_pair(connection,connection->GetNextBody());
      }
    }
  }

  // Convert tree into multibody
  m_multibody = std::unique_ptr<MultiBody>(new MultiBody(MultiBody::Type::Active));
  auto body = m_leader->GetMultiBody()->GetBase();
  AddBody(body,true); 

  std::set<Body*> addedBodies;
  addedBodies.insert(base);

  // Add all bodies
  for(auto kv : tree) {
    auto source = kv.first;

    if(!addedBodies.count(source)) {
      AddBody(source)
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

  m_bodyMap[_body] = m_multibody->AddBody(std::move(body));
}

void
Formation
AddConnection(Body* _first, Body* _second, Connection* _connection) {
  // Check if connection is to be copied or generated from constraint
  if(!_connection) {
    // Create connection from constraint
    return;
  }

  // Copy connection
  Connection connection = *_connection;
 
  bool sameDirection = true;
 
  // Check if connection is in the proper order;
  if(_connection.GetPreviousBody() != _first) {
    _connection.InvertConnection();
    sameDirection = false;
  }

  // Update body pointers
  auto first = m_bodyMap[_first];
  auto second = m_bodyMap[_second];

  connection.SetBodies(m_multibody,first,second);

  m_linkMap = std::make_pair(sameDirection,
            m_multibody->AddConnection(std::move(connection)));
}

void
Formation::
InitializePlanningSpace() {

  // Intialize the configuration space of the formation
  m_cspace = std::unique_ptr<CSpaceBoundingBox>(new CSpaceBoundingBox(m_multibody->DOF()));
  
  const auto& dofInfo = m_multibody->GetDofInfo();

  for(size_t i = 0; i < m_multibody->DOF(); i++) {
    m_cspace->SetRange(i, dofInfo[i].range);
  }
}
    
std::vector<double>
Formation::
ConvertToFormationDOF(std::vector<Cfg> _cfgs) {
  return {};
}
    
std::vector<Cfg>
Formation::
ConvertToIndividualCfgs(std::vector<double> _dofs) {
  return {};
}

/*----------------------------------------------------------------------------*/
