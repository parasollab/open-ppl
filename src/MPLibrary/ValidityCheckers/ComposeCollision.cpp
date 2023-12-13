#include "ComposeCollision.h"

#include "MPLibrary/MPLibrary.h"

#include "nonstd/io.h"
#include <algorithm>

/*------------------------------- Construction -------------------------------*/

ComposeCollision::
ComposeCollision() {
  this->SetName("ComposeCollision");
}


ComposeCollision::
ComposeCollision(XMLNode& _node) 
  : CollisionDetectionValidityMethod(_node) {
  this->SetName("ComposeCollision");

  std::string logicalOperator = _node.Read("operator", true, "", "operator");
  std::transform(logicalOperator.begin(), logicalOperator.end(),
                 logicalOperator.begin(), ::tolower);

  if(logicalOperator == "and")
    m_operator = AND;
  else if(logicalOperator == "or")
    m_operator = OR;
  else
    throw ParseException(_node.Where()) << "Operator '" << logicalOperator
                                        << "' is unknown.";

  for(auto& child : _node)
    if(child.Name() == "CollisionDetector")
      m_cdLabels.push_back(child.Read("label", true, "",
          "CollisionDetection method to include in this set."));

  if(m_cdLabels.size() < 2)
    throw ParseException(_node.Where()) << "Must specify at least two methods.";
}

/*---------------------- Collision Detection Overrides -----------------------*/

bool
ComposeCollision::
IsInsideObstacle(const Point3d& _p) {
  if(this->m_debug)
    std::cout << "ComposeCollision:: checking inside obstacle..."
              << std::endl;

  switch(m_operator) {
    case AND:
      for(auto label : m_cdLabels) {
        auto basevc = this->GetMPLibrary()->GetValidityChecker(label);
        auto vc = dynamic_cast<CollisionDetectionValidityMethod*>(basevc);
        const bool inside = vc->IsInsideObstacle(_p);

        if(this->m_debug)
          std::cout << "\t" << vc->GetNameAndLabel() << ": "
                    << (inside ? "" : "not ")
                    << "inside obstacle"
                    << std::endl;

        if(!inside)
          return false;
      }
      return true;
    case OR:
      for(auto label : m_cdLabels) {
        auto basevc = this->GetMPLibrary()->GetValidityChecker(label);
        auto vc = dynamic_cast<CollisionDetectionValidityMethod*>(basevc);
        const bool inside = vc->IsInsideObstacle(_p);

        if(this->m_debug)
          std::cout << "\t" << vc->GetNameAndLabel() << ": "
                    << (inside ? "" : "not ")
                    << "inside obstacle"
                    << std::endl;

        if(inside)
          return true;
      }
      return false;
    default:
      throw RunTimeException(WHERE, "Unknown operator is stated.");
  }
  return false;
}


bool
ComposeCollision::
IsInsideObstacle(const Point3d& _p, std::vector<size_t>* _obstIdxs) {
  throw NotImplementedException(WHERE);
  return false;
}


bool
ComposeCollision::
WorkspaceVisibility(const Point3d& _a, const Point3d& _b) {
  if(this->m_debug)
    std::cout << "ComposeCollision:: checking workspace visibility..."
              << std::endl;

  switch(m_operator) {
    case AND:
      for(auto label : m_cdLabels) {
        auto basevc = this->GetMPLibrary()->GetValidityChecker(label);
        auto vc = dynamic_cast<CollisionDetectionValidityMethod*>(basevc);
        const bool visible = vc->WorkspaceVisibility(_a, _b);

        if(this->m_debug)
          std::cout << "\t" << vc->GetNameAndLabel() << ": "
                    << (visible ? "" : "not ")
                    << "visible"
                    << std::endl;

        if(!visible)
          return false;
      }
      return true;
    case OR:
      for(auto label : m_cdLabels) {
        auto basevc = this->GetMPLibrary()->GetValidityChecker(label);
        auto vc = dynamic_cast<CollisionDetectionValidityMethod*>(basevc);
        const bool visible = vc->WorkspaceVisibility(_a, _b);

        if(this->m_debug)
          std::cout << "\t" << vc->GetNameAndLabel() << ": "
                    << (visible ? "" : "not ")
                    << "visible"
                    << std::endl;

        if(visible)
          return true;
      }
      return false;
    default:
      throw RunTimeException(WHERE, "Unknown operator is stated.");
  }
  return false;
}


bool
ComposeCollision::
IsMultiBodyCollision(CDInfo& _cdInfo, const MultiBody* const _a,
    const MultiBody* const _b, const std::string& _caller) {

  if(this->m_debug)
    std::cout << "ComposeCollision:: checking multibody collision..."
              << std::endl;

  switch(m_operator) {
    case AND:
      for(auto label : m_cdLabels) {
        auto basevc = this->GetMPLibrary()->GetValidityChecker(label);
        auto vc = dynamic_cast<CollisionDetectionValidityMethod*>(basevc);
        const bool collision = vc->IsMultiBodyCollision(_cdInfo, _a, _b, _caller);

        if(this->m_debug)
          std::cout << "\t" << vc->GetNameAndLabel() << ": "
                    << (collision ? "" : "not ")
                    << "collision"
                    << std::endl;

        if(!collision)
          return false;
      }
      return true;
    case OR:
      for(auto label : m_cdLabels) {
        auto basevc = this->GetMPLibrary()->GetValidityChecker(label);
        auto vc = dynamic_cast<CollisionDetectionValidityMethod*>(basevc);
        const bool collision = vc->IsMultiBodyCollision(_cdInfo, _a, _b, _caller);

        if(this->m_debug)
          std::cout << "\t" << vc->GetNameAndLabel() << ": "
                    << (collision ? "" : "not ")
                    << "collision"
                    << std::endl;

        if(collision)
          return true;
      }
      return false;
    default:
      throw RunTimeException(WHERE, "Unknown operator is stated.");
  }
  return false;
}

/*--------------------- ValidityCheckerMethod Overrides ----------------------*/

bool
ComposeCollision::
IsValidImpl(Cfg& _cfg, CDInfo& _cdInfo, const std::string& _caller) {
  if(this->m_debug)
    std::cout << "ComposeCollision:: checking validity..."
              << std::endl;

  switch(m_operator) {
    case AND:
      for(auto label : m_cdLabels) {
        auto vc = this->GetMPLibrary()->GetValidityChecker(label);
        const bool passed = vc->IsValid(_cfg, _cdInfo, _caller);

        if(this->m_debug)
          std::cout << "\t" << vc->GetNameAndLabel() << ": "
                    << (passed ? "passed" : "failed")
                    << std::endl;

        if(!passed)
          return false;
      }
      return true;
    case OR:
      for(auto label : m_cdLabels) {
        auto vc = this->GetMPLibrary()->GetValidityChecker(label);
        const bool passed = vc->IsValid(_cfg, _cdInfo, _caller);

        if(this->m_debug)
          std::cout << "\t" << vc->GetNameAndLabel() << ": "
                    << (passed ? "passed" : "failed")
                    << std::endl;

        if(passed)
          return true;
      }
      return false;
    default:
      throw RunTimeException(WHERE, "Unknown operator is stated.");
  }
  return false;
}


bool
ComposeCollision::
IsValidImpl(GroupCfg& _cfg, CDInfo& _cdInfo, const std::string& _caller) {
  if(this->m_debug)
    std::cout << "ComposeCollision:: checking validity..."
              << std::endl;

  switch(m_operator) {
    case AND:
      for(auto label : m_cdLabels) {
        auto vc = this->GetMPLibrary()->GetValidityChecker(label);
        const bool passed = vc->IsValid(_cfg, _cdInfo, _caller);

        if(this->m_debug)
          std::cout << "\t" << vc->GetNameAndLabel() << ": "
                    << (passed ? "passed" : "failed")
                    << std::endl;

        if(!passed)
          return false;
      }
      return true;
    case OR:
      for(auto label : m_cdLabels) {
        auto vc = this->GetMPLibrary()->GetValidityChecker(label);
        const bool passed = vc->IsValid(_cfg, _cdInfo, _caller);

        if(this->m_debug)
          std::cout << "\t" << vc->GetNameAndLabel() << ": "
                    << (passed ? "passed" : "failed")
                    << std::endl;

        if(passed)
          return true;
      }
      return false;
    default:
      throw RunTimeException(WHERE, "Unknown operator is stated.");
  }
  return false;
}
