#include "KDLModel.h"

#include "Geometry/Bodies/Connection.h"
#include "Geometry/Bodies/MultiBody.h"

#include "MPProblem/Robot/Robot.h"

#include "Utilities/PMPLExceptions.h"

#undef PI
#include "Utilities/KDLParser.h"

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>

#include <trac_ik/trac_ik.hpp>

/*----------------------- Construction -----------------------*/

KDLModel::
KDLModel(Robot* _robot) : m_robot(_robot) {}

KDLModel::
KDLModel(Robot* _robot, const std::string _filename) : m_robot(_robot), m_filename(_filename) {}

KDLModel::
KDLModel(XMLNode& _node, Robot* _robot) : m_robot(_robot) {
  // TODO::Allow explicit specification of kdl chain/tree
  m_filename = _node.Read("filename",true,"","URDF file name.");

  m_chainRoot = _node.Read("chainRoot",true,"",
                                     "Root of chain to compute FK/IK");

  m_chainTip = _node.Read("chainTip",true,"",
                                    "Tip of chain to compute FK/IK");
}

KDLModel::
~KDLModel() { }

/*------------------------- Interface ------------------------*/

KDLModel::ToolFrame
KDLModel::
ForwardKinematics(std::vector<double> _jointAngles) {

  auto tree = ParseKDLFromFile(m_filename);

  KDL::Chain chain;

  bool ret = tree.getChain(m_chainRoot,m_chainTip,chain);

  if(!ret)
    throw RunTimeException(WHERE) << "Unable to construct chain from tree.";

  auto numJoints = chain.getNrOfJoints();

  if(!m_initialized) {

    // Collect set of bodies in chain
    auto mb = m_robot->GetMultiBody();
    auto linkMap = mb->GetLinkMap();
    auto numSegments = chain.getNrOfSegments();

    for(size_t i = 0; i < numSegments; i++) {
      auto name = chain.getSegment(i).getName();
      auto index = linkMap.at(name);
      m_segments.insert(index);
    }

    // Get joint angles within chain
    std::vector<double> jointAngles;
    auto index = 0;

    for(size_t i = 0; i < mb->GetJoints().size(); i++) {
      auto connection = mb->GetJoint(i);
      auto type = connection->GetConnectionType();
      if(type == Connection::JointType::NonActuated or type == Connection::JointType::Mimic)
        continue;

      auto previous = connection->GetPreviousBodyIndex();
      auto next = connection->GetNextBodyIndex();

      if(m_segments.count(previous) and m_segments.count(next)) {
        m_includedIndices[std::make_pair(previous,next)] = index;
      }

      index++;
    }

    if(numJoints != m_includedIndices.size())
      throw RunTimeException(WHERE) << "Mismatched number of joints.";

    for(size_t i = 1; i < numSegments; i++) {
      auto index = m_includedIndices[std::make_pair(i-1,i)];
      m_jointIndices.push_back(index);
    }

    m_initialized = true;
  }


  // jntarrays
  KDL::JntArray q(numJoints);
  for(size_t i = 0; i < m_jointIndices.size(); i++) {
    auto index = m_jointIndices[i];
    q(i) = _jointAngles[index] * KDL::PI;
  }

  // forward position solver
  KDL::ChainFkSolverPos_recursive fksolver1(chain);

  // Compute current tcp position
  KDL::Frame tcp_pos_start;
  fksolver1.JntToCart(q,tcp_pos_start);

  ToolFrame out;
  for(size_t i = 0; i < 3; i++) {
    out.pos.push_back(tcp_pos_start.p[i]);
  }

  for(size_t i = 0; i < 9; i++) {
    out.ori.push_back(tcp_pos_start.M.data[i]);
  }

  return out;
}

std::vector<double>
KDLModel::
InverseKinematics(std::vector<double> _pos, std::vector<double> _ori, std::vector<double> _initialGuess) {

  auto tree = ParseKDLFromFile(m_filename);

  KDL::Chain chain;

  bool ret = tree.getChain(m_chainRoot,m_chainTip,chain);

  if(!ret)
    throw RunTimeException(WHERE) << "Unable to construct chain from tree.";

  auto numJoints = chain.getNrOfJoints();

  if(!m_initialized) {

    // Collect set of bodies in chain
    auto mb = m_robot->GetMultiBody();
    auto linkMap = mb->GetLinkMap();
    auto numSegments = chain.getNrOfSegments();

    for(size_t i = 0; i < numSegments; i++) {
      auto name = chain.getSegment(i).getName();
      auto index = linkMap.at(name);
      m_segments.insert(index);
    }

    // Get joint angles within chain
    std::vector<double> jointAngles;
    auto index = 0;

    for(size_t i = 0; i < mb->GetJoints().size(); i++) {
      auto connection = mb->GetJoint(i);
      auto type = connection->GetConnectionType();
      if(type == Connection::JointType::NonActuated or type == Connection::JointType::Mimic)
        continue;

      auto previous = connection->GetPreviousBodyIndex();
      auto next = connection->GetNextBodyIndex();

      if(m_segments.count(previous) and m_segments.count(next)) {
        m_includedIndices[std::make_pair(previous,next)] = index;
      }

      index++;
    }

    if(numJoints != m_includedIndices.size())
      throw RunTimeException(WHERE) << "Mismatched number of joints.";

    for(size_t i = 1; i < numSegments; i++) {
      auto index = m_includedIndices[std::make_pair(i-1,i)];
      m_jointIndices.push_back(index);
    }

    m_initialized = true;
  }


  // jntarrays
  KDL::JntArray q(numJoints);
  KDL::JntArray qInit(numJoints);
  for(size_t i = 0; i < m_jointIndices.size(); i++) {
    auto index = m_jointIndices[i];
    qInit(i) = _initialGuess[index] * KDL::PI;
  }

  // forward position solver
  KDL::ChainFkSolverPos_recursive fksolver1(chain);

  // inverse velocity solver
  KDL::ChainIkSolverVel_pinv iksolver1v(chain, 0.0001, 1000);

  // inverse kinematics solver
  KDL::ChainIkSolverPos_NR iksolverpos(chain,fksolver1,iksolver1v,10000,
                                    std::numeric_limits<double>::epsilon());


  size_t maxAttempts = 1;

  for(size_t i = 0; i < maxAttempts; i++) {
    // jntarrays
    KDL::JntArray q(chain.getNrOfJoints());

    // Compute current tcp position
    KDL::Frame tcp_pos_start;
    fksolver1.JntToCart(qInit,tcp_pos_start);


    // destination frame
    KDL::Vector vector(_pos[0],_pos[1],_pos.size() == 3 ? _pos[2] : 0);

    KDL::Rotation rotation(_ori[0],_ori[1],_ori[2],_ori[3],_ori[4],_ori[5],_ori[6],_ori[7],_ori[8]);

    KDL::Frame fDestination(rotation,vector);

    auto ret2 = iksolverpos.CartToJnt(qInit,fDestination,q);;

    auto retString = iksolverpos.strError(ret2);

    std::cout << retString << std::endl;

    if(ret2 < 0)
      continue;

    // TODO::Sort these alphabetically bc :(
    std::vector<double> dofs = _initialGuess;
    auto numSegments = chain.getNrOfSegments();
    for(size_t i = 0; i < numSegments-1; i++) {
      // Convert to pmpl joint values and add to dof
      //dofs.push_back(q(i)/KDL::PI);

      auto d = q(i)/KDL::PI;

      double extraRot = std::floor(d/2);
      d = d - extraRot*2;

      if(d > 1) {
        d = d - 2;
      }
      else if(d < -1) {
        d = d + 2;
      }

      auto index = m_includedIndices[std::make_pair(i,i+1)];
      dofs[index] = d;
    }

    KDL::Frame tcp_pos_end;
    fksolver1.JntToCart(q,tcp_pos_end);

    return dofs;
  }

  return {};

/*
  KDL::JntArray lowerJointLimits(chain.getNrOfJoints());
  KDL::JntArray upperJointLimits(chain.getNrOfJoints());

  
  for(size_t i = 0; i < chain.getNrOfJoints(); i++) {
    lowerJointLimits(i) = -360;
    upperJointLimits(i) = 360;
  }

  double timeout = 5.; // seconds
  double error = 1e-5;
  TRAC_IK::SolveType type=TRAC_IK::Speed;

  TRAC_IK::TRAC_IK ik_solver(chain, lowerJointLimits, upperJointLimits, timeout, error, type);  

  int rc = ik_solver.CartToJnt(qInit, fDestination, q);

  std::cout << rc << std::endl;

  std::vector<double> dofs2;
  for(size_t i = 0; i < chain.getNrOfJoints(); i++) {
    // Convert to pmpl joint values and add to dof
    //dofs.push_back(q(i)/KDL::PI);
    dofs2.push_back(q(i));
  }

  return dofs2;
  */
}

