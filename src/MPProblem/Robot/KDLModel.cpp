#include "KDLModel.h"

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

std::vector<double>
KDLModel::
InverseKinematics(std::vector<double> _pos, std::vector<std::vector<double>> _ori) {

  auto tree = ParseKDLFromFile(m_filename);

  KDL::Chain chain;

  bool ret = tree.getChain(m_chainRoot,m_chainTip,chain);

  if(!ret)
    throw RunTimeException(WHERE) << "Unable to construct chain from tree.";

  // forward position solver
  KDL::ChainFkSolverPos_recursive fksolver1(chain);

  // inverse velocity solver
  KDL::ChainIkSolverVel_pinv iksolver1v(chain, 0.0001, 1000);

  // inverse kinematics solver
  KDL::ChainIkSolverPos_NR iksolverpos(chain,fksolver1,iksolver1v,10000,
                                    std::numeric_limits<double>::epsilon());


  size_t maxAttempts = 10000;

  for(size_t i = 0; i < maxAttempts; i++) {
    // jntarrays
    KDL::JntArray q(chain.getNrOfJoints());
    KDL::JntArray qInit(chain.getNrOfJoints());


    for(size_t i = 0; i < chain.getNrOfJoints(); i++) {
      qInit(i) = (DRand() * 360) - 180;
    }
  

    // Compute current tcp position
    KDL::Frame tcp_pos_start;
    fksolver1.JntToCart(qInit,tcp_pos_start);


    // destination frame
    KDL::Vector vector(_pos[0],_pos[1],_pos.size() == 3 ? _pos[2] : 0);

    std::vector<KDL::Vector> rotationValues;
    for(size_t i = 0; i < 3; i++) {
      KDL::Vector vec(
          _ori[i][0],
          _ori[i][1],
          _ori[i][2]
          );

      rotationValues.push_back(vec);
    }

    KDL::Rotation rotation(rotationValues[0], rotationValues[1], rotationValues[2]);

    KDL::Frame fDestination(rotation,vector);

    auto ret2 = iksolverpos.CartToJnt(qInit,fDestination,q);;

    auto retString = iksolverpos.strError(ret2);

    std::cout << retString << std::endl;

    if(ret2 < 0)
      continue;

    // TODO::Sort these alphabetically bc :(
    std::vector<double> dofs;
    for(size_t i = 0; i < chain.getNrOfJoints(); i++) {
      // Convert to pmpl joint values and add to dof
      //dofs.push_back(q(i)/KDL::PI);
      dofs.push_back(q(i));
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

