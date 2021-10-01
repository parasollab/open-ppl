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
  KDL::ChainIkSolverVel_pinv iksolver1v(chain);

  // inverse kinematics solver
  KDL::ChainIkSolverPos_NR iksolverpos(chain,fksolver1,iksolver1v,10000,
                                    std::numeric_limits<double>::epsilon());

  // jntarrays
  KDL::JntArray q(chain.getNrOfJoints());
  KDL::JntArray qInit(chain.getNrOfJoints());

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

  for(size_t i = 0; i < 
 
  // TODO::Sort these alphabetically bc :(
  std::vector<double> dofs;
  for(size_t i = 0; i < chain.getNrOfJoints(); i++) {
    // Convert to pmpl joint values and add to dof
    //dofs.push_back(q(i)/KDL::PI);
    dofs.push_back(q(i));
  }

 
  return dofs;
}

