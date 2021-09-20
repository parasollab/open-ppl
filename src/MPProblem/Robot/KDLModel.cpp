#include "KDLModel.h"

#include "MPProblem/Robot/Robot.h"

#include "Utilities/KDLParser.h"
#include "Utilities/PMPLExceptions.h"

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>

//#undef PI

//#include "Orientation.h"
//#include "EulerAngle.h"


/*----------------------- Construction -----------------------*/

KDLModel::
KDLModel(Robot* _robot) : m_robot(_robot) {}

KDLModel::
KDLModel(Robot* _robot, const std::string _filename) : m_robot(_robot) {
  m_tree = ParseKDLFromFile(_filename);
}

KDLModel::
KDLModel(XMLNode& _node, Robot* _robot) : m_robot(_robot) {
  // TODO::Allow explicit specification of kdl chain/tree
  std::string filename = _node.Read("filename",true,"","URDF file name.");

  std::string chainRoot = _node.Read("chainRoot",true,"",
                                     "Root of chain to compute FK/IK");

  std::string chainTip = _node.Read("chainTip",true,"",
                                    "Tip of chain to compute FK/IK");

  m_tree = Tree(ParseKDLFromFile(filename));

  bool ret = m_tree.getChain("chainRoot","chainTip",m_chain);

  if(!ret)
    throw RunTimeException(WHERE) << "Unable to construct chain from tree.";
}

KDLModel::
~KDLModel() { }

/*------------------------- Interface ------------------------*/

std::vector<double>
KDLModel::
InverseKinematics(std::vector<double> _pos, std::vector<std::vector<double>> _ori) {

  // forward position solver
  KDL::ChainFkSolverPos_recursive fksolver1(m_chain);

  // inverse velocity solver
  KDL::ChainIkSolverVel_pinv iksolver1v(m_chain);

  // inverse kinematics solver
  KDL::ChainIkSolverPos_NR iksolverpos(m_chain,fksolver1,iksolver1v,100,
                                    std::numeric_limits<double>::epsilon());

  // jntarrays
  KDL::JntArray q(m_tree.getNrOfJoints());
  KDL::JntArray qInit(m_tree.getNrOfJoints());

  // destination frame
  KDL::Vector vector(_pos[0],_pos[2],_pos.size() == 3 ? _pos[2] : 0);

  // gamma is about the x axis
  //mathtool::EulerAngle euler(_ori[1],_ori[2],_ori[0]);
  //mathtool::Orientation matrix(euler);

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

  auto ret = iksolverpos.CartToJnt(qInit,fDestination,q);;

  std::cout << ret << std::endl;
  
  return {};
}

