#ifndef PPL_KDL_MODEL_H_
#define PPL_KDL_MODEL_H_

////////////////////////////////////////////////////////////////////////////////
/// Wrapper for KDL specification of a robot.
///
////////////////////////////////////////////////////////////////////////////////

#include "Utilities/XMLNode.h"

#include <string>

class Robot;

class KDLModel {

  public:
    ///@name LocalTypes
    ///@{

    //typedef KDL::Tree  Tree;
    //typedef KDL::Chain Chain;

    ///@}
    ///@name Construction
    ///@{
    
    KDLModel(Robot* _robot);

    KDLModel(Robot* _robot, const std::string _filename);

    KDLModel(XMLNode& _node, Robot* _robot);

    ~KDLModel();

    ///@}
    ///@name Interface
    ///@{

    /// Compute valid dofs for the robot given the 
    /// desired end-effector position and orientation
    /// using inverse kinematics.
    /// @param _pos
    /// @param _ori
    /// @return The dofs solution
    std::vector<double> InverseKinematics(std::vector<double> _pos, 
            std::vector<std::vector<double>> _ori);

    ///@}

  private:

    ///@name Internal State
    ///@{

    Robot* m_robot;

    /*
    Tree m_tree;

    Chain m_chain;
    */

    std::string m_filename;

    std::string m_chainRoot;
    std::string m_chainTip;

    ///@}    

};

#endif
