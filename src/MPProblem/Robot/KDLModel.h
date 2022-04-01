#ifndef PPL_KDL_MODEL_H_
#define PPL_KDL_MODEL_H_

////////////////////////////////////////////////////////////////////////////////
/// Wrapper for KDL specification of a robot.
///
////////////////////////////////////////////////////////////////////////////////

#include "Utilities/XMLNode.h"

#include <map>
#include <string>
#include <set>

class Robot;

class KDLModel {

  public:

    ///@name Local Types
    ///@{

    struct ToolFrame {
      std::vector<double> pos;
      std::vector<double> ori;
    };

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

    ToolFrame ForwardKinematics(std::vector<double> _jointAngles);

    /// Compute valid dofs for the robot given the 
    /// desired end-effector position and orientation
    /// using inverse kinematics.
    /// @param _pos The position vector of the end-effector frame.
    /// @param _ori The orientation matric of the end-effector frame.
    /// @return The dofs solution.
    std::vector<double> InverseKinematics(std::vector<double> _pos, 
            std::vector<double> _ori,
            std::vector<double> _initialGuess);

    ///@}

  private:

    ///@name Internal State
    ///@{

    Robot* m_robot; ///< Robot associated with this model.

    std::string m_filename; ///< URDF specification of model.

    std::string m_chainRoot; ///< Name of kinematic chain root.

    std::string m_chainTip; ///< Name of kinematic chain tip.

    bool m_initialized{false};

    std::set<size_t> m_segments; ///< Bodies in chain.

    std::vector<size_t> m_jointIndices; ///< Relevant joints

    std::map<std::pair<size_t,size_t>,size_t> m_includedIndices;

    ///@}    

};

#endif
