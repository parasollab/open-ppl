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
    /// @param _pos The position vector of the end-effector frame.
    /// @param _ori The orientation matric of the end-effector frame.
    /// @return The dofs solution.
    std::vector<double> InverseKinematics(std::vector<double> _pos, 
            std::vector<std::vector<double>> _ori);

    ///@}

  private:

    ///@name Internal State
    ///@{

    Robot* m_robot; ///< Robot associated with this model.

    std::string m_filename; ///< URDF specification of model.

    std::string m_chainRoot; ///< Name of kinematic chain root.

    std::string m_chainTip; ///< Name of kinematic chain tip.

    ///@}    

};

#endif
