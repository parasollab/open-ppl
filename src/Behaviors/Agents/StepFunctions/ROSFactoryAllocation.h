#ifndef PPL_ROS_FDCTORY_ALLOCATION_H_
#define PPL_ROS_FDCTORY_ALLOCATION_H_

#include "StepFunction.h"

#include "MPProblem/TaskHierarchy/Decomposition.h"
#include "MPProblem/TaskHierarchy/SemanticTask.h"

#include "TMPLibrary/Solution/Plan.h"
#include "TMPLibrary/Solution/TaskSolution.h"

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

class ROSFactoryAllocation : public StepFunction {

  public:
  
    ///@name Local Types
    ///@{

    struct TaskPoint {
      std::string label;
      std::vector<double> location;
      std::string topic;
    };

    ///@}
    ///@name Construction
    ///@{

    ROSFactoryAllocation(Agent* _agent, XMLNode& _node);

    ~ROSFactoryAllocation();

    ///@}
    ///@name Step Function Interface
    ///@{

    virtual void StepAgent(double _dt) override;

    ///@}

  private:

    ///@name Helper Functions
    ///@{

    void ParseTaskPoint(XMLNode& _node, ros::NodeHandle& _nh);

    void UpdateTimeRemaining(std::string _label);

    void AssignTask(std::string _robot, std::string _label, std::vector<double> _location);
    
    std::unique_ptr<Decomposition> BuildDecomposition();
  
    void PlanAllocations(Decomposition* _decomp);

    ///@}
    ///@name ROS Interface
    ///@{

    ///@}
    ///@name Internal State
    ///@{

    std::map<std::string,TaskPoint> m_taskPoints;

    std::map<std::string,double> m_timeRemaining;

    std::map<std::string,ros::Subscriber> m_timeRemainingSubs;

    std::map<std::string,ros::Publisher> m_taskQueuePubs;

    std::vector<std::unique_ptr<Decomposition>> m_decompositions;

    std::unique_ptr<Plan> m_plan;

    double m_timeThreshold;

    std::set<std::string> m_allocated;

    ///@}

};

#endif
