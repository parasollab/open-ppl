#ifndef PPL_SMART_H_
#define PPL_SMART_H_

#include "TMPLibrary/TaskEvaluators/TaskEvaluatorMethod.h"

#include "ConfigurationSpace/Cfg.h"
#include "ConfigurationSpace/GenericStateGraph.h"
#include "ConfigurationSpace/GroupCfg.h"
#include "ConfigurationSpace/GroupLocalPlan.h"
#include "ConfigurationSpace/GroupRoadmap.h"

class SMART : public TaskEvaluatorMethod {

  public: 
    ///@name Local Types
    ///@{

    typedef GroupLocalPlan<Cfg>                                  GroupLocalPlanType;
    typedef GroupRoadmap<GroupCfg,GroupLocalPlanType>            GroupRoadmapType;
    
    typedef size_t                                                       VID;
    typedef std::vector<std::pair<GroupRoadmapType*,VID>>                Vertex;
    typedef std::vector<std::pair<GroupRoadmapType*,std::pair<VID,VID>>> Edge;

    typedef GenericStateGraph<Vertex,Edge> TensorProductRoadmap;

    ///@}
    ///@name Construction
    ///@{

    SMART();

    SMART(XMLNode& _node);

    virtual ~SMART() = default;

    ///@}
    ///@name Task Evaluator Interface
    ///@{

    virtual void Initialize() override;

    ///@}
  protected:

    ///@name Helper Functions
    ///@{

    virtual bool Run(Plan* _plan = nullptr) override;

    

    ///@}
    ///@name Internal State
    ///@{

    ///@}

};

#endif
