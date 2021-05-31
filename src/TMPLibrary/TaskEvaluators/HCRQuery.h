#ifndef PPL_HCR_QUERY_H_
#define PPL_HCR_QUERY_H_

#include "TaskEvaluatorMethod.h"

#include "TMPLibrary/StateGraphs/CombinedRoadmap.h"

#include "Utilities/SSSHP.h"

class HCRQuery : public TaskEvaluatorMethod {

  public:
    ///@name Local Types
    ///@{

    typedef std::pair<bool,size_t> HPElem;
    typedef CombinedRoadmap::TMPVertex TMPVertex;
    typedef CombinedRoadmap::TMPHyperarc TMPHyperarc;

    ///@}
    ///@name Construction
    ///@{

    HCRQuery();

    HCRQuery(XMLNode& _node);

   ~ HCRQuery();

    ///@}

  protected: 
    ///@name Overrides
    ///@{

    virtual bool Run(Plan* _plan = nullptr) override;

    ///@}

  private:
    ///@name Helper Functions
    ///@{

    void ExtractPlan(std::vector<HPElem>& _path);

    std::vector<HPElem> PerformHyperpathQuery();

    std::vector<HPElem> ConstructPath(size_t _sink, 
                std::set<HPElem>& _parents, MBTOutput& _mbt);

    std::vector<HPElem> AddBranches(std::vector<HPElem> _path, 
                std::set<HPElem>& _parents, MBTOutput& _mbt);

    std::vector<HPElem> AddDanglingNodes(std::vector<HPElem> _path,
                std::set<HPElem>& _parents);

    ///@}
    ///@name Internal State
    ///@{

    std::string m_sgLabel;

    ///@}

};

#endif
