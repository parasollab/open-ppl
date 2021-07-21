#ifndef PPL_HCR_QUERY_H_
#define PPL_HCR_QUERY_H_

#include "TaskEvaluatorMethod.h"

#include "ConfigurationSpace/Cfg.h"
#include "ConfigurationSpace/Weight.h"
#include "ConfigurationSpace/Path.h"

#include "TMPLibrary/StateGraphs/CombinedRoadmap.h"

#include "Utilities/CBS.h"
#include "Utilities/SSSHP.h"

#include <vector>

class HCRQuery : public TaskEvaluatorMethod {

  public:
    ///@name Local Types
    ///@{

    typedef std::pair<bool,size_t> HPElem;
    typedef CombinedRoadmap::TMPVertex TMPVertex;
    typedef CombinedRoadmap::TMPHyperarc TMPHyperarc;

    typedef std::pair<size_t,Cfg> CT;
    typedef PathType<MPTraits<Cfg,DefaultWeight<Cfg>>> Path;
    typedef CBSNode<Robot,CT,Path> Node;

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
    bool Soc();

    Node PerformCBSQuery();

    std::vector<std::pair<Robot*,CT>> Validate(Node _node);

    std::vector<Node> SplitNode(Node _node, 
                        std::vector<std::pair<Robot*,CT>> _constraints,
                        CBSLowLevelPlanner<Robot,CT,Path> _lowlevel);

    std::unordered_map<Robot*,Path*> ExtractPaths(const std::vector<HPElem>& _hyperpath);

    void ExtractPlan(Node _node);

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

    std::string m_vcLabel;

    bool m_soc{false}; ///< Flag indiciating sum-of-cost or makespan
    ///@}

};

#endif
