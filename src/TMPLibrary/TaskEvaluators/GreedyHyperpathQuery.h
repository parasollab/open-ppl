#ifndef PPL_GREEDY_HYPERPATH_QUERY_H_
#define PPL_GREEDY_HYPERPATH_QUERY_H_

#include "SubmodeQuery.h"

#include "ConfigurationSpace/GenericStateGraph.h"

#include "TMPLibrary/Solution/Plan.h"

class GreedyHyperpathQuery : public SubmodeQuery {

  public:

    ///@name Local Types
    ///@{

    typedef SubmodeQuery::ActionHistory          ActionHistory;
    typedef size_t                               VID;
    typedef size_t                               HID;
    typedef GenericStateGraph<ActionHistory,HID> HistoryGraph;

    ///@}
    ///@name Construction
    ///@{

    GreedyHyperpathQuery();

    GreedyHyperpathQuery(XMLNode& _node);

    ~GreedyHyperpathQuery();

    ///@}
    ///@name Task Evaluator Interface
    ///@{

    virtual void Initialize();

    ///@}

  protected:

    ///@name Task Evaluator Functions
    ///@{

    virtual bool Run(Plan* _plan = nullptr) override;

    ///@}
    ///@name Helper Functions
    ///@{

    VID DFS(const VID _source);

    VID Termination(const VID _vid);

    std::vector<GreedyHyperpathQuery::VID> Frontier(const VID _vid);

    bool IsValidHistory(const ActionHistory& _history);

    ///@}
    ///@name Internal State
    ///@{

    std::unique_ptr<HistoryGraph> m_historyGraph;

    std::unordered_map<VID,double> m_heuristicValues;

    ///@}

};

#endif
