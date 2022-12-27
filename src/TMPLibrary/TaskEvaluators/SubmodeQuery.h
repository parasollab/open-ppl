#ifndef PPL_SUBMODE_QUERY_H_
#define PPL_SUBMODE_QUERY_H_

#include "TaskEvaluatorMethod.h"

#include "TMPLibrary/StateGraphs/GroundedHypergraph.h"
#include "TMPLibrary/StateGraphs/ModeGraph.h"

#include "Utilities/Hypergraph.h"
#include "Utilities/SSSHP.h"

class SubmodeQuery : public TaskEvaluatorMethod { 

  public:

    ///@name Local Types
    ///@{

    typedef GroundedHypergraph::Vertex                 GroundedVertex;
    typedef ModeGraph::ModeHypergraph                  ModeHypergraph;

    typedef std::set<size_t>                           ActionHistory;
    typedef std::pair<std::set<size_t>,ActionHistory>  PartiallyGroundedHyperarc;
    typedef std::pair<bool,size_t> HPElem;
    
    struct ActionExtendedVertex {
      size_t        groundedVID;
      ActionHistory history;

      bool operator==(const ActionExtendedVertex& _other) const {
        return groundedVID == _other.groundedVID
           and history     == _other.history;
      }
    };

    /// Hyperarcs are hids in grounded hypergraph
    typedef Hypergraph<ActionExtendedVertex,size_t> ActionExtendedHypergraph;

    ///@}
    ///@name Construction
    ///@{

    SubmodeQuery();

    SubmodeQuery(XMLNode& _node);

    virtual ~SubmodeQuery();

    ///@}
    ///@name Task Evaluator Interface
    ///@{

    virtual void Initialize();

    ///@}

  protected:

    ///@name Helper Functions
    ///@{

    virtual bool Run(Plan* _plan = nullptr);

    ActionHistory CombineHistories(size_t _vid, const std::set<size_t>& _pgh,
                                   const ActionHistory& _history);

    void ConvertToPlan(Plan* _plan);

    std::vector<HPElem> ConstructPath(size_t _sink, 
                std::set<HPElem>& _parents, const MBTOutput& _mbt);

    std::vector<HPElem> AddBranches(std::vector<HPElem> _path, 
                std::set<HPElem>& _parents, const MBTOutput& _mbt);

    std::vector<HPElem> AddDanglingNodes(std::vector<HPElem> _path,
                std::set<HPElem>& _parents);

    std::vector<HPElem> OrderPath(std::vector<HPElem> _path);

    void ComputeHeuristicValues();

    ///@}
    ///@name Hyperpath Functions
    ///@{

    void HyperpathQuery();

    SSSHPTermination HyperpathTermination(const size_t& _vid, const MBTOutput& _mbt);

    double HyperpathPathWeightFunction(
          const typename ActionExtendedHypergraph::Hyperarc& _hyperarc,
          const std::unordered_map<size_t,double> _weightMap,
          const size_t _target);

    std::set<size_t> HyperpathForwardStar(const size_t& _vid, ActionExtendedHypergraph* _h);

    double HyperpathHeuristic(const size_t& _target);

    ///@name Internal State
    ///@{

    ActionExtendedHypergraph m_actionExtendedHypergraph;

    /// Set of action extended vertices that contained previous solutions
    std::set<size_t> m_previousSolutions;

    /// Map from grounded hypergraph vertices to action extended vertices
    std::unordered_map<size_t,std::set<size_t>> m_vertexMap;

    /// Map from grounded hypergraph hyperarcs to action extended partial groundings
    std::unordered_map<size_t,std::vector<PartiallyGroundedHyperarc>> m_partiallyGroundedHyperarcs;

    size_t m_goalVID;

    /// Map from grouneded hypergraph vertex to heuristic value
    std::unordered_map<size_t,double> m_costToGoMap; 

    std::unordered_map<size_t,double> m_searchTimeHeuristicMap;

    double m_maxDistance;

    bool m_reverseActions{false};

    bool m_writeHypergraph{false};

    // Saving frontier of hypergraph search
    MBTOutput m_mbt;
    
    std::priority_queue<SSSHPElement,std::vector<SSSHPElement>,std::greater<SSSHPElement>> m_pq;

    size_t counter{0};

    std::string m_mgLabel;
    std::string m_ghLabel;

    ///@}
};

std::ostream& operator<<(std::ostream& _os, const SubmodeQuery::ActionExtendedVertex& _vertex);
std::istream& operator>>(std::istream& _is, const SubmodeQuery::ActionExtendedVertex& _vertex);
#endif
