#ifndef PMPL_GROUP_QUERY_H_
#define PMPL_GROUP_QUERY_H_

#include "QueryMethod.h"

#include "Utilities/SSSP.h"

////////////////////////////////////////////////////////////////////////////////
/// Evaluates the current group roadmap to determine whether a (group) path
/// exists which satisfies the current group task.
///
/// @todo There is a lot of copy-pasted code from QueryMethod because that
///       method calls several other objects which are not ready for groups yet.
///       We should be able to merge these two after a bit more flush-out of the
///       group support. I have marked it final to remind us that we should
///       not extend this as it will be merged.
///
/// @ingroup MapEvaluators
////////////////////////////////////////////////////////////////////////////////
class GroupQuery final : public MapEvaluatorMethod {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPBaseObject::GroupCfgType       GroupCfgType;
    typedef typename MPBaseObject::GroupRoadmapType   GroupRoadmapType;
    typedef typename MPBaseObject::GroupWeightType    WeightType;
    typedef typename GroupRoadmapType::VID            VID;
    typedef std::unordered_set<VID>                   VIDSet;

    ///@}
    ///@name Construction
    ///@{

    GroupQuery();
    GroupQuery(XMLNode& _node);
    virtual ~GroupQuery() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Initialize() override;

    ///@}
    ///@name MapEvaluator Overrides
    ///@{

    virtual bool operator()() override;

    ///@}
    ///@name Query Interface
    ///@{

    std::vector<VID> GeneratePath(const VID _start, const VIDSet& _end);

    virtual void SetPathWeightFunction(SSSPPathWeightFunction<GroupRoadmapType> _f);

    ///@}

  protected:

    ///@name Helpers
    ///@{

    void Reset(GroupRoadmapType* const _r);

    bool PerformSubQuery(const VID _start, const VIDSet& _goal);

    double StaticPathWeight(typename GroupRoadmapType::adj_edge_iterator& _ei,
        const double _sourceDistance, const double _targetDistance) const;

    double DynamicPathWeight(typename GroupRoadmapType::adj_edge_iterator& _ei,
        const double _sourceDistance, const double _targetDistance) const;

    //bool SameFormations(std::unordered_set<Formation*> _set1, std::unordered_set<Formation*> _set2);

    ///@}
    ///@name Internal State
    ///@{

    GroupRoadmapType* m_roadmap{nullptr};

    size_t m_goalIndex{0};             ///< Index of next unreached goal.

    SSSPPathWeightFunction<GroupRoadmapType> m_weightFunction;

    std::string m_vcLabel;

    ///@}

};

#endif
