#ifndef PPL_INTERACTION_H_
#define PPL_INTERACTION_H_

#include "Action.h"

#include "ConfigurationSpace/GroupLocalPlan.h"
#include "ConfigurationSpace/GroupRoadmap.h"
#include "MPLibrary/MPSolution.h"
#include "Traits/CfgTraits.h"

class Interaction : public Action {
  public:
    ///@name Local Types
    ///@{

    typedef Condition::State State;
		typedef MPSolutionType<MPTraits<Cfg,DefaultWeight<Cfg>>> MPSolution;
    typedef PathType<MPTraits<Cfg,DefaultWeight<Cfg>>>       Path;
    typedef GroupPath<MPTraits<Cfg,DefaultWeight<Cfg>>> GroupPathType;


    ///@}
    ///@name Construction
    ///@{

    Interaction();

    Interaction(XMLNode& _node);

    ~Interaction();

    ///@}
    ///@name Interface
    ///@{

    virtual void Initialize() override;

    virtual bool Valid(const State& _state) override;

    ///@}
    ///@name Accessors
    ///@{

    const std::string GetInteractionStrategyLabel() const;

    MPSolution* GetToStageSolution(const std::string& _stage) const;

    std::unique_ptr<MPSolution>&& ExtractToStageSolution(const std::string& _stage);

    GroupPathType* GetToStageGroupPath(const std::string& _stage) const;

    void SetToStageGroupPath(const std::string& _stage, GroupPathType* _path);

    std::vector<Path*> GetToStagePaths(const std::string& _stage) const;

    void SetToStagePaths(const std::string& _stage, std::vector<Path*> _paths);

    std::vector<std::shared_ptr<GroupTask>> GetToStageTasks(const std::string& _stage) const;

    void SetToStageTasks(const std::string& _stage, std::vector<std::shared_ptr<GroupTask>> _tasks);

    ///@}

  protected:
    ///@name Helper Functions
    ///@{

    virtual void SetClassName() override;

    virtual void ParseXMLNode(XMLNode& _node) override;

    ///@}
    ///@name Internal State
    ///@{

    std::string m_isLabel; ///< Interaction Strategy Label

    std::unordered_map<std::string,std::unique_ptr<MPSolution>> m_toStageSolutions;

    std::unordered_map<std::string,GroupPathType*> m_toStageGroupPaths;

    std::unordered_map<std::string,std::vector<Path*>> m_toStagePaths;

    std::unordered_map<std::string,std::vector<std::shared_ptr<GroupTask>>> m_toStageTasks;

    ///@}
};

#endif
