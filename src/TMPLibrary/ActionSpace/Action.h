#ifndef PPL_ACTION_H_
#define PPL_ACTION_H_

#include "Condition.h"

#include "TMPLibrary/TMPBaseObject.h"

class Action : public TMPBaseObject {
  public:
    ///@name Local Types
    ///@{

    typedef Condition::State State;

    ///@}
    ///@name Construction
    ///@{

    Action();

    Action(XMLNode& _node);

    ~Action();

    ///@}
    ///@name Interface
    ///@{

    virtual bool Valid(const State& _state);

    ///@}
    ///@name Accessors
    ///@{

    const std::vector<std::string>& GetStages() const;

    const std::vector<std::string>& GetStageConditions(const std::string& _stage) const;

    ///@}

  protected:
    ///@name Helper Functions
    ///@{

    virtual void SetClassName();
  
    virtual void ParseXMLNode(XMLNode& _node);

    ///@}
    ///@name Internal State
    ///@{

    std::vector<std::string> m_stages;
    std::unordered_map<std::string,std::vector<std::string>> m_stageConditions;

    ///@}
};

#endif
