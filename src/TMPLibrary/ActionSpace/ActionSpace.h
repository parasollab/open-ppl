#ifndef PPL_ACTION_SPACE
#define PPL_ACTION_SPACE

#include <iostream>
#include <unordered_map>

class Action;
class Condition;
class TMPLibrary;
class XMLNode;

class ActionSpace {
  public:
    ///@name Local Types
    ///@{

    //TODO::Maybe state should be defined here until it becomes its own class.

    template <typename Utility>
    using LabelMap = std::unordered_map<std::string, Utility*>;
    
    ///@}
    ///@name Construction
    ///@{ 

    ActionSpace() = default;

    ActionSpace(TMPLibrary* _tmpLibrary);

    virtual ~ActionSpace();

    void ParseXML(XMLNode& _node);

    ///@}
    ///@name Interface
    ///@{

    void Initialize();

    ///@}
    ///@name Conditions
    ///@{

    Condition* GetCondition(const std::string& _label);

    void SetCondition(const std::string& _label, Condition* _utility);

    ///@}
    ///@name Actions
    ///@{

    const LabelMap<Action>& GetActions();

    Action* GetAction(const std::string& _label);

    void SetAction(const std::string& _label, Action* _utility);

    ///@}

  private:
    ///@name Helper Functions
    ///@{

    template <typename Utility>
    Utility* GetUtility(const std::string& label,
                        const LabelMap<Utility>& _map);

    template <typename Utility>
    void SetUtility(const std::string& _label, Utility* _utility,
                    LabelMap<Utility>& _map);
    ///@}
    ///@name Internal State
    ///@{

    TMPLibrary* const m_tmpLibrary; ///< The owning library.

    LabelMap<Condition> m_conditions;
    LabelMap<Action> m_actions;

    ///@}

};

#endif
