#ifndef SHORT_CUTTING_PATH_MODIFIER_H_
#define SHORT_CUTTING_PATH_MODIFIER_H_

#include "PathModifierMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup PathModifiers
///
/// Perform "shortcutting" on a provided path. 
////////////////////////////////////////////////////////////////////////////////
class ShortcuttingPathModifier : public PathModifierMethod {

  public:

    typedef typename MPBaseObject::RoadmapType  RoadmapType;
    typedef typename RoadmapType::VID           VID;

    ShortcuttingPathModifier(const string& _dmLabel = "",
        const string& _lpLabel = "");
    ShortcuttingPathModifier(XMLNode& _node);

    virtual void ParseXML(XMLNode& _node);
    virtual void Print(ostream& _os) const;

    bool ModifyImpl(RoadmapType* _graph, std::vector<Cfg>& _path, std::vector<Cfg>& _newPath) override;

  private:
    string m_lpLabel; // Local planner
};

#endif

