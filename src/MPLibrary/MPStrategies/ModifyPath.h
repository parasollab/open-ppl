#ifndef MODIFY_PATH_H_
#define MODIFY_PATH_H_

#include "MPStrategyMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// This is a strategy that allows us to use PathModifiers. 
/// @ingroup MotionPlanningStrategies
////////////////////////////////////////////////////////////////////////////////
class ModifyPath : public MPStrategyMethod {

  public:

    ModifyPath(const std::string& _pathFile = "", const std::string& _mapFile = "",
        const std::string& _pmLabel = "");
    ModifyPath(XMLNode& _node);

    virtual void Initialize();
    virtual void Iterate();
    virtual void Finalize();
    virtual void Print(std::ostream& _os) const;

  protected:
    std::string m_pathFile;
    std::string m_mapFile;
    std::string m_pmLabel;

    std::vector<Cfg> m_path, m_smoothPath;
};

#endif