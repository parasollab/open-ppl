#ifndef MP_PROBLEM_H_
#define MP_PROBLEM_H_

#include <iostream>
#include <string>
#include <vector>

#include "MPProblemBase.h"

class ActiveMultiBody;
class Environment;
class Robot;
class XMLNode;

////////////////////////////////////////////////////////////////////////////////
/// Representation of a motion planning problem, including an environment,
/// queries, and robots.
/// @TODO Destroy MPProblemBase and put that stuff here.
////////////////////////////////////////////////////////////////////////////////
#ifdef _PARALLEL
class MPProblem : public stapl::p_object, public MPProblemBase
#else
class MPProblem : public MPProblemBase
#endif
{

  public:

    ///@name Construction
    ///@{

    MPProblem();
    MPProblem(const std::string& _filename);
    virtual ~MPProblem();

    /// Read an XML file.
    /// @param[in] _filename    The XML file name.
    void ReadXMLFile(const std::string& _filename);

    ///@}
    ///@name Base Filename Accessors
    ///@{

    const std::string& GetBaseFilename() const {return m_baseFilename;}
    void SetBaseFilename(const std::string& _s) {m_baseFilename = _s;}

    ///@}
    ///@name Environment Accessors
    ///@{

    Environment* GetEnvironment();
    void SetEnvironment(Environment* _e);

    ///@}
    ///@name Robot Accessors
    ///@{

    /// @return Number of Active MultiBodies
    size_t NumRobots() const;

    Robot* GetNewRobot(size_t _index) const;

    /// @param _index Requested multibody
    /// @return Pointer to active multibody
    ActiveMultiBody* GetRobot(size_t _index) const;

    std::vector<ActiveMultiBody*> GetRobots() const;

    ///@}
    ///@name Task Accessors
    ///@{

    const std::string& GetQueryFilename() const {return m_queryFilename;}
    void SetQueryFilename(const std::string& _s) {m_queryFilename = _s;}

    ///@}
    ///@name Debugging
    ///@{

    virtual void Print(std::ostream& _os) const; ///< Print each method set.

    ///@}

  protected:

    ///@name Construction Helpers
    ///@{

    /// Helper for parsing XML nodes.
    /// @param[in] _node The child node to be parsed.
    bool ParseChild(XMLNode& _node);

    ///@}
    ///@name Core Properties
    ///@{

    Environment* m_environment{nullptr};  ///< The planning environment.
    std::vector<Robot*> m_robots;              ///< The robots in our problem.

    ///@}
    ///@name Files
    ///@{

    std::string m_baseFilename;  ///< The base name for output files.
    std::string m_queryFilename; ///< The query file name.

    ///@}

};

#endif
