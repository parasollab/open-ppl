#ifndef MP_PROBLEM_H_
#define MP_PROBLEM_H_

#include <vector>

#include "MPProblemBase.h"
#include "MPProblem/ConfigurationSpace/Roadmap.h"
#include "MPProblem/Environment/Environment.h"
#include "MPProblem/Robot/Robot.h"
#include "Utilities/MetricUtils.h"
#include "Utilities/MPUtils.h"
#include "Utilities/PMPLExceptions.h"

////////////////////////////////////////////////////////////////////////////////
/// @brief Representation of a motion planning problem. It owns the environment,
///        queries, and robots.
////////////////////////////////////////////////////////////////////////////////
template<class MPTraits>
#ifdef _PARALLEL
class MPProblem : public stapl::p_object, public MPProblemBase
#else
class MPProblem : public MPProblemBase
#endif
{

  public:

    ///@name General Abstraction Types
    ///@{

    typedef typename MPTraits::MPProblemType      MPProblemType;
    typedef Roadmap<MPTraits>                     RoadmapType;
    typedef typename RoadmapType::GraphType       GraphType;
    typedef typename GraphType::vertex_descriptor VID;

    ///@}
    ///@name Construction
    ///@{

    MPProblem();
    MPProblem(const string& _filename);
    virtual ~MPProblem();

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Read an XML file.
    /// @param[in] _filename    The XML file name.
    void ReadXMLFile(const string& _filename);

    ///@}
    ///@name Base Filename Accessors
    ///@{

    const string& GetBaseFilename() const {return m_baseFilename;}
    void SetBaseFilename(const string& _s) {m_baseFilename = _s;}

    ///@}
    ///@name Environment Accessors
    ///@{

    Environment* GetEnvironment() {return m_environment;}
    void SetEnvironment(Environment* _e) {m_environment = _e;}

    ///@}
    ///@name Robot Accessors
    ///@{

    ////////////////////////////////////////////////////////////////////////////
    /// @return Number of Active MultiBodies
    size_t NumRobots() const {return m_robots.size();}

    ////////////////////////////////////////////////////////////////////////////
    /// @param _index Requested multibody
    /// @return Pointer to active multibody
    ActiveMultiBody* GetRobot(size_t _index) const;

    vector<ActiveMultiBody*> GetRobots() const;

    ///@}
    ///@name Query Accessors
    ///@{

    string GetQueryFilename() const {return m_queryFilename;}
    void SetQueryFilename(const string& _s) {m_queryFilename = _s;}

    ///@}
    ///@name Roadmap Accessors
    ///@{

    RoadmapType* GetRoadmap() {return m_roadmap;}
    void SetRoadmap(RoadmapType* _roadmap) {m_roadmap = _roadmap;}
    RoadmapType* GetBlockRoadmap() {return m_blockRoadmap;}

    ///@}
    ///@name Stat Class Accessor
    ///@{

    StatClass* GetStatClass() {return m_stats;}

    ///@}
    ///@name Debugging
    ///@{

    virtual void Print(ostream& _os) const; ///< Print each method set.

    ///@}

  protected:

    ///@name Construction Helpers
    ///@{

    virtual void Initialize(); ///< Initialize all local method sets and data.
    virtual void Clear();      ///< Destroys all objects and releases memory.

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Helper for parsing XML nodes.
    /// @param[in] _node The child node to be parsed.
    bool ParseChild(XMLNode& _node);

    ///@}
    ///@name Core Properties
    ///@{

    Environment* m_environment{nullptr};  ///< The planning environment.
    vector<Robot*> m_robots;              ///< The robots in our problem.

    RoadmapType* m_roadmap{nullptr};      ///< The free-space roadmap.
    RoadmapType* m_blockRoadmap{nullptr}; ///< The obstacle-space roadmap.
    StatClass*   m_stats{nullptr};        ///< Performance tracking object.

    ///@}
    ///@name Files
    ///@{

    string m_baseFilename;  ///< The base name for output files.
    string m_queryFilename; ///< The query file name.

    ///@}

};

/*---------------------------- Construction ----------------------------------*/

template<class MPTraits>
MPProblem<MPTraits>::
MPProblem() {
  Initialize();
};


template<class MPTraits>
MPProblem<MPTraits>::
MPProblem(const string& _filename) {
  Initialize();
  ReadXMLFile(_filename);
}


template<class MPTraits>
MPProblem<MPTraits>::
~MPProblem() {
  Clear();
}


template<class MPTraits>
void
MPProblem<MPTraits>::
Initialize() {
  Clear();
  m_roadmap = new RoadmapType();
  m_blockRoadmap = new RoadmapType();
  m_stats = new StatClass();
}


template<class MPTraits>
void
MPProblem<MPTraits>::
Clear() {
  delete m_roadmap;
  delete m_blockRoadmap;
  delete m_stats;
  delete m_environment;
  m_roadmap = nullptr;
  m_blockRoadmap = nullptr;
  m_stats = nullptr;
  m_environment = nullptr;

  for(auto& robot : m_robots)
    delete robot;
  m_robots.clear();
}

/*---------------------------- XML Helpers -----------------------------------*/

template<class MPTraits>
void
MPProblem<MPTraits>::
ReadXMLFile(const string& _filename) {
  bool envIsSet = m_environment;

  size_t sl = _filename.rfind("/");
  m_filePath = _filename.substr(0, sl == string::npos ? 0 : sl + 1);

  // Open the XML and get the root node.
  XMLNode mpNode(_filename, "MotionPlanning");

  // Find the 'Input' node.
  XMLNode* input = nullptr;
  for(auto& child : mpNode)
    if(child.Name() == "Problem")
      input = &child;

  // Throw exception if we can't find it.
  if(!envIsSet && !input)
    throw ParseException(WHERE, "Cannot find Input node in XML file '" +
        _filename + "'.");

  // Parse the input node to set the environment, robot(s), and query.
  if(!envIsSet)
    for(auto& child : *input)
      ParseChild(child);

  // Print XML details if requested.
  bool print = mpNode.Read("print", false, false, "Print all XML input");
  if(print)
    Print(cout);

  // Handle XML warnings/errors.
  bool warnings = mpNode.Read("warnings", false, false, "Report warnings");
  if(warnings) {
    bool warningsAsErrors = mpNode.Read("warningsAsErrors", false, false,
        "XML warnings considered errors");
    if(!envIsSet)
      input->WarnAll(warningsAsErrors);
  }

  // Make sure there is an environment and a robot.
  if(!m_environment)
    throw ParseException(input->Where(), "No environment specified in the "
        "problem node.");
  if(m_robots.empty())
    throw ParseException(input->Where(), "No robots specified in the problem "
        "node.");

  // Initialize the Cfg robot pointers.
  Cfg::m_robots.clear();
  Cfg::SetSize(m_robots.size());
  for(size_t i = 0; i < m_robots.size(); ++i)
    Cfg::InitRobots(m_robots[i]->GetMultiBody(), i);

  // Set the boundary for each robot.
  for(auto& robot : m_robots)
    robot->SetBoundary(GetEnvironment()->GetBoundary().get());

  // Compute the environment resolution.
  GetEnvironment()->ComputeResolution(GetRobots());
}


template<class MPTraits>
bool
MPProblem<MPTraits>::
ParseChild(XMLNode& _node) {
  if(_node.Name() == "Environment") {
    // Ignore this node if we already have an environment.
    if(!m_environment)
      m_environment = new Environment(_node);
    return true;
  }
  else if(_node.Name() == "Robot") {
    m_robots.push_back(new Robot(_node));
    return true;
  }
  else if(_node.Name() == "Query") {
    // Ignore this node if we already have a query file.
    if(m_queryFilename.empty())
      m_queryFilename = _node.Read("filename", true, "", "Query file name");
    return true;
  }
  else
    return false;
}

/*------------------------------ Robot Functions -----------------------------*/

template<class MPTraits>
ActiveMultiBody*
MPProblem<MPTraits>::
GetRobot(size_t _index) const {
  if(_index < 0 || _index >= m_robots.size())
    throw RunTimeException(WHERE,
        "Cannot access ActiveBody '" + ::to_string(_index) + "'.");
  return m_robots[_index]->GetMultiBody();
}


template<class MPTraits>
vector<ActiveMultiBody*>
MPProblem<MPTraits>::
GetRobots() const {
  vector<ActiveMultiBody*> robots;
  robots.reserve(m_robots.size());
  for(const auto& ptr : m_robots)
    robots.push_back(ptr->GetMultiBody());
  return robots;
}

/*-------------------------------- Debugging ---------------------------------*/

template<class MPTraits>
void
MPProblem<MPTraits>::
Print(ostream& _os) const {
  _os << "MPProblem" << endl;
  m_environment->Print(_os);
}

/*----------------------------------------------------------------------------*/

#endif
