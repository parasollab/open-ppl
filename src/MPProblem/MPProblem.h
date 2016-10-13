#ifndef MP_PROBLEM_H_
#define MP_PROBLEM_H_

#include <vector>

#include "MPProblemBase.h"
#include "MPProblem/ConfigurationSpace/Roadmap.h"
#include "MPProblem/Environment/Environment.h"
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

    ////////////////////////////////////////////////////////////////////////////
    /// @brief Helper for parsing XML nodes.
    /// @param[in] _node The child node to be parsed.
    bool ParseChild(XMLNode& _node);

    ///@}
    ///@name Core Properties
    ///@{

    Environment* m_environment;  ///< The environment to plan in.

    RoadmapType* m_roadmap;      ///< The free-space roadmap.
    RoadmapType* m_blockRoadmap; ///< The obstacle-space roadmap.
    StatClass*   m_stats;        ///< Performance tracking object.

    vector<shared_ptr<ActiveMultiBody>> m_robots;  ///< Robots.

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
  delete m_roadmap;
  delete m_blockRoadmap;
  delete m_stats;
  delete m_environment;
  m_robots.clear();
}


template<class MPTraits>
void
MPProblem<MPTraits>::
Initialize() {
  m_environment = nullptr;
  m_roadmap = new RoadmapType();
  m_blockRoadmap = new RoadmapType();
  m_stats = new StatClass();
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
  for(auto& child : mpNode) {
    if(child.Name() == "Input")
      input = &child;
  }

  // Throw exception if we can't it.
  if(!envIsSet && !input)
    throw ParseException(WHERE, "Cannot find Input node in XML file '" +
        _filename + "'.");

  // Parse the input node to set the environment and query.
  if(!envIsSet) {
    Cfg::GetRobots().clear();
    for(auto& child : *input)
      ParseChild(child);
    m_robots = Cfg::GetRobots();
  }

  // Read robots from Cfg class.

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
  if(_node.Name() == "Query") {
    // Ignore this node if we already have a query file.
    if(m_queryFilename.empty())
      m_queryFilename = _node.Read("filename", false, "", "Query file name");
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
  return m_robots[_index].get();
}


template<class MPTraits>
vector<ActiveMultiBody*>
MPProblem<MPTraits>::
GetRobots() const {
  vector<ActiveMultiBody*> robots;
  robots.reserve(m_robots.size());
  for(const auto& ptr : m_robots)
    robots.push_back(ptr.get());
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
