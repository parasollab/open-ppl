#ifndef MP_PROBLEM_H_
#define MP_PROBLEM_H_

#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>

class ActiveMultiBody;
class Environment;
class MPTask;
class Robot;
class XMLNode;


////////////////////////////////////////////////////////////////////////////////
/// Representation of a motion planning problem, including an environment,
/// tasks, and robots.
////////////////////////////////////////////////////////////////////////////////
#ifdef _PARALLEL
class MPProblem final : public stapl::p_object
#else
class MPProblem final
#endif
{

  public:

    ///@name Construction
    ///@{

    /// Instantiate an empty MPProblem.
    MPProblem();

    /// Instantiate an MPProblem from an XML file.
    /// @param[in] _filename The name of the XML file.
    MPProblem(const std::string& _filename);

    ~MPProblem();

    ///@}
    ///@name XML File Parsing
    ///@{

    /// Get the XML filename from which this object was parsed.
    const std::string& GetXMLFilename() const;

    /// Read an XML file.
    /// @param[in] _filename The XML file name.
    void ReadXMLFile(const std::string& _filename);

    ///@}
    ///@name Environment Accessors
    ///@{

    /// Get the environment object.
    Environment* GetEnvironment();

    /// Set the environment object.
    void SetEnvironment(Environment* _e);

    ///@}
    ///@name Robot Accessors
    ///@{

    /// Get the number of robots in our problem.
    size_t NumRobots() const noexcept;

    /// Get a specific robot by index.
    Robot* GetRobot(size_t _index) const;

    /// Get a specific robot by label.
    Robot* GetRobot(const std::string& _label) const;

    /// Get all robots in this problem.
    const std::vector<Robot*>& GetRobots() const noexcept;

    ///@}
    ///@name Task Accessors
    ///@{
    /// @TODO Add support for dynamically adding and removing tasks.

    /// Ye olde query. To be removed.
    const std::string& GetQueryFilename() const {return m_queryFilename;}
    void SetQueryFilename(const std::string& _s) {m_queryFilename = _s;}

    /// Get all of the tasks in this problem.
    const std::vector<MPTask*>& GetTasks() const noexcept;
    const std::vector<MPTask*>& GetTasks(Robot* const) const noexcept;

    /// Add a task to the problem for a given robot.
    /// @param _robot The robot to which the new task is assigned.
    /// @param _task The new task.
    void AddTask(Robot* const _robot, MPTask* const _task);

    ///@}
    ///@name Debugging
    ///@{

    virtual void Print(std::ostream& _os) const; ///< Print each method set.

    ///@}
    ///@name File Path Accessors
    ///@{

    const std::string& GetBaseFilename() const;
    void SetBaseFilename(const std::string& _s);

    static std::string GetPath(const std::string& _filename);
    static void SetPath(const std::string& _filename);

    ///@}

  protected:

    ///@name Construction Helpers
    ///@{

    /// Helper for parsing XML nodes.
    /// @param[in] _node The child node to be parsed.
    bool ParseChild(XMLNode& _node);

    /// Create a pseudo-point robot.
    void MakePointRobot();

    ///@}
    ///@name Core Properties
    ///@{

    Environment* m_environment{nullptr};  ///< The planning environment.
    std::vector<Robot*> m_robots;         ///< The robots in our problem.
    Robot* m_pointRobot{nullptr};         ///< A pseudo point-robot.
    std::vector<MPTask*> m_tasks;         ///< The tasks in our problem.

    /// @TODO Replace m_tasks with this when it is at 100%.
    std::unordered_map<Robot*, std::vector<MPTask*>> m_taskMap;   /// Map to keep track of the tasks assigned to robots

    ///@}
    ///@name Files
    ///@{

    std::string m_xmlFilename;     ///< The XML file name.
    std::string m_baseFilename;    ///< The base name for output files.
    std::string m_queryFilename;   ///< The query file name.

    static std::string m_filePath; ///< The relative path for the problem XML.

    ///@}

};

#endif
