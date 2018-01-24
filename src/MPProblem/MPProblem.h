#ifndef MP_PROBLEM_H_
#define MP_PROBLEM_H_

#include <iostream>
#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

class DynamicObstacle;
class Environment;
class MPTask;
class MultiBody;
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
    /// @param _filename The name of the XML file.
    explicit MPProblem(const std::string& _filename);

    MPProblem(const MPProblem& _other); ///< Copy.
    MPProblem(MPProblem&& _other);      ///< Move.

    ~MPProblem();

    ///@}
    ///@name Assignment
    ///@{

    MPProblem& operator=(const MPProblem& _other); ///< Copy.
    MPProblem& operator=(MPProblem&& _other);      ///< Move.

    ///@}
    ///@name XML File Parsing
    ///@{

    /// Get the XML filename from which this object was parsed.
    const std::string& GetXMLFilename() const;

    /// Read an XML file.
    /// @param _filename The XML file name.
    void ReadXMLFile(const std::string& _filename);

    ///@}
    ///@name Environment Accessors
    ///@{

    /// Get the environment object.
    Environment* GetEnvironment();

    /// Set the environment object.
    void SetEnvironment(std::unique_ptr<Environment>&& _e);

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
    const std::vector<std::unique_ptr<Robot>>& GetRobots() const noexcept;

    ///@}
    ///@name Task Accessors
    ///@{
    /// @TODO Add support for dynamically adding and removing tasks.

    /// Get the tasks currently assigned to a given robot.
    /// @param _robot The robot to retrieve tasks for.
    /// @return The set of tasks currently assigned to _robot.
    const std::list<std::unique_ptr<MPTask>>& GetTasks(Robot* const _robot)
        const noexcept;

    /// Add a task to the problem for a given robot.
    /// @param _robot The robot to which the new task is assigned.
    /// @param _task The new task.
    void AddTask(Robot* const _robot, std::unique_ptr<MPTask>&& _task);

    ///@}
    ///@name Dynamic Obstacle Accessors
    ///@{

    /// Get all of the dynamic obstacles in this problem.
    const std::vector<std::unique_ptr<DynamicObstacle>>& GetDynamicObstacles()
        const noexcept;

    ///@}
    ///@name Debugging
    ///@{

    /// Print the environment, robot, and task information.
    virtual void Print(std::ostream& _os) const;

    ///@}
    ///@name File Path Accessors
    ///@{

    /// Get the base filename for output files.
    const std::string& GetBaseFilename() const;

    /// Set the base filename for output files.
    void SetBaseFilename(const std::string& _s);

    /// Add the base path for input files to a file name.
    /// @param _filename The filename to modify.
    /// @return The base path + filename, or just the path if no name is given.
    std::string GetPath(const std::string& _filename = "");

    /// Set the base path for input files.
    void SetPath(const std::string& _filename);

    ///@}

  protected:

    ///@name Construction Helpers
    ///@{

    /// Helper for parsing XML nodes.
    /// @param _node The child node to be parsed.
    bool ParseChild(XMLNode& _node);

    /// Create a pseudo-point robot.
    void MakePointRobot();

    ///@}
    ///@name Core Properties
    ///@{

    std::unique_ptr<Environment> m_environment;             ///< The planning environment.
    std::vector<std::unique_ptr<Robot>> m_robots;           ///< The robots in our problem.
    std::unique_ptr<Robot> m_pointRobot;                    ///< A pseudo point-robot.
    std::vector<std::unique_ptr<DynamicObstacle>> m_dynamicObstacles; ///< The dynamic obstacles in our problem.

    /// Map the tasks assigned to each robot.
    std::unordered_map<Robot*, std::list<std::unique_ptr<MPTask>>> m_taskMap;

    ///@}
    ///@name Files
    ///@{

    std::string m_xmlFilename;   ///< The XML file name.
    std::string m_baseFilename;  ///< The base name for output files.
    std::string m_filePath;      ///< The relative path for the problem XML.

    ///@}

};

#endif
