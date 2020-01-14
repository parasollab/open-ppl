#ifndef MP_PROBLEM_H_
#define MP_PROBLEM_H_

#include <iostream>
#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

class Decomposition;
class DynamicObstacle;
class InteractionInformation;
class Environment;
class MPTask;
class GroupTask;
class MultiBody;
class Robot;
class RobotGroup;
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
    MPProblem(MPProblem&& _other) = delete;

    ~MPProblem();

    ///@}
    ///@name Assignment
    ///@{

    MPProblem& operator=(const MPProblem& _other); ///< Copy.
    MPProblem& operator=(MPProblem&& _other) = delete;

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
    Robot* GetRobot(const size_t _index) const noexcept;

    /// Get a specific robot by label.
    Robot* GetRobot(const std::string& _label) const noexcept;

    /// Get all robots in this problem.
    const std::vector<std::unique_ptr<Robot>>& GetRobots() const noexcept;

    /// Group versions:
    /// Get the number of robot groups in our problem.
    size_t NumRobotGroups() const noexcept;

    /// Get a specific robot group by index.
    RobotGroup* GetRobotGroup(const size_t _index) const noexcept;

    /// Get a specific robot group by group's label.
    RobotGroup* GetRobotGroup(const std::string& _label) const noexcept;

    /// Get all robot groups in this problem.
    const std::vector<std::unique_ptr<RobotGroup>>& GetRobotGroups() const noexcept;

    ///@}
    ///@name Task Accessors
    ///@{

    /// Get the unfinished tasks currently assigned to a given robot.
    /// @param _robot The robot to retrieve tasks for.
    /// @return The set of tasks currently assigned to _robot.
    std::vector<std::shared_ptr<MPTask>> GetTasks(Robot* const _robot) const
        noexcept;

    /// Group overload
    std::vector<std::shared_ptr<GroupTask>> GetTasks(RobotGroup* const _group)
        const noexcept;

    /// Add a task to the problem. The assigned robot will be taken from the
    /// task object.
    /// @param _task The new task.
    void AddTask(std::unique_ptr<MPTask>&& _task);

    /// Reassign a task to another robot.
    /// @param _task The task to reassign.
    /// @param _newOwner The new robot assigned to _task.
    void ReassignTask(MPTask* const _task, Robot* const _newOwner);

    ///@}
    ///@name Dynamic Obstacle Accessors
    ///@{

    /// Get all of the dynamic obstacles in this problem.
    const std::vector<DynamicObstacle>& GetDynamicObstacles() const noexcept;

    void AddDynamicObstacle(DynamicObstacle&& _obstacle);

    void ClearDynamicObstacles();

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

    ///@name Handoff Template Accessors
    ///@{

    /// Return the list of handoff templates defined in the problem
    /// @return vector of handoff templates
    std::vector<std::unique_ptr<InteractionInformation>>&
        GetInteractionInformations();

    ///@}

  protected:

    ///@name Construction Helpers
    ///@{

    /// Helper for parsing XML nodes.
    /// @param _node The child node to be parsed.
    void ParseChild(XMLNode& _node);

    /// Create a pseudo-point robot.
    void MakePointRobot();

    ///@}
    ///@name Core Properties
    ///@{

    std::unique_ptr<Environment> m_environment;    ///< The planning environment.

    std::vector<std::unique_ptr<Robot>> m_robots;  ///< The robots in our problem.
    std::vector<std::unique_ptr<RobotGroup>> m_robotGroups; ///< Robot groups.
    std::unique_ptr<Robot> m_pointRobot;           ///< A pseudo point-robot.

    /// The dynamic obstacles in our problem.
    std::vector<DynamicObstacle> m_dynamicObstacles;

    /// All handoff templates for a problem.
    std::vector<std::unique_ptr<InteractionInformation>> m_interactionInformations;

    /// Map the tasks assigned to each robot.
    std::unordered_map<Robot*, std::list<std::shared_ptr<MPTask>>> m_taskMap;
    std::unordered_map<RobotGroup*, std::list<std::shared_ptr<GroupTask>>>
        m_groupTaskMap;

		std::unordered_map<std::string,std::unique_ptr<Decomposition>> m_taskDecompositions;

    ///@}
    ///@name Files
    ///@{

    std::string m_xmlFilename;   ///< The XML file name.
    std::string m_baseFilename;  ///< The base name for output files.
    std::string m_filePath;      ///< The relative path for the problem XML.

    ///@}

};

#endif
