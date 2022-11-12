#include "SmartAllocator.h"

#include "Behaviors/Agents/Coordinator.h"

#include "MPProblem/Constraints/CSpaceConstraint.h"

#include "Utilities/MetricUtils.h"

#include "TMPLibrary/Solution/Plan.h"
#include "TMPLibrary/Solution/TaskSolution.h"

/*----------------------- Construction -----------------------*/

SmartAllocator::
SmartAllocator() {
  this->SetName("SmartAllocator");
}

SmartAllocator::
SmartAllocator(XMLNode& _node) : TaskAllocatorMethod(_node) {
  this->SetName("SmartAllocator");

  // Parse xml node
  m_solver = _node.Read("solver", false, "BasicPRM",
      "Provide a solver label.");


  m_clearAfterInitializing = _node.Read("reset",false,m_clearAfterInitializing,
      "Flag to reset all existing allocations after initializing.");
}


/*------------------- Allocator Interface --------------------*/

void
SmartAllocator::
AllocateTasks() {

  Initialize();

  // Compute Allocation

  auto plan = this->GetPlan();
  auto decomp = plan->GetDecomposition();
  auto semanticTasks = decomp->GetMotionTasks();


  AllocateTask(semanticTasks);

}

/*-------------------- Helper Functions ----------------------*/

void
SmartAllocator::
AllocateTask(vector<SemanticTask*> _semanticTasks) {

  auto lib = this->GetMPLibrary();
  auto problem = this->GetMPProblem();

  bool solved  = false;

  //get number of tasks.
  //_semanticTasks has a duplicate of each, so divide by 2
  int originalNumTasks = _semanticTasks.size()/2;

  // determine how many robots/tasks we have so we know the size of the cost matrix
  m_n = problem->NumRobots() -1; // subtract 1 to exclude the coordinator
  m_m = _semanticTasks.size();

  // keep track of which robots are assigned to which tasks
  vector<vector<double>> assignments(m_n, vector<double>(originalNumTasks));

  // create an nxm array. n = num robots, m = num tasks
  vector<vector<double>> costMatrix(m_n, vector<double>(m_m));
  //vector<vector<std::unique_ptr<Path>>> pathMatrix(m_n, vector<std::unique_ptr<Path>>(m_m));
  //m_pathMatrix = pathMatrix;

  // mask matrix that wil be used in Munkres algorithm
  vector<vector<double>> mask(m_n, vector<double>(m_m/2));

  // keep track of which tasks have already been assigned a robot
  vector<bool> assigned;
  for(int i=0; i<m_m/2; i++) assigned.push_back(false);

  vector<vector<std::unique_ptr<Path>>> pathMatrix;
  // keep looping until every task has a robot assigned to it
  // in this loop, find the cost matrix and then call munkres
  while(!solved) {

    // reset the mask every time or else munkres will not run
    m_mask = mask;


    // reset row and col covers for munkres
    for(int i=0; i < m_n; i++)
      m_rowCover.push_back(0);

    for(int i=0; i < m_m; i++)
      m_colCover.push_back(0);

    // Iterate through each robot/task and get cost assignments for each of them

    int i = 0;
    int j = 0;

    auto plan = this->GetPlan();
    auto coordinator = plan->GetCoordinator();

    // Iterate through all robots
    for(auto& robot : problem->GetRobots()) {

      // first we need to determine if the robot already has any tasks
      // assigned to it and add the cost of completing those tasks to the
      // cost of this task
      double previousCost = 0.0;

      if(robot.get() == coordinator->GetRobot())
        continue;

      // Does the robot have any other tasks already assigned to it?
      // If it does, we need to add the cost of those to these task costs

      for(int l=0; l<originalNumTasks; l++){
        // robot i assigned to task l = 1 if true
        if(assignments[i][l] ==1){
          // if this robot is assigned to a task, add the cost of completing
          // that task to its previous cost
          previousCost+= costMatrix[i][l];
        }
      }


      j = 0;

      // Iterate through all tasks
      for(auto& semanticTask : _semanticTasks) {

        // check if the robot is already assigned to another task
        // if so, add the cost of that task to the cost of this task
        // note: be careful about start/goal config for each of the tasks


        auto task = CreateMPTask(robot.get(), m_currentPositions[robot.get()],
              semanticTask->GetMotionTask()->GetStartConstraint());

        task->SetRobot(robot.get());

        // solve the "getting there" task
        lib->Solve(problem, task.get(), m_solution.get(), m_solver,
            LRand(), this->GetNameAndLabel()+"::"+task->GetLabel());

        // compute cost of path
        double pathCost;
        auto path = m_solution->GetPath(robot.get());

        if(path->VIDs().empty())
          continue;


        auto rm = m_solution->GetRoadmap(robot.get());
        auto pathCopy = std::unique_ptr<Path>(new Path(rm));
        pathCost = path->Length();
        *(pathCopy.get()) = *path;

        // Compute cost for robot to execute task
        auto mainTask = semanticTask->GetMotionTask().get();
        mainTask->SetRobot(robot.get());
        lib->Solve(problem,mainTask,m_solution.get());

        path = m_solution->GetPath();
        if(path->VIDs().empty())
          continue;

        *(pathCopy.get()) += path->VIDs();

        // compute cost of this task
        double taskCost = m_solution->GetPath()->Length();

        double totalCost = previousCost + pathCost + taskCost;

        double estimatedCompletion = totalCost + m_nextFreeTime[robot.get()];
        std::cout << "estimated completion = " << estimatedCompletion << std::endl;

        costMatrix[i][j] = estimatedCompletion;
        pathMatrix[i][j] = std::move(pathCopy);
        j++;

      }
      i++;
    }


    // the cost matrix we use for munkres should have the correct number of
    // tasks
    vector<vector<double>> mat(m_n, vector<double>(m_m/2));
    m_costMatrix = mat;
    vector<vector<std::unique_ptr<Path>>> path1;
    //m_pathMatrix = path1;


    // clean up the cost matrix. dont give it the duplicate tasks and only
    // give it the vlaues for tasks that have not been assigned already. If a
    // task has been assigned, give it an extremely high cost so that it is not
    // solve again
    for(int i=0; i<m_n; i++){
      int index = 0;
      for(int j=1; j<m_m; j+=2){
        path1[i][index] = std::move(pathMatrix[i][index]);
        if(!assigned[index]) {
          m_costMatrix[i][index] = costMatrix[i][j];
        }
        else m_costMatrix[i][index] = 9999999999;
        index++;
      }
    }


  // Solve the cost matrix
  m_m = m_m/2;
  Munkres();
  m_m = m_m*2;


  // After running munkres there will be a starred zero in each row/col (in mask matrix).
  // This is the robot to task assignment

  // Update plan allocation
  for(int i=0; i<m_n; i++){
    for (int j=0; j<m_m/2; j++){
      if(m_mask[i][j] == 1) {

      SaveAllocation(problem->GetRobots()[i].get(),_semanticTasks[(j*2)+1],
          std::move(path1[i][j]));

      // Also update assignments
      assignments[i][j] = m_mask[i][j];
      assigned[j] = true;
      }
    }
  }

  // check if every task has been allocated
  int count = 0;
  for(unsigned int i=0; i<assigned.size(); i++){
    if (assigned[i]) count++;
  }

  // if every task has been allocated, then we are done
  if(count == originalNumTasks) solved = true;

  }

}



std::shared_ptr<MPTask>
SmartAllocator::
CreateMPTask(Robot* _robot, const Cfg& _start, const Cfg& _goal) {

  // Create start constraint
  auto startConstraint = std::unique_ptr<Constraint>(
        new CSpaceConstraint(_robot,_start));

  // Create goal constraint
  auto goalConstraint = std::unique_ptr<Constraint>(
        new CSpaceConstraint(_robot,_goal));

  // Create task
  auto task = std::shared_ptr<MPTask>(new MPTask(_robot));
  task->SetStartConstraint(std::move(startConstraint));
  task->AddGoalConstraint(std::move(goalConstraint));

  return task;
}

std::shared_ptr<MPTask>
SmartAllocator::
CreateMPTask(Robot* _robot, const Cfg& _start, const Constraint* _goal) {

  // Create start constraint
  auto startConstraint = std::unique_ptr<Constraint>(
        new CSpaceConstraint(_robot,_start));

  // Create task
  auto task = std::shared_ptr<MPTask>(new MPTask(_robot));
  task->SetStartConstraint(std::move(startConstraint));
  auto goal = _goal->Clone();
  goal->SetRobot(_robot);
  task->AddGoalConstraint(std::move(goal));

  return task;
}

std::shared_ptr<MPTask>
SmartAllocator::
CreateMPTask(Robot* _robot, const Constraint* _start, const Constraint* _goal) {

  // Create start constraint
  //auto startConstraint = std::unique_ptr<Constraint>(
        //new CSpaceConstraint(_robot,_start));

  // Create task
  auto task = std::shared_ptr<MPTask>(new MPTask(_robot));
  auto start = _start->Clone();
  task->SetStartConstraint(std::move(start));
  auto goal = _goal->Clone();
  goal->SetRobot(_robot);
  task->AddGoalConstraint(std::move(goal));

  return task;
}

// the following are all helper functions for the munkres algorithm specifically

void
SmartAllocator::
Munkres() {

  auto stats = this->GetPlan()->GetStatClass();
  MethodTimer mt(stats,this->GetNameAndLabel()+"::Munkres");

  m_step = 1;
  bool done = false;

  while (!done) {
    if(m_step == 1) {
      SubtractSmallest();
    }
    else if(m_step == 2) {
      StarZeros();
    }
    else if(m_step == 3) {
      CoverColumns();
    }
    else if(m_step == 4) {
      PrimeZeros();
    }
    else if(m_step == 5) {
      ConstructSeries();
    }
    else if(m_step == 6) {
      AddAndSubtractValue();
    }
    else if(m_step == 7) {
      done = true;
      break;
    }
  }


  if (m_debug) {
    //print matrix
    std::cout<< "the final cost matrix is " << endl;
    for(int k = 0; k < m_n; k++) {
      for(int l = 0; l < m_m; l++) {
        std::cout << m_costMatrix[k][l] << " ";
      }
      std::cout << "\n";
    }

    std::cout<< "the current mask matrix is " << endl;
    for(int k = 0; k < m_n; k++) {
      for(int l = 0; l < m_m; l++) {
        std::cout << m_mask[k][l] << " ";
      }
      std::cout << "\n";
    }
  }


  return;


}


void
SmartAllocator::
SubtractSmallest() {

  //if (m < n) {
    // TODO:: Rotate matrix so that there are at least as many columns as there are rows
  //}

  if(m_debug) std::cout << "Subtracting smallest..." << endl;

  // find the smallest cost in the matrix
  double smallest;
  for(int i=0; i < m_n; i++){
    smallest = m_costMatrix[i][0];
    for(int j=0; j < m_m; j++){
      if(m_costMatrix[i][j] < smallest)
        smallest = m_costMatrix[i][j];
    }
    for(int j=0; j < m_m; j++){
        m_costMatrix[i][j] -= smallest;
    }
  }

  m_step = 2;


  return;


}

void
SmartAllocator::
StarZeros() {


  if(m_debug) std::cout << "starring zeros..." << endl;

  for(int i=0; i < m_n; i++){
    for(int j=0; j < m_m; j++){
      if(static_cast<int>(std::round(m_costMatrix[i][j])) == 0 && m_rowCover[i] == 0 && m_colCover[j] == 0 ) {
        m_mask[i][j] = 1;
        m_rowCover[i] = 1;
        m_colCover[j] = 1;
      }
    }
  }

  for(int i=0; i < m_n; i++){
    m_rowCover[i] = 0;
  }
  for(int j=0; j < m_m; j++){
    m_colCover[j] = 0;
  }

  m_step = 3;

}

void
SmartAllocator::
CoverColumns() {

  if(m_debug) std::cout << "covering columns..." << endl;

  int count = 0;
  for(int i=0; i < m_n; i++){
    for(int j=0; j < m_m; j++){
      if(m_mask[i][j] == 1) {
        m_colCover[j] = 1;
      }
    }
  }


  for(int j=0; j < m_m; j++){
    if(m_colCover[j] ==1)
      count += 1;
  }

  if(count >= m_m || count >=m_n)
    m_step = 7;
  else
    m_step =4;



}

void
SmartAllocator::
PrimeZeros() {

  if(m_debug) std::cout << "priming zeros..." << endl;

  // first, find a zero
  bool done = false;
  int row = -1;
  int col = -1;


  while(!done) {
    // find a zero
    vector<int> rowcol = FindZero(row, col);

    row = rowcol[0];
    col = rowcol[1];

    if (row == -1) {
      done = true;
      m_step = 6;
    }
    else {
      m_mask[row][col] = 2;

      bool starInRow = false;
      for(int i = 0; i<m_m; i++) {
        if (m_mask[row][i] == 1) {
          starInRow = true;
          col = i;
        }
      }

      if (starInRow) {
        m_rowCover[row] = 1;
        m_colCover[col] = 0;
      }
      else {
        done = true;
        m_row0 = row;
        m_col0 = col;
        m_step =5;
      }
    }
  }

  return;
}

vector<int>
SmartAllocator::
FindZero(int row, int col) {



  if(m_debug) std::cout << "finding a zero..." << endl;



  bool done = false;
  int r = 0;
  int c = 0;
  row = -1;
  col =-1;

  while (!done) {
    c = 0;
    while (true) {
      bool rowcovered = (m_rowCover[r] == 0);
      bool colcovered = (m_colCover[c] == 0);
      bool equal0 = (static_cast<int>(std::round(m_costMatrix[r][c])) == 0) ;

      if (equal0 && rowcovered && colcovered) {
        row = r;
        col = c;
        done = true;
      }
      c += 1;
      if (c >= m_m || done)
        break;
    }
  r += 1;
  if (r >= m_n)
      done = true;
  }



  vector<int> rowcol;
  rowcol.push_back(row);
  rowcol.push_back(col);


  return rowcol;

}

void
SmartAllocator::
ConstructSeries() {

  if(m_debug) std::cout << "constructing series..." << endl;


  bool done = false;
  int r = -1;
  int c = -1;

  int count = 1;
  //vector<vector<int>> path;
  int path[100][2];

  path[count-1][0] = m_row0;
  path[count-1][1] = m_col0;



  while(!done){
    // find star in the column
    r = -1;
    for(int i = 0; i<m_n; i++) {
      if (m_mask[i][path[count-1][1]] == 1)
        r = i;
    }
    //cout << "r = " << r << endl;

    if(r > -1) {
      count++;
      path[count-1][0] = r;
      path[count-1][1] = path[count-2][1];
    }
    else
      done = true;

    if(!done) {
      // find prime in the row
      for(int i =0; i< m_m; i++) {
        if(m_mask[path[count-1][0]][i] == 2)
          c = i;
      }

      count++;
      path[count-1][0] = path[count-2][0];
      path[count-1][1] = c;

    }

  }

  // augment path

  for(int i=0; i < count; i++){
    if(m_mask[path[i][0]][path[i][1]] == 1)
      m_mask[path[i][0]][path[i][1]] = 0;
    else
      m_mask[path[i][0]][path[i][1]] = 1;
  }

  //clear covers

  for(int i=0; i < m_n; i++){
    m_rowCover[i] = 0;
  }
  for(int j=0; j < m_m; j++){
    m_colCover[j] = 0;
  }

  //erase primes
  for(int i=0; i<m_n; i++){
    for(int j=0; j<m_m; j++){
      if(m_mask[i][j]==2)
        m_mask[i][j] = 0;
    }
  }

  m_step = 3;


}

void
SmartAllocator::
AddAndSubtractValue() {

  if(m_debug) std::cout << "adding/subtracting smallest value..." << endl;

  // find the smallest cost in the matrix
  double smallest = std::numeric_limits<double>::max();
  for(int i=0; i < m_n; i++){
    for(int j=0; j < m_m; j++){
      if(m_rowCover[i] ==0 && m_colCover[j] == 0) {
        if(m_costMatrix[i][j] < smallest)
          smallest = m_costMatrix[i][j];
      }
    }
  }

  for(int i=0; i < m_n; i++){
    for(int j=0; j < m_m; j++){
      if(m_rowCover[i] == 1){
        m_costMatrix[i][j] += smallest;
      }
      if(m_colCover[j] == 0){
        m_costMatrix[i][j] -= smallest;
      }
    }
  }

  if(m_debug) std::cout<< "the final cost matrix is " << endl;
  for(int k = 0; k < m_n; k++) {
    for(int l = 0; l < m_m; l++) {
      std::cout << m_costMatrix[k][l] << " ";
    }
    std::cout << "\n";
  }

  m_step = 4;


}

/*------------------------------------------------------------*/

