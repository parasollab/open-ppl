#include "SmartAllocator.h"

#include "Behaviors/Agents/Coordinator.h"

#include "MPProblem/Constraints/CSpaceConstraint.h"

#include "TMPLibrary/Solution/Plan.h"

/*----------------------- Construction -----------------------*/

SmartAllocator::
SmartAllocator() {
  this->SetName("SmartAllocator");
}

SmartAllocator::
SmartAllocator(XMLNode& _node) : TaskAllocatorMethod(_node) {
  this->SetName("SmartAllocator");

  // TODO::Parse xml node
  m_singleSolver = _node.Read("singleAgentSolver", false, "BasicPRM", "Provide single agent solver label.");

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

  //for(auto semanticTask : semanticTasks) {
  //  AllocateTask(semanticTask);
  //}
}

/*-------------------- Helper Functions ----------------------*/

void
SmartAllocator::
Initialize() {
  auto problem = this->GetMPProblem();

  // Initialize mp solution object if it is not yet initialized
  if(!m_initialized) {
    m_solution = std::unique_ptr<MPSolution>(new MPSolution(
                  this->GetPlan()->GetCoordinator()->GetRobot()));
  }

  
  // Intialize robot start positions
  auto plan = this->GetPlan();
  auto coordinator = plan->GetCoordinator();
  for(auto& robot : problem->GetRobots()) {
    if(robot.get() == coordinator->GetRobot())
      continue;

    auto startPosition = problem->GetInitialCfg(robot.get());
    m_currentPositions[robot.get()] = startPosition;
    m_solution->AddRobot(robot.get());
  }

  m_initialized = true;
}

void
SmartAllocator::
AllocateTask(vector<SemanticTask*> _semanticTasks) {

  //auto lib = this->GetMPLibrary();
  //auto problem = this->GetMPProblem();
  
  // determine how many robots/tasks we have so we know the size of the cost matrix
  //m_n = problem->NumRobots() -1; // subtract 1 to exclude the coordinator 
  //m_m = _semanticTasks.size();

  m_n = 3;
  m_m = 3;

  cout << "number of tasks = " << m_m <<endl;

  // create an nxm array. n = num robots, m = num tasks
  vector<vector<double>> costMatrix(m_n, vector<double>(m_m));
  vector<vector<double>> mask(m_n, vector<double>(m_m));
  //m_costMatrix = costMatrix;

  m_costMatrix = {{1,2,3},{2,4,6},{3,6,9}};
 
  m_mask = mask;
  
  for(int i=0; i < m_n; i++)
    m_rowCover.push_back(0);

  for(int i=0; i < m_m/2; i++)
    m_colCover.push_back(0);

  //double costMatrix[n][m];

  // Iterate through each robot/task and get cost assignments for each of them 
  /*
  int i = 0;
  int j = 0;
  // Iterate through all robots
  auto plan = this->GetPlan();
  auto coordinator = plan->GetCoordinator();
  for(auto& robot : problem->GetRobots()) {

    if(robot.get() == coordinator->GetRobot())
      continue;
    j = 0;

    for(auto& semanticTask : _semanticTasks) {
      auto task = CreateMPTask(robot.get(), m_currentPositions[robot.get()], semanticTask->GetMotionTask()->GetStartConstraint());
    
      task->SetRobot(robot.get());
  
      // solve the "getting there" task 
      lib->Solve(problem, task.get(), m_solution.get(), m_singleSolver, LRand(), this->GetNameAndLabel()+"::"+task->GetLabel());

      // compute cost of path
      double pathCost;
      if(!m_solution->GetPath()->Cfgs().empty()){ // only get the cost if we found a valid path  
        pathCost = m_solution->GetPath()->Length();
      } else {
        pathCost = std::numeric_limits<double>::max(); // otherwise, assign some huge cost for this task
      }

      // Compute cost for robot to execute task
      lib->Solve(problem,semanticTask->GetMotionTask().get(),m_solution.get());

      // compute cost of this task
      double taskCost = m_solution->GetPath()->Length();

      double totalCost = pathCost + taskCost;

      
      if (m_debug) std::cout << "the cost to assign robot " << i << " to task " << j << " is "<< totalCost << endl;

      costMatrix[i][j] = totalCost;
      
      j++;

    }

    i++;

  }

  int index = 0;
  vector<vector<double>> mat(m_n, vector<double>(m_m/2));
  for(int k = 0; k < m_n; k++) {
      index = 0;
      for(int l = 0; l < m_m; l++) {
        if (l%2!=0) {
          mat[k][index] = costMatrix[k][l];
          index++;
        }
      }
  }

  m_costMatrix = mat;

  m_m = m_m/2; */

  if (m_debug) { 
  //print matrix 
  std::cout<< "the cost matrix before running munkres is " << endl;
    for(int k = 0; k < m_n; k++) {
      for(int l = 0; l < m_m; l++) {
        std::cout << m_costMatrix[k][l] << " ";
      }
      std::cout << "\n";
    }
  }

 
  Munkres();

  //TODO:: Make actual assigments based on results of algorithm
  // There should be a starred zero in each row/col. This is the robot to task assignment
  
  // Update plan allocation
  //SaveAllocation(bestRobot,_semanticTask);

}

void
SmartAllocator::
SaveAllocation(Robot* _robot, SemanticTask* _task) {

  if(true) {
    std::cout << "Allocation "
              << _task->GetLabel()
              << " to "
              << _robot->GetLabel()
              << std::endl;
  }

  // Add the allocation to the plan
  auto plan = this->GetPlan();

  // TODO::Build our task solution object - do this much later
  plan->AddAllocation(_robot,_task);

  // Update the robot position
  auto rm = m_solution->GetRoadmap(_robot);
  auto path = m_solution->GetPath(_robot);
  auto lastVID = path->VIDs().back();
  auto lastPosition = rm->GetVertex(lastVID);
  m_currentPositions[_robot] = lastPosition;
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

// the following are all helper functions for the munkres algorithm specifically

void
SmartAllocator::
Munkres() {

  //int step = 0;
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
