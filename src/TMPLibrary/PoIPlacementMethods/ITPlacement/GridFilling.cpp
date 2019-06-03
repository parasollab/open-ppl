#include "GridFilling.h"

#include "TMPLibrary/TMPStrategies/TMPStrategyMethod.h"

#include <chrono>

GridFilling::
GridFilling(MPProblem* _problem) : PlacementMethod(_problem) {

  m_squaresX = 4;
  m_squaresY = 4;
  m_locations.reserve(m_squaresX*m_squaresY);
  m_environment = m_problem->GetEnvironment();
  m_boundary = m_environment->GetBoundary();
  CreateGrid(/*m_boundary->Type()*/);
}

//
GridFilling::
GridFilling(MPProblem* _problem, XMLNode& _node) : PlacementMethod(_problem) {
  m_label = _node.Read("label", true, "", "label for a fixed base it placement method");

  m_squaresX = _node.Read("xSquares", true, nan(""), 0., 1000., "number of squares on X axis for grid");
  m_squaresY = _node.Read("ySquares", true, nan(""), 0., 1000., "number of squares on Y axis for grid");
  m_locations.reserve(m_squaresX*m_squaresY);
  m_environment = m_problem->GetEnvironment();
  m_boundary = m_environment->GetBoundary();

  CreateGrid(/*m_boundary->Type()*/);
}

void
GridFilling::
PlaceIT(InteractionTemplate* _it, MPSolution* _solution, MPLibrary* _library, TMPStrategyMethod* _tmpMethod) {
  //_solution->AddInteractionTemplate(_it);
  auto vcm = _library->GetValidityChecker("pqp_solid");
  for(auto location : m_locations){
  	bool valid = true;
  	for(auto cfg : _it->GetPositions()){
  		auto check = cfg;
  		TranslateCfg(location, check);
  		 if(!vcm->IsValid(check, "ValidateITCfg")){
  		 	valid = false;
       }
  	}
  	if(valid){
  		_it->GetInformation()->AddTemplateLocation(location);
  	}
  }
}

void
GridFilling::
CreateGrid(/*Space _spaceType*/) {
  
  if(1==1/*_spaceType == "Workspace"*/){

  	Range<double> xDimension = m_boundary->GetRange(0);
  	Range<double> yDimension = m_boundary->GetRange(1);
  	//Range<double> zDimension = m_boundary->GetRange(2);
    vector<double> center = m_boundary->GetCenter();
    double centerX = center[0];
    double centerY = center[1];

    double xSpan = xDimension.Length()/m_squaresX;
    double ySpan = yDimension.Length()/m_squaresY;
    double oddXCorrection = m_squaresX%2 == 0 ? 0 : xSpan/2.0;
    double oddYCorrection = m_squaresY%2 == 0 ? 0 : ySpan/2.0;
    bool isOddX = oddXCorrection == 0 ? false : true;
    bool isOddY = oddYCorrection == 0 ? false : true;
    bool xToken = oddXCorrection ? true : false;
    bool squaresXOne = m_squaresX == 1 ? true : false;
    bool squaresYOne = m_squaresY == 1 ? true : false;
    oddXCorrection = squaresXOne ? -oddXCorrection : oddXCorrection;
    oddYCorrection = squaresYOne ? -oddYCorrection : oddYCorrection;
    bool squaresXOrYOne = squaresXOne || squaresYOne ? true : false;

    int loopX = 0;
    int loopY = 0;

    auto robot = m_problem->GetRobot(0);
    
    if(squaresXOne && squaresYOne){
      Cfg position({centerX,centerY,0}, robot);
      m_locations.push_back(position);
    }
    else{
      for(double i=xDimension.min+(xSpan/2.0); i<(center[0])-oddXCorrection; i=i+xSpan){
        loopX++;
        for(double j=yDimension.min+(ySpan/2.0); j<(center[1])-oddYCorrection; j=j+ySpan){
          loopY++;
          // cout << "\n" << "X: " << i << " / Y: " << j << "\n";          
          Cfg position({i,j,0}, robot);
          m_locations.push_back(position);
          Cfg position2({i,yDimension.max-(yDimension.max-abs(j)),0}, robot);
          m_locations.push_back(position2);
          
          if(!squaresXOne){
            Cfg position3({xDimension.max-(xDimension.max-abs(i)),j,0}, robot);
            m_locations.push_back(position3);
            Cfg position4({xDimension.max-(xDimension.max-abs(i)),yDimension.max-(yDimension.max-abs(j)),0}, robot);
            m_locations.push_back(position4);
          }

          if(isOddX && xToken && !squaresXOne){
            double coordX = center[0];
            Cfg position({coordX,j,0}, robot);
            m_locations.push_back(position);
            Cfg position2({coordX,yDimension.max-(yDimension.max-abs(j)),0}, robot);
            m_locations.push_back(position2);
          }
        }
        xToken = false;

        if(isOddY && !squaresYOne){
          double coordY = center[1];
          Cfg position({i,coordY,0}, robot);
          m_locations.push_back(position);  
          Cfg position2({xDimension.max-(xDimension.max-abs(i)),coordY,0}, robot);
          m_locations.push_back(position2);          
        }
      }
      if(isOddX && isOddY && !squaresXOrYOne){
        double x = center[0];
        double y = center[1];
        Cfg position({x,y,0}, robot);
        m_locations.push_back(position);
      }  
    }
  }
  else if(1!=1/*_spaceType == "CSpace"*/){
  	//TODO
  }

}


void
GridFilling::
TranslateCfg(const Cfg& _centerCfg, Cfg& _relativeCfg){
  double x = _relativeCfg[0];
  double y = _relativeCfg[1];
  double theta = _centerCfg[2];

  double newX = x*cos(theta) - y*sin(theta);
  double newY = x*sin(theta) + y*cos(theta);
  double oldTheta = _relativeCfg[2];

  _relativeCfg.SetLinearPosition({newX, newY, oldTheta});

  _relativeCfg += _centerCfg;
}
