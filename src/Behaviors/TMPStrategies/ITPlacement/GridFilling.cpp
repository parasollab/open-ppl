#include "GridFilling.h"
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
PlaceIT(InteractionTemplate* _it, MPSolution* _solution, MPLibrary* _library, Coordinator* _coordinator) {
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

    double xSpan = xDimension.Length()/m_squaresX;
    double ySpan = yDimension.Length()/m_squaresY;
    double distX = xDimension.max+xDimension.min;
    double distY = yDimension.max+yDimension.min;
    double oddXCorrection = m_squaresX%2 == 0 ? 0 : xSpan/2.0;
    double oddYCorrection = m_squaresY%2 == 0 ? 0 : ySpan/2.0;
    bool isOddX = oddXCorrection == 0 ? false : true;
    bool isOddY = oddYCorrection == 0 ? false : true;
    bool xToken = oddXCorrection ? true : false;;
    // int zSpan = zDimension.Length()/m_squares;

    auto robot = m_problem->GetRobot(0);
    
    for(double i=xDimension.min+(xSpan/2.0); i<(xDimension.max/2.0)-oddXCorrection; i=i+xSpan){
        for(double j=yDimension.min+(ySpan/2.0); j<(yDimension.max/2.0)-oddYCorrection; j=j+ySpan){
        	// cout << "\n" << "X: " << i << " / Y: " << j << "\n";          
          Cfg position({i,j,0}, robot);
        	m_locations.push_back(position);
          Cfg position2({i,distY-j,0}, robot);
          m_locations.push_back(position2);
          Cfg position3({distX-i,j,0}, robot);
          m_locations.push_back(position3);
          Cfg position4({distX-i,distY-j,0}, robot);
          m_locations.push_back(position4);

          if(isOddX && xToken){
            double coord = xDimension.max-(distX/2.0);
            Cfg position({coord,j,0}, robot);
            m_locations.push_back(position);
            Cfg position2({coord,distY-j,0}, robot);
            m_locations.push_back(position2);
          }
        }
        xToken = false;
        if(isOddY){
          double coord = yDimension.max-(distY/2.0);
          Cfg position({i,coord,0}, robot);
          m_locations.push_back(position);  
          Cfg position2({distX-i,coord,0}, robot);
          m_locations.push_back(position2);          
        }
    }
    if(isOddX && isOddY){
      double x = xDimension.max-(distX/2.0);
      double y = yDimension.max-(distY/2.0);
      Cfg position({x,y,0}, robot);
      m_locations.push_back(position);

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
