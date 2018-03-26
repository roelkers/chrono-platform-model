#include "params.h"
#include "defineParameters.h"

#include "chrono/core/ChLog.h"

params defineParameters(){

  params p;
  p.towerHeight=100;
  p.towerRadius=5;
  p.towerDensity=500;
  p.towerInitPos = ChVector<>(0, 0, 0);
  p.towerInitDir = ChVector<>(0, 0, 1);

  p.mooringLineNr = 3;
  p.mooringDiameter = 0.15;
  p.mooringYoungModulus = 200e9;
  p.mooringRaleyghDamping = 0.000;
  p.mooringNrElements = 5;
  p.mooringL = 100;
  p.mooringPosFairleadZ = 25;
  p.mooringPosBottomZ = 10;

  double sectionLength = p.mooringL/p.mooringNrElements;
  GetLog() << "sectionLength: " << sectionLength << "\n";
  p.mooringRestLength = sectionLength*0.9;

  p.rhoWater = 1000;
  p.g = 9.81;

  return p;
}
